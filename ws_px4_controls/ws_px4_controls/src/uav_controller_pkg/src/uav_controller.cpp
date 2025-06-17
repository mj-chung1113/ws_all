// uav_controller.cpp — PX4 local‑position variant (2025‑06‑13)
// * Subscribes /fmu/out/vehicle_local_position (px4_msgs::msg::VehicleLocalPosition)
// * Converts NED → ENU for internal map‑frame logic
// * Rest of state machine identical

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // geometry_msgs 메시지를 tf2 변환으로 사용하기 위함
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
// ──────────────────────────────────────────────────────────────────────────────
enum class MissionState {
    IDLE,
    TAKEOFF,
    MOVING_TO_WAYPOINT,
    SEARCHING_FOR_MARKER,
    MOVING_TO_RENDEZVOUS,
    MISSION_COMPLETE,
    LANDING
};

class UavController : public rclcpp::Node
{
public:
    UavController() : Node("uav_controller"), state_(MissionState::IDLE), current_wp_idx_(0)
    {
        // 1. CSV 파일 로드 및 좌표 자동 변환
        load_waypoints("/home/jmj/pro_asp_ws/ws_px4_controls/optimized_path.csv");
        RCLCPP_INFO(get_logger(), "load success");
        // 랑데부 포인트 설정 (월드 좌표)
        // 참고: 필요하다면 이 랑데부 포인트도 로컬 좌표로 변환해야 합니다.
        // 예: rendezvous_.x = -62.96... - DRONE_START_X;
        //     rendezvous_.y = 99.09... - DRONE_START_Y;
        //     ...
        rendezvous_.x = -62.96309452779898 - DRONE_START_X;
        rendezvous_.y =  99.09156129659259 - DRONE_START_Y;
        rendezvous_.z =  -0.13497820025109772 - DRONE_START_Z;
        
        tf2::Matrix3x3 ned_to_enu_rot_matrix;
        ned_to_enu_rot_matrix.setValue( 0, 1, 0,  // X_ENU = Y_NED
                                        1, 0, 0,  // Y_ENU = X_NED
                                        0, 0,-1); // Z_ENU = -Z_NED
        ned_to_enu_rot_matrix.getRotation(q_ned_to_enu_);

        // 퍼블리셔 및 서브스크라이버 설정
        pose_cmd_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/command/pose", 10);
        auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        // 2. 생성한 pose_qos 프로파일을 구독자 설정에 적용합니다.
        marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/x500/target_pose", pose_qos, // '10' 대신 'pose_qos'로 변경
            std::bind(&UavController::marker_cb, this, std::placeholders::_1));
        // QoS 설정을 포함한 위치 정보 서브스크라이버
        auto sensor_qos = rclcpp::SensorDataQoS();
        
        local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", sensor_qos,
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
            {
                // NED(x=N,y=E,z=D) → ENU(x=E,y=N,z=−D)
                current_position_.x = msg->position[1];   // E (y_NED) -> world X (E)
                current_position_.y = msg->position[0];   // N (x_NED) -> world Y (N)
                current_position_.z = -msg->position[2];  // D (z_NED) -> world Z (U)

                // 1. PX4 VehicleOdometry에서 받은 쿼터니언 (FRD body -> NED reference)
                tf2::Quaternion px4_q_body_to_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

                // NED 프레임에서 ENU 프레임으로의 변환 
                // 2. 변환 적용: q_body_to_enu = q_ned_to_enu * q_body_to_ned
                // 쿼터니언 곱셈은 오른쪽에서 왼쪽으로 적용됩니다: (B->C) * (A->B) = A->C
                // 따라서, (NED->ENU) * (Body->NED) = Body->ENU
                tf2::Quaternion q_body_to_enu = q_ned_to_enu_ * px4_q_body_to_ned;

                // 4. 결과를 current_orientation_에 저장
                current_orientation_.x = q_body_to_enu.x();
                current_orientation_.y = q_body_to_enu.y();
                current_orientation_.z = q_body_to_enu.z();
                current_orientation_.w = q_body_to_enu.w();
            });
            
        main_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&UavController::main_loop, this));

        RCLCPP_INFO(get_logger(), "UavController initialised. Mission starts in 2 s …");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        state_ = MissionState::TAKEOFF;
    }

private:
    // --- 월드 좌표계에서 드론의 시작 위치 (오프셋) ---
    // Gazebo 월드 파일에 정의된 드론의 초기 pose 값
    static constexpr double DRONE_START_X = -134.74925610706298; // Gazebo World X
    static constexpr double DRONE_START_Y =  61.782506989510747; // Gazebo World Y
    static constexpr double DRONE_START_Z =   0.14202179330315423;// Gazebo World Z

    // pubs / subs
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr local_pos_sub_;
    rclcpp::TimerBase::SharedPtr main_timer_;

    // mission data
    MissionState state_;
    size_t current_wp_idx_;
    std::vector<geometry_msgs::msg::Point> waypoints_;
    std::vector<geometry_msgs::msg::Point> precise_markers_;
    geometry_msgs::msg::Point current_position_{}; // ENU map‑frame position
    geometry_msgs::msg::Quaternion current_orientation_{};
    geometry_msgs::msg::Point rendezvous_;
    tf2::Quaternion q_ned_to_enu_; // 변환 쿼터니언 멤버 변수
    
    // ───── main loop ─────
    void main_loop()
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000, // 1초에 한 번씩만 로그 출력
            "Current state: %d, x: %.2f m, y: %.2f m , z: %.2f m",
            static_cast<int>(state_),
            current_position_.x , current_position_.y, current_position_.z
        );
        switch(state_)
        {
        case MissionState::IDLE: break;
        case MissionState::TAKEOFF:
            send_pose_setpoint(0, 0, 5); // 10m 고도로 이륙 명령
            if (current_position_.z > 3) {
                RCLCPP_INFO(get_logger(), "Takeoff complete! Moving to first waypoint.");
                state_ = MissionState::MOVING_TO_WAYPOINT;
            }
            break;
        case MissionState::MOVING_TO_WAYPOINT:
            // 마지막 웨이포인트를 지났다면 랑데부 상태로 전환
            if (current_wp_idx_ >= waypoints_.size()) {
                RCLCPP_INFO(get_logger(), "All waypoints visited. Moving to rendezvous point.");
                state_ = MissionState::MOVING_TO_RENDEZVOUS;
                break;
            }

            // 현재 웨이포인트로 이동 명령 발행
            publish_waypoint(current_wp_idx_);

            // 현재 웨이포인트에 도착했는지 확인
            if (is_close(current_position_, waypoints_[current_wp_idx_], 2.3)) {
                RCLCPP_INFO(get_logger(), "Arrived at waypoint %zu.", current_wp_idx_);

                // 이 웨이포인트가 ArUco 마커를 찾아야 하는 곳인지 확인
                if (isArucoWaypoint(current_wp_idx_)) {
                    // ArUco 웨이포인트 -> 마커 탐색 상태로 전환
                    RCLCPP_INFO(get_logger(), "This is an ArUco waypoint. Searching for marker.");
                    state_ = MissionState::SEARCHING_FOR_MARKER;
                } else {
                    // 일반 경유 웨이포인트 -> 즉시 다음 웨이포인트로 진행
                    RCLCPP_INFO(get_logger(), "This is a fly-through waypoint. Proceeding to next.");
                    current_wp_idx_++;
                }
            }
            break;
        case MissionState::SEARCHING_FOR_MARKER:
            if(current_wp_idx_<waypoints_.size())
                send_pose_setpoint(waypoints_[current_wp_idx_].x,waypoints_[current_wp_idx_].y,waypoints_[current_wp_idx_].z-1);
            break;
        case MissionState::MOVING_TO_RENDEZVOUS:
            send_pose_setpoint(rendezvous_.x, rendezvous_.y, rendezvous_.z);
            if (is_close(current_position_, rendezvous_, 2.3)) {
                RCLCPP_INFO(get_logger(), "Arrived at rendezvous. Mission complete.");
                // ▼▼▼ 여기에 있던 save_precise_locations(...) 호출을 제거합니다 ▼▼▼
                state_ = MissionState::MISSION_COMPLETE;
            }
            break;
        case MissionState::MISSION_COMPLETE:
            send_pose_setpoint(current_position_.x,current_position_.y,-1); state_ = MissionState::LANDING; break;
        case MissionState::LANDING:
            send_pose_setpoint(current_position_.x,current_position_.y,-1);
            if(current_position_.z<0.2) main_timer_->cancel();
            break;
        }
    }

    // marker callback
    void marker_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 1. 현재 '마커 탐색' 상태가 아니면 무시
        if (state_ != MissionState::SEARCHING_FOR_MARKER) {
            return;
        }

        // 2. 다음 상태로 즉시 변경하여 콜백 중복 실행 방지
        if (current_wp_idx_ + 1 < waypoints_.size()) {
            state_ = MissionState::MOVING_TO_WAYPOINT;
        } else {
            state_ = MissionState::MOVING_TO_RENDEZVOUS;
        }

        // 드론 base_link 프레임 기준 마커의 상대 포즈 (MultiTrackerNode에서 발행된 메시지)
        const auto& relative_marker_position = msg->pose.position;
        const auto& relative_marker_orientation = msg->pose.orientation; // 마커의 상대 회전도 포함됩니다.


        // --- 마커의 월드좌표 계산 ---
        // 1. 드론의 현재 전역 포즈 (map -> base_link 변환)
        // 현재 current_position_와 current_orientation_가 map 프레임 기준 드론의 위치/방향을 담고 있다고 가정합니다.
        // (current_position_는 geometry_msgs::msg::Point 타입이며, current_orientation_는 geometry_msgs::msg::Quaternion 타입 멤버 변수라고 가정)
        tf2::Transform drone_pose_in_map;
        drone_pose_in_map.setOrigin(tf2::Vector3(current_position_.x, current_position_.y, current_position_.z));
        
        tf2::Quaternion drone_orientation_in_map;
        tf2::fromMsg(current_orientation_, drone_orientation_in_map); // geometry_msgs::msg::Quaternion -> tf2::Quaternion 변환
        drone_pose_in_map.setRotation(drone_orientation_in_map);

        // 2. 드론 base_link 프레임 기준 마커의 상대 포즈 (base_link -> marker 변환)
        tf2::Transform marker_pose_relative_to_drone;
        marker_pose_relative_to_drone.setOrigin(tf2::Vector3(relative_marker_position.x, relative_marker_position.y, relative_marker_position.z));
        
        tf2::Quaternion marker_orientation_relative_to_drone;
        tf2::fromMsg(relative_marker_orientation, marker_orientation_relative_to_drone); // geometry_msgs::msg::Quaternion -> tf2::Quaternion 변환
        marker_pose_relative_to_drone.setRotation(marker_orientation_relative_to_drone);

        // 3. map 프레임 기준 마커의 절대 포즈 계산
        // 즉, T_map_to_marker = T_map_to_drone * T_drone_to_marker
        // (여기서 T_map_to_drone은 drone_pose_in_map, T_drone_to_marker는 marker_pose_relative_to_drone 입니다.)
        tf2::Transform marker_pose_in_map = drone_pose_in_map * marker_pose_relative_to_drone;

        // 4. 계산된 마커의 전역 위치 (map 프레임 기준)
        geometry_msgs::msg::Point final_local_pose; // 이름을 final_global_pose로 변경하는 것이 더 명확합니다.
        final_local_pose.x = marker_pose_in_map.getOrigin().x();
        final_local_pose.y = marker_pose_in_map.getOrigin().y();
        final_local_pose.z = marker_pose_in_map.getOrigin().z();

        RCLCPP_INFO(this->get_logger(), "Marker Found! Calculated Local Pose: [x: %.2f, y: %.2f, z: %.2f]",
        final_local_pose.x, final_local_pose.y, final_local_pose.z);

        // 3. 계산된 '로컬 좌표'를 리스트에 추가하고 파일에 저장합니다.
        precise_markers_.push_back(final_local_pose);
        save_precise_locations("/home/jmj/precise_marker_locations.csv");
        
        // 4. 다음 웨이포인트로 넘어가기 위해 인덱스를 증가시킵니다.
        ++current_wp_idx_;
    }

    // utils
    static bool is_close(const geometry_msgs::msg::Point&a,const geometry_msgs::msg::Point&b,double tol)
    { double dx=a.x-b.x,dy=a.y-b.y,dz=a.z-b.z; return std::sqrt(dx*dx+dy*dy+dz*dz)<tol; }

    void send_pose_setpoint(double x,double y,double z)
    {
        geometry_msgs::msg::PoseStamped sp; sp.header.frame_id="map"; sp.header.stamp=now();
        sp.pose.position.x=x; sp.pose.position.y=y; sp.pose.position.z=z; sp.pose.orientation.w=1.0; pose_cmd_pub_->publish(sp);
    }
    void publish_waypoint(size_t i)
    { if(i<waypoints_.size()) send_pose_setpoint(waypoints_[i].x, waypoints_[i]. y,waypoints_[i].z+2);
        RCLCPP_INFO(get_logger(), "wp pub :  %4f, %4f, %4f ", waypoints_[i].x,waypoints_[i].y,waypoints_[i].z+2);

    }    

    void load_waypoints(const std::string& file)
    {
        std::ifstream ifs(file);
        if (!ifs) {
            RCLCPP_ERROR(get_logger(), "Cannot open waypoint file: %s", file.c_str());
            rclcpp::shutdown();
            return;
        }
        std::string line;
        while (std::getline(ifs, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string value_str;
            geometry_msgs::msg::Point world_p{}; // 월드 좌표를 임시로 저장

            // CSV에서 월드 좌표(X, Y, Z)를 읽어옴
            std::getline(ss, value_str, ','); world_p.x = std::stod(value_str);
            std::getline(ss, value_str, ','); world_p.y = std::stod(value_str);
            std::getline(ss, value_str, ','); world_p.z = std::stod(value_str);
            
            // 월드 좌표를 드론의 로컬 좌표로 변환
            geometry_msgs::msg::Point local_p{};
            local_p.x = world_p.x - DRONE_START_X;
            local_p.y = world_p.y - DRONE_START_Y;
            local_p.z = world_p.z - DRONE_START_Z;
            
            waypoints_.push_back(local_p); // 변환된 로컬 좌표를 최종 웨이포인트로 저장
            RCLCPP_INFO(this->get_logger(), "local coord: [x: %.4f, y: %.4f, z: %.4f]",
            local_p.x, local_p.y, local_p.z);
        }
        RCLCPP_INFO(get_logger(), "Loaded and converted %zu waypoints from %s", waypoints_.size(), file.c_str());
    }

    void save_precise_locations(const std::string& file)
    {
        std::ofstream ofs(file);
        ofs.precision(15); // 정밀도 설정
        ofs << "x,y,z\n";
        for (const auto& local_p : precise_markers_) {
            // 저장 직전에 로컬 좌표(local_p)를 월드 좌표로 변환
            geometry_msgs::msg::Point world_p{};
            world_p.x = local_p.x + DRONE_START_X;
            world_p.y = local_p.y + DRONE_START_Y;
            world_p.z = local_p.z + DRONE_START_Z;

            // 최종 변환된 '월드 좌표'를 파일에 저장
            ofs << world_p.x << ',' << world_p.y << ',' << world_p.z << '\n';
        }
    }
    bool isArucoWaypoint(size_t index)
    {
        
        return (index % 3 == 0 && index != 0);
    }
};

int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<UavController>());rclcpp::shutdown();return 0;}

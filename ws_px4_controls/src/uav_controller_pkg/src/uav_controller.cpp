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
#include "px4_msgs/msg/vehicle_local_position.hpp"

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
        load_waypoints("uav_wp.csv");

        // 랑데부 포인트 설정 (월드 좌표)
        // 참고: 필요하다면 이 랑데부 포인트도 로컬 좌표로 변환해야 합니다.
        // 예: rendezvous_.x = -62.96... - DRONE_START_X;
        //     rendezvous_.y = 99.09... - DRONE_START_Y;
        //     ...
        rendezvous_.x = -62.96309452779898 - DRONE_START_X;
        rendezvous_.y =  99.09156129659259 - DRONE_START_Y;
        rendezvous_.z =  -0.13497820025109772 - DRONE_START_Z;


        // 퍼블리셔 및 서브스크라이버 설정
        pose_cmd_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/command/pose", 10);
        auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        // 2. 생성한 pose_qos 프로파일을 구독자 설정에 적용합니다.
        marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/x500/target_pose", pose_qos, // '10' 대신 'pose_qos'로 변경
            std::bind(&UavController::marker_cb, this, std::placeholders::_1));
        // QoS 설정을 포함한 위치 정보 서브스크라이버
        auto sensor_qos = rclcpp::SensorDataQoS();
        local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", sensor_qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
            {
                // NED(x=N,y=E,z=D) → ENU(x=E,y=N,z=−D)
                current_pose_.x = msg->y;      // East → x
                current_pose_.y = msg->x;      // North → y
                current_pose_.z = -msg->z;     // Up   → z
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
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::TimerBase::SharedPtr main_timer_;

    // mission data
    MissionState state_;
    size_t current_wp_idx_;
    std::vector<geometry_msgs::msg::Point> waypoints_;
    std::vector<geometry_msgs::msg::Point> precise_markers_;
    geometry_msgs::msg::Point current_pose_{}; // ENU map‑frame position
    geometry_msgs::msg::Point rendezvous_;

    // ───── main loop ─────
    void main_loop()
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000, // 1초에 한 번씩만 로그 출력
            "Current state: %d, Altitude: %.2f m",
            static_cast<int>(state_),
            current_pose_.z
        );
        switch(state_)
        {
        case MissionState::IDLE: break;
        case MissionState::TAKEOFF:
            send_pose_setpoint(0, 0, 10); // 10m 고도로 이륙 명령
            if (current_pose_.z > 9.5) {
                RCLCPP_INFO(get_logger(), "Takeoff complete! Moving to first waypoint.");
                state_ = MissionState::MOVING_TO_WAYPOINT;
            }
            break;
        case MissionState::MOVING_TO_WAYPOINT:
            if (current_wp_idx_ >= waypoints_.size()) break;
            publish_waypoint(current_wp_idx_);
            if (is_close(current_pose_, waypoints_[current_wp_idx_], 5.0)) {
                RCLCPP_INFO(get_logger(), "Arrived at waypoint %zu. Searching for marker.", current_wp_idx_);
                state_ = MissionState::SEARCHING_FOR_MARKER;
            }
            break;
        case MissionState::SEARCHING_FOR_MARKER:
            if(current_wp_idx_<waypoints_.size())
                send_pose_setpoint(waypoints_[current_wp_idx_].x,waypoints_[current_wp_idx_].y,waypoints_[current_wp_idx_].z+2);
            break;
        case MissionState::MOVING_TO_RENDEZVOUS:
            send_pose_setpoint(rendezvous_.x, rendezvous_.y, rendezvous_.z);
            if (is_close(current_pose_, rendezvous_, 3.0)) {
                RCLCPP_INFO(get_logger(), "Arrived at rendezvous. Mission complete.");
                // ▼▼▼ 여기에 있던 save_precise_locations(...) 호출을 제거합니다 ▼▼▼
                state_ = MissionState::MISSION_COMPLETE;
            }
            break;
        case MissionState::MISSION_COMPLETE:
            send_pose_setpoint(current_pose_.x,current_pose_.y,-1); state_ = MissionState::LANDING; break;
        case MissionState::LANDING:
            send_pose_setpoint(current_pose_.x,current_pose_.y,-1);
            if(current_pose_.z<0.2) main_timer_->cancel();
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

        const auto& relative_marker_pose = msg->pose.position;

        // --- 최종 로컬 좌표 계산 ---
        // 드론의 현재 전역 위치에 카메라가 탐지한 상대 위치를 더함
        geometry_msgs::msg::Point final_local_pose{};
        final_local_pose.x = current_pose_.x + relative_marker_pose.x;
        final_local_pose.y = current_pose_.y + relative_marker_pose.y;
        
        // ▼▼▼ Z값도 탐지된 상대 위치를 사용하도록 수정합니다 ▼▼▼
        final_local_pose.z = current_pose_.z + relative_marker_pose.z;

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
    void publish_waypoint(size_t i){ if(i<waypoints_.size()) send_pose_setpoint(waypoints_[i].x,waypoints_[i].y,waypoints_[i].z+5);}    

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
};

int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<UavController>());rclcpp::shutdown();return 0;}

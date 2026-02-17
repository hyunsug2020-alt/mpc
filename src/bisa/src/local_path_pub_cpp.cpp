#include "bisa/local_path_pub_cpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>

namespace bisa
{

    LocalPathPubCpp::LocalPathPubCpp()
        : Node("local_path_pub_cpp"),
          lap_start_time_(this->now())
    {

        // RCLCPP_INFO(this->get_logger(), "===========================================");
        // RCLCPP_INFO(this->get_logger(), "Local Path Publisher C++ - 1kHz");
        // RCLCPP_INFO(this->get_logger(), "Infinite Loop Mode with LapInfo");
        // RCLCPP_INFO(this->get_logger(), "===========================================");

        // [추가] 내 차량 ID 파라미터
        this->declare_parameter("target_cav_id", 1);
        int target_id = this->get_parameter("target_cav_id").as_int();
        std::string id_str = (target_id < 10) ? "0" + std::to_string(target_id) : std::to_string(target_id);

        // ★ [추가] RViz 슬롯 파라미터
        this->declare_parameter("rviz_slot", -1);
        rviz_slot_ = this->get_parameter("rviz_slot").as_int();
        this->declare_parameter("local_path_size", 220);
        this->declare_parameter("search_window", 200);
        this->declare_parameter("max_index_step", 20);
        this->declare_parameter("reinit_distance_threshold", 2.0);
        this->declare_parameter("smooth_window", 1);
        this->declare_parameter("corner_smooth_window", 0);
        this->declare_parameter("corner_curvature_threshold", 0.08);
        local_path_size_ = static_cast<size_t>(std::max<int64_t>(20, this->get_parameter("local_path_size").as_int()));
        search_window = static_cast<int>(std::max<int64_t>(10, this->get_parameter("search_window").as_int()));
        max_index_step_ = static_cast<size_t>(std::max<int64_t>(1, this->get_parameter("max_index_step").as_int()));
        reinit_distance_threshold_ = std::max(0.5, this->get_parameter("reinit_distance_threshold").as_double());
        smooth_window_ = static_cast<int>(std::max<int64_t>(0, this->get_parameter("smooth_window").as_int()));
        corner_smooth_window_ = static_cast<int>(std::max<int64_t>(0, this->get_parameter("corner_smooth_window").as_int()));
        corner_curvature_threshold_ = std::max(0.0, this->get_parameter("corner_curvature_threshold").as_double());

        // RCLCPP_INFO(this->get_logger(), "Local Path Pub for CAV_%s (RViz Slot: %d)", id_str.c_str(), rviz_slot_);

        // 기존 토픽 설정
        std::string global_path_topic = "/user_global_path_cav" + id_str;
        std::string pose_topic = "/CAV_" + id_str;
        std::string local_pub_topic = "/local_path_cav" + id_str;
        std::string lap_info_topic = "/lap_info_cav" + id_str;

        global_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            global_path_topic,
            rclcpp::QoS(10).transient_local(),
            std::bind(&LocalPathPubCpp::global_path_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&LocalPathPubCpp::pose_callback, this, std::placeholders::_1));

        local_pub_ = this->create_publisher<nav_msgs::msg::Path>(local_pub_topic, 10);
        lap_info_pub_ = this->create_publisher<bisa::msg::LapInfo>(lap_info_topic, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/car_marker_" + id_str, 10);

        // ★ [추가] RViz용 퍼블리셔 (슬롯이 유효할 때만)
        if (rviz_slot_ >= 0)
        {
            std::string rviz_local_topic = "/viz/slot" + std::to_string(rviz_slot_) + "/local_path";
            std::string rviz_marker_topic = "/viz/slot" + std::to_string(rviz_slot_) + "/car_marker";

            rviz_local_pub_ = this->create_publisher<nav_msgs::msg::Path>(rviz_local_topic, 10);
            rviz_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(rviz_marker_topic, 10);

            // RCLCPP_INFO(this->get_logger(), "RViz topics: %s, %s", rviz_local_topic.c_str(), rviz_marker_topic.c_str());
        }

        // Local path publish rate: 20Hz (aligned with MPC Ts=0.05s)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&LocalPathPubCpp::publish_local_path, this));

        // RCLCPP_INFO(this->get_logger(), "Ready!");
    }

    void LocalPathPubCpp::global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // if (!global_path_)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Global Path: %zu pts", msg->poses.size());
        // }
        global_path_ = msg;
    }

    void LocalPathPubCpp::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = msg->pose;

        // 총 주행 거리 계산
        if (prev_pose_)
        {
            double dx = current_pose_->position.x - prev_pose_->position.x;
            double dy = current_pose_->position.y - prev_pose_->position.y;
            total_distance_ += std::sqrt(dx * dx + dy * dy);
        }
        prev_pose_ = current_pose_;

        publish_car_marker();
    }

    void LocalPathPubCpp::publish_car_marker()
    {
        if (!current_pose_)
            return;

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "my_car";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = current_pose_->position;

        // Euler to Quaternion
        double roll = current_pose_->orientation.x;
        double pitch = current_pose_->orientation.y;
        double yaw = current_pose_->orientation.z;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        marker.pose.orientation = tf2::toMsg(q);

        marker.scale.x = 0.33;
        marker.scale.y = 0.15;
        marker.scale.z = 0.2;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;

        marker_pub_->publish(marker);

        if (rviz_marker_pub_)
        {
            rviz_marker_pub_->publish(marker);
        }
    }

    void LocalPathPubCpp::publish_local_path()
    {
        if (!global_path_ || !current_pose_)
            return;

        size_t total = global_path_->poses.size();
        if (total == 0)
            return;

        double x = current_pose_->position.x;
        double y = current_pose_->position.y;
        double current_yaw = current_pose_->orientation.z;

        // Find closest waypoint
        double min_d = 1e9;
        size_t closest = current_waypoint_;

        // 만약 초기화 전이라면 전체 탐색 (지난번 답변의 is_initialized_ 활용 가정)
        // 혹은 별도 플래그가 없다면, 그냥 아래 로직이 자동으로 커버합니다.
        bool need_global_search = !is_initialized_;

        if (!need_global_search)
        {
            for (int i = -search_window; i <= search_window; ++i)
            {
                long idx_temp = static_cast<long>(current_waypoint_) + i;

                // 인덱스 순환 처리 (음수 및 초과 방지)
                if (idx_temp < 0)
                    idx_temp += total;
                else if (idx_temp >= static_cast<long>(total))
                    idx_temp %= total;

                size_t idx = static_cast<size_t>(idx_temp);

                double dx = global_path_->poses[idx].pose.position.x - x;
                double dy = global_path_->poses[idx].pose.position.y - y;

                double dot = dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
                if (dot < -0.5)
                    continue;

                double d = std::sqrt(dx * dx + dy * dy);

                if (d < min_d)
                {
                    min_d = d;
                    closest = idx;
                }
            }
        }

        // 2. [핵심] "차를 들어서 옮긴 경우" 감지 (Global Re-initialization)
        // Window 탐색 결과 가장 가까운 점이 2.0m 이상 떨어져 있다면,
        // 차를 멀리 옮겼다고 판단하고 전체 경로에서 다시 찾습니다.
        if (need_global_search || min_d > reinit_distance_threshold_)
        {
            min_d = 1e9; // 거리 초기화
            for (size_t i = 0; i < total; ++i)
            {
                double dx = global_path_->poses[i].pose.position.x - x;
                double dy = global_path_->poses[i].pose.position.y - y;
                double d = std::sqrt(dx * dx + dy * dy);

                if (d < min_d)
                {
                    min_d = d;
                    closest = i;
                }
            }
            // 전역 탐색을 수행했다면 초기화 완료 처리
            is_initialized_ = true;
            // RCLCPP_WARN(this->get_logger(), "Relocated! Reset path index to %zu", closest);
        }

        // 인덱스 점프 억제: 뒤로 튀는 경우를 막고, 과도한 전방 점프를 제한
        if (is_initialized_ && total > 0)
        {
            const size_t prev = current_waypoint_;
            const size_t forward_delta = (closest + total - prev) % total;
            const size_t backward_delta = (prev + total - closest) % total;
            const bool lap_wrap = (prev > (total * 9) / 10) && (closest < total / 10);

            if (!lap_wrap && backward_delta < forward_delta)
            {
                closest = prev;
            }
            else if (forward_delta > max_index_step_ && min_d < reinit_distance_threshold_)
            {
                closest = (prev + max_index_step_) % total;
            }
        }

        // ==========================================

        // 한 바퀴 완료 감지
        // (갑자기 뒤로 옮겼을 때 랩 카운트가 오작동하지 않도록 방어 코드 추가 권장)
        // 단순히 인덱스가 줄어든 경우는 랩 변경이 아님.
        if (closest < current_waypoint_ && current_waypoint_ > total * 0.9 && closest < total * 0.1)
        {
            lap_count_++;
            lap_start_time_ = this->now();
            // RCLCPP_INFO(this->get_logger(),
            //             "🏁 Lap %d completed! Total distance: %.2fm",
            //             lap_count_, total_distance_);
        }

        current_waypoint_ = closest;

        // 진행률 계산 (0.0 ~ 100.0)
        double progress = (static_cast<double>(current_waypoint_) / total) * 100.0;

        // 현재 바퀴 경과 시간
        double elapsed_time = (this->now() - lap_start_time_).seconds();

        // LapInfo 메시지 생성 및 발행
        auto lap_info = bisa::msg::LapInfo();
        lap_info.lap_count = lap_count_;
        lap_info.progress = static_cast<float>(progress);
        lap_info.current_waypoint = static_cast<int32_t>(current_waypoint_);
        lap_info.total_waypoints = static_cast<int32_t>(total);
        lap_info.elapsed_time = static_cast<float>(elapsed_time);
        lap_info.total_distance = static_cast<float>(total_distance_);

        lap_info_pub_->publish(lap_info);

        // 10초마다 로그
        auto now = this->now();
        if ((now - last_log_time_).seconds() > 10.0)
        {
            // RCLCPP_INFO(this->get_logger(),
            //             "Lap %d | Progress: %.1f%% | Time: %.1fs | Distance: %.2fm",
            //             lap_count_, progress, elapsed_time, total_distance_);
            last_log_time_ = now;
        }

        // Local path (순환 경로)
        auto local = nav_msgs::msg::Path();
        local.header.frame_id = "world";
        local.header.stamp = this->now();

        std::vector<geometry_msgs::msg::PoseStamped> raw_points;
        raw_points.reserve(local_path_size_);
        for (size_t i = 0; i < local_path_size_; ++i)
        {
            size_t idx = (current_waypoint_ + i) % total; // 순환!
            raw_points.push_back(global_path_->poses[idx]);
        }

        // 코너 불연속 완화를 위한 이동 평균 스무딩
        std::vector<geometry_msgs::msg::PoseStamped> smoothed_points = raw_points;
        auto curvature_at_raw = [&](size_t idx) -> double
        {
            if (raw_points.size() < 3) return 0.0;
            const size_t i0 = (idx == 0) ? 0 : idx - 1;
            const size_t i1 = idx;
            const size_t i2 = std::min(idx + 1, raw_points.size() - 1);
            if (i0 == i1 || i1 == i2) return 0.0;

            const double x1 = raw_points[i0].pose.position.x;
            const double y1 = raw_points[i0].pose.position.y;
            const double x2 = raw_points[i1].pose.position.x;
            const double y2 = raw_points[i1].pose.position.y;
            const double x3 = raw_points[i2].pose.position.x;
            const double y3 = raw_points[i2].pose.position.y;
            const double dx1 = x2 - x1;
            const double dy1 = y2 - y1;
            const double dx2 = x3 - x2;
            const double dy2 = y3 - y2;
            const double cross = dx1 * dy2 - dy1 * dx2;
            const double d1 = std::hypot(dx1, dy1);
            const double d2 = std::hypot(dx2, dy2);
            if (d1 < 1e-6 || d2 < 1e-6) return 0.0;
            return (2.0 * cross) / (d1 * d2 * (d1 + d2) + 1e-6);
        };

        for (size_t i = 0; i < raw_points.size(); ++i)
        {
            int adaptive_window = smooth_window_;
            const double kappa_abs = std::abs(curvature_at_raw(i));
            if (kappa_abs > corner_curvature_threshold_)
            {
                adaptive_window = std::min(adaptive_window, corner_smooth_window_);
            }

            if (adaptive_window > 0)
            {
                double sx = 0.0;
                double sy = 0.0;
                int cnt = 0;
                const int i0 = std::max(0, static_cast<int>(i) - adaptive_window);
                const int i1 = std::min(static_cast<int>(raw_points.size()) - 1,
                                        static_cast<int>(i) + adaptive_window);
                for (int j = i0; j <= i1; ++j)
                {
                    sx += raw_points[static_cast<size_t>(j)].pose.position.x;
                    sy += raw_points[static_cast<size_t>(j)].pose.position.y;
                    ++cnt;
                }
                if (cnt > 0)
                {
                    smoothed_points[i].pose.position.x = sx / static_cast<double>(cnt);
                    smoothed_points[i].pose.position.y = sy / static_cast<double>(cnt);
                }
            }
        }

        for (size_t i = 0; i < smoothed_points.size(); ++i)
        {
            geometry_msgs::msg::PoseStamped pose = smoothed_points[i];
            size_t ref_from = i;
            size_t ref_to = i;
            if (smoothed_points.size() >= 2)
            {
                if (i + 1 < smoothed_points.size())
                {
                    ref_from = i;
                    ref_to = i + 1;
                }
                else
                {
                    ref_from = i - 1;
                    ref_to = i;
                }
            }
            const double dx = smoothed_points[ref_to].pose.position.x - smoothed_points[ref_from].pose.position.x;
            const double dy = smoothed_points[ref_to].pose.position.y - smoothed_points[ref_from].pose.position.y;
            const double yaw = (std::hypot(dx, dy) > 1e-9) ? std::atan2(dy, dx) : 0.0;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            pose.pose.orientation = tf2::toMsg(q);
            local.poses.push_back(pose);
        }

        local_pub_->publish(local);

        // ★ [추가] RViz용 토픽에도 발행
        if (rviz_local_pub_)
        {
            rviz_local_pub_->publish(local);
        }
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bisa::LocalPathPubCpp>());
    rclcpp::shutdown();
    return 0;
}

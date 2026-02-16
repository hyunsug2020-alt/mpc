#ifndef BISA_MPC_PATH_TRACKER_CPP_HPP_
#define BISA_MPC_PATH_TRACKER_CPP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include "bisa/msg/mpc_performance.hpp"
#include "bisa/mpc_controller_cpp.hpp"
#include "ltv_mpc/ltv_mpc.hpp"
#include <memory>
#include <optional>

namespace bisa {

/**
 * @brief LTV-MPC 기반 경로 추종 노드
 * 
 * 논문: "Lateral Vehicle Trajectory Optimization Using Constrained Linear Time-Varying MPC"
 * (Gutjahr et al., 2017)
 */
class MPCPathTrackerCpp : public rclcpp::Node {
public:
    MPCPathTrackerCpp();

private:
    void local_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void control_loop();
    void publish_control(double v, double w);
    void publish_predicted_path(const std::vector<std::array<double, 3>>& traj);
    void publish_performance(double model_time_us, double solver_time_us,
                             double total_time_us, int solver_iterations,
                             const std::string& solver_status);
    void update_controller_params();
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter>& params);
    
    std::unique_ptr<LTVMPC> ltv_controller_;
    std::unique_ptr<MPCControllerCpp> legacy_controller_;
    std::vector<geometry_msgs::msg::PoseStamped> local_path_;
    std::optional<geometry_msgs::msg::Pose> current_pose_;
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_raw_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pred_pub_;
    rclcpp::Publisher<bisa::msg::MPCPerformance>::SharedPtr perf_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Time last_log_time_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    double max_omega_abs_ = 1.2;
    double max_omega_rate_ = 6.0;
    double max_v_rate_ = 3.0;
    double prev_v_cmd_ = 0.0;
    double prev_w_cmd_ = 0.0;
    rclcpp::Time prev_cmd_time_;
    bool cmd_initialized_ = false;
    
    // CAV ID
    int target_cav_id_;
    bool use_ltv_mpc_{true};
};

}

#endif

#include "bisa/mpc_path_tracker_cpp.hpp"

namespace bisa {

MPCPathTrackerCpp::MPCPathTrackerCpp()
    : Node("mpc_path_tracker_cpp"), last_log_time_(this->now()) {
    
    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "LTV-MPC Path Tracker C++ - Gutjahr 2017");
    RCLCPP_INFO(this->get_logger(), "===========================================");
    
    // CAV ID 파라미터
    this->declare_parameter("target_cav_id", 1);
    target_cav_id_ = this->get_parameter("target_cav_id").as_int();
    std::string id_str = (target_cav_id_ < 10) ? 
                         "0" + std::to_string(target_cav_id_) : 
                         std::to_string(target_cav_id_);
    
    RCLCPP_INFO(this->get_logger(), "LTV-MPC Tracker for CAV_%s", id_str.c_str());
    
    // ===================================================================
    // LTV-MPC 파라미터 선언 (논문 기반)
    // ===================================================================
    
    // 예측 파라미터
    this->declare_parameter("prediction_horizon", 20);      // N
    this->declare_parameter("time_step", 0.05);             // Ts = 0.05s (20Hz 제어 주기와 일치!)
    this->declare_parameter("soft_constraint_horizon", 4);  // NS
    
    // 차량 파라미터
    this->declare_parameter("wheelbase", 0.3);              // l
    this->declare_parameter("vehicle_width", 0.15);
    
    // 입력 제약 (Equation 9)
    this->declare_parameter("u_min", -2.0);
    this->declare_parameter("u_max", 2.0);
    
    // 곡률 제약
    this->declare_parameter("kappa_min_delta", -2.0);
    this->declare_parameter("kappa_max_delta", 2.0);
    
    // 비용 함수 가중치 (Equation 10)
    this->declare_parameter("weight_position", 10.0);       // wd
    this->declare_parameter("weight_heading", 5.0);         // wtheta
    this->declare_parameter("weight_curvature", 2.0);       // wkappa
    this->declare_parameter("weight_input", 0.5);           // wu
    
    // Slack variable 가중치 (Equation 21)
    this->declare_parameter("k1_upper", 1e5);
    this->declare_parameter("k1_lower", 1e5);
    this->declare_parameter("k2_upper", 1e6);
    this->declare_parameter("k2_lower", 1e6);
    
    // 마찰 파라미터
    this->declare_parameter("friction_coeff", 0.8);         // μ
    this->declare_parameter("gravity", 9.81);               // g
    
    // 속도 제한
    this->declare_parameter("max_velocity", 3.0);
    this->declare_parameter("min_velocity", 0.5);
    
    // 파라미터 콜백 등록
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MPCPathTrackerCpp::parameter_callback, this, std::placeholders::_1));
    
    // 컨트롤러 생성 및 파라미터 업데이트
    controller_ = std::make_unique<MPCControllerCpp>();
    update_controller_params();
    
    // Topic 이름 설정 (use generic names so launch remappings work)
    std::string local_path_topic = "/local_path";
    std::string pose_topic = "/Ego_pose";
    std::string accel_topic = "/Accel";
    std::string pred_traj_topic = "/mpc_predicted_path";// Subscriber 생성
    local_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        local_path_topic, 10,
        std::bind(&MPCPathTrackerCpp::local_path_callback, this, std::placeholders::_1));
    
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&MPCPathTrackerCpp::pose_callback, this, std::placeholders::_1));
    
    // Publisher 생성
    auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>(accel_topic, qos_cmd);
    accel_pub_raw_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel_raw", qos_cmd);
    pred_pub_ = this->create_publisher<nav_msgs::msg::Path>(pred_traj_topic, 10);
    
    // 제어 루프 타이머 (20Hz - Ts=0.05s)
    // 논문에서는 Ts=0.2s이지만, 실시간 제어를 위해 더 빠른 주기 사용
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20Hz
        std::bind(&MPCPathTrackerCpp::control_loop, this));
    
    RCLCPP_INFO(this->get_logger(), "LTV-MPC Ready (20Hz control loop)");
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  - Prediction Horizon (N): %d", 
                this->get_parameter("prediction_horizon").as_int());
    RCLCPP_INFO(this->get_logger(), "  - Time Step (Ts): %.2f s", 
                this->get_parameter("time_step").as_double());
    RCLCPP_INFO(this->get_logger(), "  - Wheelbase (l): %.2f m", 
                this->get_parameter("wheelbase").as_double());
}

void MPCPathTrackerCpp::update_controller_params() {
    LTVMPCParams params;
    
    // 예측 파라미터
    params.N = this->get_parameter("prediction_horizon").as_int();
    params.Ts = this->get_parameter("time_step").as_double();
    params.NS = this->get_parameter("soft_constraint_horizon").as_int();
    
    // 차량 파라미터
    params.l = this->get_parameter("wheelbase").as_double();
    params.vehicle_width = this->get_parameter("vehicle_width").as_double();
    
    // 입력 제약
    params.u_min = this->get_parameter("u_min").as_double();
    params.u_max = this->get_parameter("u_max").as_double();
    
    // 곡률 제약
    params.kappa_min_delta = this->get_parameter("kappa_min_delta").as_double();
    params.kappa_max_delta = this->get_parameter("kappa_max_delta").as_double();
    
    // 비용 함수 가중치
    params.wd = this->get_parameter("weight_position").as_double();
    params.wtheta = this->get_parameter("weight_heading").as_double();
    params.wkappa = this->get_parameter("weight_curvature").as_double();
    params.wu = this->get_parameter("weight_input").as_double();
    
    // Slack variable 가중치
    params.k1_upper = this->get_parameter("k1_upper").as_double();
    params.k1_lower = this->get_parameter("k1_lower").as_double();
    params.k2_upper = this->get_parameter("k2_upper").as_double();
    params.k2_lower = this->get_parameter("k2_lower").as_double();
    
    // 마찰 파라미터
    params.mu = this->get_parameter("friction_coeff").as_double();
    params.g = this->get_parameter("gravity").as_double();
    
    // 속도 제한
    params.max_velocity = this->get_parameter("max_velocity").as_double();
    params.min_velocity = this->get_parameter("min_velocity").as_double();
    
    controller_->update_parameters(params);
}

rcl_interfaces::msg::SetParametersResult MPCPathTrackerCpp::parameter_callback(
    const std::vector<rclcpp::Parameter>& params) {
    
    for (const auto& param : params) {
        RCLCPP_INFO(this->get_logger(), "Parameter updated: %s", param.get_name().c_str());
    }
    
    update_controller_params();
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "LTV-MPC parameters updated successfully";
    return result;
}

void MPCPathTrackerCpp::local_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    local_path_ = msg->poses;
}

void MPCPathTrackerCpp::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = msg->pose;
}

CollisionConstraints MPCPathTrackerCpp::compute_collision_constraints() {
    // TODO: 다른 차량들의 위치 정보를 구독하여 충돌 회피 제약조건 계산
    // 현재는 기본 제약조건만 반환 (양옆 2m)
    
    CollisionConstraints constraints;
    
    // 기본 lateral 제약: -2.0m ~ 2.0m
    // 실제로는 collision_avoidance_node로부터 동적 제약을 받아야 함
    
    return constraints;
}

void MPCPathTrackerCpp::control_loop() {
    // 경로나 현재 위치가 없으면 정지
    if (local_path_.empty() || !current_pose_) {
        publish_control(0.0, 0.0);
        return;
    }
    
    if (local_path_.size() < 3) {
        publish_control(0.0, 0.0);
        return;
    }
    
    // 충돌 회피 제약조건 계산 (Equation 8)
    CollisionConstraints constraints = compute_collision_constraints();
    
    // LTV-MPC 제어 계산
    auto output = controller_->compute_control(*current_pose_, local_path_, constraints);
    
    // 제어 명령 발행
    publish_control(output.velocity, output.angular_velocity);
    
    // 예측 궤적 발행 (visualization용)
    publish_predicted_path(output.predicted_trajectory);
    
    // 로깅 (2초마다)
    auto now = this->now();
    if ((now - last_log_time_).seconds() > 2.0) {
        RCLCPP_INFO(this->get_logger(), 
                   "LTV-MPC | v: %.2f m/s, ω: %.3f rad/s",
                   output.velocity, output.angular_velocity);
        last_log_time_ = now;
    }
}

void MPCPathTrackerCpp::publish_control(double v, double w) {
    // Keep original semantics used in this project:
    //   linear.x  = velocity [m/s]
    //   angular.z = yaw-rate  [rad/s]
    geometry_msgs::msg::Accel msg;
    msg.linear.x = v;
    msg.angular.z = w;

    if (accel_pub_) accel_pub_->publish(msg);         // /Accel -> remapped to /CAV_XX_accel
    if (accel_pub_raw_) accel_pub_raw_->publish(msg); // /Accel_raw -> optional remap/debug
}

void MPCPathTrackerCpp::publish_predicted_path(
    const std::vector<std::array<double, 3>>& traj) {
    
    auto msg = nav_msgs::msg::Path();
    msg.header.frame_id = "world";
    msg.header.stamp = this->now();
    
    for (const auto& pt : traj) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = pt[0];
        pose.pose.position.y = pt[1];
        pose.pose.position.z = pt[2];
        pose.pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
    }
    
    pred_pub_->publish(msg);
}

}  // namespace bisa

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bisa::MPCPathTrackerCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
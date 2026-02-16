#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <optional>

namespace bisa {

class PriorityCollisionGate : public rclcpp::Node {
 public:
  PriorityCollisionGate() : Node("priority_collision_gate") {
    this->declare_parameter("cav_id", 1);
    this->declare_parameter("safety_threshold", 0.6);
    this->declare_parameter("release_threshold", 1.2);
    this->declare_parameter("roundabout_center_x", 0.574);
    this->declare_parameter("roundabout_center_y", -0.291);
    this->declare_parameter("roundabout_radius", 1.9);

    cav_id_ = this->get_parameter("cav_id").as_int();
    safety_threshold_ = this->get_parameter("safety_threshold").as_double();
    release_threshold_ = this->get_parameter("release_threshold").as_double();
    center_x_ = this->get_parameter("roundabout_center_x").as_double();
    center_y_ = this->get_parameter("roundabout_center_y").as_double();
    roundabout_radius_ = this->get_parameter("roundabout_radius").as_double();

    accel_sub_ = this->create_subscription<geometry_msgs::msg::Accel>(
        "/Accel_raw", 10,
        std::bind(&PriorityCollisionGate::accelCallback, this, std::placeholders::_1));

    ego_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/Ego_pose", rclcpp::SensorDataQoS(),
        std::bind(&PriorityCollisionGate::egoPoseCallback, this, std::placeholders::_1));

    hv19_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_19", rclcpp::SensorDataQoS(),
        std::bind(&PriorityCollisionGate::hv19PoseCallback, this, std::placeholders::_1));

    hv20_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_20", rclcpp::SensorDataQoS(),
        std::bind(&PriorityCollisionGate::hv20PoseCallback, this, std::placeholders::_1));

    accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);
  }

 private:
  static double distance(double x1, double y1, double x2, double y2) {
    return std::hypot(x1 - x2, y1 - y2);
  }

  bool isInsideRoundabout(double x, double y) const {
    return distance(x, y, center_x_, center_y_) <= roundabout_radius_;
  }

  bool isHighPriorityCav() const { return cav_id_ == 1 || cav_id_ == 2; }

  void egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    ego_x_ = msg->pose.position.x;
    ego_y_ = msg->pose.position.y;
  }

  void hv19PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    hv19_x_ = msg->pose.position.x;
    hv19_y_ = msg->pose.position.y;
  }

  void hv20PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    hv20_x_ = msg->pose.position.x;
    hv20_y_ = msg->pose.position.y;
  }

  bool checkConflictDistance(double threshold, double &min_d) const {
    if (!ego_x_.has_value() || !ego_y_.has_value()) return false;
    if (!isInsideRoundabout(*ego_x_, *ego_y_)) return false;

    bool conflict = false;
    min_d = 1e9;

    auto eval_hv = [&](const std::optional<double> &hx, const std::optional<double> &hy) {
      if (!hx.has_value() || !hy.has_value()) return;
      if (!isInsideRoundabout(*hx, *hy)) return;
      const double d = distance(*ego_x_, *ego_y_, *hx, *hy);
      min_d = std::min(min_d, d);
      if (d < threshold) conflict = true;
    };

    eval_hv(hv19_x_, hv19_y_);
    eval_hv(hv20_x_, hv20_y_);
    return conflict;
  }

  void accelCallback(const geometry_msgs::msg::Accel::SharedPtr msg) {
    if (!ego_x_.has_value() || !ego_y_.has_value()) {
      accel_pub_->publish(*msg);
      return;
    }

    double min_d = 1e9;
    const bool danger_now = checkConflictDistance(safety_threshold_, min_d);

    if (danger_now && !isHighPriorityCav()) {
      is_blocking_ = true;
    } else if (is_blocking_) {
      double release_min_d = 1e9;
      const bool still_close = checkConflictDistance(release_threshold_, release_min_d);
      if (!still_close) is_blocking_ = false;
    }

    geometry_msgs::msg::Accel gated = *msg;
    if (is_blocking_) {
      gated.linear.x = 0.0;
      gated.angular.z = 0.0;
    }

    accel_pub_->publish(gated);
  }

  int cav_id_{1};
  double safety_threshold_{0.6};
  double release_threshold_{1.2};
  double center_x_{0.574};
  double center_y_{-0.291};
  double roundabout_radius_{1.9};
  bool is_blocking_{false};

  std::optional<double> ego_x_;
  std::optional<double> ego_y_;
  std::optional<double> hv19_x_;
  std::optional<double> hv19_y_;
  std::optional<double> hv20_x_;
  std::optional<double> hv20_y_;

  rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr accel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr hv19_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr hv20_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
};

}  // namespace bisa

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bisa::PriorityCollisionGate>());
  rclcpp::shutdown();
  return 0;
}


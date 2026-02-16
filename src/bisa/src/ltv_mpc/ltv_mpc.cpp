#include "ltv_mpc.hpp"

#include <algorithm>
#include <cmath>

namespace bisa {

LTVMPC::LTVMPC(const LTVMPCConfig& cfg)
    : cfg_(cfg), model_(cfg), cost_(cfg), solver_(cfg), kappa_state_(0.0) {}

void LTVMPC::setConfig(const LTVMPCConfig& cfg) {
  cfg_ = cfg;
  model_.setConfig(cfg);
  cost_.setConfig(cfg);
  solver_.setConfig(cfg);
}

void LTVMPC::reset() {
  kappa_state_ = 0.0;
  kappa_initialized_ = false;
}

void LTVMPC::buildInputConstraints(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l,
                                   Eigen::VectorXd& u) const {
  const int N = cfg_.N;
  A.resize(N, N);
  A.setIdentity();
  l = Eigen::VectorXd::Constant(N, cfg_.u_min);
  u = Eigen::VectorXd::Constant(N, cfg_.u_max);
}

MPCCommand LTVMPC::computeControl(
    const geometry_msgs::msg::Pose& ego_pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& local_path) {
  MPCCommand out;

  if (local_path.size() < 5) return out;

  Eigen::VectorXd x0, z_bar, v_bar;
  std::vector<double> theta_r_seq;
  std::vector<double> kappa_r_seq;
  model_.buildReferenceProfiles(ego_pose, local_path, kappa_state_, x0, z_bar, v_bar,
                                theta_r_seq, kappa_r_seq);

  if (x0.size() != kLTVNx || z_bar.size() != cfg_.N || v_bar.size() != cfg_.N) {
    return out;
  }

  // Initialize curvature state from path reference once at startup.
  if (!kappa_initialized_ && !kappa_r_seq.empty()) {
    kappa_state_ = std::clamp(kappa_r_seq[0], cfg_.kappa_min, cfg_.kappa_max);
    kappa_initialized_ = true;
  }

  Eigen::MatrixXd A_bar, B_bar, E_bar, C_bar;
  model_.buildBatchDynamics(v_bar, A_bar, B_bar, E_bar, C_bar);

  Eigen::MatrixXd Q_bar, R_bar;
  cost_.buildStateInputWeights(Q_bar, R_bar);

  Eigen::SparseMatrix<double> P;
  Eigen::VectorXd q;
  cost_.buildQPObjective(x0, z_bar, A_bar, B_bar, E_bar, Q_bar, R_bar, P, q);

  Eigen::SparseMatrix<double> A_cons;
  Eigen::VectorXd l_cons, u_cons;
  buildInputConstraints(A_cons, l_cons, u_cons);

  Eigen::VectorXd u_opt;
  const bool solved = solver_.solve(P, q, A_cons, l_cons, u_cons, u_opt);
  if (!solved || u_opt.size() == 0) return out;

  const double u0 = u_opt(0);
  kappa_state_ += cfg_.Ts * u0;
  kappa_state_ = std::max(cfg_.kappa_min, std::min(cfg_.kappa_max, kappa_state_));

  const double v_cmd =
      std::max(cfg_.min_velocity, std::min(cfg_.max_velocity, v_bar(0)));
  const double omega_cmd = v_cmd * kappa_state_;

  out.solved = true;
  out.kappa_cmd = kappa_state_;
  out.v_cmd = v_cmd;
  out.omega_cmd = omega_cmd;
  out.u_seq.resize(u_opt.size());
  for (int i = 0; i < u_opt.size(); ++i) out.u_seq[i] = u_opt(i);

  out.predicted_xy.reserve(cfg_.N);
  double px = ego_pose.position.x;
  double py = ego_pose.position.y;
  const double qnorm =
      std::sqrt(ego_pose.orientation.x * ego_pose.orientation.x +
                ego_pose.orientation.y * ego_pose.orientation.y +
                ego_pose.orientation.z * ego_pose.orientation.z +
                ego_pose.orientation.w * ego_pose.orientation.w);
  double yaw = 0.0;
  if (std::isfinite(qnorm) && qnorm > 1e-6 &&
      std::abs(qnorm - 1.0) <= 0.15) {
    const double x = ego_pose.orientation.x / qnorm;
    const double y = ego_pose.orientation.y / qnorm;
    const double z = ego_pose.orientation.z / qnorm;
    const double w = ego_pose.orientation.w / qnorm;
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  } else {
    yaw = ego_pose.orientation.z;
  }

  double kappa_pred = kappa_state_;
  for (int k = 0; k < cfg_.N; ++k) {
    const double vk = (k < v_bar.size()) ? v_bar(k) : v_bar(v_bar.size() - 1);
    const double uk = (k < u_opt.size()) ? u_opt(k) : 0.0;
    kappa_pred += cfg_.Ts * uk;
    kappa_pred = std::max(cfg_.kappa_min, std::min(cfg_.kappa_max, kappa_pred));
    const double omega = vk * kappa_pred;
    px += vk * std::cos(yaw) * cfg_.Ts;
    py += vk * std::sin(yaw) * cfg_.Ts;
    yaw += omega * cfg_.Ts;
    out.predicted_xy.push_back({px, py, 0.0});
  }

  return out;
}

}  // namespace bisa

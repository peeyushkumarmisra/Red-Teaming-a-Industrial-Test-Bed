#ifndef CONTEXT_AWARE_IDS_EKF_ESTIMATOR_HPP_
#define CONTEXT_AWARE_IDS_EKF_ESTIMATOR_HPP_

#include <mutex>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace context_aware_ids
{
class EKFEstimator
{
public:
  // State Dimensions (7 coff of friction and payload mass)
  static constexpr int S_DIM = 8;
  // Measured Dimension (Residual)
  static constexpr int M_DIM = 1;
  explicit EKFEstimator(const pinocchio::Model& model); // Constructor
  // Updates parameters ONLY if attack_detected is false
  Eigen::VectorXd update(
    const Eigen::VectorXd& q_true,
    const Eigen::VectorXd& qd_true,
    const Eigen::VectorXd& qdd_true,
    const Eigen::VectorXd& tau_mass,
    bool attacked
  );
  // Current Parameter Estimation
  Eigen::VectorXd getFrictionCoff() const;
  double getPayloadMass() const;
  // Resting mass when contexted is switched
  void resetPayloadMass(double mass);

private:
  // To Evaluate Measurement Model
  double computeResidualNorm(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& qd,
        const Eigen::VectorXd& qdd,
        const Eigen::VectorXd& m_tau,
        const Eigen::VectorXd& state_x);
  Eigen::VectorXd x_; // State vector [c_fric and m_payload]^T
  Eigen::MatrixXd P_; // Error covariance matrix
  Eigen::MatrixXd Q_; // Process noise covariance (for allowing slow drift)
  double R_;          // Measurement noise variance
  // Private model + data for ekf
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::JointIndex ee_joint_id_ = 0;
  bool model_initialized_ = false;
  mutable std::mutex ekf_mutex_; // Thread safety for asynchronous access
};

} // namespace context_aware_ids

#endif // CONTEXT_AWARE_IDS_EKF_ESTIMATOR_HPP_
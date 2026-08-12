#include "context_aware_ids/ekf_estimator.hpp"
#include <pinocchio/algorithm/rnea.hpp>
#include <iostream>
#include <memory>

namespace context_aware_ids
{
EKFEstimator::EKFEstimator(const pinocchio::Model& model) : model_(model),
    data_(std::make_unique<pinocchio::Data>(model_)),
    ee_joint_id_(model_.joints.empty() ? 0 : model_.joints.size() - 1)
{
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    // Initialising 
    x_ = Eigen::VectorXd::Zero(S_DIM);                      // state vector x
    P_ = Eigen::MatrixXd::Identity(S_DIM, S_DIM) * 1.0;     // Initial Error covariance
    Q_ = Eigen::MatrixXd::Identity(S_DIM, S_DIM) * 1e-6;    // Process noise covariance
    R_ = 0.1;                                               // Measurement noise
}

double EKFEstimator::computeResidualNorm(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& qdd,
    const Eigen::VectorXd& m_tau,
    const Eigen::VectorXd& state_x) 
{
    // Extract parameters from state vector x
    Eigen::VectorXd friction_coeffs = state_x.head(7);
    double payload_mass = state_x(7);
    // Setting payload mass in model
    pinocchio::Inertia old_inertia = model_.inertias[ee_joint_id_];
    model_.inertias[ee_joint_id_] = pinocchio::Inertia(
        payload_mass, old_inertia.lever(), old_inertia.inertia());
    // Computing torques
    Eigen::VectorXd tau = pinocchio::rnea(model_, *data_, q, qd, qdd);
    // Adding friction torque
    Eigen::VectorXd tau_fric = (friction_coeffs.array() * qd.array()).matrix();
    Eigen::VectorXd tau_exp = tau + tau_fric;
    // Residual
    Eigen::VectorXd residual = m_tau - tau_exp;
    // L2 norm of residual
    return residual.norm();
}

Eigen::VectorXd EKFEstimator::update(
    const Eigen::VectorXd& q_true,
    const Eigen::VectorXd& qd_true,
    const Eigen::VectorXd& qdd_true,
    const Eigen::VectorXd& m_tau,
    bool attacked)
{   
    // To prevent a Data race
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    // Preventing attacks to be absorbed as mechanical wear
    if (attacked){return x_;} // Skip Update
    // Predicting Step
    Eigen::MatrixXd P_pred = P_ + Q_;
    // Computing  Measurement Jacobian H
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(M_DIM, S_DIM); 
    double epsilon = 1e-5;
    double z_pred = computeResidualNorm(q_true, qd_true, qdd_true, m_tau, x_);
    for (int i = 0; i< S_DIM; ++i)
    {
        Eigen::VectorXd x_perturbed = x_;
        x_perturbed(i) += epsilon;
        double z_perturbed = computeResidualNorm(q_true, qd_true, qdd_true, m_tau, x_perturbed);
        H(0,i) = (z_perturbed - z_pred) / epsilon;
    }
    // Update Step (Kalman Gain)
    double S = (H * P_pred * H.transpose())(0,0)+ R_;
    Eigen::VectorXd K = P_pred * H.transpose() / S;
    double z = 0.0; 
    x_ = x_ + K * (z-z_pred);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(S_DIM, S_DIM);
    P_ = (I-K*H) * P_pred;

     //After updating state, syncing the model to the NEW best estimate
    {
        pinocchio::Inertia old_inertia = model_.inertias[ee_joint_id_];
        model_.inertias[ee_joint_id_] = pinocchio::Inertia(
            x_(7), old_inertia.lever(), old_inertia.inertia());
    }
    return x_;
}

Eigen::VectorXd EKFEstimator::getFrictionCoff() const
{
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    return x_.head(7);
}

double EKFEstimator::getPayloadMass() const
{
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    return x_(7);
}

void EKFEstimator::resetPayloadMass(double mass)
{
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    x_(7) = mass;
    P_(7,7) = 1.0;  // Increasing covariance for rapid settling
}

}   // namespace context_aware_ids
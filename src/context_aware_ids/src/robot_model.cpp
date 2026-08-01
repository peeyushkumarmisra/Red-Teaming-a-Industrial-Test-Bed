#include "context_aware_ids/robot_model.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace robot_dynamics
{

pinocchio::Model RobotDynamics::getModel() const
{
    std::lock_guard<std::mutex> lock(dynamics_mutex_);
    return model_;
}

bool RobotDynamics::initialize(const std::string& urdf_path)
{
    std::lock_guard<std::mutex> lock(dynamics_mutex_);
    try
    {
        // Parse the URDF into model
        pinocchio::urdf::buildModel(urdf_path, model_);
        // Allocate the data
        data_ = std::make_unique<pinocchio::Data>(model_);
        // if the model is somehow empty
        if (model_.joints.empty()) {
            std::cerr << "[RobotDynamics] ERROR: Model has no joints." << std::endl;
            return false;
        }
        // The end-effector is the last joint in URDF
        ee_joint_id_ = model_.joints.size()-1;
        ee_mass_ = model_.inertias[ee_joint_id_].mass();
        initialized_ = true;
        std::cout <<"[RobotDynamics] Successfully loaded model from: " << urdf_path << std::endl;
        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << "[RobotDynamics] ERROR: Model initialization failed: " << e.what() << std::endl;
        return false;
    }
}

Eigen::VectorXd RobotDynamics::computeInverseDynamics(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& qdd
)
{
    std::lock_guard<std::mutex> lock(dynamics_mutex_);
    if (!initialized_){
        std::cerr << "[RobotDynamics] WARNING: computeInverseDynamics called before initialization!" << std::endl;
        return Eigen::VectorXd::Zero(model_.nv);
    }
    // Calculating tau using the RNEA
    return pinocchio::rnea(model_, *data_, q, qd, qdd);
}

Eigen::VectorXd RobotDynamics::computeGravity(const Eigen::VectorXd& q)
{
    std::lock_guard<std::mutex> lock(dynamics_mutex_);
    if (!initialized_){
        std::cerr << "[RobotDynamics] WARNING: computeGravity called before initialization!" << std::endl;

        return Eigen::VectorXd::Zero(model_.nv);
    }
    // RNEA  with zero params to isolate the gravity
    Eigen::VectorXd zero_vel = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd zero_acc = Eigen::VectorXd::Zero(model_.nv);
    return pinocchio::rnea(model_, *data_, q, zero_vel, zero_acc);
}

void RobotDynamics::setEndEffectorMass(double mass)
{
    std::lock_guard<std::mutex> lock(dynamics_mutex_);
    if (!initialized_) return;
    // Updating mass of end-effector for context switching
    const auto& old_inertia = model_.inertias[ee_joint_id_];
    model_.inertias[ee_joint_id_] = pinocchio::Inertia(mass, old_inertia.lever(), old_inertia.inertia());
    ee_mass_ = mass;
    std::cout << "[RobotDynamics] End-effector mass dynamically updated to: " << ee_mass_ << " kg" << std::endl;
}
}   // namespace robot_dynamics
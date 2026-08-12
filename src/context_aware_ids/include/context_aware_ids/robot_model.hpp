#ifndef CONTEXT_AWARE_IDS_ROBOT_MODEL_HPP_
#define CONTEXT_AWARE_IDS_ROBOT_MODEL_HPP_

#include <mutex>
#include <string>
#include <memory>
#include <Eigen/Dense>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace robot_dynamics
{

class RobotDynamics
{
public:
    // Singleton Access
    static RobotDynamics& getInstance()
    {
        static RobotDynamics instance;
        return instance;
    }

    // Deleting copy constructors to enforce singleton pattern
    RobotDynamics(const RobotDynamics&) = delete;
    void operator=(const RobotDynamics&) = delete;

    // Loading URDF from path
    bool initialize(const std::string& urdf_path);
    pinocchio::Model getModel() const; // For EKF

    // Computing torque (inverse dynamics)
    Eigen::VectorXd computeInverseDynamics(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& qd,
        const Eigen::VectorXd& qdd
    );

    Eigen::MatrixXd getMassMatrix(const Eigen::VectorXd& q);
    Eigen::VectorXd computeBiasForce(const Eigen::VectorXd& q, const Eigen::VectorXd& qd);
    // Computing Gravity Term
    Eigen::VectorXd computeGravity(const Eigen::VectorXd& q);

    // Upadating End Effoctor mass for context switching
    void setEndEffectorMass(double mass);

private:
    // Constructor for singleton
    RobotDynamics() = default;
    ~RobotDynamics() = default;

    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;

    bool initialized_ = false;
    mutable std::mutex dynamics_mutex_;

    pinocchio::JointIndex ee_joint_id_ = 0;
    double ee_mass_ = 0.0;

};
}   // namespace robot_dynamics
#endif // CONTEXT_AWARE_IDS_ROBOT_MODEL_HPP_
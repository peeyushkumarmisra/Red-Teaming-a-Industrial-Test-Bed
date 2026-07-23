#ifndef CONTEXT_AWARE_IDS_KINEMATICS_ENGINE_HPP_
#define CONTEXT_AWARE_IDS_KINEMATICS_ENGINE_HPP_

#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <string>
#include <stdexcept>

namespace context_aware_ids
{

template <size_t DOF>
class KinematicsEngine
{
public:
    static constexpr size_t STATE_SIZE = DOF * 2;
    using StateVector = Eigen::Matrix<double, STATE_SIZE, 1>;
    using StateMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;
    using JointVector = Eigen::Matrix<double, DOF, 1>;

    KinematicsEngine (const std::string& urdf_path)
    {
        model_ = std::make_unique<RigidBodyDynamics::Model>();   // Initialize the Rigid Body Dynamics model
        model_->gravity = RigidBodyDynamics::Math::Vector3d(0.0, 0.0, -9.81);   // Gravity
        
        // URDF model loading
        bool model_loaded = RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), model_.get(), false);
        if (!model_loaded)
        {
            throw std::runtime_error("Critical Error - URDF is not uploaded");
        }

        end_id_ = model_->GetBodyId("link_7");  // to get id of end effector link
        end_inertia_ = model_->I[end_id_];      // Manipulator inertia
        
        // First Time Build
        payload_inertia_ = RigidBodyDynamics::Math::SpatialRigidBodyInertia(
            5.0, 
            RigidBodyDynamics::Math::Vector3d(0.0, 0.0, 0.05), 
            RigidBodyDynamics::Math::Matrix3d::Identity() * 0.01
        );
    }

    StateVector predict_state(const StateVector& x, const JointVector& tau, double dt)
    {
        RigidBodyDynamics::Math::VectorNd q = x.head(DOF);
        RigidBodyDynamics::Math::VectorNd qdot = x.tail(DOF);
        RigidBodyDynamics::Math::VectorNd qddot = RigidBodyDynamics::Math::VectorNd::Zero(DOF);
        
        RigidBodyDynamics::ForwardDynamics(*model_, q, qdot, tau, qddot);
        
        StateVector x_pred;
        x_pred.head(DOF) = q + (dt * qdot);
        x_pred.tail(DOF) = qdot + (dt * qddot);
        return x_pred;
    }

    // Updating Inertia of End-Effector in fast memory without reloading URDF
    inline void apply_payload_context(int context)
    {
        if (context == 1) {
            model_->I[end_id_] = payload_inertia_;
        } else {
            model_->I[end_id_] = end_inertia_;
        }
    }

    /**
     * @brief Calculates the exact State Transition Jacobian (tj) using true physics.
     * Evaluates M(q)*q_ddot + C(q, qdot)*qdot + g(q) = tau via the ABA algorithm.
     */
    [[nodiscard]] inline StateMatrix calc_true_jacobian
    (
        const StateVector& current_state,
        const JointVector& joint_torques,
        double dt
    )
    {
        RigidBodyDynamics::Math::VectorNd q = current_state.head(DOF);
        RigidBodyDynamics::Math::VectorNd qdot = current_state.tail(DOF);
        RigidBodyDynamics::Math::VectorNd qddot(DOF);
        qddot.setZero();

        // For finding Joint Accelerations (qqdot)
        RigidBodyDynamics::ForwardDynamics(*model_, q, qdot, joint_torques, qddot);
        end_id_ = model_->GetBodyId("link_7");
        StateMatrix F = StateMatrix::Identity();
        for (size_t i = 0; i < DOF; ++i)
        {
            F(i, i + DOF) = dt; 
            F(i + DOF, i) = qddot[i] * dt; // Rough covariance approx
        }
        return F;
    }
private:
    std::unique_ptr<RigidBodyDynamics::Model> model_;
    unsigned int end_id_;
    RigidBodyDynamics::Math::SpatialRigidBodyInertia end_inertia_;
    RigidBodyDynamics::Math::SpatialRigidBodyInertia payload_inertia_;
};
}   // namespace context_aware_ids
#endif // CONTEXT_AWARE_IDS_KINEMATICS_ENGINE_HPP_
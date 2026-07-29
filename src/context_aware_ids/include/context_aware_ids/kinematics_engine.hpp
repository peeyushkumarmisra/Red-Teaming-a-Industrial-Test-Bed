#ifndef CONTEXT_AWARE_IDS_KINEMATICS_ENGINE_HPP_
#define CONTEXT_AWARE_IDS_KINEMATICS_ENGINE_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

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
            throw std::runtime_error("ERROR - URDF is not uploaded");
        }
        end_id_ = model_->GetBodyId("link_7");  // to get id of end effector link
        if (end_id_ != std::numeric_limits<unsigned int>::max())
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
    [[nodiscard]] inline StateMatrix calc_jacobian
    (
        const StateVector& current_state,
        const JointVector& joint_torques,
        double dt
    )
    {
        using VectorNd  = RigidBodyDynamics::Math::VectorNd;
        VectorNd q      = current_state.head(DOF);
        VectorNd qdot   = current_state.tail(DOF);
        VectorNd qddot  = VectorNd::Zero(DOF);
        RigidBodyDynamics::ForwardDynamics(*model_, q, qdot, joint_torques, qddot);
        StateMatrix F   = StateMatrix::Zero();
        // Upper-left:  dq_{k+1}/dq_k = I
        F.block(0, 0, DOF, DOF)     = Eigen::Matrix<double, DOF, DOF>::Identity();
        // Upper-right: dq_{k+1}/dqdot_k = dt * I
        F.block(0, DOF, DOF, DOF)   = Eigen::Matrix<double, DOF, DOF>::Identity();
        // Lower-right: dqdot_{k+1}/dqdot_k = I
        F.block(DOF, DOF, DOF, DOF) = Eigen::Matrix<double, DOF, DOF>::Identity();
        const double epsilon = 1e-7;
        // Lower-left: dt * d(qddot)/dq
        for (size_t i = 0; i<DOF; ++i)
        {
            VectorNd q_preturbed = q;
            const double h = epsilon * std::max(1.0, std::abs(q(i)));
            q_preturbed(i) += h;
            VectorNd qddot_preturbed = VectorNd::Zero(DOF);
            RigidBodyDynamics::ForwardDynamics(*model_, q_preturbed, qdot, joint_torques, qddot_preturbed);
            VectorNd dqddot_dqi = (qddot_preturbed - qddot) / h;
            F.block(DOF, i, DOF, 1) = dt * dqddot_dqi;
        }
        // Lower-right: dt * d(qddot)/dqdot
        for (size_t i = 0; i<DOF; ++i)
        {
            VectorNd qdot_preturbed = qdot;
            const double h = epsilon * std::max(1.0, std::abs(qdot(i)));
            qdot_preturbed(i) += h;
            VectorNd qddot_preturbed = VectorNd::Zero(DOF);
            RigidBodyDynamics::ForwardDynamics(*model_, q, qdot_preturbed, joint_torques, qddot_preturbed);
            VectorNd dqddot_dqdoti = (qddot_preturbed - qddot) / h;
            F.block(DOF, DOF+i, DOF, 1) = dt * dqddot_dqdoti;
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
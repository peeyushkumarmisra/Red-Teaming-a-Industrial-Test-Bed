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

    explicit KinematicsEngine (const std::string& urdf_path)
    {
        model_ = new RigidBodyDynamics::Model();   // Initialize the Rigid Body Dynamics model
        model_->gravity = RigidBodyDynamics::Math::Vector3d(0.0, 0.0, -9.81);   // Gravity
        
        // URDF model loading
        bool model_loaded = RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), model_, false);
        if (!model_loaded)
        {
            throw std::runtime_error("Critical Error - URDF is not uploaded");
        }

        end_id_ = model_->GetBodyId("link_7");  // to get id of end effector link
        end_inertia_ = model_->I[end_id_];       // Manipulator inertia
    }

    ~KinematicsEngine(){    delete model_;  }    // Destructor (preventing memory leaks)

    // Updating Inertia of End-Effector in fast memory without reloading URDF
    inline void apply_payload_context(int payload_context)
    {
        if (payload_context == 1)   // 5kg paylaod + last link inertia 
        {
            RigidBodyDynamics::Math::Matrix3d block_inertia;
            block_inertia << 0.008333, 0.0, 0.0,      0.0, 0.008333, 0.0,     0.0, 0.0, 0.008333; // Cube Tensor
            RigidBodyDynamics::Math::SpatialRigidBodyInertia payload_inertia 
            (
                5.0,     // Mass
                RigidBodyDynamics::Math::Vector3d(0.0, 0.0, 0.154),     // CoM at tip
                block_inertia       // 3x3 Inertia Tensor
            );
            model_->I[end_id_] = end_inertia_ + payload_inertia;
        } 
        else 
        {
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
        StateMatrix tj = StateMatrix::Identity();


        RigidBodyDynamics::Math::VectorNd q = current_state.head(DOF);
        RigidBodyDynamics::Math::VectorNd qdot = current_state.tail(DOF);
        RigidBodyDynamics::Math::VectorNd tau = joint_torques;
        RigidBodyDynamics::Math::VectorNd qddot(DOF);
        qddot.setZero();

        // For finding Joint Accelerations (qqdot)
        RigidBodyDynamics::ForwardDynamics(*model_, q, qdot, joint_torques, qddot);

        // Mapping the non-linear dynamics into the linear EKF Jacobian (tj)
        for (size_t i = 0; i< DOF; ++i) // Position changes by velocity
        {   tj(i, i+DOF) = dt;  }

        for (size_t i = 0; i< DOF; ++i) // Velocity changes by acceleration
        {
            tj(i + DOF, i) = qddot(i) * dt;
            tj(i + DOF, i + DOF) = 1.0;
        }

        return tj;
    }
private:
    RigidBodyDynamics::Model* model_;
    unsigned int end_id_;
    RigidBodyDynamics::Math::SpatialRigidBodyInertia end_inertia_;
};

}   // namespace context_aware_ids
#endif // CONTEXT_AWARE_IDS_KINEMATICS_ENGINE_HPP_
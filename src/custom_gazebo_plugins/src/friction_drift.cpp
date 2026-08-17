#include "custom_gazebo_plugins/friction_drift.hpp"

#include <chrono>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>


namespace custom_gazebo_plugins
{   
FrictionDrift::FrictionDrift() :
    curr_friction_(0.0),
    max_friction_(15.0),
    drift_rate_(0.0005),
    delay_(100.0)
{}

void FrictionDrift::Configure(
    const gz::sim::Entity &entity,                      // The model entity
    const std::shared_ptr<const sdf::Element> &sdf,     // XML parameters
    gz::sim::EntityComponentManager &ecm,               // Sim State
    gz::sim::EventManager &)                            // Event Bus
{
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)){
        gzerr << "[FrictionDrift] Plugin is not attached to a model entity." << std::endl;
        return;
    }
    // Target Joints
    std::string joint_name = "joint_a4";
    if (sdf->HasElement("joint_name")) {
        joint_name = sdf->Get<std::string>("joint_name");
    }
    if (sdf->HasElement("drift_rate")) {
        drift_rate_ = sdf->Get<double>("drift_rate");
    }
    if (sdf->HasElement("max_friction")) {
        max_friction_ = sdf->Get<double>("max_friction");
    }
    if (sdf->HasElement("delay")) { 
        delay_ = sdf->Get<double>("delay");
    }
    target_joint_ = model_.JointByName(ecm, joint_name);
    if (target_joint_ == gz::sim::kNullEntity)
    {
        gzerr << "[FrictionDrift] Target joint '" << joint_name << "' not found!" << std::endl;
        return;
    }
    // Entity Component Manager (ECM) tracks velocity and accepts force commands for joint
    ecm.CreateComponent(target_joint_, gz::sim::components::JointVelocity());
    if (!ecm.Component<gz::sim::components::JointForceCmd>(target_joint_))
    {
        ecm.CreateComponent(target_joint_, gz::sim::components::JointForceCmd({0.0}));
    }
    gzmsg << "[FrictionDrift] Attached to joint '" << joint_name 
          << "'. Degrading at rate=" << drift_rate_ 
          << "'. Delay=" << delay_ << "s"
          << "  MaxFric=" << max_friction_ << std::endl;
}

void FrictionDrift::PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm)
{
    if (info.paused) return;    // Do nothing if the sim is paused
    // Starting Drift
    double sim_time = std::chrono::duration<double>(info.simTime).count();
    if (sim_time < delay_)
    {
        curr_friction_ = 0.0;
        return; 
    }
    // Slowly Increasing Friction to max
    double dt = std::chrono::duration<double>(info.dt).count();     // Convert the timestep to seconds
    if (curr_friction_ < max_friction_)
    {
        curr_friction_ += (drift_rate_ * dt);
    }
    // Reading Current Velocity from ECS
    auto vel_comp = ecm.Component<gz::sim::components::JointVelocity>(target_joint_);
    if (!vel_comp || vel_comp->Data().empty()) return;
    double velocity = vel_comp->Data()[0];
    // Calculating friction
    double fric_tau = -curr_friction_ * velocity;
    // Injecting fric tau back to engine
    auto force_comp = ecm.Component<gz::sim::components::JointForceCmd>(target_joint_);//-------------------
    if (force_comp && !force_comp->Data().empty()){
        auto forces = force_comp->Data();
        forces[0] += fric_tau;
        ecm.SetComponentData<gz::sim::components::JointForceCmd>(target_joint_, forces);
    } else {
        ecm.CreateComponent(target_joint_, gz::sim::components::JointForceCmd({fric_tau}));
    }
}
}   // namespace custom_gazebo_plugins

// Register the plugin dynamically so Gazebo Harmonic can find it
GZ_ADD_PLUGIN(
    custom_gazebo_plugins::FrictionDrift, gz::sim::System,
    custom_gazebo_plugins::FrictionDrift::ISystemConfigure,
    custom_gazebo_plugins::FrictionDrift::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(custom_gazebo_plugins::FrictionDrift, "custom_gazebo_plugins::FrictionDrift")
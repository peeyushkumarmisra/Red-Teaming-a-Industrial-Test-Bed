#ifndef CUSTOM_GAZEBO_PLUGINS_FRICTION_DRIFT_HPP_
#define CUSTOM_GAZEBO_PLUGINS_FRICTION_DRIFT_HPP_

#include <memory>
#include <string> 
#include <chrono> 
#include <sdf/Element.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace custom_gazebo_plugins
{
class FrictionDrift : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
    FrictionDrift();
    ~FrictionDrift() override = default;
    // Loading Plugin Once
    void Configure(
        const gz::sim::Entity &entity,
        const std::shared_ptr<const sdf::Element> &sdf,
        gz::sim::EntityComponentManager &ecm,
        gz::sim::EventManager &eventMgr) override;
    // Updating at everytime
    void PreUpdate(
        const gz::sim::UpdateInfo &info,
        gz::sim::EntityComponentManager &ecm) override;

private:
    gz::sim::Model model_{gz::sim::kNullEntity};            // model_
    gz::sim::Entity target_joint_{gz::sim::kNullEntity};    // target_joint_
    double curr_friction_;
    double max_friction_;
    double drift_rate_;
    std::chrono::steady_clock::duration last_update_time_{std::chrono::steady_clock::duration::zero()};
};
}   // namespace custom_gazebo_plugins
#endif //CUSTOM_GAZEBO_PLUGINS_FRICTION_DRIFT_HPP_
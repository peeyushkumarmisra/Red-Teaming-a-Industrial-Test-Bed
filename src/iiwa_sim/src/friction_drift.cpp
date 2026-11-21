#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/float_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "iiwa_sim/friction_model.hpp"

namespace iiwa_sim {

class FrictionDrift : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
#ifdef IIWA_SIM_HAS_SYSTEM_PRIORITY
                      public gz::sim::ISystemConfigurePriority,
#endif
                      public gz::sim::ISystemPreUpdate {
 public:
#ifdef IIWA_SIM_HAS_SYSTEM_PRIORITY
  gz::sim::System::PriorityType ConfigurePriority() override { return 100; }
#endif

  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager&) override {
    const gz::sim::Model model(entity);
    if (!model.Valid(ecm)) {
      gzerr << "[FrictionDrift] must be attached to a model" << std::endl;
      return;
    }
    auto element = sdf->Clone();
    const double delay = element->Get<double>("delay", 0.0).first;
    const std::string topic =
        element->Get<std::string>("truth_topic", "/sim/friction_truth").first;
    const double rate = element->Get<double>("truth_rate", 100.0).first;
    truth_period_ = rate > 0.0 ? 1.0 / rate : 0.0;
    std::vector<FrictionChannel> channels;
    for (auto j = element->FindElement("joint"); j; j = j->GetNextElement("joint")) {
      const std::string name = j->Get<std::string>("name", "").first;
      const gz::sim::Entity joint = model.JointByName(ecm, name);
      if (joint == gz::sim::kNullEntity) {
        gzerr << "[FrictionDrift] joint not found: " << name << std::endl;
        continue;
      }
      FrictionChannel c;
      c.initial = j->Get<double>("initial", 0.0).first;
      c.rate = j->Get<double>("rate", 0.0).first;
      c.max = j->Get<double>("max", c.initial).first;
      channels.push_back(c);
      joints_.push_back(joint);
      names_.push_back(name);
      if (!ecm.Component<gz::sim::components::JointVelocity>(joint)) {
        ecm.CreateComponent(joint, gz::sim::components::JointVelocity());
      }
      gzmsg << "[FrictionDrift] " << name << ": b0=" << c.initial << " rate=" << c.rate
            << " max=" << c.max << " delay=" << delay << std::endl;
    }
    schedule_ = FrictionSchedule(delay, channels);
    composers_.assign(joints_.size(), ForceComposer{});
    torques_.assign(joints_.size(), 0.0);
    if (truth_period_ > 0.0) {
      publisher_ = node_.Advertise<gz::msgs::Float_V>(topic);
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (info.paused || joints_.empty()) {
      return;
    }
    const double t = std::chrono::duration<double>(info.simTime).count();
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const auto* velocity = ecm.Component<gz::sim::components::JointVelocity>(joints_[i]);
      const double v = velocity && !velocity->Data().empty() ? velocity->Data()[0] : 0.0;
      auto* command = ecm.Component<gz::sim::components::JointForceCmd>(joints_[i]);
      if (command == nullptr || command->Data().empty()) {
        torques_[i] = 0.0;
        continue;
      }
      torques_[i] = -schedule_.coefficient(i, t) * v;
      command->Data()[0] = composers_[i].compose(command->Data()[0], torques_[i]);
      ecm.SetChanged(joints_[i], gz::sim::components::JointForceCmd::typeId,
                     gz::sim::ComponentState::PeriodicChange);
    }
    if (truth_period_ > 0.0 && t + 1e-9 >= next_truth_) {
      next_truth_ = t + truth_period_;
      gz::msgs::Float_V msg;
      auto* stamp = msg.mutable_header()->mutable_stamp();
      const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
      stamp->set_sec(static_cast<int64_t>(ns / 1000000000));
      stamp->set_nsec(static_cast<int32_t>(ns % 1000000000));
      for (std::size_t i = 0; i < joints_.size(); ++i) {
        msg.add_data(static_cast<float>(schedule_.coefficient(i, t)));
      }
      for (std::size_t i = 0; i < joints_.size(); ++i) {
        msg.add_data(static_cast<float>(torques_[i]));
      }
      publisher_.Publish(msg);
    }
  }

 private:
  std::vector<gz::sim::Entity> joints_;
  std::vector<std::string> names_;
  std::vector<ForceComposer> composers_;
  std::vector<double> torques_;
  FrictionSchedule schedule_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  double truth_period_ = 0.01;
  double next_truth_ = 0.0;
};

}  // namespace iiwa_sim

#ifdef IIWA_SIM_HAS_SYSTEM_PRIORITY
GZ_ADD_PLUGIN(iiwa_sim::FrictionDrift, gz::sim::System, iiwa_sim::FrictionDrift::ISystemConfigure,
              iiwa_sim::FrictionDrift::ISystemConfigurePriority,
              iiwa_sim::FrictionDrift::ISystemPreUpdate)
#else
GZ_ADD_PLUGIN(iiwa_sim::FrictionDrift, gz::sim::System, iiwa_sim::FrictionDrift::ISystemConfigure,
              iiwa_sim::FrictionDrift::ISystemPreUpdate)
#endif
GZ_ADD_PLUGIN_ALIAS(iiwa_sim::FrictionDrift, "iiwa_sim::FrictionDrift")
#pragma once

#include "ids_core/config.hpp"
#include "ids_core/reference_track.hpp"
#include "ids_core/types.hpp"
#include "iiwa_control_law/robot_model.hpp"

namespace ids_core {

class ClosedLoopModel {
 public:
  ClosedLoopModel(iiwa_control_law::RobotModel& model, const ModelConfig& config,
                  InputSource source);

  void propagate(State& x, double t0, int steps, const ReferenceTrack& track,
                 const Vec7& applied_torque, Mat21* f, Mat21x7* g,
                 const Vec7* disturbance = nullptr);

  Vec7 nominalTorque(const Vec7& q, const Vec7& v, const iiwa_control_law::ReferenceSample& sample,
                     double t);

  InputSource source() const { return source_; }
  iiwa_control_law::RobotModel& robot() { return model_; }

 private:
  iiwa_control_law::RobotModel& model_;
  ModelConfig config_;
  InputSource source_;
  Mat7 da_dq_;
  Mat7 da_dv_;
  Mat7 minv_;
  Mat7 dg_dq_;
  Mat21 step_;
  Mat21x7 step_input_;
};

}  // namespace ids_core
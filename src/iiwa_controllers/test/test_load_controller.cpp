// =============================================================================
//  test_load_controller.cpp - the plugin loads in controller_manager
// -----------------------------------------------------------------------------
//  Catches packaging mistakes (plugin XML, exported symbols) that unit tests cannot see.
// =============================================================================

#include <gtest/gtest.h>

#include <memory>

#include <controller_manager/controller_manager.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

TEST(LoadReferenceTrackingController, LoadsThroughPluginlib) {
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Executor> executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  controller_manager::ControllerManager cm(executor, ros2_control_test_assets::minimal_robot_urdf,
                                           true, "test_controller_manager");
  EXPECT_NE(cm.load_controller("reference_tracking_controller",
                               "iiwa_controllers/ReferenceTrackingController"),
            nullptr);
  rclcpp::shutdown();
}

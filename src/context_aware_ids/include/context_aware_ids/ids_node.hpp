#ifndef CONTEXT_AWARE_IDS_NODE_HPP_
#define CONTEXT_AWARE_IDS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>

#include "context_aware_ids/ekf_engine.hpp"
#include "context_aware_ids/ewma_monitor.hpp"

#include <memory>
#include <string>
#include <fstream>

namespace context_aware_ids
{
class IDSNode 
    : public rclcpp::Node
{
public:
    IDSNode();

private:
    void control_loop_callback();   // Sync timer loop
    // Async network callback
    void task_context_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    bool data_received_;

    // ROS2 Communication Interface
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr context_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Composition: Unique pointers explicitly sized for 7-DOF
    std::unique_ptr<EKFEngine<7>> ekf_engine_;
    std::unique_ptr<EWMAMonitor<7>> ewma_monitor_;

    // High-speed memory caches to hold data between network messages and the timer
    Eigen::Matrix<double, 7, 1> latest_positions_;
    Eigen::Matrix<double, 7, 1> latest_torques_;
    int current_payload_context_;

    // For recoading the data
    std::ofstream csv_file_;
    uint64_t time_step_;
    std::vector<std::string> expected_joint_names_;
};
}           // namespace context_aware_ids
#endif      // CONTEXT_AWARE_IDS_NODE_HPP_
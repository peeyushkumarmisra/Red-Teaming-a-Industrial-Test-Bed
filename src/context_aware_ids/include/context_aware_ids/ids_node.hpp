#ifndef CONTEXT_AWARE_IDS_NODE_HPP_
#define CONTEXT_AWARE_IDS_NODE_HPP_

#include "context_aware_ids/dual_ewma.hpp"
#include "context_aware_ids/ekf_estimator.hpp"
#include "context_aware_ids/robot_model.hpp"

#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace context_aware_ids
{
class IDSNode 
    : public rclcpp::Node
{
public:
    IDSNode();
    ~IDSNode();

private:
    void control_loop_callback();   // Sync timer loop
    // Async network callback
    void context_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    bool data_received_{false};
    bool attacked_{false};
    int current_payload_context_{0};

     // Differentiating spike on first time
    bool first_run_{true};

    // ROS2 Communication Interface
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr context_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // For Plotting
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr residual_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr delta_pub_; 

    // Modules
    std::unique_ptr<EKFEstimator> ekf_estimator_;
    std::unique_ptr<DualEWMA> dual_ewma_;

    // Kinematic Caches
    Eigen::VectorXd q_;
    Eigen::VectorXd qd_;
    Eigen::VectorXd last_qd_;
    Eigen::VectorXd qdd_;
    Eigen::VectorXd m_tau_;
    std::vector<std::string> expected_joint_names_;

    // For recoading the data
    std::ofstream csv_file_;
    uint64_t time_step_{0};
    std::chrono::milliseconds timer_period_{1};
    rclcpp::Time last_joint_time_{0, 0, RCL_ROS_TIME};
    double max_staleness_sec_{0.01};
};
}   // namespace context_aware_ids
#endif      // CONTEXT_AWARE_IDS_NODE_HPP_
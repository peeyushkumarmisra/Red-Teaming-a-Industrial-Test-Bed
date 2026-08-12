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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

namespace context_aware_ids
{
class IDSNode 
    : public rclcpp::Node
{
public:
    IDSNode();
    ~IDSNode();

private:
    void control_loop_callback();
    void context_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
    void torque_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    bool data_received_{false};
    bool state_received_{false}; 
    bool torque_received_{false};
    bool attacked_{false};
    int current_payload_context_{0};

    // ROS2 Communication Interface
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr context_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr residual_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr delta_pub_; 
    rclcpp::TimerBase::SharedPtr timer_;

    // Modules
    std::unique_ptr<EKFEstimator> ekf_estimator_;
    std::unique_ptr<DualEWMA> dual_ewma_;

    // Kinematic Caches (From sensors)
    Eigen::VectorXd q_;
    Eigen::VectorXd qd_;
    Eigen::VectorXd qdd_;
    Eigen::VectorXd m_tau_;
    Eigen::VectorXd tau_actual_;
    std::vector<std::string> expected_joint_names_;

    // Commanded trajectory caches (from controller state)
    Eigen::VectorXd d_q_;
    Eigen::VectorXd d_qd_;
    Eigen::VectorXd d_qdd_;
    Eigen::VectorXd last_d_qd_;
    bool first_state_cb_{true};

    // Momentum observer state
    Eigen::VectorXd p_obs_;
    Eigen::VectorXd r_obs_;
    double K_obs_ = 50.0;

    // For recoading the data
    std::ofstream csv_file_;
    uint64_t time_step_{0};
    std::chrono::milliseconds timer_period_{10};
    rclcpp::Time last_joint_time_{0, 0, RCL_ROS_TIME};
    double max_staleness_sec_;
};
}   // namespace context_aware_ids
#endif      // CONTEXT_AWARE_IDS_NODE_HPP_
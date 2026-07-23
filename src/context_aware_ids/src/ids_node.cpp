#include "context_aware_ids/ids_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace context_aware_ids
{

IDSNode::IDSNode() 
    : Node("ids_node"),
    data_received_(false),
    current_payload_context_(0)
{
    // Initializing our memory caches
    latest_positions_.setZero();
    latest_torques_.setZero();

    // Declaring ROS 2 Parameters 
    this->declare_parameter<std::string>(
        "urdf_path",
        "/workspaces/thesis/install/iiwa_description/share/iiwa_description/urdf/iiwa.urdf"
    );
    std::string active_urdf_path = this->get_parameter("urdf_path").as_string();

    // Initializing maths
    RCLCPP_INFO(this->get_logger(), "Loading Maths Engine from %s", active_urdf_path.c_str());
    ekf_engine_ = std::make_unique<EKFEngine<7>>(active_urdf_path);
    ewma_monitor_ = std::make_unique<EWMAMonitor<7>>(0.1, 2.0, 0.5); // alpha = 0.1, base threshold = 2.0, mass variance allowance = 0.5
    
    // Setting up ROS 2 Communications
    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", sensor_qos, std::bind(&IDSNode::joint_state_callback, this, std::placeholders::_1)
    );
    context_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/task_context", 10, std::bind(&IDSNode::task_context_callback, this, std::placeholders::_1)
    );
    alarm_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ids_alarm", 10);
    timer_ = this->create_wall_timer(1ms, std::bind(&IDSNode::control_loop_callback, this));
    RCLCPP_INFO(this->get_logger(), "IDS Node Successfully Initialized");
}

void IDSNode::task_context_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    current_payload_context_ = msg->data;
}

void IDSNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() >= 7 && msg->effort.size() >= 7)
    {
        for (size_t i=0; i<7; ++i)
        {
            latest_positions_(i) = msg->position[i];
            latest_torques_(i) = msg->effort[i];
        }
        data_received_ = true;      // Unlock the timer loop
    }
}

void IDSNode::control_loop_callback()
{
    if (!data_received_)    {   return;   }

    // Passing Data to EKF
    auto residual = ekf_engine_->compute_residual(
        latest_positions_,
        latest_torques_,
        current_payload_context_
    );

    // Now to EWMA
    bool is_attack = ewma_monitor_->evaluate_anomaly(residual, current_payload_context_);

    // Alarm Triger
    if (is_attack)
    {
        RCLCPP_ERROR(this->get_logger(), "SECURITY FAILURE!");
        std_msgs::msg::Bool alarm_msg;
        alarm_msg.data = true;
        alarm_pub_->publish(alarm_msg);
    }
}
}   // namespace context_aware_ids

// Standard ROS 2 main
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<context_aware_ids::IDSNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
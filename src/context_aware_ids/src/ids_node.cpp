#include <cmath>
#include <chrono>
#include <vector>
#include "context_aware_ids/ids_node.hpp"

using namespace std::chrono_literals;

namespace context_aware_ids
{
IDSNode::IDSNode() 
:   Node("ids_node"),
    data_received_(false),
    current_payload_context_(0),
    time_step_(0),
    expected_joint_names_({"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"})
{
    // For recoading the data
    csv_file_.open("/workspaces/thesis/experiment_data.csv");
    if (!csv_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file");
    } else {
        csv_file_ << "TimeStep,Residual_Norm,EWMA_Norm,Active_Threshold,Attack_Flag\n";
        csv_file_.flush(); 
    }
    // Initializing our memory caches
    latest_positions_.setZero();
    latest_torques_.setZero();
    // Declaring ROS 2 Parameters 
    this->declare_parameter<std::string>("urdf_path","/workspaces/thesis/iiwa.urdf");
    std::string active_urdf_path = this->get_parameter("urdf_path").as_string();
    // Initializing maths
    RCLCPP_INFO(this->get_logger(), "Loading Maths Engine from %s", active_urdf_path.c_str());
    ekf_engine_     = std::make_unique<EKFEngine<7>>(active_urdf_path);
    // alpha = 0.1, base threshold = 2.0, mass variance allowance = 0.5
    ewma_monitor_   = std::make_unique<EWMAMonitor<7>>(0.1, 2.0, 0.5); 
    // Setting up ROS 2 Communications
    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", sensor_qos,
        std::bind(&IDSNode::joint_state_callback, this, std::placeholders::_1)
    );
    context_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/task_context", 10,
        std::bind(&IDSNode::task_context_callback, this, std::placeholders::_1)
    );
    alarm_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ids_alarm", 10);
    RCLCPP_INFO(this->get_logger(), "IDS Node Successfully Initialized");
    // Start the control loop
    timer_period_ = 1ms;
    timer_ = this->create_wall_timer(
        timer_period_, std::bind(&IDSNode::control_loop_callback, this));
}

void IDSNode::task_context_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    current_payload_context_ = msg->data;
}

void IDSNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    int matched_joints = 0;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (size_t j = 0; j < 7; ++j) {
            if (msg->name[i] == expected_joint_names_[j]) {
                latest_positions_(j) = msg->position[i];
                if (msg->effort.size() == msg->name.size() && std::isfinite(msg->effort[i])) {
                    latest_torques_(j) = msg->effort[i];
                // } else {
                //     latest_torques_(j) = 0.0; // See note below on 0 torque
                }
                matched_joints++;
                break;
            }
        }
    }
    if (matched_joints == 7) {
        data_received_ = true;
        last_joint_state_time_ = this->now();
    }
}

void IDSNode::control_loop_callback()
{
    if (!data_received_) {return;}
    // Stale telemetry detection (network / DoS / FDIA freeze)
    double stale_sec = (this->now() - last_joint_state_time_).seconds();
    if (stale_sec > max_staleness_sec_) {
        RCLCPP_ERROR(this->get_logger(),
            "STALE TELEMETRY — possible network/FDIA freeze! (%.3f s)", stale_sec);
        std_msgs::msg::Bool alarm_msg;
        alarm_msg.data = true;
        alarm_pub_->publish(alarm_msg);
        return;
    }
    // Deriving dt from timer period
    double dt = std::chrono::duration<double>(timer_period_).count();
    // Passing Data to EKF
    auto residual = ekf_engine_->compute_residual(
        latest_positions_,
        latest_torques_,
        current_payload_context_,
        dt
    );

    if (!residual.allFinite()) {
        RCLCPP_ERROR(this->get_logger(), "EKF DIVERGED — residual non-finite!");
        std_msgs::msg::Bool alarm_msg;
        alarm_msg.data = true;
        alarm_pub_->publish(alarm_msg);
        return; // will not feed a NaN residual into EWMA at all
    }

    // Now to EWMA
    bool is_attack = ewma_monitor_->evaluate_anomaly(residual, current_payload_context_);

    // For Recording Data
    double res_norm     = residual.norm();
    double ewma_norm    = ewma_monitor_->get_ewma_norm();
    double active_limit = ewma_monitor_->get_active_limit();
    csv_file_   << time_step_ << ","
                << res_norm << ","
                << ewma_norm << ","
                << active_limit << ","
                << (is_attack ? 1:0) << "\n";
    csv_file_.flush();
    // For every 100 iterations
    // if (time_step_ % 100 == 0) {csv_file_.flush();}
    time_step_++;

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
    try {
        auto node = std::make_shared<context_aware_ids::IDSNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("ids_node"),
        "NODE CRASHED WITH FATAL EXCEPTION: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
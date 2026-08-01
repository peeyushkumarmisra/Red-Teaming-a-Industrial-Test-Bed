#include <cmath>
#include <chrono>
#include <vector>
#include "context_aware_ids/ids_node.hpp"

using namespace std::chrono_literals;

namespace context_aware_ids
{
IDSNode::IDSNode() : Node("ids_node"),
    data_received_(false),
    attacked_(false),
    current_payload_context_(0),
    time_step_(0),
    max_staleness_sec_(0.01) // 10 ms limit before DoS alarm triggers
{
    // Initializing ROS 2 Parameters 
    this->declare_parameter<std::string>("urdf_path","/workspaces/thesis/iiwa.urdf");
    std::string urdf_path = this->get_parameter("urdf_path").as_string();
    // Initializing Model
    if (!robot_dynamics::RobotDynamics::getInstance().initialize(urdf_path)) {
        RCLCPP_ERROR(this->get_logger(), "ERROR: Could not load URDF into Pinocchio.");
        rclcpp::shutdown();
    }
    // Initializing Estimator & Tripwire
    ekf_estimator_ = std::make_unique<EKFEstimator>(
        robot_dynamics::RobotDynamics::getInstance().getModel());
    /*Fast EWMA: 0.2 | Slow EWMA: 0.001 | Burn-in: 5000 samples*/
    dual_ewma_ = std::make_unique<DualEWMA>(0.2, 0.001, 5000);
    
    // Memory Allocation
    q_          = Eigen::VectorXd::Zero(7);
    qd_         = Eigen::VectorXd::Zero(7);
    last_qd_    = Eigen::VectorXd::Zero(7);
    qdd_        = Eigen::VectorXd::Zero(7);
    m_tau_      = Eigen::VectorXd::Zero(7);
    expected_joint_names_ = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"};
    
    // Setting up ROS 2 Communications
    joint_sub_      = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10).best_effort(), std::bind(&IDSNode::joint_callback, this, std::placeholders::_1));
    context_sub_    = this->create_subscription<std_msgs::msg::Int8>(
        "/context", 10, std::bind(&IDSNode::context_callback, this, std::placeholders::_1));
    alarm_pub_      = this->create_publisher<std_msgs::msg::Bool>("/security/emergency_halt", 10);
    residual_pub_   = this->create_publisher<std_msgs::msg::Float64>("/ids/residual", 10);
    delta_pub_      = this->create_publisher<std_msgs::msg::Float64>("/ids/ewma_delta", 10);
    
    // For recoading the data
    csv_file_.open("/workspaces/thesis/experiment_data.csv");
    if (!csv_file_.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file");
    } 
    else 
    {
        csv_file_ << "TimeStep,Residual,EWMA_Delta,Active_Threshold,Attack_Flag\n";
        csv_file_.flush(); 
    }
    timer_ = this->create_wall_timer(timer_period_, std::bind(&IDSNode::control_loop_callback, this)); // Start the control loop
}


IDSNode::~IDSNode() 
{ 
    if (csv_file_.is_open()) 
    {
        csv_file_.close();
    } 
}


void IDSNode::context_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    if (current_payload_context_ != msg->data) 
    {
        current_payload_context_ = msg->data;
        /*Context 0 = Unloaded (0.0kg), Context 1 = Loaded (5.0kg)*/
        double new_mass = (current_payload_context_ == 1) ? 5.0:0.0; 
        // Updating to Physics Engine and Estimator
        robot_dynamics::RobotDynamics::getInstance().setEndEffectorMass(new_mass);
        ekf_estimator_->resetPayloadMass(new_mass);
        RCLCPP_INFO(this->get_logger(), "Task Context Switched. Updated physical model to payload: %.1f kg", new_mass);
    }
}

void IDSNode::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    int matched_joints = 0;
    for (size_t i = 0; i < msg->name.size(); ++i) 
    {
        for (size_t j = 0; j < 7; ++j)
        {
            if (msg->name[i] == expected_joint_names_[j])
            {
                q_(j) = msg->position[i];
                qd_(j) = msg->velocity[i];
                if (msg->effort.size() == msg->name.size() && std::isfinite(msg->effort[i]))
                {
                    m_tau_(j) = msg->effort[i];
                }
                matched_joints++;
                break;
            }
        }
    }
    if (matched_joints == 7) {
        data_received_ = true;
        last_joint_time_ = this->now();
    }
}


void IDSNode::control_loop_callback()
{
    if (!data_received_) {return;}
    // Stale telemetry detection (network / DoS / FDIA freeze)
    double stale_sec = (this->now() - last_joint_time_).seconds();
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
    qdd_ = (qd_ - last_qd_) / dt;
    last_qd_ = qd_;
    // Computing tau
    Eigen::VectorXd tau_model = robot_dynamics::RobotDynamics::getInstance().computeInverseDynamics(q_, qd_, qdd_);
    // Friction coff
    Eigen::VectorXd fric_coeff = ekf_estimator_->getFrictionCoff();
    Eigen::VectorXd tau_exp = tau_model + (fric_coeff.array() * qd_.array()).matrix();
    // Calculating Torque Residual
    Eigen::VectorXd residual_vec = m_tau_ - tau_exp;
    double residual_ = residual_vec.norm();
    // Tripwire
    attacked_ = dual_ewma_->update(residual_);
    // Halt if Attacked
    if (attacked_) {
        std_msgs::msg::Bool alarm_msg; 
        alarm_msg.data = true;
        alarm_pub_->publish(alarm_msg);
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "FDIA ATTACK DETECTED! Tripwire threshold breached.");
    }

    // Updating EKF if system is safe
    ekf_estimator_->update(q_, qd_, qdd_,m_tau_, attacked_);

    // For Recording Data
    std_msgs::msg::Float64 res_msg, delta_msg;
    res_msg.data = residual_;
    delta_msg.data = dual_ewma_->getCurrentDelta();
    residual_pub_->publish(res_msg);
    delta_pub_->publish(delta_msg);
    if (csv_file_.is_open())
    {
        csv_file_<< time_step_ << ","
                << residual_ << ","
                << dual_ewma_->getCurrentDelta() << ","
                << dual_ewma_->getThreshold() << ","
                << (attacked_ ? 1:0) << "\n";
        csv_file_.flush();
    }
    time_step_++;
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
        "NODE CRASHED WITH EXCEPTION: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
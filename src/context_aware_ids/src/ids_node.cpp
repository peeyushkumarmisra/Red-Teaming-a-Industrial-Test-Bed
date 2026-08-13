#include <cmath>
#include <chrono>
#include <vector>
#include "context_aware_ids/ids_node.hpp"

using namespace std::chrono_literals;

namespace context_aware_ids
{
IDSNode::IDSNode() : Node("ids_node"),
    data_received_(false),
    state_received_(false),
    torque_received_(false),
    attacked_(false),
    current_payload_context_(0),
    time_step_(0),
    max_staleness_sec_(0.05)
{
    // Matching simulation time to Gazebo
    //this->declare_parameter<bool>("use_sim_time", true);
    //this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    
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
    /*Fast EWMA: 0.2 | Slow EWMA: 0.001 | Burn-in: 1000 samples*/
    dual_ewma_ = std::make_unique<DualEWMA>(0.2, 0.001, 30000);
    
    // Memory Allocation
    q_          = Eigen::VectorXd::Zero(7);
    qd_         = Eigen::VectorXd::Zero(7);
    qdd_        = Eigen::VectorXd::Zero(7);
    m_tau_      = Eigen::VectorXd::Zero(7);
    tau_actual_ = Eigen::VectorXd::Zero(7);
    d_q_        = Eigen::VectorXd::Zero(7);
    d_qd_       = Eigen::VectorXd::Zero(7);
    d_qdd_      = Eigen::VectorXd::Zero(7);
    last_d_qd_  = Eigen::VectorXd::Zero(7);
    p_obs_      = Eigen::VectorXd::Zero(7);
    r_obs_      = Eigen::VectorXd::Zero(7);
    expected_joint_names_ = {
        "joint_a1", "joint_a2", "joint_a3", "joint_a4",
        "joint_a5", "joint_a6", "joint_a7"};
    
    // Setting up ROS 2 Communications
    joint_sub_      = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10).best_effort(),
        std::bind(&IDSNode::joint_callback, this, std::placeholders::_1));
    context_sub_    = this->create_subscription<std_msgs::msg::Int8>(
        "/task_context", 10,
        std::bind(&IDSNode::context_callback, this, std::placeholders::_1));
    state_sub_      = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "/vulnerable_controller/controller_state", 10,
        std::bind(&IDSNode::state_callback, this, std::placeholders::_1));
    torque_sub_     = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/vulnerable_controller/torque", 10,
        std::bind(&IDSNode::torque_callback, this, std::placeholders::_1));
    alarm_pub_      = this->create_publisher<std_msgs::msg::Bool>(
        "/security/emergency_halt", 10);
    residual_pub_   = this->create_publisher<std_msgs::msg::Float64>(
        "/ids/residual", 10);
    delta_pub_      = this->create_publisher<std_msgs::msg::Float64>(
        "/ids/ewma_delta", 10);
    
    // For recoading the data
    csv_file_.open("/workspaces/thesis/experiment_data.csv");
    if (!csv_file_.is_open()){
        RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file");
    }else{ 
        csv_file_ << "TimeStep,Context,"
                  << "qd_norm,tau_actual_norm,tau_planned_norm,"
                  << "p_norm,r_norm,residual_raw,residual_clipped,"
                  << "EWMA_Fast,EWMA_Slow,EWMA_Delta,Threshold,Attack_Flag\n";
        csv_file_.flush(); 
    }
    timer_ = this->create_wall_timer(timer_period_,
        std::bind(&IDSNode::control_loop_callback, this));
}


IDSNode::~IDSNode() 
{
    if (csv_file_.is_open()) csv_file_.close();
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
        RCLCPP_INFO(this->get_logger(), 
        "Task Context Switched. Updated physical model to payload: %.1f kg", new_mass);
    }
}

void IDSNode::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    int matched_joints = 0;
    for (size_t i=0; i<msg->name.size(); ++i) 
    {
        for (size_t j=0; j<7; ++j)
        {
            if (msg->name[i] == expected_joint_names_[j])
            {   
                q_(j)   = msg->position[i];
                qd_(j)  = msg->velocity[i];
                if (msg->effort.size() == msg->name.size() && std::isfinite(msg->effort[i]))
                {   
                    m_tau_(j) = msg->effort[i]; // for logging, not for observer
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


void IDSNode::state_callback( const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
    for (size_t i=0; i<msg->joint_names.size(); ++i)
    {
        for (size_t j=0; j<7; ++j)
        {
            if (msg->joint_names[i] == expected_joint_names_[j])
            {
                if (msg->reference.positions.size() == msg->joint_names.size())
                {   d_q_(j) = msg->reference.positions[i];    }
                if (msg->reference.velocities.size() == msg->joint_names.size())
                {   d_qd_(j) = msg->reference.velocities[i];   }
                if (msg->reference.accelerations.size() == msg->joint_names.size())
                {   d_qdd_(j) = msg->reference.accelerations[i];}
                break;
            }
        }
    }
    state_received_ = true;
}


void IDSNode::torque_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 7) 
    {
        for (size_t i = 0; i < 7; ++i) 
        {
            tau_actual_(i) = msg->data[i];
        }
        torque_received_ = true;
    }
}


void IDSNode::control_loop_callback()
{
    if (!data_received_ || !state_received_ || !torque_received_) {return;}
    // Stale telemetry detection
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
    auto& dyn = robot_dynamics::RobotDynamics::getInstance();

    // Compute clean commanded acceleration if controller didn't provide it
    if (first_state_cb_) 
    {   first_state_cb_ = false;
    } else {
        // Only differentiate if controller didn't give us accelerations
        if (d_qdd_.isZero(1e-12)) { 
            d_qdd_ = (d_qd_ - last_d_qd_) / dt;}
    }
    last_d_qd_ = d_qd_;

    // MOMENTUM OBSERVER (no qdd)
    Eigen::MatrixXd M = dyn.getMassMatrix(q_);
    Eigen::VectorXd h = dyn.computeBiasForce(q_, qd_);  // h = C*qd + G

    // Generalized Momentum Observer
    Eigen::VectorXd p = M * qd_;           // measured momentum
    Eigen::VectorXd y = tau_actual_ - h;   // Commanded torque
    p_obs_ += dt * (y + r_obs_);           // integrate
    r_obs_  = K_obs_ * (p - p_obs_);       // residual feedback
    
    // Saving the raw value before clipping
    double residual_raw = r_obs_.norm();
    double residual_ = residual_raw;
    
    // Hard clip for absolute safety
    const double CLIP = 100.0;
    if (residual_ > CLIP) residual_ = CLIP;

    // Tripwire
    attacked_ = dual_ewma_->update(residual_);
    if (attacked_) {
        std_msgs::msg::Bool alarm_msg; 
        alarm_msg.data = true;
        alarm_pub_->publish(alarm_msg);
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "FDIA ATTACK DETECTED! Tripwire threshold breached.");
    }

    // EKF: feed it a clean, model-consistent acceleration
    ekf_estimator_->update(q_, qd_, d_qdd_, tau_actual_, attacked_);

    // For Telemetry
    std_msgs::msg::Float64 res_msg, delta_msg;
    res_msg.data = residual_;
    delta_msg.data = dual_ewma_->getCurrentDelta();
    residual_pub_->publish(res_msg);
    delta_pub_->publish(delta_msg);

    // For Recording Data
    double qd_norm          = qd_.norm();
    double tau_actual_norm  = tau_actual_.norm();
    double tau_planned_norm = dyn.computeInverseDynamics(d_q_, d_qd_, d_qdd_).norm();
    double p_norm           = p.norm();
    double r_norm           = r_obs_.norm();
    double ewma_fast        = dual_ewma_->getFast();
    double ewma_slow        = dual_ewma_->getSlow();
    double ewma_delta       = dual_ewma_->getCurrentDelta();
    double threshold        = dual_ewma_->getThreshold();
    if (csv_file_.is_open()) 
    {
        csv_file_ << time_step_ << ","
                  << current_payload_context_ << ","
                  << qd_norm << ","
                  << tau_actual_norm << ","
                  << tau_planned_norm << ","
                  << p_norm << ","
                  << r_norm << ","
                  << residual_raw << ","
                  << residual_ << ","
                  << ewma_fast << ","
                  << ewma_slow << ","
                  << ewma_delta << ","
                  << threshold << ","
                  << (attacked_ ? 1 : 0) << "\n";
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
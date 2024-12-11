#ifndef CONTROLS_H
#define CONTROLS_H

#include <custom_msgs/msg/control_types.hpp>
#include <custom_msgs/msg/pid_axes_info.hpp>
#include <custom_msgs/msg/pid_gain.hpp>
#include <custom_msgs/msg/pid_gains.hpp>
#include <custom_msgs/msg/thruster_allocs.hpp>
#include <custom_msgs/srv/set_control_types.hpp>
#include <custom_msgs/srv/set_pid_gains.hpp>
#include <custom_msgs/srv/set_power_scale_factor.hpp>
#include <custom_msgs/srv/set_static_power.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <string>
#include <unordered_map>

#include "controls_types.hpp"
#include "controls_utils.hpp"
#include "pid_manager.hpp"
#include "thruster_allocator.hpp"

class Controls : public rclcpp::Node {
   private:
    // Rate at which thruster allocations are published (Hz)
    static const int THRUSTER_ALLOCS_RATE;

    // Launch file parameters
    bool sim;
    bool enable_position_pid;
    bool enable_velocity_pid;
    bool cascaded_pid;

    // Transform buffer
    // Unique pointer is used to avoid writing a custom constructor and destructor for this class
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    // Transform listener
    // Shared pointer is used to avoid writing a custom constructor and destructor for this class
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // Whether controls are enabled
    bool controls_enabled;

    // Control type to use for each axis
    AxesMap<ControlTypesEnum> control_types;

    // Desired states
    geometry_msgs::msg::Pose desired_position;
    geometry_msgs::msg::Twist desired_velocity;
    AxesMap<double> desired_power;

    // Current state
    nav_msgs::msg::Odometry state;

    // Timestamp of last state message
    rclcpp::Time last_state_msg_time;

    // Managers for both position and velocity PID loops
    LoopsMap<PIDManager> pid_managers;

    // Control efforts output by PID loops
    AxesMap<double> position_pid_outputs;
    AxesMap<double> velocity_pid_outputs;

    // Values computed by PID loops
    AxesMap<PIDInfo> position_pid_infos;
    AxesMap<PIDInfo> velocity_pid_infos;

    // Thruster allocator
    ThrusterAllocator thruster_allocator;

    // Minimum and maximum desired power for each axis
    AxesMap<double> desired_power_min;
    AxesMap<double> desired_power_max;

    // Static power (to offset buoyancy and other persistent forces)
    tf2::Vector3 static_power_global;
    tf2::Vector3 static_power_local;

    // Constant multiplier for set power
    double power_scale_factor;

    // Most recent actual power
    AxesMap<double> actual_power_map;

    // ROS topic subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_position_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_velocity_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_power_sub;

    // ROS service advertisers
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_controls_srv;
    rclcpp::Service<custom_msgs::srv::SetControlTypes>::SharedPtr set_control_types_srv;
    rclcpp::Service<custom_msgs::srv::SetPIDGains>::SharedPtr set_pid_gains_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_pid_loops_srv;
    rclcpp::Service<custom_msgs::srv::SetStaticPower>::SharedPtr set_static_power_global_srv;
    rclcpp::Service<custom_msgs::srv::SetPowerScaleFactor>::SharedPtr set_power_scale_factor_srv;

    // ROS topic publishers
    rclcpp::Publisher<custom_msgs::msg::ThrusterAllocs>::SharedPtr thruster_allocs_pub;
    rclcpp::Publisher<custom_msgs::msg::ThrusterAllocs>::SharedPtr constrained_thruster_allocs_pub;
    rclcpp::Publisher<custom_msgs::msg::ThrusterAllocs>::SharedPtr unconstrained_thruster_allocs_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_power_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr set_power_unscaled_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr set_power_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr actual_power_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr power_disparity_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr power_disparity_norm_pub;
    rclcpp::Publisher<custom_msgs::msg::PIDGains>::SharedPtr pid_gains_pub;
    rclcpp::Publisher<custom_msgs::msg::ControlTypes>::SharedPtr control_types_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr position_efforts_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_efforts_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr position_error_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_error_pub;
    rclcpp::Publisher<custom_msgs::msg::PIDAxesInfo>::SharedPtr position_pid_infos_pub;
    rclcpp::Publisher<custom_msgs::msg::PIDAxesInfo>::SharedPtr velocity_pid_infos_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr delta_time_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr static_power_global_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr static_power_local_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr power_scale_factor_pub;

    // *****************************************************************************************************************
    // ROS subscriber callbacks

    /**
     * @brief Callback for desired position messages.
     *
     * @param msg Desired position message.
     *
     * @note The orientation of the desired position must be a valid unit quaternion.
     */
    void desired_position_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    /**
     * @brief Callback for desired velocity messages.
     *
     * @param msg Desired velocity message.
     */
    void desired_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Callback for desired power messages.
     *
     * @param msg Desired power message.
     *
     * @note The value of each axis in the desired power message must be in the range [-1, 1].
     */
    void desired_power_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Callback for state messages. Runs PID loops.
     *
     * @param msg State message.
     */
    void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // *****************************************************************************************************************
    // ROS service callbacks

    /**
     * @brief Callback for enabling/disabling controls.
     *
     * @param req Request indicating whether to enable/disable controls.
     * @param res Response indicating whether controls were enabled/disabled.
     * @return True if the service response was successfully filled, false otherwise.
     */
    bool enable_controls_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                  std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    /**
     * @brief Callback for setting control types.
     *
     * @param req Request indicating control types to set for each axis.
     * @param res Response indicating whether control types were set.
     * @return True if the service response was successfully filled, false otherwise.
     */
    bool set_control_types_callback(const std::shared_ptr<custom_msgs::srv::SetControlTypes::Request> req,
                                    std::shared_ptr<custom_msgs::srv::SetControlTypes::Response> res);

    /**
     * @brief Callback for updating PID gains.
     *
     * @param req Request providing updated PID gains.
     * @param res Response indicating whether PID gains were updated.
     * @return True if the service response was successfully filled, false otherwise.
     *
     * @throws rclcpp::Exception Robot config file could not be updated with the new PID gains.
     */
    bool set_pid_gains_callback(const std::shared_ptr<custom_msgs::srv::SetPIDGains::Request> req,
                                std::shared_ptr<custom_msgs::srv::SetPIDGains::Response> res);

    /**
     * @brief Callback for resetting PID loops.
     *
     * @param req Request indicating that PID loops should be reset.
     * @param res Response indicating whether PID loops were reset.
     * @return True if the service response was successfully filled, false otherwise.
     */
    bool reset_pid_loops_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    /**
     * @brief Callback for updating static power global.
     *
     * @param req Request providing updated static power global.
     * @param res Response indicating whether static power global was updated.
     * @return True if the service response was successfully filled, false otherwise.
     *
     * @throws rclcpp::Exception Robot config file could not be updated with the new static power global.
     */
    bool set_static_power_global_callback(const std::shared_ptr<custom_msgs::srv::SetStaticPower::Request> req,
                                          std::shared_ptr<custom_msgs::srv::SetStaticPower::Response> res);

    /**
     * @brief Callback for updating power scale factor.
     *
     * @param req Request providing power scale factor to set.
     * @param res Response indicating whether power scale factor was set.
     * @return True if the service response was successfully filled, false otherwise.
     *
     * @throws rclcpp::Exception Robot config file could not be updated with the new power scale factor.
     */
    bool set_power_scale_factor_callback(const std::shared_ptr<custom_msgs::srv::SetPowerScaleFactor::Request> req,
                                         std::shared_ptr<custom_msgs::srv::SetPowerScaleFactor::Response> res);

   public:
    /**
     * @brief Construct a new Controls object. Initializes PID loops, thruster allocator, ROS subscribers, service
     *  advertisers, and publishers.
     */
    Controls();

    /**
     * @brief Loop that runs while the node is active. Allocates thrusters based on control efforts and publishes
     *  thruster allocations, along with other diagnostic information.
     */
    void run();
};

#endif
#include <fmt/core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <controls.hpp>
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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>

#include "controls_types.hpp"
#include "controls_utils.hpp"
#include "pid_manager.hpp"
#include "thruster_allocator.hpp"

const int Controls::THRUSTER_ALLOCS_INTERVAL = 50;

Controls::Controls() : Node("controls") {
    // Get parameters from launch file
    this->declare_parameter<bool>("sim", false);
    this->get_parameter("sim", sim);
    this->declare_parameter<bool>("enable_position_pid", true);
    this->get_parameter("enable_position_pid", enable_position_pid);
    this->declare_parameter<bool>("enable_velocity_pid", true);
    this->get_parameter("enable_velocity_pid", enable_velocity_pid);
    this->declare_parameter<bool>("cascaded_pid", false);
    this->get_parameter("cascaded_pid", cascaded_pid);

    // Initialize controls to be disabled
    controls_enabled = false;

    // Initialize transform buffer and listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Initialize desired position to have valid orientation
    desired_position.orientation.w = 1.0;

    // Use desired position as the default control type for all axes
    ControlsUtils::populate_axes_map<ControlTypesEnum>(ControlTypesEnum::DESIRED_POSITION, control_types);

    // Get paths to wrench matrix and its pseudoinverse from robot config file
    std::string wrench_matrix_file_path;
    std::string wrench_matrix_pinv_file_path;

    // Get PID configuration parameters from robot config file
    LoopsMap<AxesMap<double>> loops_axes_control_effort_mins;
    LoopsMap<AxesMap<double>> loops_axes_control_effort_maxes;
    LoopsMap<AxesMap<PIDDerivativeTypesEnum>> loops_axes_derivative_types;
    LoopsMap<AxesMap<double>> loops_axes_error_ramp_rates;
    LoopsMap<AxesMap<PIDGainsMap>> loops_axes_pid_gains;

    // Read robot config file and populate config variables
    ControlsUtils::read_robot_config(cascaded_pid, loops_axes_control_effort_mins, loops_axes_control_effort_maxes,
                                     loops_axes_derivative_types, loops_axes_error_ramp_rates, loops_axes_pid_gains,
                                     desired_power_min, desired_power_max, static_power_global, power_scale_factor,
                                     wrench_matrix_file_path, wrench_matrix_pinv_file_path);

    // Ensure that desired power min is less than or equal to desired power max for each axis
    for (const AxesEnum &axis : AXES)
        rcpputils::check_true(
            desired_power_min.at(axis) <= desired_power_max.at(axis),
            fmt::format(
                "Invalid desired power min and max for axis {}. Desired power min must be less than or equal to max.",
                AXES_NAMES.at(axis)));

    // Instantiate PID managers for each PID loop type
    for (const PIDLoopTypesEnum &loop : PID_LOOP_TYPES)
        pid_managers[loop] = PIDManager(loops_axes_control_effort_mins.at(loop),
                                        loops_axes_control_effort_maxes.at(loop), loops_axes_derivative_types.at(loop),
                                        loops_axes_error_ramp_rates.at(loop), loops_axes_pid_gains.at(loop));

    // Initialize PID outputs and infos to zero
    for (const AxesEnum &axis : AXES) {
        position_pid_outputs[axis] = 0;
        velocity_pid_outputs[axis] = 0;
        position_pid_infos[axis] = PIDInfo();
        velocity_pid_infos[axis] = PIDInfo();
    }

    // Initialize static power local to zero
    static_power_local = tf2::Vector3(0, 0, 0);

    // Initialize axes maps to zero
    ControlsUtils::populate_axes_map<double>(0, position_pid_outputs);
    ControlsUtils::populate_axes_map<double>(0, velocity_pid_outputs);
    ControlsUtils::populate_axes_map<double>(0, desired_power);

    // Instantiate thruster allocator
    thruster_allocator = ThrusterAllocator(wrench_matrix_file_path, wrench_matrix_pinv_file_path);

    // Subscribe to input topics
    state_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "state", 1, std::bind(&Controls::state_callback, this, std::placeholders::_1));
    desired_position_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        "controls/desired_position", 1, std::bind(&Controls::desired_position_callback, this, std::placeholders::_1));
    desired_velocity_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "controls/desired_velocity", 1, std::bind(&Controls::desired_velocity_callback, this, std::placeholders::_1));
    desired_power_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "controls/desired_power", 1, std::bind(&Controls::desired_power_callback, this, std::placeholders::_1));

    // Advertise input services
    enable_controls_srv = this->create_service<std_srvs::srv::SetBool>(
        "controls/enable",
        std::bind(&Controls::enable_controls_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_control_types_srv = this->create_service<custom_msgs::srv::SetControlTypes>(
        "controls/set_control_types",
        std::bind(&Controls::set_control_types_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_pid_gains_srv = this->create_service<custom_msgs::srv::SetPIDGains>(
        "controls/set_pid_gains",
        std::bind(&Controls::set_pid_gains_callback, this, std::placeholders::_1, std::placeholders::_2));
    reset_pid_loops_srv = this->create_service<std_srvs::srv::Trigger>(
        "controls/reset_pid_loops",
        std::bind(&Controls::reset_pid_loops_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_static_power_global_srv = this->create_service<custom_msgs::srv::SetStaticPower>(
        "controls/set_static_power_global",
        std::bind(&Controls::set_static_power_global_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_power_scale_factor_srv = this->create_service<custom_msgs::srv::SetPowerScaleFactor>(
        "controls/set_power_scale_factor",
        std::bind(&Controls::set_power_scale_factor_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize publishers for output topics
    thruster_allocs_pub = this->create_publisher<custom_msgs::msg::ThrusterAllocs>("controls/thruster_allocs", 1);
    constrained_thruster_allocs_pub =
        this->create_publisher<custom_msgs::msg::ThrusterAllocs>("controls/constrained_thruster_allocs", 1);
    unconstrained_thruster_allocs_pub =
        this->create_publisher<custom_msgs::msg::ThrusterAllocs>("controls/unconstrained_thruster_allocs", 1);
    base_power_pub = this->create_publisher<geometry_msgs::msg::Twist>("controls/base_power", 1);
    set_power_unscaled_pub = this->create_publisher<geometry_msgs::msg::Twist>("controls/set_power_unscaled", 1);
    set_power_pub = this->create_publisher<geometry_msgs::msg::Twist>("controls/set_power", 1);
    actual_power_pub = this->create_publisher<geometry_msgs::msg::Twist>("controls/actual_power", 1);
    power_disparity_pub = this->create_publisher<geometry_msgs::msg::Twist>("controls/power_disparity", 1);
    power_disparity_norm_pub = this->create_publisher<std_msgs::msg::Float64>("controls/power_disparity_norm", 1);
    pid_gains_pub = this->create_publisher<custom_msgs::msg::PIDGains>("controls/pid_gains", 1);
    control_types_pub = this->create_publisher<custom_msgs::msg::ControlTypes>("controls/control_types", 1);
    position_efforts_pub = this->create_publisher<geometry_msgs::msg::Twist>("controls/position_efforts", 1);
    velocity_efforts_pub = this->create_publisher<geometry_msgs::msg::Twist>("controls/velocity_efforts", 1);
    position_error_pub = this->create_publisher<geometry_msgs::msg::Twist>("controls/position_error", 1);
    velocity_error_pub = this->create_publisher<geometry_msgs::msg::Twist>("controls/velocity_error", 1);
    position_pid_infos_pub = this->create_publisher<custom_msgs::msg::PIDAxesInfo>("controls/position_pid_infos", 1);
    velocity_pid_infos_pub = this->create_publisher<custom_msgs::msg::PIDAxesInfo>("controls/velocity_pid_infos", 1);
    status_pub = this->create_publisher<std_msgs::msg::Bool>("controls/status", 1);
    delta_time_pub = this->create_publisher<std_msgs::msg::Float64>("controls/delta_time", 1);
    static_power_global_pub = this->create_publisher<geometry_msgs::msg::Vector3>("controls/static_power_global", 1);
    static_power_local_pub = this->create_publisher<geometry_msgs::msg::Vector3>("controls/static_power_local", 1);
    power_scale_factor_pub = this->create_publisher<std_msgs::msg::Float64>("controls/power_scale_factor", 1);

    // Start timer that runs thruster allocator
    timer =
        this->create_wall_timer(std::chrono::milliseconds(THRUSTER_ALLOCS_INTERVAL), std::bind(&Controls::run, this));
}

void Controls::desired_position_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    // Make sure desired position orientation quaternion has length 1
    if (ControlsUtils::quaternion_valid(msg->orientation))
        desired_position = *msg;
    else
        RCLCPP_WARN(this->get_logger(), "Invalid desired position orientation. Quaternion must have length 1.");
}

void Controls::desired_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) { desired_velocity = *msg; }

void Controls::desired_power_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    AxesMap<double> new_desired_power;
    ControlsUtils::twist_to_map(*msg, new_desired_power);

    // Make sure desired power is within the limits specified in the config file for each axis
    // If desired power is invalid, then warn user and don't update desired power
    for (const AxesEnum &axis : AXES) {
        if (new_desired_power.at(axis) < desired_power_min.at(axis) ||
            new_desired_power.at(axis) > desired_power_max.at(axis)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid desired power of %f for axis %s. Desired power for axis %s must be within range [%f, %f].",
                new_desired_power.at(axis), AXES_NAMES.at(axis).c_str(), AXES_NAMES.at(axis).c_str(),
                desired_power_min.at(axis), desired_power_max.at(axis));
            return;
        }
    }

    // New desired power is within limits, so update desired power
    desired_power = new_desired_power;
}

void Controls::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    state = *msg;

    // Get current time, compute delta time, and update last state message time
    // If last state message time is zero, then this is the first state message received, so delta time is zero
    rclcpp::Time current_time = this->get_clock()->now();
    double delta_time = last_state_msg_time.nanoseconds() == 0 ? 0.0 : (current_time - last_state_msg_time).seconds();
    last_state_msg_time = current_time;

    // Publish delta time
    std_msgs::msg::Float64 delta_time_msg;
    delta_time_msg.data = delta_time;
    delta_time_pub->publish(delta_time_msg);

    // Don't run PID loops if delta_time is nonpositive
    // Only occurs on first state message received
    if (delta_time <= 0.0) return;

    // Get transform from odom to base_link
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer->lookupTransform("base_link", "odom", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform from odom to base_link. %s", ex.what());
        return;
    }

    // Compute position error
    geometry_msgs::msg::PoseStamped desired_position_stamped;
    desired_position_stamped.pose = desired_position;
    desired_position_stamped.header.frame_id = "odom";
    geometry_msgs::msg::PoseStamped position_error;
    tf2::doTransform(desired_position_stamped, position_error, transformStamped);

    // Convert position error pose to twist
    geometry_msgs::msg::Twist position_error_msg;
    ControlsUtils::pose_to_twist(position_error.pose, position_error_msg);

    // Publish position error message
    position_error_pub->publish(position_error_msg);

    // Convert position error twist to map
    AxesMap<double> position_error_map;
    ControlsUtils::twist_to_map(position_error_msg, position_error_map);

    // Get delta time map
    AxesMap<double> delta_time_map;
    ControlsUtils::populate_axes_map<double>(delta_time, delta_time_map);

    // Get velocity map
    AxesMap<double> velocity_map;
    ControlsUtils::twist_to_map(state.twist.twist, velocity_map);

    // Copy velocity map to position PID provided derivatives map and negate all entries
    // Entries are negated because the derivative of position error is negative velocity
    AxesMap<double> position_pid_provided_derivatives(velocity_map);
    ControlsUtils::scale_axes_map(-1, position_pid_provided_derivatives);

    // Run position PID loop
    if (enable_position_pid)
        pid_managers.at(PIDLoopTypesEnum::POSITION)
            .run_loop(position_error_map, delta_time_map, position_pid_outputs, position_pid_infos,
                      position_pid_provided_derivatives);

    // Publish position control efforts
    geometry_msgs::msg::Twist position_efforts_msg;
    ControlsUtils::map_to_twist(position_pid_outputs, position_efforts_msg);
    position_efforts_pub->publish(position_efforts_msg);

    // Publish position PID infos
    custom_msgs::msg::PIDAxesInfo position_pid_infos_msg;
    ControlsUtils::pid_axes_map_info_struct_to_msg(position_pid_infos, position_pid_infos_msg);
    position_pid_infos_pub->publish(position_pid_infos_msg);

    // Get desired velocity map
    AxesMap<double> desired_velocity_map;
    ControlsUtils::twist_to_map(desired_velocity, desired_velocity_map);

    // Get velocity error map
    // For each axis, if PID is cascaded and control type is position, then use position PID output as velocity setpoint
    // Otherwise, use desired velocity as velocity setpoint
    AxesMap<double> velocity_error_map;
    for (const AxesEnum &axis : AXES) {
        double velocity_setpoint = (cascaded_pid && control_types.at(axis) == ControlTypesEnum::DESIRED_POSITION)
                                       ? position_pid_outputs.at(axis)
                                       : desired_velocity_map.at(axis);
        velocity_error_map[axis] = velocity_setpoint - velocity_map[axis];
    }

    // Publish velocity error
    geometry_msgs::msg::Twist velocity_error;
    ControlsUtils::map_to_twist(velocity_error_map, velocity_error);
    velocity_error_pub->publish(velocity_error);

    // Copy actual power map to velocity PID provided derivatives map and negate all entries
    // Entries are negated because the derivative of velocity error is negative acceleration
    AxesMap<double> velocity_pid_provided_derivatives(actual_power_map);
    ControlsUtils::scale_axes_map(-1, velocity_pid_provided_derivatives);

    // Run velocity PID loop
    if (enable_velocity_pid)
        pid_managers.at(PIDLoopTypesEnum::VELOCITY)
            .run_loop(velocity_error_map, delta_time_map, velocity_pid_outputs, velocity_pid_infos,
                      velocity_pid_provided_derivatives);

    // Publish velocity control efforts
    geometry_msgs::msg::Twist velocity_efforts_msg;
    ControlsUtils::map_to_twist(velocity_pid_outputs, velocity_efforts_msg);
    velocity_efforts_pub->publish(velocity_efforts_msg);

    // Publish velocity PID infos
    custom_msgs::msg::PIDAxesInfo velocity_pid_infos_msg;
    ControlsUtils::pid_axes_map_info_struct_to_msg(velocity_pid_infos, velocity_pid_infos_msg);
    velocity_pid_infos_pub->publish(velocity_pid_infos_msg);

    // Rotate static power global to equivalent vector in robot's local frame
    // Given vector v in base_link frame, v * state.pose.pose.orientation = v', where v' is the vector in the odom frame
    // that points in the same direction as v in the base_link frame, as seen by an external observer.
    // Here, we have static_power_global, a vector in the odom frame, and we would like static_power_local, a vector
    // in the base_link frame, that points in the same direction as static_power_global in the odom frame. Thus, we
    // are performing the inverse of the operation described above.
    // static_power_global * state.pose.pose.orientation.inverse() = static_power_local
    tf2::Quaternion orientation_tf2;
    tf2::fromMsg(state.pose.pose.orientation, orientation_tf2);
    static_power_local = quatRotate(orientation_tf2.inverse(), static_power_global);

    // Publish static power local
    geometry_msgs::msg::Vector3 static_power_local_msg = tf2::toMsg(static_power_local);
    static_power_local_pub->publish(static_power_local_msg);
}

bool Controls::enable_controls_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                                        std_srvs::srv::SetBool::Response::SharedPtr res) {
    controls_enabled = req->data;
    res->success = true;
    res->message = controls_enabled ? "Controls enabled." : "Controls disabled.";
    return true;
}

bool Controls::set_control_types_callback(const custom_msgs::srv::SetControlTypes::Request::SharedPtr req,
                                          custom_msgs::srv::SetControlTypes::Response::SharedPtr res) {
    res->success = ControlsUtils::control_types_to_map(req->control_types, control_types);
    res->message = res->success ? "Updated control types successfully."
                                : "Failed to update control types. One or more control types was invalid.";
    return true;
}

bool Controls::set_pid_gains_callback(const custom_msgs::srv::SetPIDGains::Request::SharedPtr req,
                                      custom_msgs::srv::SetPIDGains::Response::SharedPtr res) {
    res->success = ControlsUtils::pid_gains_valid(req->pid_gains);

    // If PID gains were valid, then update PID gains in PID managers and robot config file
    // Otherwise, do nothing
    if (res->success) {
        // Update PID gains through PID managers
        for (const custom_msgs::msg::PIDGain &pid_gain_update : req->pid_gains) {
            PIDLoopTypesEnum loop = static_cast<PIDLoopTypesEnum>(pid_gain_update.loop);
            AxesEnum axis = static_cast<AxesEnum>(pid_gain_update.axis);
            PIDGainTypesEnum gain_type = static_cast<PIDGainTypesEnum>(pid_gain_update.gain);
            pid_managers.at(loop).set_pid_gain(axis, gain_type, pid_gain_update.value);
        }

        // Get all PID gains, including those that were updated and those that were not updated
        LoopsMap<AxesMap<PIDGainsMap>> loops_axes_pid_gains;
        for (const PIDLoopTypesEnum &loop : PID_LOOP_TYPES)
            loops_axes_pid_gains[loop] = pid_managers.at(loop).get_axes_pid_gains();

        // Update robot config file with updated PID gains
        // Throws an exception if file could not be updated successfully; will shut down the node
        ControlsUtils::update_robot_config_pid_gains(loops_axes_pid_gains, cascaded_pid);
    }

    res->message = res->success ? "Updated PID gains successfully."
                                : "Failed to update PID gains. One or more PID gains was invalid.";

    return true;
}

bool Controls::reset_pid_loops_callback(const std_srvs::srv::Trigger::Request::SharedPtr,
                                        std_srvs::srv::Trigger::Response::SharedPtr res) {
    // Reset all PID loops
    for (const PIDLoopTypesEnum &loop : PID_LOOP_TYPES) pid_managers[loop].reset();

    res->success = true;
    res->message = "Reset PID loops successfully.";
    return true;
}

bool Controls::set_static_power_global_callback(const custom_msgs::srv::SetStaticPower::Request::SharedPtr req,
                                                custom_msgs::srv::SetStaticPower::Response::SharedPtr res) {
    tf2::fromMsg(req->static_power, static_power_global);

    // Update static power in robot config file
    // Throws an exception if file could not be updated successfully; will shut down the node
    ControlsUtils::update_robot_config_static_power_global(static_power_global);

    res->success = true;
    res->message = "Updated static power successfully.";

    return true;
}

bool Controls::set_power_scale_factor_callback(const custom_msgs::srv::SetPowerScaleFactor::Request::SharedPtr req,
                                               custom_msgs::srv::SetPowerScaleFactor::Response::SharedPtr res) {
    power_scale_factor = req->power_scale_factor;

    // Update power scale factor in robot config file
    // Throws an exception if file could not be updated successfully; will shut down the node
    ControlsUtils::update_robot_config_power_scale_factor(power_scale_factor);

    res->success = true;
    res->message = "Updated power scale factor successfully.";

    return true;
}

void Controls::run() {
    // Get set power based on control types
    for (int i = 0; i < AXES_COUNT; i++) {
        switch (control_types.at(AXES[i])) {
            case custom_msgs::msg::ControlTypes::DESIRED_POSITION:
                // If cascaded PID is enabled, then use velocity PID outputs as set power for DESIRED_POSITION
                // control type
                base_power[i] = (cascaded_pid) ? velocity_pid_outputs.at(AXES[i]) : position_pid_outputs.at(AXES[i]);
                break;
            case custom_msgs::msg::ControlTypes::DESIRED_VELOCITY:
                base_power[i] = velocity_pid_outputs.at(AXES[i]);
                break;
            case custom_msgs::msg::ControlTypes::DESIRED_POWER:
                base_power[i] = desired_power.at(AXES[i]);
                break;
        }
    }

    // Convert static power local to map
    AxesMap<double> static_power_local_map;
    ControlsUtils::tf_linear_vector_to_map(static_power_local, static_power_local_map);

    // Sum static power local and base power to get set power unscaled
    for (const AxesEnum &axis : AXES) set_power_unscaled[axis] = base_power[axis] + static_power_local_map.at(axis);

    // Apply power scale factor to set power
    set_power = set_power_unscaled * power_scale_factor;

    // Allocate thrusters
    thruster_allocator.allocate_thrusters(set_power, unconstrained_allocs, constrained_allocs, actual_power,
                                          power_disparity);

    // Convert thruster allocation vector to message
    ControlsUtils::eigen_vector_to_thruster_allocs_msg(constrained_allocs, constrained_allocs_msg);

    // Publish thruster allocs if controls are enabled
    if (controls_enabled) thruster_allocs_pub->publish(constrained_allocs_msg);

    // Save actual power to map
    ControlsUtils::eigen_vector_to_map(actual_power, actual_power_map);

    // Publish all other messages
    constrained_thruster_allocs_pub->publish(constrained_allocs_msg);

    ControlsUtils::eigen_vector_to_thruster_allocs_msg(unconstrained_allocs, unconstrained_allocs_msg);
    unconstrained_thruster_allocs_pub->publish(unconstrained_allocs_msg);

    ControlsUtils::eigen_vector_to_twist(base_power, base_power_msg);
    base_power_pub->publish(base_power_msg);

    ControlsUtils::eigen_vector_to_twist(set_power_unscaled, set_power_unscaled_msg);
    set_power_unscaled_pub->publish(set_power_unscaled_msg);

    ControlsUtils::eigen_vector_to_twist(set_power, set_power_msg);
    set_power_pub->publish(set_power_msg);

    ControlsUtils::eigen_vector_to_twist(actual_power, actual_power_msg);
    actual_power_pub->publish(actual_power_msg);

    ControlsUtils::eigen_vector_to_twist(power_disparity, power_disparity_msg);
    power_disparity_pub->publish(power_disparity_msg);

    power_disparity_norm_msg.data = power_disparity.norm();
    power_disparity_norm_pub->publish(power_disparity_norm_msg);

    ControlsUtils::map_to_control_types(control_types, control_types_msg);
    control_types_pub->publish(control_types_msg);

    status_msg.data = controls_enabled;
    status_pub->publish(status_msg);

    for (const PIDLoopTypesEnum &loop : PID_LOOP_TYPES)
        loops_axes_pid_gains[loop] = pid_managers.at(loop).get_axes_pid_gains();

    ControlsUtils::pid_loops_axes_gains_map_to_msg(loops_axes_pid_gains, pid_gains_msg);
    pid_gains_pub->publish(pid_gains_msg);

    static_power_global_msg = tf2::toMsg(static_power_global);
    static_power_global_pub->publish(static_power_global_msg);

    power_scale_factor_msg.data = power_scale_factor;
    power_scale_factor_pub->publish(power_scale_factor_msg);
}

int main(int argc, char **argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the controls node
    // In ROS2, the node creation, tf buffer, and listener are handled in the Controls constructor
    auto node = std::make_shared<Controls>();

    // Create the executor
    // MultiThreadedExecutor is recommended for nodes with multiple callbacks and timers
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add the node to the executor
    executor.add_node(node);

    // Spin the executor - this replaces the controls.run() call
    executor.spin();

    // Cleanup
    rclcpp::shutdown();

    return 0;
}
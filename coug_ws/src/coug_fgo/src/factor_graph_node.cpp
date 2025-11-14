/**
 * @file factor_graph_node.cpp
 * @brief A ROS 2 node for state estimation using GTSAM's fixed-lag smoother.
 * @author Nelson Durrant
 * @date Nov 2025
 *
 * This node fuses sensor data from an IMU, GPS, DVL, depth sensor, and heading sensor
 * to provide a global state estimate (pose, velocity, and IMU biases). It uses GTSAM's
 * IncrementalFixedLagSmoother to maintain and optimize a factor graph of the AUV's
 * recent history.
 *
 * Subscribers:
 * - imu/data (sensor_msgs/msg/Imu)
 * - odometry/gps (nav_msgs/msg/Odometry)
 * - odometry/depth (nav_msgs/msg/Odometry)
 * - heading/data (sensor_msgs/msg/Imu)
 * - dvl/data (geometry_msgs/msg/TwistWithCovarianceStamped)
 * - 'odom' -> 'base_link' transform
 * Publishers:
 * - odometry/global (nav_msgs/msg/Odometry)
 * - smoothed_path (nav_msgs/msg/Path)
 * - odometry/velocity (geometry_msgs/msg/TwistWithCovarianceStamped)
 * - odometry/imu_bias (geometry_msgs/msg/TwistWithCovarianceStamped)
 * - 'map' -> 'odom' transform
 */

#include "coug_fgo/factor_graph_node.hpp"

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/inference/Symbol.h>

using namespace coug_fgo::utils;

using gtsam::symbol_shorthand::B; // Bias (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Velocity (x,y,z)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

void FactorGraphNode::logLoadedParameters()
{
    RCLCPP_INFO(get_logger(), "[Node] Factor Graph Update Rate: %.2f Hz", factor_graph_update_rate_);
    RCLCPP_INFO(get_logger(), "[Node] Publish Global TF: %s", publish_global_tf_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[Node] Publish Smoothed Path: %s", publish_smoothed_path_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[Node] Publish Velocity: %s", publish_velocity_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[Node] Publish IMU Bias: %s", publish_imu_bias_ ? "true" : "false");

    RCLCPP_INFO(get_logger(), "[GTSAM] Smoother Lag: %.2f s", smoother_lag_);
    RCLCPP_INFO(get_logger(), "[GTSAM] Relinearize Threshold: %.2f", gtsam_relinearize_threshold_);
    RCLCPP_INFO(get_logger(), "[GTSAM] Relinearize Skip: %d", gtsam_relinearize_skip_);

    RCLCPP_INFO(get_logger(), "[ROS] IMU Topic: %s", imu_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] GPS Odom Topic: %s", gps_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] Depth Odom Topic: %s", depth_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] Heading Topic: %s", heading_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] DVL Topic: %s", dvl_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] Global Odom Topic: %s", global_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] Smoothed Path Topic: %s", smoothed_path_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] Velocity Topic: %s", velocity_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] IMU Bias Topic: %s", imu_bias_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] Map Frame: %s", map_frame_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] Odom Frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] Base Frame: %s", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] State Frame: %s", state_frame_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] IMU Queue Size: %d", imu_queue_size_);
    RCLCPP_INFO(get_logger(), "[ROS] GPS Queue Size: %d", gps_queue_size_);
    RCLCPP_INFO(get_logger(), "[ROS] Depth Queue Size: %d", depth_queue_size_);
    RCLCPP_INFO(get_logger(), "[ROS] Heading Queue Size: %d", heading_queue_size_);
    RCLCPP_INFO(get_logger(), "[ROS] DVL Queue Size: %d", dvl_queue_size_);

    RCLCPP_INFO(get_logger(), "[Sensors] Enable GPS: %s", enable_gps_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[Sensors] Enable Depth: %s", enable_depth_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[Sensors] Enable Heading: %s", enable_heading_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[Sensors] Enable DVL: %s", enable_dvl_ ? "true" : "false");

    RCLCPP_INFO(get_logger(), "[IMU] Use Param Covariance: %s", imu_use_parameter_covariance_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[IMU] Accel Noise Sigma: %e", accel_noise_sigma_);
    RCLCPP_INFO(get_logger(), "[IMU] Gyro Noise Sigma: %e", gyro_noise_sigma_);
    RCLCPP_INFO(get_logger(), "[IMU] Accel Bias RW Sigma: %e", accel_bias_rw_sigma_);
    RCLCPP_INFO(get_logger(), "[IMU] Gyro Bias RW Sigma: %e", gyro_bias_rw_sigma_);
    RCLCPP_INFO(get_logger(), "[IMU] Integration Covariance: %e", imu_integration_covariance_);
    RCLCPP_INFO(get_logger(), "[IMU] Gravity: [%.2f, %.2f, %.2f]", imu_gravity_[0], imu_gravity_[1], imu_gravity_[2]);

    RCLCPP_INFO(get_logger(), "[GPS] Use Param Covariance: %s", gps_use_parameter_covariance_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[GPS] Position Noise Sigma: %.3f", gps_position_noise_sigma_);

    RCLCPP_INFO(get_logger(), "[Depth] Use Param Covariance: %s", depth_use_parameter_covariance_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[Depth] Z-Position Noise Sigma: %.3f", depth_position_z_noise_sigma_);

    RCLCPP_INFO(get_logger(), "[Heading] Use Param Covariance: %s", heading_use_parameter_covariance_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[Heading] Yaw Noise Sigma: %.3f", heading_yaw_noise_sigma_);

    RCLCPP_INFO(get_logger(), "[DVL] Use Param Covariance: %s", dvl_use_parameter_covariance_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[DVL] Velocity Noise Sigma: %.3f", dvl_velocity_noise_sigma_);

    RCLCPP_INFO(get_logger(), "[Prior] Use Param Initial State: %s", prior_use_parameter_initial_state_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[Prior] Initial Position (x,y,z): [%.2f, %.2f, %.2f]", initial_state_pos_[0], initial_state_pos_[1], initial_state_pos_[2]);
    RCLCPP_INFO(get_logger(), "[Prior] Initial Yaw: %.2f", initial_state_yaw_);
    RCLCPP_INFO(get_logger(), "[Prior] Use Param Covariances: %s", prior_use_parameter_covariances_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "[Prior] Initial Position Sigma: %.3f", prior_initial_position_sigma_);
    RCLCPP_INFO(get_logger(), "[Prior] Initial Yaw Sigma: %.3f", prior_initial_yaw_sigma_);
    RCLCPP_INFO(get_logger(), "[Prior] Initial Roll/Pitch Sigma: %.3f", prior_initial_roll_pitch_sigma_);
    RCLCPP_INFO(get_logger(), "[Prior] Initial Velocity Sigma: %.3f", prior_initial_velocity_sigma_);
    RCLCPP_INFO(get_logger(), "[Prior] Initial Bias Sigma: %e", prior_initial_bias_sigma_);
}

void FactorGraphNode::loadParameters()
{
    // --- Node Settings ---
    factor_graph_update_rate_ = declare_parameter<double>("factor_graph_update_rate", 10.0);
    publish_global_tf_ = declare_parameter<bool>("publish_global_tf", true);
    publish_smoothed_path_ = declare_parameter<bool>("publish_smoothed_path", true);
    publish_velocity_ = declare_parameter<bool>("publish_velocity", false);
    publish_imu_bias_ = declare_parameter<bool>("publish_imu_bias", false);

    // --- GTSAM Settings ---
    smoother_lag_ = declare_parameter<double>("smoother_lag", 30.0);
    gtsam_relinearize_threshold_ = declare_parameter<double>("relinearize_threshold", 0.1);
    gtsam_relinearize_skip_ = declare_parameter<int>("relinearize_skip", 1);

    // --- ROS Topics, Frames, and Queues ---
    imu_topic_ = declare_parameter<std::string>("imu_topic", "imu/data");
    gps_odom_topic_ = declare_parameter<std::string>("gps_odom_topic", "odometry/gps");
    depth_odom_topic_ = declare_parameter<std::string>("depth_odom_topic", "odometry/depth");
    heading_topic_ = declare_parameter<std::string>("heading_topic", "imu/heading");
    dvl_topic_ = declare_parameter<std::string>("dvl_topic", "dvl/data");
    global_odom_topic_ = declare_parameter<std::string>("global_odom_topic", "odometry/global");
    smoothed_path_topic_ = declare_parameter<std::string>("smoothed_path_topic", "smoothed_path");
    velocity_topic_ = declare_parameter<std::string>("velocity_topic", "~/velocity");
    imu_bias_topic_ = declare_parameter<std::string>("imu_bias_topic", "~/imu_bias");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    state_frame_ = declare_parameter<std::string>("state_frame", "dvl_link");
    imu_queue_size_ = declare_parameter<int>("subscriber_queue_sizes.imu", 200);
    gps_queue_size_ = declare_parameter<int>("subscriber_queue_sizes.gps", 20);
    depth_queue_size_ = declare_parameter<int>("subscriber_queue_sizes.depth", 20);
    heading_queue_size_ = declare_parameter<int>("subscriber_queue_sizes.heading", 20);
    dvl_queue_size_ = declare_parameter<int>("subscriber_queue_sizes.dvl", 20);

    // --- Sensor Settings ---
    enable_gps_ = declare_parameter<bool>("gps.enable_gps", true);
    enable_depth_ = declare_parameter<bool>("depth.enable_depth", true);
    enable_heading_ = declare_parameter<bool>("heading.enable_heading", true);
    enable_dvl_ = declare_parameter<bool>("dvl.enable_dvl", true);

    imu_use_parameter_covariance_ = declare_parameter<bool>("imu.use_parameter_covariance", false);
    accel_noise_sigma_ = declare_parameter<double>("imu.accel_noise_sigma", 0.1);
    gyro_noise_sigma_ = declare_parameter<double>("imu.gyro_noise_sigma", 0.01);
    accel_bias_rw_sigma_ = declare_parameter<double>("imu.accel_bias_rw_sigma", 1.0e-4);
    gyro_bias_rw_sigma_ = declare_parameter<double>("imu.gyro_bias_rw_sigma", 1.0e-5);
    imu_integration_covariance_ = declare_parameter<double>("imu.integration_covariance", 1.0e-8);
    imu_gravity_ = declare_parameter<std::vector<double>>("imu.gravity", {0.0, 0.0, -9.81});

    gps_use_parameter_covariance_ = declare_parameter<bool>("gps.use_parameter_covariance", false);
    gps_position_noise_sigma_ = declare_parameter<double>("gps.position_noise_sigma", 0.5);

    depth_use_parameter_covariance_ = declare_parameter<bool>("depth.use_parameter_covariance", false);
    depth_position_z_noise_sigma_ = declare_parameter<double>("depth.position_z_noise_sigma", 0.1);

    heading_use_parameter_covariance_ = declare_parameter<bool>("heading.use_parameter_covariance", false);
    heading_yaw_noise_sigma_ = declare_parameter<double>("heading.yaw_noise_sigma", 0.05);

    dvl_use_parameter_covariance_ = declare_parameter<bool>("dvl.use_parameter_covariance", false);
    dvl_velocity_noise_sigma_ = declare_parameter<double>("dvl.velocity_noise_sigma", 0.1);

    // --- Initial State Priors ---
    prior_use_parameter_initial_state_ = declare_parameter<bool>("prior.use_parameter_initial_state", false);
    initial_state_pos_ = declare_parameter<std::vector<double>>("prior.initial_position", {0.0, 0.0, 0.0});
    initial_state_yaw_ = declare_parameter<double>("prior.initial_yaw", 0.0);
    prior_use_parameter_covariances_ = declare_parameter<bool>("prior.use_parameter_covariances", false);
    prior_initial_position_sigma_ = declare_parameter<double>("prior.initial_position_sigma", 0.5);
    prior_initial_yaw_sigma_ = declare_parameter<double>("prior.initial_yaw_sigma", 0.05);
    prior_initial_roll_pitch_sigma_ = declare_parameter<double>("prior.initial_roll_pitch_sigma", 0.1);
    prior_initial_velocity_sigma_ = declare_parameter<double>("prior.initial_velocity_sigma", 0.1);
    prior_initial_bias_sigma_ = declare_parameter<double>("prior.initial_bias_sigma", 0.02);

    logLoadedParameters();
}

void FactorGraphNode::setupRosInterfaces()
{
    // --- ROS TF Interfaces ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // --- ROS Publishers ---
    global_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(global_odom_topic_, 10);
    smoothed_path_pub_ = create_publisher<nav_msgs::msg::Path>(smoothed_path_topic_, 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(velocity_topic_, 10);
    imu_bias_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(imu_bias_topic_, 10);

    // --- ROS Callback Groups ---
    sensor_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    graph_timer_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = sensor_cb_group_;

    // --- ROS Subscribers ---
    auto imu_cb = [this](const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::scoped_lock lock(imu_queue_mutex_);
        imu_queue_.push_back(msg);
    };
    auto gps_cb = [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::scoped_lock lock(gps_queue_mutex_);
        gps_queue_.push_back(msg);
    };
    auto depth_cb = [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::scoped_lock lock(depth_queue_mutex_);
        depth_odom_queue_.push_back(msg);
    };
    auto heading_cb = [this](const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::scoped_lock lock(heading_queue_mutex_);
        heading_queue_.push_back(msg);
    };
    auto dvl_cb = [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        std::scoped_lock lock(dvl_queue_mutex_);
        dvl_queue_.push_back(msg);
    };

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_queue_size_, imu_cb, sub_options);
    gps_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        gps_odom_topic_, gps_queue_size_, gps_cb, sub_options);
    depth_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        depth_odom_topic_, depth_queue_size_, depth_cb, sub_options);
    heading_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        heading_topic_, heading_queue_size_, heading_cb, sub_options);
    dvl_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        dvl_topic_, dvl_queue_size_, dvl_cb, sub_options);

    // --- ROS Timers ---
    factor_graph_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / factor_graph_update_rate_)),
        [this]()
        { factorGraphTimerCallback(); },
        graph_timer_cb_group_);
}

FactorGraphNode::FactorGraphNode() : Node("factor_graph_node")
{
    RCLCPP_INFO(get_logger(), "Starting GTSAM State Estimator Node...");

    loadParameters();
    setupRosInterfaces();

    RCLCPP_INFO(get_logger(), "Startup complete! Waiting for sensor messages...");
}

bool FactorGraphNode::lookupInitialTransforms(const sensor_msgs::msg::Imu::SharedPtr &initial_imu,
                                              const nav_msgs::msg::Odometry::SharedPtr &initial_gps,
                                              const nav_msgs::msg::Odometry::SharedPtr &initial_depth,
                                              const sensor_msgs::msg::Imu::SharedPtr &initial_heading,
                                              const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr &initial_dvl)
{
    try
    {
        if (!have_state_to_base_tf_)
        {
            RCLCPP_INFO(get_logger(), "Looking up '%s' to '%s' transform...",
                        base_frame_.c_str(), state_frame_.c_str());
            state_to_base_tf_ = tf_buffer_->lookupTransform(
                base_frame_, state_frame_, tf2::TimePointZero, tf2::durationFromSec(2.0));
            have_state_to_base_tf_ = true;
        }

        if (!have_imu_to_state_tf_)
        {
            RCLCPP_INFO(get_logger(), "Looking up IMU static transform from '%s' to '%s'...",
                        initial_imu->header.frame_id.c_str(), state_frame_.c_str());
            imu_to_state_tf_ = tf_buffer_->lookupTransform(
                state_frame_, initial_imu->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(2.0));
            have_imu_to_state_tf_ = true;
        }

        if (enable_gps_ && !have_gps_to_state_tf_)
        {
            RCLCPP_INFO(get_logger(), "Looking up GPS static transform from '%s' to '%s'...",
                        initial_gps->child_frame_id.c_str(), state_frame_.c_str());
            gps_to_state_tf_ = tf_buffer_->lookupTransform(
                state_frame_, initial_gps->child_frame_id, tf2::TimePointZero, tf2::durationFromSec(2.0));
            have_gps_to_state_tf_ = true;
        }

        if (enable_depth_ && !have_depth_to_state_tf_)
        {
            RCLCPP_INFO(get_logger(), "Looking up depth sensor static transform from '%s' to '%s'...",
                        initial_depth->child_frame_id.c_str(), state_frame_.c_str());
            depth_to_state_tf_ = tf_buffer_->lookupTransform(
                state_frame_, initial_depth->child_frame_id, tf2::TimePointZero, tf2::durationFromSec(2.0));
            have_depth_to_state_tf_ = true;
        }

        if (enable_heading_ && !have_heading_to_state_tf_)
        {
            RCLCPP_INFO(get_logger(), "Looking up heading sensor static transform from '%s' to '%s'...",
                        initial_heading->header.frame_id.c_str(), state_frame_.c_str());
            heading_to_state_tf_ = tf_buffer_->lookupTransform(
                state_frame_, initial_heading->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(2.0));
            have_heading_to_state_tf_ = true;
        }

        if (enable_dvl_ && !have_dvl_to_state_tf_)
        {
            RCLCPP_INFO(get_logger(), "Looking up DVL static transform from '%s' to '%s'...",
                        initial_dvl->header.frame_id.c_str(), state_frame_.c_str());
            dvl_to_state_tf_ = tf_buffer_->lookupTransform(
                state_frame_, initial_dvl->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(2.0));
            have_dvl_to_state_tf_ = true;
        }
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Could not get required transforms: %s. Aborting initialization.",
                              ex.what());
        return false;
    }
    return true;
}

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
FactorGraphNode::configureImuPreintegration(const sensor_msgs::msg::Imu::SharedPtr &initial_imu)
{
    auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
    imu_params->n_gravity = gtsam::Vector3(imu_gravity_[0], imu_gravity_[1], imu_gravity_[2]);
    imu_params->body_P_sensor = toGtsam(imu_to_state_tf_.transform);

    bool use_param_accel_cov = imu_use_parameter_covariance_ || initial_imu->linear_acceleration_covariance[0] <= 0 ||
                               initial_imu->linear_acceleration_covariance[4] <= 0 || initial_imu->linear_acceleration_covariance[8] <= 0;
    if (use_param_accel_cov)
    {
        if (!imu_use_parameter_covariance_)
        {
            RCLCPP_WARN(get_logger(), "Invalid IMU linear acceleration covariance. Using 'imu.accel_noise_sigma' parameter.");
        }
        imu_params->accelerometerCovariance = gtsam::Matrix33::Identity() * pow(accel_noise_sigma_, 2);
    }
    else
    {
        imu_params->accelerometerCovariance = gtsam::Matrix33(initial_imu->linear_acceleration_covariance.data());
    }

    bool use_param_gyro_cov = imu_use_parameter_covariance_ || initial_imu->angular_velocity_covariance[0] <= 0 ||
                              initial_imu->angular_velocity_covariance[4] <= 0 || initial_imu->angular_velocity_covariance[8] <= 0;
    if (use_param_gyro_cov)
    {
        if (!imu_use_parameter_covariance_)
        {
            RCLCPP_WARN(get_logger(), "Invalid IMU angular velocity covariance. Using 'imu.gyro_noise_sigma' parameter.");
        }
        imu_params->gyroscopeCovariance = gtsam::Matrix33::Identity() * pow(gyro_noise_sigma_, 2);
    }
    else
    {
        imu_params->gyroscopeCovariance = gtsam::Matrix33(initial_imu->angular_velocity_covariance.data());
    }

    imu_params->biasAccCovariance = gtsam::Matrix33::Identity() * pow(accel_bias_rw_sigma_, 2);
    imu_params->biasOmegaCovariance = gtsam::Matrix33::Identity() * pow(gyro_bias_rw_sigma_, 2);
    imu_params->integrationCovariance = gtsam::Matrix33::Identity() * imu_integration_covariance_;

    return imu_params;
}

gtsam::Rot3 FactorGraphNode::computeInitialOrientation(const sensor_msgs::msg::Imu::SharedPtr &initial_heading)
{
    double yaw = initial_state_yaw_;
    if (enable_heading_ && !prior_use_parameter_initial_state_)
    {
        gtsam::Rot3 R_base_sensor = toGtsam(heading_to_state_tf_.transform.rotation);
        gtsam::Rot3 heading_rot_body = toGtsam(initial_heading->orientation) * R_base_sensor.inverse();
        yaw = heading_rot_body.yaw();
    }
    return gtsam::Rot3::Ypr(yaw, 0, 0);
}

gtsam::Point3 FactorGraphNode::computeInitialPosition(const nav_msgs::msg::Odometry::SharedPtr &initial_gps,
                                                      const nav_msgs::msg::Odometry::SharedPtr &initial_depth,
                                                      const gtsam::Rot3 &initial_orientation)
{
    gtsam::Point3 initial_position(initial_state_pos_[0], initial_state_pos_[1], initial_state_pos_[2]);
    if (!prior_use_parameter_initial_state_)
    {
        if (enable_gps_)
        {
            // Calculate initial XY position accounting for sensor offset
            gtsam::Pose3 T_state_gps = toGtsam(gps_to_state_tf_.transform);
            gtsam::Point3 p_base_sensor = T_state_gps.translation();
            gtsam::Point3 world_t_sensor_offset = initial_orientation.rotate(p_base_sensor);
            gtsam::Point3 measured_pos = toGtsam(initial_gps->pose.pose.position);
            initial_position = measured_pos - world_t_sensor_offset;
        }
        if (enable_depth_)
        {
            // Calculate initial Z position accounting for sensor offset
            gtsam::Pose3 T_state_depth = toGtsam(depth_to_state_tf_.transform);
            gtsam::Point3 p_base_sensor = T_state_depth.translation();
            gtsam::Point3 world_t_sensor_offset = initial_orientation.rotate(p_base_sensor);
            initial_position.z() = initial_depth->pose.pose.position.z - world_t_sensor_offset.z();
        }
    }
    return initial_position;
}

void FactorGraphNode::addPriorFactors(gtsam::NonlinearFactorGraph &graph, gtsam::Values &values,
                                      const nav_msgs::msg::Odometry::SharedPtr &initial_gps,
                                      const nav_msgs::msg::Odometry::SharedPtr &initial_depth,
                                      const sensor_msgs::msg::Imu::SharedPtr &initial_heading)
{
    gtsam::Vector6 pose_sigmas;
    pose_sigmas << prior_initial_roll_pitch_sigma_, prior_initial_roll_pitch_sigma_, prior_initial_yaw_sigma_, prior_initial_position_sigma_, prior_initial_position_sigma_, prior_initial_position_sigma_;

    if (!prior_use_parameter_covariances_)
    {
        if (enable_gps_)
        {
            if (gps_use_parameter_covariance_ || initial_gps->pose.covariance[0] <= 0 || initial_gps->pose.covariance[7] <= 0)
            {
                if (initial_gps->pose.covariance[0] <= 0 || initial_gps->pose.covariance[7] <= 0)
                {
                    RCLCPP_WARN(get_logger(), "Invalid GPS X/Y covariance. Using 'gps.position_noise_sigma' parameter.");
                }
                pose_sigmas(3) = gps_position_noise_sigma_;
                pose_sigmas(4) = gps_position_noise_sigma_;
            }
            else
            {
                pose_sigmas(3) = std::sqrt(initial_gps->pose.covariance[0]);
                pose_sigmas(4) = std::sqrt(initial_gps->pose.covariance[7]);
            }
        }

        if (enable_depth_)
        {
            if (depth_use_parameter_covariance_ || initial_depth->pose.covariance[14] <= 0)
            {
                if (initial_depth->pose.covariance[14] <= 0)
                {
                    RCLCPP_WARN(get_logger(), "Invalid depth sensor Z covariance. Using 'depth.position_z_noise_sigma' parameter.");
                }
                pose_sigmas(5) = depth_position_z_noise_sigma_;
            }
            else
            {
                pose_sigmas(5) = std::sqrt(initial_depth->pose.covariance[14]);
            }
        }

        if (enable_heading_)
        {
            if (heading_use_parameter_covariance_ || initial_heading->orientation_covariance[8] <= 0)
            {
                if (initial_heading->orientation_covariance[8] <= 0)
                {
                    RCLCPP_WARN(get_logger(), "Invalid heading sensor yaw covariance. Using 'heading.yaw_noise_sigma' parameter.");
                }
                pose_sigmas(2) = heading_yaw_noise_sigma_;
            }
            else
            {
                pose_sigmas(2) = std::sqrt(initial_heading->orientation_covariance[8]);
            }
        }
    }

    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas);
    auto vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, prior_initial_velocity_sigma_);
    auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, prior_initial_bias_sigma_);

    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), prev_pose_, pose_noise);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), prev_vel_, vel_noise);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(0), prev_bias_, bias_noise);

    values.insert(X(0), prev_pose_);
    values.insert(V(0), prev_vel_);
    values.insert(B(0), prev_bias_);
}

void FactorGraphNode::initializeSystem(const sensor_msgs::msg::Imu::SharedPtr &initial_imu,
                                       const nav_msgs::msg::Odometry::SharedPtr &initial_gps,
                                       const nav_msgs::msg::Odometry::SharedPtr &initial_depth,
                                       const sensor_msgs::msg::Imu::SharedPtr &initial_heading,
                                       const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr &initial_dvl)
{
    RCLCPP_INFO(get_logger(), "Required sensor data received. Initializing system...");

    // --- Look Up Initial Transforms ---
    if (!lookupInitialTransforms(initial_imu, initial_gps, initial_depth, initial_heading, initial_dvl))
    {
        return;
    }

    // --- Configure IMU Preintegration ---
    auto imu_params = configureImuPreintegration(initial_imu);

    // --- Add Prior Factors ---
    gtsam::Rot3 initial_orientation = computeInitialOrientation(initial_heading);
    gtsam::Point3 initial_position = computeInitialPosition(initial_gps, initial_depth, initial_orientation);

    rclcpp::Time initial_stamp = initial_imu->header.stamp;
    prev_time_ = initial_stamp.seconds();
    prev_pose_ = gtsam::Pose3(initial_orientation, initial_position);
    prev_vel_ = gtsam::Vector3(0, 0, 0);         // Assume starting from rest
    prev_bias_ = gtsam::imuBias::ConstantBias(); // Assume zero initial bias
    imu_preintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(imu_params, prev_bias_);

    gtsam::NonlinearFactorGraph initial_graph;
    gtsam::Values initial_values;

    addPriorFactors(initial_graph, initial_values, initial_gps, initial_depth, initial_heading);

    gtsam::IncrementalFixedLagSmoother::KeyTimestampMap initial_timestamps;
    initial_timestamps[X(0)] = prev_time_;
    initial_timestamps[V(0)] = prev_time_;
    initial_timestamps[B(0)] = prev_time_;

    key_to_time_[X(0)] = initial_stamp;
    time_to_key_[initial_stamp] = X(0);

    // --- Initialize Incremental Fixed-Lag Smoother ---
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = gtsam_relinearize_threshold_;
    isam2_params.relinearizeSkip = gtsam_relinearize_skip_;
    smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(smoother_lag_, isam2_params);
    smoother_->update(initial_graph, initial_values, initial_timestamps);

    system_initialized_ = true;
    RCLCPP_INFO(get_logger(), "System initialized successfully!");
}

std::pair<KeyLookupResultType, unsigned long> FactorGraphNode::getClosestNodeKey(const rclcpp::Time &msg_stamp)
{
    if (time_to_key_.empty())
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "No keys in the factor graph yet. Discarding measurement (stamp: %.2f).",
                             msg_stamp.seconds());
        return {KeyLookupResultType::TOO_OLD, 0};
    }

    auto it_last = time_to_key_.rbegin();
    const rclcpp::Time prune_time = it_last->first - rclcpp::Duration::from_seconds(smoother_lag_);
    if (msg_stamp < prune_time)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Discarding measurement (stamp: %.2f) that is older than the smoother lag (lag time: %.2f).",
                             msg_stamp.seconds(), prune_time.seconds());
        return {KeyLookupResultType::TOO_OLD, 0};
    }
    if (msg_stamp > it_last->first)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Measurement (stamp: %.2f) is newer than the last key (%.2f). Marking as future.",
                             msg_stamp.seconds(), it_last->first.seconds());
        return {KeyLookupResultType::IN_FUTURE, 0};
    }

    // O(log K)
    auto it_after = time_to_key_.lower_bound(msg_stamp);
    if (it_after == time_to_key_.begin())
    {
        return {KeyLookupResultType::VALID, gtsam::Symbol(it_after->second).index()};
    }
    auto it_before = std::prev(it_after);
    rclcpp::Duration dt_before = msg_stamp - it_before->first;
    rclcpp::Duration dt_after = it_after->first - msg_stamp;
    if (dt_before <= dt_after)
    {
        return {KeyLookupResultType::VALID, gtsam::Symbol(it_before->second).index()};
    }
    else
    {
        return {KeyLookupResultType::VALID, gtsam::Symbol(it_after->second).index()};
    }
}

void FactorGraphNode::addGpsFactors(gtsam::NonlinearFactorGraph &graph,
                                    const std::deque<nav_msgs::msg::Odometry::SharedPtr> &gps_msgs)
{
    if (!enable_gps_ || !have_gps_to_state_tf_)
        return;

    // O(M_gps * log K)
    for (const auto &msg : gps_msgs)
    {
        if (!std::isfinite(msg->pose.pose.position.x) || !std::isfinite(msg->pose.pose.position.y) || !std::isfinite(msg->pose.pose.position.z))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "GPS message contains NaN or Inf in position. Skipping measurement.");
            continue;
        }

        // O(log K)
        auto key_lookup = getClosestNodeKey(msg->header.stamp);
        switch (key_lookup.first)
        {
        case KeyLookupResultType::TOO_OLD:
            continue; // Discard
        case KeyLookupResultType::IN_FUTURE:
        {
            std::scoped_lock lock(gps_queue_mutex_);
            gps_queue_.push_front(msg); // Re-queue
        }
            continue;
        case KeyLookupResultType::VALID:
            break; // Process
        }
        unsigned long key = key_lookup.second;
        gtsam::Point3 gps_position = toGtsam(msg->pose.pose.position);

        gtsam::Pose3 T_state_gps = toGtsam(gps_to_state_tf_.transform);

        gtsam::SharedNoiseModel gps_noise;
        bool use_param_cov = gps_use_parameter_covariance_ || msg->pose.covariance[0] <= 0 || msg->pose.covariance[7] <= 0 || msg->pose.covariance[14] <= 0;

        if (use_param_cov)
        {
            if (!gps_use_parameter_covariance_)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Invalid GPS covariance in message. Using 'gps.position_noise_sigma' parameter.");
            }
            gps_noise = gtsam::noiseModel::Isotropic::Sigma(3, gps_position_noise_sigma_);
        }
        else
        {
            gtsam::Vector3 sigmas(sqrt(msg->pose.covariance[0]), sqrt(msg->pose.covariance[7]), sqrt(msg->pose.covariance[14]));
            gps_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
        }
        graph.emplace_shared<CustomGPSFactorArm>(X(key), gps_position, T_state_gps, gps_noise);
    }
}

void FactorGraphNode::addDepthFactors(gtsam::NonlinearFactorGraph &graph,
                                      const std::deque<nav_msgs::msg::Odometry::SharedPtr> &depth_msgs)
{
    if (!enable_depth_ || !have_depth_to_state_tf_)
        return;

    // O(M_depth * log K)
    for (const auto &msg : depth_msgs)
    {
        if (!std::isfinite(msg->pose.pose.position.z))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "Depth message contains NaN or Inf. Skipping measurement.");
            continue;
        }

        // O(log K)
        auto key_lookup = getClosestNodeKey(msg->header.stamp);
        switch (key_lookup.first)
        {
        case KeyLookupResultType::TOO_OLD:
            continue; // Discard
        case KeyLookupResultType::IN_FUTURE:
        {
            std::scoped_lock lock(depth_queue_mutex_);
            depth_odom_queue_.push_front(msg); // Re-queue
        }
            continue;
        case KeyLookupResultType::VALID:
            break; // Process
        }
        unsigned long key = key_lookup.second;
        double depth_z = msg->pose.pose.position.z;

        gtsam::Pose3 T_state_depth = toGtsam(depth_to_state_tf_.transform);

        gtsam::SharedNoiseModel depth_noise;
        bool use_param_cov = depth_use_parameter_covariance_ || msg->pose.covariance[14] <= 0;

        if (use_param_cov)
        {
            if (!depth_use_parameter_covariance_)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Invalid depth covariance in message. Using 'depth.position_z_noise_sigma' parameter.");
            }
            depth_noise = gtsam::noiseModel::Isotropic::Sigma(1, depth_position_z_noise_sigma_);
        }
        else
        {
            depth_noise = gtsam::noiseModel::Isotropic::Sigma(1, sqrt(msg->pose.covariance[14]));
        }
        graph.emplace_shared<CustomDepthFactorArm>(X(key), depth_z, T_state_depth, depth_noise);
    }
}

void FactorGraphNode::addHeadingFactors(gtsam::NonlinearFactorGraph &graph,
                                        const std::deque<sensor_msgs::msg::Imu::SharedPtr> &heading_msgs)
{
    if (!enable_heading_ || !have_heading_to_state_tf_)
        return;

    // O(M_heading * log K)
    for (const auto &msg : heading_msgs)
    {
        if (!std::isfinite(msg->orientation.x) || !std::isfinite(msg->orientation.y) ||
            !std::isfinite(msg->orientation.z) || !std::isfinite(msg->orientation.w))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "Heading message contains NaN or Inf in orientation. Skipping measurement.");
            continue;
        }

        // O(log K)
        auto key_lookup = getClosestNodeKey(msg->header.stamp);
        switch (key_lookup.first)
        {
        case KeyLookupResultType::TOO_OLD:
            continue; // Discard
        case KeyLookupResultType::IN_FUTURE:
        {
            std::scoped_lock lock(heading_queue_mutex_);
            heading_queue_.push_front(msg); // Re-queue
        }
            continue;
        case KeyLookupResultType::VALID:
            break; // Process
        }
        unsigned long key = key_lookup.second;

        gtsam::Rot3 heading_rot_sensor = toGtsam(msg->orientation);
        gtsam::Rot3 R_base_sensor = toGtsam(heading_to_state_tf_.transform.rotation);

        gtsam::SharedNoiseModel heading_noise;
        bool use_param_cov = heading_use_parameter_covariance_ || msg->orientation_covariance[0] <= 0 || msg->orientation_covariance[4] <= 0 || msg->orientation_covariance[8] <= 0;

        if (use_param_cov)
        {
            if (!heading_use_parameter_covariance_)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Invalid heading covariance in message. Using 'heading' parameter values.");
            }
            gtsam::Vector3 sigmas(prior_initial_roll_pitch_sigma_, prior_initial_roll_pitch_sigma_, heading_yaw_noise_sigma_);
            heading_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
        }
        else
        {
            gtsam::Vector3 sigmas(prior_initial_roll_pitch_sigma_, prior_initial_roll_pitch_sigma_, sqrt(msg->orientation_covariance[8]));
            heading_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
        }
        graph.emplace_shared<CustomHeadingFactorArm>(X(key), heading_rot_sensor, R_base_sensor, heading_noise);
    }
}

gtsam::Vector3 FactorGraphNode::getInterpolatedGyro(const rclcpp::Time &msg_stamp,
                                                    const std::deque<sensor_msgs::msg::Imu::SharedPtr> &imu_msgs)
{
    double target_time = msg_stamp.seconds();

    auto compare_stamp = [](const sensor_msgs::msg::Imu::SharedPtr &imu_msg, double time)
    {
        return rclcpp::Time(imu_msg->header.stamp).seconds() < time;
    };
    // O(log H)
    auto it_after = std::lower_bound(imu_msgs.begin(), imu_msgs.end(), target_time, compare_stamp);
    if (it_after == imu_msgs.begin())
    {
        // Target time is at or before the first IMU message.
        return toGtsam(imu_msgs.front()->angular_velocity);
    }
    if (it_after == imu_msgs.end())
    {
        // Target time is after the last IMU message.
        return toGtsam(imu_msgs.back()->angular_velocity);
    }

    sensor_msgs::msg::Imu::SharedPtr msg_after = *it_after;
    sensor_msgs::msg::Imu::SharedPtr msg_before = *(it_after - 1);
    double t1 = rclcpp::Time(msg_before->header.stamp).seconds();
    double t2 = rclcpp::Time(msg_after->header.stamp).seconds();
    double dt = t2 - t1;

    if (dt < 1e-9)
    {
        return toGtsam(msg_before->angular_velocity);
    }

    gtsam::Vector3 gyro1 = toGtsam(msg_before->angular_velocity);
    gtsam::Vector3 gyro2 = toGtsam(msg_after->angular_velocity);
    double alpha = (target_time - t1) / dt;
    gtsam::Vector3 interpolated_gyro = (1.0 - alpha) * gyro1 + alpha * gyro2;

    return interpolated_gyro;
}

void FactorGraphNode::addDvlFactors(gtsam::NonlinearFactorGraph &graph,
                                    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> &dvl_msgs,
                                    const std::deque<sensor_msgs::msg::Imu::SharedPtr> &imu_msgs)
{
    if (!enable_dvl_ || !have_dvl_to_state_tf_)
        return;

    // O(M_dvl * (log K + log H))
    for (const auto &msg : dvl_msgs)
    {
        if (!std::isfinite(msg->twist.twist.linear.x) || !std::isfinite(msg->twist.twist.linear.y) || !std::isfinite(msg->twist.twist.linear.z))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "DVL message contains NaN or Inf in velocity. Skipping measurement.");
            continue;
        }

        // O(log K)
        auto key_lookup = getClosestNodeKey(msg->header.stamp);
        switch (key_lookup.first)
        {
        case KeyLookupResultType::TOO_OLD:
            continue; // Discard
        case KeyLookupResultType::IN_FUTURE:
        {
            std::scoped_lock lock(dvl_queue_mutex_);
            dvl_queue_.push_front(msg); // Re-queue
        }
            continue;
        case KeyLookupResultType::VALID:
            break; // Process
        }
        unsigned long key = key_lookup.second;

        gtsam::SharedNoiseModel dvl_noise;
        bool use_param_cov = dvl_use_parameter_covariance_ || msg->twist.covariance[0] <= 0 || msg->twist.covariance[7] <= 0 || msg->twist.covariance[14] <= 0;

        if (use_param_cov)
        {
            if (!dvl_use_parameter_covariance_)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Invalid DVL covariance in message. Using 'dvl.velocity_noise_sigma' parameter.");
            }
            dvl_noise = gtsam::noiseModel::Isotropic::Sigma(3, dvl_velocity_noise_sigma_);
        }
        else
        {
            gtsam::Vector3 sigmas(sqrt(msg->twist.covariance[0]), sqrt(msg->twist.covariance[7]), sqrt(msg->twist.covariance[14]));
            dvl_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
        }

        // Can we can skip linear interpolation with clever state frame positioning?
        gtsam::Vector3 dvl_vel_sensor = toGtsam(msg->twist.twist.linear);
        if (state_frame_ == msg->header.frame_id)
        {
            graph.emplace_shared<CustomDVLFactor>(X(key), V(key), dvl_vel_sensor, dvl_noise);
        }
        else
        {
            // O(log H)
            gtsam::Vector3 measured_gyro = getInterpolatedGyro(msg->header.stamp, imu_msgs);
            gtsam::Pose3 T_state_dvl = toGtsam(dvl_to_state_tf_.transform);
            graph.emplace_shared<CustomDVLFactorArm>(X(key), V(key), B(key), dvl_vel_sensor, measured_gyro, T_state_dvl, dvl_noise);
        }
    }
}

void FactorGraphNode::publishGlobalOdom(const gtsam::Pose3 &current_pose,
                                        const gtsam::Vector3 &current_vel,
                                        const gtsam::Matrix &pose_covariance,
                                        const gtsam::Matrix &vel_covariance)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose = toPoseMsg(current_pose);
    odom_msg.twist.twist.linear = toVectorMsg(current_vel);

    /**
     * GTSAM Covariance Matrix Layout:       ROS Odometry Covariance Layout:
     * [ R_3x3 | R_p_3x3 ]                   [ P_3x3 | P_r_3x3 ]
     * [ p_R_3x3 | P_3x3 ]                   [ r_P_3x3 | R_3x3 ]
     *
     * where R is rotational covariance and P is positional covariance.
     */
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            odom_msg.pose.covariance[i * 6 + j] = pose_covariance(i + 3, j + 3);
            odom_msg.pose.covariance[(i + 3) * 6 + (j + 3)] = pose_covariance(i, j);
            odom_msg.pose.covariance[i * 6 + (j + 3)] = pose_covariance(i + 3, j);
            odom_msg.pose.covariance[(i + 3) * 6 + j] = pose_covariance(i, j + 3);
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            odom_msg.twist.covariance[i * 6 + j] = vel_covariance(i, j);
        }
    }

    global_odom_pub_->publish(odom_msg);
}

void FactorGraphNode::publishVelocity(const gtsam::Vector3 &current_vel_in_map,
                                      const gtsam::Matrix &vel_covariance)
{
    geometry_msgs::msg::TwistWithCovarianceStamped vel_msg;
    vel_msg.header.stamp = get_clock()->now();
    // Velocity of the 'state_frame_' expressed in 'map_frame_'.
    vel_msg.header.frame_id = map_frame_;
    vel_msg.twist.twist.linear = toVectorMsg(current_vel_in_map);

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            vel_msg.twist.covariance[i * 6 + j] = vel_covariance(i, j);
        }
    }
    for (int i = 3; i < 6; ++i)
    {
        for (int j = 3; j < 6; ++j)
        {
            vel_msg.twist.covariance[i * 6 + j] = (i == j) ? 1e-9 : 0.0;
        }
    }
    velocity_pub_->publish(vel_msg);
}

void FactorGraphNode::publishImuBias(const gtsam::imuBias::ConstantBias &current_bias,
                                     const gtsam::Matrix &bias_covariance)
{
    geometry_msgs::msg::TwistWithCovarianceStamped bias_msg;
    bias_msg.header.stamp = get_clock()->now();
    bias_msg.header.frame_id = state_frame_;

    // Use 'linear' for accelerometer bias and 'angular' for gyroscope bias
    bias_msg.twist.twist.linear = toVectorMsg(current_bias.accelerometer());
    bias_msg.twist.twist.angular = toVectorMsg(current_bias.gyroscope());

    // GTSAM bias covariance is [ accel_3x3 | ... ]
    //                          [ ...       | gyro_3x3  ]
    // ROS twist covariance is  [ linear_3x3 | ... ]
    //                          [ ...        | angular_3x3 ]
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            bias_msg.twist.covariance[i * 6 + j] = bias_covariance(i, j);
        }
    }

    imu_bias_pub_->publish(bias_msg);
}

void FactorGraphNode::broadcastGlobalTf(const gtsam::Pose3 &current_pose)
{
    try
    {
        geometry_msgs::msg::TransformStamped odom_to_state_tf_msg =
            tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);

        gtsam::Pose3 odom_to_base_gtsam = toGtsam(odom_to_state_tf_msg.transform);
        gtsam::Pose3 map_to_odom_gtsam = current_pose * odom_to_base_gtsam.inverse();

        geometry_msgs::msg::TransformStamped map_to_odom_tf_msg;
        map_to_odom_tf_msg.header.stamp = this->get_clock()->now();
        map_to_odom_tf_msg.header.frame_id = map_frame_;
        map_to_odom_tf_msg.child_frame_id = odom_frame_;
        map_to_odom_tf_msg.transform.translation = toVectorMsg(map_to_odom_gtsam.translation());
        map_to_odom_tf_msg.transform.rotation = toQuatMsg(map_to_odom_gtsam.rotation());
        tf_broadcaster_->sendTransform(map_to_odom_tf_msg);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Could not lookup transform from '%s' to '%s': %s. Skipping global TF broadcast.",
                              odom_frame_.c_str(), base_frame_.c_str(), ex.what());
    }
}

void FactorGraphNode::publishSmoothedPath(const gtsam::Values &results)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = get_clock()->now();
    path_msg.header.frame_id = map_frame_;

    gtsam::Pose3 T_base_state = toGtsam(state_to_base_tf_.transform);
    gtsam::Pose3 T_state_base = T_base_state.inverse();

    // O(K * log K)
    for (const auto &time_key_pair : time_to_key_)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = map_frame_;
        pose_stamped.header.stamp = time_key_pair.first;
        gtsam::Key key = time_key_pair.second;

        // O(log K)
        if (results.exists(key))
        {
            gtsam::Pose3 T_map_state = results.at<gtsam::Pose3>(key);
            gtsam::Pose3 T_map_base = T_map_state * T_state_base;

            pose_stamped.pose = toPoseMsg(T_map_base);
            path_msg.poses.push_back(pose_stamped);
        }
    }
    smoothed_path_pub_->publish(path_msg);
}

void FactorGraphNode::factorGraphTimerCallback()
{
    auto start_time = get_clock()->now();
    double cycle_duration = 1.0 / factor_graph_update_rate_;
    if (time_debt_ >= cycle_duration)
    {
        time_debt_ -= cycle_duration;
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Processing overflow detected. Skipping factor graph update. "
                              "Consider lowering the 'factor_graph_update_rate' or 'smoother_lag' parameters.");
        return;
    }

    // --- Initialize the System ---
    if (!system_initialized_)
    {
        std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_msgs_init;
        std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_msgs_init;
        std::deque<sensor_msgs::msg::Imu::SharedPtr> heading_msgs_init;
        std::deque<nav_msgs::msg::Odometry::SharedPtr> depth_msgs_init;
        std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> dvl_msgs_init;

        {
            std::scoped_lock lock(imu_queue_mutex_, gps_queue_mutex_, heading_queue_mutex_, depth_queue_mutex_, dvl_queue_mutex_);
            imu_msgs_init = imu_queue_;
            gps_msgs_init = gps_queue_;
            heading_msgs_init = heading_queue_;
            depth_msgs_init = depth_odom_queue_;
            dvl_msgs_init = dvl_queue_;
        }

        bool can_initialize = !imu_msgs_init.empty() &&
                              (!enable_gps_ || !gps_msgs_init.empty()) &&
                              (!enable_heading_ || !heading_msgs_init.empty()) &&
                              (!enable_depth_ || !depth_msgs_init.empty()) &&
                              (!enable_dvl_ || !dvl_msgs_init.empty());

        if (!can_initialize)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Waiting for required sensors: [ IMU: %s | GPS: %s | Heading: %s | Depth: %s | DVL: %s ]",
                                 !imu_msgs_init.empty() ? "OK" : "Waiting...",
                                 enable_gps_ ? (!gps_msgs_init.empty() ? "OK" : "Waiting...") : "Disabled",
                                 enable_heading_ ? (!heading_msgs_init.empty() ? "OK" : "Waiting...") : "Disabled",
                                 enable_depth_ ? (!depth_msgs_init.empty() ? "OK" : "Waiting...") : "Disabled",
                                 enable_dvl_ ? (!dvl_msgs_init.empty() ? "OK" : "Waiting...") : "Disabled");

            return;
        }

        sensor_msgs::msg::Imu::SharedPtr initial_imu = imu_msgs_init.back();
        nav_msgs::msg::Odometry::SharedPtr initial_gps = enable_gps_ ? gps_msgs_init.back() : nullptr;
        sensor_msgs::msg::Imu::SharedPtr initial_heading = enable_heading_ ? heading_msgs_init.back() : nullptr;
        nav_msgs::msg::Odometry::SharedPtr initial_depth = enable_depth_ ? depth_msgs_init.back() : nullptr;
        geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr initial_dvl = enable_dvl_ ? dvl_msgs_init.back() : nullptr;

        initializeSystem(initial_imu, initial_gps, initial_depth, initial_heading, initial_dvl);

        if (system_initialized_)
        {
            std::scoped_lock lock(imu_queue_mutex_, gps_queue_mutex_, heading_queue_mutex_, depth_queue_mutex_, dvl_queue_mutex_);

            // O(N_init_imu)
            auto imu_prune_stamp = initial_imu->header.stamp;
            auto imu_prune_it = std::remove_if(imu_queue_.begin(), imu_queue_.end(),
                                               [&](const auto &msg)
                                               { return rclcpp::Time(msg->header.stamp) <= imu_prune_stamp; });
            imu_queue_.erase(imu_prune_it, imu_queue_.end());

            if (enable_gps_)
            {
                // O(N_init_gps)
                auto prune_it = std::remove_if(gps_queue_.begin(), gps_queue_.end(),
                                               [&](const auto &msg)
                                               { return rclcpp::Time(msg->header.stamp) <= imu_prune_stamp; });
                gps_queue_.erase(prune_it, gps_queue_.end());
            }
            if (enable_heading_)
            {
                // O(N_init_heading)
                auto prune_it = std::remove_if(heading_queue_.begin(), heading_queue_.end(),
                                               [&](const auto &msg)
                                               { return rclcpp::Time(msg->header.stamp) <= imu_prune_stamp; });
                heading_queue_.erase(prune_it, heading_queue_.end());
            }
            if (enable_depth_)
            {
                // O(N_init_depth)
                auto prune_it = std::remove_if(depth_odom_queue_.begin(), depth_odom_queue_.end(),
                                               [&](const auto &msg)
                                               { return rclcpp::Time(msg->header.stamp) <= imu_prune_stamp; });
                depth_odom_queue_.erase(prune_it, depth_odom_queue_.end());
            }
            if (enable_dvl_)
            {
                // O(N_init_dvl)
                auto prune_it = std::remove_if(dvl_queue_.begin(), dvl_queue_.end(),
                                               [&](const auto &msg)
                                               { return rclcpp::Time(msg->header.stamp) <= imu_prune_stamp; });
                dvl_queue_.erase(prune_it, dvl_queue_.end());
            }
        }
        return;
    }

    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_msgs;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_msgs;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> depth_msgs;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> heading_msgs;
    std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> dvl_msgs;

    {
        std::scoped_lock lock(imu_queue_mutex_, gps_queue_mutex_, depth_queue_mutex_, heading_queue_mutex_, dvl_queue_mutex_);
        if (imu_queue_.empty())
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "No IMU messages available. Aborting factor graph update.");
            return;
        }
        imu_msgs = std::move(imu_queue_);
        gps_msgs = std::move(gps_queue_);
        depth_msgs = std::move(depth_odom_queue_);
        heading_msgs = std::move(heading_queue_);
        dvl_msgs = std::move(dvl_queue_);
    }

    // O(M_imu*log(M_imu) + H)
    std::sort(imu_msgs.begin(), imu_msgs.end(), [](const sensor_msgs::msg::Imu::SharedPtr &a, const sensor_msgs::msg::Imu::SharedPtr &b)
              { return rclcpp::Time(a->header.stamp) < rclcpp::Time(b->header.stamp); });
    imu_history_.insert(imu_history_.end(), imu_msgs.begin(), imu_msgs.end());
    std::inplace_merge(imu_history_.begin(), imu_history_.end() - imu_msgs.size(), imu_history_.end(), [](const sensor_msgs::msg::Imu::SharedPtr &a, const sensor_msgs::msg::Imu::SharedPtr &b)
                       { return rclcpp::Time(a->header.stamp) < rclcpp::Time(b->header.stamp); });
    rclcpp::Time prune_time = get_clock()->now() - rclcpp::Duration::from_seconds(smoother_lag_);
    auto prune_it_end = std::lower_bound(imu_history_.begin(), imu_history_.end(), prune_time,
                                         [](const sensor_msgs::msg::Imu::SharedPtr &msg, const rclcpp::Time &time)
                                         {
                                             return rclcpp::Time(msg->header.stamp) < time;
                                         });
    imu_history_.erase(imu_history_.begin(), prune_it_end);

    // --- Factor Graph Construction ---
    gtsam::NonlinearFactorGraph new_graph;
    gtsam::Values new_values;
    gtsam::IncrementalFixedLagSmoother::KeyTimestampMap new_timestamps;

    double last_imu_time = prev_time_;
    rclcpp::Time last_imu_stamp;
    gtsam::NavState predicted_state_for_optimizer;

    // --- Add Pre-integrated IMU Factor --
    // O(M_imu)
    for (const auto &imu_msg : imu_msgs)
    {
        double current_imu_time = rclcpp::Time(imu_msg->header.stamp).seconds();
        double dt = current_imu_time - last_imu_time;

        if (dt < 0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Received IMU message with negative dt (%.4f). Skipping measurement.",
                                 dt);
            continue;
        }

        if (dt > 0.5)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Large IMU dt (%.4f). Pre-integration accuracy may be reduced.",
                                 dt);
        }

        if (dt > 1e-9)
        {
            gtsam::Vector3 accel = toGtsam(imu_msg->linear_acceleration);
            gtsam::Vector3 gyro = toGtsam(imu_msg->angular_velocity);

            if (!std::isfinite(accel.norm()) || !std::isfinite(gyro.norm()))
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                      "IMU measurement contains NaN or Inf. Skipping measurement.");
                continue;
            }

            imu_preintegrator_->integrateMeasurement(accel, gyro, dt);
        }
        last_imu_time = current_imu_time;
        last_imu_stamp = imu_msg->header.stamp;
    }

    new_graph.emplace_shared<gtsam::CombinedImuFactor>(
        X(prev_step_), V(prev_step_), X(current_step_), V(current_step_),
        B(prev_step_), B(current_step_), *imu_preintegrator_);

    double dt_since_last_factor = last_imu_time - prev_time_;
    double dt_sqrt = sqrt(dt_since_last_factor);
    gtsam::Vector6 bias_sigmas;
    bias_sigmas << gtsam::Vector3::Constant(dt_sqrt * accel_bias_rw_sigma_),
        gtsam::Vector3::Constant(dt_sqrt * gyro_bias_rw_sigma_);
    new_graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
        B(prev_step_), B(current_step_), gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas));

    // O(log K)
    time_to_key_[last_imu_stamp] = X(current_step_);
    key_to_time_[X(current_step_)] = last_imu_stamp;

    // --- Add Other Sensor Factors ---
    // O(M_gps * log K)
    addGpsFactors(new_graph, gps_msgs);
    // O(M_depth * log K)
    addDepthFactors(new_graph, depth_msgs);
    // O(M_heading * log K)
    addHeadingFactors(new_graph, heading_msgs);
    // O(M_dvl * (log K + log H))
    addDvlFactors(new_graph, dvl_msgs, imu_history_);

    // --- Smoother Update ---
    predicted_state_for_optimizer = imu_preintegrator_->predict(gtsam::NavState(prev_pose_, prev_vel_), prev_bias_);
    new_values.insert(X(current_step_), predicted_state_for_optimizer.pose());
    new_values.insert(V(current_step_), predicted_state_for_optimizer.velocity());
    new_values.insert(B(current_step_), prev_bias_);

    new_timestamps[X(current_step_)] = last_imu_time;
    new_timestamps[V(current_step_)] = last_imu_time;
    new_timestamps[B(current_step_)] = last_imu_time;

    gtsam::Values smoothed_results_for_path;
    gtsam::Matrix new_pose_cov, new_vel_cov, new_bias_cov;
    try
    {
        // O(K^c) for some c > 1
        smoother_->update(new_graph, new_values, new_timestamps);

        gtsam::Pose3 new_pose = smoother_->calculateEstimate<gtsam::Pose3>(X(current_step_));
        gtsam::Vector3 new_vel = smoother_->calculateEstimate<gtsam::Vector3>(V(current_step_));
        gtsam::imuBias::ConstantBias new_bias = smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(current_step_));
        new_pose_cov = smoother_->marginalCovariance(X(current_step_));
        new_vel_cov = smoother_->marginalCovariance(V(current_step_));
        new_bias_cov = smoother_->marginalCovariance(B(current_step_));
        if (publish_smoothed_path_)
        {
            // O(K)
            smoothed_results_for_path = smoother_->calculateEstimate();
        }

        prev_pose_ = new_pose;
        prev_vel_ = new_vel;
        prev_bias_ = new_bias;
        imu_preintegrator_->resetIntegrationAndSetBias(prev_bias_);

        rclcpp::Time prune_time = last_imu_stamp - rclcpp::Duration::from_seconds(smoother_lag_);
        // O(log K + K_prune)
        auto prune_it_end = time_to_key_.lower_bound(prune_time);
        for (auto it = time_to_key_.begin(); it != prune_it_end; ++it)
        {
            key_to_time_.erase(it->second);
        }
        time_to_key_.erase(time_to_key_.begin(), prune_it_end);

        prev_time_ = last_imu_time;
        prev_step_ = current_step_++;
    }
    catch (const gtsam::IndeterminantLinearSystemException &e)
    {
        RCLCPP_FATAL(get_logger(), "GTSAM IndeterminantLinearSystemException: %s",
                     e.what());
        rclcpp::shutdown();
        return;
    }

    // --- Publish Global Odometry ---
    gtsam::Pose3 T_base_state = toGtsam(state_to_base_tf_.transform);
    gtsam::Pose3 transformed_prev_pose = prev_pose_ * T_base_state.inverse();
    publishGlobalOdom(transformed_prev_pose, prev_vel_, new_pose_cov, new_vel_cov);

    // --- Publish Velocity ---
    if (publish_velocity_)
    {
        publishVelocity(prev_vel_, new_vel_cov);
    }

    // --- Publish IMU Bias ---
    if (publish_imu_bias_)
    {
        publishImuBias(prev_bias_, new_bias_cov);
    }

    // --- Broadcast Global TF ---
    if (publish_global_tf_)
    {
        broadcastGlobalTf(transformed_prev_pose);
    }

    // --- Publish Smoothed Path ---
    if (publish_smoothed_path_)
    {
        // O(K * log K)
        publishSmoothedPath(smoothed_results_for_path);
    }

    auto end_time = get_clock()->now();
    rclcpp::Duration processing_duration = end_time - start_time;
    double overrun_or_underrun = processing_duration.seconds() - cycle_duration;
    time_debt_ += overrun_or_underrun;
    if (time_debt_ < 0)
    {
        time_debt_ = 0;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FactorGraphNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
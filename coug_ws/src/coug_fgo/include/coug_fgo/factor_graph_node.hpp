#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <deque>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "coug_fgo/factors/gps_factor_arm.hpp"
#include "coug_fgo/factors/depth_factor_arm.hpp"
#include "coug_fgo/factors/heading_factor_arm.hpp"
#include "coug_fgo/factors/dvl_factor_arm.hpp"
#include "coug_fgo/factors/dvl_factor.hpp"
#include "coug_fgo/utils/conversion_utils.hpp"

enum class KeyLookupResultType
{
    VALID,    // A valid key was found.
    TOO_OLD,  // The measurement is older than the oldest key in the graph.
    IN_FUTURE // The measurement is in the future compared to the newest key in the graph.
};

class FactorGraphNode : public rclcpp::Node
{
public:
    FactorGraphNode();

private:
    // --- Main Functions ---
    void initializeSystem(const sensor_msgs::msg::Imu::SharedPtr &initial_imu,
                          const nav_msgs::msg::Odometry::SharedPtr &initial_gps,
                          const nav_msgs::msg::Odometry::SharedPtr &initial_depth,
                          const sensor_msgs::msg::Imu::SharedPtr &initial_heading,
                          const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr &initial_dvl);
    void factorGraphTimerCallback();

    // --- Helper Functions ---
    void logLoadedParameters();
    void loadParameters();
    void setupRosInterfaces();
    bool lookupInitialTransforms(const sensor_msgs::msg::Imu::SharedPtr &initial_imu,
                                 const nav_msgs::msg::Odometry::SharedPtr &initial_gps,
                                 const nav_msgs::msg::Odometry::SharedPtr &initial_depth,
                                 const sensor_msgs::msg::Imu::SharedPtr &initial_heading,
                                 const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr &initial_dvl);
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
    configureImuPreintegration(const sensor_msgs::msg::Imu::SharedPtr &initial_imu);
    gtsam::Rot3 computeInitialOrientation(const sensor_msgs::msg::Imu::SharedPtr &initial_heading);
    gtsam::Point3 computeInitialPosition(const nav_msgs::msg::Odometry::SharedPtr &initial_gps,
                                         const nav_msgs::msg::Odometry::SharedPtr &initial_depth,
                                         const gtsam::Rot3 &initial_orientation);
    void addPriorFactors(gtsam::NonlinearFactorGraph &graph, gtsam::Values &values,
                         const nav_msgs::msg::Odometry::SharedPtr &initial_gps,
                         const nav_msgs::msg::Odometry::SharedPtr &initial_depth,
                         const sensor_msgs::msg::Imu::SharedPtr &initial_heading);
    std::pair<KeyLookupResultType, unsigned long> getClosestNodeKey(const rclcpp::Time &msg_stamp);
    void addGpsFactors(gtsam::NonlinearFactorGraph &graph, const std::deque<nav_msgs::msg::Odometry::SharedPtr> &gps_msgs);
    void addDepthFactors(gtsam::NonlinearFactorGraph &graph, const std::deque<nav_msgs::msg::Odometry::SharedPtr> &depth_msgs);
    void addHeadingFactors(gtsam::NonlinearFactorGraph &graph, const std::deque<sensor_msgs::msg::Imu::SharedPtr> &heading_msgs);
    gtsam::Vector3 getInterpolatedGyro(const rclcpp::Time &msg_stamp, const std::deque<sensor_msgs::msg::Imu::SharedPtr> &imu_msgs);
    void addDvlFactors(gtsam::NonlinearFactorGraph &graph, const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> &dvl_msgs,
                       const std::deque<sensor_msgs::msg::Imu::SharedPtr> &imu_msgs);
    void publishSmoothedPath(const gtsam::Values &results);
    void publishGlobalOdom(const gtsam::Pose3 &current_pose, const gtsam::Vector3 &current_velocity,
                           const gtsam::Matrix &pose_covariance, const gtsam::Matrix &vel_covariance);
    void broadcastGlobalTf(const gtsam::Pose3 &current_pose);
    void publishVelocity(const gtsam::Vector3 &current_vel_in_map,
                         const gtsam::Matrix &vel_covariance);
    void publishImuBias(const gtsam::imuBias::ConstantBias &current_bias,
                        const gtsam::Matrix &bias_covariance);

    // --- System State ---
    bool system_initialized_ = false;
    double prev_time_ = 0.0;
    size_t prev_step_ = 0;
    size_t current_step_ = 1;
    std::unordered_map<gtsam::Key, rclcpp::Time> key_to_time_;
    std::map<rclcpp::Time, gtsam::Key> time_to_key_;
    double time_debt_ = 0.0;

    // --- GTSAM Objects ---
    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrator_;
    gtsam::Pose3 prev_pose_;
    gtsam::Vector3 prev_vel_;
    gtsam::imuBias::ConstantBias prev_bias_;

    // --- Message Queues ---
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_history_;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_queue_;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> depth_odom_queue_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> heading_queue_;
    std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> dvl_queue_;

    // --- Multithreading ---
    rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
    rclcpp::CallbackGroup::SharedPtr graph_timer_cb_group_;
    std::mutex imu_queue_mutex_;
    std::mutex gps_queue_mutex_;
    std::mutex depth_queue_mutex_;
    std::mutex heading_queue_mutex_;
    std::mutex dvl_queue_mutex_;

    // --- Transformations ---
    bool have_variable_to_base_tf_ = false;
    bool have_imu_to_variable_tf_ = false;
    bool have_gps_to_variable_tf_ = false;
    bool have_depth_to_variable_tf_ = false;
    bool have_heading_to_variable_tf_ = false;
    bool have_dvl_to_variable_tf_ = false;
    geometry_msgs::msg::TransformStamped variable_to_base_tf_;
    geometry_msgs::msg::TransformStamped imu_to_variable_tf_;
    geometry_msgs::msg::TransformStamped gps_to_variable_tf_;
    geometry_msgs::msg::TransformStamped depth_to_variable_tf_;
    geometry_msgs::msg::TransformStamped heading_to_variable_tf_;
    geometry_msgs::msg::TransformStamped dvl_to_variable_tf_;

    // --- ROS Interfaces ---
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr imu_bias_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr depth_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr heading_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr factor_graph_timer_;

    // --- Parameters ---
    // Node Settings
    double factor_graph_update_rate_;
    bool publish_global_tf_, publish_smoothed_path_;
    bool publish_velocity_, publish_imu_bias_;

    // GTSAM Settings
    double smoother_lag_, gtsam_relinearize_threshold_;
    int gtsam_relinearize_skip_;

    // Topics, Frames, and Queues
    std::string imu_topic_, gps_odom_topic_, depth_odom_topic_, heading_topic_, dvl_topic_;
    std::string global_odom_topic_, smoothed_path_topic_;
    std::string velocity_topic_, imu_bias_topic_;
    std::string map_frame_, odom_frame_, base_frame_, variable_frame_;
    int imu_queue_size_, gps_queue_size_, depth_queue_size_, heading_queue_size_, dvl_queue_size_;

    // Sensor Settings
    std::vector<double> imu_gravity_;
    double accel_noise_sigma_, gyro_noise_sigma_;
    double accel_bias_rw_sigma_, gyro_bias_rw_sigma_;
    double imu_integration_covariance_;
    double gps_position_noise_sigma_;
    double depth_position_z_noise_sigma_;
    double heading_yaw_noise_sigma_;
    double dvl_velocity_noise_sigma_;
    bool imu_use_parameter_covariance_, gps_use_parameter_covariance_, depth_use_parameter_covariance_, heading_use_parameter_covariance_, dvl_use_parameter_covariance_;
    bool enable_gps_, enable_depth_, enable_heading_, enable_dvl_;

    // Prior Settings
    bool prior_use_parameter_initial_state_;
    std::vector<double> initial_state_pos_;
    double initial_state_yaw_;
    bool prior_use_parameter_covariances_;
    double prior_initial_position_sigma_, prior_initial_yaw_sigma_, prior_initial_roll_pitch_sigma_;
    double prior_initial_velocity_sigma_, prior_initial_bias_sigma_;
};
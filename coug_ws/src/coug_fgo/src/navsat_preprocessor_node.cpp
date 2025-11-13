/**
 * @file navsat_preprocessor_node.cpp
 * @brief A ROS 2 node to convert NavSatFix messages to ENU Odometry messages.
 * @author Nelson Durrant
 * @date Nov 2025
 *
 * This node subscribes to GPS NavSatFix messages, sets an origin on the first valid fix,
 * and converts subsequent GPS fixes to local ENU coordinates published as Odometry messages.
 *
 * Subscribers:
 * - gps/fix (sensor_msgs/msg/NavSatFix)
 * Publishers:
 * - odometry/gps (nav_msgs/msg/Odometry)
 * - origin (sensor_msgs/msg/NavSatFix)
 */

#include "coug_fgo/navsat_preprocessor_node.hpp"

NavsatPreprocessorNode::NavsatPreprocessorNode() : Node("navsat_preprocessor_node")
{
    RCLCPP_INFO(get_logger(), "Starting NavSat Preprocessor Node...");
    loadParameters();
    setupRosInterfaces();
    RCLCPP_INFO(get_logger(), "Startup complete! Waiting for NavSatFix messages to set origin...");
}

void NavsatPreprocessorNode::loadParameters()
{
    origin_pub_rate_ = declare_parameter<double>("origin_pub_rate", 1.0);

    // --- ROS Topics and Frames ---
    input_topic_ = declare_parameter<std::string>("input_topic", "gps/fix");
    odom_output_topic_ = declare_parameter<std::string>("odom_output_topic", "odometry/gps");
    origin_output_topic_ = declare_parameter<std::string>("origin_output_topic", "origin");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    logLoadedParameters();
}

void NavsatPreprocessorNode::logLoadedParameters()
{
    RCLCPP_INFO(get_logger(), "[Node] Origin Publish Rate: %.2f Hz", origin_pub_rate_);

    RCLCPP_INFO(get_logger(), "[ROS] Input Topic: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] Odom Output Topic: %s", odom_output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] Origin Output Topic: %s", origin_output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "[ROS] Map Frame: %s", map_frame_.c_str());
}

void NavsatPreprocessorNode::setupRosInterfaces()
{
    // --- ROS Publishers ---
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_output_topic_, 10);
    origin_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(origin_output_topic_, 10);

    // --- ROS Callback Groups ---
    sub_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = sub_cb_group_;

    // --- ROS Subscribers ---
    navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        input_topic_, 10,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
        { navsatCallback(msg); },
        sub_options);

    // --- ROS Timers ---
    origin_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / origin_pub_rate_)),
        [this]()
        { originTimerCallback(); },
        timer_cb_group_);
}

void NavsatPreprocessorNode::navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Received NavSatFix with no fix. Awaiting valid GPS data...");
        return;
    }

    // --- Set Origin ---
    if (!origin_set_)
    {
        origin_navsat_ = *msg;

        geographic_msgs::msg::GeoPoint origin_geo_point;
        origin_geo_point.latitude = msg->latitude;
        origin_geo_point.longitude = msg->longitude;
        origin_geo_point.altitude = msg->altitude;

        try
        {
            origin_utm_ = geodesy::UTMPoint(origin_geo_point);
            origin_set_ = true;

            RCLCPP_INFO(get_logger(), "GPS origin set to (Lat: %.6f, Lon: %.6f, Alt: %.2f)",
                        origin_navsat_.latitude, origin_navsat_.longitude, origin_navsat_.altitude);
            RCLCPP_INFO(get_logger(), "Origin UTM coordinate: (Zone: %d%c, E: %.2f, N: %.2f, U: %.2f)",
                        origin_utm_.zone, origin_utm_.band, origin_utm_.easting, origin_utm_.northing, origin_utm_.altitude);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to convert origin to UTM: %s. Waiting for next message.", e.what());
            return;
        }
        return;
    }

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = msg->header.frame_id;

    if (convertToEnu(msg, odom_msg))
    {
        odom_pub_->publish(odom_msg);
    }
}

void NavsatPreprocessorNode::originTimerCallback()
{
    if (origin_set_)
    {
        origin_pub_->publish(origin_navsat_);
    }
}

bool NavsatPreprocessorNode::convertToEnu(const sensor_msgs::msg::NavSatFix::SharedPtr &msg, nav_msgs::msg::Odometry &odom_msg)
{
    geographic_msgs::msg::GeoPoint current_geo_point;
    current_geo_point.latitude = msg->latitude;
    current_geo_point.longitude = msg->longitude;
    current_geo_point.altitude = msg->altitude;

    geodesy::UTMPoint current_utm_point;
    try
    {
        current_utm_point = geodesy::UTMPoint(current_geo_point);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Failed to convert NavSatFix to UTM: %s. Skipping message.", e.what());
        return false;
    }

    if (current_utm_point.zone != origin_utm_.zone || current_utm_point.band != origin_utm_.band)
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "GPS message is in a different UTM zone (%d%c) than the origin (%d%c). This is not currently supported. Skipping message.",
                              current_utm_point.zone, current_utm_point.band, origin_utm_.zone, origin_utm_.band);
        return false;
    }

    odom_msg.pose.pose.position.x = current_utm_point.easting - origin_utm_.easting;   // UTM Easting -> ENU x
    odom_msg.pose.pose.position.y = current_utm_point.northing - origin_utm_.northing; // UTM Northing -> ENU y
    odom_msg.pose.pose.position.z = current_utm_point.altitude - origin_utm_.altitude; // Altitude -> ENU z

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
    {
        odom_msg.pose.covariance[0] = msg->position_covariance[0];  // x-x
        odom_msg.pose.covariance[1] = msg->position_covariance[1];  // x-y
        odom_msg.pose.covariance[2] = msg->position_covariance[2];  // x-z
        odom_msg.pose.covariance[6] = msg->position_covariance[3];  // y-x
        odom_msg.pose.covariance[7] = msg->position_covariance[4];  // y-y
        odom_msg.pose.covariance[8] = msg->position_covariance[5];  // y-z
        odom_msg.pose.covariance[12] = msg->position_covariance[6]; // z-x
        odom_msg.pose.covariance[13] = msg->position_covariance[7]; // z-y
        odom_msg.pose.covariance[14] = msg->position_covariance[8]; // z-z
    }
    else
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "NavSatFix message has unknown position covariance. Skipping message.");
        return false;
    }

    odom_msg.pose.covariance[21] = 1e9; // roll
    odom_msg.pose.covariance[28] = 1e9; // pitch
    odom_msg.pose.covariance[35] = 1e9; // yaw

    for (int i = 0; i < 36; i++)
    {
        if (i % 7 == 0)
            odom_msg.twist.covariance[i] = -1.0;
    }

    return true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavsatPreprocessorNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
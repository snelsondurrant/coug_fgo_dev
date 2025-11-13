#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class NavsatPreprocessorNode : public rclcpp::Node
{
public:
    NavsatPreprocessorNode();

private:
    // --- Main Functions ---
    void navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void originTimerCallback();
    
    // --- Helper Functions ---
    void loadParameters();
    void logLoadedParameters();
    void setupRosInterfaces();
    bool convertToEnu(const sensor_msgs::msg::NavSatFix::SharedPtr &msg, nav_msgs::msg::Odometry &odom_msg);

    // --- Multithreading ---
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    // --- ROS Interfaces ---
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr origin_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
    rclcpp::TimerBase::SharedPtr origin_timer_;

    // --- ROS Parameters ---
    double origin_pub_rate_;
    std::string input_topic_;
    std::string odom_output_topic_;
    std::string origin_output_topic_;
    std::string map_frame_;

    // --- State Variables ---
    bool origin_set_ = false;
    sensor_msgs::msg::NavSatFix origin_navsat_;
    geodesy::UTMPoint origin_utm_;
};
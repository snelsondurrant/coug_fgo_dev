#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

namespace coug_fgo
{
    namespace utils
    {
        gtsam::Point3 toGtsam(const geometry_msgs::msg::Point &msg);
        gtsam::Vector3 toGtsam(const geometry_msgs::msg::Vector3 &msg);
        gtsam::Rot3 toGtsam(const geometry_msgs::msg::Quaternion &msg);
        gtsam::Pose3 toGtsam(const geometry_msgs::msg::Pose &msg);
        gtsam::Pose3 toGtsam(const geometry_msgs::msg::Transform &msg);
        geometry_msgs::msg::Point toPointMsg(const gtsam::Point3 &gtsam_obj);
        geometry_msgs::msg::Vector3 toVectorMsg(const gtsam::Vector3 &gtsam_obj);
        geometry_msgs::msg::Quaternion toQuatMsg(const gtsam::Rot3 &gtsam_obj);
        geometry_msgs::msg::Pose toPoseMsg(const gtsam::Pose3 &gtsam_obj);
    } // namespace utils
} // namespace coug_fgo
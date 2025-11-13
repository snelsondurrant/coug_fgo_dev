#include "coug_fgo/utils/conversion_utils.hpp"

namespace coug_fgo
{
    namespace utils
    {
        gtsam::Point3 toGtsam(const geometry_msgs::msg::Point &msg) { return {msg.x, msg.y, msg.z}; }
        gtsam::Vector3 toGtsam(const geometry_msgs::msg::Vector3 &msg) { return {msg.x, msg.y, msg.z}; }
        gtsam::Rot3 toGtsam(const geometry_msgs::msg::Quaternion &msg) { return gtsam::Rot3::Quaternion(msg.w, msg.x, msg.y, msg.z); }
        gtsam::Pose3 toGtsam(const geometry_msgs::msg::Pose &msg) { return {toGtsam(msg.orientation), toGtsam(msg.position)}; }
        gtsam::Pose3 toGtsam(const geometry_msgs::msg::Transform &msg) { return gtsam::Pose3(toGtsam(msg.rotation), toGtsam(msg.translation)); }
        geometry_msgs::msg::Point toPointMsg(const gtsam::Point3 &gtsam_obj)
        {
            geometry_msgs::msg::Point msg;
            msg.x = gtsam_obj.x();
            msg.y = gtsam_obj.y();
            msg.z = gtsam_obj.z();
            return msg;
        }
        geometry_msgs::msg::Vector3 toVectorMsg(const gtsam::Vector3 &gtsam_obj)
        {
            geometry_msgs::msg::Vector3 msg;
            msg.x = gtsam_obj.x();
            msg.y = gtsam_obj.y();
            msg.z = gtsam_obj.z();
            return msg;
        }
        geometry_msgs::msg::Quaternion toQuatMsg(const gtsam::Rot3 &gtsam_obj)
        {
            gtsam::Quaternion q = gtsam_obj.toQuaternion();
            geometry_msgs::msg::Quaternion msg;
            msg.w = q.w();
            msg.x = q.x();
            msg.y = q.y();
            msg.z = q.z();
            return msg;
        }
        geometry_msgs::msg::Pose toPoseMsg(const gtsam::Pose3 &gtsam_obj)
        {
            geometry_msgs::msg::Pose msg;
            msg.position = toPointMsg(gtsam_obj.translation());
            msg.orientation = toQuatMsg(gtsam_obj.rotation());
            return msg;
        }
    } // namespace utils
} // namespace coug_fgo
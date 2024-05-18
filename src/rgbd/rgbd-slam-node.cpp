#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "camera/color/image_raw");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/aligned_depth_to_color/image_raw");

    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f result = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msgRGB->header.stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "camera_link";
    // inverse the transform
    Eigen::Matrix3f R = result.rotationMatrix();
    Eigen::Vector3f t = result.translation();
    Eigen::Matrix3f R_inv = R.transpose();
    Eigen::Vector3f t_inv = -R_inv * t;
    Eigen::Quaternionf q(R_inv);
    transform.transform.translation.x = t_inv.x();
    transform.transform.translation.y = t_inv.y();
    transform.transform.translation.z = t_inv.z();
    transform.transform.rotation.w = q.w();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    tf_broadcaster_->sendTransform(transform);
}

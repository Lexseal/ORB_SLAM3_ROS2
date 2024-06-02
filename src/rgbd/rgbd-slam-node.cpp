#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/color/image_raw");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/aligned_depth_to_color/image_raw");

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/camera/imu", 10, std::bind(&RgbdSlamNode::grabImu, this, _1));

    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    p_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    p_tfListener = std::make_shared<tf2_ros::TransformListener>(*p_tfBuffer, true);
}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::grabImu(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
{
    ORB_SLAM3::IMU::Point imu_data(msgIMU->linear_acceleration.x, msgIMU->linear_acceleration.y,
                                   msgIMU->linear_acceleration.z, msgIMU->angular_velocity.x, msgIMU->angular_velocity.y, msgIMU->angular_velocity.z,
                                   Utility::StampToSec(msgIMU->header.stamp));
    imu_hist.push_back(imu_data);
    while (imu_hist.size() > max_imu_data_size) {
        imu_hist.pop_front();
    }
}

Eigen::Matrix4f transformStampedToEigenMatrix4f(const geometry_msgs::msg::TransformStamped& transformStamped) {
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

    // Extract translation
    const auto& translation = transformStamped.transform.translation;
    transformation(0, 3) = translation.x;
    transformation(1, 3) = translation.y;
    transformation(2, 3) = translation.z;

    // Extract rotation (quaternion)
    const auto& rotation = transformStamped.transform.rotation;
    tf2::Quaternion quat(rotation.x, rotation.y, rotation.z, rotation.w);
    tf2::Matrix3x3 mat(quat);

    // Convert tf2::Matrix3x3 to Eigen::Matrix3f
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            transformation(i, j) = mat[i][j];
        }
    }

    return transformation;
}

geometry_msgs::msg::Transform eigenMatrix4fToTransform(const Eigen::Matrix4f& transformation) {
    geometry_msgs::msg::Transform pose;

    // Extract translation
    pose.translation.x = transformation(0, 3);
    pose.translation.y = transformation(1, 3);
    pose.translation.z = transformation(2, 3);

    // Extract rotation (quaternion)
    Eigen::Matrix3f rotationMatrix = transformation.block<3, 3>(0, 0);
    Eigen::Quaternionf quaternion(rotationMatrix);
    pose.rotation.x = quaternion.x();
    pose.rotation.y = quaternion.y();
    pose.rotation.z = quaternion.z();
    pose.rotation.w = quaternion.w();

    return pose;
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

    std::vector<ORB_SLAM3::IMU::Point> imu_meas;
    // get everything between last_processed and current time stamp of the image
    for (auto it = imu_hist.begin(); it != imu_hist.end(); it++) {
        if (it->t > last_processed) {
            imu_meas.push_back(*it);
        } else if (it->t >= Utility::StampToSec(msgRGB->header.stamp)) {
            break;
        }
    }
    last_processed = Utility::StampToSec(msgRGB->header.stamp);

    Sophus::SE3f result = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp), imu_meas);

    // find the transform from map to base_link
    rclcpp::Time ros_time(msgRGB->header.stamp.sec, msgRGB->header.stamp.nanosec);

    Eigen::Matrix4f t0 = Eigen::Matrix4f::Identity();
    t0(0, 3) = 0.28;
    t0(1, 3) = 0;
    t0(2, 3) = 0.36;
    Eigen::Matrix4f t1 = Eigen::Matrix4f::Identity();
    t1.block<3, 3>(0, 0) = Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    Eigen::Matrix4f t2 = Eigen::Matrix4f::Identity();
    t2.block<3, 3>(0, 0) = Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX()).toRotationMatrix();
    Eigen::Matrix4f footprint_to_camera = t0 * t1 * t2;
    
    // we also need to get the odom to footprint transform
    geometry_msgs::msg::TransformStamped odom_to_footprint_msg;
    try {
        odom_to_footprint_msg = p_tfBuffer->lookupTransform("odom", "base_footprint", ros_time);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return;
    }
    Eigen::Matrix4f odom_to_footprint = transformStampedToEigenMatrix4f(odom_to_footprint_msg);

    // inverse the transform
    Eigen::Matrix3f R = result.rotationMatrix();
    Eigen::Vector3f t = result.translation();
    Eigen::Matrix3f R_inv = R.transpose();
    Eigen::Vector3f t_inv = -R_inv * t;
    
    Eigen::Matrix4f map_to_camera = Eigen::Matrix4f::Identity();
    map_to_camera.block<3, 3>(0, 0) = R_inv;
    map_to_camera(0, 3) = t_inv.x();
    map_to_camera(1, 3) = t_inv.y();
    map_to_camera(2, 3) = t_inv.z();
    map_to_camera = footprint_to_camera * map_to_camera;

    // find the transform from odom to base_footprint
    Eigen::Matrix4f odom_to_camera = odom_to_footprint * footprint_to_camera;
    Eigen::Matrix4f map_to_odom = map_to_camera * odom_to_camera.inverse();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msgRGB->header.stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    transform.transform = eigenMatrix4fToTransform(map_to_odom);

    tf_broadcaster_->sendTransform(transform);
}

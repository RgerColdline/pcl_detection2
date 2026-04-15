#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <mutex>

namespace pcl_detection2
{
namespace adapters
{
class PcTfMatrix
{
  public:
    static Eigen::Matrix4f poseToMatrix(const geometry_msgs::Pose &pose) {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) =
            Eigen::Quaternionf(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                               pose.orientation.z)
                .toRotationMatrix();
        transform(0, 3) = pose.position.x;
        transform(1, 3) = pose.position.y;
        transform(2, 3) = pose.position.z;
        return transform;
    }

    /**
     * @brief 获取无人机的位姿矩阵
     */
    bool get_transform(Eigen::Affine3f &transform) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!lastest_msg_) return false;

        transform = Eigen::Affine3f::Identity();
        transform.matrix() = poseToMatrix(lastest_msg_->pose);

        return true;
    }

    bool get_transform_matrix(Eigen::Matrix4f &transform) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!lastest_msg_) return false;

        transform = poseToMatrix(lastest_msg_->pose);
        return true;
    }

    /**
     * @brief 获取无人机的 Pose（位置 + 四元数）
     * @param pose 输出的 Pose 消息
     * @return 是否成功获取
     */
    bool get_pose(geometry_msgs::Pose &pose) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!lastest_msg_) return false;

        pose = lastest_msg_->pose;
        return true;
    }

    /**
     * @brief 获取无人机的位置（Eigen::Vector4f）
     * @param position 输出的位置向量 [x, y, z, 1]
     * @return 是否成功获取
     */
    bool get_position(Eigen::Vector4f &position) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!lastest_msg_) return false;

        position << lastest_msg_->pose.position.x, lastest_msg_->pose.position.y,
            lastest_msg_->pose.position.z, 1.0f;
        return true;
    }

    void odometry_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        lastest_msg_ = msg;
    }

  private:
    std::mutex mutex_;
    geometry_msgs::PoseStamped::ConstPtr lastest_msg_;
};
}  // namespace adapters
}  // namespace pcl_detection2

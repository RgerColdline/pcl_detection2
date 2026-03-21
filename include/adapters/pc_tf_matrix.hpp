#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <Eigen/Dense>

namespace pcl_detection2
{
namespace adapters
{
class PcTfMatrix
{
  public:
    /**
     * @brief 获取无人机的位姿矩阵
     */
    bool get_transform(Eigen::Affine3f &transform) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!lastest_msg_) return false;

        transform = Eigen::Affine3f::Identity();
        transform.translation() << lastest_msg_->pose.position.x, lastest_msg_->pose.position.y,
            lastest_msg_->pose.position.z;
        transform.linear() =
            Eigen::Quaternionf(lastest_msg_->pose.orientation.w, lastest_msg_->pose.orientation.x,
                               lastest_msg_->pose.orientation.y, lastest_msg_->pose.orientation.z)
                .toRotationMatrix();

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
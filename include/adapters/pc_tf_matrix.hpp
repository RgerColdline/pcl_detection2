#pragma once

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
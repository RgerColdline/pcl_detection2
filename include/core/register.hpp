#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <ros/ros.h>

#include <Eigen/Dense>

namespace pcl_detection2
{
namespace core
{
class Register
{
  public:
    Register(float ndt_resolution = 1.0f, float step_size = 0.1f, float trans_eps = 0.01f,
             int max_iter = 30, Eigen::Matrix4f last_transform = Eigen::Matrix4f::Identity())
        : ndt_resolution_(ndt_resolution), step_size_(step_size), trans_eps_(trans_eps),
          max_iter_(max_iter), last_transform_(last_transform) {
        // 初始化NDT
        ndt_.setResolution(ndt_resolution_);
        ndt_.setStepSize(step_size_);
        ndt_.setTransformationEpsilon(trans_eps_);
        ndt_.setMaximumIterations(max_iter_);

        aligned_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    /**
     * @brief 执行源点云到目标点云的配准 (Source -> Target)
     * @param source 源点云 (移动)
     * @param target 目标点云 (不动)
     * @param initial_guess 初始位姿猜测（可选）
     * @return 将源点云变换到目标点云坐标系的变换矩阵 T_target_source
     */
    const Eigen::Matrix4f &
    registerSourceToTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr &target) {
        if (source->empty() || target->empty()) {
            ROS_WARN_THROTTLE(1, "配准输入存在空点云，返回上次配准矩阵");
            return last_transform_;
        }

        // 设置输入：源是新点云，目标是全局地图
        ndt_.setInputSource(source);
        ndt_.setInputTarget(target);

        // 执行配准
        ndt_.align(*aligned_cloud_, last_transform_);

        if (ndt_.hasConverged()) {
            Eigen::Matrix4f final_transform = ndt_.getFinalTransformation();
            ROS_DEBUG_THROTTLE(
                1,
                "配准成功! 匹配分数: %f, 矩阵:\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f",
                ndt_.getFitnessScore(), final_transform(0, 0), final_transform(0, 1),
                final_transform(0, 2), final_transform(0, 3), final_transform(1, 0),
                final_transform(1, 1), final_transform(1, 2), final_transform(1, 3),
                final_transform(2, 0), final_transform(2, 1), final_transform(2, 2),
                final_transform(2, 3), final_transform(3, 0), final_transform(3, 1),
                final_transform(3, 2), final_transform(3, 3));
            last_transform_ = final_transform;
        }
        else {
            ROS_WARN("配准失败,返回上次配准矩阵");
        }
        return last_transform_;
    }

    // 保留您可能需要的其他方法...

  private:
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;

    float ndt_resolution_;
    float step_size_;
    float trans_eps_;
    int max_iter_;

    Eigen::Matrix4f last_transform_;
};
}  // namespace core
}  // namespace pcl_detection2
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>

#include <Eigen/Dense>

namespace pcl_detection2
{
namespace core
{
class Register
{
  public:
    /**
     * @brief 配准构造
     * @param max_correspondence_distance 体素分辨率
     * @param max_iter 迭代次数
     * @param transformation_epsilon 不知道
     * @param euclidean_fitness_epsilon 收敛阈值
     * @param last_transform 初矩阵
     */
    Register(float max_correspondence_distance = 0.5f,  // ICP specific param
             int max_iter                      = 50,    // ICP specific param
             float transformation_epsilon      = 1e-8,  // ICP specific param
             float euclidean_fitness_epsilon   = 0.01,  // ICP specific param
             Eigen::Matrix4f last_transform    = Eigen::Matrix4f::Identity())
        : max_correspondence_distance_(max_correspondence_distance), max_iter_(max_iter),
          transformation_epsilon_(transformation_epsilon),
          euclidean_fitness_epsilon_(euclidean_fitness_epsilon), last_transform_(last_transform) {

        // Initialize ICP
        icp_.setMaxCorrespondenceDistance(
            max_correspondence_distance_);     // Maximum distance for a correspondence to be valid
        icp_.setMaximumIterations(max_iter_);  // Maximum number of iterations before termination
        icp_.setTransformationEpsilon(
            transformation_epsilon_);          // Maximum allowed difference between two consecutive
                                               // transformations
        icp_.setEuclideanFitnessEpsilon(
            euclidean_fitness_epsilon_);       // Maximum allowed fitness difference between two

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
        icp_.setInputSource(source);
        icp_.setInputTarget(target);

        // 执行配准
        icp_.align(*aligned_cloud_, last_transform_);

        if (icp_.hasConverged()) {
            Eigen::Matrix4f final_transform = icp_.getFinalTransformation();
            ROS_DEBUG_THROTTLE(
                1,
                "配准成功! 匹配分数: %f, 矩阵:\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f",
                icp_.getFitnessScore(), final_transform(0, 0), final_transform(0, 1),
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
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;

    float max_correspondence_distance_;
    int max_iter_;
    double transformation_epsilon_    = 1e-8;  // ICP specific param
    double euclidean_fitness_epsilon_ = 0.01;  // ICP specific param

    Eigen::Matrix4f last_transform_;
};
}  // namespace core
}  // namespace pcl_detection2
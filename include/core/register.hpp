#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <limits>

namespace pcl_detection2
{
namespace core
{
template <typename PointT> class Register
{
  public:
    using PointCloudT    = pcl::PointCloud<PointT>;
    using PointCloudPtrT = typename PointCloudT::Ptr;

    struct Result {
        bool converged                  = false;
        double fitness_score            = std::numeric_limits<double>::infinity();
        Eigen::Matrix4f final_transform = Eigen::Matrix4f::Identity();
        PointCloudPtrT aligned_cloud{new PointCloudT};
    };
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
             float transformation_epsilon    = 1e-8,  // ICP specific param
             float euclidean_fitness_epsilon = 0.01)  // ICP specific param
        : max_correspondence_distance_(max_correspondence_distance), max_iter_(max_iter),
          transformation_epsilon_(transformation_epsilon),
          euclidean_fitness_epsilon_(euclidean_fitness_epsilon) {

        // Initialize ICP
        icp_.setMaxCorrespondenceDistance(
            max_correspondence_distance_);     // Maximum distance for a correspondence to be valid
        icp_.setMaximumIterations(max_iter_);  // Maximum number of iterations before termination
        icp_.setTransformationEpsilon(
            transformation_epsilon_);          // Maximum allowed difference between two consecutive
                                               // transformations
        icp_.setEuclideanFitnessEpsilon(
            euclidean_fitness_epsilon_);       // Maximum allowed fitness difference between two

        aligned_cloud_.reset(new PointCloudT);
    }

    /**
     * @brief 执行源点云到目标点云的配准 (Source -> Target)
     * @param source 源点云 (移动)
     * @param target 目标点云 (不动)
     * @param initial_guess 初始位姿猜测
     * @return 将源点云变换到目标点云坐标系的变换矩阵 T_target_source
     */
    Result registerSourceToTarget(const PointCloudPtrT &source, const PointCloudPtrT &target,
                                  const Eigen::Matrix4f &initial_guess) {
        Result result;
        result.final_transform = initial_guess;

        if (source->empty() || target->empty()) {
            ROS_WARN_THROTTLE(1, "配准输入存在空点云，跳过本次 ICP");
            return result;
        }

        icp_.setInputSource(source);
        icp_.setInputTarget(target);

        icp_.align(*aligned_cloud_, initial_guess);

        result.converged     = icp_.hasConverged();
        result.fitness_score = icp_.getFitnessScore();

        if (result.converged) {
            result.final_transform = icp_.getFinalTransformation();
            *result.aligned_cloud  = *aligned_cloud_;
            ROS_DEBUG_THROTTLE(
                1,
                "配准成功! 匹配分数: %f, 矩阵:\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f",
                result.fitness_score, result.final_transform(0, 0), result.final_transform(0, 1),
                result.final_transform(0, 2), result.final_transform(0, 3),
                result.final_transform(1, 0), result.final_transform(1, 1),
                result.final_transform(1, 2), result.final_transform(1, 3),
                result.final_transform(2, 0), result.final_transform(2, 1),
                result.final_transform(2, 2), result.final_transform(2, 3),
                result.final_transform(3, 0), result.final_transform(3, 1),
                result.final_transform(3, 2), result.final_transform(3, 3));
        }
        else {
            ROS_WARN_THROTTLE(1, "配准失败，回退到预测位姿");
        }
        return result;
    }

  private:
    pcl::IterativeClosestPoint<PointT, PointT> icp_;
    PointCloudPtrT aligned_cloud_;

    float max_correspondence_distance_;
    int max_iter_;
    double transformation_epsilon_    = 1e-8;  // ICP specific param
    double euclidean_fitness_epsilon_ = 0.01;  // ICP specific param
};
}  // namespace core
}  // namespace pcl_detection2

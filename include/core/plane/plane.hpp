#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <Eigen/Dense>
#include <vector>

namespace pcl_detection2
{
namespace core
{
namespace plane
{

/**
 * @brief 平面参数结构体，用于在平面提取、投影、反投影各阶段间传递数据
 */
struct PlaneParams
{
    using PointT      = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;
    using Ptr         = typename PointCloudT::Ptr;
    using ConstPtr    = typename PointCloudT::ConstPtr;

    /// 平面模型系数 [a, b, c, d]，平面方程: ax + by + cz + d = 0
    Eigen::Vector4f coefficients = Eigen::Vector4f::Zero();

    /// 内点索引
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

    /// 内点云
    Ptr cloud{new PointCloudT};

    /// 平面中心点 (内点均值)
    Eigen::Vector3f center = Eigen::Vector3f::Zero();

    /// 平面法向量 (归一化)
    Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();

    /// 内点数
    int point_count = 0;
};

}  // namespace plane
}  // namespace core
}  // namespace pcl_detection2

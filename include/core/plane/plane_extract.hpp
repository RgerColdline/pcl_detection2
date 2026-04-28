#pragma once

#include "plane.hpp"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <ros/ros.h>

#include <Eigen/Dense>
#include <vector>

namespace pcl_detection2
{
namespace core
{
namespace plane
{

/**
 * @brief 平面提取类
 *
 * 接收点云 → ROI 裁剪 → 欧式聚类 → 对每个聚类 RANSAC 平面提取
 * 输出多个平面参数结构体
 */
class PlaneExtract
{
  public:
    using PointT      = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;
    using Ptr         = typename PointCloudT::Ptr;
    using ConstPtr    = typename PointCloudT::ConstPtr;
    using KdTreeT     = pcl::search::KdTree<PointT>;
    using KdTreePtrT  = typename KdTreeT::Ptr;

    PlaneExtract() = default;

    /**
     * @brief 从私有 nh 加载参数
     */
    void init(ros::NodeHandle &pnh) {
        pnh_ = pnh;

        // --- ROI 参数 ---
        pnh_.param("ring_plane/roi_x_min", roi_x_min_, -1.5f);
        pnh_.param("ring_plane/roi_x_max", roi_x_max_, 0.5f);
        pnh_.param("ring_plane/roi_y_min", roi_y_min_, -1.0f);
        pnh_.param("ring_plane/roi_y_max", roi_y_max_, 1.0f);
        pnh_.param("ring_plane/roi_z_min", roi_z_min_, 0.3f);
        pnh_.param("ring_plane/roi_z_max", roi_z_max_, 2.0f);

        // --- 聚类参数 ---
        pnh_.param("ring_plane/cluster_tolerance", cluster_tolerance_, 0.05f);
        pnh_.param("ring_plane/cluster_min_size", cluster_min_size_, 100);
        pnh_.param("ring_plane/cluster_max_size", cluster_max_size_, 50000);

        // --- RANSAC 参数 ---
        pnh_.param("ring_plane/ransac_distance_threshold", ransac_distance_threshold_, 0.03f);
        pnh_.param("ring_plane/ransac_min_inliers", ransac_min_inliers_, 50);

        // --- 平面筛选 ---
        pnh_.param("ring_plane/min_plane_area_points", min_plane_area_points_, 500);
        // 法向量与水平面的最大夹角（度），只保留近似竖直的平面
        pnh_.param("ring_plane/max_plane_vertical_angle", max_plane_vertical_angle_, 30.0f);

        kdtree_ = KdTreePtrT(new KdTreeT);
    }

    /**
     * @brief 从输入点云提取所有平面
     * @param input 输入点云（已变换到世界坐标系）
     * @return 提取到的平面列表
     */
    std::vector<PlaneParams> extract(const ConstPtr &input) {
        std::vector<PlaneParams> results;
        if (!input || input->empty()) return results;

        // ---- Step 1: ROI 裁剪 ----
        Ptr roi_cloud(new PointCloudT);
        {
            pcl::CropBox<PointT> crop;
            crop.setInputCloud(input);
            crop.setMin(Eigen::Vector4f(roi_x_min_, roi_y_min_, roi_z_min_, 1.0f));
            crop.setMax(Eigen::Vector4f(roi_x_max_, roi_y_max_, roi_z_max_, 1.0f));
            crop.filter(*roi_cloud);
        }

        if (roi_cloud->size() < static_cast<size_t>(cluster_min_size_)) {
            ROS_DEBUG("[PlaneExtract] ROI 后点数不足 (%zu < %d)", roi_cloud->size(),
                      cluster_min_size_);
            return results;
        }

        // ---- Step 2: 欧式聚类 ----
        std::vector<pcl::PointIndices> cluster_indices;
        {
            kdtree_->setInputCloud(roi_cloud);

            pcl::EuclideanClusterExtraction<PointT> ec;
            ec.setClusterTolerance(cluster_tolerance_);
            ec.setMinClusterSize(cluster_min_size_);
            ec.setMaxClusterSize(cluster_max_size_);
            ec.setSearchMethod(kdtree_);
            ec.setInputCloud(roi_cloud);
            ec.extract(cluster_indices);
        }

        ROS_DEBUG("[PlaneExtract] ROI 点云 %zu 个点，聚类得到 %zu 个簇", roi_cloud->size(),
                  cluster_indices.size());

        // ---- Step 3: 对每个聚类做 RANSAC 平面提取 ----
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(roi_cloud);

        for (const auto &indices : cluster_indices) {
            // 构造该聚类的子点云
            pcl::PointIndices::Ptr cluster_inliers(new pcl::PointIndices(indices));
            Ptr cluster_cloud(new PointCloudT);
            extract.setIndices(cluster_inliers);
            extract.setNegative(false);
            extract.filter(*cluster_cloud);

            if (cluster_cloud->size() < static_cast<size_t>(ransac_min_inliers_)) continue;

            // RANSAC 平面拟合
            pcl::SACSegmentation<PointT> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ransac_distance_threshold_);
            seg.setInputCloud(cluster_cloud);

            pcl::PointIndices inlier_indices;
            pcl::ModelCoefficients coeffs;
            seg.segment(inlier_indices, coeffs);

            if (inlier_indices.indices.size() < static_cast<size_t>(ransac_min_inliers_)) continue;

            // 提取内点云
            Ptr inlier_cloud(new PointCloudT);
            extract.setInputCloud(cluster_cloud);
            extract.setIndices(pcl::PointIndices::Ptr(new pcl::PointIndices(inlier_indices)));
            extract.setNegative(false);
            extract.filter(*inlier_cloud);

            // ---- Step 4: 平面筛选 ----
            PlaneParams plane;
            plane.coefficients = Eigen::Vector4f(coeffs.values[0], coeffs.values[1],
                                                 coeffs.values[2], coeffs.values[3]);
            plane.inliers.reset(new pcl::PointIndices(inlier_indices));
            plane.cloud    = inlier_cloud;
            plane.point_count = static_cast<int>(inlier_indices.indices.size());

            // 平面法向量
            plane.normal = plane.coefficients.head<3>().normalized();

            // 只保留近似竖直的平面（法向量接近水平）
            float vertical_angle = std::acos(std::abs(plane.normal.dot(Eigen::Vector3f::UnitZ())));
            vertical_angle       = vertical_angle * 180.0f / M_PI;
            if (std::abs(90.0f - vertical_angle) > max_plane_vertical_angle_) {
                ROS_DEBUG("[PlaneExtract] 跳过非竖直平面，与竖直面夹角 %.1f° > %.1f°",
                           std::abs(90.0f - vertical_angle), max_plane_vertical_angle_);
                continue;
            }

            // 平面点数筛选
            if (plane.point_count < min_plane_area_points_) {
                ROS_DEBUG("[PlaneExtract] 跳过小平面，点数 %d < %d", plane.point_count,
                           min_plane_area_points_);
                continue;
            }

            // 计算平面中心
            plane.center = Eigen::Vector3f::Zero();
            for (const auto &pt : inlier_cloud->points) {
                plane.center += pt.getVector3fMap();
            }
            plane.center /= static_cast<float>(inlier_cloud->size());

            results.push_back(plane);
        }

        ROS_DEBUG("[PlaneExtract] 提取到 %zu 个有效平面", results.size());
        return results;
    }

  private:
    ros::NodeHandle pnh_;

    // ROI 参数
    float roi_x_min_ = -1.5f, roi_x_max_ = 0.5f;
    float roi_y_min_ = -1.0f, roi_y_max_ = 1.0f;
    float roi_z_min_ = 0.3f, roi_z_max_ = 2.0f;

    // 聚类参数
    float cluster_tolerance_  = 0.05f;
    int cluster_min_size_     = 100;
    int cluster_max_size_     = 50000;

    // RANSAC 参数
    float ransac_distance_threshold_ = 0.03f;
    int ransac_min_inliers_          = 50;

    // 平面筛选
    int min_plane_area_points_    = 500;
    float max_plane_vertical_angle_ = 30.0f;

    KdTreePtrT kdtree_;
};

}  // namespace plane
}  // namespace core
}  // namespace pcl_detection2

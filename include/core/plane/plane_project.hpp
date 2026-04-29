#pragma once

#include "plane.hpp"

#include <pcl/common/transforms.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/ros.h>

#include <Eigen/Dense>
#include <vector>
#include <algorithm>

namespace pcl_detection2
{
namespace core
{
namespace plane
{

/**
 * @brief 平面点云投影到 2D CV 图像
 *
 * 将平面点云旋转使平面法向量对齐 Z 轴，然后投影到 XY 平面为 CV 二值图像。
 * 输出：
 *  - cv::Mat         二值图像 (有激光点=255，无点=0)
 *  - Eigen::Matrix4f 投影矩阵 (3D→2D像素的变换)
 */
class PlaneProject
{
  public:
    using PointT      = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;
    using Ptr         = typename PointCloudT::Ptr;
    using ConstPtr    = typename PointCloudT::ConstPtr;

    /**
     * @brief 投影结果
     */
    struct ProjectResult
    {
        cv::Mat image;                 // 二值图像 CV_8UC1
        Eigen::Matrix4f tf_plane;      // 原始→平面坐标系变换 (plane是Z轴对齐)
        Eigen::Matrix3f tf_2d;         // 平面坐标系3D→像素变换 (第一行: col scale, 第二行: row scale)
        float x_min = 0, y_min = 0;   // 平面坐标系下的最小范围 (用于反投影)
    };

    PlaneProject() = default;

    /**
     * @brief 从私有 nh 加载参数
     */
    void init(ros::NodeHandle &pnh) {
        pnh_ = pnh;
        pnh_.param("ring_plane/projection_resolution", resolution_, 0.05f);  // m/pixel
        pnh_.param("ring_plane/projection_margin", margin_pixels_, 20.0f);       // 图像边界留白 (pixels)
    }

    /**
     * @brief 将平面点云投影为二值 CV 图像
     * @param plane 平面参数 (cloud + coefficients)
     * @return 投影结果
     */
    ProjectResult project(const PlaneParams &plane) {
        ProjectResult result;

        if (!plane.cloud || plane.cloud->empty()) return result;

        // ---- Step 1: 计算旋转矩阵，使平面法向量对齐 Z 轴 ----
        const Eigen::Vector3f src_normal = plane.normal;
        const Eigen::Vector3f dst_normal = Eigen::Vector3f::UnitZ();

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        if (std::abs(src_normal.dot(dst_normal) - 1.0f) < 1e-6f) {
            // 已经对齐，无需旋转
        }
        else if (std::abs(src_normal.dot(dst_normal) + 1.0f) < 1e-6f) {
            // 法向量反向，绕 X 轴旋转 180°
            transform.block<3, 3>(0, 0) =
                Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix();
        }
        else {
            // 通用旋转：src_normal → dst_normal
            Eigen::Vector3f rot_axis  = src_normal.cross(dst_normal).normalized();
            float rot_angle           = std::acos(src_normal.dot(dst_normal));
            transform.block<3, 3>(0, 0) =
                Eigen::AngleAxisf(rot_angle, rot_axis).toRotationMatrix();
        }

        // 平移：使平面中心到原点
        transform.block<3, 1>(0, 3) = -transform.block<3, 3>(0, 0) * plane.center;

        // ---- Step 2: 将内点变换到平面坐标系 ----
        Ptr transformed(new PointCloudT);
        pcl::transformPointCloud(*plane.cloud, *transformed, transform);

        // ---- Step 3: 计算投影范围 ----
        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();

        for (const auto &pt : transformed->points) {
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
        }

        result.x_min = min_x;
        result.y_min = min_y;

        // 扩展边界
        float margin = margin_pixels_ * resolution_;
        min_x -= margin;
        max_x += margin;
        min_y -= margin;
        max_y += margin;

        // ---- Step 4: 创建 CV 图像 ----
        float range_x  = max_x - min_x;
        float range_y  = max_y - min_y;
        int img_cols   = std::max(1, static_cast<int>(std::ceil(range_x / resolution_)));
        int img_rows   = std::max(1, static_cast<int>(std::ceil(range_y / resolution_)));

        result.image = cv::Mat::zeros(img_rows, img_cols, CV_8UC1);

        // ---- Step 5: 填充图像 (有点=255) ----
        float inv_resolution = 1.0f / resolution_;
        for (const auto &pt : transformed->points) {
            int col = static_cast<int>((pt.x - min_x) * inv_resolution);
            int row = static_cast<int>((pt.y - min_y) * inv_resolution);
            if (col >= 0 && col < img_cols && row >= 0 && row < img_rows) {
                result.image.at<uint8_t>(row, col) = 255;
            }
        }

        // ---- Step 6: 形态学闭合，填充 small gaps ----
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(result.image, result.image, cv::MORPH_CLOSE, kernel);

        // ---- Step 7: 裁剪图像到实际有点区域 (去掉四周黑边) ----
        std::vector<cv::Point> nonzero_pts;
        cv::findNonZero(result.image, nonzero_pts);
        if (!nonzero_pts.empty()) {
            cv::Rect crop_rect = cv::boundingRect(nonzero_pts);
            // 不留余量
            int pad = 0;
            crop_rect.x      = std::max(0, crop_rect.x - pad);
            crop_rect.y      = std::max(0, crop_rect.y - pad);
            crop_rect.width  = std::min(result.image.cols - crop_rect.x, crop_rect.width + 2 * pad);
            crop_rect.height = std::min(result.image.rows - crop_rect.y, crop_rect.height + 2 * pad);

            if (crop_rect.width > 0 && crop_rect.height > 0) {
                result.image = result.image(crop_rect).clone();

                // 更新投影原点：像素(0,0)对应的3D坐标 = margin调整后的min + crop偏移
                float crop_x_m = crop_rect.x * resolution_;
                float crop_y_m = crop_rect.y * resolution_;
                result.x_min = min_x + crop_x_m;
                result.y_min = min_y + crop_y_m;
            }
        }

        // ---- Step 8: 构建投影矩阵 (使用裁剪后的 x_min/y_min) ----
        result.tf_plane = transform;

        // tf_2d: 将平面坐标系的 (x, y) 映射到像素 (col, row)
        result.tf_2d = Eigen::Matrix3f::Identity();
        result.tf_2d(0, 0) = inv_resolution;   // col = (x - x_min) / res
        result.tf_2d(0, 2) = -result.x_min * inv_resolution;
        result.tf_2d(1, 1) = inv_resolution;   // row = (y - y_min) / res
        result.tf_2d(1, 2) = -result.y_min * inv_resolution;

        return result;
    }

  private:
    ros::NodeHandle pnh_;
    float resolution_     = 0.05f;   // m/pixel
    float margin_pixels_  = 20;      // 图像边界留白
};

}  // namespace plane
}  // namespace core
}  // namespace pcl_detection2

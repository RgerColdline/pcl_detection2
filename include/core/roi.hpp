#pragma once

#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <memory>

namespace pcl_detection2
{

namespace core
{
/**
 * @brief 裁掉立方体内或外点云
 */
template <typename PointT> class CropBoxRoi
{
  public:
    using PointCloudT    = pcl::PointCloud<PointT>;
    using PointCloudPtrT = typename PointCloudT::Ptr;

    /**
     * @brief roi器初始化
     * @param uav_radius
     * @param x_min
     * @param x_max
     * @param y_min
     * @param y_max
     * @param z_min
     * @param z_max
     */
    CropBoxRoi(float uav_radius, float x_min, float x_max, float y_min, float y_max, float z_min,
               float z_max)
        : uav_radius_(uav_radius), x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max),
          z_min_(z_min), z_max_(z_max) {
        roi_filter_ = std::make_unique<pcl::CropBox<PointT>>();
        uav_vec_ << uav_radius_, uav_radius_, uav_radius_, 0;
    }
    /**
     * @brief 裁掉立方体内或外点云
     * @param input 输入点云
     * @param output 输出点云
     */
    void filterROI(PointCloudPtrT input, PointCloudPtrT output) {
        PointCloudPtrT temp_cloud(new PointCloudT);
        /***********template 3**************/

        roi_filter_->setInputCloud(input);
        roi_filter_->setMin(Eigen::Vector4f(x_min_, y_min_, z_min_, 1.0));
        roi_filter_->setMax(Eigen::Vector4f(x_max_, y_max_, z_max_, 1.0));
        roi_filter_->setNegative(false);
        roi_filter_->filter(*output);

        /***********************************/
    }
    /**
     * @brief 裁掉立方体内或外点云
     * @param input 输入点云
     * @param output 输出点云
     * @param uav_position 无人机位置
     */
    void xuavROI(PointCloudPtrT input, PointCloudPtrT output, const Eigen::Vector4f &uav_position) {
        PointCloudPtrT temp_cloud(new PointCloudT);
        /***********template 3**************/

        roi_filter_->setInputCloud(input);
        roi_filter_->setMin(uav_position - uav_vec_);
        roi_filter_->setMax(uav_position + uav_vec_);
        roi_filter_->setNegative(true);
        roi_filter_->filter(*output);

        /***********************************/
    }

  private:
    std::unique_ptr<pcl::CropBox<PointT>> roi_filter_;
    float uav_radius_;
    Eigen::Vector4f uav_vec_;
    float x_min_;
    float x_max_;
    float y_min_;
    float y_max_;
    float z_min_;
    float z_max_;
};
}  // namespace core
}  // namespace pcl_detection2
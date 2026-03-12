#pragma once

#include <livox_ros_driver2/CustomMsg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <cmath>

namespace pcl_detection2
{
namespace adapters
{

/**
 * @brief Livox 点云转换工具
 */
template <typename PointT> class LivoxConverter
{
  public:
    using PointCloudT    = typename pcl::PointCloud<PointT>;
    using PointCloudPtrT = typename PointCloudT::Ptr;

    /**
     * @brief 将 Livox CustomMsg 转换为 PCL 点云
     * @param livox_msg Livox 点云消息
     * @param output_cloud 输出的 PCL 点云
     * @param debug_level 调试级别：0=无输出，1=只输出摘要，2=输出详细点信息
     */
    static bool convert(const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg,
                        PointCloudPtrT &output_cloud, int debug_level = 0) {
        if (!livox_msg) {
            ROS_ERROR("livox 点云消息为空！！！");
            return false;
        }

        output_cloud                  = PointCloudPtrT(new PointCloudT());
        output_cloud->header.stamp    = livox_msg->header.stamp.toSec() * 1e6;
        output_cloud->header.frame_id = livox_msg->header.frame_id;

        output_cloud->clear();  // 清空点云（虽然刚创建，但确保安全）
        output_cloud->reserve(livox_msg->point_num);

        // 关键转换步骤（过滤异常点）
        int count             = 0;
        int invalid_count     = 0;
        const float MAX_COORD = 20.0f;  // 最大有效坐标（米）

        for (const auto &point : livox_msg->points) {
            // 过滤无效点（NaN、Inf、超出范围）
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
                std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z) ||
                std::abs(point.x) > MAX_COORD || std::abs(point.y) > MAX_COORD ||
                std::abs(point.z) > MAX_COORD)
            {
                invalid_count++;
                continue;
            }

            PointT p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;

            // 调试级别 2：输出前 5 个点的详细信息
            if (debug_level >= 2 && count < 5) {
                ROS_INFO("第%d个点：原点云=(%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)", count, point.x,
                         point.y, point.z, p.x, p.y, p.z);
            }
            count++;

            output_cloud->push_back(p);
        }

        // 警告：异常点过多
        if (invalid_count > 0 && debug_level >= 1) {
            ROS_WARN("Livox 转换：过滤 %d 个异常点（NaN/Inf/超出范围），保留 %lu 个点",
                     invalid_count, output_cloud->size());
        }

        // 调试级别 1 或以上：输出摘要
        if (debug_level >= 1) {
            ROS_INFO("Livox 转换：输入=%d, 输出=%lu", livox_msg->point_num, output_cloud->size());
        }

        output_cloud->width    = output_cloud->size();
        output_cloud->height   = 1;
        output_cloud->is_dense = true;  // 已过滤无效点

        return !output_cloud->empty();
    }
};

}  // namespace adapters
}  // namespace pcl_detection2

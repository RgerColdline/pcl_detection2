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
    using PointCloudT    = pcl::PointCloud<PointT>;
    using PointCloudPtrT = typename PointCloudT::Ptr;

    /**
     * @brief 将 Livox CustomMsg 转换为 PCL 点云
     * @param livox_msg Livox 点云消息
     * @param output_cloud 输出的 PCL 点云
     * @param tf_ini_intensity 初始强度值（仅 PointXYZI 类型使用）
     * @param debug_level 调试级别：0=无输出，1=只输出摘要，2=输出详细点信息
     */
    static bool convert(const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg,
                        PointCloudPtrT &output_cloud, float tf_ini_intensity = 0.5f,
                        const float uav_radius = 0.3f, const float max_coord = 20.0f,
                        int debug_level = 0) {
        if (!livox_msg) {
            ROS_ERROR("livox 点云消息为空！！！");
            return false;
        }

        output_cloud                  = PointCloudPtrT(new PointCloudT());
        output_cloud->header.stamp    = livox_msg->header.stamp.toSec() * 1e6;
        output_cloud->header.frame_id = livox_msg->header.frame_id;

        output_cloud->clear();
        output_cloud->reserve(livox_msg->point_num);

        int count         = 0;
        int invalid_count = 0;

        for (const auto &point : livox_msg->points) {
            // if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
            //     std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z) ||
            //     std::abs(point.x) > max_coord || std::abs(point.x) < uav_radius ||
            //     std::abs(point.y) > max_coord || std::abs(point.y) < uav_radius ||
            //     std::abs(point.z) > max_coord)
            // {
            //     invalid_count++;
            //     if (debug_level >= 2 && invalid_count % 5 == 1) {
            //         ROS_INFO("第%d个无效点：(%.3f,%.3f,%.3f)", invalid_count, point.x, point.y,
            //                  point.z);
            //     }
            //     continue;
            // }
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
                std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z) ||
                std::abs(point.x) > max_coord || std::abs(point.y) > max_coord ||
                std::abs(point.z) > max_coord ||
                (std::abs(point.x) < uav_radius && std::abs(point.y) < uav_radius))
            {
                invalid_count++;
                if (debug_level >= 2 && invalid_count % 5 == 1) {
                    ROS_INFO("第%d个无效点：(%.3f,%.3f,%.3f)", invalid_count, point.x, point.y,
                             point.z);
                }
                continue;
            }

            PointT p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;

            if constexpr (std::is_same_v<PointT, pcl::PointXYZI>) {
                p.intensity = tf_ini_intensity;
            }

            if (debug_level >= 2 && count < 5) {
                ROS_INFO("第%d个点：原点云=(%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)", count, point.x,
                         point.y, point.z, p.x, p.y, p.z);
            }
            count++;

            output_cloud->push_back(p);
        }

        if (invalid_count > 0 && debug_level >= 1) {
            ROS_WARN_THROTTLE(1, "Livox 转换：过滤 %d 个异常点，保留 %lu 个点", invalid_count,
                              output_cloud->size());
        }

        if (debug_level >= 1) {
            ROS_INFO_THROTTLE(1, "Livox 转换：输入=%d, 输出=%lu", livox_msg->point_num,
                              output_cloud->size());
        }

        output_cloud->width    = output_cloud->size();
        output_cloud->height   = 1;
        output_cloud->is_dense = true;

        return !output_cloud->empty();
    }
};

}  // namespace adapters
}  // namespace pcl_detection2

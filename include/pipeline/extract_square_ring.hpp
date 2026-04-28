#pragma once

#include "core/plane/plane.hpp"
#include "core/plane/plane_extract.hpp"
#include "core/plane/plane_project.hpp"
#include "core/plane/point_back_project.hpp"
#include "core/template_match/square_ring_match.h"

#include <pcl_detection2/SquareRing.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace pcl_detection2
{
namespace pipeline
{

/**
 * @brief 方环提取编排类
 *
 * 整合 PlaneExtract → PlaneProject → SquareRingMatch → PointBackProject 全流程，
 * 订阅点云话题，发布 SquareRing 消息和 rviz 可视化标记。
 *
 * 修复：每个检测到的方环独立发布一条 SquareRing 消息，
 *       一条 MarkerArray 汇总所有环的可视化。
 */
class ExtractSquareRing
{
  public:
    using PointT      = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;
    using Ptr         = typename PointCloudT::Ptr;
    using ConstPtr    = typename PointCloudT::ConstPtr;

    ExtractSquareRing() = default;

    /**
     * @brief 初始化所有子模块和 ROS 通信
     */
    void init(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
        nh_  = nh;
        pnh_ = pnh;

        // --- 初始化子模块 ---
        plane_extract_.init(pnh_);
        plane_project_.init(pnh_);
        square_ring_match_.init(pnh_);

        // --- 加载编排参数 ---
        pnh_.param("ring_pipeline/enabled", enabled_, true);
        pnh_.param("ring_pipeline/input_cloud_topic", input_cloud_topic_,
                   std::string("/pcl_detection2/downsampled_accumulated_cloud"));
        pnh_.param("ring_pipeline/publish_debug_cloud", publish_debug_cloud_, true);
        pnh_.param("ring_pipeline/max_planes", max_planes_, 2);
        pnh_.param("ring_pipeline/ring_half_thickness", ring_half_thickness_, 0.25f);

        // --- ROS 通信 ---
        cloud_sub_ =
            nh_.subscribe(input_cloud_topic_, 1, &ExtractSquareRing::cloudCallback, this);

        ring_pub_ = nh_.advertise<pcl_detection2::SquareRing>("/pcl_detection2/square_ring", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/pcl_detection2/ring_markers", 5);

        if (publish_debug_cloud_) {
            plane_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
                "/pcl_detection2/ring_plane_cloud", 5);
        }

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        ROS_INFO("[ExtractSquareRing] 初始化完成，订阅: %s", input_cloud_topic_.c_str());
    }

    /**
     * @brief 直接处理点云 (供同进程内调用，避免 topic 跳转)
     * @param cloud 输入点云 (已变换到世界坐标系)
     */
    void processCloud(const ConstPtr &cloud) {
        process(cloud);
    }

  private:
    /**
     * @brief 点云回调：执行完整方环检测管线
     */
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
        if (!enabled_) return;

        Ptr cloud(new PointCloudT);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        process(cloud);
    }

    /**
     * @brief 核心处理管线
     */
    void process(const ConstPtr &cloud) {
        if (!cloud || cloud->empty()) return;

        // 构建 header
        std_msgs::Header header;
        header.stamp    = ros::Time::now();
        header.frame_id = "map";

        // ---- Step 1: 平面提取 ----
        auto planes = plane_extract_.extract(cloud);
        if (planes.empty()) {
            ROS_DEBUG_THROTTLE(2, "[ExtractSquareRing] 未检测到平面");
            return;
        }

        // 限制处理的平面数量 (按点数排序，取最大的几个)
        if (static_cast<int>(planes.size()) > max_planes_) {
            std::sort(planes.begin(), planes.end(),
                      [](const core::plane::PlaneParams &a, const core::plane::PlaneParams &b) {
                          return a.point_count > b.point_count;
                      });
            planes.resize(max_planes_);
        }

        // ---- Step 2: 投影 + 模板匹配 + 反投影 ----
        visualization_msgs::MarkerArray marker_array;
        int ring_id = 0;

        auto drone_pos = getDronePosition(header.stamp);

        for (auto &plane : planes) {
            // Step 2a: 平面 → 2D 图像
            auto proj_result = plane_project_.project(plane);
            if (proj_result.image.empty()) continue;

            // Step 2b: 模板匹配 (现在返回多个结果)
            auto match_results = square_ring_match_.match(proj_result.image);

            for (const auto &match_result : match_results) {
                if (!match_result.matched || match_result.corners.size() != 4) continue;

                // Step 2c: 反投影 → 3D 角点
                auto ring_3d = point_back_project_.computeRingPoints(
                    match_result.corners, proj_result, plane, drone_pos);

                if (ring_3d.corners.size() != 4) continue;

                // ---- Step 3: 为每个环构建独立消息并发布 ----
                pcl_detection2::SquareRing ring_msg;
                ring_msg.header = header;
                ring_msg.header.stamp = header.stamp;  // 每条消息独立时间戳

                for (const auto &corner : ring_3d.corners) {
                    geometry_msgs::Point p;
                    p.x = corner.x();
                    p.y = corner.y();
                    p.z = corner.z();
                    ring_msg.corners.push_back(p);
                }

                ring_msg.center_point.x = ring_3d.center.x();
                ring_msg.center_point.y = ring_3d.center.y();
                ring_msg.center_point.z = ring_3d.center.z();

                ring_msg.front_point.x = ring_3d.front_point.x();
                ring_msg.front_point.y = ring_3d.front_point.y();
                ring_msg.front_point.z = ring_3d.front_point.z();

                ring_msg.back_point.x = ring_3d.back_point.x();
                ring_msg.back_point.y = ring_3d.back_point.y();
                ring_msg.back_point.z = ring_3d.back_point.z();

                // 计算宽高
                float width  = (ring_3d.corners[0] - ring_3d.corners[1]).norm();
                float height = (ring_3d.corners[1] - ring_3d.corners[2]).norm();
                ring_msg.width  = width;
                ring_msg.height = height;

                // 每条环独立发布
                ring_pub_.publish(ring_msg);

                // ---- 构建 rviz 可视化标记 ----
                addRingMarkers(marker_array, ring_3d, ring_id, header.stamp);

                ++ring_id;
            }

            // ---- 调试点云 (只发一次，所有平面合并) ----
            if (publish_debug_cloud_ && plane.cloud && !plane.cloud->empty()) {
                sensor_msgs::PointCloud2 debug_cloud;
                pcl::toROSMsg(*plane.cloud, debug_cloud);
                debug_cloud.header = header;
                debug_cloud.header.frame_id = "map";
                plane_cloud_pub_.publish(debug_cloud);
            }
        }

        // ---- 发布可视化标记 ----
        if (ring_id > 0) {
            marker_pub_.publish(marker_array);
            ROS_DEBUG("[ExtractSquareRing] 检测到 %d 个方环", ring_id);
        }
    }

    /**
     * @brief 获取无人机在 map 系下的位置
     */
    Eigen::Vector3f getDronePosition(const ros::Time &stamp) {
        Eigen::Vector3f pos(0, 0, 0);
        try {
            geometry_msgs::TransformStamped tf =
                tf_buffer_->lookupTransform("map", "base_link", stamp, ros::Duration(0.1));
            pos.x() = tf.transform.translation.x;
            pos.y() = tf.transform.translation.y;
            pos.z() = tf.transform.translation.z;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(5, "[ExtractSquareRing] TF 查询失败: %s", ex.what());
        }
        return pos;
    }

    /**
     * @brief 为 rviz 创建方环可视化标记
     */
    void addRingMarkers(visualization_msgs::MarkerArray &array,
                        const core::plane::PointBackProject::RingPoints3D &ring,
                        int ring_id,
                        const ros::Time &stamp) {
        const std::string ns = "ring_" + std::to_string(ring_id);

        // ---- 方环线框 (绿色) ----
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp    = stamp;
            marker.ns              = ns;
            marker.id              = ring_id * 10 + 0;
            marker.type            = visualization_msgs::Marker::LINE_STRIP;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.scale.x         = 0.05;   // 线宽
            marker.color.r         = 0.0f;
            marker.color.g         = 1.0f;
            marker.color.b         = 0.0f;
            marker.color.a         = 1.0f;

            // 闭合四边形
            for (int i = 0; i < 4; ++i) {
                geometry_msgs::Point p;
                p.x = ring.corners[i].x();
                p.y = ring.corners[i].y();
                p.z = ring.corners[i].z();
                marker.points.push_back(p);
            }
            // 闭合：回到第一个点
            marker.points.push_back(marker.points[0]);

            array.markers.push_back(marker);
        }

        // ---- 中心点 (红色球体) ----
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp    = stamp;
            marker.ns              = ns;
            marker.id              = ring_id * 10 + 1;
            marker.type            = visualization_msgs::Marker::SPHERE;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
            marker.pose.position.x = ring.center.x();
            marker.pose.position.y = ring.center.y();
            marker.pose.position.z = ring.center.z();

            array.markers.push_back(marker);
        }

        // ---- 前方点 (绿色箭头) ----
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp    = stamp;
            marker.ns              = ns;
            marker.id              = ring_id * 10 + 2;
            marker.type            = visualization_msgs::Marker::ARROW;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.scale.x         = 0.05;  // shaft diameter
            marker.scale.y         = 0.1;   // head diameter
            marker.scale.z         = 0.1;   // head length
            marker.color.r         = 0.0f;
            marker.color.g         = 1.0f;
            marker.color.b         = 0.0f;
            marker.color.a         = 1.0f;

            geometry_msgs::Point start, end;
            start.x = ring.center.x();
            start.y = ring.center.y();
            start.z = ring.center.z();
            end.x   = ring.front_point.x();
            end.y   = ring.front_point.y();
            end.z   = ring.front_point.z();
            marker.points.push_back(start);
            marker.points.push_back(end);

            array.markers.push_back(marker);
        }

        // ---- 后方点 (蓝色箭头) ----
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp    = stamp;
            marker.ns              = ns;
            marker.id              = ring_id * 10 + 3;
            marker.type            = visualization_msgs::Marker::ARROW;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.scale.x         = 0.05;
            marker.scale.y         = 0.1;
            marker.scale.z         = 0.1;
            marker.color.r         = 0.0f;
            marker.color.g         = 0.0f;
            marker.color.b         = 1.0f;
            marker.color.a         = 1.0f;

            geometry_msgs::Point start, end;
            start.x = ring.center.x();
            start.y = ring.center.y();
            start.z = ring.center.z();
            end.x   = ring.back_point.x();
            end.y   = ring.back_point.y();
            end.z   = ring.back_point.z();
            marker.points.push_back(start);
            marker.points.push_back(end);

            array.markers.push_back(marker);
        }
    }

    // ---- 成员变量 ----
    ros::NodeHandle nh_, pnh_;

    // 子模块
    core::plane::PlaneExtract plane_extract_;
    core::plane::PlaneProject plane_project_;
    core::plane::PointBackProject point_back_project_;
    core::SquareRingMatch square_ring_match_;

    // ROS 通信
    ros::Subscriber cloud_sub_;
    ros::Publisher ring_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher plane_cloud_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 参数
    bool enabled_               = true;
    std::string input_cloud_topic_;
    bool publish_debug_cloud_   = true;
    int max_planes_             = 2;
    float ring_half_thickness_  = 0.25f;
};

}  // namespace pipeline
}  // namespace pcl_detection2

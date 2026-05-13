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
#include <ros/package.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>
#include <string>
#include <sstream>

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
        pnh_.param("ring_pipeline/dump_enabled", dump_enabled_, false);

        // 环3D位置过滤器参数
        pnh_.param("ring_filter/enabled", ring_filter_enabled_, false);
        pnh_.param("ring_filter/x_min", ring_filter_x_min_, -2.0f);
        pnh_.param("ring_filter/x_max", ring_filter_x_max_, -0.3f);
        pnh_.param("ring_filter/y_min", ring_filter_y_min_, -0.4f);
        pnh_.param("ring_filter/y_max", ring_filter_y_max_, 0.4f);
        pnh_.param("ring_filter/z_min", ring_filter_z_min_, 0.5f);
        pnh_.param("ring_filter/z_max", ring_filter_z_max_, 2.0f);

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

        ROS_DEBUG("[ExtractSquareRing] 初始化完成，订阅: %s", input_cloud_topic_.c_str());
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

        // ---- 临时调试：延时保存投影图 ----
        bool do_dump = shouldDumpNow();

        ROS_DEBUG("[ExtractSquareRing] 处理 %zu 个平面，开始投影+匹配", planes.size());

        // ---- Step 2: 投影 + 模板匹配 (收集所有候选) ----
        struct RingCandidate {
            float score;
            int plane_idx;
            core::plane::PlaneParams plane;
            core::plane::PlaneProject::ProjectResult proj;
            core::SquareRingMatch::MatchResult match;
        };
        std::vector<RingCandidate> candidates;

        auto drone_pos = getDronePosition(header.stamp);

        int plane_idx = 0;
        for (auto &plane : planes) {
            // Step 2a: 平面 → 2D 图像
            auto proj_result = plane_project_.project(plane);
            if (proj_result.image.empty()) continue;

            // Step 2b: 模板匹配
            auto match_results = square_ring_match_.match(proj_result.image);

            // ---- 临时调试：保存投影图 (放大到匹配时实际使用的尺寸) ----
            if (do_dump) {
                cv::Mat dump_img = proj_result.image;
                double dump_scale = 1.0;
                int min_side = std::min(dump_img.cols, dump_img.rows);
                if (min_side > 0 && min_side < 200) {
                    dump_scale = 200.0 / min_side;
                    cv::resize(dump_img, dump_img, cv::Size(), dump_scale, dump_scale,
                               cv::INTER_LINEAR);
                    cv::threshold(dump_img, dump_img, 127, 255, cv::THRESH_BINARY);
                }
                // 同步放大 match_results 的角点坐标
                auto scaled_matches = match_results;
                if (dump_scale != 1.0) {
                    for (auto &m : scaled_matches) {
                        for (auto &c : m.corners) { c.x *= dump_scale; c.y *= dump_scale; }
                    }
                }
                dumpOneImage(dump_img, scaled_matches);
            }

            ROS_DEBUG("[ExtractSquareRing] 平面 #%d: 匹配到 %zu 个环候选 (size=%dx%d)",
                     plane_idx, match_results.size(),
                     proj_result.image.cols, proj_result.image.rows);

            for (const auto &match_result : match_results) {
                if (!match_result.matched || match_result.corners.size() != 4) continue;
                candidates.push_back(
                    {match_result.score, plane_idx, plane, proj_result, match_result});
            }

            // ---- 调试点云 ----
            if (publish_debug_cloud_ && plane.cloud && !plane.cloud->empty()) {
                sensor_msgs::PointCloud2 debug_cloud;
                pcl::toROSMsg(*plane.cloud, debug_cloud);
                debug_cloud.header = header;
                debug_cloud.header.frame_id = "map";
                plane_cloud_pub_.publish(debug_cloud);
            }

            ++plane_idx;
        }

        // ---- Step 3: 发布方环 ----
        // 有模板匹配结果时丢弃fallback碎片，避免混合显示
        if (!candidates.empty()) {
            bool has_template = false;
            for (const auto &c : candidates) {
                if (c.score >= 0.75f) { has_template = true; break; }  // 模板加分后≥0.75
            }
            if (has_template) {
                size_t before = candidates.size();
                candidates.erase(
                    std::remove_if(candidates.begin(), candidates.end(),
                                   [](const RingCandidate &c) { return c.score <= 0.70f; }),
                    candidates.end());
                ROS_DEBUG("[ExtractSquareRing] 过滤fallback: %zu → %zu 候选 (保留模板匹配)",
                         before, candidates.size());
            }
        }

        // 最佳候选 → SquareRing 消息; 全部候选 → rviz markers (最佳=金色高亮)
        ROS_DEBUG("[ExtractSquareRing] 收集完成: %zu 个候选 (跨 %d 个平面)",
                 candidates.size(), plane_idx);

        if (!candidates.empty()) {
            // 按得分降序排序
            std::sort(candidates.begin(), candidates.end(),
                      [](const RingCandidate &a, const RingCandidate &b) {
                          return a.score > b.score;
                      });

            ROS_DEBUG("[ExtractSquareRing] Top-3 候选得分:");
            for (size_t i = 0; i < std::min(candidates.size(), size_t(3)); ++i) {
                ROS_DEBUG("  #%zu: score=%.4f plane=#%d tpl=%d scale=(%.2f,%.2f)",
                         i, candidates[i].score, candidates[i].plane_idx,
                         candidates[i].match.template_id,
                         candidates[i].match.scale_x, candidates[i].match.scale_y);
            }

            visualization_msgs::MarkerArray all_markers;
            int marker_id = 0;
            bool best_published = false;

            for (size_t i = 0; i < candidates.size(); ++i) {
                const auto &rc = candidates[i];
                bool is_best = (i == 0);

                auto ring_3d = point_back_project_.computeRingPoints(
                    rc.match.corners, rc.proj, rc.plane, drone_pos);

                if (ring_3d.corners.size() != 4) {
                    if (is_best) {
                        ROS_WARN("[ExtractSquareRing] 最佳候选反投影失败: "
                                 "score=%.4f plane=#%d",
                                 rc.score, rc.plane_idx);
                    }
                    continue;
                }

                // 环3D位置过滤（世界坐标系）
                if (ring_filter_enabled_) {
                    float cx = ring_3d.center.x();
                    float cy = ring_3d.center.y();
                    float cz = ring_3d.center.z();
                    if (cx < ring_filter_x_min_ || cx > ring_filter_x_max_ ||
                        cy < ring_filter_y_min_ || cy > ring_filter_y_max_ ||
                        cz < ring_filter_z_min_ || cz > ring_filter_z_max_) {
                        if (is_best) {
                            ROS_WARN("[ExtractSquareRing] 最佳候选超出位置范围: "
                                     "center=(%.2f,%.2f,%.2f) score=%.4f",
                                     cx, cy, cz, rc.score);
                        }
                        continue;
                    }
                }

                // 添加到总 marker array (最佳=金色, 其他=半透明绿)
                addRingMarkers(all_markers, ring_3d, marker_id, header.stamp, is_best);
                marker_id += (is_best ? 4 : 2);  // 最佳4个marker, 其他2个

                if (is_best) {
                    // 发布唯一的 SquareRing 消息
                    pcl_detection2::SquareRing ring_msg;
                    ring_msg.header = header;
                    ring_msg.header.stamp = header.stamp;

                    for (const auto &corner : ring_3d.corners) {
                        geometry_msgs::Point p;
                        p.x = corner.x(); p.y = corner.y(); p.z = corner.z();
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

                    float w = (ring_3d.corners[0] - ring_3d.corners[1]).norm();
                    float h = (ring_3d.corners[1] - ring_3d.corners[2]).norm();
                    ring_msg.width  = w;
                    ring_msg.height = h;

                    ring_pub_.publish(ring_msg);
                    best_published = true;
                }
            }

            // 发布所有 rviz markers
            marker_pub_.publish(all_markers);

            if (best_published) {
                const auto &best = candidates[0];
                auto best_3d = point_back_project_.computeRingPoints(
                    best.match.corners, best.proj, best.plane, drone_pos);
                float w = (best_3d.corners[0] - best_3d.corners[1]).norm();
                float h = (best_3d.corners[1] - best_3d.corners[2]).norm();
                ROS_DEBUG("[ExtractSquareRing] >>> 发布: score=%.4f size=%.2fx%.2f m "
                         "center=(%.2f,%.2f,%.2f) markers=%zu (共 %zu 候选)",
                         best.score, w, h,
                         best_3d.center.x(), best_3d.center.y(), best_3d.center.z(),
                         all_markers.markers.size(), candidates.size());
            } else {
                ROS_WARN("[ExtractSquareRing] 最佳候选发布失败");
            }
        } else {
            ROS_DEBUG("[ExtractSquareRing] 无候选，未发布方环");
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
     * @brief 临时调试：判断是否到了保存投影图的时间窗口
     *
     * 首次 process() 调用后 15s，之后每 5s 一次窗口，共 3 次。
     * 每个窗口中所有平面的投影图都会被保存。
     *
     * @return true = 本次 process() 应该保存所有平面投影图
     */
    bool shouldDumpNow() {
        if (!dump_enabled_) return false;
        if (dump_count_ >= 3) return false;

        const ros::Time now = ros::Time::now();

        // 首次记录起始时间
        if (dump_start_time_.isZero()) {
            dump_start_time_ = now;
            ROS_DEBUG("[ExtractSquareRing] 投影图 dump 已预约: %ds 后开始，每5s一次，共3次", 15);
            return false;
        }

        const double elapsed = (now - dump_start_time_).toSec();
        if (elapsed < 15.0) return false;

        // 计算应该处于第几个 5s 窗口
        const int expected_window = static_cast<int>((elapsed - 15.0) / 5.0);
        if (expected_window >= 3) { dump_count_ = 3; return false; }

        // 是否已进入新窗口？
        if (expected_window >= dump_count_) {
            dump_plane_idx_ = 0;          // 重置平面计数
            dump_count_     = expected_window + 1;  // 标记本窗口已消费
            return true;
        }
        return false;
    }

    /**
     * @brief 临时调试：保存单张投影图 (含匹配框)
     */
    void dumpOneImage(const cv::Mat &proj_img,
                      const std::vector<core::SquareRingMatch::MatchResult> &matches) {
        if (proj_img.empty()) return;

        // 解析输出目录
        if (dump_save_dir_.empty()) {
            try {
                dump_save_dir_ = ros::package::getPath("pcl_detection2") + "/temp";
            }
            catch (const ros::Exception &e) {
                ROS_ERROR("[ExtractSquareRing] dump 失败: %s", e.what());
                dump_count_ = 3;
                return;
            }
            mkdir(dump_save_dir_.c_str(), 0755);
        }

        // 转 BGR，画绿色匹配框
        cv::Mat vis;
        cv::cvtColor(proj_img, vis, cv::COLOR_GRAY2BGR);

        for (const auto &m : matches) {
            if (!m.matched || m.corners.size() != 4) continue;
            cv::Scalar color(0, 255, 0);
            for (int i = 0; i < 4; ++i) {
                cv::line(vis, m.corners[i], m.corners[(i + 1) % 4], color, 2);
            }
            cv::Point2f center(0, 0);
            for (int i = 0; i < 4; ++i) center += m.corners[i];
            center *= 0.25f;
            char buf[32];
            snprintf(buf, sizeof(buf), "%.2f", m.score);
            cv::putText(vis, buf, center, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }

        // 文件名: ring_dump_<窗口号>_p<平面号>_<时间戳>.png
        std::ostringstream fname;
        fname << dump_save_dir_ << "/ring_dump_" << (dump_count_ - 1)
              << "_p" << dump_plane_idx_
              << "_" << ros::Time::now().sec << ".png";

        cv::imwrite(fname.str(), vis);

        ROS_DEBUG("[ExtractSquareRing] dump #%d plane-%d: %s (%dx%d, %zu rings)",
                 dump_count_ - 1, dump_plane_idx_,
                 fname.str().c_str(), vis.cols, vis.rows, matches.size());

        ++dump_plane_idx_;
    }

    /**
     * @brief 为 rviz 创建方环可视化标记
     * @param is_best 是否为最佳候选 (金色高亮 vs 半透明绿)
     */
    void addRingMarkers(visualization_msgs::MarkerArray &array,
                        const core::plane::PointBackProject::RingPoints3D &ring,
                        int ring_id,
                        const ros::Time &stamp,
                        bool is_best = false) {
        const std::string ns = "ring_" + std::to_string(ring_id);

        // 颜色: 最佳=金色, 其他=半透明绿
        float cr = is_best ? 1.0f : 0.0f;
        float cg = is_best ? 0.65f : 1.0f;
        float cb = is_best ? 0.0f : 0.0f;
        float ca = is_best ? 1.0f : 0.35f;
        float line_w = is_best ? 0.08f : 0.03f;

        // ---- 方环线框 ----
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp    = stamp;
            marker.ns              = ns;
            marker.id              = ring_id;
            marker.type            = visualization_msgs::Marker::LINE_STRIP;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.scale.x         = line_w;
            marker.color.r         = cr;
            marker.color.g         = cg;
            marker.color.b         = cb;
            marker.color.a         = ca;

            for (int i = 0; i < 4; ++i) {
                geometry_msgs::Point p;
                p.x = ring.corners[i].x();
                p.y = ring.corners[i].y();
                p.z = ring.corners[i].z();
                marker.points.push_back(p);
            }
            marker.points.push_back(marker.points[0]);  // 闭合
            array.markers.push_back(marker);
        }

        // ---- 中心点 (最佳=红色大球, 其他=小绿球) ----
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp    = stamp;
            marker.ns              = ns;
            marker.id              = ring_id + 1;
            marker.type            = visualization_msgs::Marker::SPHERE;
            marker.action          = visualization_msgs::Marker::ADD;
            float s = is_best ? 0.12f : 0.06f;
            marker.scale.x = marker.scale.y = marker.scale.z = s;
            marker.color.r = is_best ? 1.0f : 0.0f;
            marker.color.g = is_best ? 0.2f : 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = ca;
            marker.pose.position.x = ring.center.x();
            marker.pose.position.y = ring.center.y();
            marker.pose.position.z = ring.center.z();
            array.markers.push_back(marker);
        }

        // 非最佳候选不画箭头 (减少视觉干扰)
        if (!is_best) return;

        // ---- 前方点 (金色箭头) ----
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp    = stamp;
            marker.ns              = ns;
            marker.id              = ring_id + 2;
            marker.type            = visualization_msgs::Marker::ARROW;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.scale.x         = 0.06;
            marker.scale.y         = 0.12;
            marker.scale.z         = 0.12;
            marker.color.r         = 1.0f;
            marker.color.g         = 0.65f;
            marker.color.b         = 0.0f;
            marker.color.a         = 1.0f;

            geometry_msgs::Point start, end;
            start.x = ring.center.x(); start.y = ring.center.y(); start.z = ring.center.z();
            end.x   = ring.front_point.x(); end.y = ring.front_point.y(); end.z = ring.front_point.z();
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
            marker.id              = ring_id + 3;
            marker.type            = visualization_msgs::Marker::ARROW;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.scale.x         = 0.06;
            marker.scale.y         = 0.12;
            marker.scale.z         = 0.12;
            marker.color.r         = 0.0f;
            marker.color.g         = 0.4f;
            marker.color.b         = 1.0f;
            marker.color.a         = 1.0f;

            geometry_msgs::Point start, end;
            start.x = ring.center.x(); start.y = ring.center.y(); start.z = ring.center.z();
            end.x   = ring.back_point.x(); end.y = ring.back_point.y(); end.z = ring.back_point.z();
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

    // 环3D位置过滤器
    bool ring_filter_enabled_   = false;
    float ring_filter_x_min_    = -2.0f;
    float ring_filter_x_max_    = -0.3f;
    float ring_filter_y_min_    = -0.4f;
    float ring_filter_y_max_    = 0.4f;
    float ring_filter_z_min_    = 0.5f;
    float ring_filter_z_max_    = 2.0f;

    // ---- 临时调试: 延时保存投影图 ----
    ros::Time dump_start_time_;
    std::string dump_save_dir_;
    int dump_count_ = 0;
    int dump_plane_idx_ = 0;
    bool dump_enabled_ = false;
};

}  // namespace pipeline
}  // namespace pcl_detection2

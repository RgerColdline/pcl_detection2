#pragma once

#include "core/template_match/template_loader.h"

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace pcl_detection2
{
namespace core
{

/**
 * @brief 方环检测类（固定尺寸找缺口方案）
 *
 * 在投影的二值图像中定位方环孔洞。
 * 2026-08 重构：弃用多尺度模板匹配（169×N 次 matchTemplate），改为
 *   "找缺口 + 固定尺寸验证"：
 *   投影图反相 → findContours 找黑区缺口（环孔洞）→ minAreaRect
 *   验证边长落在固定方环尺寸范围内 → 输出 4 角点。
 * 方环检测结果仅用于可视化（实际任务走定点/穿环，不参与控制），性能优先。
 */
class SquareRingMatch
{
  public:
    SquareRingMatch() = default;

    /**
     * @brief 从私有 nh 加载参数
     */
    void init(ros::NodeHandle &pnh) {
        pnh_ = pnh;

        // --- 模板列表（YAML数组，已弃用，仅保留便于回退） ---
        pnh_.param("ring_match/template_paths", template_paths_, std::vector<std::string>{});
        if (template_paths_.empty()) {
            std::string single_path;
            pnh_.param("ring_match/template_path", single_path, std::string(""));
            if (!single_path.empty()) {
                template_paths_.push_back(single_path);
            }
        }

        // --- 旧匹配参数（已弃用，仅保留便于回退） ---
        pnh_.param("ring_match/match_threshold", match_threshold_, 0.65f);
        pnh_.param("ring_match/match_method", match_method_,
                   static_cast<int>(cv::TM_CCOEFF_NORMED));
        pnh_.param("ring_match/scale_x_min", scale_x_min_, 0.8);
        pnh_.param("ring_match/scale_x_max", scale_x_max_, 1.2);
        pnh_.param("ring_match/scale_x_step", scale_x_step_, 0.05);
        pnh_.param("ring_match/scale_y_min", scale_y_min_, 0.8);
        pnh_.param("ring_match/scale_y_max", scale_y_max_, 1.2);
        pnh_.param("ring_match/scale_y_step", scale_y_step_, 0.05);
        pnh_.param("ring_match/template_thresholds", template_thresholds_, std::vector<double>{});
        pnh_.param("ring_match/nms_iou_threshold", nms_iou_threshold_, 0.3);

        // --- 形态学参数 ---
        pnh_.param("ring_match/morphology_kernel_size", morphology_kernel_size_, 5);

        // --- 旧轮廓检测 fallback 参数（已弃用，仅保留便于回退） ---
        pnh_.param("ring_match/expected_size_min", expected_size_min_, 20);
        pnh_.param("ring_match/expected_size_max", expected_size_max_, 200);
        pnh_.param("ring_match/contour_area_ratio_min", contour_area_ratio_min_, 0.3f);

        // --- 最大检测环数 ---
        pnh_.param("ring_match/max_rings_per_image", max_rings_per_image_, 3);

        // --- 固定尺寸方环检测参数（找缺口 + 固定尺寸验证，替代多尺度模板匹配）---
        pnh_.param("ring_match/fixed_ring_min_px", fixed_ring_min_px_, 10);
        pnh_.param("ring_match/fixed_ring_max_px", fixed_ring_max_px_, 45);
        pnh_.param("ring_match/max_aspect_ratio", max_aspect_ratio_, 2.0f);
        pnh_.param("ring_match/min_fill_ratio", min_fill_ratio_, 0.5f);
        pnh_.param("ring_match/min_hole_area", min_hole_area_, 80.0);

        // --- 环孔洞验证（旧模板路径使用，保留便于回退） ---
        pnh_.param("ring_match/min_hole_ratio", min_hole_ratio_, 0.3f);

        // 加载模板 (通过通用 TemplateLoader，可移植路径解析)
        // 注：多尺度模板匹配已弃用（改固定尺寸找缺口），模板仅保留便于回退
        TemplateLoader loader("pcl_detection2");
        templates_ = loader.load(template_paths_);
    }

    /**
     * @brief 匹配结果
     */
    struct MatchResult
    {
        bool matched = false;
        std::vector<cv::Point2f> corners;  // 4个角点 (像素坐标，顺时针)
        float score     = 0.0f;
        int template_id = -1;              // 匹配到的模板索引
        double scale_x = 1.0, scale_y = 1.0;
    };

    /**
     * @brief 在投影图像中匹配方环孔洞（支持多个环）
     * @param projected_image 投影的二值图像 (CV_8UC1, 有点=255, 无点=0)
     * @return 匹配结果列表
     */
    std::vector<MatchResult> match(const cv::Mat &projected_image) {
        std::vector<MatchResult> results;

        if (projected_image.empty()) return results;

        // ---- Step 0: 形态学闭合，填充 LiDAR 点云稀疏导致的空隙 ----
        // 投影二值图来自 LiDAR 点云，因为点云稀疏 + 累积偏移，
        // 方环边界上往往有很多孔洞和断裂，需要用较大核闭合。
        cv::Mat closed_img;
        cv::Mat morph_kernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(morphology_kernel_size_, morphology_kernel_size_));
        cv::morphologyEx(projected_image, closed_img, cv::MORPH_CLOSE, morph_kernel);
        if (morphology_kernel_size_ > 3) {
            // 大核闭合后可能过度膨胀，用同样核做一次 OPEN 恢复尺寸
            cv::morphologyEx(closed_img, closed_img, cv::MORPH_OPEN, morph_kernel);
        }

        // ---- 固定尺寸方环检测：找缺口（孔洞）+ 固定尺寸验证 ----
        // 不再做多尺度模板匹配（169×N 次 matchTemplate），检测结果仅用于可视化，性能优先
        results = detectRingByHole(closed_img);

        ROS_DEBUG("[SquareRingMatch] 固定尺寸缺口检测: %zu 个方环", results.size());
        return results;
    }

  private:
    /**
     * @brief 固定尺寸方环检测：找缺口（孔洞）+ 固定尺寸验证
     *
     * 思路：投影二值图（有点=255，无点=0）中，方环孔洞是没有激光点的黑色缺口。
     *       反相后黑区变白区 → findContours 找所有白区连通域（原图黑区）→
     *       排除触碰边界的背景缺口 → minAreaRect 验证边长落在固定方环尺寸
     *       （1.1m @ 0.05m/px ≈ 22px）范围内 + 方形度 + 填充度 → 输出 4 角点。
     * 复杂度：1 次形态学 + 1 次 findContours + 每候选 O(1) 验证，亚毫秒级。
     *
     * @param image 形态学闭合后的二值图
     * @return 匹配结果列表（固定高分，仅用于可视化）
     */
    std::vector<MatchResult> detectRingByHole(const cv::Mat &image) {
        std::vector<MatchResult> results;
        const int img_w = image.cols, img_h = image.rows;
        if (img_w <= 0 || img_h <= 0) return results;

        // ---- Step 1: 反相 → 黑区（缺口/孔洞）变白 ----
        cv::Mat inverted;
        cv::bitwise_not(image, inverted);

        // ---- Step 2: 找所有白色连通域（即原图的黑色区域）----
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(inverted, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        const int border_margin = 2;

        for (const auto &contour : contours) {
            if (static_cast<int>(results.size()) >= max_rings_per_image_) break;

            double area = cv::contourArea(contour);
            if (area < min_hole_area_) continue;

            cv::Rect bbox = cv::boundingRect(contour);

            // 排除触碰图像边界的区域（环外背景缺口）
            if (bbox.x <= border_margin || bbox.y <= border_margin ||
                bbox.x + bbox.width >= img_w - border_margin ||
                bbox.y + bbox.height >= img_h - border_margin)
                continue;

            // 排除占图像面积过大的区域（整个背景）
            double img_area = static_cast<double>(img_w) * img_h;
            if (img_area > 0 && area > img_area * 0.5) continue;

            // ---- Step 3: 固定尺寸验证 ----
            cv::RotatedRect rrect = cv::minAreaRect(contour);
            float short_side      = std::min(rrect.size.width, rrect.size.height);
            float long_side       = std::max(rrect.size.width, rrect.size.height);

            // 边长必须落在固定方环尺寸范围内（环孔 1.1m @ 0.05m/px ≈ 22px）
            if (short_side < fixed_ring_min_px_ || short_side > fixed_ring_max_px_) continue;

            // 方形度：长边/短边 不能太离谱（透视容忍）
            if (long_side > short_side * max_aspect_ratio_) continue;

            // 填充度：孔洞应接近实心（轮廓面积占外接矩形比例足够高）
            double bbox_area = static_cast<double>(bbox.width) * bbox.height;
            if (bbox_area > 0 && area < bbox_area * min_fill_ratio_) continue;

            // ---- Step 4: 输出 4 角点 ----
            cv::Point2f box[4];
            rrect.points(box);

            MatchResult result;
            result.matched     = true;
            result.score       = 0.8f;  // 固定高分（仅可视化用，统一走模板加分分支）
            result.template_id = -1;
            result.corners     = sortCornersClockwise({box[0], box[1], box[2], box[3]});
            results.push_back(result);

            ROS_DEBUG("[SquareRingMatch] 固定尺寸缺口检出: 短边=%.1fpx 长边=%.1fpx aspect=%.2f",
                      short_side, long_side, long_side / short_side);
        }

        return results;
    }

    /**
     * @brief 将4个点按顺时针排序 (左上→右上→右下→左下)
     */
    static std::vector<cv::Point2f> sortCornersClockwise(const std::vector<cv::Point2f> &pts) {
        std::vector<cv::Point2f> sorted = pts;
        std::sort(sorted.begin(), sorted.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
            return (a.x + a.y) < (b.x + b.y);
        });
        cv::Point2f tl = sorted[0];
        cv::Point2f br = sorted[3];

        cv::Point2f tr, bl;
        float max_diff = -1e9f, min_diff = 1e9f;
        for (const auto &p : pts) {
            float diff = p.x - p.y;
            if (diff > max_diff) {
                max_diff = diff;
                tr       = p;
            }
            if (diff < min_diff) {
                min_diff = diff;
                bl       = p;
            }
        }

        return {tl, tr, br, bl};
    }

    ros::NodeHandle pnh_;

    // 模板（旧多尺度匹配用，已弃用，仅保留便于回退）
    std::vector<std::string> template_paths_;
    std::vector<cv::Mat> templates_;
    std::vector<double> template_thresholds_;

    // 旧匹配参数（已弃用，仅保留便于回退）
    float match_threshold_ = 0.65f;
    int match_method_      = cv::TM_CCOEFF_NORMED;
    double scale_x_min_ = 0.8, scale_x_max_ = 1.2, scale_x_step_ = 0.05;
    double scale_y_min_ = 0.8, scale_y_max_ = 1.2, scale_y_step_ = 0.05;
    double nms_iou_threshold_     = 0.3;
    int expected_size_min_        = 20;
    int expected_size_max_        = 200;
    float contour_area_ratio_min_ = 0.3f;
    float min_hole_ratio_         = 0.3f;

    // 形态学
    int morphology_kernel_size_ = 5;

    // 每张图最大环数
    int max_rings_per_image_ = 3;

    // 固定尺寸方环检测参数（当前使用）
    int fixed_ring_min_px_  = 10;
    int fixed_ring_max_px_  = 45;
    float max_aspect_ratio_ = 2.0f;
    float min_fill_ratio_   = 0.5f;
    double min_hole_area_   = 80.0;
};

}  // namespace core
}  // namespace pcl_detection2

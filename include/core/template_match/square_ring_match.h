#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/ros.h>

#include "core/template_match/template_loader.h"

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

namespace pcl_detection2
{
namespace core
{

/**
 * @brief 方环模板匹配类
 *
 * 从文件加载模板，通过多尺度模板匹配在投影的二值图像中定位方环孔洞。
 * 参考 cloud_recognition-race_2025_10 的 template_match.cpp 实现。
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

        // --- 模板列表（YAML数组） ---
        pnh_.param("ring_match/template_paths", template_paths_,
                   std::vector<std::string>{});

        // --- 如果 template_paths 为空，回退到单模板路径 ---
        if (template_paths_.empty()) {
            std::string single_path;
            pnh_.param("ring_match/template_path", single_path, std::string(""));
            if (!single_path.empty()) {
                template_paths_.push_back(single_path);
            }
        }

        // --- 匹配参数 ---
        pnh_.param("ring_match/match_threshold", match_threshold_, 0.65f);
        pnh_.param("ring_match/match_method", match_method_, static_cast<int>(cv::TM_CCOEFF_NORMED));
        pnh_.param("ring_match/scale_x_min", scale_x_min_, 0.8);
        pnh_.param("ring_match/scale_x_max", scale_x_max_, 1.2);
        pnh_.param("ring_match/scale_x_step", scale_x_step_, 0.05);
        pnh_.param("ring_match/scale_y_min", scale_y_min_, 0.8);
        pnh_.param("ring_match/scale_y_max", scale_y_max_, 1.2);
        pnh_.param("ring_match/scale_y_step", scale_y_step_, 0.05);

        // --- 每个模板的独立阈值 (可选) ---
        pnh_.param("ring_match/template_thresholds", template_thresholds_,
                   std::vector<double>{});

        // --- 非极大值抑制 IoU 阈值 ---
        pnh_.param("ring_match/nms_iou_threshold", nms_iou_threshold_, 0.3);

        // --- 形态学参数 ---
        pnh_.param("ring_match/morphology_kernel_size", morphology_kernel_size_, 5);

        // --- 轮廓检测 fallback 参数 ---
        pnh_.param("ring_match/expected_size_min", expected_size_min_, 20);
        pnh_.param("ring_match/expected_size_max", expected_size_max_, 200);
        pnh_.param("ring_match/contour_area_ratio_min", contour_area_ratio_min_, 0.3f);

        // --- 最大检测环数 ---
        pnh_.param("ring_match/max_rings_per_image", max_rings_per_image_, 3);

        // --- 环孔洞验证：过滤实墙误匹配 ---
        pnh_.param("ring_match/min_hole_ratio", min_hole_ratio_, 0.3f);

        // 加载模板 (通过通用 TemplateLoader，可移植路径解析)
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
        float score = 0.0f;
        int template_id = -1;              // 匹配到的模板索引
        double scale_x = 1.0, scale_y = 1.0;
    };

    /**
     * @brief 在投影图像中匹配方环孔洞（支持多个环）
     * @param projected_image 投影的二值图像 (CV_8UC1, 有点=255, 无点=0)
     * @return 匹配结果列表（按得分降序）
     */
    std::vector<MatchResult> match(const cv::Mat &projected_image) {
        std::vector<MatchResult> results;

        if (projected_image.empty()) return results;

        // ---- 放大过小的投影图，使环孔洞像素尺寸追上模板 ----
        cv::Mat work_img = projected_image;
        double img_scale = 1.0;
        int min_side = std::min(projected_image.cols, projected_image.rows);
        if (min_side > 0 && min_side < 200) {
            img_scale = 200.0 / min_side;
            cv::resize(projected_image, work_img, cv::Size(), img_scale, img_scale,
                       cv::INTER_LINEAR);
            cv::threshold(work_img, work_img, 127, 255, cv::THRESH_BINARY);
            ROS_INFO("[SquareRingMatch] 投影图过小 (%dx%d), 放大 %.1fx → %dx%d",
                     projected_image.cols, projected_image.rows,
                     img_scale, work_img.cols, work_img.rows);
        }

        if (templates_.empty()) {
            // 无模板时回退到轮廓检测
            ROS_DEBUG("[SquareRingMatch] 无模板，使用轮廓检测");
            auto contour_results = detectByContour(work_img);
            results.insert(results.end(), contour_results.begin(), contour_results.end());
            // 缩放回原始坐标
            if (img_scale != 1.0) scaleResultsBack(results, img_scale);
            return results;
        }

        // 计算 x/y 方向的缩放步数 (避免浮点累加误差)
        int nx = std::max(1, static_cast<int>(std::round((scale_x_max_ - scale_x_min_) / scale_x_step_)) + 1);
        int ny = std::max(1, static_cast<int>(std::round((scale_y_max_ - scale_y_min_) / scale_y_step_)) + 1);

        // ---- 多尺度多模板匹配：收集所有候选 ---- &
        std::vector<MatchResult> candidates;

        for (size_t t = 0; t < templates_.size(); ++t) {
            const cv::Mat &templ = templates_[t];
            double t_threshold = (t < template_thresholds_.size())
                                     ? template_thresholds_[t]
                                     : match_threshold_;

            for (int i = 0; i < nx; ++i) {
                double sx = scale_x_min_ + i * scale_x_step_;
                for (int j = 0; j < ny; ++j) {
                    double sy = scale_y_min_ + j * scale_y_step_;

                    cv::Mat resized;
                    cv::resize(templ, resized, cv::Size(), sx, sy, cv::INTER_LINEAR);

                    if (resized.cols > work_img.cols ||
                        resized.rows > work_img.rows)
                        continue;

                    int result_cols = work_img.cols - resized.cols + 1;
                    int result_rows = work_img.rows - resized.rows + 1;
                    if (result_cols <= 0 || result_rows <= 0) continue;

                    cv::Mat match_result;
                    cv::matchTemplate(work_img, resized, match_result, match_method_);

                    bool is_inverse = (match_method_ == cv::TM_SQDIFF ||
                                       match_method_ == cv::TM_SQDIFF_NORMED);

                    // 遍历所有匹配位置，收集满足阈值的候选
                    for (int y = 0; y < result_rows; ++y) {
                        for (int x = 0; x < result_cols; ++x) {
                            double val = static_cast<double>(match_result.at<float>(y, x));

                            bool pass = is_inverse ? (val <= (1.0 - t_threshold))
                                                   : (val >= t_threshold);
                            if (!pass) continue;

                            cv::Point pt1(x, y);
                            cv::Point pt2(x + resized.cols, y + resized.rows);

                            MatchResult cand;
                            cand.matched     = true;
                            cand.score       = static_cast<float>(is_inverse ? (1.0 - val) : val);
                            cand.template_id = static_cast<int>(t);
                            cand.scale_x     = sx;
                            cand.scale_y     = sy;
                            // 先用匹配框的角点占位，后续在 NMS 后细化
                            cand.corners = {
                                cv::Point2f(pt1.x, pt1.y),
                                cv::Point2f(pt2.x, pt1.y),
                                cv::Point2f(pt2.x, pt2.y),
                                cv::Point2f(pt1.x, pt2.y)};
                            // 暂存 pt1/pt2 用于 NMS
                            // (用 corners[0] 和 corners[2] 等效)
                            candidates.push_back(cand);
                        }
                    }
                }
            }
        }

        // ---- 非极大值抑制 (NMS) ----
        size_t raw_count = candidates.size();
        candidates = nonMaxSuppression(candidates);

        // ---- 限制数量并细化角点 ----
        if (static_cast<int>(candidates.size()) > max_rings_per_image_) {
            candidates.resize(max_rings_per_image_);
        }

        ROS_INFO("[SquareRingMatch] 模板匹配: raw=%zu → NMS=%zu (max_per_image=%d), 开始孔洞验证",
                 raw_count, candidates.size(), max_rings_per_image_);

        int rejected = 0;
        for (auto &cand : candidates) {
            // 保存原始匹配框 (用于后续环孔洞验证)
            cv::Point2f pt1 = cand.corners[0];
            cv::Point2f pt2 = cand.corners[2];
            cv::Rect orig_match_rect(static_cast<int>(pt1.x), static_cast<int>(pt1.y),
                                     static_cast<int>(pt2.x - pt1.x),
                                     static_cast<int>(pt2.y - pt1.y));
            orig_match_rect &= cv::Rect(0, 0, work_img.cols, work_img.rows);

            // ---- 环孔洞验证：排除实墙误匹配 (使用原始匹配框) ----
            // 必须在轮廓细化之前执行，因为细化后角点会变成内孔轮廓，
            // 此时内孔区域必然是纯黑的，无法区分真环和墙缝
            if (!validateRingHole(work_img, orig_match_rect)) {
                ROS_INFO("[SquareRingMatch] 孔洞验证拒绝 #%d: score=%.3f (疑似实墙)",
                         ++rejected, cand.score);
                continue;
            }

            // 用匹配框从原始图中提取 ROI，用 minAreaRect 细化角点
            cv::Rect roi = orig_match_rect;

            if (roi.width > 5 && roi.height > 5) {
                cv::Mat roi_img = work_img(roi);
                cv::Mat inverted;
                cv::bitwise_not(roi_img, inverted);

                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(inverted, contours, cv::RETR_EXTERNAL,
                                 cv::CHAIN_APPROX_SIMPLE);

                if (!contours.empty()) {
                    auto max_contour = *std::max_element(
                        contours.begin(), contours.end(),
                        [](const auto &a, const auto &b) {
                            return cv::contourArea(a) < cv::contourArea(b);
                        });

                    float roi_area = roi.width * roi.height;
                    if (cv::contourArea(max_contour) >
                        roi_area * contour_area_ratio_min_) {
                        cv::RotatedRect rect = cv::minAreaRect(max_contour);
                        cv::Point2f box[4];
                        rect.points(box);

                        for (int k = 0; k < 4; ++k) {
                            box[k].x += roi.x;
                            box[k].y += roi.y;
                        }

                        cand.corners = sortCornersClockwise(
                            {box[0], box[1], box[2], box[3]});
                    }
                }
            }

            // 尺寸评分：环越小分越低 (短边<30px线性扣分，以上不扣)
            cv::Rect corner_bbox = cv::boundingRect(cand.corners);
            float hole_short = std::min(corner_bbox.width, corner_bbox.height);
            float size_factor = 1.0f;
            if (hole_short < 150.0f) {
                size_factor = hole_short / 150.0f;        // 太小 → 扣分
            }
            cand.score = cand.score * size_factor;

            // 模板匹配加分加成，确保优先于轮廓fallback
            cand.score += 0.30f;
            results.push_back(cand);
            ROS_DEBUG("[SquareRingMatch] 匹配成功+孔洞验证通过: t=%d scale=(%.2f,%.2f) score=%.3f",
                      cand.template_id, cand.scale_x, cand.scale_y, cand.score);
        }

        ROS_INFO("[SquareRingMatch] 模板路径验证: %zu 通过 / %zu 候选 (拒绝=%d)",
                 results.size(), candidates.size(), rejected);

        // ---- Fallback: 轮廓检测 ----
        if (results.empty()) {
            ROS_INFO("[SquareRingMatch] 模板路径 0 结果 → 进入轮廓fallback");
            auto contour_results = detectByContour(work_img);
            ROS_INFO("[SquareRingMatch] 轮廓fallback检出: %zu 个四边形候选", contour_results.size());
            // 轮廓检测结果也需要孔洞验证
            // 轮廓角点指向内部孔洞，需向外扩展匹配框来验证环体
            int contour_rejected = 0;
            for (const auto &cr : contour_results) {
                if (cr.corners.size() != 4) continue;
                cv::Rect bbox = cv::boundingRect(cr.corners);
                int expand = std::max(4, std::min(bbox.width, bbox.height) / 4);
                cv::Rect expanded(
                    std::max(0, bbox.x - expand),
                    std::max(0, bbox.y - expand),
                    std::min(work_img.cols - std::max(0, bbox.x - expand),
                             bbox.width + 2 * expand),
                    std::min(work_img.rows - std::max(0, bbox.y - expand),
                             bbox.height + 2 * expand));
                if (validateRingHole(work_img, expanded)) {
                    results.push_back(cr);
                } else {
                    ++contour_rejected;
                }
            }
            ROS_INFO("[SquareRingMatch] 轮廓fallback验证: %zu 通过 / %zu 候选 (拒绝=%d)",
                     results.size(), contour_results.size(), contour_rejected);
        }

        // ---- 缩放回原始图像坐标 ----
        if (img_scale != 1.0) scaleResultsBack(results, img_scale);

        ROS_INFO("[SquareRingMatch] 最终返回: %zu 个方环", results.size());
        return results;
    }

  private:
    /**
     * @brief 将匹配结果的角点坐标从放大图缩放回原始图
     */
    static void scaleResultsBack(std::vector<MatchResult> &results, double scale) {
        for (auto &r : results) {
            for (auto &c : r.corners) {
                c.x /= static_cast<float>(scale);
                c.y /= static_cast<float>(scale);
            }
        }
    }

    /**
     * @brief IoU-based NMS，按得分降序处理，保留非重叠的高分匹配
     */
    std::vector<MatchResult> nonMaxSuppression(const std::vector<MatchResult> &candidates) {
        if (candidates.empty()) return candidates;

        // 按 score 降序排序
        std::vector<size_t> indices(candidates.size());
        for (size_t i = 0; i < indices.size(); ++i) indices[i] = i;
        std::sort(indices.begin(), indices.end(),
                  [&](size_t a, size_t b) {
                      return candidates[a].score > candidates[b].score;
                  });

        std::vector<bool> suppressed(candidates.size(), false);
        std::vector<MatchResult> filtered;

        for (size_t i = 0; i < indices.size(); ++i) {
            size_t idx_i = indices[i];
            if (suppressed[idx_i]) continue;

            const auto &box_i = candidates[idx_i];
            cv::Point2f pt1_i = box_i.corners[0];
            cv::Point2f pt2_i = box_i.corners[2];
            double area_i = (pt2_i.x - pt1_i.x) * (pt2_i.y - pt1_i.y);

            filtered.push_back(box_i);

            for (size_t j = i + 1; j < indices.size(); ++j) {
                size_t idx_j = indices[j];
                if (suppressed[idx_j]) continue;

                const auto &box_j = candidates[idx_j];
                cv::Point2f pt1_j = box_j.corners[0];
                cv::Point2f pt2_j = box_j.corners[2];
                double area_j = (pt2_j.x - pt1_j.x) * (pt2_j.y - pt1_j.y);

                double inter_x1 = std::max(pt1_i.x, pt1_j.x);
                double inter_y1 = std::max(pt1_i.y, pt1_j.y);
                double inter_x2 = std::min(pt2_i.x, pt2_j.x);
                double inter_y2 = std::min(pt2_i.y, pt2_j.y);
                double inter_w  = std::max(0.0, inter_x2 - inter_x1);
                double inter_h  = std::max(0.0, inter_y2 - inter_y1);
                double inter_area = inter_w * inter_h;
                double union_area = area_i + area_j - inter_area;

                double iou = (union_area > 0) ? (inter_area / union_area) : 0.0;
                if (iou > nms_iou_threshold_) {
                    suppressed[idx_j] = true;
                }
            }
        }

        return filtered;
    }

    /**
     * @brief 通过轮廓检测直接找到方形孔洞 (fallback)
     */
    std::vector<MatchResult> detectByContour(const cv::Mat &image) {
        std::vector<MatchResult> results;

        cv::Mat inverted;
        cv::bitwise_not(image, inverted);

        // 膨胀合并碎片化的孔洞区域 (三边环或不完整投影会导致孔洞断裂)
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::dilate(inverted, inverted, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(inverted, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty()) return results;

        const int img_w = image.cols, img_h = image.rows;
        const int border_margin = 2;  // 距图像边缘的最小距离(px)

        // 收集所有候选，按面积排序取前 N
        std::vector<MatchResult> candidates;

        for (const auto &contour : contours) {
            double area = cv::contourArea(contour);
            if (area < 100) continue;

            cv::Rect bbox = cv::boundingRect(contour);

            // 排除触碰图像边界的轮廓 (是投影图的黑边框，不是真正的环孔洞)
            if (bbox.x <= border_margin || bbox.y <= border_margin ||
                bbox.x + bbox.width  >= img_w - border_margin ||
                bbox.y + bbox.height >= img_h - border_margin) {
                ROS_DEBUG("[SquareRingMatch] 轮廓fallback: 丢弃边界轮廓 bbox=(%d,%d %dx%d)",
                          bbox.x, bbox.y, bbox.width, bbox.height);
                continue;
            }

            // 排除占图像面积>50%的轮廓
            double img_area = img_w * img_h;
            if (area > img_area * 0.5) continue;

            // 尺寸和比例检查
            if (bbox.width < expected_size_min_ || bbox.height < expected_size_min_)
                continue;
            if (bbox.width > expected_size_max_ || bbox.height > expected_size_max_)
                continue;

            float aspect = static_cast<float>(bbox.width) / static_cast<float>(bbox.height);
            if (aspect < 0.4f || aspect > 2.5f) continue;

            // 尝试 approxPolyDP (4顶点优先)
            std::vector<cv::Point> approx;
            double epsilon = 0.02 * cv::arcLength(contour, true);
            cv::approxPolyDP(contour, approx, epsilon, true);

            std::vector<cv::Point2f> corners_4;

            if (approx.size() == 4) {
                // 完美：4顶点四边形
                for (const auto &p : approx)
                    corners_4.push_back(cv::Point2f(p.x, p.y));
            } else {
                // 非标准形状 (3边或不规则) → 用 minAreaRect 推断4角
                cv::RotatedRect rrect = cv::minAreaRect(contour);
                cv::Point2f box[4];
                rrect.points(box);
                for (int k = 0; k < 4; ++k) corners_4.push_back(box[k]);

                // 验证 minAreaRect 结果合理
                cv::Rect rbbox = rrect.boundingRect();
                float r_aspect = static_cast<float>(rbbox.width) / static_cast<float>(rbbox.height);
                if (r_aspect < 0.3f || r_aspect > 3.0f) continue;
            }

            // 验证4角点都在图像范围内
            bool ok = true;
            for (const auto &c : corners_4) {
                if (c.x < 0 || c.y < 0 || c.x >= img_w || c.y >= img_h) { ok = false; break; }
            }
            if (!ok) continue;

            MatchResult result;
            result.corners = sortCornersClockwise(corners_4);
            result.matched = true;
            // 4顶点精确匹配分高，minAreaRect推断分低
            result.score = (approx.size() == 4) ? 0.70f : 0.55f;
            candidates.push_back(result);
        }

        // 按 bbox 面积降序，取前 max_rings_per_image_
        std::sort(candidates.begin(), candidates.end(),
                  [](const MatchResult &a, const MatchResult &b) {
                      double area_a = cv::boundingRect(a.corners).area();
                      double area_b = cv::boundingRect(b.corners).area();
                      return area_a > area_b;
                  });

        int count = std::min(static_cast<int>(candidates.size()), max_rings_per_image_);
        for (int i = 0; i < count; ++i) {
            results.push_back(candidates[i]);
        }

        return results;
    }

    /**
     * @brief 将4个点按顺时针排序 (左上→右上→右下→左下)
     */
    static std::vector<cv::Point2f> sortCornersClockwise(const std::vector<cv::Point2f> &pts) {
        std::vector<cv::Point2f> sorted = pts;
        std::sort(sorted.begin(), sorted.end(),
                  [](const cv::Point2f &a, const cv::Point2f &b) {
                      return (a.x + a.y) < (b.x + b.y);
                  });
        cv::Point2f tl = sorted[0];
        cv::Point2f br = sorted[3];

        cv::Point2f tr, bl;
        float max_diff = -1e9f, min_diff = 1e9f;
        for (const auto &p : pts) {
            float diff = p.x - p.y;
            if (diff > max_diff) { max_diff = diff; tr = p; }
            if (diff < min_diff) { min_diff = diff; bl = p; }
        }

        return {tl, tr, br, bl};
    }

    /**
     * @brief 环孔洞验证 — 排除实墙误匹配（使用原始模板匹配框）
     *
     * 问题：模板匹配可能把实墙也匹配为"环"（因为边框相似）。
     *       但真环的匹配区域中心应该是空的（激光无点），
     *       实墙匹配区域中心全是点。
     *
     * 做法：取原始匹配框（模板匹配的完整区域），验证中心区域
     *       黑像素占比 ≥ min_hole_ratio_（是空洞）。
     *
     * 注意：必须在轮廓细化之前调用，因为细化后角点指向内孔，
     *       内孔区域必然是纯黑的，无法区分真环和墙缝。
     *
     * @param projected_image 投影二值图
     * @param match_rect      原始模板匹配框 (图像坐标系)
     * @return true = 是真环，false = 疑似实墙
     */
    bool validateRingHole(const cv::Mat &projected_image,
                          const cv::Rect &match_rect) const {
        if (match_rect.width < 10 || match_rect.height < 10) return true;

        cv::Mat roi = projected_image(match_rect);

        // 中心区域：边界各缩 25%
        int margin_x = std::max(1, match_rect.width / 4);
        int margin_y = std::max(1, match_rect.height / 4);
        cv::Rect inner_rect(margin_x, margin_y,
                             match_rect.width  - 2 * margin_x,
                             match_rect.height - 2 * margin_y);

        if (inner_rect.width <= 0 || inner_rect.height <= 0) return true;

        cv::Mat inner = roi(inner_rect);
        int inner_total = inner.total();
        int inner_white = cv::countNonZero(inner);
        int inner_black = inner_total - inner_white;
        float hole_pct = static_cast<float>(inner_black) / inner_total;

        bool pass = (hole_pct >= min_hole_ratio_);

        ROS_DEBUG("[SquareRingMatch] 孔洞验证: center black=%.1f%% threshold=%.1f%% → %s",
                  hole_pct * 100.0f, min_hole_ratio_ * 100.0f,
                  pass ? "通过" : "拒绝(实墙)");

        return pass;
    }

    ros::NodeHandle pnh_;

    // 模板
    std::vector<std::string> template_paths_;
    std::vector<cv::Mat> templates_;
    std::vector<double> template_thresholds_;

    // 匹配参数
    float match_threshold_ = 0.65f;
    int match_method_      = cv::TM_CCOEFF_NORMED;

    // 多尺度 (默认 ±20%)
    double scale_x_min_  = 0.8, scale_x_max_  = 1.2, scale_x_step_  = 0.05;
    double scale_y_min_  = 0.8, scale_y_max_  = 1.2, scale_y_step_  = 0.05;

    // NMS
    double nms_iou_threshold_ = 0.3;

    // 形态学
    int morphology_kernel_size_ = 5;

    // 轮廓 fallback
    int expected_size_min_        = 20;
    int expected_size_max_        = 200;
    float contour_area_ratio_min_ = 0.3f;

    // 每张图最大环数
    int max_rings_per_image_ = 3;

    // 环孔洞验证：中心区域最少黑像素占比（低于此值视为实墙）
    float min_hole_ratio_ = 0.3f;
};

}  // namespace core
}  // namespace pcl_detection2

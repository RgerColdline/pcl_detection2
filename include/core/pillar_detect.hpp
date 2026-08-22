#pragma once

// ============================================================
// 文件: pillar_detect.hpp (pcl_detection2)
// 日期: 2026-08
// 功能: 柱子布局检测 -- 坐标判定为主，模板匹配兜底
//   - 坐标判定(主)：统计 4 个候选柱位(pillar_pos, odom 系)判定半径内
//     的 ROI 点数，A 侧(柱1)/B 侧(柱2)各出 1 根(同侧取点数高者)，
//     按 CASE_PILLARS 拼成 case 0~3；连续 confirm_frames 帧组合一致才检出
//   - 模板匹配(兜底)：同侧两候选都过阈值且点数接近(歧义)时，
//     用 templates/pillar_case_00~03.png 模板匹配裁定整局 case
//   - 输入：main.cpp 收到 /pcl_detection2/start_pillar_detect 后逐帧喂入
//     (cloud_source=fastlio 时为 /fastlio_map 降采样点云，camera_init 与 odom 重合)
//   - 输出：detect() 返回 detected=true 时 main.cpp 发布
//     /pcl_detection2/pillar_case_id (Int32) 并自动停止匹配
// case 命名规则（与 raicom_vision_laser/config/traverse_map.yaml、main_control
// 的 TRAV_CASE_PILLARS 一致，全队统一勿单独改）：
//   候选索引: 0=A左 1=A右 2=B左 3=B右（场系坐标见 config/pcl_detection2.yaml）
//   case0 = A左+B左   case1 = A左+B右   case2 = A右+B左   case3 = A右+B右
// ============================================================

#include "core/template_match/template_loader.h"

#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace pcl_detection2
{
namespace core
{

class PillarDetect
{
  public:
    using PointT      = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;
    using Ptr         = typename PointCloudT::Ptr;
    using ConstPtr    = typename PointCloudT::ConstPtr;

    struct Result
    {
        int case_id      = -1;    // 0~3，-1=未检出
        float score      = 0.0f;  // 坐标判定=两柱点数均值；模板兜底=合成分
        bool detected    = false;
        bool by_template = false; // true=本局由模板兜底裁定
    };

    struct PillarPosition
    {
        float x = 0.0f;
        float y = 0.0f;
    };

    // case 描述（日志用），与 CASE_PILLARS 顺序一致
    static constexpr const char* CASE_DESC[4] = {
        "A左+B左（两柱都在 x=2.7）",
        "A左+B右（(2.7,1.55)+(3.3,2.8)）",
        "A右+B左（(3.3,1.55)+(2.7,2.8)）",
        "A右+B右（两柱都在 x=3.3）"
    };

    PillarDetect() = default;

    // ==================== 初始化（读 yaml 参数 + 加载兜底模板） ====================
    void init(ros::NodeHandle &pnh) {
        // ROI 参数（z 带裁剪，坐标判定与模板兜底共用）
        pnh.param("pillar_roi/x_min", roi_x_min_, -3.5f);
        pnh.param("pillar_roi/x_max", roi_x_max_, -1.2f);
        pnh.param("pillar_roi/y_min", roi_y_min_, -3.15f);
        pnh.param("pillar_roi/y_max", roi_y_max_, 0.5f);
        pnh.param("pillar_roi/z_min", roi_z_min_, 0.3f);
        pnh.param("pillar_roi/z_max", roi_z_max_, 1.5f);
        pnh.param("pillar_roi/resolution", resolution_, 0.05f);
        pnh.param("pillar_roi/match_threshold", match_threshold_, 0.5f);
        pnh.param("pillar_roi/proj_upscale", proj_upscale_, 1);
        pnh.param("pillar_roi/tpl_upscale", tpl_upscale_, 1);
        pnh.param("pillar_roi/density_weight", density_weight_, 0.3f);

        // 坐标判定参数（主判定）
        pnh.param("pillar_coord/radius", coord_radius_, 0.25f);
        pnh.param("pillar_coord/min_points", coord_min_points_, 5);
        pnh.param("pillar_coord/confirm_frames", confirm_frames_, 3);
        pnh.param("pillar_coord/ambiguous_ratio", ambiguous_ratio_, 0.8f);

        // 计算投影图尺寸（+0.5f 防浮点截断，如 2.3/0.05=45.999->46）
        img_cols_ = std::max(1, static_cast<int>((roi_x_max_ - roi_x_min_) / resolution_ + 0.5f));
        img_rows_ = std::max(1, static_cast<int>((roi_y_max_ - roi_y_min_) / resolution_ + 0.5f));

        // 4 个候选柱位（odom 系）
        auto loadPillarPos = [&](const std::string &key, PillarPosition &pos) {
            std::vector<double> v;
            if (pnh.getParam("pillar_pos/" + key, v) && v.size() >= 2) {
                pos.x = static_cast<float>(v[0]);
                pos.y = static_cast<float>(v[1]);
            }
        };
        loadPillarPos("p1_pos1", pillar_pos_[0]);  // A左
        loadPillarPos("p1_pos2", pillar_pos_[1]);  // A右
        loadPillarPos("p2_pos1", pillar_pos_[2]);  // B左
        loadPillarPos("p2_pos2", pillar_pos_[3]);  // B右

        // 兜底模板（加载失败仅告警不致命：坐标判定正常时用不到）
        TemplateLoader loader("pcl_detection2");
        templates_ = loader.load({"pillar_case_00.png", "pillar_case_01.png",
                                  "pillar_case_02.png", "pillar_case_03.png"});

        ROS_INFO("[PillarDetect] 初始化: 坐标判定 radius=%.2fm min_points=%d confirm=%d帧 "
                 "歧义比=%.2f | ROI x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f] | 兜底模板=%zu",
                 coord_radius_, coord_min_points_, confirm_frames_, ambiguous_ratio_,
                 roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_,
                 templates_.size());
    }

    // 重新触发检测时清空连续确认状态（main.cpp 的 start 回调里调用）
    void reset() {
        last_case_   = -1;
        case_streak_ = 0;
    }

    // ==================== 逐帧检测 ====================
    Result detect(const ConstPtr &cloud) {
        Result result;
        if (!cloud || cloud->empty()) {
            ROS_WARN_THROTTLE(5.0, "[Pillar] 输入点云为空，无法检测");
            return result;
        }

        // ---- Step 1: ROI 裁剪（z 带内，覆盖 4 个候选柱位）----
        Ptr roi_cloud(new PointCloudT);
        {
            pcl::CropBox<PointT> crop;
            crop.setMin(Eigen::Vector4f(roi_x_min_, roi_y_min_, roi_z_min_, 1.0f));
            crop.setMax(Eigen::Vector4f(roi_x_max_, roi_y_max_, roi_z_max_, 1.0f));
            crop.setInputCloud(cloud);
            crop.filter(*roi_cloud);
        }
        if (roi_cloud->empty()) {
            ROS_WARN_THROTTLE(5.0, "[Pillar] ROI 裁剪后无点（地图未覆盖柱区或坐标系不符）");
            return result;
        }

        // ---- Step 2: 统计 4 个候选柱位半径内点数 ----
        int counts[4] = {0, 0, 0, 0};
        countNearPillars(*roi_cloud, counts);

        // ---- Step 3: Z 平面投影二值图（模板兜底 + dump 用）----
        cv::Mat proj_img = projectToImage(*roi_cloud);
        cv::Mat kernel   = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::dilate(proj_img, proj_img, kernel);

        // ---- Step 4: 坐标判定，A/B 侧各出 1 根 ----
        int a_idx = -1, b_idx = -1;
        bool ambiguous = false;
        int frame_case = decideByCoord(counts, a_idx, b_idx, ambiguous);

        // ---- Step 5: 坐标歧义 -> 模板兜底裁定 ----
        float tpl_score  = 0.0f;
        bool by_template = false;
        if (ambiguous) {
            frame_case  = decideByTemplate(proj_img, counts, tpl_score);
            by_template = (frame_case >= 0);
        }

        // ---- Step 6: 连续 confirm_frames 帧组合一致才检出（防单帧噪声）----
        if (frame_case >= 0) {
            if (frame_case == last_case_) {
                ++case_streak_;
            }
            else {
                last_case_   = frame_case;
                case_streak_ = 1;
            }

            if (case_streak_ >= confirm_frames_) {
                result.case_id     = frame_case;
                result.detected    = true;
                result.by_template = by_template;
                result.score       = by_template
                                         ? tpl_score
                                         : 0.5f * (counts[a_idx] + counts[b_idx]);
            }
        }
        else {
            case_streak_ = 0;
        }

        ROS_INFO_THROTTLE(1.0,
                          "[Pillar] ROI点=%zu A左=%d A右=%d B左=%d B右=%d (阈值%d) "
                          "本帧case=%d 连续%d/%d%s",
                          roi_cloud->size(), counts[0], counts[1], counts[2], counts[3],
                          coord_min_points_, frame_case, case_streak_, confirm_frames_,
                          by_template ? " [模板兜底]" : "");

        if (result.detected) {
            ROS_INFO("[Pillar] ★ 检测成功 case=%d（%s）A左=%d A右=%d B左=%d B右=%d "
                     "连续%d帧 %s score=%.1f",
                     result.case_id, CASE_DESC[result.case_id], counts[0], counts[1],
                     counts[2], counts[3], case_streak_,
                     result.by_template ? "模板兜底" : "坐标判定", result.score);
        }

        // ---- dump 投影图/点云到 temp（前 3 个窗口，调参用）----
        dumpProjection(proj_img, *roi_cloud, *cloud, frame_case, counts);

        return result;
    }

  private:
    // ==================== 统计候选柱位半径内点数 ====================
    // pillar_pos_ 映射: [0]=A左 [1]=A右 [2]=B左 [3]=B右（XY 平面距离，z 已在 ROI 带内）
    void countNearPillars(const PointCloudT &roi_cloud, int counts[4]) const {
        const float r2 = coord_radius_ * coord_radius_;
        for (const auto &pt : roi_cloud.points) {
            for (int i = 0; i < 4; ++i) {
                const float dx = pt.x - pillar_pos_[i].x;
                const float dy = pt.y - pillar_pos_[i].y;
                if (dx * dx + dy * dy <= r2) {
                    ++counts[i];
                }
            }
        }
    }

    // ==================== 坐标判定：A/B 侧各出 1 根拼 case ====================
    // 返回 case 0~3；-1 = 本帧无结论（某侧两候选都不过阈值：柱区未扫够，继续等）
    // ambiguous=true 表示同侧两候选点数接近分不清，外层走模板兜底
    int decideByCoord(const int counts[4], int &a_idx, int &b_idx, bool &ambiguous) const {
        a_idx     = pickSide(0, 1, counts, ambiguous);
        b_idx     = pickSide(2, 3, counts, ambiguous);
        if (a_idx < 0 || b_idx < 0) return -1;
        // (A候选, B候选) -> case：case0=(0,2) case1=(0,3) case2=(1,2) case3=(1,3)
        return a_idx * 2 + (b_idx - 2);
    }

    // 单侧挑选：返回选中的候选索引
    //   >=0 = 该侧唯一过阈值者，或两候选都过阈值但点数差距明显取高者
    //   -1  = 该侧两候选都不过阈值（继续等）
    //   -2  = 两候选都过阈值且点数接近（歧义，置 ambiguous）
    int pickSide(int i, int j, const int counts[4], bool &ambiguous) const {
        const bool pi = counts[i] >= coord_min_points_;
        const bool pj = counts[j] >= coord_min_points_;
        if (pi && pj) {
            const int lo = std::min(counts[i], counts[j]);
            const int hi = std::max(counts[i], counts[j]);
            if (hi > 0 && static_cast<float>(lo) >= ambiguous_ratio_ * hi) {
                ambiguous = true;
                return -2;
            }
            return counts[i] >= counts[j] ? i : j;
        }
        if (pi) return i;
        if (pj) return j;
        return -1;
    }

    // ==================== Z 平面投影二值图（模板兜底/dump 用） ====================
    cv::Mat projectToImage(const PointCloudT &roi_cloud) const {
        cv::Mat img    = cv::Mat::zeros(img_rows_, img_cols_, CV_8UC1);
        const float inv_res = 1.0f / resolution_;
        for (const auto &pt : roi_cloud.points) {
            const int col = static_cast<int>((pt.x - roi_x_min_) * inv_res);
            const int row = static_cast<int>((pt.y - roi_y_min_) * inv_res);
            if (col >= 0 && col < img_cols_ && row >= 0 && row < img_rows_) {
                img.at<uint8_t>(row, col) = 255;
            }
        }
        return img;
    }

    // ==================== 模板兜底裁定（仅坐标歧义时调用） ====================
    // 合成分 = 模板分 × (1 + density_weight × 密度分)，需 > match_threshold
    // 密度分复用坐标判定的 counts（case 两柱位点数之和归一化）
    // 返回 case 0~3，无置信结果返回 -1
    int decideByTemplate(const cv::Mat &proj_img, const int counts[4], float &score) const {
        if (templates_.empty()) {
            ROS_WARN_THROTTLE(5.0, "[Pillar] 坐标判定歧义且模板未加载，本帧放弃");
            return -1;
        }

        // 投影图/模板放大后二值化
        cv::Mat proj_scaled;
        cv::resize(proj_img, proj_scaled, cv::Size(), proj_upscale_, proj_upscale_,
                   cv::INTER_NEAREST);
        cv::threshold(proj_scaled, proj_scaled, 127, 255, cv::THRESH_BINARY);

        float match_scores[4] = {0, 0, 0, 0};
        for (size_t t = 0; t < templates_.size() && t < 4; ++t) {
            cv::Mat tpl_scaled;
            cv::resize(templates_[t], tpl_scaled, cv::Size(), tpl_upscale_, tpl_upscale_,
                       cv::INTER_NEAREST);
            cv::threshold(tpl_scaled, tpl_scaled, 127, 255, cv::THRESH_BINARY);

            cv::Mat match_res;
            cv::matchTemplate(proj_scaled, tpl_scaled, match_res, cv::TM_CCOEFF_NORMED);
            double max_val;
            cv::minMaxLoc(match_res, nullptr, &max_val);
            match_scores[t] = static_cast<float>(max_val);
        }

        // 密度分：case->柱位 case0={0,2} case1={0,3} case2={1,2} case3={1,3}
        static const int CASE_PILLARS[4][2] = {{0, 2}, {0, 3}, {1, 2}, {1, 3}};
        int case_counts[4];
        int max_count = 0;
        for (int c = 0; c < 4; ++c) {
            case_counts[c] = counts[CASE_PILLARS[c][0]] + counts[CASE_PILLARS[c][1]];
            max_count      = std::max(max_count, case_counts[c]);
        }
        const float norm = static_cast<float>(std::max(max_count, 1));

        int best_id    = -1;
        float best     = match_threshold_;
        for (int t = 0; t < 4; ++t) {
            const float final_score =
                match_scores[t] * (1.0f + density_weight_ * case_counts[t] / norm);
            if (final_score > best) {
                best     = final_score;
                best_id  = t;
            }
        }

        ROS_INFO_THROTTLE(1.0,
                          "[Pillar] 模板兜底: t0=%.3f t1=%.3f t2=%.3f t3=%.3f -> case=%d (%.3f)",
                          match_scores[0], match_scores[1], match_scores[2], match_scores[3],
                          best_id, best);

        if (best_id >= 0) score = best;
        return best_id;
    }

    // ==================== dump 投影图/点云（前 3 个窗口，调参用） ====================
    void dumpProjection(const cv::Mat &proj_img, const PointCloudT &roi_cloud,
                        const PointCloudT &full_cloud, int frame_case, const int counts[4]) {
        static int dump_count  = 0;
        static int frame_count = 0;
        static std::string save_dir;

        if (dump_count >= 3) return;

        ++frame_count;
        // 每5帧保存一次，共3次
        if (frame_count < 3 || (frame_count % 5) != 0) return;

        if (save_dir.empty()) {
            save_dir = ros::package::getPath("pcl_detection2") + "/temp";
            mkdir(save_dir.c_str(), 0755);
        }

        // 转BGR，画判定结果覆盖
        cv::Mat vis;
        cv::cvtColor(proj_img, vis, cv::COLOR_GRAY2BGR);

        // 在 4 个候选柱位画判定圈：有点=红圈，无点=蓝圈
        const float inv_res = 1.0f / resolution_;
        for (int i = 0; i < 4; ++i) {
            const int col = static_cast<int>((pillar_pos_[i].x - roi_x_min_) * inv_res);
            const int row = static_cast<int>((pillar_pos_[i].y - roi_y_min_) * inv_res);
            const cv::Scalar color =
                (counts[i] >= coord_min_points_) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            cv::circle(vis, cv::Point(col, row),
                       static_cast<int>(coord_radius_ * inv_res), color, 1);
        }

        std::ostringstream fname;
        fname << save_dir << "/pillar_dump_" << dump_count << "_f" << frame_count << "_case"
              << frame_case << ".png";
        cv::imwrite(fname.str(), vis);

        // 同步保存ROI点云和全量地图点云为PCD
        std::ostringstream pcd_fname;
        pcd_fname << save_dir << "/pillar_dump_" << dump_count << "_f" << frame_count << ".pcd";
        pcl::io::savePCDFileBinary(pcd_fname.str(), roi_cloud);
        std::ostringstream full_fname;
        full_fname << save_dir << "/pillar_dump_" << dump_count << "_f" << frame_count
                   << "_full.pcd";
        pcl::io::savePCDFileBinary(full_fname.str(), full_cloud);
        ROS_INFO("[Pillar] dump #%d: %s + %s (%zu pts) + %s (%zu pts)", dump_count + 1,
                 fname.str().c_str(), pcd_fname.str().c_str(), roi_cloud.size(),
                 full_fname.str().c_str(), full_cloud.size());
        ++dump_count;
    }

    // ---- 参数（config/pcl_detection2.yaml）----
    float roi_x_min_ = -3.5f, roi_x_max_ = -1.2f;
    float roi_y_min_ = -3.15f, roi_y_max_ = 0.5f;
    float roi_z_min_ = 0.3f, roi_z_max_ = 1.5f;
    float resolution_      = 0.05f;  // 投影分辨率 m/px
    float match_threshold_ = 0.5f;   // 模板兜底合成分阈值
    int proj_upscale_      = 1;      // 投影图放大倍数（>tpl 给匹配留滑动空间）
    int tpl_upscale_       = 1;      // 模板放大倍数
    float density_weight_  = 0.3f;   // 兜底合成分中密度分权重

    float coord_radius_    = 0.25f;  // 柱位判定半径 (m)
    int coord_min_points_  = 5;      // 半径内点数 >= 此值判"有柱"
    int confirm_frames_    = 3;      // 连续一致帧数
    float ambiguous_ratio_ = 0.8f;   // 同侧点数比 >= 此值 -> 歧义走模板

    int img_cols_ = 0, img_rows_ = 0;  // 投影图尺寸（init 由 ROI/resolution 算出）
    std::vector<cv::Mat> templates_;   // 兜底模板 pillar_case_00~03.png

    PillarPosition pillar_pos_[4];     // [0]=A左 [1]=A右 [2]=B左 [3]=B右（odom 系）

    // ---- 连续确认状态 ----
    int last_case_   = -1;             // 上一帧判定 case
    int case_streak_ = 0;              // 连续一致帧数
};

}  // namespace core
}  // namespace pcl_detection2

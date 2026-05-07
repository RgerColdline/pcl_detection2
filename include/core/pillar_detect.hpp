#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "core/template_match/template_loader.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sstream>

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

namespace pcl_detection2
{
namespace core
{

/**
 * @brief 柱子位置组合检测
 *
 * 将ROI点云投影到Z平面为二值图，与4个预先生成的柱子位置模板匹配，
 * 返回最佳匹配的配置编号（0-3）。
 *
 * 模板文件: templates/pillar_case_00~03.png (26x53, 白圆点=柱子位置)
 */
class PillarDetect
{
  public:
    using PointT      = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;
    using Ptr         = typename PointCloudT::Ptr;
    using ConstPtr    = typename PointCloudT::ConstPtr;

    struct Result
    {
        int case_id   = -1;
        float score   = 0.0f;
        bool detected = false;
    };

    PillarDetect() = default;

    void init(ros::NodeHandle &pnh) {
        // ROI参数
        pnh.param("pillar_roi/x_min", roi_x_min_, -3.0f);
        pnh.param("pillar_roi/x_max", roi_x_max_, -1.7f);
        pnh.param("pillar_roi/y_min", roi_y_min_, -2.65f);
        pnh.param("pillar_roi/y_max", roi_y_max_, 0.0f);
        pnh.param("pillar_roi/z_min", roi_z_min_, 0.3f);
        pnh.param("pillar_roi/z_max", roi_z_max_, 1.5f);
        pnh.param("pillar_roi/resolution", resolution_, 0.05f);
        pnh.param("pillar_roi/match_threshold", match_threshold_, 0.3f);

        // 计算投影图尺寸
        img_cols_ = std::max(1, static_cast<int>((roi_x_max_ - roi_x_min_) / resolution_));
        img_rows_ = std::max(1, static_cast<int>((roi_y_max_ - roi_y_min_) / resolution_));

        // 从文件加载4个模板
        TemplateLoader loader("pcl_detection2");
        std::vector<std::string> tpl_paths = {
            "pillar_case_00.png", "pillar_case_01.png",
            "pillar_case_02.png", "pillar_case_03.png"
        };
        templates_ = loader.load(tpl_paths);

        ROS_INFO("[PillarDetect] 初始化完成: ROI=%.1f~%.1f x %.1f~%.1f, 图=%dx%d, 模板=%zu",
                 roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_,
                 img_cols_, img_rows_, templates_.size());
    }

    Result detect(const ConstPtr &cloud) {
        Result result;
        if (!cloud || cloud->empty() || templates_.empty()) return result;

        // Step 1: ROI裁剪
        pcl::CropBox<PointT> crop;
        crop.setMin(Eigen::Vector4f(roi_x_min_, roi_y_min_, roi_z_min_, 1.0f));
        crop.setMax(Eigen::Vector4f(roi_x_max_, roi_y_max_, roi_z_max_, 1.0f));
        crop.setInputCloud(cloud);
        Ptr roi_cloud(new PointCloudT);
        crop.filter(*roi_cloud);
        if (roi_cloud->empty()) return result;

        // Step 2: Z平面投影
        cv::Mat proj_img = cv::Mat::zeros(img_rows_, img_cols_, CV_8UC1);
        float inv_res = 1.0f / resolution_;
        for (const auto &pt : roi_cloud->points) {
            int col = static_cast<int>((pt.x - roi_x_min_) * inv_res);
            int row = static_cast<int>((pt.y - roi_y_min_) * inv_res);
            if (col >= 0 && col < img_cols_ && row >= 0 && row < img_rows_) {
                proj_img.at<uint8_t>(row, col) = 255;
            }
        }

        // Step 3: 形态学膨胀
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::dilate(proj_img, proj_img, kernel);

        // Step 4: 与4个模板匹配
        float best_score = match_threshold_;
        int best_id = -1;
        std::string score_detail;
        for (size_t t = 0; t < templates_.size(); ++t) {
            cv::Mat match_res;
            cv::matchTemplate(proj_img, templates_[t], match_res, cv::TM_CCOEFF_NORMED);
            float score = match_res.at<float>(0, 0);
            char buf[32];
            snprintf(buf, sizeof(buf), " t%zu=%.3f", t, score);
            score_detail += buf;
            if (score > best_score) { best_score = score; best_id = static_cast<int>(t); }
        }

        ROS_INFO_THROTTLE(1.0,
            "[Pillar] ROI点=%zu 投影白点=%d/%d 图=%dx%d 模板=%zu 阈值=%.2f 得分:%s",
            roi_cloud->size(), cv::countNonZero(proj_img),
            img_rows_ * img_cols_, img_cols_, img_rows_,
            templates_.size(), match_threshold_, score_detail.c_str());

        if (best_id >= 0) {
            result.case_id  = best_id;
            result.score    = best_score;
            result.detected = true;
            ROS_INFO("[Pillar] ★ 检测成功 case=%d score=%.3f", best_id, best_score);
        }

        // ---- dump投影图到temp ----
        dumpProjection(proj_img, best_id, best_score);

        return result;
    }

  private:
    void dumpProjection(const cv::Mat &proj_img, int best_id, float best_score) {
        static int dump_count = 0;
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

        // 转BGR，画模板覆盖
        cv::Mat vis;
        cv::cvtColor(proj_img, vis, cv::COLOR_GRAY2BGR);

        // 高亮最佳匹配模板
        if (best_id >= 0 && best_id < static_cast<int>(templates_.size())) {
            for (int r = 0; r < vis.rows; ++r) {
                for (int c = 0; c < vis.cols; ++c) {
                    if (templates_[best_id].at<uint8_t>(r, c) == 255) {
                        if (proj_img.at<uint8_t>(r, c) == 255) {
                            vis.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 255, 0);  // 命中=绿
                        } else {
                            vis.at<cv::Vec3b>(r, c) = cv::Vec3b(255, 0, 0);  // 模板有投影无=蓝
                        }
                    }
                }
            }
        }

        std::ostringstream fname;
        fname << save_dir << "/pillar_dump_" << dump_count
              << "_f" << frame_count
              << "_case" << best_id << "_s" << int(best_score * 100) << ".png";
        cv::imwrite(fname.str(), vis);
        ++dump_count;
        ROS_INFO("[Pillar] dump #%d: %s", dump_count, fname.str().c_str());
    }

    float roi_x_min_ = -3.0f, roi_x_max_ = -1.7f;
    float roi_y_min_ = -2.65f, roi_y_max_ = 0.0f;
    float roi_z_min_ = 0.3f, roi_z_max_ = 1.5f;
    float resolution_ = 0.05f;
    float match_threshold_ = 0.3f;
    int img_cols_ = 0, img_rows_ = 0;
    std::vector<cv::Mat> templates_;
};

}  // namespace core
}  // namespace pcl_detection2

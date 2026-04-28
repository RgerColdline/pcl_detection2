#pragma once

/**
 * @file template_loader.h
 * @brief 通用模板图片加载器 — 可移植路径解析
 *
 * 使用方式：
 *   #include "core/template_match/template_loader.h"
 *   auto loader = pcl_detection2::core::TemplateLoader("pcl_detection2");
 *   auto imgs = loader.load({"obs1.png", "obs2.png"});
 *   // 或只加载单张
 *   cv::Mat img = loader.loadOne("obs1.png");
 *
 * 路径规则：
 *   - 相对路径 (不以 / 开头) → 相对 <pkg_path>/templates/
 *   - 绝对路径 (以 / 开头)    → 直接加载 (向后兼容)
 *
 * catkin_make 即可用，无需改路径，别人下载包后开箱即用。
 */

#include <ros/package.h>
#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <string>
#include <vector>

namespace pcl_detection2
{
namespace core
{

class TemplateLoader
{
public:
    /**
     * @param package_name 模板所在 ROS 包名 (如 "pcl_detection2")
     * @param subdir       包内的模板子目录 (默认 "templates")
     */
    explicit TemplateLoader(const std::string &package_name,
                            const std::string &subdir = "templates")
        : package_name_(package_name), subdir_(subdir) {
        resolvePackageRoot();
    }

    /**
     * @brief 加载单个模板图片
     * @param rel_path 相对或绝对路径
     * @return cv::Mat 灰度图，失败返回空
     */
    cv::Mat loadOne(const std::string &rel_path) {
        if (rel_path.empty()) return cv::Mat();

        std::string full = resolvePath(rel_path);
        cv::Mat img      = cv::imread(full, cv::IMREAD_GRAYSCALE);

        if (img.empty()) {
            ROS_WARN("[TemplateLoader] 无法加载: %s", full.c_str());
        }
        return img;
    }

    /**
     * @brief 批量加载模板
     * @param paths 模板路径列表 (相对或绝对)
     * @return 成功加载的 cv::Mat 灰度图列表
     */
    std::vector<cv::Mat> load(const std::vector<std::string> &paths) {
        std::vector<cv::Mat> result;
        result.reserve(paths.size());

        for (const auto &p : paths) {
            cv::Mat img = loadOne(p);
            if (!img.empty()) {
                ROS_INFO("[TemplateLoader] 加载模板: %s (%dx%d)",
                         resolvePath(p).c_str(), img.cols, img.rows);
                result.push_back(img);
            }
        }

        if (result.empty() && !paths.empty()) {
            ROS_WARN("[TemplateLoader] 无有效模板，请检查路径和文件");
        }

        return result;
    }

    /** @brief 包根路径 */
    const std::string &packageRoot() const { return package_root_; }

    /**
     * @brief 解析路径：相对→绝对
     */
    std::string resolvePath(const std::string &input) const {
        if (!input.empty() && input[0] == '/') {
            return input;                         // 绝对路径
        }
        return package_root_ + "/" + subdir_ + "/" + input;
    }

private:
    void resolvePackageRoot() {
        try {
            package_root_ = ros::package::getPath(package_name_);
        }
        catch (const ros::Exception &e) {
            ROS_ERROR("[TemplateLoader] 无法获取包 '%s' 路径: %s",
                      package_name_.c_str(), e.what());
            package_root_.clear();
        }
    }

    std::string package_name_;
    std::string subdir_;
    std::string package_root_;
};

}  // namespace core
}  // namespace pcl_detection2

#pragma once

#include "plane.hpp"
#include "plane_project.hpp"

#include <Eigen/Dense>
#include <vector>

namespace pcl_detection2
{
namespace core
{
namespace plane
{

/**
 * @brief 2D 像素点反投影到 3D 平面
 *
 * 接收像素坐标、投影矩阵、平面参数，输出 3D 世界坐标点
 */
class PointBackProject
{
  public:
    using PointT      = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;

    PointBackProject() = default;

    /**
     * @brief 将一组 2D 像素点反投影到 3D 平面
     *
     * @param pixel_points   像素坐标 (col, row)
     * @param project_result PlaneProject 的输出 (含投影/变换矩阵)
     * @param plane          平面参数（只需要 coefficients）
     * @return 3D 点坐标 (在世界坐标系下)
     */
    std::vector<Eigen::Vector3f> backProject(
        const std::vector<cv::Point2f> &pixel_points,
        const PlaneProject::ProjectResult &project_result,
        const PlaneParams &plane) const
    {
        std::vector<Eigen::Vector3f> results;
        results.reserve(pixel_points.size());

        // 获取 plane 的系数
        const Eigen::Vector3f plane_normal = plane.coefficients.head<3>();
        const float plane_d = plane.coefficients[3];

        // 变换矩阵的逆（平面坐标系 → 世界坐标系）
        const Eigen::Matrix4f tf_plane_inv = project_result.tf_plane.inverse();

        for (const auto &pix : pixel_points) {
            // Step 1: 像素 → 平面坐标系 (x, y)
            // col = tf_2d(0,0)*x + tf_2d(0,2)  =>  x = (col - tf_2d(0,2)) / tf_2d(0,0)
            // row = tf_2d(1,1)*y + tf_2d(1,2)  =>  y = (row - tf_2d(1,2)) / tf_2d(1,1)
            float x_plane = (pix.x - project_result.tf_2d(0, 2)) / project_result.tf_2d(0, 0);
            float y_plane = (pix.y - project_result.tf_2d(1, 2)) / project_result.tf_2d(1, 1);
            float z_plane = 0.0f;  // 在平面坐标系中 Z=0

            Eigen::Vector4f pt_plane(x_plane, y_plane, z_plane, 1.0f);

            // Step 2: 平面坐标系 → 世界坐标系
            Eigen::Vector4f pt_world = tf_plane_inv * pt_plane;

            // Step 3: 将点投影到原始平面上（修正数值误差）
            Eigen::Vector3f pt = pt_world.head<3>();
            float dist_to_plane  = plane_normal.dot(pt) + plane_d;
            pt -= dist_to_plane * plane_normal;

            results.push_back(pt);
        }

        return results;
    }

    /**
     * @brief 反投影并计算中心点、前后点
     *
     * @param pixel_points    4个角点 (像素坐标)
     * @param project_result  PlaneProject 输出
     * @param plane           平面参数
     * @param drone_position  无人机当前位置
     * @return {corners_3d, center, front, back}
     */
    struct RingPoints3D
    {
        std::vector<Eigen::Vector3f> corners;   // 4个角点
        Eigen::Vector3f center;                  // 中心点
        Eigen::Vector3f front_point;             // 前方点
        Eigen::Vector3f back_point;              // 后方点
    };

    RingPoints3D computeRingPoints(
        const std::vector<cv::Point2f> &pixel_points,
        const PlaneProject::ProjectResult &project_result,
        const PlaneParams &plane,
        const Eigen::Vector3f &drone_position) const
    {
        RingPoints3D result;

        result.corners = backProject(pixel_points, project_result, plane);

        if (result.corners.size() != 4) return result;

        // 计算中心点 (4个角点均值)
        result.center = Eigen::Vector3f::Zero();
        for (const auto &c : result.corners) {
            result.center += c;
        }
        result.center /= 4.0f;

        // 纠正中心到平面
        float d = plane.coefficients.head<3>().dot(result.center) + plane.coefficients[3];
        result.center -= d * plane.coefficients.head<3>();

        // 前方点 = 中心 + 法向量 * 距离，后方点 = 中心 - 法向量 * 距离
        float offset = 0.5f;  // 默认偏移 0.5m
        result.front_point = result.center + plane.normal * offset;
        result.back_point  = result.center - plane.normal * offset;

        // 根据无人机位置选择前后：前方点靠近无人机
        float dist_front = (result.front_point - drone_position).norm();
        float dist_back  = (result.back_point - drone_position).norm();

        if (dist_back < dist_front) {
            std::swap(result.front_point, result.back_point);
        }

        return result;
    }

};

}  // namespace plane
}  // namespace core
}  // namespace pcl_detection2

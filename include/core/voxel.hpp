#pragma once

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <limits>

namespace pcl_detection2
{
namespace core
{

/**
 * @brief 统一体素滤波器，支持多种 intensity 处理模式
 *
 * 完全重写 applyFilter，自己控制体素化逻辑
 */
template <typename PointT> class VoxelGridUnified
{
  public:
    using PointCloudT         = pcl::PointCloud<PointT>;
    using PointCloudPtrT      = typename PointCloudT::Ptr;
    using PointCloudConstPtrT = typename PointCloudT::ConstPtr;

    /**
     * @brief intensity 处理模式
     */
    enum class Mode {
        AVERAGE,     ///< 原生：平均（用于初次降采样）
        ACCUMULATE,  ///< 累积：概率累积模型（用于地图积累）
        MAX          ///< 膨胀：取最大值（用于膨胀后处理）
    };

    /**
     * @brief 构造函数
     * @param leaf_size 体素大小
     * @param r_decay_coeff 保留系数
     */
    VoxelGridUnified(float leaf_size = 0.05f, float r_decay_coeff = 0.9f)
        : leaf_size_(leaf_size), r_decay_coeff_(r_decay_coeff), mode_(Mode::AVERAGE) {}

    /**
     * @brief 设置体素大小
     */
    void setLeafSize(float leaf_size) { leaf_size_ = leaf_size; }

    /**
     * @brief 设置衰减系数（仅 ACCUMULATE 模式使用）
     */
    void setDecayCoefficient(float coeff) { r_decay_coeff_ = coeff; }

    /**
     * @brief 设置处理模式
     */
    void setMode(Mode mode) { mode_ = mode; }

    /**
     * @brief 设置输入点云
     */
    void setInputCloud(const PointCloudConstPtrT input) { input_ = input; }

    /**
     * @brief 执行滤波（模式作为参数传入）
     * @param input 输入点云
     * @param output 输出点云
     * @param mode 处理模式
     */
    void filterCloud(const PointCloudConstPtrT input, PointCloudPtrT output, Mode mode) {
        mode_  = mode;
        input_ = input;
        filter(*output);
    }

    /**
     * @brief 执行滤波（使用当前设置的模式）
     */
    void filter(PointCloudT &output) {
        if (!input_ || input_->empty()) {
            output.clear();
            return;
        }

        // 1. 计算点云边界
        Eigen::Vector4f min_p, max_p;
        pcl::getMinMax3D(*input_, min_p, max_p);

        // 2. 计算体素网格参数
        inverse_leaf_size_[0] = 1.0f / leaf_size_;
        inverse_leaf_size_[1] = 1.0f / leaf_size_;
        inverse_leaf_size_[2] = 1.0f / leaf_size_;

        min_b_[0]             = static_cast<int>(std::floor(min_p[0] * inverse_leaf_size_[0]));
        max_b_[0]             = static_cast<int>(std::floor(max_p[0] * inverse_leaf_size_[0]));
        min_b_[1]             = static_cast<int>(std::floor(min_p[1] * inverse_leaf_size_[1]));
        max_b_[1]             = static_cast<int>(std::floor(max_p[1] * inverse_leaf_size_[1]));
        min_b_[2]             = static_cast<int>(std::floor(min_p[2] * inverse_leaf_size_[2]));
        max_b_[2]             = static_cast<int>(std::floor(max_p[2] * inverse_leaf_size_[2]));

        div_b_                = max_b_ - min_b_ + Eigen::Vector4i::Ones();
        divb_mul_             = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

        // 3. 构建体素索引
        struct VoxelPointIndex
        {
            unsigned int idx;          // 体素索引
            unsigned int point_index;  // 点云中的索引
        };
        std::vector<VoxelPointIndex> index_vector;
        index_vector.reserve(input_->size());

        for (size_t i = 0; i < input_->size(); ++i) {
            const PointT &pt = input_->points[i];
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

            int ijk0 = static_cast<int>(std::floor(pt.x * inverse_leaf_size_[0]) - min_b_[0]);
            int ijk1 = static_cast<int>(std::floor(pt.y * inverse_leaf_size_[1]) - min_b_[1]);
            int ijk2 = static_cast<int>(std::floor(pt.z * inverse_leaf_size_[2]) - min_b_[2]);

            int idx  = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
            index_vector.push_back({static_cast<unsigned int>(idx), static_cast<unsigned int>(i)});
        }

        // 4. 排序：同一体素的点排在一起
        std::sort(index_vector.begin(), index_vector.end(),
                  [](const VoxelPointIndex &a, const VoxelPointIndex &b) { return a.idx < b.idx; });

        // 5. 统计每个体素的起止位置
        std::vector<std::pair<unsigned int, unsigned int>> first_and_last_indices_vector;
        unsigned int index = 0;
        while (index < index_vector.size()) {
            unsigned int i = index + 1;
            while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx) ++i;
            first_and_last_indices_vector.emplace_back(index, i);
            index = i;
        }

        // 6. 计算每个体素的输出点
        output.points.resize(first_and_last_indices_vector.size());
        output.width          = static_cast<std::uint32_t>(first_and_last_indices_vector.size());
        output.height         = 1;
        output.is_dense       = true;
        output.header         = input_->header;                    // 复制 header
        output.sensor_origin_ = input_->sensor_origin_;            // 复制传感器原点
        output.sensor_orientation_ = input_->sensor_orientation_;  // 复制传感器方向

        size_t output_idx          = 0;
        for (const auto &cp : first_and_last_indices_vector) {
            unsigned int first_index = cp.first;
            unsigned int last_index  = cp.second;
            unsigned int count       = last_index - first_index;

            // 计算坐标平均值
            float x = 0, y = 0, z = 0;
            for (unsigned int i = first_index; i < last_index; ++i) {
                const PointT &pt = input_->points[index_vector[i].point_index];
                x += pt.x;
                y += pt.y;
                z += pt.z;
            }
            float inv_count             = 1.0f / count;
            output.points[output_idx].x = x * inv_count;
            output.points[output_idx].y = y * inv_count;
            output.points[output_idx].z = z * inv_count;

            // 根据模式计算 intensity
            switch (mode_) {
            case Mode::AVERAGE: {
                float sum_intensity = 0;
                for (unsigned int i = first_index; i < last_index; ++i) {
                    sum_intensity += input_->points[index_vector[i].point_index].intensity;
                }
                output.points[output_idx].intensity = sum_intensity * inv_count;
                break;
            }
            case Mode::ACCUMULATE: {
                float r = 1.0f;
                for (unsigned int i = first_index; i < last_index; ++i) {
                    float factor = 1.0f - input_->points[index_vector[i].point_index].intensity *
                                              r_decay_coeff_;
                    factor = std::max(0.0f, std::min(1.0f, factor));
                    r *= factor;
                }
                output.points[output_idx].intensity = 1.0f - r;
                break;
            }
            case Mode::MAX: {
                float max_intensity = -std::numeric_limits<float>::max();
                for (unsigned int i = first_index; i < last_index; ++i) {
                    max_intensity = std::max(max_intensity,
                                             input_->points[index_vector[i].point_index].intensity);
                }
                output.points[output_idx].intensity = max_intensity;
                break;
            }
            }

            ++output_idx;
        }
    }

  private:
    float leaf_size_;
    float r_decay_coeff_;
    Mode mode_;
    PointCloudConstPtrT input_;

    // 体素网格参数
    Eigen::Array3f inverse_leaf_size_;
    Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
};

}  // namespace core
}  // namespace pcl_detection2

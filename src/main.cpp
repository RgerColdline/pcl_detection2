#include "adapters/livox_converter.hpp"
#include "adapters/pc_tf_matrix.hpp"
#include "core/register.hpp"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <unordered_set>

// 点云积累类（连续降采样 + ROI过滤 + 膨胀 + 腐蚀 + 平面投影）
class CloudAccumulator
{
  public:
    CloudAccumulator(ros::NodeHandle &nh) : nh_(nh) {
        // 初始化点云指针（原始→降采样→ROI→膨胀→腐蚀→投影）
        raw_livox_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        downsampled_livox_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        tf_livox_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        register_map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        raw_accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        downsampled_accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        roi_filtered_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        dilated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        eroded_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        projected_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);  // 新增：投影后点云

        // 订阅/发布器
        cloud_sub_ = nh_.subscribe("/livox/lidar", 1, &CloudAccumulator::cloudCallback, this);
        odometry_sub_ =
            nh_.subscribe("/mavros/local_position/pose", 1,
                          &pcl_detection2::adapters::PcTfMatrix::odometry_cb, &tf_adapter_);
        accumulated_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/accumulated_cloud", 1);
        downsampled_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/downsampled_accumulated_cloud", 1);
        roi_filtered_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/roi_filtered_accumulated_cloud", 1);
        dilated_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/dilated_accumulated_cloud", 1);
        eroded_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/eroded_accumulated_cloud", 1);
        projected_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "/projected_accumulated_cloud", 1);  // 新增：投影结果发布

        // 内部模块初始化
        register_ = std::make_unique<pcl_detection2::core::Register>(0.5f, 50, 1e-8f, 0.001f,
                                                                     Eigen::Matrix4f::Identity());

        // 启用参数
        nh_.setParam("/pcl_enable", false);
        // 降采样参数
        voxel_leaf_size_    = 0.05;  // 5cm体素分辨率

        // ROI区域参数
        roi_x_min_          = -0.5;
        roi_x_max_          = 4.5;
        roi_y_min_          = -7.0;
        roi_y_max_          = 1.0;
        roi_z_min_          = 0.3;
        roi_z_max_          = 2;

        // 膨胀参数
        dilation_radius_    = 0.25;  // 膨胀半径（米）
        dilation_steps_     = 12;    // 膨胀步数

        // 腐蚀参数
        erosion_radius_     = 0.15;  // 腐蚀搜索半径（米）
        erosion_min_points_ = 3;     // 最小点数阈值

        // 新增：投影参数（默认投影到Z=0的地面平面）
        projection_plane_z_ = 0.0;  // 投影目标平面的Z坐标（可自定义为其他值，如0.1m）
    }

    // 点云回调函数：接收 → 累加 → 降采样 → ROI过滤 → 膨胀 → 腐蚀 → 投影 → 发布
    // void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    void cloudCallback(const livox_ros_driver2::CustomMsg::Ptr &livox_msg) {
        // 1. ROS转PCL点云
        // pcl::fromROSMsg(*cloud_msg, *raw_livox_cloud_);
        if (nh_.getParam("/pcl_enable", pcl_enable_)) {
            if (!pcl_enable_) {
                ROS_INFO_THROTTLE(1, "等待pcl启动中");
                return;
            }
        }

        if (!pcl_detection2::adapters::LivoxConverter<pcl::PointXYZ>::convert(livox_msg,
                                                                              raw_livox_cloud_, 0))
        {
            ROS_WARN("转换点云失败");
            return;
        }

        if (raw_livox_cloud_->empty()) {
            ROS_WARN("接收到空点云，跳过积累");
            return;
        }
        ROS_INFO("[积累] 接收新点云：%zu 个点", raw_livox_cloud_->size());

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(raw_livox_cloud_);
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_filter.filter(*downsampled_livox_cloud_);

        Eigen::Affine3f transform;
        if (!tf_adapter_.get_transform(transform)) {
            ROS_WARN_THROTTLE(1.0, "里程计无消息，等待中");
            return;
        }
        pcl::transformPointCloud(*downsampled_livox_cloud_, *tf_livox_cloud_, transform);

        if (roi_filtered_cloud_->empty()) {
            ROS_WARN("首次点云，跳过配准");
        }
        else {
            pcl::transformPointCloud(
                *roi_filtered_cloud_, *register_map_cloud_,
                register_->registerSourceToTarget(roi_filtered_cloud_, tf_livox_cloud_));
        }

        // *raw_accumulated_cloud_ = *downsampled_accumulated_cloud_ + *tf_livox_cloud_;
        *register_map_cloud_ += *tf_livox_cloud_;

        /***********template 2**************/

        // voxel_filter.setInputCloud(raw_accumulated_cloud_);
        voxel_filter.setInputCloud(register_map_cloud_);
        voxel_filter.filter(*downsampled_accumulated_cloud_);

        /***********************************/

        // ROI区域过滤
        filterROI(downsampled_accumulated_cloud_, roi_filtered_cloud_);

        // 点云腐蚀处理
        erodePointCloud(roi_filtered_cloud_, eroded_cloud_);

        // 点云膨胀处理
        dilatePointCloud(eroded_cloud_, dilated_cloud_);

        // 点云平面投影处理
        projectPointCloud(dilated_cloud_, projected_cloud_);

        // 8. 发布各类点云
        publishAccumulatedCloud(livox_msg->header);
        publishDownsampledCloud(livox_msg->header);
        publishROIFilteredCloud(livox_msg->header);
        publishDilatedCloud(livox_msg->header);
        publishErodedCloud(livox_msg->header);
        publishProjectedCloud(livox_msg->header);  // 新增：发布投影后点云
    }

  private:
    // ROI过滤核心函数
    void filterROI(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        /***********template 3**************/

        pcl::CropBox<pcl::PointXYZ> roi_filter;
        roi_filter.setInputCloud(input);
        roi_filter.setMin(Eigen::Vector4f(roi_x_min_, roi_y_min_, roi_z_min_, 1.0));
        roi_filter.setMax(Eigen::Vector4f(roi_x_max_, roi_y_max_, roi_z_max_, 1.0));
        roi_filter.filter(*output);

        /***********************************/
    }

    struct VoxelKeyHash
    {
        std::size_t operator() (const Eigen::Vector3i &key) const {
            return std::hash<int>()(key[0]) ^ (std::hash<int>()(key[1]) << 1) ^
                   (std::hash<int>()(key[2]) << 2);
        }
    };
    // 点云膨胀核心函数
    void dilatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
        // float resolution = 0.05f;
        // float radius     = 0.15f;
        output->clear();
        if (input->empty()) return;

        // /***********template 4**************/
        // using VoxelSet = std::unordered_set<Eigen::Vector3i, VoxelKeyHash>;
        // VoxelSet occupied_voxels;

        // // 1. 体素化：将点映射到网格索引
        // for (const auto &pt : input->points)
        // {
        //     Eigen::Vector3i idx(
        //         static_cast<int>(std::floor(pt.x / resolution)),
        //         static_cast<int>(std::floor(pt.y / resolution)),
        //         static_cast<int>(std::floor(pt.z / resolution)));
        //     occupied_voxels.insert(idx);
        // }

        // // 2. 球形膨胀：遍历邻域并校验欧氏距离
        // VoxelSet dilated_voxels;
        // int voxel_range = static_cast<int>(std::ceil(radius / resolution));
        // float radius_sq = radius * radius;

        // for (const auto &voxel : occupied_voxels)
        // {
        //     for (int dx = -voxel_range; dx <= voxel_range; ++dx)
        //     {
        //         for (int dy = -voxel_range; dy <= voxel_range; ++dy)
        //         {
        //             for (int dz = -voxel_range; dz <= voxel_range; ++dz)
        //             {
        //                 // 球形校验：体素中心距离 <= 膨胀半径
        //                 float dist_sq = (dx * resolution) * (dx * resolution) +
        //                                 (dy * resolution) * (dy * resolution) +
        //                                 (dz * resolution) * (dz * resolution);

        //                 if (dist_sq <= radius_sq)
        //                 {
        //                     dilated_voxels.insert(voxel + Eigen::Vector3i(dx, dy, dz));
        //                 }
        //             }
        //         }
        //     }
        // }

        // // 3. 重构：将体素索引转回点云坐标 (体素中心)
        // output->reserve(dilated_voxels.size());
        // float half_res = resolution * 0.5f;
        // for (const auto &voxel : dilated_voxels)
        // {
        //     pcl::PointXYZ p;
        //     p.x = (voxel[0] + 0.5f) * resolution;
        //     p.y = (voxel[1] + 0.5f) * resolution;
        //     p.z = (voxel[2] + 0.5f) * resolution;
        //     output->push_back(p);
        // }
        // /***********************************/

        static pcl::PointCloud<pcl::PointXYZ>::Ptr temppc(new pcl::PointCloud<pcl::PointXYZ>);
        temppc->clear();
        const float deta_round = 3 * M_PI / dilation_steps_;
        const float deta_above = M_PI / dilation_steps_;
        for (auto &point : input->points) {
            for (int i = 0; i < dilation_steps_; ++i) {
                float delta = i * deta_round;
                for (int i = 0; i < dilation_steps_; i++) {
                    float delta_z = i * deta_above;
                    pcl::PointXYZ dilated_point;
                    dilated_point.x = point.x + dilation_radius_ * cos(delta) * cos(delta_z);
                    dilated_point.y = point.y + dilation_radius_ * sin(delta) * cos(delta_z);
                    dilated_point.z = point.z + dilation_radius_ * sin(delta_z);
                    temppc->push_back(dilated_point);
                }
            }
        }
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(temppc);
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_filter.filter(*output);
    }

    // 点云腐蚀核心函数
    void erodePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
        output->clear();
        if (input->empty()) return;

        // // 构建KD-Tree：用于快速搜索邻域点
        // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        // kdtree.setInputCloud(input);

        // // 遍历每个点，统计邻域点数
        // std::vector<int> point_indices;
        // std::vector<float> point_distances;
        // for (const auto &point : *input)
        // {
        //     // 搜索radius范围内的所有邻域点（包含自身）
        //     /***********template 5**************/

        //     /***********************************/
        // }

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter;
        sorfilter.setInputCloud(input);
        sorfilter.setMeanK(5);
        sorfilter.setStddevMulThresh(0.5);
        sorfilter.filter(*output);
    }

    // 新增：点云平面投影核心函数（默认投影到Z=projection_plane_z_平面）
    void projectPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
        output->clear();
        if (input->empty()) return;

        // // 遍历所有点，将Z坐标强制设为投影平面的Z值（X/Y保持不变）
        // for (const auto &point : *input)
        // {
        //     pcl::PointXYZ projected_point;
        //     projected_point.x = point.x;             // X坐标不变
        //     projected_point.y = point.y;             // Y坐标不变
        //     projected_point.z = projection_plane_z_; // Z坐标投影到指定平面
        //     output->push_back(projected_point);
        // }

        // /***********template 6**************/

        // /***********************************/
        pcl::ModelCoefficients::Ptr ground(new pcl::ModelCoefficients);
        ground->values = {0.0, 0.0, 1.0, -0.7};  // [a,b,c,d]

        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(input);
        proj.setModelCoefficients(ground);
        proj.setCopyAllData(false);  // 仅输出投影后的内点
        proj.filter(*output);
    }

    // 发布带颜色点云工具函数
    //  void toMsgColor(pcl::PointCloud)

    // 发布原始积累点云
    void publishAccumulatedCloud(const std_msgs::Header &header) {
        if (raw_accumulated_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*raw_accumulated_cloud_, output_msg);
        output_msg.header = header;
        accumulated_cloud_pub_.publish(output_msg);
    }

    // 发布降采样后点云
    void publishDownsampledCloud(const std_msgs::Header &header) {
        if (downsampled_accumulated_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*downsampled_accumulated_cloud_, output_msg);
        output_msg.header = header;
        downsampled_cloud_pub_.publish(output_msg);
    }

    // 发布ROI过滤后点云
    void publishROIFilteredCloud(const std_msgs::Header &header) {
        if (roi_filtered_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*roi_filtered_cloud_, output_msg);
        output_msg.header = header;
        roi_filtered_pub_.publish(output_msg);
    }

    // 发布膨胀后点云
    void publishDilatedCloud(const std_msgs::Header &header) {
        if (dilated_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*dilated_cloud_, output_msg);
        output_msg.header = header;
        dilated_cloud_pub_.publish(output_msg);
    }

    // 发布腐蚀后点云
    void publishErodedCloud(const std_msgs::Header &header) {
        if (eroded_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*eroded_cloud_, output_msg);
        output_msg.header = header;
        eroded_cloud_pub_.publish(output_msg);
    }

    // 新增：发布投影后点云
    void publishProjectedCloud(const std_msgs::Header &header) {
        if (projected_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*projected_cloud_, output_msg);
        output_msg.header = header;
        projected_cloud_pub_.publish(output_msg);
        ROS_INFO("[发布] 投影点云已发布到 /projected_accumulated_cloud，点数：%zu",
                 projected_cloud_->size());
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odometry_sub_;
    ros::Publisher accumulated_cloud_pub_;
    ros::Publisher downsampled_cloud_pub_;
    ros::Publisher roi_filtered_pub_;
    ros::Publisher dilated_cloud_pub_;
    ros::Publisher eroded_cloud_pub_;
    ros::Publisher projected_cloud_pub_;  // 新增：投影点云发布器

    pcl_detection2::adapters::PcTfMatrix tf_adapter_;
    std::unique_ptr<pcl_detection2::core::Register> register_;

    // 点云容器（六级处理：原始 → 降采样 → ROI过滤 → 膨胀 → 腐蚀 → 投影）
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_livox_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_livox_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tf_livox_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr register_map_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_accumulated_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_accumulated_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_filtered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr dilated_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr eroded_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_;  // 新增：投影后点云

    // 参数
    // 启用参数
    bool pcl_enable_;
    // 降采样参数
    double voxel_leaf_size_;
    // ROI参数
    double roi_x_min_, roi_x_max_;
    double roi_y_min_, roi_y_max_;
    double roi_z_min_, roi_z_max_;
    // 膨胀参数
    double dilation_radius_;
    int dilation_steps_;
    // 腐蚀参数
    double erosion_radius_;
    int erosion_min_points_;
    // 新增：投影参数
    double projection_plane_z_;  // 投影平面的Z坐标（默认Z=0，地面平面）
};

// 主函数
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cloud_template_node");
    ros::NodeHandle nh;

    CloudAccumulator accumulator(nh);
    ros::spin();

    return 0;
}

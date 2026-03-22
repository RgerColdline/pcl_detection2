#include "adapters/livox_converter.hpp"
#include "adapters/pc_tf_matrix.hpp"
#include "core/register.hpp"
#include "core/roi.hpp"
#include "core/voxel.hpp"

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
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
    using PointT         = pcl::PointXYZI;
    using PointCloudT    = pcl::PointCloud<PointT>;
    using PointCloudPtrT = PointCloudT::Ptr;
    using VoxelFilterT   = pcl_detection2::core::VoxelGridUnified<PointT>;
    CloudAccumulator(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh) {
        // 初始化点云指针（原始→降采样→ROI→膨胀→腐蚀→投影）
        raw_livox_cloud_.reset(new PointCloudT);
        downsampled_livox_cloud_.reset(new PointCloudT);
        tf_livox_cloud_.reset(new PointCloudT);
        register_map_cloud_.reset(new PointCloudT);
        raw_accumulated_cloud_.reset(new PointCloudT);
        downsampled_accumulated_cloud_.reset(new PointCloudT);
        roi_filtered_cloud_.reset(new PointCloudT);
        dilated_cloud_.reset(new PointCloudT);
        eroded_cloud_.reset(new PointCloudT);
        projected_cloud_.reset(new PointCloudT);  // 新增：投影后点云

        // 订阅/发布器
        cloud_sub_ = nh_.subscribe("/livox/lidar", 1, &CloudAccumulator::cloudCallback, this);
        odometry_sub_ =
            nh_.subscribe("/mavros/local_position/pose", 1,
                          &pcl_detection2::adapters::PcTfMatrix::odometry_cb, &tf_adapter_);
        accumulated_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/accumulated_cloud", 1);
        raw_livox_cloud_pub_   = nh_.advertise<sensor_msgs::PointCloud2>("/raw_livox_cloud", 1);
        downsampled_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/downsampled_accumulated_cloud", 1);
        roi_filtered_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/roi_filtered_accumulated_cloud", 1);
        dilated_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/dilated_accumulated_cloud", 1);
        eroded_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/eroded_accumulated_cloud", 1);
        projected_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "/projected_accumulated_cloud", 1);  // 新增：投影结果发布

        // 启用参数
        pnh_.param("initial_enable", pcl_enable_, false);
        nh_.setParam("/pcl_enable", pcl_enable_);

        // livox roi参数
        pnh_.param("livox_roi/uav_radius", livox_roi_uav_radius_, 0.3f);
        pnh_.param("livox_roi/max_coord", livox_roi_max_coord_, 20.0f);
        pnh_.param("livox_roi/debug_level", livox_roi_debug_level_, 0);

        // 降采样参数
        pnh_.param("voxel/leaf_size", voxel_leaf_size_, 0.05);

        // 地图累计参数
        pnh_.param("map/decay_coeff", map_decay_coeff_, 0.3f);
        map_r_decay_coeff_ = 1 - map_decay_coeff_;
        pnh_.param("map/ini_intensity", map_ini_intensity_, 50.0f);
        map_tf_ini_intensity_ = map_ini_intensity_ / map_r_decay_coeff_ / 100;

        // 配准参数
        pnh_.param("register/max_correspondence_distance", register_max_correspondence_distance_,
                   0.5f);
        pnh_.param("register/max_iter", register_max_iter_, 50);
        pnh_.param("register/transformation_epsilon", register_transformation_epsilon_, 1e-8f);
        pnh_.param("register/euclidean_fitness_epsilon", register_euclidean_fitness_epsilon_,
                   0.001f);

        // ROI区域参数
        pnh_.param("roi/x_min", roi_x_min_, -0.5f);
        pnh_.param("roi/x_max", roi_x_max_, 4.5f);
        pnh_.param("roi/y_min", roi_y_min_, -7.0f);
        pnh_.param("roi/y_max", roi_y_max_, 1.0f);
        pnh_.param("roi/z_min", roi_z_min_, 0.3f);
        pnh_.param("roi/z_max", roi_z_max_, 2.0f);

        // 膨胀参数
        pnh_.param("dilation/radius", dilation_radius_, 0.25);
        pnh_.param("dilation/steps", dilation_steps_, 12);

        // 腐蚀参数
        pnh_.param("erosion/mean_k", erosion_mean_k_, 5);
        pnh_.param("erosion/thresh", erosion_thresh_, 0.5);

        // 投影参数（默认投影到Z=0的地面平面）
        pnh_.param("projection/plane_z", projection_plane_z_, 0.7f);

        // roi器初始化
        crop_box_ = std::make_unique<pcl_detection2::core::CropBoxRoi<PointT>>(
            livox_roi_uav_radius_, roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_,
            roi_z_max_);

        // 配准器初始化
        register_ = std::make_unique<pcl_detection2::core::Register<PointT>>(
            register_max_correspondence_distance_, register_max_iter_,
            register_transformation_epsilon_, register_euclidean_fitness_epsilon_);

        // 体素滤波器初始化
        voxel_filter_ = std::make_unique<pcl_detection2::core::VoxelGridUnified<PointT>>(
            voxel_leaf_size_, map_r_decay_coeff_);
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

        if (!pcl_detection2::adapters::LivoxConverter<PointT>::convert(
                livox_msg, raw_livox_cloud_, map_tf_ini_intensity_, livox_roi_uav_radius_,
                livox_roi_max_coord_, livox_roi_debug_level_))
        {
            ROS_WARN("转换点云失败");
            return;
        }

        if (raw_livox_cloud_->empty()) {
            ROS_WARN("接收到空点云，跳过积累");
            return;
        }
        ROS_INFO_THROTTLE(1, "[积累] 接收新点云：%zu 个点", raw_livox_cloud_->size());

        // 初次降采样：使用 AVERAGE 模式
        voxel_filter_->filterCloud(raw_livox_cloud_, downsampled_livox_cloud_,
                                   VoxelFilterT::Mode::AVERAGE);

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

        // 积累后降采样：使用 ACCUMULATE 模式
        voxel_filter_->filterCloud(register_map_cloud_, downsampled_accumulated_cloud_,
                                   VoxelFilterT::Mode::ACCUMULATE);

        /***********************************/

        // ROI区域过滤
        crop_box_->filterROI(downsampled_accumulated_cloud_, roi_filtered_cloud_);

        // 点云腐蚀处理
        erodePointCloud(roi_filtered_cloud_, eroded_cloud_);

        // 点云膨胀处理
        dilatePointCloud(eroded_cloud_, dilated_cloud_);

        // 点云平面投影处理
        projectPointCloud(dilated_cloud_, projected_cloud_);

        // 8. 发布各类点云
        publishAccumulatedCloud(livox_msg->header);
        publishRawLivoxCloud(livox_msg->header);
        publishDownsampledCloud(livox_msg->header);
        publishROIFilteredCloud(livox_msg->header);
        publishDilatedCloud(livox_msg->header);
        publishErodedCloud(livox_msg->header);
        publishProjectedCloud(livox_msg->header);  // 新增：发布投影后点云
    }

  private:
    // 点云膨胀核心函数
    void dilatePointCloud(PointCloudPtrT input, PointCloudPtrT output) {
        // float resolution = 0.05f;
        // float radius     = 0.15f;
        output->clear();
        if (input->empty()) return;

        static PointCloudPtrT temppc(new PointCloudT);
        temppc->clear();
        const float deta_round = 3 * M_PI / dilation_steps_;
        const float deta_above = M_PI / dilation_steps_;
        for (auto &point : input->points) {
            for (int i = 0; i < dilation_steps_; ++i) {
                float delta = i * deta_round;
                for (int j = 0; j < dilation_steps_; j++) {
                    float delta_z = j * deta_above;
                    PointT dilated_point;
                    pcl::copyPoint(point, dilated_point);
                    dilated_point.x = point.x + dilation_radius_ * cos(delta) * cos(delta_z);
                    dilated_point.y = point.y + dilation_radius_ * sin(delta) * cos(delta_z);
                    dilated_point.z = point.z + dilation_radius_ * sin(delta_z);
                    temppc->push_back(dilated_point);
                }
            }
        }
        // 膨胀后降采样：使用 MAX 模式
        voxel_filter_->filterCloud(temppc, output, VoxelFilterT::Mode::MAX);
    }

    // 点云腐蚀核心函数
    void erodePointCloud(PointCloudPtrT input, PointCloudPtrT output) {
        output->clear();
        if (input->empty()) return;

        pcl::StatisticalOutlierRemoval<PointT> sorfilter;
        sorfilter.setInputCloud(input);
        sorfilter.setMeanK(erosion_mean_k_);
        sorfilter.setStddevMulThresh(erosion_thresh_);
        sorfilter.filter(*output);
    }

    // 新增：点云平面投影核心函数（默认投影到Z=projection_plane_z_平面）
    void projectPointCloud(PointCloudPtrT input, PointCloudPtrT output) {
        output->clear();
        if (input->empty()) return;

        pcl::ModelCoefficients::Ptr ground(new pcl::ModelCoefficients);
        ground->values = {0.0, 0.0, 1.0, -projection_plane_z_};  // [a,b,c,d]

        pcl::ProjectInliers<PointT> proj;
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


    // 发布原始livox点云
    void publishRawLivoxCloud(const std_msgs::Header &header) {
        if (raw_livox_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*raw_livox_cloud_, output_msg);
        output_msg.header = header;
        raw_livox_cloud_pub_.publish(output_msg);
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
        ROS_INFO_THROTTLE(1, "[发布] 投影点云已发布到 /projected_accumulated_cloud，点数：%zu",
                          projected_cloud_->size());
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odometry_sub_;
    ros::Publisher accumulated_cloud_pub_;
    ros::Publisher raw_livox_cloud_pub_;
    ros::Publisher downsampled_cloud_pub_;
    ros::Publisher roi_filtered_pub_;
    ros::Publisher dilated_cloud_pub_;
    ros::Publisher eroded_cloud_pub_;
    ros::Publisher projected_cloud_pub_;  // 新增：投影点云发布器

    pcl_detection2::adapters::PcTfMatrix tf_adapter_;
    std::unique_ptr<pcl_detection2::core::CropBoxRoi<PointT>> crop_box_;
    std::unique_ptr<pcl_detection2::core::Register<PointT>> register_;
    std::unique_ptr<pcl_detection2::core::VoxelGridUnified<PointT>> voxel_filter_;

    // 点云容器（六级处理：原始 → 降采样 → ROI过滤 → 膨胀 → 腐蚀 → 投影）
    PointCloudPtrT raw_livox_cloud_;
    PointCloudPtrT downsampled_livox_cloud_;
    PointCloudPtrT tf_livox_cloud_;
    PointCloudPtrT register_map_cloud_;
    PointCloudPtrT raw_accumulated_cloud_;
    PointCloudPtrT downsampled_accumulated_cloud_;
    PointCloudPtrT roi_filtered_cloud_;
    PointCloudPtrT dilated_cloud_;
    PointCloudPtrT eroded_cloud_;
    PointCloudPtrT projected_cloud_;  // 新增：投影后点云

    // 参数
    // 启用参数
    bool pcl_enable_;
    // livox ROI参数
    float livox_roi_uav_radius_;
    float livox_roi_max_coord_;
    int livox_roi_debug_level_;
    // 降采样参数
    double voxel_leaf_size_;
    // 地图积累参数
    float map_decay_coeff_;
    float map_r_decay_coeff_;
    float map_ini_intensity_;
    float map_tf_ini_intensity_;
    // 配准参数
    float register_max_correspondence_distance_;
    int register_max_iter_;
    float register_transformation_epsilon_;
    float register_euclidean_fitness_epsilon_;
    // ROI参数
    float roi_x_min_, roi_x_max_;
    float roi_y_min_, roi_y_max_;
    float roi_z_min_, roi_z_max_;
    // 膨胀参数
    double dilation_radius_;
    int dilation_steps_;
    // 腐蚀参数
    int erosion_mean_k_;
    double erosion_thresh_;
    // 新增：投影参数
    float projection_plane_z_;  // 投影平面的Z坐标（默认Z=0，地面平面）
};

// 主函数
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cloud_template_node");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    CloudAccumulator accumulator(nh, pnh);
    ros::spin();

    return 0;
}

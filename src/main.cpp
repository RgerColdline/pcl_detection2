#include "adapters/livox_converter.hpp"
#include "adapters/pc_tf_matrix.hpp"
#include "core/register.hpp"
#include "core/roi.hpp"
#include "core/voxel.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <deque>
#include <sstream>
#include <vector>

class CloudAccumulator
{
  public:
    using PointT         = pcl::PointXYZI;
    using PointCloudT    = pcl::PointCloud<PointT>;
    using PointCloudPtrT = PointCloudT::Ptr;
    using VoxelFilterT   = pcl_detection2::core::VoxelGridUnified<PointT>;
    using RegisterT      = pcl_detection2::core::Register<PointT>;

    struct TimingReport {
        ros::WallTime last_tick = ros::WallTime::now();
        std::vector<std::pair<std::string, double>> stages_ms;

        void reset() {
            last_tick = ros::WallTime::now();
            stages_ms.clear();
        }

        void mark(const std::string &label) {
            const ros::WallTime now = ros::WallTime::now();
            const double elapsed_ms = (now - last_tick).toSec() * 1000.0;
            stages_ms.emplace_back(label, elapsed_ms);
            last_tick = now;
        }
    };

    struct TimingSummary {
        ros::WallTime window_start = ros::WallTime::now();
        std::vector<std::pair<std::string, double>> stage_sum_ms;
        int frame_count = 0;

        void addFrame(const TimingReport &timing) {
            if (stage_sum_ms.empty()) {
                stage_sum_ms = timing.stages_ms;
            }
            else {
                const std::size_t count = std::min(stage_sum_ms.size(), timing.stages_ms.size());
                for (std::size_t i = 0; i < count; ++i) {
                    stage_sum_ms[i].second += timing.stages_ms[i].second;
                }
            }
            ++frame_count;
        }

        bool shouldFlush(const ros::WallTime &now) const {
            return (now - window_start).toSec() >= 1.0;
        }

        void reset(const ros::WallTime &now) {
            window_start = now;
            stage_sum_ms.clear();
            frame_count = 0;
        }
    };

    CloudAccumulator(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh) {
        raw_livox_cloud_.reset(new PointCloudT);
        downsampled_livox_cloud_.reset(new PointCloudT);
        predicted_livox_cloud_.reset(new PointCloudT);
        aligned_livox_cloud_.reset(new PointCloudT);
        local_map_cloud_.reset(new PointCloudT);
        downsampled_local_map_cloud_.reset(new PointCloudT);
        roi_filtered_cloud_.reset(new PointCloudT);
        eroded_cloud_.reset(new PointCloudT);
        dilated_cloud_.reset(new PointCloudT);
        projected_cloud_.reset(new PointCloudT);

        cloud_sub_ = nh_.subscribe("/livox/lidar", 1, &CloudAccumulator::cloudCallback, this);
        odometry_sub_ =
            nh_.subscribe("/mavros/local_position/pose", 1,
                          &pcl_detection2::adapters::PcTfMatrix::odometry_cb, &tf_adapter_);

        accumulated_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/pcl_detection2/accumulated_cloud", 1);
        raw_livox_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/pcl_detection2/raw_livox_cloud", 1);
        downsampled_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>(
                "/pcl_detection2/downsampled_accumulated_cloud", 1);
        roi_filtered_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>(
                "/pcl_detection2/roi_filtered_accumulated_cloud", 1);
        dilated_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/pcl_detection2/dilated_accumulated_cloud", 1);
        eroded_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/pcl_detection2/eroded_accumulated_cloud", 1);
        projected_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/pcl_detection2/projected_accumulated_cloud",
                                                    1);

        pnh_.param("initial_enable", pcl_enable_, false);
        nh_.setParam("/pcl_enable", pcl_enable_);
        pnh_.param("debug/timing_enable", timing_enable_, false);

        pnh_.param("livox_roi/uav_radius", livox_roi_uav_radius_, 0.3f);
        pnh_.param("livox_roi/max_coord", livox_roi_max_coord_, 20.0f);
        pnh_.param("livox_roi/debug_level", livox_roi_debug_level_, 0);

        pnh_.param("voxel/leaf_size", voxel_leaf_size_, 0.05);

        pnh_.param("map/decay_coeff", map_decay_coeff_, 0.3f);
        map_r_decay_coeff_ = 1.0f - map_decay_coeff_;
        pnh_.param("map/ini_intensity", map_ini_intensity_, 50.0f);
        map_tf_ini_intensity_ = map_ini_intensity_ / map_r_decay_coeff_ / 100.0f;
        pnh_.param("map/local_frame_num", local_map_frame_num_, 15);

        pnh_.param("register/max_correspondence_distance", register_max_correspondence_distance_,
                   0.5f);
        pnh_.param("register/max_iter", register_max_iter_, 50);
        pnh_.param("register/transformation_epsilon", register_transformation_epsilon_, 1e-8f);
        pnh_.param("register/euclidean_fitness_epsilon", register_euclidean_fitness_epsilon_,
                   0.001f);
        pnh_.param("register/max_fitness_score", register_max_fitness_score_, 0.3f);
        pnh_.param("register/max_translation_delta", register_max_translation_delta_, 0.6f);
        pnh_.param("register/max_rotation_delta_deg", register_max_rotation_delta_deg_, 20.0f);

        pnh_.param("roi/x_min", roi_x_min_, -0.5f);
        pnh_.param("roi/x_max", roi_x_max_, 4.5f);
        pnh_.param("roi/y_min", roi_y_min_, -7.0f);
        pnh_.param("roi/y_max", roi_y_max_, 1.0f);
        pnh_.param("roi/z_min", roi_z_min_, 0.3f);
        pnh_.param("roi/z_max", roi_z_max_, 2.0f);

        pnh_.param("dilation/radius", dilation_radius_, 0.25);
        pnh_.param("dilation/steps", dilation_steps_, 12);

        pnh_.param("erosion/mean_k", erosion_mean_k_, 5);
        pnh_.param("erosion/thresh", erosion_thresh_, 0.5);

        pnh_.param("projection/plane_z", projection_plane_z_, 0.7f);

        crop_box_ = std::make_unique<pcl_detection2::core::CropBoxRoi<PointT>>(
            livox_roi_uav_radius_, roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_,
            roi_z_max_);
        register_ = std::make_unique<RegisterT>(register_max_correspondence_distance_,
                                                register_max_iter_,
                                                register_transformation_epsilon_,
                                                register_euclidean_fitness_epsilon_);
        voxel_filter_ = std::make_unique<VoxelFilterT>(voxel_leaf_size_, map_r_decay_coeff_);
    }

    void cloudCallback(const livox_ros_driver2::CustomMsg::Ptr &livox_msg) {
        TimingReport timing;
        if (timing_enable_) timing.reset();

        if (nh_.getParam("/pcl_enable", pcl_enable_) && !pcl_enable_) {
            ROS_INFO_THROTTLE(1, "等待pcl启动中");
            return;
        }

        if (!pcl_detection2::adapters::LivoxConverter<PointT>::convert(
                livox_msg, raw_livox_cloud_, map_tf_ini_intensity_, livox_roi_uav_radius_,
                livox_roi_max_coord_, livox_roi_debug_level_))
        {
            ROS_WARN_THROTTLE(1, "转换点云失败");
            return;
        }
        if (timing_enable_) timing.mark("convert");

        if (raw_livox_cloud_->empty()) {
            ROS_WARN_THROTTLE(1, "接收到空点云，跳过本次更新");
            return;
        }

        voxel_filter_->filterCloud(raw_livox_cloud_, downsampled_livox_cloud_,
                                   VoxelFilterT::Mode::AVERAGE);
        if (timing_enable_) timing.mark("downsample");
        if (downsampled_livox_cloud_->empty()) {
            ROS_WARN_THROTTLE(1, "降采样后为空，跳过本次更新");
            return;
        }

        Eigen::Matrix4f odom_pose = Eigen::Matrix4f::Identity();
        if (!tf_adapter_.get_transform_matrix(odom_pose)) {
            ROS_WARN_THROTTLE(1, "里程计无消息，等待中");
            return;
        }
        if (timing_enable_) timing.mark("odom");

        if (!pose_initialized_) {
            last_odom_pose_ = odom_pose;
            last_map_pose_  = Eigen::Matrix4f::Identity();
            pcl::transformPointCloud(*downsampled_livox_cloud_, *aligned_livox_cloud_,
                                     last_map_pose_);
            pose_initialized_ = true;
            ROS_INFO("初始化局部地图，首帧直接写入地图窗口");
            if (timing_enable_) timing.mark("predict");
        }
        else {
            const Eigen::Matrix4f odom_delta = last_odom_pose_.inverse() * odom_pose;
            const Eigen::Matrix4f predicted_pose = last_map_pose_ * odom_delta;

            pcl::transformPointCloud(*downsampled_livox_cloud_, *predicted_livox_cloud_,
                                     predicted_pose);
            if (timing_enable_) timing.mark("predict");

            bool registration_accepted = false;
            if (!downsampled_local_map_cloud_->empty()) {
                const auto registration_result = register_->registerSourceToTarget(
                    downsampled_livox_cloud_, downsampled_local_map_cloud_, predicted_pose);

                const Eigen::Matrix4f correction_delta =
                    predicted_pose.inverse() * registration_result.final_transform;
                const float translation_delta = correction_delta.block<3, 1>(0, 3).norm();
                const float rotation_delta_deg = rotationDeltaDeg(correction_delta);

                registration_accepted = registration_result.converged &&
                                        registration_result.fitness_score <=
                                            register_max_fitness_score_ &&
                                        translation_delta <= register_max_translation_delta_ &&
                                        rotation_delta_deg <= register_max_rotation_delta_deg_;

                if (registration_accepted) {
                    *aligned_livox_cloud_ = *registration_result.aligned_cloud;
                    last_map_pose_        = registration_result.final_transform;
                }
                else {
                    *aligned_livox_cloud_ = *predicted_livox_cloud_;
                    last_map_pose_        = predicted_pose;
                    ROS_WARN_THROTTLE(
                        1,
                        "ICP回退预测值: converged=%d fitness=%.4f delta_t=%.3f delta_r=%.2fdeg",
                        registration_result.converged, registration_result.fitness_score,
                        translation_delta, rotation_delta_deg);
                }
            }
            else {
                *aligned_livox_cloud_ = *predicted_livox_cloud_;
                last_map_pose_        = predicted_pose;
                ROS_WARN_THROTTLE(1, "局部地图为空，使用里程计预测位姿");
            }

            if (registration_accepted) {
                ROS_INFO_THROTTLE(1, "ICP修正已接受，局部地图点数: %zu",
                                  downsampled_local_map_cloud_->size());
            }
            if (timing_enable_) timing.mark("register");
        }

        last_odom_pose_ = odom_pose;

        updateLocalMap(aligned_livox_cloud_);
        if (timing_enable_) timing.mark("local_map");
        updateObstacleCloud();
        if (timing_enable_) timing.mark("obstacle");

        publishAccumulatedCloud(livox_msg->header);
        publishRawLivoxCloud(livox_msg->header);
        publishDownsampledCloud(livox_msg->header);
        publishROIFilteredCloud(livox_msg->header);
        publishDilatedCloud(livox_msg->header);
        publishErodedCloud(livox_msg->header);
        publishProjectedCloud(livox_msg->header);
        if (timing_enable_) {
            timing.mark("publish");
            logTimingReport(timing);
        }
    }

  private:
    static float rotationDeltaDeg(const Eigen::Matrix4f &transform) {
        const float trace = transform.block<3, 3>(0, 0).trace();
        const float cos_theta = std::max(-1.0f, std::min(1.0f, (trace - 1.0f) * 0.5f));
        return std::acos(cos_theta) * 180.0f / static_cast<float>(M_PI);
    }

    void logTimingReport(const TimingReport &timing) const {
        timing_summary_.addFrame(timing);

        const ros::WallTime now = ros::WallTime::now();
        if (!timing_summary_.shouldFlush(now)) return;

        const double window_sec = (now - timing_summary_.window_start).toSec();
        const double fps = window_sec > 0.0 ? timing_summary_.frame_count / window_sec : 0.0;
        double total_ms = 0.0;
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(2);
        oss << "[pcl_timing]"
            << " frames=" << timing_summary_.frame_count
            << " fps=" << fps;
        for (const auto &stage : timing_summary_.stage_sum_ms) {
            const double avg_ms = timing_summary_.frame_count > 0
                                      ? stage.second / timing_summary_.frame_count
                                      : 0.0;
            total_ms += avg_ms;
            oss << ' ' << stage.first << '=' << avg_ms << "ms";
        }
        oss << " total=" << total_ms << "ms";
        ROS_INFO_STREAM(oss.str());
        timing_summary_.reset(now);
    }

    void updateLocalMap(const PointCloudPtrT &aligned_cloud) {
        if (!aligned_cloud || aligned_cloud->empty()) return;

        local_map_frames_.emplace_back(new PointCloudT(*aligned_cloud));
        while (local_map_frames_.size() > static_cast<size_t>(local_map_frame_num_)) {
            local_map_frames_.pop_front();
        }

        local_map_cloud_->clear();
        for (const auto &frame : local_map_frames_) {
            *local_map_cloud_ += *frame;
        }

        voxel_filter_->filterCloud(local_map_cloud_, downsampled_local_map_cloud_,
                                   VoxelFilterT::Mode::ACCUMULATE);
    }

    void updateObstacleCloud() {
        roi_filtered_cloud_->clear();
        eroded_cloud_->clear();
        dilated_cloud_->clear();
        projected_cloud_->clear();

        if (downsampled_local_map_cloud_->empty()) return;

        crop_box_->filterROI(downsampled_local_map_cloud_, roi_filtered_cloud_);
        erodePointCloud(roi_filtered_cloud_, eroded_cloud_);
        dilatePointCloud(eroded_cloud_, dilated_cloud_);
        projectPointCloud(dilated_cloud_, projected_cloud_);
    }

    void dilatePointCloud(PointCloudPtrT input, PointCloudPtrT output) {
        output->clear();
        if (input->empty()) return;

        PointCloudPtrT temp_cloud(new PointCloudT);
        const float delta_round = 3.0f * static_cast<float>(M_PI) / dilation_steps_;
        const float delta_above = static_cast<float>(M_PI) / dilation_steps_;

        for (const auto &point : input->points) {
            for (int i = 0; i < dilation_steps_; ++i) {
                const float delta = i * delta_round;
                for (int j = 0; j < dilation_steps_; ++j) {
                    const float delta_z = j * delta_above;
                    PointT dilated_point;
                    pcl::copyPoint(point, dilated_point);
                    dilated_point.x = point.x + dilation_radius_ * std::cos(delta) * std::cos(delta_z);
                    dilated_point.y = point.y + dilation_radius_ * std::sin(delta) * std::cos(delta_z);
                    dilated_point.z = point.z + dilation_radius_ * std::sin(delta_z);
                    temp_cloud->push_back(dilated_point);
                }
            }
        }

        voxel_filter_->filterCloud(temp_cloud, output, VoxelFilterT::Mode::MAX);
    }

    void erodePointCloud(PointCloudPtrT input, PointCloudPtrT output) {
        output->clear();
        if (input->empty()) return;

        pcl::StatisticalOutlierRemoval<PointT> sor_filter;
        sor_filter.setInputCloud(input);
        sor_filter.setMeanK(erosion_mean_k_);
        sor_filter.setStddevMulThresh(erosion_thresh_);
        sor_filter.filter(*output);
    }

    void projectPointCloud(PointCloudPtrT input, PointCloudPtrT output) {
        output->clear();
        if (input->empty()) return;

        pcl::ModelCoefficients::Ptr ground(new pcl::ModelCoefficients);
        ground->values = {0.0, 0.0, 1.0, -projection_plane_z_};

        pcl::ProjectInliers<PointT> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(input);
        proj.setModelCoefficients(ground);
        proj.setCopyAllData(false);
        proj.filter(*output);
    }

    void publishAccumulatedCloud(const std_msgs::Header &header) {
        if (local_map_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*local_map_cloud_, output_msg);
        output_msg.header = header;
        accumulated_cloud_pub_.publish(output_msg);
    }

    void publishRawLivoxCloud(const std_msgs::Header &header) {
        if (raw_livox_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*raw_livox_cloud_, output_msg);
        output_msg.header = header;
        raw_livox_cloud_pub_.publish(output_msg);
    }

    void publishDownsampledCloud(const std_msgs::Header &header) {
        if (downsampled_local_map_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*downsampled_local_map_cloud_, output_msg);
        output_msg.header = header;
        downsampled_cloud_pub_.publish(output_msg);
    }

    void publishROIFilteredCloud(const std_msgs::Header &header) {
        if (roi_filtered_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*roi_filtered_cloud_, output_msg);
        output_msg.header = header;
        roi_filtered_pub_.publish(output_msg);
    }

    void publishDilatedCloud(const std_msgs::Header &header) {
        if (dilated_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*dilated_cloud_, output_msg);
        output_msg.header = header;
        dilated_cloud_pub_.publish(output_msg);
    }

    void publishErodedCloud(const std_msgs::Header &header) {
        if (eroded_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*eroded_cloud_, output_msg);
        output_msg.header = header;
        eroded_cloud_pub_.publish(output_msg);
    }

    void publishProjectedCloud(const std_msgs::Header &header) {
        if (projected_cloud_->empty()) return;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*projected_cloud_, output_msg);
        output_msg.header = header;
        projected_cloud_pub_.publish(output_msg);
        ROS_INFO_THROTTLE(1,
                          "[发布] 投影点云已发布到 /pcl_detection2/projected_accumulated_cloud，点数：%zu",
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
    ros::Publisher projected_cloud_pub_;

    pcl_detection2::adapters::PcTfMatrix tf_adapter_;
    std::unique_ptr<pcl_detection2::core::CropBoxRoi<PointT>> crop_box_;
    std::unique_ptr<RegisterT> register_;
    std::unique_ptr<VoxelFilterT> voxel_filter_;

    PointCloudPtrT raw_livox_cloud_;
    PointCloudPtrT downsampled_livox_cloud_;
    PointCloudPtrT predicted_livox_cloud_;
    PointCloudPtrT aligned_livox_cloud_;
    PointCloudPtrT local_map_cloud_;
    PointCloudPtrT downsampled_local_map_cloud_;
    PointCloudPtrT roi_filtered_cloud_;
    PointCloudPtrT eroded_cloud_;
    PointCloudPtrT dilated_cloud_;
    PointCloudPtrT projected_cloud_;
    std::deque<PointCloudPtrT> local_map_frames_;

    bool pose_initialized_ = false;
    Eigen::Matrix4f last_odom_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f last_map_pose_  = Eigen::Matrix4f::Identity();

    bool pcl_enable_;
    bool timing_enable_;
    mutable TimingSummary timing_summary_;
    float livox_roi_uav_radius_;
    float livox_roi_max_coord_;
    int livox_roi_debug_level_;
    double voxel_leaf_size_;
    float map_decay_coeff_;
    float map_r_decay_coeff_;
    float map_ini_intensity_;
    float map_tf_ini_intensity_;
    int local_map_frame_num_;
    float register_max_correspondence_distance_;
    int register_max_iter_;
    float register_transformation_epsilon_;
    float register_euclidean_fitness_epsilon_;
    float register_max_fitness_score_;
    float register_max_translation_delta_;
    float register_max_rotation_delta_deg_;
    float roi_x_min_;
    float roi_x_max_;
    float roi_y_min_;
    float roi_y_max_;
    float roi_z_min_;
    float roi_z_max_;
    double dilation_radius_;
    int dilation_steps_;
    int erosion_mean_k_;
    double erosion_thresh_;
    float projection_plane_z_;
};

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cloud_template_node");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    CloudAccumulator accumulator(nh, pnh);
    ros::spin();

    return 0;
}

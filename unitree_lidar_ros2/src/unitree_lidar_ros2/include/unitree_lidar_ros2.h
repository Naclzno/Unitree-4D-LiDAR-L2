/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <cstdint>
<<<<<<< HEAD
#include <fstream>
#include <iomanip>
=======
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
>>>>>>> test1

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// SDK
#include "unitree_lidar_sdk_pcl.h"


using std::placeholders::_1;

class UnitreeLidarSDKNode : public rclcpp::Node
{
public:
    explicit UnitreeLidarSDKNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~UnitreeLidarSDKNode();

    void timer_callback();

protected:
    // ROS
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // Unitree Lidar Reader
    UnitreeLidarReader *lsdk_;

    // Config params
    int work_mode_;
    int initialize_type_;
    int local_port_;
    std::string local_ip_;
    int lidar_port_;
    std::string lidar_ip_;
    std::string serial_port_;
    int baudrate_;
    
    int cloud_scan_num_;
    bool use_system_timestamp_;
    double range_min_;
    double range_max_;

    std::string cloud_frame_;
    std::string cloud_topic_;

    std::string imu_frame_;
    std::string imu_topic_;

    bool save_cloud_txt_;
<<<<<<< HEAD
    std::string cloud_txt_path_;
    bool cloud_txt_append_;
    int cloud_txt_save_every_n_;
    uint64_t cloud_txt_frame_count_;
    std::ofstream cloud_txt_stream_;

    void savePointCloudToTxt(const PointCloudUnitree &cloud);
=======
    std::string cloud_txt_save_mode_;
    std::string cloud_txt_path_;
    std::string cloud_txt_dir_;
    int cloud_txt_save_every_n_;
    uint64_t cloud_txt_frame_count_;

    void savePointCloudToTxt(const PointCloudUnitree &cloud);
    bool writePointCloudTxtFile(const std::string &file_path, const PointCloudUnitree &cloud, uint64_t frame_count);
>>>>>>> test1
};

///////////////////////////////////////////////////////////////////

UnitreeLidarSDKNode::~UnitreeLidarSDKNode()
{
<<<<<<< HEAD
    if (cloud_txt_stream_.is_open())
    {
        cloud_txt_stream_.close();
    }
=======
>>>>>>> test1
}

UnitreeLidarSDKNode::UnitreeLidarSDKNode(const rclcpp::NodeOptions &options)
    : Node("unitre_lidar_sdk_node", options)
{
    // load config parameters
    declare_parameter<int>("initialize_type", 2);
    declare_parameter<int>("work_mode", 0);
    declare_parameter<bool>("use_system_timestamp", true);
    declare_parameter<double>("range_min", 0);
    declare_parameter<double>("range_max", 50);
    declare_parameter<int>("cloud_scan_num", 18);

    declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 4000000);

    declare_parameter<int>("lidar_port", 6101);
    declare_parameter<std::string>("lidar_ip", "192.168.1.2");
    declare_parameter<int>("local_port", 6201);
    declare_parameter<std::string>("local_ip", "192.168.1.62");

    declare_parameter<std::string>("cloud_frame", "unilidar_lidar");
    declare_parameter<std::string>("cloud_topic", "unilidar/cloud");

    declare_parameter<std::string>("imu_frame", "unilidar_imu");
    declare_parameter<std::string>("imu_topic", "unilidar/imu");

    declare_parameter<bool>("save_cloud_txt", false);
<<<<<<< HEAD
    declare_parameter<std::string>("cloud_txt_path", "/tmp/unitree_lidar_cloud.txt");
    declare_parameter<bool>("cloud_txt_append", true);
=======
    declare_parameter<std::string>("cloud_txt_save_mode", "overwrite_one_file");
    declare_parameter<std::string>("cloud_txt_path", "/tmp/unitree_lidar_cloud.txt");
    declare_parameter<std::string>("cloud_txt_dir", "/tmp/unitree_lidar_cloud_frames");
>>>>>>> test1
    declare_parameter<int>("cloud_txt_save_every_n", 1);

    work_mode_ = get_parameter("work_mode").as_int();
    initialize_type_ = get_parameter("initialize_type").as_int();

    serial_port_ = get_parameter("serial_port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();

    lidar_port_ = get_parameter("lidar_port").as_int();
    lidar_ip_ = get_parameter("lidar_ip").as_string();
    local_port_ = get_parameter("local_port").as_int();
    local_ip_ = get_parameter("local_ip").as_string();

    cloud_scan_num_ = get_parameter("cloud_scan_num").as_int();
    use_system_timestamp_ = get_parameter("use_system_timestamp").as_bool();
    range_max_ = get_parameter("range_max").as_double();
    range_min_ = get_parameter("range_min").as_double();

    cloud_frame_ = get_parameter("cloud_frame").as_string();
    cloud_topic_ = get_parameter("cloud_topic").as_string();
    
    imu_frame_ = get_parameter("imu_frame").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();

    save_cloud_txt_ = get_parameter("save_cloud_txt").as_bool();
<<<<<<< HEAD
    cloud_txt_path_ = get_parameter("cloud_txt_path").as_string();
    cloud_txt_append_ = get_parameter("cloud_txt_append").as_bool();
=======
    cloud_txt_save_mode_ = get_parameter("cloud_txt_save_mode").as_string();
    cloud_txt_path_ = get_parameter("cloud_txt_path").as_string();
    cloud_txt_dir_ = get_parameter("cloud_txt_dir").as_string();
>>>>>>> test1
    cloud_txt_save_every_n_ = get_parameter("cloud_txt_save_every_n").as_int();
    cloud_txt_frame_count_ = 0;

    if (cloud_txt_save_every_n_ < 1)
    {
        RCLCPP_WARN(this->get_logger(), "cloud_txt_save_every_n must be >= 1, reset to 1");
        cloud_txt_save_every_n_ = 1;
    }

    // Initialize UnitreeLidarReader
    lsdk_ = createUnitreeLidarReader();

    std::cout << "initialize_type_ = " << initialize_type_ << std::endl;

    if (initialize_type_ == 1)
    {
        lsdk_->initializeSerial(serial_port_, baudrate_,
                                cloud_scan_num_, use_system_timestamp_, range_min_, range_max_);
    }
    else if (initialize_type_ == 2)
    {
        lsdk_->initializeUDP(lidar_port_, lidar_ip_, local_port_, local_ip_,
                             cloud_scan_num_, use_system_timestamp_, range_min_, range_max_);
    }
    else
    {
        std::cout << "initialize_type is not right! exit now ...\n";
        exit(0);
    }

    lsdk_->setLidarWorkMode(work_mode_);

    if (save_cloud_txt_)
    {
<<<<<<< HEAD
        std::ios_base::openmode open_mode = std::ios::out;
        open_mode |= cloud_txt_append_ ? std::ios::app : std::ios::trunc;

        cloud_txt_stream_.open(cloud_txt_path_, open_mode);
        if (!cloud_txt_stream_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "failed to open point cloud txt file: %s", cloud_txt_path_.c_str());
            save_cloud_txt_ = false;
        }
        else
        {
            cloud_txt_stream_ << "# Unitree L2 point cloud txt\n";
            cloud_txt_stream_ << "# columns: cloud_stamp cloud_id point_index x y z intensity relative_time ring\n";
            RCLCPP_INFO(this->get_logger(), "saving point cloud txt to: %s", cloud_txt_path_.c_str());
=======
        if (cloud_txt_save_mode_ != "overwrite_one_file" && cloud_txt_save_mode_ != "separate_files")
        {
            RCLCPP_ERROR(this->get_logger(),
                         "cloud_txt_save_mode must be overwrite_one_file or separate_files, got: %s",
                         cloud_txt_save_mode_.c_str());
            save_cloud_txt_ = false;
        }
        else if (cloud_txt_save_mode_ == "separate_files")
        {
            std::error_code error_code;
            std::filesystem::create_directories(cloud_txt_dir_, error_code);
            if (error_code)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "failed to create point cloud txt directory: %s, error: %s",
                             cloud_txt_dir_.c_str(),
                             error_code.message().c_str());
                save_cloud_txt_ = false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "saving each point cloud frame to txt files in: %s", cloud_txt_dir_.c_str());
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "saving latest point cloud frame to txt file: %s", cloud_txt_path_.c_str());
>>>>>>> test1
        }
    }

    // ROS2
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&UnitreeLidarSDKNode::timer_callback, this));
}

void UnitreeLidarSDKNode::timer_callback()
{
    int result = lsdk_->runParse();
    static pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    // RCLCPP_INFO(this->get_logger(), "result = %d", result);

    if (result == LIDAR_IMU_DATA_PACKET_TYPE)
    {
        LidarImuData imu;
        lsdk_->getImuData(imu);

        if (lsdk_->getImuData(imu))
        {
            // publish imu message
            rclcpp::Time timestamp(imu.info.stamp.sec, imu.info.stamp.nsec);

            sensor_msgs::msg::Imu imuMsg;
            imuMsg.header.frame_id = imu_frame_;
            imuMsg.header.stamp = timestamp;

            imuMsg.orientation.x = imu.quaternion[0];
            imuMsg.orientation.y = imu.quaternion[1];
            imuMsg.orientation.z = imu.quaternion[2];
            imuMsg.orientation.w = imu.quaternion[3];

            imuMsg.angular_velocity.x = imu.angular_velocity[0];
            imuMsg.angular_velocity.y = imu.angular_velocity[1];
            imuMsg.angular_velocity.z = imu.angular_velocity[2];

            imuMsg.linear_acceleration.x = imu.linear_acceleration[0];
            imuMsg.linear_acceleration.y = imu.linear_acceleration[1];
            imuMsg.linear_acceleration.z = imu.linear_acceleration[2];

            pub_imu_->publish(imuMsg);

            // publish tf from initial imu to real-time imu
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = this->now(); // 使用当前时间
            transformStamped.header.frame_id = imu_frame_ + "_initial"; // 父坐标系
            transformStamped.child_frame_id = imu_frame_; // 子坐标系
            transformStamped.transform.translation.x = 0;
            transformStamped.transform.translation.y = 0;
            transformStamped.transform.translation.z = 0;
            transformStamped.transform.rotation.x = imu.quaternion[1];
            transformStamped.transform.rotation.y = imu.quaternion[2];
            transformStamped.transform.rotation.z = imu.quaternion[3];
            transformStamped.transform.rotation.w = imu.quaternion[0];
            broadcaster_->sendTransform(transformStamped);

            // publish tf from imu to lidar
            transformStamped.header.frame_id = imu_frame_; // 父坐标系
            transformStamped.child_frame_id = cloud_frame_; // 子坐标系
            transformStamped.transform.translation.x = 0.007698;
            transformStamped.transform.translation.y = 0.014655;
            transformStamped.transform.translation.z = -0.00667;
            transformStamped.transform.rotation.x = 0;
            transformStamped.transform.rotation.y = 0;
            transformStamped.transform.rotation.z = 0;
            transformStamped.transform.rotation.w = 1;
            broadcaster_->sendTransform(transformStamped);
        }
    }
    else if (result == LIDAR_POINT_DATA_PACKET_TYPE)
    {
        // RCLCPP_INFO(this->get_logger(), "POINT_CLOUD");
        PointCloudUnitree cloud;
        if (lsdk_->getPointCloud(cloud))
        {
            savePointCloudToTxt(cloud);

            transformUnitreeCloudToPCL(cloud, cloudOut);

            rclcpp::Time timestamp(
                static_cast<int32_t>(cloud.stamp),
                static_cast<uint32_t>((cloud.stamp - static_cast<int32_t>(cloud.stamp)) * 1e9));

            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloudOut, cloud_msg);
            cloud_msg.header.frame_id = cloud_frame_;
            cloud_msg.header.stamp = timestamp;

            pub_cloud_->publish(cloud_msg);
        }
    }
}

void UnitreeLidarSDKNode::savePointCloudToTxt(const PointCloudUnitree &cloud)
{
<<<<<<< HEAD
    if (!save_cloud_txt_ || !cloud_txt_stream_.is_open())
=======
    if (!save_cloud_txt_)
>>>>>>> test1
    {
        return;
    }

    cloud_txt_frame_count_++;
    if ((cloud_txt_frame_count_ - 1) % static_cast<uint64_t>(cloud_txt_save_every_n_) != 0)
    {
        return;
    }

<<<<<<< HEAD
    cloud_txt_stream_ << "# frame " << cloud_txt_frame_count_
                      << " stamp " << std::fixed << std::setprecision(9) << cloud.stamp
                      << " id " << cloud.id
                      << " points " << cloud.points.size()
                      << " ring_num " << cloud.ringNum << '\n';

    cloud_txt_stream_ << std::fixed << std::setprecision(9);
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        const PointUnitree &point = cloud.points[i];
        cloud_txt_stream_ << cloud.stamp << ' '
                          << cloud.id << ' '
                          << i << ' '
                          << point.x << ' '
                          << point.y << ' '
                          << point.z << ' '
                          << point.intensity << ' '
                          << point.time << ' '
                          << point.ring << '\n';
    }

    if (!cloud_txt_stream_)
    {
        RCLCPP_ERROR(this->get_logger(), "failed while writing point cloud txt file: %s", cloud_txt_path_.c_str());
        save_cloud_txt_ = false;
    }
}
=======
    std::string file_path = cloud_txt_path_;
    if (cloud_txt_save_mode_ == "separate_files")
    {
        std::ostringstream file_name;
        file_name << "cloud_" << std::setw(9) << std::setfill('0') << cloud_txt_frame_count_ << ".txt";
        file_path = (std::filesystem::path(cloud_txt_dir_) / file_name.str()).string();
    }

    if (!writePointCloudTxtFile(file_path, cloud, cloud_txt_frame_count_))
    {
        RCLCPP_ERROR(this->get_logger(), "failed to write point cloud txt file: %s", file_path.c_str());
        save_cloud_txt_ = false;
    }
}

bool UnitreeLidarSDKNode::writePointCloudTxtFile(
    const std::string &file_path,
    const PointCloudUnitree &cloud,
    uint64_t frame_count)
{
    std::ofstream file(file_path, std::ios::out | std::ios::trunc);
    if (!file.is_open())
    {
        return false;
    }

    file << "# Unitree L2 point cloud txt\n";
    file << "# columns: cloud_stamp cloud_id point_index x y z intensity relative_time ring\n";
    file << "# frame " << frame_count
         << " stamp " << std::fixed << std::setprecision(9) << cloud.stamp
         << " id " << cloud.id
         << " points " << cloud.points.size()
         << " ring_num " << cloud.ringNum << '\n';

    file << std::fixed << std::setprecision(9);
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        const PointUnitree &point = cloud.points[i];
        file << cloud.stamp << ' '
             << cloud.id << ' '
             << i << ' '
             << point.x << ' '
             << point.y << ' '
             << point.z << ' '
             << point.intensity << ' '
             << point.time << ' '
             << point.ring << '\n';
    }

    return static_cast<bool>(file);
}
>>>>>>> test1

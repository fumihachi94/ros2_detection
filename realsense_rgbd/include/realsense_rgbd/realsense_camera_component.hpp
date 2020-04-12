/* Copyright 2020 Fumiya Sato 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MY_REALSENSE__MY_REALSENSE_CAMERA_NODE_COMPONENT_HPP_
#define MY_REALSENSE__MY_REALSENSE_CAMERA_NODE_COMPONENT_HPP_

// cpplint: c system headers
#include <eigen3/Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <console_bridge/console.h>
#include <opencv2/imgproc/imgproc.hpp>     // 追加：2020.3.16
#include <opencv2/highgui/highgui.hpp>     // 追加：2020.3.16
#include <opencv2/objdetect/objdetect.hpp> // 追加：2020.3.16
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
// cpplint: c++ system headers
#include <algorithm>
#include <csignal>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
// cpplint: other headers
//#include "my_realsense_ros2_camera/visibility_control.h"
#include "realsense_rgbd/constants.hpp"
#include "realsense_camera_msgs/msg/imu_info.hpp"
#include "realsense_camera_msgs/msg/extrinsics.hpp"

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION \
                                                           : REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros2_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;
using realsense_camera_msgs::msg::Extrinsics;
using realsense_camera_msgs::msg::IMUInfo;

namespace realsense_rgbd
{
using stream_index_pair = std::pair<rs2_stream, int>;

const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};

const std::vector<std::vector<stream_index_pair>> IMAGE_STREAMS = {{{DEPTH, INFRA1, INFRA2},
                                                                    {COLOR},
                                                                    {FISHEYE}}};

const std::vector<std::vector<stream_index_pair>> HID_STREAMS = {{GYRO, ACCEL}};

class PipelineSyncer : public rs2::asynchronous_syncer
{
public:
    void operator()(rs2::frame f) const
    {
        invoke(std::move(f));
    }
};

class RealSenseCameraComponent : public rclcpp::Node
{
public:
  //COMPOSITION_PUBLIC
  explicit RealSenseCameraComponent(const rclcpp::NodeOptions& options);
  //virtual ~RealSenseCameraComponent(){};
  virtual void onInit();

private:
  void getParameters();

  void setupDevice();

  void setupPublishers();

  void setupStreams();

  void updateStreamCalibData(const rs2::video_stream_profile & video_profile);

  Eigen::Quaternionf rotationMatrixToQuaternion(float rotation[9]) const;

  void publishStaticTransforms();

  void alignFrame(
    const rs2_intrinsics & from_intrin,
    const rs2_intrinsics & other_intrin,
    rs2::frame             from_image,
    uint32_t               output_image_bytes_per_pixel,
    const rs2_extrinsics & from_to_other,
    std::vector<uint8_t> & out_vec);
  
  rs2_extrinsics getRsExtrinsics(
    const stream_index_pair & from_stream,
    const stream_index_pair & to_stream);

  void publishAlignedDepthImg(rs2::frame depth_frame, const rclcpp::Time & t);

  void publishPCTopic(const rclcpp::Time & t);

  void publishAlignedPCTopic(const rclcpp::Time & t);

  Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics & extrinsics) const;

  Extrinsics getFisheye2ImuExtrinsicsMsg();

  Extrinsics getFisheye2DepthExtrinsicsMsg();

  IMUInfo getImuInfo(const stream_index_pair & stream_index);

  void tryGetLogSeverity(rs2_log_severity & severity) const;

  void publishFrame(rs2::frame f, const rclcpp::Time & t);

  bool getEnabledProfile(
    const stream_index_pair & stream_index, 
    rs2::stream_profile & profile);

  struct float3
  {
    float x, y, z;
  };

  rclcpp::Clock _ros_clock;
  std::unique_ptr<rs2::context> _ctx;

  std::map<stream_index_pair, std::unique_ptr<rs2::sensor>> _sensors;

  std::string _serial_no;
  float _depth_scale_meters;

  std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
  std::map<stream_index_pair, int> _width;
  std::map<stream_index_pair, int> _height;
  std::map<stream_index_pair, int> _fps;
  std::map<stream_index_pair, bool> _enable;
  std::map<stream_index_pair, std::string> _stream_name;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster_;

  std::map<stream_index_pair, image_transport::Publisher> _image_publishers;
  std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> _imu_publishers;
  std::map<stream_index_pair, int> _image_format;
  std::map<stream_index_pair, rs2_format> _format;
  std::map<stream_index_pair,
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _info_publisher;
  std::map<stream_index_pair,
    rclcpp::Publisher<realsense_camera_msgs::msg::IMUInfo>::SharedPtr> _imu_info_publisher;
  std::map<stream_index_pair, cv::Mat> _image;
  std::map<stream_index_pair, std::string> _encoding;

  std::string _base_frame_id;
  std::map<stream_index_pair, std::string> _frame_id;
  std::map<stream_index_pair, std::string> _optical_frame_id;
  std::map<stream_index_pair, int> _seq;
  std::map<stream_index_pair, int> _unit_step_size;
  std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> _camera_info;
  rclcpp::Publisher<realsense_camera_msgs::msg::Extrinsics>::SharedPtr _fe_to_depth_publisher,
    _fe_to_imu_publisher;

  rclcpp::QoS qos;
  bool _intialize_time_base;
  double _camera_time_base;
  std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;

  image_transport::Publisher _align_depth_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _align_depth_camera_publisher;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _align_pointcloud_publisher;

  rclcpp::Time _ros_time_base;
  rclcpp::Logger logger_ = rclcpp::get_logger("realsense_camera_component");
  rclcpp::TimerBase::SharedPtr timer_;
  bool _sync_frames;
  bool _pointcloud;
  bool _align_pointcloud;
  bool _align_depth;
  PipelineSyncer _syncer;
  rs2_extrinsics _depth2color_extrinsics;

  rs2::frameset _aligned_frameset;
};
}  // namespace realsense_rgbd

#endif  //MY_REALSENSE__MY_REALSENSE_CAMERA_NODE_COMPONENT_HPP_

#ifndef TRACER_PIPER_COLOR_GRASP__COLOR_POSE_ESTIMATOR_HPP_
#define TRACER_PIPER_COLOR_GRASP__COLOR_POSE_ESTIMATOR_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"

namespace tracer_piper_color_grasp
{

struct ColorDetection
{
  geometry_msgs::msg::PointStamped surface_point;
  double confidence{0.0};
  double depth_m{0.0};
  double contour_area{0.0};
  int pixel_u{0};
  int pixel_v{0};
  int64_t stamp_nanoseconds{0};
};

class ColorPoseEstimator
{
public:
  ColorPoseEstimator(rclcpp::Node * node, tf2_ros::Buffer * tf_buffer);

  bool waitForStableDetection(
    const std::string & color,
    const std::function<bool()> & stop_requested,
    ColorDetection & detection,
    std::string & error);

  bool cameraReady() const;

private:
  struct HsvRange
  {
    cv::Scalar lower;
    cv::Scalar upper;
  };

  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  std::optional<ColorDetection> detectOnce(
    const std::string & color, std::string & error);
  cv::Mat makeColorMask(const cv::Mat & hsv, const std::string & color) const;
  std::optional<double> medianDepth(
    const cv::Mat & depth, const cv::Mat & mask,
    const cv::Rect & bounds) const;
  std::vector<int64_t> integerArrayParameter(
    const std::string & name, const std::vector<int64_t> & defaults);
  HsvRange makeRange(const std::vector<int64_t> & values) const;
  void publishDebugImage(
    const cv::Mat & image, const std_msgs::msg::Header & header) const;

  rclcpp::Node * node_;
  tf2_ros::Buffer * tf_buffer_;

  rclcpp::Subscription<Image>::SharedPtr rgb_subscription_;
  rclcpp::Subscription<Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<Image>::SharedPtr debug_image_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;

  mutable std::mutex sensor_mutex_;
  Image::ConstSharedPtr latest_rgb_;
  Image::ConstSharedPtr latest_depth_;
  CameraInfo::ConstSharedPtr latest_camera_info_;

  std::map<std::string, std::vector<HsvRange>> hsv_ranges_;
  std::string reference_frame_;
  std::string optical_frame_;
  double min_contour_area_;
  double max_contour_area_;
  double min_rectangularity_;
  double min_aspect_ratio_;
  double max_aspect_ratio_;
  double min_depth_m_;
  double max_depth_m_;
  double sync_tolerance_s_;
  double tf_timeout_s_;
  double detection_timeout_s_;
  double stability_tolerance_m_;
  int stable_frames_;
  int min_depth_samples_;
  int morphology_kernel_;
};

}  // namespace tracer_piper_color_grasp

#endif  // TRACER_PIPER_COLOR_GRASP__COLOR_POSE_ESTIMATOR_HPP_

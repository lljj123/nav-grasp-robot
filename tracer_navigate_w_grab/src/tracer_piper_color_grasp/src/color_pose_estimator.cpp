#include "tracer_piper_color_grasp/color_pose_estimator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <sstream>
#include <thread>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tracer_piper_color_grasp
{

namespace
{

template<typename T>
T parameterOrDeclare(
  rclcpp::Node * node, const std::string & name, const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

double pointDistance(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

ColorPoseEstimator::ColorPoseEstimator(
  rclcpp::Node * node, tf2_ros::Buffer * tf_buffer)
: node_(node),
  tf_buffer_(tf_buffer),
  reference_frame_(parameterOrDeclare<std::string>(node_,
      "pose_reference_frame", "piper_base_link")),
  optical_frame_(parameterOrDeclare<std::string>(node_,
      "camera_optical_frame", "wrist_camera_color_optical_frame")),
  min_contour_area_(parameterOrDeclare<double>(node_,
      "vision.min_contour_area", 250.0)),
  max_contour_area_(parameterOrDeclare<double>(node_,
      "vision.max_contour_area", 120000.0)),
  min_rectangularity_(parameterOrDeclare<double>(node_,
      "vision.min_rectangularity", 0.55)),
  min_aspect_ratio_(parameterOrDeclare<double>(node_,
      "vision.min_aspect_ratio", 0.50)),
  max_aspect_ratio_(parameterOrDeclare<double>(node_,
      "vision.max_aspect_ratio", 2.00)),
  min_depth_m_(parameterOrDeclare<double>(node_,
      "vision.min_depth_m", 0.10)),
  max_depth_m_(parameterOrDeclare<double>(node_,
      "vision.max_depth_m", 1.20)),
  sync_tolerance_s_(parameterOrDeclare<double>(node_,
      "vision.sync_tolerance_s", 0.15)),
  tf_timeout_s_(parameterOrDeclare<double>(node_,
      "vision.tf_timeout_s", 0.30)),
  detection_timeout_s_(parameterOrDeclare<double>(node_,
      "vision.detection_timeout_s", 4.0)),
  stability_tolerance_m_(parameterOrDeclare<double>(node_,
      "vision.stability_tolerance_m", 0.012)),
  stable_frames_(parameterOrDeclare<int>(node_,
      "vision.stable_frames", 4)),
  min_depth_samples_(parameterOrDeclare<int>(node_,
      "vision.min_depth_samples", 25)),
  morphology_kernel_(parameterOrDeclare<int>(node_,
      "vision.morphology_kernel", 5))
{
  min_contour_area_ = std::max(1.0, min_contour_area_);
  max_contour_area_ = std::max(min_contour_area_, max_contour_area_);
  min_depth_m_ = std::max(0.01, min_depth_m_);
  max_depth_m_ = std::max(min_depth_m_, max_depth_m_);
  sync_tolerance_s_ = std::max(0.001, sync_tolerance_s_);
  tf_timeout_s_ = std::max(0.01, tf_timeout_s_);
  detection_timeout_s_ = std::max(0.1, detection_timeout_s_);
  stability_tolerance_m_ = std::max(0.0001, stability_tolerance_m_);
  stable_frames_ = std::max(1, stable_frames_);
  min_depth_samples_ = std::max(1, min_depth_samples_);

  hsv_ranges_["red"] = {
    makeRange(integerArrayParameter(
        "vision.red_hsv_low", {0, 100, 70, 10, 255, 255})),
    makeRange(integerArrayParameter(
        "vision.red_hsv_high", {170, 100, 70, 179, 255, 255}))
  };
  hsv_ranges_["yellow"] = {
    makeRange(integerArrayParameter(
        "vision.yellow_hsv", {18, 90, 70, 40, 255, 255}))
  };
  hsv_ranges_["green"] = {
    makeRange(integerArrayParameter(
        "vision.green_hsv", {40, 65, 45, 95, 255, 255}))
  };

  const auto rgb_topic = parameterOrDeclare<std::string>(node_,
    "camera_image_topic", "/wrist_camera/image");
  const auto depth_topic = parameterOrDeclare<std::string>(node_,
    "camera_depth_topic", "/wrist_camera/depth_image");
  const auto camera_info_topic = parameterOrDeclare<std::string>(node_,
    "camera_info_topic", "/wrist_camera/camera_info");
  const auto debug_topic = parameterOrDeclare<std::string>(node_,
    "debug_image_topic", "/color_grasp/debug_image");
  const auto target_topic = parameterOrDeclare<std::string>(node_,
    "target_pose_topic", "/color_grasp/target_pose");

  rgb_subscription_ = node_->create_subscription<Image>(
    rgb_topic, rclcpp::SensorDataQoS(),
    [this](Image::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      latest_rgb_ = std::move(message);
    });
  depth_subscription_ = node_->create_subscription<Image>(
    depth_topic, rclcpp::SensorDataQoS(),
    [this](Image::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      latest_depth_ = std::move(message);
    });
  camera_info_subscription_ = node_->create_subscription<CameraInfo>(
    camera_info_topic, rclcpp::SensorDataQoS(),
    [this](CameraInfo::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      latest_camera_info_ = std::move(message);
    });

  debug_image_publisher_ = node_->create_publisher<Image>(
    debug_topic, rclcpp::SensorDataQoS());
  target_pose_publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    target_topic, rclcpp::QoS(10));
}

std::vector<int64_t> ColorPoseEstimator::integerArrayParameter(
  const std::string & name, const std::vector<int64_t> & defaults)
{
  const auto values =
    parameterOrDeclare<std::vector<int64_t>>(node_, name, defaults);
  if (values.size() != 6U) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Parameter '%s' must contain six integers; using defaults.",
      name.c_str());
    return defaults;
  }
  return values;
}

ColorPoseEstimator::HsvRange ColorPoseEstimator::makeRange(
  const std::vector<int64_t> & values) const
{
  return {
    cv::Scalar(values[0], values[1], values[2]),
    cv::Scalar(values[3], values[4], values[5])
  };
}

bool ColorPoseEstimator::cameraReady() const
{
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  return latest_rgb_ && latest_depth_ && latest_camera_info_;
}

cv::Mat ColorPoseEstimator::makeColorMask(
  const cv::Mat & hsv, const std::string & color) const
{
  cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
  const auto ranges = hsv_ranges_.find(color);
  if (ranges == hsv_ranges_.end()) {
    return mask;
  }

  for (const auto & range : ranges->second) {
    cv::Mat partial;
    cv::inRange(hsv, range.lower, range.upper, partial);
    cv::bitwise_or(mask, partial, mask);
  }

  int kernel_size = std::max(1, morphology_kernel_);
  if ((kernel_size % 2) == 0) {
    ++kernel_size;
  }
  const auto kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  return mask;
}

std::optional<double> ColorPoseEstimator::medianDepth(
  const cv::Mat & depth, const cv::Mat & mask,
  const cv::Rect & bounds) const
{
  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>(bounds.area()));

  const auto image_bounds = cv::Rect(0, 0, depth.cols, depth.rows);
  const auto clipped = bounds & image_bounds;
  for (int row = clipped.y; row < clipped.y + clipped.height; ++row) {
    for (int column = clipped.x; column < clipped.x + clipped.width; ++column) {
      if (mask.at<uint8_t>(row, column) == 0U) {
        continue;
      }

      double value_m = std::numeric_limits<double>::quiet_NaN();
      if (depth.type() == CV_32FC1) {
        value_m = static_cast<double>(depth.at<float>(row, column));
      } else if (depth.type() == CV_16UC1) {
        value_m =
          static_cast<double>(depth.at<uint16_t>(row, column)) / 1000.0;
      }

      if (std::isfinite(value_m) &&
        value_m >= min_depth_m_ && value_m <= max_depth_m_)
      {
        samples.push_back(value_m);
      }
    }
  }

  if (samples.size() < static_cast<std::size_t>(min_depth_samples_)) {
    return std::nullopt;
  }
  const auto middle = samples.begin() +
    static_cast<std::ptrdiff_t>(samples.size() / 2U);
  std::nth_element(samples.begin(), middle, samples.end());
  return *middle;
}

void ColorPoseEstimator::publishDebugImage(
  const cv::Mat & image, const std_msgs::msg::Header & header) const
{
  auto message = cv_bridge::CvImage(
    header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();
  debug_image_publisher_->publish(*message);
}

std::optional<ColorDetection> ColorPoseEstimator::detectOnce(
  const std::string & color, std::string & error)
{
  Image::ConstSharedPtr rgb;
  Image::ConstSharedPtr depth;
  CameraInfo::ConstSharedPtr camera_info;
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    rgb = latest_rgb_;
    depth = latest_depth_;
    camera_info = latest_camera_info_;
  }

  if (!rgb || !depth || !camera_info) {
    error = "等待腕部 RGB、深度图和 CameraInfo";
    return std::nullopt;
  }

  const auto rgb_stamp = stampToNanoseconds(rgb->header.stamp);
  const auto depth_stamp = stampToNanoseconds(depth->header.stamp);
  const double stamp_difference =
    std::abs(static_cast<double>(rgb_stamp - depth_stamp)) / 1.0e9;
  if (stamp_difference > sync_tolerance_s_) {
    error = "RGB 与深度图时间差过大";
    return std::nullopt;
  }

  cv::Mat bgr;
  cv::Mat depth_image;
  try {
    bgr = cv_bridge::toCvCopy(
      rgb, sensor_msgs::image_encodings::BGR8)->image;
    depth_image = cv_bridge::toCvShare(depth)->image;
  } catch (const cv_bridge::Exception & exception) {
    error = std::string("图像编码转换失败：") + exception.what();
    return std::nullopt;
  }

  if (bgr.empty() || depth_image.empty() ||
    bgr.size() != depth_image.size())
  {
    error = "RGB 与深度图尺寸不一致";
    return std::nullopt;
  }
  if (depth_image.type() != CV_32FC1 &&
    depth_image.type() != CV_16UC1)
  {
    error = "深度图仅支持 32FC1 或 16UC1";
    return std::nullopt;
  }

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::Mat color_mask = makeColorMask(hsv, color);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(
    color_mask.clone(), contours,
    cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  int best_index = -1;
  double best_area = 0.0;
  cv::Rect best_bounds;
  for (std::size_t index = 0; index < contours.size(); ++index) {
    const double area = cv::contourArea(contours[index]);
    if (area < min_contour_area_ || area > max_contour_area_) {
      continue;
    }
    const auto bounds = cv::boundingRect(contours[index]);
    if (bounds.height <= 0 || bounds.width <= 0) {
      continue;
    }
    const double aspect =
      static_cast<double>(bounds.width) / static_cast<double>(bounds.height);
    const double rectangularity =
      area / static_cast<double>(bounds.width * bounds.height);
    if (aspect < min_aspect_ratio_ || aspect > max_aspect_ratio_ ||
      rectangularity < min_rectangularity_)
    {
      continue;
    }
    if (area > best_area) {
      best_index = static_cast<int>(index);
      best_area = area;
      best_bounds = bounds;
    }
  }

  if (best_index < 0) {
    cv::putText(
      bgr, "No " + color + " cube", cv::Point(20, 35),
      cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    publishDebugImage(bgr, rgb->header);
    error = "未找到符合面积和形状条件的目标";
    return std::nullopt;
  }

  cv::Mat contour_mask = cv::Mat::zeros(color_mask.size(), CV_8UC1);
  cv::drawContours(
    contour_mask, contours, best_index, cv::Scalar(255), cv::FILLED);
  const auto depth_m = medianDepth(depth_image, contour_mask, best_bounds);
  if (!depth_m.has_value()) {
    error = "目标区域没有足够的有效深度样本";
    return std::nullopt;
  }

  const auto moments = cv::moments(contours[best_index]);
  if (std::abs(moments.m00) < 1.0e-6) {
    error = "目标轮廓中心计算失败";
    return std::nullopt;
  }
  const int center_u = static_cast<int>(std::lround(moments.m10 / moments.m00));
  const int center_v = static_cast<int>(std::lround(moments.m01 / moments.m00));

  const double fx = camera_info->k[0];
  const double fy = camera_info->k[4];
  const double cx = camera_info->k[2];
  const double cy = camera_info->k[5];
  if (fx <= 0.0 || fy <= 0.0) {
    error = "CameraInfo 中的焦距无效";
    return std::nullopt;
  }

  geometry_msgs::msg::PointStamped optical_point;
  optical_point.header = rgb->header;
  if (optical_point.header.frame_id.empty()) {
    optical_point.header.frame_id = optical_frame_;
  }
  optical_point.point.z = *depth_m;
  optical_point.point.x =
    (static_cast<double>(center_u) - cx) * (*depth_m) / fx;
  optical_point.point.y =
    (static_cast<double>(center_v) - cy) * (*depth_m) / fy;

  geometry_msgs::msg::PointStamped reference_point;
  try {
    reference_point = tf_buffer_->transform(
      optical_point, reference_frame_,
      tf2::durationFromSec(tf_timeout_s_));
  } catch (const tf2::TransformException & exception) {
    error = std::string("目标坐标 TF 转换失败：") + exception.what();
    return std::nullopt;
  }

  // cv::drawContours(
  //   bgr, contours, best_index, cv::Scalar(255, 255, 255), 2);
  // cv::rectangle(bgr, best_bounds, cv::Scalar(255, 255, 255), 2);
  // cv::circle(
  //   bgr, cv::Point(center_u, center_v), 5,
  //   cv::Scalar(255, 255, 255), cv::FILLED);
  // std::ostringstream label;
  // label.precision(3);
  // label << std::fixed << color << "  z=" << *depth_m << "m";
  // cv::putText(
  //   bgr, label.str(),
  //   cv::Point(best_bounds.x, std::max(25, best_bounds.y - 8)),
  //   cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2);
  // publishDebugImage(bgr, rgb->header);



  // 目标轮廓和边界框
cv::drawContours(
  bgr, contours, best_index,
  cv::Scalar(0, 255, 0), 2);

cv::rectangle(
  bgr, best_bounds,
  cv::Scalar(0, 255, 0), 3);

// 目标中心十字
const cv::Point center(center_u, center_v);

cv::circle(
  bgr, center, 6,
  cv::Scalar(0, 0, 255), cv::FILLED);

cv::line(
  bgr,
  cv::Point(center_u - 15, center_v),
  cv::Point(center_u + 15, center_v),
  cv::Scalar(0, 0, 255), 2);

cv::line(
  bgr,
  cv::Point(center_u, center_v - 15),
  cv::Point(center_u, center_v + 15),
  cv::Scalar(0, 0, 255), 2);

// 左上角信息面板
cv::rectangle(
  bgr,
  cv::Point(8, 8),
  cv::Point(630, 168),
  cv::Scalar(20, 20, 20),
  cv::FILLED);

auto draw_text =
  [&bgr](const std::string & text, int row,
  const cv::Scalar & color_value)
  {
    cv::putText(
      bgr,
      text,
      cv::Point(20, row),
      cv::FONT_HERSHEY_SIMPLEX,
      0.58,
      color_value,
      2,
      cv::LINE_AA);
  };

std::ostringstream line1;
line1.precision(3);
line1 << std::fixed
      << "Target: " << color
      << "  bbox=[" << best_bounds.x
      << "," << best_bounds.y
      << "," << best_bounds.width
      << "," << best_bounds.height << "]";

std::ostringstream line2;
line2.precision(3);
line2 << std::fixed
      << "Pixel: u=" << center_u
      << " v=" << center_v
      << "  depth=" << *depth_m << " m";

std::ostringstream line3;
line3.precision(3);
line3 << std::fixed
      << "Camera XYZ: "
      << optical_point.point.x << ", "
      << optical_point.point.y << ", "
      << optical_point.point.z << " m";

std::ostringstream line4;
line4.precision(3);
line4 << std::fixed
      << reference_frame_ << " XYZ: "
      << reference_point.point.x << ", "
      << reference_point.point.y << ", "
      << reference_point.point.z << " m";

std::ostringstream line5;
line5.precision(1);
line5 << std::fixed
      << "Area: " << best_area
      << " px  RGB-depth dt: "
      << stamp_difference * 1000.0 << " ms";

draw_text(line1.str(), 35, cv::Scalar(0, 255, 0));
draw_text(line2.str(), 65, cv::Scalar(255, 255, 255));
draw_text(line3.str(), 95, cv::Scalar(255, 220, 80));
draw_text(line4.str(), 125, cv::Scalar(80, 220, 255));
draw_text(line5.str(), 155, cv::Scalar(200, 200, 200));

publishDebugImage(bgr, rgb->header);

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header = reference_point.header;
  target_pose.pose.position = reference_point.point;
  target_pose.pose.orientation.w = 1.0;
  target_pose_publisher_->publish(target_pose);

  ColorDetection detection;
  detection.surface_point = reference_point;
  detection.depth_m = *depth_m;
  detection.contour_area = best_area;
  detection.pixel_u = center_u;
  detection.pixel_v = center_v;
  detection.stamp_nanoseconds = rgb_stamp;
  const double area_score = std::min(1.0, best_area / 1500.0);
  const double sync_score =
    std::max(0.0, 1.0 - stamp_difference / sync_tolerance_s_);
  detection.confidence = 0.5 * area_score + 0.5 * sync_score;
  error.clear();
  return detection;
}

bool ColorPoseEstimator::waitForStableDetection(
  const std::string & color,
  const std::function<bool()> & stop_requested,
  ColorDetection & detection,
  std::string & error)
{
  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::duration<double>(detection_timeout_s_);
  std::vector<ColorDetection> recent;
  int64_t previous_stamp = -1;

  while (std::chrono::steady_clock::now() < deadline) {
    if (stop_requested()) {
      error = "任务已取消或急停";
      return false;
    }

    auto candidate = detectOnce(color, error);
    if (candidate.has_value() &&
      candidate->stamp_nanoseconds != previous_stamp)
    {
      previous_stamp = candidate->stamp_nanoseconds;
      recent.push_back(*candidate);
      if (recent.size() > static_cast<std::size_t>(stable_frames_)) {
        recent.erase(recent.begin());
      }

      if (recent.size() == static_cast<std::size_t>(stable_frames_)) {
        geometry_msgs::msg::Point mean;
        for (const auto & item : recent) {
          mean.x += item.surface_point.point.x;
          mean.y += item.surface_point.point.y;
          mean.z += item.surface_point.point.z;
        }
        const double count = static_cast<double>(recent.size());
        mean.x /= count;
        mean.y /= count;
        mean.z /= count;

        double maximum_distance = 0.0;
        double confidence_sum = 0.0;
        for (const auto & item : recent) {
          maximum_distance = std::max(
            maximum_distance, pointDistance(item.surface_point.point, mean));
          confidence_sum += item.confidence;
        }
        if (maximum_distance <= stability_tolerance_m_) {
          detection = recent.back();
          detection.surface_point.point = mean;
          detection.confidence = confidence_sum / count;
          error.clear();
          return true;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (error.empty()) {
    error = "目标检测超时";
  } else {
    error = "目标检测超时：" + error;
  }
  return false;
}

}  // namespace tracer_piper_color_grasp

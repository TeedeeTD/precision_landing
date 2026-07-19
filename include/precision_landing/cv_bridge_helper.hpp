#ifndef PRECISION_LANDING__CV_BRIDGE_HELPER_HPP_
#define PRECISION_LANDING__CV_BRIDGE_HELPER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstring>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace precision_landing
{

inline cv::Mat imageMsgToMat(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  int type = -1;
  if (msg->encoding == "mono8") {
    type = CV_8UC1;
  } else if (msg->encoding == "bgr8") {
    type = CV_8UC3;
  } else if (msg->encoding == "rgb8") {
    type = CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported image encoding in cv_bridge_helper: " + msg->encoding);
  }

  // Create Mat pointing directly to the message data (zero-copy wrapper)
  return cv::Mat(msg->height, msg->width, type, const_cast<uint8_t*>(&msg->data[0]), msg->step);
}

inline cv::Mat imageMsgToMat(const sensor_msgs::msg::Image::SharedPtr& msg)
{
  int type = -1;
  if (msg->encoding == "mono8") {
    type = CV_8UC1;
  } else if (msg->encoding == "bgr8") {
    type = CV_8UC3;
  } else if (msg->encoding == "rgb8") {
    type = CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported image encoding in cv_bridge_helper: " + msg->encoding);
  }

  return cv::Mat(msg->height, msg->width, type, &msg->data[0], msg->step);
}

inline sensor_msgs::msg::Image::UniquePtr matToImageMsg(
  const cv::Mat& mat,
  const std_msgs::msg::Header& header,
  const std::string& encoding)
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header = header;
  msg->height = mat.rows;
  msg->width = mat.cols;
  msg->encoding = encoding;
  msg->is_bigendian = false;
  msg->step = mat.step;
  size_t size = mat.step * mat.rows;
  msg->data.resize(size);
  std::memcpy(&msg->data[0], mat.data, size);
  return msg;
}

} // namespace precision_landing

#endif // PRECISION_LANDING__CV_BRIDGE_HELPER_HPP_

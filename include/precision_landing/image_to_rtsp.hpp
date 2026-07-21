#ifndef PRECISION_LANDING__IMAGE_TO_RTSP_HPP_
#define PRECISION_LANDING__IMAGE_TO_RTSP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <string>

namespace precision_landing
{

class ImageToRtsp : public rclcpp::Node
{
public:
  ImageToRtsp(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ImageToRtsp();

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // Parameters
  std::string image_topic_;
  std::string rtsp_url_;
  double fps_;

  // ROS Subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // OpenCV VideoWriter
  cv::VideoWriter writer_;

  int frame_count_{0};
};

} // namespace precision_landing

#endif // PRECISION_LANDING__IMAGE_TO_RTSP_HPP_

#include "precision_landing/image_to_rtsp.hpp"
#include "precision_landing/cv_bridge_helper.hpp"
#include <chrono>

namespace precision_landing
{

ImageToRtsp::ImageToRtsp(const rclcpp::NodeOptions & options)
: Node("image_to_rtsp", options)
{
  // Declare Parameters
  this->declare_parameter<std::string>("image_topic", "/siyi/fractal_debug");
  this->declare_parameter<std::string>("rtsp_url", "rtmp://127.0.0.1:1935/siyi_aruco");
  this->declare_parameter<double>("fps", 25.0);

  // Get Parameters
  image_topic_ = this->get_parameter("image_topic").as_string();
  rtsp_url_ = this->get_parameter("rtsp_url").as_string();
  fps_ = this->get_parameter("fps").as_double();

  // Create Subscriber
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    10,
    std::bind(&ImageToRtsp::image_callback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(
    this->get_logger(),
    "ImageToRtsp started: sub=%s, push=%s, fps=%.1f",
    image_topic_.c_str(), rtsp_url_.c_str(), fps_
  );
}

ImageToRtsp::~ImageToRtsp()
{
  if (writer_.isOpened()) {
    writer_.release();
    RCLCPP_INFO(this->get_logger(), "VideoWriter released");
  }
}

void ImageToRtsp::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  try {
    cv::Mat frame = precision_landing::imageMsgToMat(msg);
    if (frame.empty()) {
      return;
    }

    // Convert grayscale to BGR if necessary
    cv::Mat bgr_frame;
    if (frame.channels() == 1) {
      cv::cvtColor(frame, bgr_frame, cv::COLOR_GRAY2BGR);
    } else {
      bgr_frame = frame;
    }

    // Initialize VideoWriter on first frame
    if (!writer_.isOpened()) {
      int width = bgr_frame.cols;
      int height = bgr_frame.rows;

      RCLCPP_INFO(
        this->get_logger(),
        "Initializing VideoWriter with resolution %dx%d at %.1f FPS",
        width, height, fps_
      );

      std::string pipeline =
        "appsrc ! videoconvert ! "
        "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! "
        "flvmux streamable=true ! "
        "rtmpsink location=\"" + rtsp_url_ + "\"";

      writer_.open(pipeline, cv::CAP_GSTREAMER, 0, fps_, cv::Size(width, height), true);

      if (!writer_.isOpened()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to open GStreamer RTMP writer pipeline: %s",
          pipeline.c_str()
        );
        return;
      }

      RCLCPP_INFO(this->get_logger(), "GStreamer RTMP writer opened successfully");
    }

    writer_.write(bgr_frame);

    // Periodically print stats
    frame_count_++;
    if (frame_count_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Pushed %d frames to %s", frame_count_, rtsp_url_.c_str());
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in image callback: %s", e.what());
  }
}

} // namespace precision_landing

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(precision_landing::ImageToRtsp)

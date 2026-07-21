#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "precision_landing/image_to_rtsp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<precision_landing::ImageToRtsp>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

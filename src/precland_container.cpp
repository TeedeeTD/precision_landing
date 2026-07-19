#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

int main(int argc, char * argv[])
{
  // Force linker to link OpenCV 4.8.0 and load its symbols at startup
  cv::Mat dummy = cv::Mat::zeros(1, 1, CV_8U);
  (void)dummy;

  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);
  exec->spin();
  rclcpp::shutdown();
  return 0;
}

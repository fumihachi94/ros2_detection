#include <rclcpp/rclcpp.hpp>
#include "vision_detection_msgs/msg/stereo_vision.hpp"
#include <chrono>

int main(int argc, char * argv[]){
  using namespace std::chrono_literals;

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("minimal_publisher");
  auto publisher = node->create_publisher<vision_detection_msgs::msg::StereoVision>("msg_test",10);
  auto message = std::make_shared<vision_detection_msgs::msg::StereoVision>();

  rclcpp::WallRate loop_rate(1s);

  while(rclcpp::ok()){
    message->frame = 1;
    RCLCPP_INFO(node->get_logger(),"Pub:%d",message->frame);
    publisher->publish(*message);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
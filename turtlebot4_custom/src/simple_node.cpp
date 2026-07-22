#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"

class TurtleBot4FirstNode : public rclcpp::Node
{
public:
  TurtleBot4FirstNode()
  : Node("turtlebot4_first_cpp_node"), lights_on_(false)
  {
    // Subscribe to the interface_buttons topic (relative, so a namespaced
    // run reaches that robot's Create 3)
    interface_buttons_subscriber_ =
      this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
      "interface_buttons",
      rclcpp::SensorDataQoS(),
      std::bind(&TurtleBot4FirstNode::interface_buttons_callback, this, std::placeholders::_1));

    // Create a publisher for the cmd_lightring topic
    lightring_publisher_ = this->create_publisher<irobot_create_msgs::msg::LightringLeds>(
      "cmd_lightring",
      rclcpp::SensorDataQoS());
  }

private:
  // Interface buttons subscription callback
  void interface_buttons_callback(
    const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg)
  {
    // Button 1 is pressed
    if (create3_buttons_msg->button_1.is_pressed) {
      RCLCPP_INFO(this->get_logger(), "Button 1 Pressed!");

      button_1_function();
    }
  }

  // Perform a function when Button 1 is pressed.
  void button_1_function()
  {
    // Create a ROS 2 message
    auto lightring_msg = irobot_create_msgs::msg::LightringLeds();
    // Stamp the message with the current time
    lightring_msg.header.stamp = this->get_clock()->now();

    // Lights are currently off
    if (!lights_on_) {
      // Override system lights
      lightring_msg.override_system = true;

      // LED 0
      lightring_msg.leds[0].red = 255;
      lightring_msg.leds[0].blue = 0;
      lightring_msg.leds[0].green = 0;

      // LED 1
      lightring_msg.leds[1].red = 0;
      lightring_msg.leds[1].blue = 255;
      lightring_msg.leds[1].green = 0;

      // LED 2
      lightring_msg.leds[2].red = 0;
      lightring_msg.leds[2].blue = 0;
      lightring_msg.leds[2].green = 255;

      // LED 3
      lightring_msg.leds[3].red = 255;
      lightring_msg.leds[3].blue = 255;
      lightring_msg.leds[3].green = 0;

      // LED 4
      lightring_msg.leds[4].red = 255;
      lightring_msg.leds[4].blue = 0;
      lightring_msg.leds[4].green = 255;

      // LED 5
      lightring_msg.leds[5].red = 0;
      lightring_msg.leds[5].blue = 255;
      lightring_msg.leds[5].green = 255;
    }
    // Lights are currently on
    else {
      // Disable system override. The system will take back control of the lightring.
      lightring_msg.override_system = false;
    }
    // Publish the message
    lightring_publisher_->publish(lightring_msg);
    // Toggle the lights on status
    lights_on_ = !lights_on_;
  }

  // Interface Button Subscriber
  rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr
    interface_buttons_subscriber_;
  // Lightring Publisher
  rclcpp::Publisher<irobot_create_msgs::msg::LightringLeds>::SharedPtr lightring_publisher_;
  // Lights on status
  bool lights_on_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleBot4FirstNode>());
  rclcpp::shutdown();
  return 0;
}

/* MIT License

Copyright (c) 2022 Vignesh Ravichandran Radhakrishnan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cpp_pubsub/srv/modify_string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

// parameter types
using PARAMETER_EVENT = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HANDLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

/**
 * @brief MinimalPublisher class has the publisher and server node. The
 * Publisher node publishes a message and the server modifies the string
 *
 */
class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    try {
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description = "This parameter is updated by given argument in "
                               "launch file and it is "
                               "used by publisher and subscriber node!";
      this->declare_parameter("frequency", 2, param_desc);
      auto frequency =
          this->get_parameter("frequency").get_parameter_value().get<int>();

      RCLCPP_INFO_STREAM(this->get_logger(), "Param value : " << frequency);
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(1000 / frequency)),
          std::bind(&MinimalPublisher::timer_callback, this));

      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Publisher node");
      server = this->create_service<cpp_pubsub::srv::ModifyString>(
          "service_node",
          std::bind(&MinimalPublisher::modify_string, this,
                    std::placeholders::_1, std::placeholders::_2));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Server");
      // tf
      tf_static_broadcaster_ =
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      // Publish static transforms once at startup
      this->make_transforms();

    } catch (...) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Error encountered at time of initialization!!");
      RCLCPP_FATAL_STREAM(this->get_logger(), "Publisher may not work!!");
      RCLCPP_WARN_STREAM(this->get_logger(), "Some error occured !");
    }
  }

  /**
   * @brief Callback function for processing the request of modifying the string
   *
   * @param request : string
   * @param response : string
   */
  void modify_string(
      const std::shared_ptr<cpp_pubsub::srv::ModifyString::Request> request,
      std::shared_ptr<cpp_pubsub::srv::ModifyString::Response> response) {
    response->output =
        request->input + "String Modification from Service - Vignesh";

    server_response_message = response->output;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request to Server\ninput: '%s'",
                request->input.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response is : '%s'",
                response->output.c_str());
  }

private:
  /**
   * @brief Publish the message on topic at a defined rate
   *
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = server_response_message;

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Able to update message data ");
    RCLCPP_INFO(this->get_logger(), "Publishing the message : '%s'",
                message.data.c_str());

    publisher_->publish(message);
  }

  void make_transforms() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "child_robot";

    t.transform.translation.x = 1;
    t.transform.translation.y = 2;
    t.transform.translation.z = 3;
    tf2::Quaternion q;
    q.setRPY(0.1, 0.2, 0.3);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<cpp_pubsub::srv::ModifyString>::SharedPtr server;
  std::string server_response_message = "Hey !! This is Vignesh here ";
  size_t count_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  // RCLCPP_WARN_STREAM(node->get_logger(), "Shutting Down!! " << 4);

  return 0;
}
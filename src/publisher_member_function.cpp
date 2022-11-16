// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpp_pubsub/srv/modify_string.hpp" 

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    try{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Publisher node");  
    server = this->create_service<cpp_pubsub::srv::ModifyString>("server_node", std::bind(&MinimalPublisher::modifyString,this,std::placeholders::_1,std::placeholders::_2));   
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Server");  
    }
     catch(...){
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error encountered at time of initialization!!");
      RCLCPP_FATAL_STREAM(this->get_logger(), "Publisher may not work!!");
    }
  }

  void modifyString(const std::shared_ptr<cpp_pubsub::srv::ModifyString::Request> request,   
          std::shared_ptr<cpp_pubsub::srv::ModifyString::Response>       response) {
  response->output = request->input + "String Modification from Service - Vignesh";

  server_response_message = response->output;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request to Server\ninput: '%s'",request->input.c_str()); //+
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response is : '%s'",response->output.c_str());}


 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = server_response_message;

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Able to update message data ");  
    RCLCPP_INFO(this->get_logger(), "Publishing the message : '%s'", message.data.c_str());

    publisher_->publish(message);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::MinimalPublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<cpp_pubsub::srv::ModifyString>::SharedPtr server;               
  std::string server_response_message = "Hey !! This is Vignesh here ";
  size_t count_;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  RCLCPP_WARN_STREAM(node->get_logger(), "Shutting Down!! " << 4);

  return 0;
}

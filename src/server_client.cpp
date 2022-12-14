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
#include <iostream>
#include <memory>
#include <string>

#include "cpp_pubsub/srv/modify_string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/**
 * @brief ServerClient class creates a node to give the new string to the
 * server.
 * @details New string is given by user using an argument are then send to
 * server, which is in the publisher node, and then server modifies the string
 * and save for further use by publisher using the same topic.
 */

class ServerClient : public rclcpp::Node {
 public:
  ServerClient() : Node("server_client") {
    client = this->create_client<cpp_pubsub::srv::ModifyString>("service_node");
  }
  auto getRequest(char **argv) {
    auto request = std::make_shared<cpp_pubsub::srv::ModifyString::Request>();
    request->input = argv[1];
    return request;
  }

  rclcpp::Client<cpp_pubsub::srv::ModifyString>::SharedPtr client;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // To start the service and check if the service requested exist
  std::shared_ptr<ServerClient> SClient = std::make_shared<ServerClient>();
  while (!SClient->client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service is not available, waiting again...");
  }

  // To pass data from the command line arguement into the input data for
  // request
  auto request = SClient->getRequest(argv);
  auto result = SClient->client->async_send_request(request);
  // Wait for the result.
  // If the result is successful, send the output data/message modified by the
  // service.
  if (rclcpp::spin_until_future_complete(SClient, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Modified string '%s'",
                result.get()->output.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service modify_string");
  }

  rclcpp::shutdown();
  return 0;
}

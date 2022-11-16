// Copyright [2022] <Vignesh Ravichandran Radhakrishnan (rr94@umd.edu)>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "cpp_pubsub/srv/modify_string.hpp"

using namespace std::chrono_literals;

class ServerClient : public rclcpp::Node{
    public:
        ServerClient() : Node("server_client"){
            client = this->create_client<cpp_pubsub::srv::ModifyString>("service_node"); 
        }
        auto getRequest(char **argv){
            auto request = std::make_shared<cpp_pubsub::srv::ModifyString::Request>();      
            request->input = argv[1]; 
            return request;
        }
    
        rclcpp::Client<cpp_pubsub::srv::ModifyString>::SharedPtr client;
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);

   // To start the service and check if the service requested exist 
    std::shared_ptr<ServerClient> ServClient = std::make_shared<ServerClient>();
    while (!ServClient->client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service is not available, waiting again...");
    }

    // To pass data from the command line arguement into the input data for request
    auto request = ServClient->getRequest(argv);
    auto result = ServClient->client->async_send_request(request);
    // Wait for the result.
    // If the result is successful, send the output data/message modified by the service.
    if (rclcpp::spin_until_future_complete(ServClient, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Modified string '%s'",result.get()->output.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service modify_string");  
    }

    rclcpp::shutdown();
    return 0;
}
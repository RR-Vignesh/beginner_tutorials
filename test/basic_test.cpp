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

#include "rclcpp/rclcpp.hpp"
#include <gtest/gtest.h>
#include <stdlib.h>

#include "std_msgs/msg/string.hpp"

/**
 * @brief The class TaskPlanningFixture is to plan multiple tests
 * 
 */
class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture()
      : tester_node_(std::make_shared<rclcpp::Node>("basic_test")) {
    RCLCPP_ERROR_STREAM(tester_node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }
/**
 * @brief To Set Up the object
 * 
 */
  void SetUp() override {
    // Setup things that should occur before every test instance should go here
    RCLCPP_ERROR_STREAM(tester_node_->get_logger(), "DONE WITH SETUP!!");
  }
/**
 * @brief To end the test case
 * 
 */
  void TearDown() override { std::cout << "DONE WITH TEARDOWN" << std::endl; }

 protected:
  rclcpp::Node::SharedPtr tester_node_;
};

/* TEST_F(TaskPlanningFixture, TrueIsTrueTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);
} */

TEST_F(TaskPlanningFixture, TestNumPublishers) {
  tester_node_ = rclcpp::Node::make_shared("test_publisher");
  // auto test_pub =
     // tester_node_->create_publisher<std_msgs::msg::String>("chatter", 10.0);

  auto number_of_publishers = tester_node_->count_publishers("chatter");
  EXPECT_EQ(1, static_cast<int>(number_of_publishers));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}

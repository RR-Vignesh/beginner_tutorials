// Description: Test if a simple task plan works

#include <gtest/gtest.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"


#include "std_msgs/msg/string.hpp"

class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture()
      : tester_node_(std::make_shared<rclcpp::Node>("basic_test"))
  {
    RCLCPP_ERROR_STREAM(tester_node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    // Setup things that should occur before every test instance should go here
    RCLCPP_ERROR_STREAM(tester_node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

 protected:
  rclcpp::Node::SharedPtr tester_node_;
};

/* TEST_F(TaskPlanningFixture, TrueIsTrueTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);
} */

TEST_F(TaskPlanningFixture, TestNumPublishers) {
  tester_node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub =
      tester_node_->create_publisher<std_msgs::msg::String>("chatter", 10.0);

  auto number_of_publishers = tester_node_->count_publishers("chatter");
  EXPECT_EQ(1, static_cast<int>(number_of_publishers));
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
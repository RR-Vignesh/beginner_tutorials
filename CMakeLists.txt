cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_export_dependencies(rosidl_default_runtime)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_action
   )

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ModifyString.srv"
)
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(
  talker 
  rclcpp 
  std_msgs 
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(server_client src/server_client.cpp)
ament_target_dependencies(server_client rclcpp)


install(TARGETS
  talker
  listener
  server_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

rosidl_target_interfaces(talker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(server_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  

  if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  set(TESTFILES
  test/basic_test.cpp)
  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES}) 
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp std_msgs)

install(TARGETS
test_${PROJECT_NAME}
DESTINATION lib/${PROJECT_NAME})

endif()  # BUILD_TESTING

ament_package()
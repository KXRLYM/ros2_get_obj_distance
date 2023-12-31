cmake_minimum_required(VERSION 3.8)
project(ros2_get_obj_distance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "~/ws_yolov8/src/*;${CMAKE_MODULE_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(ros2_get_obj_distance src/ros2_get_obj_distance.cpp)
ament_target_dependencies(ros2_get_obj_distance
  "rclcpp"
  "cv_bridge"
  "image_transport"
  "std_msgs"
  "sensor_msgs"
  "yolov8_msgs"
  "geometry_msgs"
)

install(TARGETS
  ros2_get_obj_distance
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()

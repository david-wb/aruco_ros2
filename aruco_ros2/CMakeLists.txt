cmake_minimum_required(VERSION 3.8)
project(aruco_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Create the executable for the node
add_executable(aruco_ros2 src/aruco_ros2.cpp)

# Link the OpenCV libraries to your node
ament_target_dependencies(aruco_ros2
  rclcpp
  std_msgs
  OpenCV
  image_transport
  cv_bridge
  tf2_ros
  geometry_msgs
  sensor_msgs
  aruco_msgs
  tf2_geometry_msgs
)

# Include OpenCV headers
target_include_directories(aruco_ros2 PRIVATE ${OpenCV_INCLUDE_DIRS})

# Link OpenCV libraries to your node
target_link_libraries(aruco_ros2 ${OpenCV_LIBRARIES})


# Install the executable
install(TARGETS
aruco_ros2
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()

cmake_minimum_required(VERSION 3.8)
project(rov_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

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
# Khai báo file C++ cần biên dịch
add_executable(rov_brain_node src/rov_brain_node.cpp)

# Cấp quyền sử dụng các thư viện ROS 2 cho file C++
ament_target_dependencies(rov_brain_node rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros)

# Chỉ định nơi lưu trữ sau khi biên dịch xong
install(TARGETS
  rov_brain_node
  DESTINATION lib/${PROJECT_NAME}
)
# Yêu cầu copy thư mục launch vào hệ thống khi build
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# Yêu cầu CMake copy các file Python vào hệ thống để chạy bằng 'ros2 run'
install(PROGRAMS
  rov_teleop.py
  depth_pid.py
  tf_broadcaster.py
  buoy_bridge.py
  rov_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()



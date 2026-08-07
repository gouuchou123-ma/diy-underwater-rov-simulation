cmake_minimum_required(VERSION 3.8)
project(asem3)

# Tìm kiếm lõi build của ROS 2
find_package(ament_cmake REQUIRED)

# Lệnh sinh tử: Yêu cầu ROS 2 copy thư mục meshes và urdf vào nhân hệ thống
install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

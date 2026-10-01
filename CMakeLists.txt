cmake_minimum_required(VERSION 3.16)
project(gn10-map-generator)

# CMake Policy 設定 (PCL周りの警告を抑制)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Add executable
add_executable(pointcloud_to_map src/pointcloud_to_map.cpp)

# Include PCL headers into the executable target
target_include_directories(pointcloud_to_map PUBLIC
  ${PCL_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(pointcloud_to_map
  rclcpp
  sensor_msgs
  pcl_conversions
  nlohmann_json
  tf2
  tf2_ros
  tf2_sensor_msgs
)
target_link_libraries(pointcloud_to_map
  ${PCL_LIBRARIES}
)

# Install executable
install(TARGETS
  pointcloud_to_map
  DESTINATION lib/${PROJECT_NAME}
)

find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(map_visualizer_node src/map_visualizer_node.cpp)

ament_target_dependencies(map_visualizer_node
  rclcpp
  visualization_msgs
  nlohmann_json
)

install(TARGETS
  map_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
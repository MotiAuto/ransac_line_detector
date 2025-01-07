cmake_minimum_required(VERSION 3.5)
project(ransac_line_detector)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
ament_auto_find_build_dependencies()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_add_library(target_ransac_line_detector SHARED
  DIRECTORY src
)

target_include_directories(target_ransac_line_detector
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_ransac_line_detector
  PLUGIN "ransac_line_detector::RansacLineDetector"
  EXECUTABLE ransac_line_detector_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)
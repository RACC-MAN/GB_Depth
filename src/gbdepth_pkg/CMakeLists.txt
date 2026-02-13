cmake_minimum_required(VERSION 3.8)
project(gbdepth_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/camera_node.cpp
  src/capture_node.cpp
  src/depth_estimation_node.cpp
  src/ground_estimation_node.cpp
  src/pcl_estimation_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES} 
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "CameraNode"
  EXECUTABLE camera_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "CaptureNode"
  EXECUTABLE capture_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "DepthEstimationNode"
  EXECUTABLE depth_estimation_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "GroundEstimationNode"
  EXECUTABLE ground_estimation_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "PclEstimationNode"
  EXECUTABLE pcl_estimation_node
)

target_link_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)

install(FILES DESTINATION share/${PROJECT_NAME})
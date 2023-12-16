cmake_minimum_required(VERSION 3.5)
project(mono_color_segmentation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

set(executable_name mono_color_segmentation)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  cv_bridge
  OpenCV
)

# Declare a C++ library
add_library(${library_name} SHARED
  src/mono_color_segmentation.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

# Declare a C++ executable
add_executable(${executable_name}
  src/main.cpp
)
target_link_libraries(${executable_name} ${library_name})
ament_target_dependencies(${executable_name}
  ${dependencies}
)


rclcpp_components_register_nodes(${library_name} "MonoColorSegmentation")

# Install the node and component library
install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Include CMake macro for ament package
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()

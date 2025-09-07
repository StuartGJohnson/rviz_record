cmake_minimum_required(VERSION 3.8)
project(rviz_record)

# ===== C++ / Qt =====
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# IMPORTANT: keep AUTOMOC OFF globally to avoid scanning rosidl-generated files
set(CMAKE_AUTOMOC OFF)

# ===== Dependencies =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(PkgConfig REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)


# GStreamer (core + appsrc) via pkg-config
pkg_check_modules(GST    REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(GSTAPP REQUIRED IMPORTED_TARGET gstreamer-app-1.0)

# ---- ROS interfaces (services) ----
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

# Export public headers (for downstreams)
ament_export_include_directories(include)

# (Optional belt-and-suspenders) make the headers visible to tools scanning compile_commands
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ===== Plugin library (name avoids clash with package target) =====
add_library(rviz_record_plugin SHARED
  src/record_panel.cpp
  include/record_panel.hpp
)

# Enable AUTOMOC only for this target
set_target_properties(rviz_record_plugin PROPERTIES AUTOMOC ON)

# Include dirs for build & install
target_include_directories(rviz_record_plugin
  BEFORE PRIVATE
  ${rviz_ogre_vendor_INCLUDE_DIRS}
)

target_include_directories(rviz_record_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Good practice for Qt plugins
#target_compile_definitions(rviz_record_plugin PRIVATE QT_NO_KEYWORDS)

# Link deps
target_link_libraries(rviz_record_plugin
  PkgConfig::GST
  PkgConfig::GSTAPP
  Qt5::Widgets
)
ament_target_dependencies(rviz_record_plugin
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  pluginlib
)

# Modern linkage to generated typesupport (replaces deprecated rosidl_target_interfaces)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
if(cpp_typesupport_target)
  target_link_libraries(rviz_record_plugin ${cpp_typesupport_target})
endif()

# Ensure build order against rosidl-generated headers
add_dependencies(rviz_record_plugin
  ${PROJECT_NAME}__rosidl_generator_cpp
)

# Export the RViz plugin description
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# ===== Install =====
install(TARGETS rviz_record_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install public headers to <prefix>/include/rviz_record/...
install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Ship srv files (optional, for reference)
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

# Export ament deps for downstream packages
ament_export_dependencies(rclcpp rviz_common pluginlib)

ament_package()

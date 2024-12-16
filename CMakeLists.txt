cmake_minimum_required(VERSION 3.8)
project(voronoi_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dep_pkgs rclcpp geometry_msgs nav2_costmap_2d pluginlib)
set(library_name ${PROJECT_NAME}_plugin)

# Include directories target_include_directories( ${PROJECT_NAME} PRIVATE
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)

set(library_name ${PROJECT_NAME}_plugin)

add_library(${library_name} SHARED src/voronoi_planner.cpp)
ament_target_dependencies(${library_name} ${dep_pkgs})
include_directories(include)

# IDK What this does for now
target_compile_definitions(${library_name}
                           PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core voronoi_planner.xml)

# === Installation ===
install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

install(FILES voronoi_planner.xml DESTINATION share/${PROJECT_NAME})
# Add the plugin test executable
add_executable(plugin_test src/test.cpp)

# Link necessary libraries
target_link_libraries(plugin_test rclcpp pluginlib)

# Install the test executable
install(TARGETS plugin_test DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

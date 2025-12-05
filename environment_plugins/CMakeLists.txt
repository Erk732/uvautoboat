cmake_minimum_required(VERSION 3.10)
project(environment_plugins)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Create the shared library
add_library(dead_zone_plugin SHARED src/dead_zone_plugin.cc)

# Include directories
target_include_directories(dead_zone_plugin PRIVATE
  ${gz-sim8_INCLUDE_DIRS}
  ${gz-plugin2_INCLUDE_DIRS}
  ${gz-math7_INCLUDE_DIRS}
  ${gz-common5_INCLUDE_DIRS}
)

# Link libraries (critical for proper linking)
target_link_libraries(dead_zone_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
  gz-common5::gz-common5
)

# Set C++ standard
target_compile_features(dead_zone_plugin PRIVATE cxx_std_17)

# Install the plugin library
install(TARGETS dead_zone_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install package.xml for ament
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install ament resource index marker
install(FILES resource/environment_plugins
  DESTINATION share/ament_index/resource_index/packages
)

# Configure and install environment hook (auto-sets GZ_SIM_SYSTEM_PLUGIN_PATH)
configure_file(
  hooks/environment_plugins.dsv.in
  ${CMAKE_CURRENT_BINARY_DIR}/hooks/environment_plugins.dsv
  @ONLY
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/hooks/environment_plugins.dsv
  DESTINATION share/${PROJECT_NAME}/hooks
)

ament_package()

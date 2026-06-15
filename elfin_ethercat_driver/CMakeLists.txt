cmake_minimum_required(VERSION 3.16)
project(elfin_ethercat_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (required by ROS 2 Jazzy)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(INSTALL_LIB_DIR "${PROJECT_BINARY_DIR}/lib")

set(CMAKE_SKIP_BUILD_RPATH FALSE)

set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${PROJECT_BINARY_DIR}")

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(soem_ros2 REQUIRED)
find_package(elfin_robot_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  SHARED
  "src/elfin_ethercat_driver.cpp"
  "src/elfin_ethercat_client.cpp"
  "src/elfin_ethercat_io_client.cpp"
  "src/elfin_ethercat_manager.cpp"
  include/${PROJECT_NAME}/elfin_ethercat_driver.h
  include/${PROJECT_NAME}/elfin_ethercat_client.h
  include/${PROJECT_NAME}/elfin_ethercat_io_client.h
  include/${PROJECT_NAME}/elfin_ethercat_manager.h
)

# Boost is baked into the shared library's NEEDED entries; downstream consumers
# link the .so via ament_export_libraries and do not need to re-find Boost.
target_link_libraries(${PROJECT_NAME}_lib Boost::system Boost::thread)

set(DEPENDENCIES
  rclcpp
  rclpy
  std_msgs
  soem_ros2
  elfin_robot_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  tf2
  hardware_interface
  rosidl_default_generators
)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(EXECUTABLE_NAME "elfin_ethercat_driver")

add_executable(${EXECUTABLE_NAME}
src/elfin_ethercat_driver_node.cpp
)

target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib Boost::system Boost::thread)
ament_target_dependencies(${EXECUTABLE_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

# Install and export the library so downstream packages (elfin_ros_control)
# can link against it without hard-coded build/install paths.
# ament_export_libraries (not export targets) avoids baking Boost/soem
# interface properties into an imported target for consumers.
install(
  TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(${DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS  ${EXECUTABLE_NAME}  
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY 
    # launch
    DESTINATION 
    share/${PROJECT_NAME}
)

ament_package()

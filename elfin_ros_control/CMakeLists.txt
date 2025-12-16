cmake_minimum_required(VERSION 3.5)
project(elfin_ros_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(elfin_robot_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(elfin_ethercat_driver)
find_package(pluginlib REQUIRED REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
)

set(DEPENDENCIES
  "rclcpp"
  "rclpy"
  "std_msgs"
  "std_srvs"
  "controller_manager"
  "controller_manager_msgs"
  "elfin_robot_msgs"
  "trajectory_msgs"
  "sensor_msgs"
  "tf2"
  "hardware_interface"
  "controller_interface"
  "pluginlib"
  "rosidl_default_generators"
)

add_library(elfin_hardware_interface
        SHARED
        src/elfin_hardware_interface.cpp
        src/elfin_hardware_onlyposition_interface.cpp
        )

target_link_libraries(
	elfin_hardware_interface
)

target_include_directories(
        elfin_hardware_interface
        PUBLIC
        ${elfin_ethercat_driver_INCLUDE_DIRS}
        PRIVATE
        include
)

ament_target_dependencies(
        elfin_hardware_interface
        ${DEPENDENCIES}
)

pluginlib_export_plugin_description_file(hardware_interface elfin_hardware_interface.xml)

install(
        TARGETS elfin_hardware_interface
        LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

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


install(
    DIRECTORY 
    DESTINATION
    share/${PROJECT_NAME}
)

ament_package()

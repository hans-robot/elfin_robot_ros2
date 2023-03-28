cmake_minimum_required(VERSION 3.5)
project(elfin_basic_api)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(elfin_robot_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/elfin_basic_api.cpp"
  "src/elfin_teleop_api.cpp"
  "src/elfin_motion_api.cpp"
)

set(DEPENDENCIES
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "moveit_core"
  "moveit_planners_ompl"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "control_msgs"
  "controller_manager_msgs"
  "elfin_robot_msgs"
  "tf2"
  "tf2_ros"
)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(EXECUTABLE_NAME "elfin_basic_api_node")

add_executable(${EXECUTABLE_NAME} 
src/elfin_basic_api_node.cpp
src/elfin_basic_api.cpp
src/elfin_teleop_api.cpp
src/elfin_motion_api.cpp
)

target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
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

# REQUIRED to -> Launch simulation:
install(
    DIRECTORY 
    launch
    DESTINATION 
    share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/elfin_gui.py
  scripts/test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY  
  scripts/btn_icon
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()

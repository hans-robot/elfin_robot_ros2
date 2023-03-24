cmake_minimum_required(VERSION 3.5)
project(soem_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

unset( CATKIN_INSTALL_INTO_PREFIX_ROOT )

if(WIN32)
  set(OS "win32")
else()
  set(OS "linux")
endif()

# explicitely find pthread to be able to explicitely export the pthread dependency via catkin_package
# see https://github.com/ros/ros_comm/issues/135
set(PTHREAD_LIB "")
find_package(Threads)
if(CMAKE_THREAD_LIBS_INIT)
  string(LENGTH ${CMAKE_THREAD_LIBS_INIT} _length)
  if(_length GREATER 2)
    string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 0 2 _prefix)
    if(${_prefix} STREQUAL "-l")
      math(EXPR _rest_len "${_length} - 2")
      string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 2 ${_rest_len} PTHREAD_LIB)
    endif()
  endif()
endif()

message("phtread=${PTHREAD_LIB}")

ament_export_include_directories(include)

# SOEM is, by default, built with position-dependent code as a static library.
# To be able to include SOEM in a dynamic library (default ROS convention),
# it needs to be built with position-independent code.
# Thus, we add -fPIC here.
# This allows us to still include this in any shared libraries we create.
add_compile_options(-fPIC)
# removed any pre-defined `WIN32_LEAN_AND_MEAN`.
# Otherwise, many symbols will be reported as missing.
remove_definitions(-DWIN32_LEAN_AND_MEAN)
#catkin_lint: ignore duplicate_cmd
#catkin_lint: ignore subproject
add_subdirectory(SOEM)

ament_export_libraries(soem ${PTHREAD_LIB})
ament_package(CONFIG_EXTRAS ${CMAKE_CURRENT_SOURCE_DIR}/cmake/soem-ros-extras.cmake.in)
                             


# =============================================================================
#  ids_ros/CMakeLists.txt - the IDS as a ROS 2 node
# -----------------------------------------------------------------------------
#  The parameter handling is a small ROS-free library (tested anywhere); the node
#  itself is built only when ROS 2 (ament) is present.
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(ids_ros VERSION 1.2.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake QUIET)
if(NOT ament_cmake_FOUND)
  include(CTest)
endif()
find_package(ids_core REQUIRED)

# ROS parameters <-> IdsConfig, kept free of ROS so its tests run without ROS.
add_library(ids_ros_parameters STATIC src/parameters.cpp)
set_target_properties(ids_ros_parameters PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ids_ros_parameters PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ids_ros_parameters PUBLIC ids_core::ids_core)

# The node: subscribes to the trusted reference and the measurements, publishes decisions.
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(ids_interfaces REQUIRED)
  add_executable(ids_node src/ids_node.cpp)
  target_link_libraries(ids_node ids_ros_parameters)
  ament_target_dependencies(ids_node rclcpp std_msgs std_srvs ids_interfaces)
  install(TARGETS ids_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
endif()

set(IDS_ROS_TESTS test_parameters test_conversions)

if(BUILD_TESTING)
  if(ament_cmake_FOUND)
    find_package(ament_cmake_gtest REQUIRED)
  else()
    find_package(GTest REQUIRED)
  endif()
  foreach(t ${IDS_ROS_TESTS})
    if(ament_cmake_FOUND)
      ament_add_gtest(${t} test/${t}.cpp)
    else()
      add_executable(${t} test/${t}.cpp)
      target_link_libraries(${t} GTest::gtest_main)
      add_test(NAME ${t} COMMAND ${t})
    endif()
    target_link_libraries(${t} ids_ros_parameters)
  endforeach()
endif()

if(ament_cmake_FOUND)
  ament_package()
endif()

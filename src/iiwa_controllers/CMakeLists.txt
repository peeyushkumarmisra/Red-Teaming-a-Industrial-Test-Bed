# =============================================================================
#  iiwa_controllers/CMakeLists.txt - the robot's tracking controller
# -----------------------------------------------------------------------------
#  The control logic is a ROS-free library (tracking core) with its own tests; the
#  ros2_control plugin is a thin layer that is built only when ROS 2 is present.
#  The IDS contract test (controller + IDS together) is built when ids_core is found.
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(iiwa_controllers VERSION 1.2.0 LANGUAGES CXX)

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
find_package(iiwa_control_law REQUIRED)

# Controller logic without ROS: testable anywhere, shared by the plugin.
add_library(iiwa_tracking_core STATIC src/tracking_core.cpp)
set_target_properties(iiwa_tracking_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(iiwa_tracking_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(iiwa_tracking_core PUBLIC iiwa_control_law::iiwa_control_law)

if(ament_cmake_FOUND)
  find_package(controller_interface REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(realtime_tools REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(ids_interfaces REQUIRED)
  add_library(${PROJECT_NAME} SHARED src/reference_tracking_controller.cpp)
  target_link_libraries(${PROJECT_NAME} iiwa_tracking_core)
  ament_target_dependencies(${PROJECT_NAME}
    controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools
    std_msgs ids_interfaces)
  # Register the plugin so controller_manager can load it by name.
  pluginlib_export_plugin_description_file(controller_interface iiwa_controllers.xml)
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

set(IIWA_CONTROLLERS_TESTS test_tracking_core test_encoder_model)

if(BUILD_TESTING)
  if(ament_cmake_FOUND)
    find_package(ament_cmake_gtest REQUIRED)
  else()
    find_package(GTest REQUIRED)
  endif()
  foreach(t ${IIWA_CONTROLLERS_TESTS})
    if(ament_cmake_FOUND)
      ament_add_gtest(${t} test/${t}.cpp)
    else()
      add_executable(${t} test/${t}.cpp)
      target_link_libraries(${t} GTest::gtest_main)
      add_test(NAME ${t} COMMAND ${t})
    endif()
    target_link_libraries(${t} iiwa_tracking_core)
    target_include_directories(${t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  endforeach()
  # Contract test: the real controller logic feeding the real IDS (needs ids_core).
  find_package(ids_core QUIET)
  if(ids_core_FOUND)
    if(ament_cmake_FOUND)
      ament_add_gtest(test_ids_contract test/test_ids_contract.cpp TIMEOUT 300)
    else()
      add_executable(test_ids_contract test/test_ids_contract.cpp)
      target_link_libraries(test_ids_contract GTest::gtest_main)
      add_test(NAME test_ids_contract COMMAND test_ids_contract)
    endif()
    target_link_libraries(test_ids_contract iiwa_tracking_core ids_core::ids_core)
    target_include_directories(test_ids_contract PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  endif()
  if(ament_cmake_FOUND)
    find_package(controller_manager REQUIRED)
    find_package(ros2_control_test_assets REQUIRED)
    # Loads the actual plugin through controller_manager (ROS only).
    ament_add_gtest(test_load_controller test/test_load_controller.cpp)
    ament_target_dependencies(test_load_controller
      controller_manager hardware_interface ros2_control_test_assets)
  endif()
endif()

if(ament_cmake_FOUND)
  ament_package()
endif()

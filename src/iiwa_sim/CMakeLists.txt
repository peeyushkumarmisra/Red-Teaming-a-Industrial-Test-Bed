cmake_minimum_required(VERSION 3.16)
project(iiwa_sim VERSION 1.2.0 LANGUAGES CXX)

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

add_library(iiwa_friction_model INTERFACE)
target_include_directories(iiwa_friction_model INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(gz_sim_vendor QUIET)
find_package(gz_plugin_vendor QUIET)
find_package(gz_transport_vendor QUIET)
find_package(gz_msgs_vendor QUIET)
find_package(gz-sim8 QUIET)

if(gz-sim8_FOUND)
  find_package(gz-plugin2 REQUIRED COMPONENTS register)
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_LIBRARIES gz-sim8::gz-sim8)
  check_cxx_source_compiles("
#include <memory>
#include <gz/sim/System.hh>
class P : public gz::sim::ISystemConfigurePriority {
 public:
  gz::sim::System::PriorityType ConfigurePriority() override { return 0; }
};
int main() { return 0; }
" IIWA_SIM_HAS_SYSTEM_PRIORITY)
  unset(CMAKE_REQUIRED_LIBRARIES)
  add_library(iiwa_friction_drift SHARED src/friction_drift.cpp)
  target_link_libraries(iiwa_friction_drift
    iiwa_friction_model
    gz-sim8::gz-sim8
    gz-plugin2::register
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10)
  if(IIWA_SIM_HAS_SYSTEM_PRIORITY)
    target_compile_definitions(iiwa_friction_drift PRIVATE IIWA_SIM_HAS_SYSTEM_PRIORITY)
  else()
    message(WARNING "gz-sim8 has no ISystemConfigurePriority: FrictionDrift relies on being declared after gz_ros2_control")
  endif()
  install(TARGETS iiwa_friction_drift LIBRARY DESTINATION lib)
elseif(ament_cmake_FOUND)
  message(WARNING "gz-sim8 not found: the FrictionDrift plugin is not built")
endif()

if(BUILD_TESTING)
  if(ament_cmake_FOUND)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_gtest(test_friction_model test/test_friction_model.cpp)
    ament_add_pytest_test(test_description test/test_description.py TIMEOUT 120)
  else()
    find_package(GTest REQUIRED)
    add_executable(test_friction_model test/test_friction_model.cpp)
    target_link_libraries(test_friction_model GTest::gtest_main)
    add_test(NAME test_friction_model COMMAND test_friction_model)
  endif()
  target_link_libraries(test_friction_model iiwa_friction_model)
endif()

if(ament_cmake_FOUND)
  install(DIRECTORY config meshes urdf worlds DESTINATION share/${PROJECT_NAME})
  install(FILES NOTICE DESTINATION share/${PROJECT_NAME})
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
  ament_package()
endif()
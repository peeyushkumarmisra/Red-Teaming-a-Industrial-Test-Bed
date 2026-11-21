# =============================================================================
#  ids_core/CMakeLists.txt - the detector as a plain C++ library
# -----------------------------------------------------------------------------
#  Builds the whole intrusion detection algorithm as one library with no ROS
#  dependency. Why it matters: the algorithm can be unit-tested, benchmarked and
#  replayed offline on recordings without a robot, a simulator or ROS running.
#  The ROS node (ids_ros) is a thin wrapper around it.
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(ids_core VERSION 1.2.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

# ament is optional: the library and its tests also build with plain CMake.
find_package(ament_cmake QUIET)
if(NOT ament_cmake_FOUND)
  include(CTest)
endif()
find_package(iiwa_control_law REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)

# The detection pipeline, in the order data flows through it:
# model -> filter -> averages -> detector, plus limits, fingerprint and calibration.
add_library(${PROJECT_NAME} SHARED
  src/closed_loop_model.cpp
  src/ekf.cpp
  src/dual_ewma.cpp
  src/detector.cpp
  src/limit_selection.cpp
  src/sha256.cpp
  src/calibration.cpp
  src/ids_pipeline.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC iiwa_control_law::iiwa_control_law Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp::yaml-cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# One test program per building block, plus end-to-end scenarios and a real-time check.
set(IDS_CORE_TESTS
  test_robot_model
  test_closed_loop_model
  test_ekf
  test_dual_ewma
  test_detector
  test_calibration
  test_pipeline
  test_scenarios
  test_realtime
)

if(BUILD_TESTING)
  if(NOT ament_cmake_FOUND)
    find_package(GTest REQUIRED)
  else()
    find_package(ament_cmake_gtest REQUIRED)
  endif()
  foreach(t ${IDS_CORE_TESTS})
    if(ament_cmake_FOUND)
      # Scenario tests simulate minutes of robot motion, hence the long timeout.
      ament_add_gtest(${t} test/${t}.cpp TIMEOUT 600)
    else()
      add_executable(${t} test/${t}.cpp)
      target_link_libraries(${t} GTest::gtest_main)
      add_test(NAME ${t} COMMAND ${t})
    endif()
    target_link_libraries(${t} ${PROJECT_NAME})
    target_include_directories(${t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_compile_definitions(${t} PRIVATE
      IDS_CORE_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/data/iiwa14.urdf")
  endforeach()
  # Benchmark: time per 10 ms window (the real-time budget).
  add_executable(bench_ids_step bench/bench_ids_step.cpp)
  target_link_libraries(bench_ids_step ${PROJECT_NAME})
  target_include_directories(bench_ids_step PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_compile_definitions(bench_ids_step PRIVATE
    IDS_CORE_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/data/iiwa14.urdf")
endif()

if(ament_cmake_FOUND)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(iiwa_control_law Eigen3 pinocchio)
  ament_package()
else()
  install(EXPORT export_${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
    FILE ${PROJECT_NAME}Targets.cmake
  )
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    "include(CMakeFindDependencyMacro)\nfind_dependency(iiwa_control_law)\nfind_dependency(Eigen3)\nfind_dependency(pinocchio)\ninclude(\${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Targets.cmake)\n")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    DESTINATION lib/cmake/${PROJECT_NAME})
endif()

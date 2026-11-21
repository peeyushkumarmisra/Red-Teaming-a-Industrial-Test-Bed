cmake_minimum_required(VERSION 3.16)
project(iiwa_control_law VERSION 1.2.0 LANGUAGES CXX)

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
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(${PROJECT_NAME} SHARED src/robot_model.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen pinocchio::pinocchio)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  if(ament_cmake_FOUND)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_control_law test/test_control_law.cpp)
    target_link_libraries(test_control_law ${PROJECT_NAME})
  else()
    find_package(GTest REQUIRED)
    add_executable(test_control_law test/test_control_law.cpp)
    target_link_libraries(test_control_law ${PROJECT_NAME} GTest::gtest_main)
    add_test(NAME test_control_law COMMAND test_control_law)
  endif()
endif()

if(ament_cmake_FOUND)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3 pinocchio)
  ament_package()
else()
  install(EXPORT export_${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
    FILE ${PROJECT_NAME}Targets.cmake
  )
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    "include(CMakeFindDependencyMacro)\nfind_dependency(Eigen3)\nfind_dependency(pinocchio)\ninclude(\${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Targets.cmake)\n")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    DESTINATION lib/cmake/${PROJECT_NAME})
endif()
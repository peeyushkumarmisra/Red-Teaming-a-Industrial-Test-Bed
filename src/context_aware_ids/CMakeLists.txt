cmake_minimum_required(VERSION 3.8)
project(context_aware_ids)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Forceing the compiler to use C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON) # Mkae sure it not run in other verson 

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -Wpedantic")

# find dependencies
find_package(ament_cmake REQUIRED)  # ROS 2 CMake build tools
find_package(rclcpp REQUIRED)       # ROS 2 C++ client library
find_package(std_msgs REQUIRED)     # Standard Message Definition
find_package(sensor_msgs REQUIRED)  # Sensor Message Definition
find_package(Eigen3 REQUIRED)       # Eigen3 library
find_package(RBDL REQUIRED)         # Rigid Body Dynamics Library

# Compiling the main ROS 2 orchestrator node
add_executable(ids_node src/ids_node.cpp)

# Including header directories
target_include_directories(ids_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Linking the external libraries
target_link_libraries(ids_node
  Eigen3::Eigen
  rbdl
  rbdl_urdfreader
)

target_compile_features(ids_node PUBLIC c_std_99 cxx_std_20) 

# Linking the ROS 2 dependencies
ament_target_dependencies(ids_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

# Installing the compiled binary into the ROS 2 workspace
install(TARGETS ids_node
  DESTINATION lib/${PROJECT_NAME})

# Testing block for linters and code style checks
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE) # Skips copyright checking for now
  set(ament_cmake_cpplint_FOUND TRUE)   # Skips cpplint checking for now
  ament_lint_auto_find_test_dependencies() # Finds other testing dependencies
endif()

# Installing header files into the workspace
install(DIRECTORY include/ DESTINATION include)

# Generating the final ROS 2 environment
ament_package()

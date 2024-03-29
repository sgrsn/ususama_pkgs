cmake_minimum_required(VERSION 3.5)
project(mecanum_io)

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

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(mecanum_io_demo src/mecanum_io_demo.cpp)
target_link_libraries(mecanum_io_demo  Threads::Threads)
target_include_directories(mecanum_io_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mecanum_io_demo)
ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)


target_include_directories(mecanum_io_demo
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # 通常の指定
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> # Eigen3用の指定
    $<INSTALL_INTERFACE:include> # 通常指定
)

add_executable(mecanum_node src/mecanum_node.cpp)
target_include_directories(mecanum_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mecanum_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros)
target_include_directories(mecanum_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # 通常の指定
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> # Eigen3用の指定
    $<INSTALL_INTERFACE:include> # 通常指定
)
install(TARGETS mecanum_io_demo mecanum_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
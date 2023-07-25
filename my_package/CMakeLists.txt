cmake_minimum_required(VERSION 3.5)
project(my_package)

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
find_package(rclcpp REQUIRED)
find_package(parking_interface REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(include/my_package)




#add_executable (Hungarian src/Hungarian.cpp)
#add_executable (KalmanTracker src/KalmanTracker.cpp)

# add_library(${PROJECT_NAME} SHARED
#   src/KalmanTracker.cpp
#   src/Hungarian.cpp
# )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_executable (Parkingfusion src/Parkingfusion.cpp)
# ament_target_dependencies(Parkingfusion std_msgs rclcpp parking_interface)
add_executable (radar_pub src/radar_pub.cpp)
ament_target_dependencies(radar_pub std_msgs rclcpp parking_interface geometry_msgs)
add_executable (image_pub src/image_pub.cpp)
ament_target_dependencies(image_pub std_msgs rclcpp parking_interface geometry_msgs)
# add_executable (vision_pub src/vision_pub.cpp)
# ament_target_dependencies(vision_pub std_msgs geometry_msgs visualization_msgs rclcpp parking_interface)
add_executable (test_tracking src/test_tracking.cpp src/Hungarian.cpp src/KalmanTracker.cpp)
ament_target_dependencies(test_tracking std_msgs geometry_msgs visualization_msgs rclcpp parking_interface OpenCV)
# add_executable(my_node src/my_node.cpp)

# add_executable (tracking_pub src/tracking_pub.cpp src/Hungarian.cpp src/KalmanTracker.cpp)
# ament_target_dependencies(tracking_pub std_msgs rclcpp parking_interface OpenCV)




# target_link_libraries(tracking_pub Hungarian KalmanTracker)
# target_include_directories(my_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_include_directories(vision_pub PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# install(TARGETS my_node
#   DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  # Parkingfusion
  radar_pub
  image_pub
  # vision_pub
  # tracking_pub
  test_tracking
  DESTINATION lib/${PROJECT_NAME})

# target_link_libraries(${PROJECT_NAME}
# ${OpenCV_LIBS}
# )

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

cmake_minimum_required(VERSION 3.5)
project(turtlesim_controller)

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
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Harmonic.srv"
 )

add_executable(controller src/exercise1.cpp)
ament_target_dependencies(controller
  rclcpp turtlesim geometry_msgs)
  
rosidl_target_interfaces(controller
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
add_executable(harmonic src/server.cpp)
ament_target_dependencies(harmonic
  rclcpp)
  
rosidl_target_interfaces(harmonic
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
add_library(harmonic_component SHARED
  src/server_comp.cpp)
  
rclcpp_components_register_node(harmonic_component
  PLUGIN "turtlesim_controller::HarmonicServer"
  EXECUTABLE comp_harmonic)
  
ament_target_dependencies(harmonic_component rclcpp rclcpp_components)

rosidl_target_interfaces(harmonic_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp") 
  
add_library(controller_component SHARED
  src/exercise1_comp.cpp)
  
rclcpp_components_register_node(controller_component
  PLUGIN "turtlesim_controller::MyController"
  EXECUTABLE comp_controller)
  
ament_target_dependencies(controller_component rclcpp rclcpp_components turtlesim geometry_msgs)

rosidl_target_interfaces(controller_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp") 
  
  
install(TARGETS
  controller
  harmonic
  harmonic_component
  controller_component
  comp_harmonic
  comp_controller
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib)
  
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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

ament_package()

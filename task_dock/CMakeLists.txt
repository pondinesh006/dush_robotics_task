cmake_minimum_required(VERSION 3.8)
project(task_dock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToDock.srv"
  "action/GoToDock.action"
)

add_executable(docking_server src/server.cpp)
add_executable(robot_client src/client.cpp)

add_executable(docking_action_server src/action_server.cpp)
add_executable(robot_action_client src/action_client.cpp)

ament_export_dependencies(rosidl_default_runtime)
ament_target_dependencies(docking_server
  rclcpp
)

ament_target_dependencies(robot_client
  rclcpp
)

ament_target_dependencies(docking_action_server
  rclcpp
  rclcpp_action
)

ament_target_dependencies(robot_action_client
  rclcpp
  rclcpp_action
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(docking_server
  ${cpp_typesupport_target})

target_link_libraries(robot_client
  ${cpp_typesupport_target})

target_link_libraries(docking_action_server
  ${cpp_typesupport_target})

target_link_libraries(robot_action_client
  ${cpp_typesupport_target})

target_include_directories(docking_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(docking_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(robot_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(docking_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(docking_action_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(robot_action_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_action_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS docking_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS robot_client
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS docking_action_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS robot_action_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)

project(tucan_msgs)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# get all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB TUCAN_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

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

rosidl_generate_interfaces(${PROJECT_NAME}
	${TUCAN_MSGS}
	DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
	ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

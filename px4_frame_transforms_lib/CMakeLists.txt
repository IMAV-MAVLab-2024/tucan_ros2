cmake_minimum_required(VERSION 3.8)
project(px4_frame_transforms_lib)


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

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
set(BUILD_SHARED_LIBS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include SYSTEM ${Eigen3_INCLUDE_DIRS})

set(HEADER_FILES
    include/${PROJECT_NAME}/frame_transforms.h
)

add_library(${PROJECT_NAME}
  ${HEADER_FILES}
  src/lib/frame_transforms.cpp
)

ament_target_dependencies(px4_frame_transforms_lib Eigen3 geometry_msgs sensor_msgs)

ament_export_targets(px4_frame_transforms_lib HAS_LIBRARY_TARGET)

ament_export_dependencies(Eigen3 eigen3_cmake_module geometry_msgs sensor_msgs)

# Install header files
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

install(TARGETS px4_frame_transforms_lib
	EXPORT px4_frame_transforms_lib
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

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

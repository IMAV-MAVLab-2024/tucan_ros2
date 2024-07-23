cmake_minimum_required(VERSION 3.5)
project(qgroundcontrol)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

if(NOT DEFINED SKIP_QGROUNDCONTROL OR NOT ${SKIP_QGROUNDCONTROL})
  if(NOT EXISTS "${CMAKE_BINARY_DIR}/QGroundControl.AppImage")
    file(DOWNLOAD
      https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage
      QGroundControl.AppImage
    )
  endif()

  install(PROGRAMS
    ${CMAKE_BINARY_DIR}/QGroundControl.AppImage
    TYPE BIN
  )
else()
  message(WARNING "Skipping QGroundControl")
endif()

ament_package()

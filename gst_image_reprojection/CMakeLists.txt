cmake_minimum_required(VERSION 3.16)
project(gst_image_reprojection C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.18 gstreamer-base-1.0 gstreamer-video-1.0)
pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)

add_library(gstimagereprojection MODULE
  src/gstimagereprojection.c
)

target_include_directories(gstimagereprojection PRIVATE
  ${GST_INCLUDE_DIRS}
  ${JSON_GLIB_INCLUDE_DIRS}
)

target_compile_options(gstimagereprojection PRIVATE
  ${GST_CFLAGS_OTHER}
  ${JSON_GLIB_CFLAGS_OTHER}
  -Wall
  -Wextra
  -Wpedantic
  -Wno-unused-parameter
)

target_link_libraries(gstimagereprojection
  ${GST_LIBRARIES}
  ${JSON_GLIB_LIBRARIES}
  m
)

set_target_properties(gstimagereprojection PROPERTIES
  OUTPUT_NAME "gstimagereprojection"
  PREFIX ""
)

install(TARGETS gstimagereprojection
  LIBRARY DESTINATION lib/gstreamer-1.0
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

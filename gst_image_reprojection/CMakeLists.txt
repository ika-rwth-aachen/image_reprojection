cmake_minimum_required(VERSION 3.16)
project(gst_image_reprojection C)

include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.22 gstreamer-base-1.0 gstreamer-video-1.0)
pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)

add_library(gstimagereprojection MODULE
  src/gstimagereprojection.c
)

if(CMAKE_CUDA_COMPILER)
  target_sources(gstimagereprojection PRIVATE src/gstimagereprojection_cuda.cu)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.18")
    set_property(TARGET gstimagereprojection PROPERTY CUDA_ARCHITECTURES OFF)
  endif()
  target_compile_definitions(gstimagereprojection PRIVATE HAVE_CUDA=1)
  target_compile_options(gstimagereprojection PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:-gencode=arch=compute_75,code=compute_75>
    $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>
  )
  target_link_libraries(gstimagereprojection CUDA::cudart)
endif()

target_include_directories(gstimagereprojection PRIVATE
  ${GST_INCLUDE_DIRS}
  ${JSON_GLIB_INCLUDE_DIRS}
)

target_compile_options(gstimagereprojection PRIVATE
  ${GST_CFLAGS_OTHER}
  ${JSON_GLIB_CFLAGS_OTHER}
  $<$<COMPILE_LANGUAGE:C>:-Wall>
  $<$<COMPILE_LANGUAGE:C>:-Wextra>
  $<$<COMPILE_LANGUAGE:C>:-Wpedantic>
  $<$<COMPILE_LANGUAGE:C>:-Wno-unused-parameter>
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

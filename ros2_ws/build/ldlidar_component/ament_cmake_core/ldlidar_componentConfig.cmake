# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ldlidar_component_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ldlidar_component_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ldlidar_component_FOUND FALSE)
  elseif(NOT ldlidar_component_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ldlidar_component_FOUND FALSE)
  endif()
  return()
endif()
set(_ldlidar_component_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ldlidar_component_FIND_QUIETLY)
  message(STATUS "Found ldlidar_component: 0.2.0 (${ldlidar_component_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ldlidar_component' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ldlidar_component_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ldlidar_component_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${ldlidar_component_DIR}/${_extra}")
endforeach()

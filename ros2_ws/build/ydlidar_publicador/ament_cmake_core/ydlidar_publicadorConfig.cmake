# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ydlidar_publicador_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ydlidar_publicador_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ydlidar_publicador_FOUND FALSE)
  elseif(NOT ydlidar_publicador_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ydlidar_publicador_FOUND FALSE)
  endif()
  return()
endif()
set(_ydlidar_publicador_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ydlidar_publicador_FIND_QUIETLY)
  message(STATUS "Found ydlidar_publicador: 1.0.1 (${ydlidar_publicador_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ydlidar_publicador' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ydlidar_publicador_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ydlidar_publicador_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ydlidar_publicador_DIR}/${_extra}")
endforeach()

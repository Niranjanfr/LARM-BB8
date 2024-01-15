# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_grpe_larm_mother_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED grpe_larm_mother_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(grpe_larm_mother_FOUND FALSE)
  elseif(NOT grpe_larm_mother_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(grpe_larm_mother_FOUND FALSE)
  endif()
  return()
endif()
set(_grpe_larm_mother_CONFIG_INCLUDED TRUE)

# output package information
if(NOT grpe_larm_mother_FIND_QUIETLY)
  message(STATUS "Found grpe_larm_mother: 0.0.0 (${grpe_larm_mother_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'grpe_larm_mother' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT grpe_larm_mother_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(grpe_larm_mother_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${grpe_larm_mother_DIR}/${_extra}")
endforeach()

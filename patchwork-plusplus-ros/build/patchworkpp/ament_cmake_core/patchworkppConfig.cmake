# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_patchworkpp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED patchworkpp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(patchworkpp_FOUND FALSE)
  elseif(NOT patchworkpp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(patchworkpp_FOUND FALSE)
  endif()
  return()
endif()
set(_patchworkpp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT patchworkpp_FIND_QUIETLY)
  message(STATUS "Found patchworkpp: 0.0.0 (${patchworkpp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'patchworkpp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${patchworkpp_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(patchworkpp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${patchworkpp_DIR}/${_extra}")
endforeach()

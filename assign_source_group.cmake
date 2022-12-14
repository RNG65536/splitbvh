# Assigns the given files to source groups identical to their location.
function(assign_source_group)
  foreach(_SOURCE IN ITEMS ${ARGN})
    if (IS_ABSOLUTE "${_SOURCE}")
      file(RELATIVE_PATH _SOURCE_REL "${CMAKE_CURRENT_SOURCE_DIR}" "${_SOURCE}")
    else()
      set(_SOURCE_REL "${_SOURCE}")
    endif()
    get_filename_component(_SOURCE_PATH "${_SOURCE_REL}" PATH)
    if(WIN32)
      string(REPLACE "/" "\\" _SOURCE_PATH_MSVC "${_SOURCE_PATH}")
      source_group("${_SOURCE_PATH_MSVC}" FILES "${_SOURCE}")
    else()
      source_group("${_SOURCE_PATH}" FILES "${_SOURCE}")
    endif()
  endforeach()
endfunction(assign_source_group)

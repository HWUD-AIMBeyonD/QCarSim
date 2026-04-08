#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "twist_mux_controller::twist_mux_controller" for configuration "Release"
set_property(TARGET twist_mux_controller::twist_mux_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(twist_mux_controller::twist_mux_controller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libtwist_mux_controller.so"
  IMPORTED_SONAME_RELEASE "libtwist_mux_controller.so"
  )

list(APPEND _cmake_import_check_targets twist_mux_controller::twist_mux_controller )
list(APPEND _cmake_import_check_files_for_twist_mux_controller::twist_mux_controller "${_IMPORT_PREFIX}/lib/libtwist_mux_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "low_pass_filter::low_pass_filter" for configuration "Release"
set_property(TARGET low_pass_filter::low_pass_filter APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(low_pass_filter::low_pass_filter PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblow_pass_filter.so"
  IMPORTED_SONAME_RELEASE "liblow_pass_filter.so"
  )

list(APPEND _cmake_import_check_targets low_pass_filter::low_pass_filter )
list(APPEND _cmake_import_check_files_for_low_pass_filter::low_pass_filter "${_IMPORT_PREFIX}/lib/liblow_pass_filter.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

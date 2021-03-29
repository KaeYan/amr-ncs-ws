#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "free_fleet" for configuration ""
set_property(TARGET free_fleet APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(free_fleet PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libfree_fleet.so"
  IMPORTED_SONAME_NOCONFIG "libfree_fleet.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS free_fleet )
list(APPEND _IMPORT_CHECK_FILES_FOR_free_fleet "${_IMPORT_PREFIX}/lib/libfree_fleet.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

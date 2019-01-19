#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "osal" for configuration ""
set_property(TARGET osal APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(osal PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "rt"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libosal.so"
  IMPORTED_SONAME_NOCONFIG "libosal.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS osal )
list(APPEND _IMPORT_CHECK_FILES_FOR_osal "${_IMPORT_PREFIX}/lib/libosal.so" )

# Import target "oshw" for configuration ""
set_property(TARGET oshw APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(oshw PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "pthread"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liboshw.so"
  IMPORTED_SONAME_NOCONFIG "liboshw.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS oshw )
list(APPEND _IMPORT_CHECK_FILES_FOR_oshw "${_IMPORT_PREFIX}/lib/liboshw.so" )

# Import target "soem" for configuration ""
set_property(TARGET soem APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(soem PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "osal;oshw"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsoem.so"
  IMPORTED_SONAME_NOCONFIG "libsoem.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS soem )
list(APPEND _IMPORT_CHECK_FILES_FOR_soem "${_IMPORT_PREFIX}/lib/libsoem.so" )

# Import target "simple_test" for configuration ""
set_property(TARGET simple_test APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(simple_test PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/simple_test"
  )

list(APPEND _IMPORT_CHECK_TARGETS simple_test )
list(APPEND _IMPORT_CHECK_FILES_FOR_simple_test "${_IMPORT_PREFIX}/bin/simple_test" )

# Import target "red_test" for configuration ""
set_property(TARGET red_test APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(red_test PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/red_test"
  )

list(APPEND _IMPORT_CHECK_TARGETS red_test )
list(APPEND _IMPORT_CHECK_FILES_FOR_red_test "${_IMPORT_PREFIX}/bin/red_test" )

# Import target "slaveinfo" for configuration ""
set_property(TARGET slaveinfo APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(slaveinfo PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/slaveinfo"
  )

list(APPEND _IMPORT_CHECK_TARGETS slaveinfo )
list(APPEND _IMPORT_CHECK_FILES_FOR_slaveinfo "${_IMPORT_PREFIX}/bin/slaveinfo" )

# Import target "eepromtool" for configuration ""
set_property(TARGET eepromtool APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(eepromtool PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/eepromtool"
  )

list(APPEND _IMPORT_CHECK_TARGETS eepromtool )
list(APPEND _IMPORT_CHECK_FILES_FOR_eepromtool "${_IMPORT_PREFIX}/bin/eepromtool" )

# Import target "ebox" for configuration ""
set_property(TARGET ebox APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ebox PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/ebox"
  )

list(APPEND _IMPORT_CHECK_TARGETS ebox )
list(APPEND _IMPORT_CHECK_FILES_FOR_ebox "${_IMPORT_PREFIX}/bin/ebox" )

# Import target "firm_update" for configuration ""
set_property(TARGET firm_update APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(firm_update PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/firm_update"
  )

list(APPEND _IMPORT_CHECK_TARGETS firm_update )
list(APPEND _IMPORT_CHECK_FILES_FOR_firm_update "${_IMPORT_PREFIX}/bin/firm_update" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

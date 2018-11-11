
find_program(
  IWYU_PATH
  NAMES include-what-you-use iwyu
  DOC "Path to include-what-you-use executable"
)

if(NOT IWYU_PATH)
  message(STATUS "iwyu (include-what-you-use) not found.")
else()
  message(STATUS "iwyu found: ${IWYU_PATH}")
  option(USE_IWYU "Use analyzer include-what-you-use" ON)

  set(IWYU_PATH_AND_OPTIONS
      ${IWYU_PATH}
      -Wno-unknown-warning-option
    )
endif()

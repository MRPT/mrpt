
find_program(
  IWYU_PATH
  NAMES include-what-you-use iwyu
  DOC "Path to include-what-you-use executable"
)

if(NOT IWYU_PATH)
  if ($ENV{VERBOSE})
     message(STATUS "iwyu (include-what-you-use) not found.")
  endif()
else()
  if ($ENV{VERBOSE})
    message(STATUS "iwyu found: ${IWYU_PATH}")
  endif()
  option(USE_IWYU "Use analyzer include-what-you-use" ON)

  set(IWYU_PATH_AND_OPTIONS
      ${IWYU_PATH}
      -Wno-unknown-warning-option
    )
endif()

if (CMAKE_VERSION VERSION_GREATER "3.6.0")
find_program(
  CLANG_TIDY_EXE
  NAMES "clang-tidy"
  DOC "Path to clang-tidy executable"
  )
if(NOT CLANG_TIDY_EXE)
  message(STATUS "clang-tidy not found.")
else()
  message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
  set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "-checks=-*,cppcoreguidelines-*")
endif()
else()
message(STATUS "clang-tidy: not using. CMake version must be >=3.6")
endif()

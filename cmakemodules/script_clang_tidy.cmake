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
  option(USE_CLANG_TIDY "Use clang-tidy analyzer" ON)
  if (USE_CLANG_TIDY)
    #  cppcoreguidelines checks: not all for now...
    #  cppcoreguidelines-no-malloc
    #  cppcoreguidelines-owning-memory
    #  cppcoreguidelines-pro-bounds-array-to-pointer-decay
    #  cppcoreguidelines-pro-bounds-constant-array-index
    #  cppcoreguidelines-pro-type-const-cast
    #  cppcoreguidelines-pro-type-vararg  (used A LOT in mrpt::format()...)
    set(CLANG_TIDY_CHECKS "-checks=-*,bugprone-*,cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-pro-type-cstyle-cast,cppcoreguidelines-pro-type-member-init,cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-type-static-cast-downcast,cppcoreguidelines-pro-type-union-access,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions" CACHE STRING "clang tidy cheks to run" CACHE STRING "clang-tidy checks to run")
    mark_as_advanced(CLANG_TIDY_CHECKS)
    set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "${CLANG_TIDY_CHECKS}")
  endif()
endif()
else()
message(STATUS "clang-tidy: not using. CMake version must be >=3.6")
endif()

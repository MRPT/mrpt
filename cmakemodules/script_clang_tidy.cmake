find_program(
  CLANG_TIDY_EXE
  NAMES "clang-tidy"
  DOC "Path to clang-tidy executable"
  )
if(NOT CLANG_TIDY_EXE)
    if ($ENV{VERBOSE})
      message(STATUS "clang-tidy not found.")
  endif()
else()
  message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
  option(MRPT_USE_CLANG_TIDY "Use clang-tidy analyzer" OFF)
  if (MRPT_USE_CLANG_TIDY)
    #  cppcoreguidelines checks: not all for now...
    #  cppcoreguidelines-no-malloc
    #  cppcoreguidelines-owning-memory
    #  cppcoreguidelines-pro-bounds-array-to-pointer-decay
    #  cppcoreguidelines-pro-bounds-constant-array-index
    #  cppcoreguidelines-pro-type-const-cast
    #  cppcoreguidelines-pro-type-vararg  (used A LOT in mrpt::format()...)
    #  cppcoreguidelines-pro-bounds-pointer-arithmetic: permit this one
    #  cppcoreguidelines-pro-type-cstyle-cast: just too many to fix
    #  cppcoreguidelines-pro-type-reinterpret-cast: too many, we need to support C API for sensors, sockets, etc.
    set(CLANG_TIDY_CHECKS "-checks=-*,bugprone-*,-bugprone-narrowing-conversions,-cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-pro-type-member-init,cppcoreguidelines-pro-type-static-cast-downcast,cppcoreguidelines-pro-type-union-access,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions" CACHE STRING "clang tidy cheks to run")
    mark_as_advanced(CLANG_TIDY_CHECKS)
    set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "${CLANG_TIDY_CHECKS}")
  endif()
endif()

option(MRPT_EXCEPTIONS_WITH_CALL_STACK "Report callstack upon exceptions" "ON")
set(MRPT_EXCEPTIONS_CALL_STACK_MAX_DEPTH "50" CACHE STRING "Maximum number of stack levels to show upon exceptions.")
mark_as_advanced(MRPT_EXCEPTIONS_CALL_STACK_MAX_DEPTH)

# MRPT_TRY_START/END blocks
# ===================================================
set(MRPT_HAS_STACKED_EXCEPTIONS ON CACHE BOOL "Enable MRPT_TRY_START/END blocks (disable it for speed up).")

# ASSERT_ blocks
set(MRPT_HAS_ASSERT ON CACHE BOOL "Enable ASSERT_ statements (disable it for speed up).")


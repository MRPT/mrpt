\defgroup mrpt_core_exception_grp Nested exceptions helpers
\ingroup mrpt_core_grp

MRPT exceptions carry the call stack at the throw point, so an error raised
deep inside a library can be traced back from the top-level `catch`. All
these helpers are defined in `<mrpt/core/exceptions.h>`.

# Throwing and asserting

- `THROW_EXCEPTION(msg)`, `THROW_EXCEPTION_FMT(fmt, ...)`: throw a
  `std::logic_error` with the message, source file, line, function name and
  call stack. `THROW_TYPED_EXCEPTION(msg, type)` throws another exception
  class.
- `ASSERT_(cond)`, `ASSERTMSG_(cond, msg)`: throw if the condition is false.
- `ASSERT_EQUAL_(a, b)`, `ASSERT_NOT_EQUAL_()`, `ASSERT_LT_()`,
  `ASSERT_LE_()`, `ASSERT_GT_()`, `ASSERT_GE_()`, `ASSERT_NEAR_(a, b, tol)`:
  comparisons whose error message includes the values of both operands.
- `ASSERTDEB_(cond)` and the other `ASSERTDEB_*` variants: like the above,
  but only checked if `_DEBUG` is defined.

# Tracing exceptions through the call stack

Wrapping the body of a function between `MRPT_START` and `MRPT_END` catches
any `std::exception` thrown inside it (also from the standard library or
third-party code) and rethrows it wrapped in an
mrpt::ExceptionWithCallBack, which records the call stack. Exceptions already
wrapped are passed through unchanged. `MRPT_END_WITH_CLEAN_UP(stuff)` also
runs some code before rethrowing.

At the top level, print the full report with `e.what()` or
mrpt::exception_to_str(). File names and line numbers of each frame are
shown when debug symbols are available.

# Example

See: \ref core_exceptions_example/main.cpp
\snippet core_exceptions_example/main.cpp example-nested-exceptions
Output:
\include core_exceptions_example/console.out

See also \ref core_backtrace_example, which captures the current call stack
with mrpt::callStackBackTrace() without throwing.

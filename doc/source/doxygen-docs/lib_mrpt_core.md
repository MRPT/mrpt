\defgroup mrpt_core_grp [mrpt-core]

Core components for MRPT.

[TOC]

# Library mrpt-core

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-core-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

`mrpt-core` provides the foundation utilities used throughout MRPT:

- **Exceptions and assertions**: `MRPT_START`/`MRPT_END`, `ASSERT_*` macros,
  `mrpt::exception_to_str()`.
- **Logging**: mrpt::system::COutputLogger — flexible leveled logging (DEBUG,
  INFO, WARN, ERROR) with optional callbacks and console color output.
- **Time**: mrpt::Clock — a high-resolution monotonic clock; mrpt::system::TTimeStamp
  for timestamps; mrpt::system::CTimeLogger for named profiling.
- **String utilities**: `mrpt::format()` (printf-style into `std::string`),
  `mrpt::system::trim()`, tokenizers, etc.
- **Filesystem helpers**: `mrpt::system::fileExists()`,
  `mrpt::system::createDirectory()`, `mrpt::system::extractFileExtension()`, etc.
- **Smart containers**: `mrpt::containers::circular_buffer`,
  `mrpt::containers::bimap`, `mrpt::containers::ts_hash_map` (thread-safe).
- **Thread utilities**: `mrpt::lockHelper()`, `mrpt::WorkerThreadsPool`.
- **Serialization base**: `mrpt::serialization::CSerializable` — base class for
  all serializable objects; versioned `serializeTo`/`serializeFrom` interface.
- **RTTI**: `mrpt::rtti::CObject` — lightweight run-time type info for MRPT
  class hierarchies; `CLASS_ID()`, `IS_CLASS()`, `IS_DERIVED()` macros.

# Library contents

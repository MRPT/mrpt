\defgroup mrpt_system_grp [mrpt-system]

An OS abstraction layer and other miscellaneous functionality: filesystem, date
and time, memory handling and much more.

[TOC]

# Library mrpt-system


This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-system-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

## Overview

`mrpt-system` gathers portable (Linux, Windows, macOS) utilities that most
MRPT programs need:

- **Filesystem**: `<mrpt/system/filesystem.h>` with functions such as
  mrpt::system::fileExists(), mrpt::system::createDirectory(),
  mrpt::system::extractFileExtension() and mrpt::system::pathJoin().
- **Date and time**: mrpt::system::TTimeStamp (an mrpt::Clock time point) and
  conversions to and from text in `<mrpt/system/datetime.h>`.
- **Logging and profiling**: mrpt::system::COutputLogger, the base of all
  MRPT classes that emit log messages with verbosity levels, and
  mrpt::system::CTimeLogger, which collects time statistics of named code
  sections.
- **Loop timing**: mrpt::system::CRateTimer and
  mrpt::system::CControlledRateTimer run a loop at a fixed rate.
- **Files and processes**: mrpt::system::CDirectoryExplorer,
  mrpt::system::CFileSystemWatcher, mrpt::system::executeCommand().
- **Strings**: mrpt::system::tokenize(), mrpt::system::trim(), base64 and
  UTF-8 encoding, mrpt::system::md5(), mrpt::system::compute_CRC32().

## Example: profiling a code block

```cpp
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>

mrpt::system::CTimeLogger profiler;  // prints its stats when destroyed

{
  // RAII: measures the time until the end of this scope
  mrpt::system::CTimeLoggerEntry tle(profiler, "prepare_output");

  const std::string dir = mrpt::system::pathJoin({mrpt::system::getcwd(), "out"});
  if (!mrpt::system::directoryExists(dir))
  {
    mrpt::system::createDirectory(dir);
  }
}

const mrpt::system::TTimeStamp t = mrpt::Clock::now();
std::cout << "Now: " << mrpt::system::dateTimeLocalToString(t) << "\n";
```

## Examples

- \ref system_datetime_example: timestamps and date/time formatting.
- \ref system_dirs_files_manipulation: creating, renaming and deleting files
  and directories.
- \ref system_directory_explorer_example, \ref system_file_system_watcher:
  listing and watching directories.
- \ref system_control_rate_timer_example: running a loop at a fixed rate.
- \ref system_progress_bar: a console progress bar.
- Python: [mrpt_system_example.py](pymrpt_example_mrpt_system_example.html).

# Library contents

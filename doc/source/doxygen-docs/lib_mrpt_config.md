\defgroup mrpt_config_grp [mrpt-config]

C++ library for configuration files in the "INI" file format.

For YAML format, see mrpt::containers::yaml. For generic serialization to
schema-based files, see \ref mrpt_serialization_grp

[TOC]

# Library mrpt-config

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-config-dev

See: \ref mrpt_from_cmake

See: \ref config_file_format

## Overview

Most MRPT algorithms (maps, SLAM, localization, navigation, sensor drivers)
read their parameters from INI-like files with `[sections]` and
`key = value` lines. This module implements reading and writing those files,
including the MRPT extensions to the format: `@define` variables,
`$eval{...}` math expressions and `$env{...}` environment variables (see
\ref config_file_format).

## Key classes

- mrpt::config::CConfigFileBase: the common API to read and write typed
  values (`read_double()`, `read_int()`, `read_string()`, `read_vector()`,
  `read_enum()`, `write()`...), with a default value or an exception if a
  key is missing.
- mrpt::config::CConfigFile: a configuration file on disk.
- mrpt::config::CConfigFileMemory: the same, for text held in memory.
- mrpt::config::CConfigFilePrefixer: a view of another config object that
  adds prefixes to section or key names, to emulate nested sections.
- mrpt::config::CLoadableOptions: base of all MRPT parameter structures that
  can be loaded from (and saved to) a section of a config file. Macros like
  `MRPT_LOAD_CONFIG_VAR()` help implementing `loadFromConfigFile()`.

## Example

```cpp
#include <mrpt/config/CConfigFileMemory.h>

// A CConfigFile would read the same contents from a file on disk:
mrpt::config::CConfigFileMemory cfg(R"(
[robot]
name      = rover1
max_speed = 0.8   // m/s
)");

const double v = cfg.read_double("robot", "max_speed", 0.5);
const int n = cfg.read_int("robot", "num_wheels", 4);  // not present: 4
const std::string name = cfg.read_string("robot", "name", "", true /*fail if not found*/);

cfg.write("robot", "max_accel", 1.2);
std::cout << cfg.getContent();
```

## Examples

- Python: [mrpt_config_example.py](pymrpt_example_mrpt_config_example.html).
- Many real configuration files for MRPT applications can be found under
  [modules/mrpt_data/config_files](https://github.com/MRPT/mrpt/tree/develop/modules/mrpt_data/config_files).

# Library contents

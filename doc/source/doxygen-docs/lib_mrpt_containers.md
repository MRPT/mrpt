\defgroup mrpt_containers_grp [mrpt-containers]

Container and STL-helper classes.

[TOC]

# Library mrpt-containers

This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-containers-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

Find below some examples of use.

## Overview

`mrpt-containers` provides generic containers that complement the C++
standard library, plus a YAML/JSON document class used throughout MRPT to
pass and store parameters. It only depends on \ref mrpt_core_grp and
\ref mrpt_typemeta_grp.

## Key classes

- mrpt::containers::yaml: a YAML/JSON document (maps, sequences, scalars and
  comments) that can be parsed from text or files, built in code, queried
  with type conversion, and printed back as YAML.
- mrpt::containers::circular_buffer: a fixed-capacity ring buffer.
- mrpt::containers::CThreadSafeQueue: a FIFO queue to pass objects between
  threads.
- mrpt::containers::bimap: a bidirectional map, searchable by key or value.
- mrpt::containers::CDynamicGrid, mrpt::containers::CDynamicGrid3D: 2D and 3D
  grids of cells that can grow as needed, the basis of many grid maps.
- mrpt::containers::vector_with_small_size_optimization: a vector that only
  uses the heap above a given size.

## YAML-like parameter container

See mrpt::containers::yaml, and the examples therein.

```cpp
#include <mrpt/containers/yaml.h>

const auto p = mrpt::containers::yaml::FromText(R"(
robot:
  name: rover1
  max_speed: 0.8
  sensors: [lidar, imu]
)");

const auto& robot = p["robot"];
const double v = robot["max_speed"].as<double>();
const int n = robot.getOrDefault<int>("num_wheels", 4);  // not present: 4
for (const auto& s : robot["sensors"].asSequence())
{
  std::cout << s.as<std::string>() << "\n";
}

mrpt::containers::yaml out = mrpt::containers::yaml::Map();
out["v"] = v;
out["n"] = n;
out.printAsYAML(std::cout);
```

Note that references to nodes of a map or sequence are invalidated when new
siblings are added to it.

## Examples

- \ref containers_yaml_example: parsing, building, iterating and printing
  YAML documents, including comments.
- \ref containers_params_by_name: passing a variable set of named parameters
  to a function.
- Python: [mrpt_containers_example.py](pymrpt_example_mrpt_containers_example.html).

# Library contents

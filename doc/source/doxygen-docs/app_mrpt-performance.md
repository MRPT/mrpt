\page app_mrpt-performance mrpt-performance


Benchmark suite of MRPT: runs a set of timing tests of common operations
(poses and matrices math, random numbers, images, point clouds, occupancy
and 3D grid maps, octomaps, ICP and scan matching, ICP-SLAM on a sample
dataset, graphs and graph-SLAM, 3D range scans, strings, YAML, system
functions, ...) and reports the time per execution of each one. Use it to
compare the performance of different MRPT versions, compilers or machines,
or to detect performance regressions.

# Usage

    mrpt-performance [options]

| Option | Description |
|---|---|
| `-c`, `--match-contains TEXT` | Run only the tests whose name contains this substring. |
| `-t`, `--tables` | Don't run any test; build the HTML comparison tables from the stored results instead. |
| `-r`, `--release` | Don't add the `dev` postfix to the name of the results file. |
| `-h`, `--help`, `--version` | Show help or version. |

Some tests use datasets from `mrpt_data` (e.g. the ICP-SLAM and RGB-D
tests). A test that throws an exception (e.g. missing data) is reported as
skipped, and the rest continue.

# Examples

    # Run all tests:
    mrpt-performance

    # Run only the ICP-related tests:
    mrpt-performance -c icp

# Output

Results are printed to the console (time per test) and written to these
files in the current directory:

- `mrpt-performance.html`: a table with the description, execution time and
  rate of each test, plus the CPU information of the machine.
- `perf-results-<version>[dev]-<compiler>-<bits>bit.dat`: the results in
  binary form, for later comparison.

With `--tables`, all the `perf-results-*.dat` files in the current directory
are read, and an `index.html`, one page per configuration and one comparison
page per pair of configurations are generated in the `perf-html/`
subdirectory, which must exist.

# See also

- mrpt::system::CTimeLogger, for profiling your own code.

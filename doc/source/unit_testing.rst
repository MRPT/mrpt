.. _unit_testing:

======================
Unit testing in MRPT
======================

Running unit tests
----------------------

Useful build targets: 

- ``make test``: Uses CTest to run all tests, compiling them first if needed. 
  By default, no output is sent to console, but it can be changed defining the 
  environment variable ``CTEST_OUTPUT_ON_FAILURE=1`` to show the output of failing tests.

- ``make test_legacy``: A custom MRPT target that runs all tests, showing all their 
  outputs to console.

- ``make tests_build_all``: Compile all unit test targets, so a subsequent call to run
  tests does not need to build anything.

- ``make test_gdb``: Runs all tests under ``gdb``, printing the stack trace on crashes.

- ``make test_valgrind``: Runs all tests inside ``valgrind``, a great tool to find memory leaks and other memory errors.

- ``make test_helgrind``: Runs all tests inside ``valgrind``, with the tool ``helgrind`` to detect multithreading errors.

- ``make test_mrpt_foo``: Builds the test application for the mrpt library "foo".

- ``make run_tests_mrpt_foo``: Builds and runs the test for the mrpt library "foo".


Rationale
-------------

Each MRPT `library <modules.html>`_ has its own set of `unit tests <https://en.wikipedia.org/wiki/Unit_testing>`_ 
to verify and enssure that classes and functions behave as expected.
There are hundreds of tests covering operations from elemental matrix operations up to the execution of complex SLAM
algorithms with predefined datasets, in many cases using randomized data and several input datasets for each test.
In all cases, the results are compared to the expected values and errors are reported upon mismatches.
This reduces the chances of introducing `regressions <https://en.wikipedia.org/wiki/Software_regression>`_ in the future.

MRPT uses Google ``gtest`` unit testing library.
CMake scripts take care of of recognizing all those source files that implement tests 
(which are identified by filename suffix ``*_unittest.cpp``, as explained `here <tutorial-lib-layout.html>`_)
and move them into special binary targets which are executed upon testing.
One test target is created per mrpt module. 
Take a look at any file named ``*_unittest.cpp`` for examples (e.g. `math/src/geometry_unittest.cpp <https://github.com/MRPT/mrpt/blob/develop/libs/math/src/geometry_unittest.cpp>`_)


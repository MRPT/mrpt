.. _mrpt_from_cmake:

####################################
Using MRPT in your CMake project
####################################

.. note::
    See online complete example with a CMake + .cpp file `here <https://github.com/MRPT/mrpt/tree/develop/doc/mrpt_example1>`_.

Finding MRPT from CMake
-------------------------

Each MRPT library is an independent CMake package named ``mrpt_<module>``,
which exports the imported target ``mrpt::mrpt_<module>``. Find only the
libraries you use: their dependencies are found and linked transitively.

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.16)
  project(myapp)

  find_package(mrpt_poses REQUIRED)
  find_package(mrpt_gui REQUIRED)

  add_executable(myapp main.cpp)

  # This also adds all required flags, include directories, etc.
  target_link_libraries(myapp
    mrpt::mrpt_poses
    mrpt::mrpt_gui
  )

Optionally, ``find_package(mrpt_common REQUIRED)`` provides the helpers
``mrpt_add_executable()`` and ``mrpt_add_library()`` used by MRPT itself, which
also set the C++ standard and compiler flags.

Library headers are included as ``#include <mrpt/<module>/<Class>.h>``, and
their contents live in the ``mrpt::<module>`` namespace.

For MRPT 2.x
-------------------------

In MRPT 2.x, packages were named ``mrpt-<module>`` and targets ``mrpt::<module>``:

.. code-block:: cmake

  find_package(mrpt-poses)
  find_package(mrpt-gui)

  add_executable(myapp  main.cpp)
  target_link_libraries(myapp mrpt::poses mrpt::gui)

See :doc:`page_porting_mrpt3` for all the changes between MRPT 2.x and 3.x.

For MRPT 1.x
-------------------------

Prior to MRPT 2.0.0, the correct way to search for MRPT was:

.. code-block:: cmake

  # Find MRPT libraries:
  find_package(MRPT REQUIRED poses gui)

  add_executable(myapp  main.cpp)
  target_link_libraries(myapp ${MRPT_LIBRARIES})

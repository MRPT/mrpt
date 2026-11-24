.. _downloadmrpt:

##############
Download MRPT
##############

MRPT 3 can be installed in several ways. Pick the one that fits your project:

.. list-table::
  :header-rows: 1
  :widths: 22 40 38

  * - Route
    - Best for
    - Contents
  * - `Python (pip)`_
    - Python users, notebooks, quick experiments
    - Python bindings, plus C++ headers and libraries
  * - `ROS 2`_
    - ROS 2 users and packages
    - Every module as a separate package, plus the apps
  * - `Ubuntu PPA`_
    - C++ development on Ubuntu without ROS
    - C++ libraries, headers and apps
  * - `Build from sources`_
    - Contributors, other platforms (Windows, macOS), custom options
    - Everything

Python (pip)
============

.. code-block:: bash

   pip install mrpt3

Linux (x86_64, aarch64), Python 3.10 or newer.

.. code-block:: python

   from mrpt.poses import CPose3D
   print(CPose3D(1.0, 2.0, 0.0, 0.5, 0.0, 0.0) + CPose3D(0.5, 0.0, 0.0, 0.0, 0.0, 0.0))

The wheel includes the Python bindings of all modules except hardware drivers,
plus the C++ headers, shared libraries and CMake files (``mrpt.sdk``), so other
wheels can build against them. ``mrpt.opengl`` and ``mrpt.gui`` need the system
OpenGL libraries. See the :ref:`python_api`, the :ref:`python_examples`, and the
interactive `MRPT notebooks <https://github.com/MRPT/mrpt-notebooks>`_.

ROS 2
=====

MRPT 3 is released into ROS 2 Humble, Jazzy, Kilted, Lyrical and Rolling, with
one package per module:

.. code-block:: bash

   # All libraries and apps:
   sudo apt install ros-$ROS_DISTRO-mrpt-apps-cli ros-$ROS_DISTRO-mrpt-apps-gui
   # Or only the modules you need, e.g.:
   sudo apt install ros-$ROS_DISTRO-mrpt-maps

Binary builds of every package, per distro and platform (Ubuntu amd64/arm64, RHEL, Fedora):
`ROS build farm status <https://github.com/MRPT/mrpt/blob/develop/packaging/ROS_BUILD_STATUS.md>`_.
To use MRPT from your own ROS 2 packages, read :doc:`tutorial-mrpt-ros2`.

Ubuntu PPA
==========

For Ubuntu 24.04 and 26.04:

.. code-block:: bash

   sudo add-apt-repository ppa:joseluisblancoc/mrpt3-stable   # or mrpt3-develop, for nightly builds
   sudo apt install libmrpt-dev mrpt-apps

PPA status: `mrpt3-stable <https://launchpad.net/~joseluisblancoc/+archive/ubuntu/mrpt3-stable>`_
(releases, from the ``master`` branch) and
`mrpt3-develop <https://launchpad.net/~joseluisblancoc/+archive/ubuntu/mrpt3-develop>`_
(nightly builds, from the ``develop`` branch).

.. note::
   The ``mrpt`` packages in the official Debian and Ubuntu repositories may be
   older than the PPA. See `which distributions package MRPT <https://repology.org/project/mrpt/versions>`_.

Build from sources
==================

.. code-block:: bash

   git clone https://github.com/MRPT/mrpt.git --recursive

Then follow :ref:`compiling`. This is the route for Windows (Visual Studio) and
macOS, which are tested in continuous integration.

Using MRPT from your CMake project
==================================

Whatever the install route, C++ projects find MRPT modules with ``find_package()``.
See :ref:`mrpt_from_cmake`.

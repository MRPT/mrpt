.. _tutorial-first-program:

###############################
Your first MRPT 3 program
###############################

This tutorial builds a small, complete program, in C++ and in Python, that
aligns two 2D point clouds with the Iterative Closest Point (ICP) algorithm:

- a random 2D point cloud plays the role of a reference map,
- a copy of it, seen from a different pose, plays the role of a new scan,
- `mrpt::slam::CICP <class_mrpt_slam_CICP.html>`_ recovers that relative pose,
  together with its uncertainty (a covariance matrix).

.. contents:: :local:


1. Install MRPT
==================

You only need one of these. See :ref:`downloadmrpt` for all options.

.. tab-set::

    .. tab-item:: Python (pip)

        Linux (x86_64, aarch64), Python 3.10 or newer:

        .. code-block:: bash

            pip install mrpt3

        The wheel also ships the C++ headers, libraries and CMake files, so it
        is enough for the C++ program too (see below).

    .. tab-item:: ROS 2

        .. code-block:: bash

            sudo apt install ros-$ROS_DISTRO-mrpt-slam
            # or everything, including all apps:
            sudo apt install ros-$ROS_DISTRO-mrpt-apps-cli ros-$ROS_DISTRO-mrpt-apps-gui

        Then ``source /opt/ros/$ROS_DISTRO/setup.bash`` in each terminal.
        This installs the Python bindings as well. See :ref:`tutorial-mrpt-ros2`.

    .. tab-item:: Ubuntu PPA

        .. code-block:: bash

            sudo add-apt-repository ppa:joseluisblancoc/mrpt3-stable
            sudo apt install libmrpt-dev mrpt-apps

    .. tab-item:: From source

        .. code-block:: bash

            git clone https://github.com/MRPT/mrpt.git --recursive
            cd mrpt
            colcon build

        Then ``source install/setup.bash``. See :ref:`compiling` for
        dependencies and build options.


2. The C++ program
=====================

Save this as ``first_icp.cpp``:

.. code-block:: cpp

    #include <mrpt/maps/CSimplePointsMap.h>
    #include <mrpt/poses/CPose2D.h>
    #include <mrpt/random.h>
    #include <mrpt/slam/CICP.h>

    #include <cmath>
    #include <iostream>

    int main()
    {
      auto& rng = mrpt::random::getRandomGenerator();
      rng.randomize(1234);  // fixed seed: same output on every run

      // A random 2D point cloud: the reference map.
      mrpt::maps::CSimplePointsMap ref;
      for (int i = 0; i < 200; i++)
      {
        ref.insertPoint(rng.drawUniform(-10.0f, 10.0f), rng.drawUniform(-10.0f, 10.0f));
      }

      // The same points, as seen from a sensor at "truePose" (x, y, phi):
      const mrpt::poses::CPose2D truePose(0.5, -0.3, mrpt::DEG2RAD(10.0));
      mrpt::maps::CSimplePointsMap scan = ref;
      scan.changeCoordinatesReference(-truePose);

      // Align them with ICP, starting from the identity:
      mrpt::slam::CICP icp;
      icp.options.maxIterations = 100;
      // Initial max. pairing distance: thresholdDist (m) + thresholdAng * range
      icp.options.thresholdDist = 1.0;
      icp.options.thresholdAng = mrpt::DEG2RAD(10.0);

      mrpt::slam::CICP::TReturnInfo info;
      const auto pdf = icp.Align(&ref, &scan, mrpt::poses::CPose2D(), info);

      // The result is a probability density: a mean and a covariance.
      const auto [cov, mean] = pdf->getCovarianceAndMean();

      std::cout << "True pose:      " << truePose << "\n";
      std::cout << "ICP estimate:   " << mean << "\n";
      std::cout << "Iterations:     " << info.nIterations << "\n";
      std::cout << "Goodness:       " << info.goodness << "\n";
      std::cout << "Covariance (x, y, phi):\n" << cov << "\n";
      std::cout << "Std. dev.: x=" << std::sqrt(cov(0, 0)) << " m, y=" << std::sqrt(cov(1, 1))
                << " m, phi=" << mrpt::RAD2DEG(std::sqrt(cov(2, 2))) << " deg\n";
    }

Some remarks:

- ``changeCoordinatesReference(-truePose)`` expresses all points relative to
  ``truePose``, which is what a sensor placed there would see.
- ``Align()`` returns a smart pointer to a
  `mrpt::poses::CPosePDF <class_mrpt_poses_CPosePDF.html>`_: a probability
  distribution over SE(2) poses, not just a single pose. See
  :ref:`tutorial-pdf-over-poses`.
- The initial guess (the identity) is 0.5 m and 10 degrees away from the
  solution: at 10 m from the origin, that rotation alone displaces points by
  1.7 m. That is why the correspondence thresholds are enlarged here: with the
  default ones (0.75 m and 0.15 degrees), ICP gets stuck in a wrong local
  minimum. See :ref:`tutorial-icp-alignment` for all ICP options.


3. CMakeLists.txt
====================

Each MRPT library is an independent CMake package named ``mrpt_<module>``,
which exports the target ``mrpt::mrpt_<module>``. Find only the libraries you
use directly: their dependencies (here, ``mrpt_maps``, ``mrpt_poses``,
``mrpt_random``...) come along transitively. See :ref:`mrpt_from_cmake`.

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.16)
    project(my_first_mrpt_app LANGUAGES CXX)

    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    # Each MRPT library is a separate CMake package; its dependencies come along:
    find_package(mrpt_slam REQUIRED)

    add_executable(first_icp first_icp.cpp)
    target_link_libraries(first_icp mrpt::mrpt_slam)


4. Build and run
===================

With MRPT installed from ROS 2 (after sourcing its ``setup.bash``), from the
PPA, or from source (after sourcing ``install/setup.bash``), CMake finds it
as is:

.. code-block:: bash

    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build
    ./build/first_icp

With the pip wheel, pass its CMake prefix in the configure step instead:

.. code-block:: bash

    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$(python3 -c 'import mrpt.sdk; print(mrpt.sdk.get_cmake_prefix_path())')"

Output:

.. code-block:: text

    True pose:      (0.500,-0.300,10.00deg)
    ICP estimate:   (0.500,-0.300,10.00deg)
    Iterations:     19
    Goodness:       1
    Covariance (x, y, phi):
     1.0083e-05  2.0744e-08 1.04957e-07
     2.0744e-08 1.00052e-05 2.62433e-08
    1.04957e-07 2.62433e-08 1.32781e-07
    Std. dev.: x=0.00317537 m, y=0.0031631 m, phi=0.0208781 deg

The estimate matches the true pose, and the covariance tells how certain it
is: a few millimeters and hundredths of a degree, since these points have no
noise at all.


5. Optional: show the result in a 3D window
==============================================

To visualize both point clouds with `mrpt::gui::CDisplayWindow3D
<class_mrpt_gui_CDisplayWindow3D.html>`_, add these headers:

.. code-block:: cpp

    #include <mrpt/gui/CDisplayWindow3D.h>
    #include <mrpt/viz/CGridPlaneXY.h>

this code at the end of ``main()``:

.. code-block:: cpp

    // Optional: show the reference map (blue) and the scan moved to the ICP
    // estimate (red). They should overlap perfectly.
    scan.changeCoordinatesReference(mean);
    ref.renderOptions.point_size = 6.0f;
    scan.renderOptions.point_size = 3.0f;
    scan.renderOptions.color = mrpt::img::TColorf(1.0f, 0.0f, 0.0f);

    mrpt::gui::CDisplayWindow3D win("ICP result", 800, 600);
    {
      auto& scene = win.get3DSceneAndLock();
      scene->insert(mrpt::viz::CGridPlaneXY::Create(-10, 10, -10, 10, 0, 1));
      scene->insert(ref.getVisualization());
      auto glScan = scan.getVisualization();
      glScan->setLocation(0, 0, 0.05);  // slightly above, so it is not hidden
      scene->insert(glScan);
      win.unlockAccess3DScene();
    }
    win.setCameraZoom(30);
    win.repaint();
    std::cout << "Press a key to exit.\n";
    win.waitForKey();

and link against ``mrpt_gui`` too:

.. code-block:: cmake

    find_package(mrpt_slam REQUIRED)
    find_package(mrpt_gui REQUIRED)

    add_executable(first_icp first_icp.cpp)
    target_link_libraries(first_icp mrpt::mrpt_slam mrpt::mrpt_gui)

Every red point should lie on top of a blue one. See
:doc:`tutorial-3d-navigation-cheatsheet` to move the camera with the mouse.


6. The same in Python
========================

The Python API mirrors the C++ one (see :ref:`python_api`). Save this as
``first_icp.py``:

.. code-block:: python

    import math

    import numpy as np

    from mrpt.maps import CSimplePointsMap
    from mrpt.poses import CPose2D, CPosePDFGaussian
    from mrpt.random import getRandomGenerator
    from mrpt.slam import CICP

    rng = getRandomGenerator()
    rng.randomize(1234)  # fixed seed: same output on every run

    # A random 2D point cloud (the reference map), and the same points
    # as seen from a sensor at "true_pose" (x, y, phi):
    true_pose = CPose2D(0.5, -0.3, math.radians(10.0))
    ref = CSimplePointsMap()
    scan = CSimplePointsMap()
    for _ in range(200):
        x = rng.drawUniform(-10.0, 10.0)
        y = rng.drawUniform(-10.0, 10.0)
        ref.insertPoint(x, y)
        local = true_pose.inverseComposePoint(x, y)
        scan.insertPoint(local.x, local.y)

    # Align them with ICP, starting from the identity:
    icp = CICP()
    icp.options.maxIterations = 100
    # Initial max. pairing distance: thresholdDist (m) + thresholdAng * range
    icp.options.thresholdDist = 1.0
    icp.options.thresholdAng = math.radians(10.0)

    pdf, info = icp.AlignPDF(ref, scan, CPosePDFGaussian())
    cov, mean = pdf.getCovarianceAndMean()

    print("True pose:   ", true_pose)
    print("ICP estimate:", mean)
    print("Iterations:  ", info.nIterations)
    print(f"Goodness:     {info.goodness:.3f}")
    print("Covariance (x, y, phi):")
    print(cov)

    std = np.sqrt(np.diag(cov.as_numpy()))
    print(f"Std. dev.: x={std[0]:.5f} m, y={std[1]:.5f} m, phi={math.degrees(std[2]):.4f} deg")

Differences with the C++ version: ``AlignPDF()`` takes the initial guess as a
Gaussian pose PDF and returns a ``(pdf, info)`` tuple, and ``as_numpy()``
converts MRPT matrices into NumPy arrays.

Run it with ``python3 first_icp.py``:

.. code-block:: text

    True pose:    [0.500000 -0.300000 10.000000deg]
    ICP estimate: [0.500000 -0.300000 10.000000deg]
    Iterations:   16
    Goodness:     1.000
    Covariance (x, y, phi):
    [CMatrixDouble33]
    1.00482e-05 7.63716e-10 7.99814e-08
    7.63716e-10       1e-05 1.26788e-09
    7.99814e-08 1.26788e-09 1.32781e-07
    Std. dev.: x=0.00317 m, y=0.00316 m, phi=0.0209 deg

The random points differ slightly from the C++ run, hence the different
number of iterations, but the result is the same.

Optionally, show the result in a 3D window by appending this to the script.
It uses `mrpt::gui::CDisplayWindowGUI <class_mrpt_gui_CDisplayWindowGUI.html>`_,
and runs until the window is closed:

.. code-block:: python

    # Optional: show the reference map (blue) and the scan moved to the ICP
    # estimate (red). They should overlap perfectly.
    import time

    from mrpt.gui import CDisplayWindowGUI
    from mrpt.viz import CGridPlaneXY, CPointCloud, Scene

    aligned = []
    for i in range(len(scan)):
        x, y, _ = scan.getPoint(i)
        p = mean.composePoint(x, y)
        aligned.append([p.x, p.y, 0.05])  # slightly above, so it is not hidden

    gl_ref = CPointCloud()
    gl_ref.setPoints(ref.getPointsAsNumpy())
    gl_ref.setPointSize(6.0)
    gl_ref.setColor(0.0, 0.0, 1.0)

    gl_aligned = CPointCloud()
    gl_aligned.setPoints(np.array(aligned))
    gl_aligned.setPointSize(3.0)
    gl_aligned.setColor(1.0, 0.0, 0.0)

    scene = Scene()
    scene.insert(CGridPlaneXY())
    scene.insert(gl_ref)
    scene.insert(gl_aligned)

    win = CDisplayWindowGUI("ICP result", 800, 600)
    win.camera().setZoomDistance(30)
    win.background_scene = scene
    while win.spinOnce():  # until the window is closed
        time.sleep(0.02)


Next steps
==============

- More C++ examples: :ref:`examples`. Python examples: :ref:`python_examples`.
- ICP in depth: :ref:`tutorial-icp-alignment`.
- Using MRPT from ROS 2 packages: :ref:`tutorial-mrpt-ros2`.
- 3D LiDAR odometry and mapping built on MRPT: :ref:`tutorial-mrpt-mola`.

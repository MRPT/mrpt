####
MRPT
####

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Get it

  download-mrpt
  compiling
  license
  page_changelog
  authors

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Learn it

  tutorials
  applications
  examples
  python_examples
  wrappers
  supported-sensors
  unit_testing
  children-projects

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Advanced

  modules
  python_api
  contributing
  bibliography
  doxygen-index
  Docs for other versions <https://docs.mrpt.org/reference/>

.. raw:: html

  <div class="mrpt-hero">
    <p class="mrpt-tagline"><b>The Mobile Robot Programming Toolkit</b>: C++ and Python
    libraries and apps for SLAM, localization, sensors, 3D geometry and visualization.</p>
    <a href="https://mrpt.github.io/videos/mrpt_hero_v1.mp4"><img
      src="https://mrpt.github.io/videos/mrpt_hero_v2.avif"
      alt="MRPT in action: mapping and navigation with MOLA and MRPT, LiDAR odometry, MRPT GUI apps, 3D rendering and MVSim"/></a>
    <p class="mrpt-hero-buttons">
      <a class="sd-btn sd-btn-primary sd-shadow-sm" href="download-mrpt.html">Install MRPT</a>
      <a class="sd-btn sd-btn-outline-primary" href="tutorial-first-program.html">Your first program</a>
      <a class="sd-btn sd-btn-outline-secondary" href="https://github.com/MRPT/mrpt">GitHub repository</a>
    </p>
  </div>

What is MRPT?
=============

MRPT is a set of portable, well-tested and open-source (`BSD-3 <license.html>`_)
C++ libraries, Python bindings and ready-to-use `applications <applications.html>`_
for mobile robotics and computer vision research and development. It powers the
`MOLA <https://github.com/MOLAorg/mola>`_ SLAM framework and the
`MVSim <https://github.com/MRPT/mvsim>`_ simulator, and it is released into ROS 2.

.. grid:: 1 2 3 3
  :gutter: 3

  .. grid-item-card:: Sensors and datasets
    :link: group_mrpt_obs_grp
    :link-type: doc
    :class-card: mrpt-feature-card

    Drivers for LiDARs, cameras, IMUs and GNSS; a common observation model, and
    the rawlog dataset format with tools to record, inspect and edit datasets.

  .. grid-item-card:: Maps and SLAM
    :link: group_mrpt_maps_grp
    :link-type: doc
    :class-card: mrpt-feature-card

    Point clouds, occupancy grids, voxel and landmark maps; ICP, particle filter,
    EKF and graph SLAM and localization.

  .. grid-item-card:: Poses and probability
    :link: group_mrpt_poses_grp
    :link-type: doc
    :class-card: mrpt-feature-card

    2D/3D poses and points, Lie groups, uncertainty propagation, Gaussian and
    particle-based PDFs, Bayesian filtering.

  .. grid-item-card:: Navigation and planning
    :link: group_mrpt_nav_grp
    :link-type: doc
    :class-card: mrpt-feature-card

    Reactive navigation with trajectory generators (PTGs), holonomic methods and
    obstacle avoidance; kinematics.

  .. grid-item-card:: 3D graphics and GUIs
    :link: group_mrpt_viz_grp
    :link-type: doc
    :class-card: mrpt-feature-card

    Scene graphs, offscreen rendering, 3D windows and plots, and ImGui-based
    GUIs for your own tools.

  .. grid-item-card:: Math, geometry, images
    :link: group_mrpt_math_grp
    :link-type: doc
    :class-card: mrpt-feature-card

    Matrices, optimization, RANSAC, geometry, images and camera models,
    serialization and many system utilities.

Where to start
==============

.. grid:: 1 2 3 3
  :gutter: 3

  .. grid-item-card:: Install
    :link: downloadmrpt
    :link-type: ref
    :class-card: mrpt-feature-card

    pip, ROS 2, Ubuntu PPA, or build from sources.

  .. grid-item-card:: Tutorials
    :link: tutorials
    :link-type: doc
    :class-card: mrpt-feature-card

    From your first program to SLAM, ICP, motion models and Kalman filters.

  .. grid-item-card:: Applications
    :link: applications
    :link-type: doc
    :class-card: mrpt-feature-card

    Dataset viewers, SLAM and localization, navigation simulators, converters.

  .. grid-item-card:: C++ API
    :link: modules
    :link-type: doc
    :class-card: mrpt-feature-card

    Reference documentation of all MRPT modules and classes.

  .. grid-item-card:: Python API
    :link: python_api
    :link-type: doc
    :class-card: mrpt-feature-card

    The ``mrpt3`` Python package (``pip install mrpt3``).

  .. grid-item-card:: Examples
    :link: examples
    :link-type: doc
    :class-card: mrpt-feature-card

    Over a hundred C++ examples and dozens of Python examples.

What's new in MRPT 3
====================

- **Modular packages**: each module (``mrpt_core``, ``mrpt_math``, ``mrpt_maps``...) is
  an independent CMake/colcon package, released into ROS 2 as ``ros-$ROS_DISTRO-mrpt-*``.
- **Python wheels** on PyPI: ``pip install mrpt3``.
- **New GUI apps** built on ImGui (RawLogViewer, SceneViewer3D, navlog-viewer,...).
- **Cleaner, modern C++ API**: ``mrpt::viz`` for 3D scenes, return values instead
  of output parameters, ``std::optional``, scoped enums, and fewer dependencies.

Porting existing code? Read :doc:`page_porting_mrpt3`.

Ecosystem and community
=======================

- Projects built on MRPT: `MOLA <https://github.com/MOLAorg/mola>`_ (SLAM and
  localization), `mp2p_icp <https://github.com/MOLAorg/mp2p_icp>`_,
  `MVSim <https://github.com/MRPT/mvsim>`_, ROS 2 packages and more in
  :ref:`children-projects`. See also :doc:`tutorial-mrpt-ros2` and :doc:`tutorial-mrpt-mola`.
- Questions and ideas: `GitHub Discussions <https://github.com/MRPT/mrpt/discussions>`_.
  Bug reports: `GitHub issues <https://github.com/MRPT/mrpt/issues>`_.
- :ref:`contributing` bug fixes and new features is welcome.
- Sample config files and datasets live in
  `modules/mrpt_data <https://github.com/MRPT/mrpt/tree/develop/modules/mrpt_data>`_,
  and more datasets are listed `online <https://www.mrpt.org/robotics_datasets>`_.
- The `ChangeLog <page_changelog.html>`_ for this version, and
  `performance benchmarks <https://performance.mrpt.org/>`_.
- Citing MRPT: see the
  `README <https://github.com/MRPT/mrpt#citing-mrpt>`_ (Zenodo DOI).

Indices and tables
==================

* :ref:`genindex`

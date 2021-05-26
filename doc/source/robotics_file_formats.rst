.. _robotics_file_formats:

########################
Robotics file formats
########################

MRPT defines a set of standard file formats with the intention of easing
the exchange of robotic raw datasets and, also, of already-built maps, between
different programs and C++ classes.

.. contents:: :local:

1. Datasets
------------

**Typical extension**: ``.rawlog``

Description:MRPT defines a binary format for robotic datasets, or rawlogs,
which can store the raw observations gathered by an arbitrary mix of sensors
for its posterior processing. The format is described in detail in the
Rawlog format page, among associated software tools and converters.

Related applications:

- :ref:`app_RawLogViewer`
- All offline SLAM programs.

2. Graph SLAM maps
--------------------

**Typical extension(s)**: ``.graph``, ``.graphbin``

Description: MRPT adopted the text format for 2D and 3D graphs (edges & nodes)
used by TORO, G2O, and others.

Related applications:

- :ref:`app_graph-slam`

3. “Simple maps”
------------------

**Typical extension(s)**: ``.simplemap``, ``.simplemap.gz``

Description: Sometimes called “view-based maps” in the literature: a set of
pairs of poses and their associated observations. Implemented in the type
mrpt::maps::CSimpleMap. Files can be transparently saved/loaded
with gzip-compression to save disk space.

Related applications:

- :ref:`app_robot-map-gui`
- :ref:`app_observations2map`

4. Occupancy grid maps
------------------------

**Typical extension(s)**: ``.gridmap``, ``.gridmap.gz``

Description: The serialization of an ``mrpt::maps::COccupancyGridMap2D`` object.

5. Scenes in 3D
-----------------

**Typical extension**: ``.3dscene``

Description: Files are usually transparently saved/loaded with
gzip-compression to save disk space.

Related applications:

- :ref:`doxid-app__scene_viewer3_d`.
- See: :ref:`doxid-tutorial_3_d_scenes`.

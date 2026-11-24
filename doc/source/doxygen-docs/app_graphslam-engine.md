\page app_graphslam-engine graphslam-engine


\note This application is **not built in MRPT 3.x yet**: it still needs
further porting, so it is not included in the MRPT 3 binaries. The underlying
library (mrpt::graphslam::CGraphSlamEngine) is still part of MRPT.

`graphslam-engine` runs graph-based SLAM on a rawlog dataset, using
mrpt::graphslam::CGraphSlamEngine. The building blocks are selected on the
command line:

- a node registration decider (`-n`), which decides when to add a new pose
  node to the graph, e.g. at fixed intervals or after enough displacement
  (mrpt::graphslam::deciders::CFixedIntervalsNRD,
  mrpt::graphslam::deciders::CICPCriteriaNRD);
- an edge registration decider (`-e`), which adds constraints between nodes,
  e.g. by ICP scan matching and loop closure detection
  (mrpt::graphslam::deciders::CICPCriteriaERD,
  mrpt::graphslam::deciders::CLoopCloserERD);
- a graph optimizer (`-o`), e.g. mrpt::graphslam::optimizers::CLevMarqGSO.

It builds 2D (`--2d`) or 3D (`--3d`) pose graphs from odometry, 2D laser
scans or RGB-D observations, with optional visualization of the graph and
comparison against a ground truth (`-g`). The options of each component are
given in an INI file (`-i`); sample files are in
`modules/mrpt_data/config_files/graphslam-engine/`, and sample datasets in
`modules/mrpt_data/datasets/graphslam-engine-demos/`.

A detailed guide of the design and usage of the application is in the MRPT
source tree, under `doc/graphslam-engine-guide/` (LaTeX sources; run `make`
there to build the PDF).

# See also

- [graph-slam](page_app_graph-slam.html): optimization and visualization of
  existing pose graphs.
- mrpt::graphslam::CGraphSlamEngine

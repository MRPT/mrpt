# MRPT Python Examples

Python examples for the MRPT 3.0 Python bindings.  Each script demonstrates
one MRPT module and doubles as documentation for the Python API.

## Running an example

```bash
# After building and installing the bindings (colcon build ...):
python3 mrpt_examples_py/mrpt_math_example.py
```

## Example index

| Script | Module(s) exercised | Topics |
|--------|---------------------|--------|
| `mrpt_bayes_example.py` | `mrpt.bayes` | CParticleFilter config, algorithm/resampling enums, TParticleFilterStats |
| `mrpt_comms_example.py` | `mrpt.comms` | CClientTCPSocket, CSerialPort, context managers |
| `mrpt_core_example.py` | `mrpt.core` | Clock, WorkerThreadsPool, abs_diff, reverse_bytes |
| `mrpt_config_example.py` | `mrpt.config` | CConfigFile, CConfigFileMemory, CLoadableOptions |
| `mrpt_containers_example.py` | `mrpt.containers` | YAML parse/query/serialize |
| `mrpt_expr_example.py` | `mrpt.expr` | CRuntimeCompiledExpression, formula evaluation |
| `mrpt_graphs_example.py` | `mrpt.graphs` | CNetworkOfPoses2D/3D, nodes, edges, save/load, Dijkstra shortest paths |
| `mrpt_gui_example.py` | `mrpt.gui`, `mrpt.viz` | CDisplayWindow3D, scene population, camera |
| `mrpt_hwdrivers_example.py` | `mrpt.hwdrivers` | Sensor drivers by class name + .ini config, CJoystick |
| `mrpt_img_example.py` | `mrpt.img` | CImage, TCamera, numpy integration |
| `mrpt_io_example.py` | `mrpt.io` | CMemoryStream, CFileInputStream/OutputStream, gz streams, zstd/gzip CCompressed streams, memory compression |
| `mrpt_kinematics_example.py` | `mrpt.kinematics` | CVehicleVelCmd, CVehicleSimul_DiffDriven/Holo |
| `mrpt_maps_example.py` | `mrpt.maps` | CSimplePointsMap, COccupancyGridMap2D, numpy, CMultiMetricMap from config, CVoxelMap, COctoMap |
| `mrpt_math_example.py` | `mrpt.math` | Points, Poses, Segments, Lines, Plane, BoundingBox, Twists, CPolygon, CHistogram, wrap2pi, matrices |
| `mrpt_obs_example.py` | `mrpt.obs` | CObservation2DRangeScan, IMU, Odometry, odometry motion models, CActionCollection, CSensoryFrame, CRawlog, CSimpleMap |
| `mrpt_poses_example.py` | `mrpt.poses` | CPose2D/3D, SE(2)/SE(3) composition, Lie algebra, particle PDFs |
| `mrpt_random_example.py` | `mrpt.random` | CRandomGenerator, uniform/Gaussian scalar and array draws, multivariate Gaussian |
| `mrpt_rtti_example.py` | `mrpt.rtti` | getAllRegisteredClasses |
| `mrpt_serialization_example.py` | `mrpt.serialization` | objectToBytes / bytesToObject round-trip |
| `mrpt_slam_example.py` | `mrpt.slam` | CICP alignment, CMetricMapBuilderICP incremental SLAM |
| `mrpt_system_example.py` | `mrpt.system` | CTicTac, CTimeLogger, CRC, base64 |
| `mrpt_tfest_example.py` | `mrpt.tfest` | SE(3) robust least-squares with RANSAC |
| `mrpt_topography_example.py` | `mrpt.topography` | TGeodeticCoords, WGS84↔ECEF↔ENU, UTM, reference ellipsoids |
| `mrpt_viz_example.py` | `mrpt.viz` | Scene, CAssimpModel, point cloud |
| `lines-3d-geometry-example.py` | `mrpt.math` | 3D geometry primitives |
| `matrices.py` | `mrpt.math` | Matrix / vector operations |
| `se2-poses-example.py` | `mrpt.poses` | SE(2) pose arithmetic |
| `se3-poses-example.py` | `mrpt.poses` | SE(3) pose arithmetic |
| `ros-poses-convert.py` | `mrpt.poses` | ROS ↔ MRPT pose conversion |
| `global_localization.py` | `mrpt.maps`, `mrpt.slam` | Monte Carlo localization on a dataset (`--no-gui` for headless runs) |
| `rbpf_slam.py` | `mrpt.slam` | Rao-Blackwellized particle filter SLAM on a dataset |
| `opengl-demo-gui.py` | `mrpt.gui`, `mrpt.viz` | OpenGL demo in a 3D window |
| `hwdriver-tao-imu-usb.py` | `mrpt.hwdrivers`, `mrpt.obs` | Reading a Taobotics USB IMU (needs the device) |

## Notes

- Scripts that open GUI windows (`mrpt_gui_example.py`, `mrpt_viz_example.py`,
  `opengl-demo-gui.py`) require a display and exit gracefully in headless
  environments.
- `global_localization.py` and `rbpf_slam.py` default to the demo datasets in
  `modules/mrpt_data` when run from the MRPT source tree.
- The `COLCON_IGNORE` file in this directory prevents colcon from trying to
  build these scripts as a colcon package — they are meant to be run directly.

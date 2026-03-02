# Pybind11 Wrapping Plan for MRPT 3.x

This document is a step-by-step plan for AI agents to extend and polish the
pybind11 Python bindings across all MRPT modules. Follow the conventions
described in `agents.md` §5.

## Legend

- **DONE** — Module already has active bindings (`mrpt_add_python_module` uncommented, `_py.cpp` and `__init__.py` exist).
- **NEW** — Module needs bindings created from scratch (create `python_bindings/` dir, `_py.cpp`, `python/mrpt/<name>/__init__.py`, uncomment or add CMake call).
- **EXTEND** — Module has bindings but important classes are missing.
- **SKIP** — Module is low-priority / internal / not useful from Python.

---

## Phase 0: Polish existing bindings (EXTEND)

These 12 modules already have active bindings. Extend them with missing classes
that are important for Python users.

### 0.1 mrpt_math (EXTEND)

**Currently wrapped:** `CMatrixDouble`, `CMatrixDoubleXX` (22/33/44/66/77), `CVectorFixedDouble{2,3,6}`, `CVectorDouble`, `TPoint2D`, `TPoint3D`, `TPoint2Df`, `TPoint3Df`, `TPose2D`, `TPose3D`.

**Add:**
- `TLine2D`, `TLine3D`, `TSegment2D`, `TSegment3D`, `TPlane`, `TObject2D`, `TObject3D` — geometry primitives used extensively in MRPT algorithms.
- `TBoundingBox` — bounding box queries.
- `TTwist2D`, `TTwist3D` — velocity representations.
- `TPose3DQuat` — quaternion pose.
- `CQuaternion<double>` (as `CQuaternionDouble`) — quaternion math.
- `CPolygon` — 2D polygon.
- `CHistogram` — histogram class.
- `CSplineInterpolator1D` — 1D spline.
- `wrap2pi()`, `angDistance()` free functions.
- `geometry.h` free functions: `intersect()`, `distance()`, `project3D()`, `project2D()`.
- `slerp()` free function.
- `ransac()` template — consider wrapping `RANSAC_Template` with Python callable for the fitting/distance functors.

**Skip:** `CSparseMatrixTemplate`, `MatrixBlockSparseCols`, `KDTreeCapable` (internal), `CLevenbergMarquardt` (use scipy instead), `data_utils.h` (internal).

### 0.2 mrpt_poses (EXTEND)

**Currently wrapped:** `CPose2D`, `CPose3D`, `CPosePDF`, `CPose3DPDF`, `CPose3DPDFGaussian`, `CPose3DPDFGaussianInf`, `SE_average<2>`, `SE_average<3>`.

**Add:**
- `CPose3DQuat` — quaternion-based 3D pose. Wrap constructors, conversion to/from `CPose3D`, `x/y/z/qr/qx/qy/qz` properties.
- `CPoint2D`, `CPoint3D` — serializable point classes (used in maps and observations).
- `CPosePDFGaussian` — 2D pose with Gaussian uncertainty.
- `CPosePDFGaussianInf` — 2D pose with information matrix.
- `CPose3DPDFParticles` — particle-based 3D pose PDF (used in PF-SLAM).
- `CPosePDFParticles` — particle-based 2D pose PDF (used in MCL).
- `CPose3DQuatPDFGaussian` — quaternion-based Gaussian PDF.
- `CPose2DInterpolator`, `CPose3DInterpolator` — trajectory interpolation (very useful from Python for playback/analysis).
- `CPointPDFGaussian` — Gaussian point uncertainty.
- `CPoseRandomSampler` — draw samples from pose PDFs.

**Skip:** `CPoses2DSequence`, `CPoses3DSequence` (legacy), `gtsam_wrappers.h` (C++ interop only), `Lie/` subdirectory (advanced, C++ template-heavy), `FrameTransformer` (ROS-specific).

### 0.3 mrpt_img (EXTEND)

**Currently wrapped:** `CImage`, `TColor`, `TCamera`, `DistortionModel`, `TPixelCoord`, `colormap`.

**Add:**
- `TStereoCamera` — stereo camera parameters.
- `TColorManager` — automatic color assignment (useful for visualization scripts).
- `camera_geometry.h` functions: `projectPoints_with_distortion()`, `undistort_points()`, `undistort_image()`.
- Ensure `CImage.loadFromFile()` / `CImage.saveToFile()` are wrapped.
- Add `CImage.from_numpy(array)` static factory (complement to `as_numpy()`).
- `CImage.getWidth()`, `getHeight()`, `isColor()`, `getChannelCount()`.

**Skip:** `CMappedImage` (rarely used), `CCanvas` (drawing base, already partially via CImage).

### 0.4 mrpt_viz (EXTEND)

**Currently wrapped:** `CVisualObject`, `CSetOfObjects`, `CCamera`, `CPointCloud`, `CPointCloudColoured`, `CAssimpModel`, `Scene`, `Viewport`.

**Add:**
- `CGridPlaneXY`, `CGridPlaneXZ` — ground plane grids (very common in demos).
- `CAxis` — coordinate axes visualization.
- `CBox`, `CSphere`, `CCylinder`, `CArrow` — basic geometric primitives.
- `CText`, `CText3D` — text labels.
- `CSetOfLines`, `CSimpleLine` — line drawing.
- `CEllipsoid2D`, `CEllipsoid3D` — uncertainty ellipsoids.
- `CFrustum` — camera frustum visualization.
- `CColorBar` — color scale bar.
- `stock_objects.h`: `CornerXYZ()`, `CornerXYZSimple()`, `RobotPioneer()`, `RobotRhodon()`.
- `CTexturedPlane`, `CMesh`, `CMeshFast` — surface/terrain rendering.
- `CSkyBox` — environment skybox.
- `CAnimatedAssimpModel` — animated 3D models.
- `TLightParameters` — scene lighting.
- Ensure `Scene::saveToFile()` / `Scene::loadFromFile()` are wrapped.

**Skip:** `CGeneralizedEllipsoidTemplate` (base template), `PLY_import_export.h` (use trimesh/open3d), `opengl_fonts.h` (internal), `graph_tools.h` (internal use).

### 0.5 mrpt_core (EXTEND)

**Currently wrapped:** `abs_diff`, `DEG2RAD`/`RAD2DEG`, `reverse_bytes`, `Stringifyable`, `Clock`, `WorkerThreadsPool`.

**Add:**
- `format()` — printf-style string formatting (though Python has f-strings, it's useful when interacting with MRPT APIs).
- `round()`, `sign()` from `bits_math.h`.
- `cpu_info()` from `cpu.h` — check for SIMD support.
- `backtrace_symbols()` — for debugging.
- `exception_to_str()` — convert MRPT exceptions to strings.

**Skip:** Most of `core/` is internal C++ infrastructure (aligned allocators, SSE macros, type traits, pimpl). Not useful from Python.

### 0.6 mrpt_system (EXTEND)

**Currently wrapped:** `CTicTac`, `CTimeLogger`, `CTimeLoggerEntry`, `CTimeLoggerSaveAtDtor`, CRC functions, base64, `unitsFormat`.

**Add:**
- `datetime.h` functions: `dateTimeToString()`, `timestampTotime_t()`, `now()`, `timeDifference()`, `dateTimeLocalToString()`.
- `filesystem.h` functions: `fileExists()`, `directoryExists()`, `fileNameChangeExtension()`, `extractFileDirectory()`, `extractFileName()`, `extractFileExtension()`, `getTempFileName()`, `createDirectory()`.
- `os.h`: `os::kbhit()`, `os::getch()` — interactive console input.
- `CDirectoryExplorer` — list files in directories.
- `CRateTimer`, `CControlledRateTimer` — rate-limited loops.
- `COutputLogger` — the MRPT logging system base class. Wrap `setLoggerName()`, `setMinLoggingLevel()`, `logStr()`.
- `progress.h`: `ProgressBar` — progress bar for loops.
- `CObserver` / `CObservable` / `mrptEvent` — event system (needed for GUI events).

**Skip:** `CGenericMemoryPool` (internal), `scheduler.h` (internal), `memory.h` (internal), `CConsoleRedirector` (niche).

### 0.7 mrpt_config (EXTEND)

**Currently wrapped:** `CConfigFileBase`, `CConfigFile`, `CConfigFileMemory`, `CLoadableOptions`, `config_parser`.

**Add:**
- `CConfigFilePrefixer` — useful for wrapping config sections.
- Ensure all `read_*()` methods are wrapped: `read_int`, `read_float`, `read_bool`, `read_string`, `read_vector`.
- `getAllSections()`, `getAllKeys()`.

### 0.8 mrpt_containers (EXTEND)

**Currently wrapped:** `yaml` (as `YAML`).

**Add:**
- Fix the `__getitem__` that is `#if 0`'d out — implement type-dispatching for string/int/float/bool return.
- Add `__contains__` (`has()`), `__delitem__`.
- Add `CDynamicGrid<T>` and `CDynamicGrid3D<T>` — 2D/3D grid containers (used by maps internally, but also useful standalone).
- `circular_buffer` — wrap as Python collection with `push_back`, `pop_front`, `__len__`, `__getitem__`.

**Skip:** Most container utilities (`bimap`, `map_as_vector`, `ts_hash_map`, etc.) have direct Python equivalents.

### 0.9 mrpt_serialization (EXTEND)

**Currently wrapped:** `CSerializable`, `CArchive`, `objectToBytes`, `bytesToObject`.

**Add:**
- `archiveFrom()` free function — create CArchive from file streams (needed to load/save rawlogs).
- Wrap `CArchive` reading/writing for `CObservation`, `CAction` types.
- File-based serialization helpers: `ObjectToFile()`, `FileToObject()` or equivalent lambdas.

### 0.10 mrpt_rtti (OK as-is)

Coverage is adequate: `CObject`, `TRuntimeClassId`, `classFactory`, registry functions.

### 0.11 mrpt_tfest (OK as-is)

Coverage is adequate: `TMatchingPair`, `TMatchingPairList`, `se2_l2`, `se3_l2_robust`.

### 0.12 mrpt_expr (OK as-is)

Coverage is adequate: `CRuntimeCompiledExpression` with Python callbacks.

---

## Phase 1: Critical new modules (NEW)

These modules contain essential robotics classes that Python users need most.

### 1.1 mrpt_obs (NEW — HIGH PRIORITY)

This module defines all sensor observation types. Without it, Python users cannot
work with MRPT datasets or sensor data.

**Files to create:**
- `modules/mrpt_obs/python_bindings/mrpt_obs_py.cpp`
- `modules/mrpt_obs/python/mrpt/obs/__init__.py`
- Uncomment `mrpt_add_python_module(obs ...)` in `modules/mrpt_obs/CMakeLists.txt`

**Classes to wrap:**

*Core:*
- `CObservation` — base class. Wrap `timestamp`, `sensorLabel`, `sensorPose`, `GetRuntimeClass()`.
- `CSensoryFrame` — collection of simultaneous observations. Wrap as iterable container.
- `CRawlog` — raw log file. Wrap `loadFromFile()`, iteration, size, `getAsObservation()`, `getAsAction()`.
- `CActionCollection` — collection of actions.
- `CAction` — base action class.
- `CActionRobotMovement2D` — odometry. Wrap `computeFromOdometry()`, `rawOdometryIncrementReading`, motion model params.
- `CActionRobotMovement3D` — 3D odometry.

*Range sensors:*
- `CObservation2DRangeScan` — 2D laser scan. Wrap `aperture`, `rightToLeft`, `maxRange`, `getScanRange()`, `setScanRange()`, `getScanRangeValidity()`, `scan` as numpy array, `validRange` as numpy array.
- `CObservation3DRangeScan` — RGB-D sensor. Wrap `hasRangeImage`, `rangeImage` (as numpy 2D float array), `hasIntensityImage`, `intensityImage` (as CImage), `hasPoints3D`, `unprojectInto()`.
- `CObservationRotatingScan` — rotating 3D lidar.
- `CObservationVelodyneScan` — Velodyne-specific.
- `CObservationRange` — generic range reading.

*Camera:*
- `CObservationImage` — single image observation. Wrap `image`, `cameraParams`.
- `CObservationStereoImages` — stereo pair.
- `CObservation3DScene` — 3D scene snapshot.

*Other sensors:*
- `CObservationIMU` — IMU data. Wrap `rawMeasurements`, timestamp.
- `CObservationGPS` — GPS. Wrap messages, timestamp, sensor pose.
- `CObservationOdometry` — odometry observation.
- `CObservationRobotPose` — external robot pose observation.
- `CObservationBearingRange` — bearing-range measurements.
- `CObservationBeaconRanges` — beacon range measurements.

*Map infrastructure (defined in mrpt_obs):*
- `CMetricMap` — abstract base for all maps. Wrap `isEmpty()`, `clear()`, `getAs3DObject()`, `saveMetricMapRepresentationToFile()`.
- `CSimpleMap` — map of poses + observations. Wrap iteration, `size()`, `get()`, `loadFromFile()`, `saveToFile()`.
- `TMetricMapInitializer` — map initializer from config.

*Projection params:*
- `T3DPointsProjectionParams` — params for `unprojectInto`.
- `T3DPointsTo2DScanParams` — params for range scan conversion.

**NumPy integration (critical):**
- `CObservation2DRangeScan.scan` → 1D float32 numpy array (zero-copy if possible)
- `CObservation3DRangeScan.rangeImage` → 2D float32 numpy array
- `CObservation3DRangeScan.intensityImage` → CImage (which has `as_numpy()`)
- `CObservationIMU.rawMeasurements` → numpy array

### 1.2 mrpt_maps (NEW — HIGH PRIORITY)

**Files to create:**
- `modules/mrpt_maps/python_bindings/mrpt_maps_py.cpp`
- `modules/mrpt_maps/python/mrpt/maps/__init__.py`
- Add `mrpt_add_python_module(maps ...)` to CMakeLists.txt

**Classes to wrap:**

*Point maps:*
- `CPointsMap` — abstract base. Wrap `size()`, `getPoint()`, `insertPoint()`, `setPoint()`, `getPointsBuffer()` (as Nx3 numpy), `loadFromFile()`, `saveToFile()`, `getAs3DObject()`, `clear()`, `kdTreeNClosestPoint2D/3D()`, `isEmpty()`.
- `CSimplePointsMap` — basic XYZ point cloud. Most common map type.
- `CGenericPointsMap` — XYZ + arbitrary fields. Wrap `registerField_float()`, `setPointField_float()`, `getPointField_float()`.

*Grid maps:*
- `COccupancyGridMap2D` — 2D occupancy grid. Wrap `setSize()`, `getResolution()`, `getSizeX/Y()`, `getCell()`, `setCell()`, `loadFromBitmapFile()`, `saveAsBitmapFile()`, `getAsMatrix()` (as numpy), `insertObservation()`, `computeObservationLikelihood()`, `laserScanSimulator()`.
- `COccupancyGridMap3D` — 3D occupancy grid.

*Voxel maps:*
- `CVoxelMap` — basic voxel map.
- `CVoxelMapRGB` — colored voxel map.

*Octomap:*
- `COctoMap` — OctoMap integration.
- `CColouredOctoMap` — colored OctoMap.

*Other:*
- `CMultiMetricMap` — container of heterogeneous maps. Wrap `maps` vector access, `insertObservation()`.
- `CLandmarksMap` — landmark-based map (from mrpt_slam, but defined in mrpt_maps).
- `CBeaconMap` — beacon map.
- `CHeightGridMap2D` — elevation map.

**NumPy integration:**
- `CPointsMap.getPointsBufferAsNumpy()` — return Nx3 float64 array (zero-copy via Eigen::Map).
- `COccupancyGridMap2D.getAsMatrix()` — return 2D float array.
- `CSimplePointsMap` constructor from Nx3 numpy array.

### 1.3 mrpt_gui (NEW — HIGH PRIORITY)

**Files to create:**
- `modules/mrpt_gui/python_bindings/mrpt_gui_py.cpp`
- `modules/mrpt_gui/python/mrpt/gui/__init__.py`
- Add `mrpt_add_python_module(gui ...)` to CMakeLists.txt

**Classes to wrap:**
- `CDisplayWindow3D` — the main 3D visualization window. Wrap `get3DSceneAndLock()`, `unlockAccess3DScene()`, `forceRepaint()`, `setCameraAzimuthDeg()`, `setCameraElevationDeg()`, `setCameraZoom()`, `setCameraPointingToPoint()`, `setFOV()`, `isOpen()`, `keyHit()`, `getPushedKey()`, `waitForKey()`, `getRenderingFPS()`, `addTextMessage()`, `captureImagesStart()`, `getLastWindowImage()`.
- `CDisplayWindow` — 2D image display. Wrap `showImage()`, `isOpen()`, `waitForKey()`.
- `CDisplayWindowPlots` — 2D plotting. Wrap `plot()`, `axis()`, `axis_equal()`, `clf()`, `hold_on()`, `hold_off()`.
- `CDisplayWindowGUI` — NanoGUI-based window. Wrap `background_scene`, `camera()`, `drawAll()`, `setVisible()`, `addLoopCallback()`.
- `CGlCanvasBase` — OpenGL canvas base (for camera manipulation API).
- `MRPT2NanoguiGLCanvas` — embeddable GL canvas.

**Important:** Provide a context manager for lock/unlock:
```python
with win.scene_lock() as scene:
    scene.insert(obj)
```
Implement via `__enter__`/`__exit__` in Python `__init__.py`.

**GIL consideration:** The 3D window runs its own rendering thread. When calling `get3DSceneAndLock()`, the GIL must be released during the lock to avoid deadlocks. Use `py::call_guard<py::gil_scoped_release>()` or wrap in lambda with `py::gil_scoped_release`.

### 1.4 mrpt_slam (NEW — HIGH PRIORITY)

**Files to create:**
- `modules/mrpt_slam/python_bindings/mrpt_slam_py.cpp`
- `modules/mrpt_slam/python/mrpt/slam/__init__.py`
- Add `mrpt_add_python_module(slam ...)` to CMakeLists.txt

**Classes to wrap:**
- `CICP` — Iterative Closest Point. Wrap `Align()`, `Align3D()`, `options` (nested struct), `TReturnInfo`.
- `CMetricMapBuilderICP` — ICP-based map builder. Wrap `initialize()`, `processActionObservation()`, `getCurrentlyBuiltMap()`, `getCurrentPoseEstimation()`, `options`.
- `CMetricMapBuilderRBPF` — Rao-Blackwellized PF SLAM. Wrap same interface as ICP builder.
- `CMonteCarloLocalization2D` — 2D MCL. Wrap `options`, `executeOn()`, `getParticlesCount()`, `getMostLikelyParticle()`, PDF access.
- `CMonteCarloLocalization3D` — 3D MCL.
- `CGridMapAligner` — map alignment. Wrap `Align()`, `options`.
- `CIncrementalMapPartitioner` — partition maps into submaps.
- `TKLDParams` — KLD-sampling parameters.
- `TMonteCarloLocalizationParams` — MCL parameters.
- `data_association.h` functions: `data_association_full_covariance()`, `data_association_independent_predictions()`.

### 1.5 mrpt_io (NEW — MEDIUM PRIORITY)

Needed for file I/O operations (loading rawlogs, etc.).

**Files to create:**
- `modules/mrpt_io/python_bindings/mrpt_io_py.cpp`
- `modules/mrpt_io/python/mrpt/io/__init__.py`
- Add `mrpt_add_python_module(io ...)` to CMakeLists.txt

**Classes to wrap:**
- `CFileGZInputStream` — gzip-compressed file input (used for rawlog loading).
- `CFileGZOutputStream` — gzip-compressed file output.
- `CFileInputStream`, `CFileOutputStream` — plain file streams.
- `CMemoryStream` — in-memory stream.
- `CStream` — base class (abstract, needed in hierarchy for CArchive).
- `zip.h` functions: `compress()`, `decompress()`.
- `vector_loadsave.h`: `loadBinaryFile()`, `vectorToBinaryFile()`.

### 1.6 mrpt_nav (NEW — MEDIUM PRIORITY)

**Files to create:**
- `modules/mrpt_nav/python_bindings/mrpt_nav_py.cpp`
- `modules/mrpt_nav/python/mrpt/nav/__init__.py`
- Add `mrpt_add_python_module(nav ...)` to CMakeLists.txt

**Classes to wrap:**
- `CReactiveNavigationSystem` — reactive navigator. Wrap `initialize()`, `navigate()`, `navigationStep()`, `options`.
- `CRobot2NavInterface` — base interface (Python subclassing needed for simulation).
- `CRobot2NavInterfaceForSimulator` — simulator interface.
- `CAbstractNavigator` — base navigator.
- `CWaypointsNavigator` — waypoint-following navigator.
- `TWaypoint`, `TWaypointSequence`, `TWaypointStatus` — waypoint types.
- `CParameterizedTrajectoryGenerator` — PTG base class.
- `CPTG_DiffDrive_C`, `CPTG_DiffDrive_alpha`, `CPTG_Holo_Blend` — common PTGs.
- `PlannerSimple2D` — A*-based 2D path planner.
- `CHolonomicVFF`, `CHolonomicND`, `CHolonomicFullEval` — holonomic methods.
- `CLogFileRecord` — navigation log.

---

## Phase 2: Supporting modules (NEW)

### 2.1 mrpt_graphs (NEW)

**Files to create:** standard pattern.

**Classes to wrap:**
- `CNetworkOfPoses<CPose2D>` (as `CNetworkOfPoses2D`), `CNetworkOfPoses<CPose3D>` (as `CNetworkOfPoses3D`) — pose graphs. Wrap `insertEdge()`, `nodeCount()`, `edgeCount()`, node/edge iteration, `saveToTextFile()`, `loadFromTextFile()`.
- `dijkstra_path()` — shortest path.
- `CAStarAlgorithm` — A* algorithm.
- `CDirectedGraph`, `CDirectedTree` — generic graph types.

### 2.2 mrpt_bayes (NEW)

**Files to create:** standard pattern.

**Classes to wrap:**
- `CParticleFilter` — the generic particle filter. Wrap `executeOn()`, `TParticleFilterOptions`, `TParticleFilterStats`.
- `CKalmanFilterCapable` — EKF/UKF base (template-heavy; consider wrapping specific instantiations or providing a Python-subclassable version).
- `CParticleFilterCapable` — base for particle filter implementations.

**Note:** The template-heavy nature of these classes makes direct wrapping complex. Consider wrapping only the non-template infrastructure and relying on the `mrpt_slam` MCL/RBPF classes for concrete particle filter use.

### 2.3 mrpt_kinematics (NEW)

**Files to create:** standard pattern.

**Classes to wrap:**
- `CVehicleSimul_DiffDriven` — differential-drive vehicle simulator. Wrap `movementCommand()`, `simulateOneTimeStep()`, `getCurrentGTPose()`, `getCurrentOdometricPose()`, `setCurrentGTPose()`.
- `CVehicleSimul_Holo` — holonomic vehicle simulator.
- `CVehicleVelCmd_DiffDriven` — velocity command for diff-drive.
- `CVehicleVelCmd_Holo` — velocity command for holonomic.
- `CKinematicChain` — robotic arm chain.

### 2.4 mrpt_random (NEW)

**Files to create:** standard pattern.

**Classes to wrap:**
- `CRandomGenerator` — wrap the singleton `mrpt::random::getRandomGenerator()`.
- Wrap: `drawUniform()`, `drawGaussian1D()`, `drawGaussian1D_normalized()`, `drawUniformVector()`, `drawGaussianMultivariate()`.
- `randomize()` — seed the generator.

**Note:** Python has `numpy.random`, but wrapping this ensures MRPT C++ code and Python code share the same RNG state, which matters for reproducibility.

### 2.5 mrpt_topography (NEW)

**Files to create:** standard pattern.

**Functions to wrap from `conversions.h`:**
- `geodeticToENU_WGS84()` — GPS lat/lon/alt to local ENU.
- `ENUToGeodetic()` — local ENU to GPS.
- `geocentricToGeodetic()`, `geodeticToGeocentric()`.
- `UTMToGeodetic()`, `geodeticToUTM()`.

**Data types from `data_types.h`:**
- `TGeodeticCoords` — lat/lon/alt.
- `TEllipsoid` — reference ellipsoid.
- `TCoords` — DMS coordinates.

### 2.6 mrpt_vision (NEW)

**Files to create:** standard pattern.

**Classes to wrap:**
- `CFeatureExtraction` — feature detector/descriptor. Wrap `detectFeatures()`, `computeDescriptors()`, `options`.
- `TKeyPoint`, `TKeyPointList` — keypoint types.
- `CUndistortMap` — undistortion map (precomputed, faster than per-frame).
- `CStereoRectifyMap` — stereo rectification.
- `CImagePyramid` — image pyramid.
- `chessboard_find_corners.h`: `findChessboardCorners()`.
- `chessboard_camera_calib.h`: `checkerBoardCameraCalibration()`.

**Skip:** `CDifodo` (niche), `pnp_algos.h` (removed in v3), `CVideoFileWriter` (use OpenCV/ffmpeg from Python).

### 2.7 mrpt_hwdrivers (NEW — LOW PRIORITY)

Hardware drivers are less commonly used from Python (most users use ROS drivers),
but wrapping the generic sensor interface is useful.

**Classes to wrap:**
- `CGenericSensor` — base class. Wrap `loadConfig()`, `initialize()`, `doProcess()`, `getObservations()`.
- `CCameraSensor` — generic camera. Wrap same interface + `getNextFrame()`.
- `CHokuyoURG` — Hokuyo laser.
- `CVelodyneScanner` — Velodyne lidar.
- `CGPSInterface` — GPS receiver.
- `CKinect` — Kinect sensor.
- `CFFMPEG_InputStream` — video file reader.

**Skip:** Most specific sensor drivers (SICK, Phidget, NI-DAQ, etc.) — too hardware-specific for Python use.

### 2.8 mrpt_comms (NEW — LOW PRIORITY)

**Classes to wrap:**
- `CClientTCPSocket` — TCP client.
- `CServerTCPSocket` — TCP server.
- `CSerialPort` — serial port.
- `net_utils.h`: `http_get()`, `http_request()`.
- `nodelets.h`: `Topic`, `Subscriber` — in-process pub/sub.

**Skip:** `CInterfaceFTDI` (hardware-specific).

### 2.9 mrpt_graphslam (NEW — LOW PRIORITY)

**Classes to wrap:**
- `levmarq()` — graph-SLAM optimization. Wrap the main `graphslam::optimize_graph_spa_levmarq()` function.
- Skip the complex `CGraphSlamEngine` and its many template-heavy components — they are designed for C++ apps.

### 2.10 mrpt_libapps_cli / mrpt_libapps_gui (NEW)

Application-level code (rawlog-edit, SLAM apps).

**Classes to wrap:**
- RawlogEditApp
- RawlogGrabberApp
- MonteCarloLocalization_App

---

## Phase 3: Skip or defer

### 3.1 mrpt_opengl (SKIP)

This module contains low-level OpenGL rendering infrastructure (shaders, buffers,
textures, framebuffers). Not useful from Python — users interact with `mrpt_viz`
and `mrpt_gui` instead.

### 3.2 mrpt_typemeta (SKIP)

Compile-time C++ metaprogramming utilities (`TTypeName`, `TEnumType`, `static_string`).
No runtime value for Python.

### 3.3 mrpt_imgui (SKIP for now)

ImGui integration. Could be useful in future but depends on `mrpt_gui` bindings first.


---

## Phase 4: Python examples and testing

For each newly wrapped module, create or update a Python example in `mrpt_examples_py/`:

| Module | Example file | Content |
|--------|-------------|---------|
| `obs` | `mrpt_obs_example.py` | Load a rawlog, iterate observations, print types/timestamps, access scan data as numpy |
| `maps` | `mrpt_maps_example.py` | Create point cloud from numpy, build occupancy grid, insert scans, save as image |
| `gui` | `mrpt_gui_example.py` | Open 3D window, insert objects, animate, handle keyboard |
| `slam` | `mrpt_slam_example.py` | Run ICP on two point clouds, run MCL with a gridmap |
| `nav` | `mrpt_nav_example.py` | Configure reactive navigator, run navigation loop with simulated robot |
| `graphs` | `mrpt_graphs_example.py` | Build pose graph, optimize, visualize |
| `io` | `mrpt_io_example.py` | Load/save rawlog, compress/decompress data |
| `random` | `mrpt_random_example.py` | Seed RNG, draw samples, compare with numpy |
| `topography` | `mrpt_topography_example.py` | Convert GPS coords to ENU and back |
| `vision` | `mrpt_vision_example.py` | Detect chessboard corners, calibrate camera |
| `kinematics` | `mrpt_kinematics_example.py` | Simulate diff-drive robot |
| `bayes` | `mrpt_bayes_example.py` | Run particle filter (via slam MCL) |
| `hwdrivers` | `mrpt_hwdrivers_example.py` | Open camera, grab frames |
| `comms` | `mrpt_comms_example.py` | TCP client/server echo |

Also update existing examples to use newly wrapped classes where they improve the demo.

---

## Implementation order (recommended)

1. **Phase 0** — Polish existing bindings (low effort, immediate value)
2. **Phase 1.1** `mrpt_obs` — Unlocks dataset loading
3. **Phase 1.5** `mrpt_io` — Needed for file-based rawlog loading
4. **Phase 1.2** `mrpt_maps` — Unlocks map creation and manipulation
5. **Phase 1.3** `mrpt_gui` — Unlocks visualization
6. **Phase 1.4** `mrpt_slam` — Unlocks SLAM algorithms
7. **Phase 2.4** `mrpt_random` — Quick to implement
8. **Phase 2.5** `mrpt_topography` — Quick to implement (free functions)
9. **Phase 2.1** `mrpt_graphs` — Needed for graph-SLAM
10. **Phase 2.3** `mrpt_kinematics` — Needed for navigation simulation
11. **Phase 1.6** `mrpt_nav` — Navigation
12. **Phase 2.2** `mrpt_bayes` — Particle filter infrastructure
13. **Phase 2.6** `mrpt_vision` — Computer vision
14. **Phase 2.7-2.9** — Low priority modules
15. **Phase 4** — Examples and testing throughout

---

## Notes for agents

- Always build with `colcon build --packages-up-to mrpt_<module>` and test with `python3 -c "import mrpt.<module>"` after changes.
- Run existing Python examples to check for regressions.
- When wrapping a class that inherits from `CSerializable`, include it in the pybind11 class template: `py::class_<T, CSerializable, std::shared_ptr<T>>`.
- When wrapping abstract base classes, declare them without `py::init<>()` — they cannot be instantiated from Python but are needed for the inheritance hierarchy.
- When a C++ method has output parameters, convert to return values (single value or `py::make_tuple()`).
- Prefer zero-copy numpy access where possible (Eigen::Map, py::array_t with base object).
- Test with both `import mrpt` (namespace auto-import) and `from mrpt.<module> import ClassName` (direct import).

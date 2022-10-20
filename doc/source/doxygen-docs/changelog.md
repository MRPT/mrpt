\page changelog Change Log

# Version 2.5.6: UNRELEASED
- Changes in libraries:
  - \ref mrpt_system_grp
    - These functions are now thread-safe if built in a system with the `localtime_r()` variant of `localtime()`:
      - mrpt::system::timestampToParts()
      - mrpt::system::dateTimeLocalToString()
      - mrpt::system::timeLocalToString()

# Version 2.5.5: Released October 19th, 2022
- Changes in applications:
  - prg-configurator:
    - A maximum trajectory time can be specified now for rendering PTGs.
    - New CLI arguments `--ini`, `--ini-section` to automate loading custom INI files.
- Changes in libraries:
  - \ref mrpt_containers_grp
    - mrpt::container::yaml:
      - Clearer error messages when an invalid type conversion is requested.
      - It now does not throw internal exceptions when trying to convert strings to bool.
  - \ref mrpt_imgs_grp
      - mrpt::img::CImage::filledRectangle() is now implemented using the fast opencv draw function instead of the slow mrpt::img::CCanvas default base implementation.
  - \ref mrpt_math_grp
      - Correct copyright notes for embedded version of the CSparse sources (PR [#1255](https://github.com/MRPT/mrpt/pull/1255)).
  - \ref mrpt_typemeta_grp
      - mrpt::typemeta::TEnumType<> on invalid names, it now prints all valid known enum names in its exception error message.

# Version 2.5.4: Released September 24th, 2022
- Changes in libraries:
  - \ref mrpt_opengl_grp
    - mrpt::opengl::CFBORender is now faster, using a LUT for converting from logarithmic to linear depth values.
  - \ref mrpt_ros1bridge_grp
    - Implemented missing mrpt::ros1bridge::toROS() for point clouds.
  - \ref mrpt_ros2bridge_grp
    - Implemented missing mrpt::ros2bridge::toROS() for point clouds.
- BUG FIXES:
  - Fix build on hppa for parisc architecture too (not supported flag `-mtune=native`)
  - nanogui: Fix mismatched memory allocator/free in serialization code.
  - Fix potential segfault in RawLogViewer while building the tree view.

# Version 2.5.3: Released September 6th, 2022
- Changes in libraries:
  - \ref mrpt_gui_grp
    - nanogui::mainloop() (and mrpt::gui::CDisplayWindowGUI()) now allows defining a minimum period for calls to user callback functions via a new second optional parameter.
  - \ref mrpt_obs_grp
    - Not all `CObservation*` classes were declared in `<mrpt/obs/obs_frwds.h>`. Now it is corrected.
- BUG FIXES:
  - Fix build on hppa (parisc64) architecture (not supported flag `-mtune=native`)

# Version 2.5.2: Released August 30th, 2022
- BUG FIXES:
  - mrpt::math::MatrixBase::eig_symmetric() now is ensured not to return negative eigenvalues due to numerical innacuracies in some platforms (i386).
  - mrpt::maps::COccupancyGridMap2D::getAsPointCloud() did not return the outermost cells as obstacles.
  - Fix unit test errors when compiling with LTO (Closes [Debian bug #1015550](https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=1015550))

# Version 2.5.1: Released August 4th, 2022
- Changes in applications:
  - RawLogViewer:
    - Bold points when selecting a sensor in the timeline UI.
- Changes in libraries:
  - Embedded nanoflann version upgraded to v1.4.3
- BUG FIXES:
  - Fix regression: mrpt::opengl::CAssimpModel may fail to resolve texture relative paths.
  - mrpt::opengl::CMesh was implementing texture coordinates flipping (u,v) wrt documented behavior.
  - Fix `static_assert()` failure in CTicTac when building with latest emscripten clang compiler.
  - Fix regression: yaml parsing boolean values may throw an exception inside `std::stoi()`.

# Version 2.5.0: Released July 18th, 2022
- Changes in applications:
  - RawLogViewer:
    - New time-line UI for quickly navigating and selecting observations.
  - New application:
    - ros-map-yaml2mrpt: CLI tool to import ROS map_server maps into MRPT formats.
- Changes in libraries:
  - \ref mrpt_containers_grp
    - New functions mrpt::containers::find_closest() and mrpt::containers::find_closest_with_tolerance().
    - mrpt::containers::yaml now also keeps information about line and column positions for each token, see mrpt::containers::yaml::node_t::marks
  - \ref mrpt_core_grp
    - mrpt::Clock::toDouble() now returns 0 for default-constructed (invalid) time_point.
  - \ref mrpt_opengl_grp
    - mrpt::opengl::CMesh supports having (x,y) limits with `maxCoord<minCoord` for flipped elevation and image meshes.
    - New flag mrpt::opengl::CAssimpModel::LoadFlags::IgnoreMaterialColor for mrpt::opengl::CAssimpModel::loadScene()
    - A new rendering mode for default no-perspective transformations.
      See mrpt::opengl::CCamera::setNoProjection()
  - \ref mrpt_poses_grp
    - Add correct displacement covariance calculation between two poses with cross-correlation via new method mrpt::poses::CPose3DQuatPDFGaussian::inverseCompositionCrossCorrelation() (Closes [#1242](https://github.com/MRPT/mrpt/issues/1242))
  - \ref mrpt_system_grp
    - New funtions mrpt::system::toAbsolutePath(), mrpt::system::pathJoin()
    - Most functions in \ref filesystem ported to C++17 std::filesystem
  - \ref mrpt_tfest_grp
    - New method TMatchingPairList::overallSquareError() for SE(3) poses (CPose3D).
- Deprecations:
    - The following macros, which were already deprecated, have been removed:  `ASSERT_BELOW_`, `ASSERT_ABOVE_()`, `ASSERT_BELOWEQ_()`, `ASSERT_ABOVEEQ_()`
- Build system:
  - Update fallback embedded version of octomap to v1.9.6
- BUG FIXES:
  - FIX: OpenGL API errors if several CWxGLCanvasBase instances are updated simultaneously in the same program.
  - mrpt::opengl::COpenGLViewport would throw if an uninitialized image is passed for rendering in "image mode".
  - mrpt::system::formatTimeInterval() reported an incorrect number of milliseconds.
  - Fix detection of Boost python module.
  - Calling mrpt::opengl::CRenderizable::setColor_u8() did not force a regeneration of opengl buffer objects in all cases.

# Version 2.4.10: Relased June 24th, 2022
- Changes in applications:
  - ptg-configurator:
    - New menu action to export selected path to matlab/octave script.
  - RawLogViewer:
    - Visual improvements and display of timestamps in local time too.
- Changes in libraries:
  - \ref mrpt_obs_grp
    - New set of functions to help visualize observations: \ref customizable_obs_viz_grp
  - \ref mrpt_poses_grp
    - Adds covariance mapping to SE(3) for GTSAM (Closes [#1229](https://github.com/MRPT/mrpt/issues/1229))
  - \ref mrpt_ros1bridge_grp
    - Import mrptToROSLoggerCallback() from the now obsolete mrpt_bridge package into mrpt::ros1bridge.
- Build system
  - Fix ROS version detection; select ROS2 if packages for both versions are found.
- BUG FIXES:
  - Fix mrpt-comms rare timeout in busy build farms.
  - mrpt::ros1bridge and mrpt::ros2bridge were not correctly exporting the `fromROS()` function for LaserScan messages.

# Version 2.4.9: Released June 7th, 2022
- Changes in libraries
  - \ref mrpt_math_grp
    - new method mrpt::math::TPlane::signedDistance()
  - \ref mrpt_ros2bridge_grp
    - Fixed missing `find_package()` in module config.cmake file.
- BUG FIXES:
  - Fix wrong handling of cmake exported built-in version of Eigen3 (Closes [#1235](https://github.com/MRPT/mrpt/issues/1235))
  - Fix pymrpt Python3 module location (Closes [#1232](https://github.com/MRPT/mrpt/issues/1232))

# Version 2.4.8: Released May 26th, 2022
- Build system:
  - Fixed various ROS-level public dependencies.

# Version 2.4.7: Released May 26th, 2022
- Examples:
  - gui_depth_camera_distortion: Added option to change distortion model.
- Build system
  - Fixed ROS-level public dependency on suitesparse.
  - Enable tinyxml2 for ROS builds.
- Changes in libraries:
  - \ref mrpt_maps_grp
    - Method mrpt::maps::CPointsMap::addFrom() removed, it overlapped with mrpt::maps::CPointsMap::insertAnotherMap()
    - New optional parameter in mrpt::maps::CPointsMap::insertAnotherMap()
  - \ref mrpt_obs_grp
    - New option mrpt::obs::T3DPointsProjectionParams::onlyPointsWithIntensityColor

# Version 2.4.6: Released May 24th, 2022
- Build system
  - Fixed ROS-level public dependencies via package.xml

# Version 2.4.5: Released May 22nd, 2022
- Changes in libraries:
  - New module \ref mrpt_ros2bridge_grp to support conversions to/from ROS2 data types and MRPT classes.
- Build system:
  - Fix detection of dependencies for both ROS1 and ROS2.

# Version 2.4.4: Released March 5th, 2022
- New web-based applications
  - All MRPT modules (including \ref mrpt_opengl_grp and mrpt-nanogui) are now compatible with Emscripten so they can run as Javascript + wasm on any modern browser.
- Changes in applications:
  - RawlogViewer:
    - Browse dialog: Smarter coloring of pointclouds; check all & none buttons for sensor layers.
  - rawlog-edit:
    - Operation `--camera-params` now also works for mrpt::obs::CObservation3DRangeScan observations.
    - New operation `--describe`.
- Changes in libraries:
  - \ref mrpt_poses_grp
    - mrpt::poses::CPose3DQuat: Remove use of obsolete base class std::iterator.
- 3rdparty libraries:
  - Updated libfyaml to v0.7.12.
- Build system:
  - Allow using libfyaml-dev system package if found.
  - ROS package.xml: update dependencies so all sensors and mrpt-ros1bridge are enabled.
  - Fix detection of ROS1 native `*_msgs` packages as build dependencies.
  - If ROS environment variables are detected at CMake configure time, unit tests are disabled by default (to reduce build time in build farms).
- BUG FIXES:
  - ASSERT_NEAR_() did not work correctly when arguments were expressions with operators.
  - Fixed incorrect parsing of strings with whitespaces in mrpt::from_string<>() when converting to std::string
  - mrpt::obs::CObservation3DRangeScan::get_unproj_lut() was ignoring the depth camera distortion model and always assumed plumb_bob.
  - mrpt::ros1bridge converter for IMU observations now correctly handles missing IMU readings (ROS convention of "-1" in covariance).

# Version 2.4.3: Released Feb 22nd, 2022
- Changes in applications:
  - navlog-viewer:
    - The timestamp is now always shown.
- BUG FIXES:
  - Do not run offscreen rendering unit tests in MIPS arch, since they seem to fail in autobuilders.
  - mrpt::vision::checkerBoardCameraCalibration() did not return the distortion model (so if parameters are printed, it would look like no distortion at all!).
  - mrpt::gui::CDisplayWindowGUI::createManagedSubWindow() created the subwindows helper UI on top of the other user windows. It now remains on the back of other windows.

# Version 2.4.2: Released Feb 3rd, 2022
- Changes in libraries:
  - \ref mrpt_containers_grp
    - mrpt::container::yaml::operator(size_t) added, conditionally to `size_t` being a different type than `uint64_t` and such (Fixes build errors on OSX).
  - \ref mrpt_core_grp
    - mrpt::callStackBackTrace() (and exception backtraces) now only use BFD to solve for line numbers in DEBUG builds, to avoid the large delay in processing each exception.
    - New method mrpt::WorkerThreadsPool::size().
  - \ref mrpt_expr_grp
    - ExprTk updated to latest version.
  - \ref mrpt_gui_grp
    - GUI windows can now have custom icons via mrpt::gui::CDisplayWindowGUI::setIcon() or mrpt::gui::CDisplayWindowGUI::setIconFromData()
  - \ref mrpt_img_grp
    - New static method mrpt::img::CImage::LoadFromFile()
  - \ref mrpt_math_grp
    - Vector and matrix classes: add [[nodiscard]] to static "constructor" methods to avoid mistakes.
  - \ref mrpt_opengl_grp
    - mrpt::opengl::CFBORender now does not rely on GLUT to create opengl contexts, but on EGL.
  - \ref mrpt_typemeta_grp
    - Add syntactic sugar function mrpt::typemeta::str2enum<>().
- BUG FIXES:
  - mrpt::opengl::CFBORender did only render the `main` viewport, it now processes all of them.
  - Fix FTBFS with ffmpeg 5.0 (Debian Bug #[1004585](https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=1004585))

# Version 2.4.1: Released Jan 5th, 2022
- Changes in build system:
    - Disable -flto in nanogui (to avoid an [Eigen regression](https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=1000780)).
- Changes in applications:
  - rawlog-edit:
    - New flag `--externals-filename-format`
  - RawlogViewer:
    - Better handling of exceptions failing to load delayed-load images.
- Changes in libraries:
  - \ref mrpt_core_grp
    - Remove unused header `<mrpt/3rdparty/llvm/propagate_const.h>`.
  - \ref mrpt_graphs_grp
    - mrpt::graphs::CDijkstra now has an optional maximum topological search range.
  - \ref mrpt_math_grp
    - New geometry functions:
      - mrpt::math::intersect(const TPolygon2D& subject, const TPolygon2D& clipping)
      - mrpt::math::signedArea(const mrpt::math::TPolygon2D& p)
  - \ref mrpt_obs_grp
    - New function mrpt::obs::format_externals_filename()
  - Embedded copy of nanoflann: upgraded to v1.4.0.
- BUG FIXES:
  - Fix bug in mrpt::math::getAngle(const TPlane&, const TPlane&).
  - Fix exception if mrpt::opengl::CFBORender is used with setProjectiveFromPinhole() camera models.
  - Fix CMake Warning at cmakemodules/FindFilesystem.cmake and failure to detect the std::filesystem feature in some g++ versions.
  - Fix numerical innacuracies with planar bounding boxes, fixed via new `epsilon` parameter in mrpt::math::TBoundingBox::intersection()
  - Fix sluggish rendering in opengl+wxWidgets controls (e.g. within RawLogViewer, etc.).

# Version 2.4.0: Released Dec 12th, 2021
- Changes in build system:
  - Most important CMake variables now are prefixed with `MRPT_` to avoid name collisions if using MRPT as a git submodule in a larger project.
  - `GNUInstallDirs` directories are now always honored when installing.
- Changes in acpplications:
  - ptg-configurator:
    - Show selected PTG path output motion command.
  - navlog-viewer:
    - New checkbox to enforce 2D orthogonal view, which is now the default view.
  - rawlog-edit
    - The `--info` command now also shows the first and last timestamp in a rawlog.
  - RawLogViewer:
    - Show mrpt::obs::CObservationPointCloud 3D point clouds in main window and scan animation dialog.
    - Displays timestamp as the user tracks the timeline scroll bar.
  - rosbag2rawlog:
    - PointCloud2 messages are now only converted to mrpt::obs::CObservationRotatingScan is this latter class is specified in the YAML file.
- Changes in libraries:
  - \ref mrpt_apps_grp
    - Application rawlog-edit is now available as the C++ class mrpt::apps::RawlogEditApp
  - \ref mrpt_containers_grp
    - New methods mrpt::containers::bimap::erase_by_key(),mrpt::containers::bimap::erase_by_value()
    - mrpt::containers::vector_with_small_size_optimization has new methods `at()` and `push_back()` for a smoother transition from STL containers.
    - mrpt::containers::yaml and libfyaml updated to latest version (more memory efficient parser).
  - \ref mrpt_core_grp
    - New base virtual interface class mrpt::Stringifyable unifying the asString() method already offered by many MRPT classes.
  - \ref mrpt_img_grp
    - **[API change]** mrpt::img::TCamera methods changed to allow defining fish-eye camera models too.
  - \ref mrpt_io_grp
    - GZIP compressed streams now also support open and append. See new mrpt::io::CFileGZOutputStream::open() signature.
    - New enum mrpt::io::OpenMode for clearer-to-read code.
    - Moved lazy-load operations to mrpt::io::setLazyLoadPathBase() and companion functions, since the older names mentioned images but this setting actually affects other sensors too.
  - \ref mrpt_math_grp
    - New function mrpt::math::xcorr()
    - New header `<mrpt/math/gtsam_wrappers.h>`, see \ref mrpt_gtsam_wrappers
    - New method mrpt::math::TBoundingBox::containsPoint()
  - \ref mrpt_maps_grp
    - Optimization: mrpt::maps::CPointsMap::insertAnotherMap() avoids matrix multiplication if SE(3) identity is passed as insertion pose.
    - **[API change]** mrpt::maps::CSimpleMap docs improved, API modernized and made const-correct including returned shared_ptr instances as ConstPtr where applicable.
  - \ref mrpt_nav_grp
    - mrpt::nav::CParameterizedTrajectoryGenerator::initTPObstacleSingle() now always initializes to the maximum free distance, instead of saturating free space when heading to a target waypoint.
    - **[API change]** mrpt::nav::CParameterizedTrajectoryGenerator::getPathPose() had two overloaded signatures, which is not recommended being one of them a virtual method. Only the return-by-value is left.
  - \ref mrpt_obs_grp
    - Fix const-correctness of mrpt::obs::CObservation::unload() for consistency with load().
    - **[API change]** Replaced all API signatures taking an optional mrpt::poses::CPose3D as pointers (with default=nullptr) with a modern `std::optional<>`.
  - \ref mrpt_opengl_grp
    - New method mrpt::opengl::COpenGLViewport::setClonedCameraFrom()
    - mrpt::opengl::CFBORender changes:
      - More consistent naming of API methods: mrpt::opengl::CFBORender::render_RGB().
      - New method to render into a depth image mrpt::opengl::CFBORender::render_RGBD().
    - mrpt::opengl::CCamera::setProjectiveFromPinhole() now allows defining a camera by means of a pinhole model.
    - New class mrpt::opengl::COpenGLFramebuffer, used to refactor mrpt::opengl::CFBORender
    - New methods to control face culling:
      - mrpt::opengl::CRenderizableShaderTriangles::cullFaces()
      - mrpt::opengl::CRenderizableShaderTexturedTriangles::cullFaces()
    - Remove specular light effects in the default shaders, to fix buggy behavior.
    - **[API change]** New mrpt::opengl::Visualizable interface replaces former getAs3DObject() in all mrpt::maps and mrpt::poses classes with an uniform API, avoiding shared_ptr if possible.
    - mrpt::opengl::CTexturedPlane now more efficiently renders as plain triangles if no texture has been assigned.
    - Custom user OpenGL shaders can now be defined and installed to replace MRPT defaults.
    Refer to example: \ref opengl_custom_shaders_demo
  - \ref mrpt_poses_grp
    - New function mrpt::poses::sensor_poses_from_yaml()
    - New header `<mrpt/poses/gtsam_wrappers.h>`, see \ref mrpt_gtsam_wrappers
  - \ref mrpt_random_grp
    - New function mrpt::random::partial_shuffle()
    - New function mrpt::random::portable_uniform_distribution()
  - \ref mrpt_serialization_grp
    - Implemented serialization of mrpt::containers::bimap in the new header `#include <mrpt/serialization/bimap_serialization.h>`.
    - Enums can now be binary-serialized too via `>>` / `<<` streaming operators into an mrpt::serialization::CArchive.
    - mrpt::serialization::CArchive and mrpt::io::CStreams now have virtual methods to provide human-friendly self-descriptions, useful to debug which stream causes an error in serialization.
  - \ref mrpt_system_grp
    - Backwards-compatible change: New function mrpt::system::InvalidTimeStamp() used now inside the macro INVALID_TIMESTAMP, so the macro always returns a const reference instead of returning by value.
    - New function mrpt::system::consoleColorAndStyle()
    - mrpt::system::intervalFormat() now generates more human-friendly strings for time periods larger than 1 second (e.g. "1 year, 3 days, 8 hours").
  - \ref mrpt_tfest_grp
    - **[API change]** mrpt::tfest::TMatchingPair members are now called "local" vs "global" instead of the former, more confusing, "this" vs "other".
  - \ref mrpt_vision_grp
    - SIFT descriptors can now be evaluated for arbitrary keypoint coordinates.
- BUG FIXES:
  - Fix potential race conditions in:
    - mrpt::rtti class registry
    - The global mrpt::random::getRandomGenerator()
    - mrpt::typemeta::TEnumTypeFiller
  - Image-mode was not serialized in mrpt::opengl::COpenGLViewport
  - nanogui: avoid potential divide by zero.
  - mrpt::comms::CClientTCPSocket crashed if socket handle >=1024 in Linux (Closes [#1157](https://github.com/MRPT/mrpt/issues/1157))
  - Fix error generating and parsing TUM RGBD dataset rawlog files.
  - Fix regresion in mrpt::opengl::CFBORender::render() throwing an exception if the input image was empty.
  - Fix incorrect handling of negative, fractional viewport sizes in mrpt::opengl::COpenGLViewport
  - Fix: Should not scale velocity commands when in slow down, in CAbstractPTGBasedReactive::generate_vel_cmd() (Closes [#1175](https://github.com/MRPT/mrpt/issues/1175)).
  - mrpt::system::CDirectoryExplorer did not fill in correct absolute paths if a relative path was passed as starting directory to scan.
  - Fix mrpt::obs::CSensoryFrame::operator+=() did not perform what it was supposed to do.

# Version 2.3.2: Released Jul 14, 2021
- Changes in applications:
  - RawLogViewer:
    - More tree view icons.
    - "Play video" window now also shows timestamps.
  - SceneViewer3D:
    - New command-line flag `--imgdir` to define the base path for lazy-load images.
  - rawlog-edit:
    - New operation `--export-txt` exploiting the new export-to-txt API in mrpt::obs::CObservation
  - navlog-viewer:
    - New UI tools to manually pick and export selected PTG selections to a training YAML file.
- Changes in libraries:
  - \ref mrpt_containers_grp
    - YAML macros `MCP_LOAD_OPT()`, `MCP_LOAD_REQ()`, and `MCP_SAVE()` now also support reading and writing enums directly as YAML, transparently converting numerical values to/from their symbolic names.
  - \ref mrpt_core_grp
    - Added C++14 helper templates mrpt::uint_select_by_bytecount_t and mrpt::int_select_by_bytecount_t
  - \ref mrpt_gui_grp
    - mrpt::gui::CDisplayWindowGUI: improved API to allow multiple callback handlers, and to report exceptions in them.
    - New 3D navigation key binding: SHIFT+scroll wheel, for fast up/down pure vertical motion of the camera point.
  - \ref mrpt_img_grp
    - mrpt::img::CImage::loadFromFile() now avoids memory allocations if there was already an image in memory with the same size.
  - \ref mrpt_obs_grp
    - mrpt::obs::CObservation now has a common API to export datasets to TXT/CSV files, see methods exportTxtSupported(), exportTxtHeader(), exportTxtDataRow(). It has been implemented in all suitable observation classes.
    - mrpt::obs::CObservationImage::unload() defaulted to doing nothing. It now correctly unloads lazy-load images.
  - \ref mrpt_poses_grp
    - New methods mrpt::math::TTwist2D::rotated() and mrpt::math::TTwist3D::rotated()
  - \ref mrpt_system_grp
    - mrpt::system::CTimeLogger:
      - Include custom `name` in underlying mrpt::system::COutputLogger name.
      - Fix all valgrind/helgrind warning messages.
    - New functions mrpt::system::firstNLines() and mrpt::system::nthOccurrence()
- BUG FIXES:
  - mrpt::img::CImage::isEmpty() should return false for delay-load images.
  - Fix build error with GCC 8 in `mrpt/containers/yaml.h`.
  - Fix exception rendering empty point clouds due to invalid bounding box.
  - Fix broken 2D plots rendering in Ubuntu 20.04 (and probably other systems), via an update in mpWindow to properly use wxAutoBufferedPaintDC.
  - mrpt::img::CImage::getPixelDepth() should force loading lazy load images.
  - Fixed wrong rendering of different textures within the same opengl shader program.
  - Fixed potential crashes inside BFD if using BFD and calling mrpt::callStackBackTrace() from several parallel threads.

# Version 2.3.1: Released May 26th, 2021
- General cmake scripts:
  - `find_package(mrpt-xxx)` is now much faster.
- Changes in applications:
  - RawLogViewer:
    - Browse scans window now has a check-box list to show/hide individual sensors.
  - SceneViewer3D:
    - Graceful failure when loading a corrupted 3Dscene file.
- Changes in libraries:
  - \ref mrpt_core_grp
    - Removed mrpt::reverseBytesInPlace(long double) for it not being portable.
  - \ref mrpt_containers_grp
    - New environment variable MRPT_YAML_PARSER_VERBOSE controlling mrpt::containers::yaml
  - \ref mrpt_hwdrivers_grp
    - New argument to pass custom ffmpeg options to mrpt::hwdrivers::CFFMPEG_InputStream::openURL(). New default is to prefer stream over TCP for more reliable IP cameras reading.
    - mrpt::hwdrivers::CHokuyoURG now has a parameter for between-data communications timeout (`comms_between_timeout_ms`).
  - \ref mrpt_gui_grp
    - mrpt::gui::CDisplayWindowGUI new methods to minimize/restore subwindows.
  - \ref mrpt_math_grp
    - New method mrpt::math::TLine3D::closestPointTo()
    - New methods mrpt::math::TPose3D::translation(), mrpt::math::TPose2D::translation().
  - \ref mrpt_obs_grp
    - New mrpt::obs::CActionCollection::insert() overload for smart pointers.
    - New method mrpt::obs::CObservation2DRangeScan::getScanAngle() and clarify docs on class members.
    - New class mrpt::obs::CObservation3DScene.
    - mrpt::obs::CObservationIMU now uses std::array instead of std::vector (faster due to less dynamic memory).
  - \ref mrpt_opengl_grp
    - Deprecate mrpt::opengl::COpenGLScene::dumpListOfObjects() in favor of new mrpt::opengl::COpenGLScene::asYAML()
    - New method mrpt::opengl::CSimpleLine::setLineCoords() accepting mrpt::math::TPoint3D (older signature deprecated).
  - \ref mrpt_system_grp
    - New return-by-value signature for mrpt::system::CDirectoryExplorer::explore(), older version deprecated.
    - mrpt::system::extractFileDirectory() returns `"."` instead of an empty string for filenames without any explicit full path.
- BUG FIXES:
  - Fix wrong formatting of empty *string* values (not *null* values) in mrpt::containers::yaml.
  - Fix exception loading old datasets with stereo observations, via a new argument in mrpt::img::CImage::makeSureImageIsLoaded()
  - Fix unhandled deserialization of v2 of mrpt::opengl::CPlanarLaserScan
  - Fix build errors with MinGW.

# Version 2.3.0: Released April 25th, 2021
- General build changes:
  - CMake >=3.8.0 is now required to ensure proper handling of dependencies compile options.
- Changes in applications:
  - ptg-configurator: target now also comprises a heading angle.
  - RawLogViewer:
    - New tab with CObservation3DRangeScan visualization options.
    - All icons have been updated for a more modern look.
- Changes in libraries:
  - \ref mrpt_containers_grp
    - add method mrpt::containers::map_as_vector::at()
  - \ref mrpt_graphs_grp
    - mrpt::graphs::CDijkstra:
      - now no longer requires a field `nodes` in input graphs.
      - add convenient return by value getTreeGraph()
      - Deprecate mrpt::graphs::CDijkstra::Visitor virtual class API in favor of new C++11 std::function-based mrpt::graphs::CDijkstra::visitor_t
  - \ref mrpt_math_grp
    - Removed redundant mrpt::math::pointIntoPolygon2D()  -> mrpt::math::TPolygon2D::contains()
    - Removed redundant mrpt::math::SegmentsIntersection() ->  mrpt::math::intersect(mrpt::math::TSegment2D,mrpt::math::TSegment2D)
    - Removed redundant mrpt::math::distancePointToPolygon2D() -> TPolygon2D::distance()
    - Moved mrpt::math::minDistBetweenLines() -> mrpt::math::TLine3D::distance()
  - \ref mrpt_opengl_grp
    - mrpt::opengl::CAssimpModel now uses a texture cache to speed up and reduce RAM usage if loading the same textures in different objects.
  - \ref mrpt_system_grp
    - New function mrpt::system::progress()
- BUG FIXES:
  - ptg-configurator: Fix failure to list existing PTGs, due to RTTI unregistered name "CParameterizedTrajectoryGenerator".
  - mrpt::opengl::COpenGLViewport::get3DRayForPixelCoord() returned wrong pixel coordinates when in orthogonal projection mode.
  - mrpt::opengl::CArrow: Fix wrong normal calculation (wrong rendering reflections).
  - mrpt::opengl::CPointCloud::markAllPointsAsNew() and mrpt::opengl::CPointCloudColoured::markAllPointsAsNew() did not refresh OpenGL buffers.
  - mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathTwist() returned much larger velocities than the actual values.
  - Fix broken Debian dependencies for libmrpt-vision-lgpl (Closes [Debian bug #986071](https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=986071)).
  - `mrpt::maps::CPointsMap::load2Dor3D_from_text_stream()` for 2D maps left uninitialized values in `z`. Fixed to load zeros instead.
  - Fixed crash in mrpt::vision::checkerBoardCameraCalibration() causing segfault in the camera-calib app.

------
# Version 2.2.0: Released March 10th, 2021
- Changes in libraries:
  - \ref mrpt_vision_grp
    - Remove all obsolete `SIFTOptions.implementation` values. `OpenCV` is now the only possibility.
  - \ref mrpt_nav_grp
    - mrpt::nav::TWaypoint now uses std::optional instead of magic numbers in some fields.
    - mrpt::nav::TWaypoint now has std::any fields to hold user-given extra data.
- BUG FIXES:
  - Fix invalid bounding box returned by octree_getBoundingBox() and mrpt::opengl point cloud classes when empty (Closes [#1145](https://github.com/MRPT/mrpt/issues/1145)).
  - Fix potential infinite recursion in exceptions with stack trace (Closes [#1141](https://github.com/MRPT/mrpt/issues/1141)).
  - Fix potential race conditions accessing waypoint lists in mrpt::nav::CWaypointsNavigator
  - Fix build errors with gcc-11.

------
# Version 2.1.8: Released Feb 23rd, 2021
- Changes in applications:
  - RawLogViewer:
    - "Scan animation" window: now also shows the timestamp of observations.
  - camera-calib and kinect-stereo-calib:
    - New option to save camera calibration results as YAML files.
  - navlog-viewer:
    - New option to enable orthogonal view.
- General build changes:
  - Fix excessive alignment in aarch64 (32->16 bytes).
  - clang-format: enforce and upgraded to use clang-format-10.
  - Fix building against the non-legacy GL library (Linux).
  - nanoflann source code is no longer included as a copy: it will be used as the system library libnanoflann-dev, or as a git submodule if the former is not found.
- Changes in libraries:
  - \ref mrpt_containers_grp
    - New YAML to/from matrix methods: mrpt::containers::yaml::FromMatrix(), mrpt::containers::yaml::toMatrix()
  - \ref mrpt_core_grp
    - New CMake build flags `MRPT_EXCEPTIONS_WITH_CALL_STACK` to optionally disable reporting call stacks upon exceptions and `MRPT_EXCEPTIONS_CALL_STACK_MAX_DEPTH` to set their maximum depth.
  - \ref mrpt_hwdrivers_grp
    - mrpt::hwdrivers::CHokuyoURG now has a parameter for communications timeout (`comms_timeout_ms`).
  - \ref mrpt_math_grp
    - New class mrpt::math::TBoundingBox
  - \ref mrpt_maps_grp
      - Const correctness fixed in all mrpt::maps::CMetricMap classes.
  - \ref mrpt_opengl_grp
    - mrpt::opengl::CFrustum() new constructor from mrpt::img::TCamera()
  - \ref mrpt_poses_grp
    - mrpt::poses::CPose3D: Add more syntactic sugger static constructors.
  - \ref mrpt_slam_grp
    - mrpt::slam::TMonteCarloLocalizationParams map parameters are now shared pointers instead of plain pointers for safer code.
- BUG FIXES:
  - Log `*_THROTTLE_*` macros (e.g. MRPT_LOG_THROTTLE_DEBUG) did not report the message the first time they were called, which seems a safer behavior.
  - Reverted changed behavior: mrpt::config::CConfigFile did not throw if a non-existing file was passed to its constructor, but it throws in MRPT 2.1.{0-7}.
  - Fix build against opencv 2.4.x (version in Ubuntu Xenial 16.04).
  - Fixed: CHokuyoURG::initialize() won't report sensor status as ssError if it fails to communicate with the sensor, incorrectly leaving it as ssInitializing instead.
  - Fixed: mrpt::opengl::CTexturedPlane::setPlaneCorners() did not check for incorrect null width or height.
  - Fixed: mrpt::opengl textured objects leaking memory (Closes [#1136](https://github.com/MRPT/mrpt/issues/1136)).
  - Fix bug in parsing CARMEN logs: mrpt::obs::carmen_log_parse_line() returned all scan ranges marked as "invalid".

------
# Version 2.1.7: Released Jan 2nd, 2021
- BUG FIXES:
  - Fix bash syntax error in PPA release scripts.
  - Fix [Debian bug #978209](https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=978209): FTBFS: mainwindow.h:218:2: error: reference to Tracker is ambiguous

------
# Version 2.1.6: Released Dec 14th, 2020
- Changes in libraries:
  - \ref mrpt_core_grp
    - Disable the use of BFD for symbols in stack traces by default in Debian builds. It is still used if found in the system and in Ubuntu PPAs.
- BUG FIXES:
  - Fix [Debian bug #976803](https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=976803): mrpt uses private binutils shared library.
  - Fix [Debian bug #977247](https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=977247]): fail to link pymrpt against opencv.

------
# Version 2.1.5: Released Dec 6th, 2020
- Changes in libraries:
  - \ref mrpt_containers_grp
    - Both mrpt::containers::CDynamicGrid and mrpt::containers::CDynamicGrid3D are now compatible with range-based for loops, and also have a data() method.
  - \ref mrpt_core_grp
    - Added mrpt::LockHelper::unlock()
    - Added mrpt::Clock::nowDouble()
    - New method mrpt::WorkerThreadsPool::name()
    - Function mrpt::system::callStackBackTrace() moved to mrpt::callStackBackTrace()
    - mrpt::callStackBackTrace() now uses BFD to find out line numbers if debug info (at least -g1) is available.
    - Stacked exceptions changes:
      - Line numbers will be now shown if built with debug info (>= -g1).
      - Exceptions in STL or any other 3rd-party library will be also reported with exact call point line number, as long as MRPT_START/MRPT_END is used in the user function.
      - No further need to call mrpt::exception_to_str(), just calling what() will return a detailed stack backtrace.
      - New function mrpt::winerror2str()
  - \ref mrpt_gui_grp
    - New method mrpt::gui::CGlCanvasBase::CamaraParams::FromCamera()
  - \ref mrpt_math_grp
    - Added missing method for consistent API across pose classes: mrpt::math::TPose3D::operator+()
  - \ref mrpt_system_grp
    - mrpt::system::COutputLogger::writeLogToFile() will now save *all* messages despite the runtime log verbosity level.
- BUG FIXES:
  - Fix error rendering an opengl scene with mrpt::opengl::CCamera objects in it.
  - rawlog-edit silently ignored when more than one operation was requested.
  - Fix FTBFS against libjsoncpp 1.9.4 (Closes [#1118](https://github.com/MRPT/mrpt/issues/1118))
  - Fix AppStream errors and warnings in Debian Tracker.

------
# Version 2.1.4: Released Nov 8th, 2020
- Changes in libraries:
  - \ref mrpt_core_grp
    - mrpt::format() is no longer a template but a function, to use GCC automated printf-format warnings.
  - \ref mrpt_containers_grp
    - mrpt::containers::yaml avoids throwing internal exceptions as part of regular valid conversions, and better support and report of out-of-range integers.
  - \ref mrpt_math_grp
    - mrpt::math::linspace() added overload returning by value.
  - \ref mrpt_random_grp
    - mrpt::random::CRandomGenerator::permuteVector() added overload returning by value.
  - \ref mrpt_tfest_grp
    - mrpt::tfest::TMatchingPairListTempl<T>::saveAsMATLABScript() now draws 3D correspondences too.
    - RANSAC method mrpt::tfest::se3_l2_ransac() now uses more correct SO(3) metric for angular distance threshold instead of independent yaw/pitch/roll angles.
- BUG FIXES:
  - Fix wrong Debian dependencies of libmrpt-dev

------
# Version 2.1.3: Released Oct 21st, 2020
- Changes in libraries:
  - \ref mrpt_config_grp
    - Refactor parsing functionality as new exposed method mrpt::config::config_parser()
  - \ref mrpt_gui_grp
    - mrpt::gui::CDisplayWindowGUI subwindows control UI automatically keep tracks of focused subwindows.
- Build system:
  - Support for ccache, enabled by default if found.
  - Fix build with system libsimpleini-dev.
- BUG FIXES:
  - Fix mrpt::opengl::CFBORender requiring images with origin at the bottom-left corner.

------
# Version 2.1.2: Released Oct 20th, 2020
- BUG FIXES:
  - Fix wrong coloring of graph edges in mrpt::opengl::graph_visualize() (Closes [#1111](https://github.com/MRPT/mrpt/issues/1111)).
  - Fix Debian Lintian error: exporting copyrighted sources as part of simpleini submodule.

------
# Version 2.1.1: Released Oct 19th, 2020
- Changes in applications:
  - SceneViewer3D:
    - Command-line argument is now interpreted as ASSIMP model to open if it is not a 3Dscene.
    - New menu: "File -> Import -> Show image" useful to test image-mode viewport rendering.
- Changes in libraries:
  - \ref mrpt_core_grp
    - mrpt::Clock now has a simulated time mode. See mrpt::Clock::setSimulatedTime()
  - \ref mrpt_gui_grp
    - Useless nanogui_win() converted into mrpt::gui::CDisplayWindowGUI::nanogui_screen()
    - nanogui: New methods: nanogui::Screen::mouseState(), nanogui::Screen::mouseModifiers()
    - Managed subwindows with minimize/restore capability. See mrpt::gui::CDisplayWindowGUI::createManagedSubWindow()
  - \ref mrpt_img_grp
    - New method mrpt::img::CImage::channelCount()
  - \ref mrpt_opengl_grp
    - New load flags in mrpt::opengl::CAssimpModel::loadScene()
- BUG FIXES:
  - navlog-viewer: Crash when clicking "play" (Closes [#1103](https://github.com/MRPT/mrpt/issues/1103)).
  - RawLogViewer: Fix wrong indices in tree view. Fix freezed progress bar loading a second rawlog.
  - RawLogViewer: Fix wrong rendering if font is missing in the system (Ubuntu 20.04)
  - rawlog-edit: Fix --cut operation leaving empty sensory frames/action collections.
  - mrpt::opengl::CCylinder::setHasBases() was ignored since last OpenGL3 refactor.
  - Fix building against OpenCV 4.4
  - Correct texture loading in mrpt::opengl::CAssimpModel.
  - Fix wrong aspect ratio of image-mode opengl viewports (Closes [#1101](https://github.com/MRPT/mrpt/issues/1101)).

------
# Version 2.1.0: Released Aug 31st, 2020
- Incompatible API changes:
  - mrpt::system::TParameters has been removed, superseded by mrpt::containers::yaml.
  - Remove mrpt::hwdrivers::CRovio
  - Removed old mrpt 1.5.x backwards-compatible `<mrpt/utils/...>` headers (Closes #1083).
- Changes in libraries:
  - \ref mrpt_containers_grp
    - New class mrpt::containers::yaml for nested, YAML-like data structures.
  - \ref mrpt_core_grp
    - New mrpt::for_<> constexpr for loop helper function.
    - New function mrpt::demangle()
    - New class mrpt::WorkerThreadsPool
    - New macro ASSERT_NEAR_(). Defined new macros with correct English names ASSERT_LT_(), etc. deprecating the former ones.
    - mrpt::get_env() gets specialization for bool.
  - \ref mrpt_math_grp
    - New static methods with semantic-rich names: mrpt::math::TPlane::From3Points(), mrpt::math::TPlane::FromPointAndLine(), ...
    - New asString() methods in mrpt::math::TPlane, mrpt::math::TLine2D, mrpt::math::TLine3D
  - \ref mrpt_tfest_grp
    - New templatized mrpt::tfest::TMatchingPairTempl<> and mrpt::tfest::TMatchingPairListTempl<>
    - New mrpt::tfest::se3_l2() for `double` precision.
- Build:
    - yamlcpp is no longer a build dependency.
    - Less RAM and time required to build debug builds or to load in the debugger.
- BUG FIXES:
  - Avoid crash in camera-calib app when clicking "Close" while capturing a live video.
  - Fix potential Eigen crash in matrixes inverse() and inverse_LLt() if building mrpt and user code with different optimization flags.
  - Wrong parsing of env variables in mrpt::get_env() when called more than once.
  - mrpt::system::CTimeLogger: Fix wrong formatting (parent entry prefix collapse) in summary stats table.
  - mrpt::opengl::CEllipsoid2D was not RTTI registered.
  - Fix wrong copy of internal parameters while copying mrpt::maps::CMultiMetricMap objects.

------
# Version 2.0.4: Released Jun 20, 2020
- Changes in applications:
  - rawlog-edit, rawlog-grabber: Now allows loading external "plugin" modules (.so) with user-defined types.
  - RawLogViewer, navlog-viewer, ptg-configurator allows more than one "plugin" modules to be loaded.
- Changes in libraries:
  - \ref mrpt_math_grp
    - New semantically-rich named static methods:
      - mrpt::math::TLine3D::FromPointAndDirector()
      - mrpt::math::TLine3D::FromTwoPoints()
      - mrpt::math::TLine2D::FromCoefficientsABC()
      - mrpt::math::TLine2D::FromTwoPoints()
  - \ref mrpt_obs_grp
    - CObservation3DRangeScan::points3D_convertToExternalStorage() stores point clouds with points as rows (vs as columns as it did before).
  - \ref mrpt_opengl_grp
    - Emit warnings to std::cerr whenever opengl memory is leaked due to OpenGL buffers being created and destroyed in different threads.
    - Overlaid text messages are now also (de)serialized in mrpt::opengl::COpenGLViewport, and hence in 3D scenes in general.
    - All opengl shader base classes now expose their internal buffers as const ref. See children of mrpt::opengl::CRenderizable
  - \ref mrpt_system_grp
    - New class: mrpt::system::CControlledRateTimer (+ associated example)
    - New functions: mrpt::system::loadPluginModule(), mrpt::system::loadPluginModules()
    - mrpt::system::CRateTimer: enforce use of high-resolution monothonic clock.
    - mrpt::system::CTicTac: enforce use of nanosecond monothonic clock.
    - Misplaced functions moved to their proper namespace: mrpt::io::vectorToTextFile()
    - New functions: mrpt::system::thread_name() to get and set thread names for debuggers.
    - mrpt::system::setConsoleColor(): Do not change color if stdout/stderr are not real terminals.
  - \ref mrpt_nav_grp
    - mrpt::nav::PlannerSimple2D does not throw an exception if goal/source is out of map bounds.
- BUG FIXES:
    - mrpt::obs::CObservation3DRangeScan would try to (incorrectly) "autofix" camera resolution if loading an externally-stored observation.
    - mrpt::maps::CPointsMap::determineMatching2D(): avoid potential multi-thread problems with a vector::swap()
    - Fix build against opencv <3.4.4
    - Fix potential pointer to local returned in CParticleFilterData
    - Fix: mrpt::maps::CPointsMapXYZI::setFromPCLPointCloudXYZI() was using a non-existing method.
    - Fix: mrpt::nav::PlannerSimple2D did not honored maximum path length correctly.
    - Fix race condition in CGenericCamera_AVI unit test.

------
# Version 2.0.3: Released May 13, 2020
- Changes in applications:
  - navlog-viewer: Can now navigate with keyboard arrows too.
  - RawLogViewer: better 3D pointcloud coloring in observation view and in "scan animation" view.
- Changes in libraries:
  - \ref mrpt_maps_grp
    - Point cloud classes mrpt::maps::CPointsMap: New methods:
      - load2D_from_text_stream()
      - load3D_from_text_stream()
      - save2D_to_text_stream()
      - save3D_to_text_stream()
  - \ref mrpt_poses_grp
      - More accurate analytical Jacobians for CPose3DQuatPDFGaussian::inverse()  (Closes #1053)
  - BUG FIXES:
    - Incorrect number of points loaded when trying to load point clouds from incorrectly-formatted text files.
    - Fix build error in riscv64 (gcc doesn't know mtune=native for that arch)
    - Fix spurious unit test failures in mrpt::apps::RawlogGrabberApp due to system load.

------
# Version 2.0.2: Released May 4th, 2020
- Changes in applications:
  - navlog-viewer: Ported to the new nanogui UI system (fixes random OpenGL context errors in former version).
- Changes in libraries:
  - mrpt_containers_grp
    - mrpt::containers::vector_with_small_size_optimization: Get rid of potential uninitialized usage GCC warnings.
  - mrtp_hwdrivers_grp
    - Remove support for obsolete XSens MTi 3rd generation devices. Removed class mrpt::hwdrivers::CIMUXSens. 4th+ generation still supported.
  - mrpt_gui_grp
    - Fix mouse-motion rotation glitches if clicking inside a nanogui control.
    - Fix cmake errors building user programs in Windows (missing glfw dependency).
  - mrpt_math_grp
      - mrpt::math::RANSAC_Template made more generic to support custom dataset and model types.
  - mrpt_opengl_grp
    - Fix displaying of uninitialized textured in mrpt::opengl::CTexturedPlane. It now uses the default solid color of the object.
  - mrpt_ros1bridge_grp
    - Narrower build and run time dependencies: rosbag -> rosbag_storage

------
# Version 2.0.1: Released April 3rd, 2020
- Changes in applications:
  - RawLogViewer: new "-l xxx.so" flag to load datasets with types defined in external projects.
- Changes in libraries:
  - mrpt_obs_grp
    - mrpt::obs::CObservationRobotPose: Fixed missing serialization of sensorPose

------
# Version 2.0.0: Released March 29th, 2020
- **Most important changes:**
  - MRPT now requires **C++17** to build and use. See this page for a guide to port existing code to MRPT 2.0: \ref porting_mrpt2
  - Support for old namespaces `mrpt-scanmatching`, `mrpt-reactivenav` is over.
  - Backwards compatible headers for "maps" and "observations" in mrpt::slam are removed. They moved to their own namespaces in MRPT v1.3.0 (Jan 2015).
  - All pointer typedefs are now in their respective classes: FooPtr -> Foo::Ptr
  - Add support for serialization with std::variant
  - PbMap has been factored out into [its own repository](https://github.com/MRPT/pbmap)
  - XML-based database C++ classes have been removed from MRPT.
- Changes in applications:
  - RawLogViewer:
    - The ICP module now supports Velodyne 3D scans.
  - rawlog-edit:
    - New operation: `--de-externalize`
  - pf-localization:
    - Odometry is now used also for observation-only rawlogs.
- Changes in libraries:
  - All `otherlibs` subdirectories have been renamed to `3rdparty` since it is a widespread name used in most projects.
  - \ref mrpt_base_grp => Refactored into several smaller libraries, one per namespace.
    - Removed class std::vector<std::string>. Replace by STL containers of `std::string` and functions mrpt::system::stringListAsString() in \ref string_manage.
  - \ref mrpt_core_grp  [NEW IN MRPT 2.0.0]
    - Memory alignment of aligned_allocator_cpp11<> is set to 16,32 or 64 depending on whether AVX optimizations are enabled, to be compatible with Eigen.
    - mrpt::cpu::supports(): a new cross-OS CPU feature detection function.
    - mrpt::Clock allows users to select between Realtime or Monotonic sources.
    - Removed custom macro MRPT_UNUSED_PARAM (replaced by c++17 attribute).
    - Add syntactic suggar mrpt::lockHelper()
  - \ref mrpt_math_grp  [NEW IN MRPT 2.0.0]
    - Removed functions (replaced by C++11/14 standard library):
      - mrpt::math::erf, mrpt::math::erfc, std::isfinite, mrpt::math::std::isnan
      - `mrpt::math::make_vector<>` => `std::vector<>{...}` braced initializator
    - Removed the include file: `<mrpt/math/jacobians.h>`. Replace by `<mrpt/math/num_jacobian.h>` or individual methods in \ref mrpt_poses_grp classes.
  - \ref mrpt_config_grp  [NEW IN MRPT 2.0.0]
    - mrpt::config::CConfigFileBase::write() now supports enum types.
  - \ref mrpt_gui_grp
    - New class mrpt::gui::CDisplayWindowGUI exposing powerful GUI possibilities via the nanogui project.
  - \ref mrpt_img_grp  [NEW IN MRPT 2.0.0]
    - mrpt::img::TCamera distortion parameters now also supports the extra K4,K5,K6 distortion parameters.
  - \ref mrpt_serialization_grp  [NEW IN MRPT 2.0.0]
    - New method mrpt::serialization::CArchive::ReadPOD() and macro `MRPT_READ_POD()` for reading unaligned POD variables.-
    - Add support for `$env{}` syntax to evaluate environment variables.
  - \ref mrpt_slam_grp
    - rbpf-slam: Add support for simplemap continuation.
    - CICP: parameter `onlyClosestCorrespondences` deleted (always true now).
    - mrpt::slam::CICP API: Simplified and modernized to use only one output parameter, using std::optional.
  - \ref mrpt_system_grp
    - functions to get timestamp as *local* time were removed, since they don't make sense. All timestamps in MRPT are UTC, and they can be formated as dates in either UTC or local time frames.
    - Added: mrpt::system::WorkerThreadsPool
  - \ref mrpt_rtti_grp  [NEW IN MRPT 2.0.0]
    - All classes are now registered (and de/serialized) with their full name including namespaces. A backwards-compatible flag has been added to mrpt::rtti::findRegisteredClass().
    - CLASS_INIT() macro for automatic registration of classes has been removed, since it is not well-defined in which order global objects will be initialized.
      Therefore, manual registration (as already done in registerAllClasses.cpp files) is left as the unique registration system.
      This fixes warning messages "[mrpt::rtti::registerClass] Warning: Invoked with a nullptr".
  - \ref mrpt_nav_grp
    - Removed deprecated mrpt::nav::THolonomicMethod.
    - mrpt::nav::CAbstractNavigator: callbacks in mrpt::nav::CRobot2NavInterface are now invoked *after* `navigationStep()` to avoid problems if user code invokes the navigator API to change its state.
    - Added methods to load/save mrpt::nav::TWaypointSequence to configuration files.
    - Waypoints now have a field `speed_ratio` which is directly forwarded to the low-level reactive navigator.
  - \ref mrpt_comms_grp [NEW IN MRPT 2.0.0]
    - This new module has been created to hold all serial devices & networking classes, with minimal dependencies.
  - \ref mrpt_maps_grp
    - mrpt::maps::CMultiMetricMap has been greatly simplified and now it is actually defined in the mrpt-maps library.
    - New map type: mrpt::maps::CPointsMapXYZI for pointclouds with an intensity channel.
    - New observation class: mrpt::obs::CObservationPointCloud
    - Added optional "channel" attribute to CReflectivityGridMap2D and CObservationReflectivity to support different colors of light.
  - \ref mrpt_hwdrivers_grp
    - COpenNI2Generic: is safer in multithreading apps.
    - CHokuyoURG:
      - Rewrite driver to be safer and reduce mem allocs.
      - New parameter `scan_interval` to decimate scans.
    - VelodyneCalibration: Can now load YAML files, in addition to XML.
    - New sensor state enum value: mrpt::hwdrivers::CGenericSensor::ssUninitialized
    - NMEA GPS parser: now also recognizes all existing talker IDs (GP, GN, GA, etc.)
  - \ref mrpt_opengl_grp
    - Update Assimp lib version 4.0.1 -> 4.1.0 (when built as ExternalProject)
    - Rendering engine rewritten to work using OpenGL Core (GLSL 3.3) instead of Legacy fixed functions.
  - \ref mrpt_obs_grp
    - mrpt::obs::CObservation2DRangeScan: Deprecated access to scan data via proxy objects `obs->scan[i]`, `obs->validRange[i]`, `obs->intensity[i]` has been deleted. Please use the alternative getters/setters: `obs->getScanRange(i)`, etc.
    - mrpt::obs::T3DPointsProjectionParams and mrpt::obs::CObservation3DRangeScan::unprojectInto now together support organized PCL point clouds.
    - New method: mrpt::obs::CObservation3DRangeScan::rangeImage_getAsImage()
    - Support for multiple-return sensors in mrpt::obs::CObservation3DRangeScan.
    - New NMEA frame class: Message_NMEA_GSA
  - \ref mrpt_poses_grp  [NEW IN MRPT 2.0.0]
    - Reorganized all Lie Algebra methods into \ref mrpt_poses_lie_grp
    - Removed CPose3DRotVec, since its conceptual design is identical to Lie tangent space vectors.
  - \ref mrpt_vision_grp
    - Removed FASTER methods, and the libCVD 3rd party dependency.

- BUG FIXES:
  - Fix reactive navigator inconsistent state if navigation API is called from within rnav callbacks.
  - Fix incorrect evaluation of "ASSERT" formulas in mrpt::nav::CMultiObjectiveMotionOptimizerBase
  - Fix aborting reading from LMS111 scanner on first error.
  - Fix == operator on CPose3D: it now uses an epsilon for comparing the rotation matrices.
  - Fix accessing unaligned POD variables deserializing CObservationGPS (via the new `MRPT_READ_POD()` macro).
  - Fix segfault in CMetricMap::loadFromSimpleMap() if the provided CMetricMap has empty smart pointers.
  - Fix crash in CGPSInterface when not setting an external mutex.
  - Fix potential crashes in RawLogViewer while editing list of observations.
  - Fix incorrect conversion from quaternion to CPose3D.

<a name="1.5.7">
<h2>Version 1.5.7: Released 24/APR/2019  </h2></a>
- <b>Detailed list of changes:</b>
	- \ref mrpt_base_grp
		- The following features have been finally ported to C++11. User code now requires, at least, C++11 enabled:
			- stlplus-based smart pointers replaced by std::shared_ptr. Backwards compatibility API is maintained.
			- mrpt::system::TThreadHandle now is a wrapper around std::thread.
			- Atomic counters now based on std::atomic. Custom implementation has been removed.
			- stlplus source code has been removed.
		- mrpt::utils::COutputLogger: change log str format from "[name|type|time]" to "[time|type|name]".
	- \ref mrpt_graphslam_grp
		- levenberg-Marquardt graphslam modified to use more stable SE(2) Jacobians.
		- CNetworkOfPoses: read/write format made compatible with G2O EDGE_SE2 types.
	- \ref mrpt_nav_grp
		- Add virtual method CAbstractPTGBasedReactive::getHoloMethod()
		- New method CAbstractPTGBasedReactive::enableRethrowNavExceptions() to rethrow exceptions during navigation.
		- Waypoints now have a field `speed_ratio` which is directly forwarded to the low-level reactive navigator.
	- BUG FIXES:
		- Fix missing "-ldl" linker flag.
		- Fix building against wxWidgets 3.1.1 in Windows (zlib link error).
		- Fix potential segfault in 3D reactive navigator.

<hr>
<a name="1.5.6">
<h2>Version 1.5.6: Released 24/APR/2018 </h2></a>
  - Applications:
    - pf-localization:
      - Odometry is now used also for observation-only rawlogs.
  - \ref mrpt_hwdrivers_grp
    - mrpt::hwdrivers::COpenNI2Generic: added mutexes for safer
multi-threading operation.
    - mrpt::hwdrivers::CHokuyoURG: Added a new parameter to skip scans.
Driver clean up to be safer and perform less memory allocs.
  - \ref mrpt_maps_grp
    - COccupancyGridMap2D: New LIDAR insertion parameters:
maxFreenessUpdateCertainty, maxFreenessInvalidRanges.
  - \ref mrpt_reactivenav_grp
    - CAbstractPTGBasedReactive: Added new score `holo_stage_eval`.
  - BUG FIXES:
    - circular_buffer: exception made state preserving

<hr>
<a name="1.5.5">
<h2>Version 1.5.5: (Under development) </h2></a>
- <b>Detailed list of changes:</b>
  - \ref mrpt_nav_grp
    - mrpt::nav::CHolonomicFullEval now uses an internal sin/cos LUT cache
for improved performance.
  - \ref mrpt_hwdrivers_grp
    - A new class for SICK TIM561(TIM55x/TIM56x) lidar:
      - A new source file named CSICKTim561Eth_2050101.cpp, which supports
SICK TIM series lidar including Tim55x, Tim56x
      - mrpt::hwdrivers::CSICKTim561Eth
    - A new test sample for SICK TIM561(TIM55x/TIM56x) lidar:
      - sample/SICK_tim561eth_test/test.cpp
  - BUG FIXES:
    - Fix likelihood computation in mrpt::maps::CReflectivityGridMap2D
(which led to crash)
    - Fixed regression in particle resampling affecting RBPF-SLAM methods.
Introduced in Dec. 2016 with [this
commit](https://github.com/MRPT/mrpt/commit/691973813bdc53d3faa7088b092eb041aa80d0ce).

<hr>
<a name="1.5.4">
<h2>Version 1.5.4: Released 31/OCT/2017 </h2></a>
- <b>Detailed list of changes:</b>
  - \ref mrpt_base_grp
    - Fix potential uninitialized value in
CRobot2DPoseEstimator::getLatestRobotPose()
    - MRPT_getCompilationDate() returns time as well
  - \ref mrpt_gui_grp
    - mrpt::gui::mrptEventMouseMove:  Added new mrpt::gui windows event
type.
  - Build system:
    - Fix MRPTConfig.cmake for system octomap libraries.
    - Fix package-contains-vcs-control-file (.gitingore) Lintian error.
    - Fix compiling without liboctomap-dev in Ubuntu PPA.
  - BUG FIXES:
    - Fix waypoint reactive navigator edge case in which "end event" won't
be issued.
    - Fix waypoint reactive navigator error while doing final aligning
(missing and dupplicated nav-end events).
    - Fix aborting reading from LMS111 scanner on first error.
    - Fix waypoint reactive navigator edge case in which "end event" won't
be issued.
    - Fix corrupted pointers in CNetworkOfPoses after copy or move
operations.
    - Fix invalid TP-targets generated during reactive navigation.
    - Fix memory leak in reactivenav engine.
    - Fix potential out-of-range access in
CObservation3DRangeScan::convertTo2DScan()

<hr>
<a name="1.5.3">
<h2>Version 1.5.3: Released 13/AUG/2017  </h2></a>
- <b>Detailed list of changes:</b>
  - CMake >=3.1 is now required for use of ExternalProjects.
  - Scripts `packaging/prepare_{debian,release}.sh` have been refactored and
simplified.
  - Removed embedded source code versions of Eigen, assimp and octomap.
Downloaded and built as ExternalProjects if not present in the system.
  - Releases will be signed with PGP from now on and posted as binary
attachments to GitHub releases.

<hr>
<a name="1.5.2">
<h2>Version 1.5.2: Released 6/AUG/2017 </h2></a>
- <b>Detailed list of changes:</b>
  - Changes in libraries:
    - \ref mrpt_base_grp
      - Added methods:
        - mrpt::synch::CCriticalSection::try_enter()
        - mrpt::synch::CCriticalSectionRecursive::try_enter()
    - \ref mrpt_nav_grp
      - mrpt::nav::CAbstractNavigator: callbacks in
mrpt::nav::CRobot2NavInterface are now invoked *after* `navigationStep()` to
avoid problems if user code invokes the navigator API to change its state.
      - Added methods to load/save mrpt::nav::TWaypointSequence to
configuration files.
    - \ref mrpt_slam_grp
      - rbpf-slam: Add support for simplemap continuation.
  - BUG FIXES:
    - Fix reactive navigator inconsistent state if navigation API is called
from within rnav callbacks.
    - Fix incorrect evaluation of "ASSERT" formulas in
mrpt::nav::CMultiObjectiveMotionOptimizerBase

<hr>
<a name="1.5.1">
<h2>Version 1.5.1: Released 21/JUN/2017  </h2></a>
- <b>Detailed list of changes:</b>
  - Changes in libraries:
    - \ref mrpt_nav_grp
      - fix const-correctness:
[commit](https://github.com/MRPT/mrpt/commit/7e79003d2adeb7b170fa04e0bc34d42707e07306)
      - More flexible callback behavior:
[commit](https://github.com/MRPT/mrpt/commit/5b054336a1ac75f6e4f8741e5049971917a2980a)


<hr>
<a name="1.5.0">
<h2>Version 1.5.0: Released 10-JUN-2018</h2></a>
  - Changes in apps:
    - New app
[PTG-configurator](http://www.mrpt.org/list-of-mrpt-apps/application-ptg-configurator/)
    -
[ReactiveNavigationDemo](http://www.mrpt.org/list-of-mrpt-apps/application-reactivenavigationdemo/)
has been totally rebuilt as a 3D visualizer capable of testing different
navigation algorithms and robot kinematics.
    - [RawLogViewer](http://www.mrpt.org/list-of-mrpt-apps/rawlogviewer/):
      - Now displays a textual and graphical representation of all
observation timestamps, useful to quickly detect sensor "shortages" or temporary
failures.
      - New menu operation: "Edit" -> "Rename selected observation"
      - mrpt::obs::CObservation3DRangeScan pointclouds are now shown in
local coordinates wrt to the vehicle/robot, not to the sensor.
    -
[rawlog-edit](http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-edit/):
New flag: `--txt-externals`
  - Changes in libraries:
    - \ref mrpt_base_grp
      - New API to interface ZeroMQ: \ref noncstream_serialization_zmq
      - Deprecated function (since 1.3.0) deleted:
mrpt::system::registerFatalExceptionHandlers()
      - New method mrpt::poses::CPosePDFParticles::resetAroundSetOfPoses()
      - Class mrpt::utils::CRobotSimulator renamed ==>
mrpt::kinematics::CVehicleSimul_DiffDriven
      - New twist (linear + angular velocity state) classes:
mrpt::math::TTwist2D, mrpt::math::TTwist3D
      - New template method: mrpt::utils::CStream::ReadAsAndCastTo
      - Added missing method mrpt::poses::CPose2D::inverseComposePoint()
for consistency with CPose3D
      - New class std::recursive_mutex
      - New class mrpt::system::COutputLogger replaces the classes
mrpt::utils::CDebugOutputCapable (deprecated) and mrpt::utils::CLog (removed).
      - New macros for much more versatily logging:
        - MRPT_LOG_DEBUG(), MRPT_LOG_INFO(), MRPT_LOG_WARN(),
MRPT_LOG_ERROR()
        - MRPT_LOG_DEBUG_STREAM, MRPT_LOG_INFO_STREAM,
MRPT_LOG_WARN_STREAM, MRPT_LOG_ERROR_STREAM
      - New functions for polynomial roots: see \ref polynomial_roots
      - New functions for signal filtering: see \ref filtering_grp
      - New functions for Fresnel integrals: see \fresnel_integrals_grp
      - New classes mrpt::math::CAtan2LookUpTable,
mrpt::math::CAtan2LookUpTableMultiRes
      - [API change] The following functions are no longer static methods:
(since their classes are now derived from the state-aware
mrpt::system::COutputLogger)
        - mrpt::math::RANSAC_Template::execute()
        - mrpt::math::CLevenbergMarquardtTempl::execute()
      - Deleted methods in Eigen-extensions: leftDivideSquare(),
rightDivideSquare()
      - Removed support for **named** semaphores in
mrpt::synch::CSemaphore
      - new method mrpt::system::CTimeLogger::getLastTime()
      - Removed mrpt::utils::CStartUpClassesRegister, replaced by the new
macro MRPT_INITIALIZER()
      - New class mrpt::utils::CRateTimer
      - mrpt::poses::CRobot2DPoseEstimator now uses a more generic
odometry-based velocity model (vx,vy,omega).
      - New template mrpt::utils::ts_hash_map<> for thread-safe,
std::map-like containers based on hash functions.
      - Included exprtk header-only library to runtime compile &
evaluation of mathematical expressions, under `<mrpt/3rdparty/exprtk.hpp>`
      - New smart pointer templates: `mrpt::utils::copy_ptr<>`,
`mrpt::utils::poly_ptr<>`.
      - New colormap: mrpt::utils::hot2rgb()
      - New function mrpt::system::find_mrpt_shared_dir()
      - New class mrpt::containers::CDynamicGrid3D<>
      - New function mrpt::comms::net::http_request()
      - New function mrpt::system::now_double()
      - New function mrpt::rtti::getAllRegisteredClassesChildrenOf()
      - Safer CClassRegistry: detect and warn on attempts to duplicated
class registration.
      - New class mrpt::expr::CRuntimeCompiledExpression
      - mrpt::config::CConfigFile and mrpt::config::CConfigFileMemory now
can parse config files with end-of-line backslash to split long strings into
several lines.
      - New class mrpt::poses::FrameTransformer
      - mrpt::poses classes now have all their constructors from
mrpt::math types marked as explicit, to avoid potential ambiguities and
unnoticed conversions.
      - [Sophus](https://github.com/strasdat/Sophus/) is now used
internally for some Lie Algebra methods, and also exposed to the user as
`#include <mrpt/3rdparty/sophus/so3.hpp>`, etc. as part of mrpt-base
    - \ref mrpt_bayes_grp
      - [API change] `verbose` is no longer a field of
mrpt::bayes::CParticleFilter::TParticleFilterOptions. Use the
setVerbosityLevel() method of the CParticleFilter class itself.
      - [API change] mrpt::bayes::CProbabilityParticle (which affects all
PF-based classes in MRPT) has been greatly simplified via usage of the new
mrpt::utils::copy_ptr<> pointee-copy-semantics smart pointer.
    - \ref mrpt_graphs_grp
      - New class mrpt::graphs::ScalarFactorGraph, a simple but extensible
linear GMRF solver. Refactored from mrpt::maps::CGasConcentrationGridMap2D, etc.
    - \ref mrpt_gui_grp
      - mrpt::gui::CWxGLCanvasBase is now derived from
mrpt::opengl::CTextMessageCapable so they can draw text labels
      - New class mrpt::gui::CDisplayWindow3DLocker for exception-safe 3D
scene lock in 3D windows.
    - \ref mrpt_hwdrivers_grp
      - Using rplidar newest SDK 1.5.6 instead of 1.4.3, which support
rplidar A1 and rplidar A2
      - mrpt::hwdrivers::CNTRIPEmitter can now also dump raw NTRIP data to
a file
    - \ref mrpt_kinematics_grp
      - New classes for 2D robot simulation:
        - mrpt::kinematics::CVehicleSimul_DiffDriven
        - mrpt::kinematics::CVehicleSimul_Holo
      - New classes for 2D robot kinematic motion commands. See children
of mrpt::kinematics::CVehicleVelCmd
    - \ref mrpt_maps_grp
      - mrpt::maps::COccupancyGridMap2D::loadFromBitmapFile() correct
description of `yCentralPixel` parameter.
      - mrpt::maps::CPointsMap `liblas` import/export methods are now in a
separate header. See \ref mrpt_maps_liblas_grp and \ref dep-liblas
      - New class mrpt::maps::CRandomFieldGridMap3D
      - New class mrpt::maps::CPointCloudFilterByDistance
    - \ref mrpt_obs_grp
      - [ABI change] mrpt::obs::CObservation2DRangeScan
        - range scan vectors are now protected for safety.
        - New getter/setter methods.
        - backwards-compatible proxies added for read-only from range
scan members.
      - [ABI change] mrpt::obs::CObservation3DRangeScan:
        - Now uses more SSE2 optimized code
        - Depth filters are now available for
mrpt::obs::CObservation3DRangeScan::unprojectInto() and
mrpt::obs::CObservation3DRangeScan::convertTo2DScan()
        - New switch
mrpt::obs::CObservation3DRangeScan::EXTERNALS_AS_TEXT for runtime selection of
externals format.
      - mrpt::obs::CObservation2DRangeScan now has an optional field for
intensity.
      - mrpt::obs::CRawLog can now holds objects of arbitrary type, not
only actions/observations. This may be useful for richer logs aimed at
debugging.
      - mrpt::obs::CObservationVelodyneScan::generatePointCloud() can now
generate the microseconds-precise timestamp for each individual point (new param
`generatePerPointTimestamp`).
    - \ref mrpt_opengl_grp
      - [ABI change] mrpt::opengl::CAxis now has many new options exposed
to configure its look.
      - mrpt::opengl::CSetOfLines can now optionally show vertices as
dots.
      - lib3DS is no longer shipped as an embedded version. A system
library in Linux is required to use mrpt::opengl::C3DSScene. Use
mrpt::opengl::CAssimpModel as a more powerful alternative.
    - \ref mrpt_slam_grp
      - [API change] mrpt::slam::CMetricMapBuilder::TOptions does not have
a `verbose` field anymore. It's supersedded now by the verbosity level of the
CMetricMapBuilder class itself.
      - [API change] getCurrentMetricMapEstimation() renamed
mrpt::slam::CMultiMetricMapPDF::getAveragedMetricMapEstimation() to avoid
confusions.
    - \ref mrpt_hwdrivers_grp
      - mrpt::hwdrivers::CGenericSensor: external image format is now
`png` by default instead of `jpg` to avoid losses.
      - [ABI change] mrpt::hwdrivers::COpenNI2Generic:
        - refactored to expose more methods and allow changing
parameters via its constructor.
        - Now supports reading from an IR, RGB and Depth channels
independenty.
      -  mrpt::hwdrivers::CHokuyoURG now can optionally return intensity
values.
      - Deleted old, unused classes:
        - mrpt::hwdrivers::CBoardIR
        - mrpt::hwdrivers::CBoardDLMS
        - mrpt::hwdrivers::CPtuHokuyo
      - mrpt::hwdrivers::CHokuyoURG no longer as a "verbose" field. It's
superseded now by the COutputLogger interface.
      - mrpt::hwdrivers::CActivMediaRobotBase and the embedded ARIA
library have been removed. Nowadays, one can access to ARIA robots via ROS
packages more easily than via MRPT.
    - \ref mrpt_maps_grp
      - mrpt::maps::CMultiMetricMapPDF added method
CMultiMetricMapPDF::prediction_and_update_pfAuxiliaryPFStandard().
    - \ref mrpt_nav_grp
      - New mrpt::nav::CWaypointsNavigator interface for waypoint
list-based navigation.
      - [ABI & API change] PTG classes refactored (see new virtual base
class mrpt::nav::CParameterizedTrajectoryGenerator and its derived classes):
        - Old classes `CPTG%d` have been renamed to describe each path
type. Old PTGs #6 and #7 have been removed for lack of practical use.
        - New separate classes for PTGs based on numerically-integrated
paths and on closed-form formulations.
        - Old deprecated method of PTGs `lambdaFunction()` removed.
        - Parameters are no longer passed via a
mrpt::system::TParameters class, but via a mrpt::config::CConfigFileBase which
makes parameter passing to PTGs much more maintainable and consistent.
        - PTGs now have a score_priority field to manually set hints
about preferences for path planning.
        - PTGs are now mrpt::config::CLoadableOptions classes
      - New classes:
        - mrpt::nav::CMultiObjectiveMotionOptimizerBase
    - \ref mrpt_graphslam_grp
      - Extend mrpt-graphslam lib to execute simulated/real-time
graphSLAM. mrpt-graphslam supports 2D/3D execution of graphSLAM, utilizing
        LaserScans, odometry information.
      - Develop application `graphslam-engine` that executes graphSLAM via
        the mrpt-graphslam lib
      - mrpt::grpahslam::CGraphSlamEngine as the generic object that
         manages graphSLAM, Node/Edge registration decider
         classes under the mrpt::graphslam::deciders namesapce, optimizer
         wrapper classes under mrpt::graphslam::optimizers
  - Changes in build system:
    - [Windows only] `DLL`s/`LIB`s now have the signature
`lib-${name}${2-digits-version}${compiler-name}_{x32|x64}.{dll/lib}`, allowing
several MRPT versions to coexist in the system PATH.
    - [Visual Studio only] There are no longer `pragma comment(lib...)` in
any MRPT header, so it is the user responsibility to correctly tell user
projects to link against MRPT libraries. Normally, this is done with the
standard command `TARGET_LINK_LIBRARIES(MYTARGET ${MRPT_LIBS})`.
    - Debian package: depends on libopenni-dev
    - Optional dependency `liblas`: minimum required version is now 1.6.0
(Ubuntu Trusty or above).
    - Update of embedded copy of nanoflann to version 1.2.0.
    - New script for automated dumping stack traces on unit tests failures
(`tests/run_all_tests_gdb.sh`)
    - Fix build against wxWidgets 3.1.*
    - Embedded version of gtest upgraded to 1.8.0
  - BUG FIXES:
    - Fix inconsistent state after calling
mrpt::obs::CObservation3DRangeScan::swap()
    - Fix SEGFAULT in mrpt::obs::CObservation3DRangeScan if trying to build
a pointcloud in an external container (mrpt::opengl, mrpt::maps)
    - Fix mrpt::hwdrivers::CHokuyoURG can return invalid ray returns as
valid ranges.
    - Fix PTG look-up-tables will always fail to load from cache files and
will re-generate (Closes [GitHub #243](https://github.com/MRPT/mrpt/issues/243))
    - Fix mrpt::maps::COccupancyGridMap2D::simulateScanRay() fails to mark
out-of-range ranges as "invalid".
    - Fix mrpt::io::CMemoryStream::Clear() after assigning read-only
memory blocks.
    - Fix point into polygon checking not working for concave polygons. Now,
mrpt::math::TPolygon2D::contains() uses the winding number test which works for
any geometry.
    - Fix inconsistent internal state after externalizing
mrpt::obs::CObservation3DRangeScan
    - Fix a long outstanding bug regarding losing of keystroke events in
CDisplayWindow3D windows (Closes #13 again)
    - Fix wrong units for negative numbers in mrpt::system::unitsFormat()
    - Fix potential thread-unsafe conditions while inserting a
mrpt::obs::CObservation2DRangeScan into a pointmap with SSE2 optimizations
enabled.
    - CStream: Fix memory leak if an exception (e.g. EOF) is found during
object deserialization.
    - Fix a bug in the `onlyUniqueRobust` option for point cloud matching
(affecting CICP, etc.). Thanks [Shuo](https://github.com/ygzhangsoya)!

<hr>
<a name="1.4.0">
  <h2>Version 1.4.0: Released 22-APR-2016  </h2></a>
  - <b>Most important changes:</b>
    - Support for Velodyne LIDAR sensors.
    - New minor version number due to changes in the API of these classes
(read details below): mrpt::obs::CObservationGPS, mrpt::hwdrivers::CGPSInterface
    - [Python bindings](https://github.com/MRPT/mrpt/wiki/PythonBindings)
added for a subset of MRPT functionality (Thanks Peter Rudolph and Nikolaus
Demmel!)
  - <b>Detailed list of changes:</b>
    - New apps:
      -
[gps2rawlog](http://www.mrpt.org/list-of-mrpt-apps/application-gps2rawlog/):
Application to parse raw dumps of a GPS (GNSS) receiver output.
      -
[image2gridmap](http://www.mrpt.org/list-of-mrpt-apps/application-image2gridmap/):
Small tool to import any image as an MRPT gridmap object file (`*.gridmap`).
      -
[velodyne-view](http://www.mrpt.org/list-of-mrpt-apps/application-velodyne-view/):
Application to test, visualize and grab data from a live Velodyne sensor or from
a PCAP record.
    - Changes in apps:
      -
[rawlog-grabber](http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/):
Now does not show GPS and IMU debug data in console, unless
`MRPT_HWDRIVERS_VERBOSE` environment variable is set.
      -
[rawlog-edit](http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-edit/):
New operation: `--export-gps-all`
    - Changes in libraries:
      - \ref mrpt_base_grp
        - [ABI change] mrpt::system::tokenize() new parameter
`skipBlankTokens`
        - mrpt::utils::circular_buffer now has peek() methods
        - Eigen::MatrixBase<Derived>::loadFromTextFile() now also
accepts `,` as column separator.
        - New functions:
          - mrpt::system::timestampAdd()
          - mrpt::utils::compute_CRC32()
          - mrpt::utils::saturate<>()
        - mrpt::containers::CDynamicGrid<> now uses `double` instead of
`float` for all dimensions and coordinate computations.
        - Priority with these functions now work properly in GNU/Linux;
though, see the notes in their documentation for required permissions:
          - mrpt::system::changeCurrentProcessPriority()
          - mrpt::system::changeThreadPriority()
        - New classes/structures:
          - mrpt::math::TPointXYZIu8, mrpt::math::TPointXYZRGBu8,
mrpt::math::TPointXYZfIu8, mrpt::math::TPointXYZfRGBu8
      - \ref mrpt_hwdrivers_grp
        - New class mrpt::hwdrivers::CVelodyneScanner
        - mrpt::hwdrivers::CNTRIPEmitter now has a parameter to
enable/disable sending back the data from the serial port to the NTRIP caster.
        - <b>[API changed]</b> mrpt::hwdrivers::CGPSInterface API
clean-up and made more generic so any stream can be used to parse GNSS messages,
not only serial ports.
        - New class mrpt::hwdrivers::CStereoGrabber_Bumblebee_libdc1394
for capturing without PGR Flycapture but directly through libdc1394.
        - Removed class mrpt::hwdrivers::CStereoGrabber_Bumblebee ,
superseded by mrpt::hwdrivers::CImageGrabber_FlyCapture2 which is capable of
both monocular and stereo grabbing.
      - \ref mrpt_maps_grp
        - New class mrpt::maps::CHeightGridMap2D_MRF
        - New base class mrpt::maps::CHeightGridMap2D_Base
        - mrpt::maps::COccupancyGridMap2D:
          - New method
mrpt::maps::COccupancyGridMap2D::copyMapContentFrom()
          - New likelihood parameter `LF_useSquareDist`
          - New parameter
mrpt::maps::COccupancyGridMap2D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS
          - mrpt::maps::COccupancyGridMap2D::simulateScanRay() is now
~40% (GCC) to ~250% (MSVC) faster by default.
          - New method
mrpt::maps::COccupancyGridMap2D::laserScanSimulatorWithUncertainty()
        - New method
mrpt::maps::CHeightGridMap2D::insertIndividualPoint()
        - mrpt::maps::CMetricMap::compute3DMatchingRatio() has a
simplified API now
      - \ref mrpt_obs_grp
        - New class mrpt::obs::CObservationVelodyneScan
        - mrpt::obs::CSinCosLookUpTableFor2DScans now can build a table
from a mrpt::obs::T2DScanProperties structure, which now also has its separate
header file for better modularity.
        - <b>[API changed]</b> mrpt::obs::CObservationGPS now stores
only one message per objects. API clean-up and extended so the number of GNSS
message types is larger and more scalable.
        - mrpt::obs::gnss: A new namespace with many new data structures
for GPS-related messages
        - mrpt::obs::CObservation3DRangeScan: projection of RGBD images
to 3D points now correctly filters out invalid points, which were in previous
versions mapped as (0,0,0) points (relative to the sensor). In turn, this leads
to point clouds of a dynamic number of points. In case of needing the (u,v)
pixel coordinates of projected points, checkout the new fields `points3D_idxs_x`
& `points3D_idxs_y`.
        - New class mrpt::obs::CObservation2DRangeScanWithUncertainty
      - \ref mrpt_opengl_grp
        - New class mrpt::opengl::CMesh3D to render 3D models/meshes
        - New method
mrpt::opengl::CPointCloudColoured::recolorizeByCoordinate()
      - \ref mrpt_slam_grp
        - Small clean up of mrpt::slam::CICP API, add separate variable
to select covariance estimation method.
      - \ref mrpt_topography_grp
        - New function mrpt::topography::geocentricToENU_WGS84()
      - \ref mrpt_vision_grp
        - Removed the old GPL-licensed Hess SIFT implementation.
        - mrpt::vision::CDifOdo has been refactored and now does faster
image pyramid computation (By Mariano Jaimez)
        - mrpt::maps::CLandmarksMap changes:
          - `beaconMaxRange` & `alphaRatio` parameters have been
removed since they were not used.
          - New likelihood parameter `beaconRangesUseObservationStd`
to allow using different uncertainty values with each observation.
    - Changes in build system:
      - [Python
bindings](https://github.com/MRPT/mrpt/wiki/PythonBindings) added for a subset
of MRPT functionality (Thanks Peter Rudolph!)
      - Code ported to support the new libftdi1-dev (Fixes Debian bug
#810368, GitHub issue #176)
      - Fix building with gcc 6.0 (Closes Debian bug #811812)
      - CMake new option: `DISABLE_MRPT_AUTO_CLASS_REGISTRATION` to reduce
the footprint of MRPT statically-linked programs.
      - Fix building against wxWidgets 3.1
    - BUG FIXES:
      - mrpt::math::CQuaternion<> did not check for unit norm in Release
builds.
      - Fix build errors against OpenCV 3.0.0+ without opencv_contrib
modules.
      - mrpt::hwdrivers::CHokuyoURG now correctly handles opening both USB
and Ethernet Hokuyo devices (Closes Github issue #180)
      - Fixed mrpt::comms::net::DNS_resolve_async() may SIGSEGV in slow
networks.
      - mrpt::opengl::CMesh::updateColorsMatrix() did not ignore cells
masked out.
      - Wrong weights used in mrpt::poses::CPosePDFSOG::getMean()
      - Removed ad-hoc bias addition in range-only predictions in
landmarks maps.
      - Error loading height map count in
mrpt::maps::TSetOfMetricMapInitializers (Closes GitHub issue <a
href="https://github.com/MRPT/mrpt/issues/205" >#205</a>.
      - Fix "gray images" grabbed in Windows when capturing the render
output of 3D windows (Thanks Mariano J.T. & Christian Kerl from TUM!)
      - Fix typos and wxWidgets align errors in RawLogViewer GUI (Closes
#219)
      - mrpt::nav::CHolonomicND & mrpt::nav::CHolonomicVFF didn't use the
full range of output velocities.
      - mrpt::img::CImage::loadFromFile() now does not leave the image in
undefined state if the load operation fails.
      - mrpt::hwdrivers::CLMS100Eth failed to load "pose_yaw" parameter
from config file.
      -
mrpt::obs::CObservation3DRangeScan::doDepthAndIntensityCamerasCoincide() did not
correctly return `false` for negative offsets between the camera poses.

<hr>
<a name="1.3.2">
  <h2>Version 1.3.2: Released 3-NOV-2015 </h2></a>
  - Changes in Apps:
    -
[rawlog-edit](http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-edit/):
      - New operation: `--list-poses`
      - `--list-images` now also works with 3D range scans
  - Changes in libraries:
    - The library mrpt-srba has been moved out of MRPT and now is an
independent project: https://github.com/MRPT/srba
    - \ref mrpt_base_grp
      - mrpt::math::KDTreeCapable::TKDTreeSearchParams: Removed parameter
nChecks, which was ignored by nanoflann anyway.
    - \ref mrpt_hwdrivers_grp
      - mrpt::hwdrivers::CCameraSensor: Implemented OpenNI2 support for
CCameraSensor
    - \ref mrpt_nav_grp
      - mrpt::nav::CAbstractPTGBasedReactive: Maximum acceleration filter
(SPEEDFILTER_TAU) now follows paths better (Thanks to Steven Butner, UCSB/ECE)
  - Changes in build system:
    - `FIND_PACKAGE(MRPT)` will return libraries in the var
`MRPT_LIBRARIES`, following the CMake convention. The old variable name
`MRPT_LIBS` will be also returned for backward compatibility.
  - BUG FIXES:
    - Fix excessive width of paths drawn by
CMetricMapBuilderRBPF::drawCurrentEstimationToImage()
    - Fix image distortion: k3 may be ignored. (Thanks to CBaiz)
    - Fix Debian bugs.

<hr>
<a name="1.3.1">
  <h2>Version 1.3.1: Released 18-JUL-2015 </h2></a>
  - Changes in apps:
    -
[navlog-viewer](http://www.mrpt.org/list-of-mrpt-apps/application-navlog-viewer/):
Now shows more information on navigation logs.
    - New app
[icp-slam-live](http://www.mrpt.org/list-of-mrpt-apps/application-icp-slam-live/):
Real-time ICP-SLAM with a LIDAR sensor.
  - Changes in libraries:
    - \ref mrpt_base_grp
      - New helper templates: mrpt::utils::int_select_by_bytecount<>,
mrpt::uint_select_by_bytecount<>
      - New methods to evaluate SO(2), SO(3), SE(2) and SE(3) averages and
weighted averages. See:
        - Header <mrpt/poses/SO_SE_average.h>
        - mrpt::poses::SO_average<2>, mrpt::poses::SO_average<3>
        - mrpt::poses::SE_average<2>, mrpt::poses::SE_average<3>
    - \ref mrpt_hwdrivers_grp
      - New sensors supported:
        - mrpt::hwdrivers::CIMUIntersense
        - mrpt::hwdrivers::CSkeletonTracker
      - New parameter
mrpt::hwdrivers::CHokuyoURG::m_disable_firmware_timestamp to override faulty
Hokuyo timestamps with PC time.
      - mrpt::hwdrivers::CRoboPeakLidar::turnOn() and turnOff() now really
implement turning on/off the RPLidar motor.
    - \ref mrpt_maps_grp
      - New method mrpt::maps::COccupancyGridMap2D::getAsPointCloud()
    - \ref mrpt_nav_grp
      - Removed old base class CPathPlanningMethod
      - CPathPlanningCircularRobot => mrpt::nav::PlannerSimple2D: Class
renamed (and better described) for consistency with other planners
      - mrpt::nav::CReactiveNavigationSystem:
        - Documentation has been added about all existing parameters,
and template config files provided as starting points.
        - The loadConfigFile() method with 2 config files has been
deprecated favoring the newer, simpler single config file.
        - The "ROBOT_NAME" parameter is no longer employed. A minor side
effect (probably affecting no one) is that PTG cache files are no longer named
differently for different robots.
      - mrpt::nav::CParameterizedTrajectoryGenerator: New methods to save
and load trajectories to binary streams. Used to debug in navlog-viewer.
    - \ref mrpt_obs_grp
      - mrpt::obs::CObservation3DRangeScan now supports pixel labels
(semantic mapping, etc.)
      - New class mrpt::obs::CObservationSkeleton to hold body tracking
information (by Francisco Angel Moreno)
      - mrpt::obs::CObservationIMU has new data fields and fields are
better documented to reflect whether they refer to local/global coordinate
frames
    - \ref mrpt_vision_grp
      - mrpt::vision::CImageGrabber_dc1394: Changed default Bayer filter
from NEAREST to HQLINEAR
  - BUG FIXES:
      - Fix ocasional (false) failure of RANSAC unit tests due to their
non-deterministic nature.
      - Fix build error with MSVC 2010 in mrpt-hmtslam (Closes #127).
      - Fixed potential wrong bounding box results in
mrpt::maps::CPointsMap::boundingBox() when SSE2 optimization is enabled.
      - mrpt::obs::CObservation6DFeatures: Fixed random crashes related to
non-aligned memory in 32bit builds (Fixes #141)
      - Fix Debian bug
[#786349](https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=786349) on Eigen2
support.
      - mrpt::hwdrivers::CIMUXSens_MT4: Fix crash in destructor of objects
not attached to a physical device.
      - Fix wrong quaternion cross product when target variable is one of
the operands. Also affected the += operator of mrpt::poses::CPose3DQuat (Fixes
#148)
      - mrpt::hwdrivers::CKinect with libfreenect driver: Fix potential
memory corruption.
      - Fix a bug in mrpt::tfest::se3_l2_robust() that led to it returning
without trying to find a good consensus solution. It affected the demo app
kinect-3d-slam (Fixes #156)
      - Fix wrong feature points in
CFeatureExtraction::extractFeaturesKLT()  (Fixes #138)

<hr>
<a name="1.3.0">
  <h2>Version 1.3.0: Released 12-JAN-2015 </h2></a>
  - <b>Most important changes:</b>
    - Classes in libraries \ref mrpt_obs_grp and \ref mrpt_maps_grp now
belong to new namespaces (mrpt::obs, mrpt::maps) instead of the old mrpt::slam
    - No more `using namespace`s polute MRPT headers. <b>Errors in user
projects</b> missing `using namespace XXX` that might be formerly masked will
now reveal. <b>This is a good thing</b>, though admitedly annoying...
    - New library \ref mrpt_nav_grp, subsumming the old \ref
mrpt_reactivenav_grp.
    - New library \ref mrpt_tfest_grp, a refactor of the old \ref
mrpt_scanmatching_grp.
    - <b>Backwards compatible headers</b> have been provided to ease the
transition of user code for all those library changes. Warning messages will be
shown recommending deprecated replacements.
  - <b>Detailed list of changes:</b>
    - Lib changes:
      - Clean up of the bad practice of `using namespace` in public scopes
of headers. May lead to user code failing for missing `using namespace`s which
were previously masked.
      - Namespace "slam" deprecated in libraries mrpt-obs and mrpt-maps
(used for historical reasons):
        - New namespaces  \ref mrpt_obs_grp and \ref mrpt_maps_grp.
        - #include files moved from old paths <mrpt/slam/...> =>
<mrpt/{obs,maps}/...>
        - Backward compatible headers added in <mrpt/slam/...> until
mrpt 2.0.0
      - New library \ref mrpt_nav_grp, subsumming the old mrpt-reactivenav
(\ref mrpt_reactivenav_grp).
      - \ref mrpt_reactivenav_grp is now a meta-library, depending on \ref
mrpt_nav_grp.
      - \ref mrpt_tfest_grp : Old library mrpt-scanmatching (\ref
mrpt_scanmatching_grp) has been refactored, its API clean-up, and renamed \ref
mrpt_tfest_grp
      - \ref mrpt_scanmatching_grp is now a meta-library, depending on
\ref mrpt_tfest_grp.
      - These classes have been moved between libs for a more sensible
organization:
        - mrpt::slam::CDetectorDoorCrossing ==>
mrpt::detectors::CDetectorDoorCrossing
        - mrpt::slam::CPathPlanningMethod & CPathPlanningCircularRobot:
\ref mrpt_slam_grp ==> \ref mrpt_nav_grp
    - Build System / General changes:
      - Many optimizations in function arguments (value vs ref). Forces
ABI incompatibility with previous versions, hence the change to a new minor
version number.
      - Updated embedded version of Eigen to 3.2.3
      - Kinect: Dropped support for the CL NUI API, which seems
discontinued. Alternatives in use are libfreenect and OpenNI2.
      - libfreenect is now detected in the system and used instead of
compiling the embedded copy of it.
      - Embedded copy of libfreenect has been updated to (23/oct/2014). It
now supports "Kinect for Windows".
      - More selective linking of .so files to avoid useless dependencies
(Fixes #52).
      - (Windows only) MRPT can now be safely built with libusb support
(Freenect, Kinect,...) and it will run on systems without libusb installed, by
means of /DELAYLOAD linking flags.
      - More unit tests.
    - Changes in classes:
      - [mrpt-base]
        - New function mrpt::math::angDistance()
      - [mrpt-hwdrivers]
        - mrpt::hwdrivers::CIMUXSens_MT4: (by Joe Burmeister for Suave
Aerial Software)
          - Upgrade to latest XSens SDK 4.2.1. Requires libudev-dev in
Linux
          - Add GPS observations to CIMUXSens_MT4 for Xsens devices
like GTi-G-700 which have GPS
        - mrpt::hwdrivers::CImageGrabber_dc1394: Length of ring buffer
is now configurable via TCaptureOptions_dc1394::ring_buffer_size
      - [mrpt-maps]
        - Important refactor of internal code related to
mrpt::maps::CMultiMetricMap:
          - All maps (derived from mrpt::maps::CMetricMap) now have a
more uniform interface.
          - Each map now has a `MapDefinition` structure with all its
parameters. See docs for mrpt::maps::TMetricMapInitializer
          - Introduced mrpt::maps::TMapGenericParams to hold
parameters shared in all maps.
      - [mrpt-obs]
        - CObservation::getDescriptionAsText(): New virtual method to
obstain a textual description of observations. Refactoring of messy code
previously in the RawLogViewer app.
      - [mrpt-vision]
        - mrpt::vision::CFeatureExtraction: Removed (unused) optional
ROI parameter in detectors.
    - BUG FIXES:
      - mrpt::poses::CRobot2DPoseEstimator could estimate wrong angular
velocities for orientations near +-180deg.
      - mrpt::system::CDirectoryExplorer::sortByName() didn't sort in
descending order
      - Fixed crashes from MATLAB .mex files:
mrpt::system::registerFatalExceptionHandlers() has no longer effect, and will be
removed in future releases. (Thanks to Jesús Briales García for all the
testing!)
      - Fixed potential crash for Eigen unaligned memory access in 32bit
builds in mrpt::slam::CGridMapAligner and other places ([Closes
#94](https://github.com/MRPT/mrpt/issues/94))

<hr>
<a name="1.2.2">
  <h2>Version 1.2.2: Released 12-SEP-2014  </h2></a>
  - Changes in apps:
    - <a
href="http://www.mrpt.org/list-of-mrpt-apps/application-sceneviewer3d/"
>SceneViewer3D</a>:
      - New menu "File" -> "Import" -> "3D model" which supports many
standard formats (via mrpt::opengl::CAssimpModel)
  - New classes:
    - [mrpt-hwdrivers]
      - mrpt::hwdrivers::CRoboPeakLidar to interface Robo Peak LIDAR
scanners.
    - [mrpt-opengl]
      - mrpt::opengl::CAssimpModel for rendering complex 3D models (many
supported formats) in OpenGL scenes.
  - Changes in classes:
    - Consistency in all "laser scan" classes: angular increments between
rays are now FOV/(N-1) instead of FOV/N.
    - [mrpt-base]
      - New method mrpt::img::CImage::loadTGA()
      - *IMPORTANT*: Changed behavior of CSerializable/CObject macros (see
bugfix below), introducing the new macros DEFINE_SERIALIZABLE_POST_*. May
require changes in user code if serializable classes are defined:
        - Previous version:
          \code
            DEFINE_SERIALIZABLE_PRE_*(...)
            class XXX {
              DEFINE_SERIALIZABLE(XXX)
            };
          \endcode
        - Must be changed in this version to:
          \code
            DEFINE_SERIALIZABLE_PRE_*(...)
            class XXX {
              DEFINE_SERIALIZABLE(XXX)
            };
            DEFINE_SERIALIZABLE_POST_*(...)
          \endcode
    - [mrpt-hwdrivers]
      - Bumblebee2 Linux support in
mrpt::hwdrivers::CImageGrabber_FlyCapture2 via Triclops (by Jesus Briales)
    - [mrpt-maps]
      - New method mrpt::maps::COccupancyGridMap2D::getRawMap()
      - New method
mrpt::maps::CColouredPointsMap::getPCLPointCloudXYZRGB()
    - [mrpt-opengl]
      - mrpt::opengl::CWxGLCanvasBase (affects all 3D rendering classes):
better handling of internal timers for smoother updates while rendering in
multithreading apps.
    - [mrpt-srba]
      - New method to recover the global coordinates graph-slam problem
for a RBA map: mrpt::srba::RbaEngine::get_global_graphslam_problem() (see
example
[MRPT]\samples\srba-examples\srba-tutorials\tutorial-srba-how-to-recover-global-map.cpp)
  - BUG FIXES:
    - mrpt::img::CImage constructor from a matrix crashed.
    - Unit tests: Named semaphores are not tested anymore if it's detected
that the kernel version doesn't support them (Fix Debian 758725).
    - mrpt::synch::CSemaphore [Linux]: didn't call sem_unlink().
    - mrpt::gui::CDisplayWindow3D didn't implement get/set FOV.
    - Valgrind: Fixed potential unaligned memory access warning in point
clouds.
    - Fix build error with AppleClang 5.1 (Closes #71).
    - mrpt::utils::CClientTCPSocket: Use a connection success check that
works on all platforms
    - Important bug fixed regarding a missing dynamic_cast<> in smart
pointers casting. See above possible implications in user code. properly (Patch
by Joe Burmeister).

<hr>
<a name="1.2.1">
  <h2>Version 1.2.1: Released 10-JUL-2014 </h2></a>
  - Changes in classes:
    - [mrpt-base]
      - All points and poses now have a method setToNaN(), e.g.
mrpt::poses::CPose3D::setToNaN()
    - [mrpt-hwdrivers]
      - mrpt::hwdrivers::COpenNI2Sensor now has better support for opening
several RGBD cameras (by Kenzaburo Miyawaki & Eduardo Fernandez)
  - Build system:
    - Fix compilation of SRBA with DEBUG_GARBAGE_FILL_ALL_NUMS=1
    - Fix de-serialization error in mrpt::reactivenav::CLogFileRecord (and
new unit tests added to avoid regressions).
    - Several Debian bugs closed (see packaging/debian/changelog), including
build errors in uncommon platforms (MIPS, kFreeBSD, etc.)

<hr>
<a name="1.2.0">
  <h2>Version 1.2.0: Released 25-JUN-2014  </h2></a>
  - <b>Most important changes:</b>
    - Public header files (.h) have undergone a serious refactoring to
minimize unnecesary dependencies and reduce compile time and memory as much as
possible. As a side effect, user code might need to add new #include<> lines.
This change justifies the new minor version series 1.2.X.
    - MRPT now cleanly builds in clang and OSX.
    - Support for new camera drivers (OpenNI2, DUO3D).
    - Many bug fixes.
  - <b>Detailed list of changes:</b>
    - Changes in apps:
      - [rawlog-edit](http://www.mrpt.org/Application%3Arawlog-edit):
        - New operations: --export-odometry-txt, --recalc-odometry
        - New flag: --rectify-centers-coincide
    - New examples:
      - kitti_dataset2rawlog
    - New classes:
      - [mrpt-base]
        - mrpt::math::ContainerType<CONTAINER>::element_t to allow
handling either Eigen or STL containers seamlessly.
        - mrpt::config::CConfigFilePrefixer
      - [mrpt-hwdrivers]
        - mrpt::hwdrivers::COpenNI2Sensor: Interface to OpenNI2 cameras,
capable of reading from an array of OpenNI2 RGBD cameras (By Eduardo Fernandez)
        - mrpt::hwdrivers::CDUO3DCamera: Interface to DUO3D cameras (By
Francisco Angel Moreno)
        - mrpt::hwdrivers::CGPS_NTRIP: A combination of GPS receiver +
NTRIP receiver capable of submitting GGA frames to enable RTCM 3.0
      - [mrpt-obs]
        - mrpt::obs::CObservation6DFeatures
    - Changes in classes:
      - [mrpt-base]
        - Robust kernel templates moved from mrpt::vision to mrpt::math.
See mrpt::math::RobustKernel<>. Added unit tests for robust kernels.
        - mrpt::poses::CPose3D has new SE(3) methods:
mrpt::poses::CPose3D::jacob_dexpeD_de(),
mrpt::poses::CPose3D::jacob_dAexpeD_de()
        - More efficient mrpt::utils::OctetVectorToObject() (avoid
memory copy).
        - Fixed const-correctness of mrpt::img::CImage::forceLoad() and
mrpt::img::CImage::unload()
      - [mrpt-hwdrivers]
        - mrpt::hwdrivers::CCameraSensor: Added a hook for user code to
run before saving external image files:
mrpt::hwdrivers::CCameraSensor::addPreSaveHook()
        - mrpt::hwdrivers::CNationalInstrumentsDAQ now supports analog
and digital outputs.
        - New method mrpt::hwdrivers::CNTRIPClient::sendBackToServer()
      - [mrpt-srba]
        - Now also implements SE(3) relative graph-slam.
      - [mrpt-vision]
        - mrpt::vision::checkerBoardStereoCalibration: More robust
handling of stereo calibration patterns. OpenCV sometimes detects corners in the
wrong order between (left/right) images, so we detect the situation and fix it.
        - mrpt::vision::findMultipleChessboardsCorners():
          - Now enforces a consistent counterclockwise XYZ coordinate
frame at each detected chessboard.
          - Much more robust in distingishing quads of different
sizes.
    - Build system / public API:
      - Fixes to build in OS X -
[Patch](https://gist.github.com/randvoorhies/9283072) by Randolph Voorhies.
      - Removed most "using namespace" from public headers, as good
practice.
      - Refactoring of MRPT headers.
        - <mrpt/utils/stl_extensions.h> has been split into:
          - <mrpt/serialization/stl_serialization.h>
          - <mrpt/containers/circular_buffer.h>
          - <mrpt/utils/list_searchable.h>
          - <mrpt/containers/bimap.h>
          - <mrpt/utils/map_as_vector.h>
          - <mrpt/containers/traits_map.h>
          - <mrpt/serialization/stl_serialization.h>
          - <mrpt/containers/printf_vector.h>
          - <mrpt/containers/stl_containers_utils.h>
          - <mrpt/utils/ci_less.h>
      - Deleted methods and functions:
        - mrpt::system::breakpoint()
        - mrpt::vector_float is now mrpt::math::CVectorFloat,
mrpt::vector_double is mrpt::math::CVectorDouble, for name consistency. Also,
using Eigen::VectorXf is preferred for new code.
        - mrpt::CImage::rectifyImage() with parameters as separate
vectors.
        - mrpt::maps::CPointsMap::getPoint() with mrpt::poses::CPoint3D
arguments.
        - mrpt::vision::correctDistortion() -> use CImage method instead
        - All previous deprecated functions.
      - Embedded Eigen updated to version 3.2.1
[(commit)](https://github.com/MRPT/mrpt/commit/47913da94a27e98a9115f85b2a530b6c14a10b8f)
[(commit)](https://github.com/MRPT/mrpt/commit/33258761d3b75bf133d38aecb257c64e4d76b21e)
    - BUG FIXES:
      - RawlogViewer app: Fixed abort while converting SF->obs.only
datasets when there is no odometry.
      - mrpt::obs::CSensoryFrame: The cached point map is now invalidated
with any change to the list of observations so it's rebuild upon next call.
      - New implementation of mrpt::synch::CSemaphore avoids crashes in OS
X - by Randolph Voorhies.
      - mrpt::opengl::CArrow was always drawn of normalized length.
      - FlyCapture2 monocular & stereo cameras could return an incorrect
timestamp (only in Linux?).
      - mrpt::system::createDirectory() returned false (error) when the
directory already existed.
      - mrpt::vision::CStereoRectifyMap::rectify() didn't update the left
& right camera poses inside mrpt::obs::CObservationStereoImages objects while
rectifying.
      - RawLogViewer: Operation "convert to SF format" didn't take into
account odometry observations.
      - Fix build errors with GCC 4.9
      - Fix crash of mrpt::hwdrivers::CIMUXSens_MT4's destructor when it
fails to scan and open a device.
      - Fix potential crash in
mrpt::slam::data_association_full_covariance with JCBB when no individually
compatible matching exists
[(commit)](https://github.com/MRPT/mrpt/commit/482472ebd80a3484dce63d294b1ac4e8f001e1eb)

<hr>
 <a name="1.1.0">
  <h2>Version 1.1.0: Released 22-FEB-2014  </h2></a>
  - New apps:
    -
[DifOdometry-Camera](http://www.mrpt.org/list-of-mrpt-apps/application-difodometry-camera).
(By Mariano Jaimez Tarifa)
    -
[DifOdometry-Datasets](http://www.mrpt.org/list-of-mrpt-apps/application-difodometry-datasets).
(By Mariano Jaimez Tarifa)
  - New classes:
    - [mrpt-base]
      - mrpt::synch::CPipe: OS-independent pipe support.
    - [mrpt-hwdrivers]
      - mrpt::hwdrivers::CIMUXSens_MT4 : Support for 4th generation xSens
MT IMU devices.
      - mrpt::hwdrivers::CNationalInstrumentsDAQ: Support for acquisition
boards compatible with National Instruments DAQmx Base -
[(commit)](https://github.com/MRPT/mrpt/commit/a82a7e37997cfb77e7ee9e903bdb2a55e3040b35).
      - mrpt::hwdrivers::CImageGrabber_FlyCapture2: Support for Point Grey
Research's cameras via the FlyCapture2 libray -
[(commits)](https://github.com/MRPT/mrpt/pull/5/commits).
    - [mrpt-maps]
      - There are now two versions of octomaps (by Mariano Jaimez
Tarifa/Jose Luis Blanco) -
[(commit)](http://code.google.com/p/mrpt/source/detail?r=3443)
        - mrpt::maps::COctoMap (only occupancy)
        - mrpt::maps::CColouredOctoMap (occupancy + RGB color)
    - [mrpt-obs]
      - mrpt::obs::CObservationRawDAQ, a placeholder for raw and generic
measurements from data acquisition devices. -
[(commit)](http://code.google.com/p/mrpt/source/detail?r=3459)
    - [mrpt-opengl]
      - mrpt::opengl::CMeshFast, an open gl object that draws a "mesh" as
a structured point cloud which is faster to render (by Mariano Jaimez Tarifa).
-[(commit)](https://github.com/MRPT/mrpt/commit/9306bb4a585387d4c85b3f6e41dd2cbe5a354e80)
      - mrpt::opengl::CVectorField2D, an opengl object that shows a 2D
Vector Field (by Mariano Jaimez Tarifa). -
[(commit)](http://code.google.com/p/mrpt/source/detail?r=3461)
    - [mrpt-reactivenav]
      - mrpt::reactivenav::CAbstractPTGBasedReactive, as part of a large
code refactoring of these classes:
[(commit)](https://github.com/MRPT/mrpt/pull/4)
        - mrpt::reactivenav::CReactiveNavigationSystem
        - mrpt::reactivenav::CReactiveNavigationSystem3D
    - [mrpt-vision]
      - mrpt::vision::CDifodo, a class which implements visual odometry
based on depth images and the "range flow constraint equation". (by Mariano
Jaimez Tarifa) -
[(commit)](https://github.com/MRPT/mrpt/commit/e6ab5595f70cb889d07658c0b540c27e495a1cfb)
  - Changes in classes:
    - Clean up and slight optimization of metric map matching API: -
[(commit)](http://code.google.com/p/mrpt/source/detail?r=3446)
      - <b>Methods marked as deprecated: </b>
        - mrpt::maps::CMetricMap::computeMatchingWith2D() -->
mrpt::maps::CMetricMap::determineMatching2D()
        - mrpt::maps::CMetricMap::computeMatchingWith3D() -->
mrpt::maps::CMetricMap::determineMatching3D()
      - New structures:
        - mrpt::slam::TMatchingParams
        - mrpt::slam::TMatchingExtraResults
    - mrpt::maps::CPointsMap::TInsertionOptions now have methods to
save/load from binary streams, making more maintainable the serialization of
point maps -
[(commit)](https://github.com/MRPT/mrpt/commit/544d439c3462228b07344142de68e5bc10c1a2e3)
    - New options in point maps:
mrpt::maps::CPointsMap::TInsertionOptions::insertInvalidPoints -
[(commit)](https://github.com/MRPT/mrpt/pull/8)
    - mrpt::obs::CObservationIMU now includes data fields for 3D
magnetometers and altimeters. -
[(commit)](http://code.google.com/p/mrpt/source/detail?r=3451)
    - Method renamed mrpt::utils::CEnhancedMetaFile::selectVectorTextFont()
to avoid shadowing mrpt::CCanvas::selectTextFont()
    - mrpt::reactivenav::CParameterizedTrajectoryGenerator: New methods:
      -
mrpt::reactivenav::CParameterizedTrajectoryGenerator::inverseMap_WS2TP() for
inverse look-up of WS to TP space -
[(commit)](https://github.com/MRPT/mrpt/commit/4d04ef50e3dea581bed6287d4ea6593034c47da3)
      -
mrpt::reactivenav::CParameterizedTrajectoryGenerator::renderPathAsSimpleLine() -
[(commit)](https://github.com/MRPT/mrpt/commit/a224fc2489ad00b3ab116c84e8d4a48532a005df)
    - Changed the signature of
mrpt::reactivenav::build_PTG_collision_grids() to become more generic for 2D
& 2.5D PTGs -
[(commit)](https://github.com/MRPT/mrpt/commit/7bd68e49a4ba3bf08f194678787816c65de1d685)
  - Deleted classes:
    - mrpt::utils::CEvent, which was actually unimplemented (!)
    - mrpt::hwdrivers::CInterfaceNI845x has been deleted. It didn't offer
features enough to justify a class.
  - New examples:
    - [MRPT]/samples/threadsPipe
    - [MRPT]/samples/NIDAQ_test
    - [MRPT]/openNI2_RGBD_demo (by Mariano Jaimez Tarifa)
    - [MRPT]/openNI2_proximity_demo (by Mariano Jaimez Tarifa)
  - Build system:
    - Fixed compilation with clang.
    - Fixed building against OpenCV 3.0.0 (GIT head)
    - Updated to the latest nanoflann 1.1.7.
    - Updated to Eigen 3.2.0 -
[(commit)](http://code.google.com/p/mrpt/source/detail?r=3455)
    - Binary packages for Windows now include .pdb files to help debugging
with Visual Studio.
  - BUG FIXES:
    - Fixed potential infinity loop in mrpt::math::make_vector<1,T>()
    - Fixed build error with GCC when experimental parallelization is
enabled. [(commit)](http://code.google.com/p/mrpt/source/detail?r=3441)
    - mrpt::reactivenav::CReactiveNavigationSystem complained about missing
config variables ROBOTMODEL_TAU & ROBOTMODEL_DELAY, which were removed in
MRPT 1.0.2 - [(commit)](http://code.google.com/p/mrpt/source/detail?r=3452)
    - Fixed potential mem alignment errors (Eigen's UnalignedArrayAssert) in
SRBA for 32bit builds.
[(commit)](http://code.google.com/p/mrpt/source/detail?r=3457)
    - mrpt::topography::geodeticToENU_WGS84() and related functions used a
local +Z axis aligned to the line towards the Earth center; now the Z axis
points normally to the ellipsoid surface. The difference with the previous
behavior is small but may be of a few millimeters for each meter from the
reference point. [(commit)](http://code.google.com/p/mrpt/source/detail?r=3473)
    - Potential crash when setting mpPolygon::setPoints() with empty vectors
- [(commit)](http://code.google.com/p/mrpt/source/detail?r=3478)
    - mrpt::reactivenav::CReactiveNavigationSystem and
mrpt::reactivenav::CReactiveNavigationSystem3D didn't obey the
"enableConsoleOutput" constructor flag -
[(commit)](https://github.com/MRPT/mrpt/commit/db7b0e76506af2c24f119a28443a1e8f1a217861)
    - mrpt::synch::CSemaphore::waitForSignal() : Fixed error when thread got
an external signal
[(commit)](https://github.com/MRPT/mrpt/commit/511e95f03480537ff18ad2cad178c504b1cfbb53)

 <hr>
 <a name="1.0.2">
  <h2>Version 1.0.2: Released 2-AUG-2013 (SVN 3435)  </h2></a>
  - New apps:
    -
[ReactiveNav3D-Demo](http://www.mrpt.org/Application%3AReactiveNav3D-Demo) (By
Mariano Jaimez Tarifa)
  - Changes in apps:
    - [rawlog-edit](http://www.mrpt.org/Application%3Arawlog-edit):
      - New operations: --list-timestamps, --remap-timestamps,
--export-2d-scans-txt, --export-imu-txt
  - New classes:
    - [mrpt-base]
      - mrpt::poses::CPose3DRotVec is now fully implemented (By Francisco
Angel Moreno).
    - [mrpt-opengl]
      - mrpt::opengl::CLight - OpenGL scenes now allow customization of
OpenGL lighting. See also new lighting methods in mrpt::opengl::COpenGLViewport
- <a href="http://code.google.com/p/mrpt/source/detail?r=3409" >r3409</a>
    - [mrpt-reactivenav]
      - mrpt::reactivenav::CReactiveNavigationSystem3D - By Mariano Jaimez
Tarifa - <a href="http://code.google.com/p/mrpt/source/detail?r=3389" >r3389</a>
  - New functions:
    - [mrpt-opengl]
      - mrpt::opengl::stock_objects::RobotRhodon()
  - Changes in classes:
    - [mrpt-base]
      - Generic particle filter classes now allow directly resampling to a
dynamic number of particles. Affected methods: - <a
href="http://code.google.com/p/mrpt/source/detail?r=3381" >r3381</a>
        - mrpt::bayes::CParticleFilterCapable::performResampling()
        - mrpt::bayes::CParticleFilterCapable::computeResampling()
      - New method: CImage::loadFromXPM() - <a
href="http://code.google.com/p/mrpt/source/detail?r=3397" >r3397</a>
    - [mrpt-maps]
      - mrpt::maps::COctoMap now exposes the inner octomap::OcTree object.
See example samples/octomap_simple - <a
href="http://code.google.com/p/mrpt/source/detail?r=4304" >r4304</a>
    - [mrpt-openg]
      - mrpt::opengl::CBox now be also rendered as a solid box + line
borders. See mrpt::opengl::CBox::enableBoxBorder()
      - mrpt::opengl::COctoMapVoxels - <a
href="http://code.google.com/p/mrpt/source/detail?r=4329" >r4329</a>
        - Fixed calculation of normals (fix shading)
        - Added new coloring scheme to
mrpt::opengl::COctoMapVoxels::visualization_mode_t : "FIXED"
        - By default, light effects are disabled in this object, because
shadows aren't computed anyway and the effect isn't pleasant.
        - Voxels cubes are sorted in ascending Z order so the visual
effect is correct when rendering with transparency.
    - [mrpt-reactivenav]
      - mrpt::reactivenav::CParameterizedTrajectoryGenerator: The "low
pass filter" has been removed since it wasn't practical and was never used;
thus, parameters "TAU" and "DELAY" has been removed. - <a
href="http://code.google.com/p/mrpt/source/detail?r=3395" >r3395</a>
      - Methods removed since they weren't implemented in any derived
class and there are no plans for doing it.
        - mrpt::reactivenav::CReactiveNavigationSystem ::evaluate()
        - mrpt::reactivenav::CReactiveNavigationSystem ::setParams()
  - Build system:
    - Updated to nanoflann 1.1.7: ICP is ~5% faster.
    - More unit tests:
      - [mrpt-base] geometry module.
  - BUG FIXES:
    - CTimeLogger::registerUserMeasure() ignored the enable/disable state of
the logger - <a href="http://code.google.com/p/mrpt/source/detail?r=3382"
>r3382</a>
    - mrpt-srba: SEGFAULT in 32bit builds due to missing
 - <a
href="http://code.google.com/p/mrpt/source/detail?r=3429" >r3429</a>

 <br/>
 <hr>
 <a name="1.0.1">
  <h2>Version 1.0.1: Released 12-MAY-2013 (SVN 3370)  </h2></a>
  - Changes in apps:
    - <a href="http://www.mrpt.org/Application%3ARawLogViewer"
>RawLogViewer</a>:
      - Better description of the "too much memory used" warning while
loading large datasets.
    - <a href="http://www.mrpt.org/Application%3Arobotic-arm-kinematics"
>robotic-arm-kinematics</a>:
      - Now allows changing the orientation of the first DOF (X,Y,Z).
  - New classes:
    - [mrpt-hwdrivers]
      - mrpt::hwdrivers::CInterfaceNI845x: An interface for this USB
SPI/I2C data acquisition board.
      - mrpt::hwdrivers::CCANBusReader: A class to record CAN bus frames
with a CAN232 converter.
    - [mrpt-obs]
      - mrpt::obs::CObservationCANBusJ1939
  - New functions:
    - New opengl_stock objects:
      - mrpt::opengl::stock_objects::Hokuyo_URG()
      - mrpt::opengl::stock_objects::Hokuyo_UTM()
      - mrpt::opengl::stock_objects::Househam_Sprayer()
    - mrpt::math::saveEigenSparseTripletsToFile() - <a
href="http://code.google.com/p/mrpt/source/detail?r=3351" >r3351</a>
  - New examples:
      - gmrf_map_demo
  - Changes in classes:
    - [mrpt-maps]
      - mrpt::maps::COccupancyGridMap2D now also evalutes likelihoods for
sonar-like observations (mrpt::obs::CObservationRange), allowing particle-filter
localization with these sensors - <a
href="http://code.google.com/p/mrpt/source/detail?r=3330" >r3330</a>
      - New method
mrpt::slam::CRandomFieldGridMap2D::insertIndividualReading()
    - [mrpt-kinematics]
      - mrpt::kinematics::CKinematicChain: Now allows changing the
orientation of the first DOF (X,Y,Z).
  - Removed stuff:
    - Backwards-compatibility typedef mrpt::vision::TKLTFeatureStatus has
been removed. Replace with mrpt::vision::TFeatureTrackStatus
    - KLT-specific values for mrpt::vision::TFeatureTrackStatus has been
removed, since they were not used in detected features anyway.
  - Build system:
    - Fixed a potential build error if including FFMPEG's <time.h> instead
of the standard header - <a
href="http://code.google.com/p/mrpt/source/detail?r=3316" >r3316</a>
    - Fixed determination of GCC version for all GCC builds - <a
href="http://code.google.com/p/mrpt/source/detail?r=3324" >r3324</a>
    - Updated to Eigen 3.1.3 - <a
href="http://code.google.com/p/mrpt/source/detail?r=3349" >r3349</a>
    - Updated to nanoflann 1.1.5
  - BUG FIXES:
    - Unit tests "SchurTests" for mrpt-srba incorrectly reported errors due
to an improperly initialized reference to a local variable - <a
href="http://code.google.com/p/mrpt/source/detail?r=3318" >r3318</a>
    - Debian packages: added missing binary deps for libmrpt-dev  - <a
href="http://code.google.com/p/mrpt/source/detail?r=3335" >r3335</a>

 <hr>
 <a name="1.0.0">
  <h2>Version 1.0.0: Released 1-MAR-2013 (SVN 3287)  </h2></a>
  - <b>Most important changes:</b>
    - New library with a flexible implementation of Sparser Relative Bundle
Adjustment (RBA), as presented in ICRA 2013: <a href="http://www.mrpt.org/srba"
>mrpt-srba</a>.
    - New library for Plane-based Maps: <a
href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a> (also presented in ICRA
2013).
    - Some MRPT modules are now header-only libraries.
    - Support for a new Octomap metric map, via the octomap library. See
mrpt::maps::COctoMap and detailed changes below.
    - Support for importing/exporting point clouds in the standard LAS
format (Look for liblas below).
    - Better support for custom builds of MRPT (selective building of
individual apps and libs, etc.)
    - Ready for Visual Studio 2012 and GCC 4.7
    - From now on, MRPT is released under the "New BSD" license.
    - Many bug fixes.
  - <b>Detailed list of changes:</b>
    - New apps:
      - <a
href="http://www.mrpt.org/list-of-mrpt-apps/application-srba-slam"
>srba-slam</a>: A command-line frontend for the Relative Bundle Adjustment
engine in mrpt-srba.
      - <a
href="http://www.mrpt.org/list-of-mrpt-apps/application-holonomic-navigator-demo"
>holonomic-navigator-demo</a>
      - <a
href="http://www.mrpt.org/list-of-mrpt-apps/application-robotic-arm-kinematics"
>robotic-arm-kinematics</a>: A GUI for experimenting with Denavit-Hartenberg
parameters.
    - Changes in apps:
      - <a href="http://www.mrpt.org/Application%3Anavlog-viewer"
>navlog-viewer</a>:
        - Fixed some minor visualization errors.
      - <a href="http://www.mrpt.org/Application%3ARawLogViewer"
>RawLogViewer</a>:
        - Import sequence of images as rawlog: Didn't detect "png" file
extension as images - <a
href="http://code.google.com/p/mrpt/source/detail?r=2940" >r2940</a> - Closes <a
href="http://code.google.com/p/mrpt/issues/detail?id=34" >#34</a>
        - The GUI toolbar has been ported from wxWidget's ToolBar to
sets of wxCustomButton's to avoid visualization problems in wx 2.9.X - <a
href="http://code.google.com/p/mrpt/source/detail?r=2950" >r2950</a>
      - <a
href="http://www.mrpt.org/list-of-mrpt-apps/application-ReactiveNavigationDemo"
>ReactiveNavigationDemo</a>:
        - The default holonomic navigation method is now the VFF, since
after the last bug fixes and tunes it seems to work quite well.
      - <a href="http://www.mrpt.org/Application%3ASceneViewer"
>SceneViewer3D</a>:
        - The GUI toolbar has been ported from wxWidget's ToolBar to
sets of wxCustomButton's to avoid visualization problems in wx 2.9.X - <a
href="http://code.google.com/p/mrpt/source/detail?r=2952" >r2952</a>
        - Added a new menu: "File -> Import -> From LAS file..." - <a
href="http://code.google.com/p/mrpt/source/detail?r=3244" >r3244</a>
      - <a href="http://www.mrpt.org/Application%3Agrid-matching"
>grid-matching</a>: new argument "--aligner" to select aligner method - <a
href="http://code.google.com/p/mrpt/source/detail?r=3021" >r3021</a>
    - New classes:
      - [mrpt-base]
        - mrpt::math::MatrixBlockSparseCols, a templated column-indexed
efficient storage of block-sparse Jacobian or Hessian matrices, together with
other arbitrary information - <a
href="http://code.google.com/p/mrpt/source/detail?r=2995" >r2995</a>
        - mrpt::utils::ignored_copy_ptr<>
        - mrpt::system::CTimeLoggerEntry
      - [mrpt-obs]
        - mrpt::obs::CObservationWindSensor - <a
href="http://code.google.com/p/mrpt/source/detail?r=3050" >r3050</a>
      - [mrpt-maps]
        - mrpt::maps::COctoMap
      - [mrpt-opengl]
        - mrpt::opengl::COctoMapVoxels
    - Deleted classes:
      - [mrpt-vision]
        - CFeatureTracker_FAST and CFeatureTracker_PatchMatch have been
removed since they didn't work robustly. Replace with
mrpt::vision::CFeatureTracker_KL
    - New libraries:
      - [mrpt-kinematics] See mrpt::kinematics
      - [mrpt-pbmap] See <a href="group__mrpt__pbmap__grp.html"
>mrpt-pbmap</a>.
      - [mrpt-srba] See <a href="http://www.mrpt.org/srba" >mrpt-srba</a>.
    - Changes in libraries:
      - These libs are now header-only: <a
href="http://code.google.com/p/mrpt/source/detail?r=3035" >r3035</a>, <a
href="http://code.google.com/p/mrpt/source/detail?r=3045" >r3045</a>
        - [mrpt-bayes]
        - [mrpt-graphs]
        - [mrpt-graphslam]
      - Integration of the Octomap C++ library (new BSD License) by Kai M.
Wurm et al.: <a href="http://code.google.com/p/mrpt/source/detail?r=3081"
>r3081</a>, <a href="http://code.google.com/p/mrpt/source/detail?r=3083"
>r3083</a>, <a href="http://code.google.com/p/mrpt/source/detail?r=3084"
>r3084</a>, <a href="http://code.google.com/p/mrpt/source/detail?r=3086"
>r3086</a>, <a href="http://code.google.com/p/mrpt/source/detail?r=3087"
>r3087</a>, <a href="http://code.google.com/p/mrpt/source/detail?r=3088"
>r3088</a>, <a href="http://code.google.com/p/mrpt/source/detail?r=3093"
>r3093</a>
        - The main new classes are mrpt::maps::COctoMap &
mrpt::opengl::COctoMapVoxels
        - mrpt::maps::CMultiMetricMap now allows the seamless
integration of octomaps in many MRPT map building or localization algorithms.
        - New example: samples/octomap_simple
    - Changes in classes:
      - [mrpt-base]
        - Eigen::MatrixBase<Derived>::loadFromTextFile(), and all MRPT
derived matrix classes, are now much faster loading huge matrices from text
files - <a href="http://code.google.com/p/mrpt/source/detail?r=2997" >r2997</a>
        - The typedef Eigen::MatrixBase<Derived>::typename of MRPT's
plugin to Eigen classes has been REMOVED, to avoid conflicts with some part of
Eigen's sparse classes. Use Matrix::Scalar instead - <a
href="http://code.google.com/p/mrpt/source/detail?r=3065" >r3065</a>
        - New method mrpt::poses::CPose3DQuat::inverse()
        - New methods mrpt::poses::SE_traits::pseudo_exp()
        - mrpt::system::CTimeLogger:
          - New method mrpt::system::CTimeLogger::getStats() for
programatic execution time stats analysis - <a
href="http://code.google.com/p/mrpt/source/detail?r=2998" >r2998</a>
          - New method
mrpt::system::CTimeLogger::registerUserMeasure() for making stats of
user-providen values - <a
href="http://code.google.com/p/mrpt/source/detail?r=3005" >r3005</a>
        - mrpt::utils::map_as_vector<> can be now customized to use
different underlying STL containers for storage - <a
href="http://code.google.com/p/mrpt/source/detail?r=3001" >r3001</a>
        - mrpt::containers::CDynamicGrid::setSize() now also accepts a
"fill_value" argument.
        - Added method mrpt::math::TPoint2D::norm() for consistency with
mrpt::math::TPoint3D
        - Better support for saving (and not only loading) plain text
configuration files, including commented files with default values of all
existing parameters: - <a
href="http://code.google.com/p/mrpt/source/detail?r=2954" >r2954</a>
          - All mrpt::config::CConfigFileBase::write() now have an
extended signature for formatting.
          -
mrpt::config::CLoadableOptions::dumpToTextStreamstd::ostream::Seek() now
supports files larger than 2GB by using uint64_t instead of long (still see
issue report for another patch required for MSVC2010) - (Closes <a
href="http://code.google.com/p/mrpt/issues/detail?id=39" >issue 39</a>, thanks
Robert Schattschneider) - <a
href="http://code.google.com/p/mrpt/source/detail?r=3042" >r3042</a>
        - mrpt::typemeta::TTypeName<> moved to its own header
<mrpt/typemeta/TTypeName.h> while refactoring
<mrpt/serialization/CSerializable.h>
- <a href="http://code.google.com/p/mrpt/source/detail?r=3044" >r3044</a>
        - mrpt::config::CConfigFileBase::write() now has signatures for
"uint32_t" and "uint64_t" in both 32 and 64bit builds, instead of relying of the
"size_t" type. This was done to fix build errors in some GCC versions under
32bits.
        - mrpt::poses::CPose2D now caches the cos() and sin() of phi,
with a huge performance improvement in most common operations.
      - [mrpt-bayes]
        - mrpt::bayes::CKalmanFilterCapable (and all EKF-SLAM methods
based on it) are now much faster. The implementation now exploits the sparsity
of the Jacobian (~25% faster in a test 6D EKF-SLAM dataset) - <a
href="http://code.google.com/p/mrpt/source/detail?r=3059" >r3059</a>, <a
href="http://code.google.com/p/mrpt/source/detail?r=3060" >r3060</a>, <a
href="http://code.google.com/p/mrpt/source/detail?r=3061" >r3061</a>
        - mrpt::bayes::CParticleFilterCapable now makes use of the
Curiously Recurring Template Pattern (CRTP) design instead of ugly #define
macros - <a href="http://code.google.com/p/mrpt/source/detail?r=3182" >r3182</a>
      - [mrpt-graphs]
        - mrpt::graphs::CNetworkOfPoses2D,
mrpt::graphs::CNetworkOfPoses3D,... and so on, are now all typedef's instead of
classes, since serialization is now implemented as pure templatized code, thus
avoiding the need to declare derived auxiliary classes  - <a
href="http://code.google.com/p/mrpt/source/detail?r=3044" >r3044</a>
      - [mrpt-gui]
        - mrpt::gui::CDisplayWindow3D::addTextMessage() (and other
opengl text routines) now allows drawing text with a shadow effect - <a
href="http://code.google.com/p/mrpt/source/detail?r=3007" >r3007</a>
      - [mrpt-hwdrivers]
        - New method
mrpt::hwdrivers::CActivMediaRobotBase::areMotorsEnabled()
        - mrpt::hwdrivers::CGenericSensor (and all derived classes) now
allocate objects aligned in memory with
        - New static method mrpt::hwdrivers::CGPSInterface::parse_NMEA()
      - [mrpt-maps]
        - Better integration of point cloud classes with PCL: - <a
href="http://code.google.com/p/mrpt/source/detail?r=2943" >r2943</a>
          - mrpt::maps::CPointsMap::loadPCDFile()
          - mrpt::maps::CPointsMap::setFromPCLPointCloud()
          - mrpt::maps::CColouredPointsMap::setFromPCLPointCloudRGB()
        - Point cloud loading & saving in the standard ASPRS LiDAR LAS
format (if liblas is installed in the system, see http://www.liblas.org/ ). See
also the ready-to-use import menu in SceneViewer3D - <a
href="http://code.google.com/p/mrpt/source/detail?r=3244" >r3244</a>
          - mrpt::maps::CPointsMap::loadLASFile()
          - mrpt::maps::CPointsMap::saveLASFile()
        - Integration of wind measurements in gas-concentration maps (by
Javier G. Monroy) - <a href="http://code.google.com/p/mrpt/source/detail?r=3050"
>r3050</a>
      - [mrpt-obs]
        - New method mrpt::obs::CObservationGPS::clear()
      - [mrpt-opengl]
        - Evaluation of bounding box of opengl objects. New methods: -
<a href="http://code.google.com/p/mrpt/source/detail?r=3026" >r3026</a>
          - mrpt::opengl::CRenderizable::getBoundingBox()
          - mrpt::opengl::COpenGLScene::getBoundingBox()
          - mrpt::opengl::COpenGLViewport::getBoundingBox()
        -
mrpt::opengl::COctreePointRenderer::octree_get_graphics_boundingboxes() has a
new flag to draw solid boxes at each leaf node - <a
href="http://code.google.com/p/mrpt/source/detail?r=3033" >r3033</a>
        - mrpt::opengl::COpenGLViewport has a new set of "global OpenGL
switches" that affect the rendering of entire scenes - <a
href="http://code.google.com/p/mrpt/source/detail?r=3185" >r3185</a>
        - Classes drawing lines now by default enable anti-aliasing (can
be disabled by the programmer): - <a
href="http://code.google.com/p/mrpt/source/detail?r=3185" >r3185</a>
          - mrpt::opengl::CGridPlaneXY, mrpt::opengl::CGridPlaneXZ
          - mrpt::opengl::CSimpleLine
          - mrpt::opengl::CSetOfLines
      - [mrpt-reactivenav]
        - Much code of mrpt::reactivenav classes have undergone a
clean-up, slight optimizations and a translation of old Spanish names/comments
to English - <a href="http://code.google.com/p/mrpt/source/detail?r=2939"
>r2939</a>, <a href="http://code.google.com/p/mrpt/source/detail?r=2942"
>r2942</a>, <a href="http://code.google.com/p/mrpt/source/detail?r=2958"
>r2958</a>, <a href="http://code.google.com/p/mrpt/source/detail?r=3091"
>r3091</a>
        -
mrpt::reactivenav::CParameterizedTrajectoryGenerator::CCollisionGrid now has a
more maintainable binary serialization format - <a
href="http://code.google.com/p/mrpt/source/detail?r=2939" >r2939</a>
        -
mrpt::reactivenav::CParameterizedTrajectoryGenerator::debugDumpInFiles() now
also saves text files which can be used to visualize PTGs from MATLAB (see
scripts/viewPTG.m) - <a
href="http://code.google.com/p/mrpt/source/detail?r=3009" >r3009</a>
        - mrpt::reactivenav::CHolonomicVFF and
mrpt::reactivenav::CHolonomicND now have more configurable parameters, loadable
from config files. See their documentation.
        - Repulsive forces from obstacles in
mrpt::reactivenav::CHolonomicVFF are now automatically normalized wrt the
density of the 360deg view of obstacles and forces follow a "1/range" law
instead of the old "exp(-range)".
        - Solved a stability issue in C-S paths, in
mrpt::reactivenav::CPTG_DiffDrive_CS (By Mariano Jaimez Tarifa) - <a
href="http://code.google.com/p/mrpt/source/detail?r=3085" >r3085</a>
      - [mrpt-scanmatching]
        - mrpt::scanmatching::robustRigidTransformation():
          - Changed behavior not to allow features to appear in
duplicated pairings.
          - Added a consistency test to avoid seeding RANSAC with an
inconsistent initial model.
      - [mrpt-slam]
        - mrpt::slam::CMetricMapBuilderICP now does not integrate the
small pose changes due to odometry and/or relocalization when considering the
distance and angle thresholds. This means that fewer map updates are now done
for the same ICP-SLAM parameters, which should lead to "less noisy" maps.
    - New functions:
      - [mrpt-base]
        - mrpt::utils::abs_diff()
        - mrpt::system::getMRPTLicense()
        - mrpt::system::getFileModificationTime()
        - mrpt::math::noncentralChi2PDF_CDF() is now exposed (was
private)
        - mrpt::utils::sprintf_container()
        - mrpt::poses::operator -(mrpt::poses::CPose3DQuat)
        - max3() and min3() moved from the global namespace to
mrpt::utils::max3() and mrpt::utils::min3()
    - New examples:
      - octomap_simple
      - ransac-data-association
    - Build system:
      - Update to nanoflann 1.1.4 - <a
href="http://code.google.com/p/mrpt/source/detail?r=2937" >r2937</a>, <a
href="http://code.google.com/p/mrpt/source/detail?r=3017" >r3017</a>
      - Update to Eigen 3.1.2 - <a
href="http://code.google.com/p/mrpt/source/detail?r=3064" >r3064</a>
      - MRPT's root "CMakeLists.txt" has undergone a big refactoring and
cleanup - <a href="http://code.google.com/p/mrpt/source/detail?r=2961"
>r2961</a>
      - Backward compatible "mrpt-core" has been removed as a fake lib for
which to search with CMake from user programs - <a
href="http://code.google.com/p/mrpt/source/detail?r=2961" >r2961</a>
      - More system libs are detected in Linux (libclang-dev, lib3ds-dev),
discarding embedded versions then - <a
href="http://code.google.com/p/mrpt/source/detail?r=2963" >r2963</a> - <a
href="http://code.google.com/p/mrpt/issues/detail?id=17" >Closes #17</a>
      - Automatic detection of supported SIMD extensions (SSE*) from CMake
(only for Linux OS) - <a
href="http://code.google.com/p/mrpt/source/detail?r=3013" >r3013</a>
      - Fixed building with Visual Studio 2012 (MSVC11) - <a
href="http://code.google.com/p/mrpt/source/detail?r=3017" >r3017</a>
      - MRPT now allows defining header-only libraries with the
define_mrpt_lib_header_only() macro - <a
href="http://code.google.com/p/mrpt/source/detail?r=3034" >r3034</a>, <a
href="http://code.google.com/p/mrpt/source/detail?r=3035" >r3035</a>
      - More unit tests:
        - for all probability distribution functions in mrpt::math,
        - for the parser in mrpt::hwdrivers::CGPSInterface::parse_NMEA()
        - for the octomap map
        - for serialization/deserealization of many classes.
      - Added new documentation page: <a href="env-vars.html" >environment
variables</a>.
      - Removed the build flag "MRPT_BACKCOMPATIB_08X".
      - Fixes for building under Mac OSX: <a
href="http://code.google.com/p/mrpt/source/detail?r=3181" >r3181</a>
      - Enable some c++11 features if the compiler supports them - <a
href="http://code.google.com/p/mrpt/source/detail?r=3273" >r3273</a>
    - BUG FIXES:
      - Build: Fixed detection of OpenCV 2.4.2+ installed in the system
via CMake config file instead of pkg-config, which seems to be broken. - <a
href="http://code.google.com/p/mrpt/source/detail?r=3019" >r3019</a>
      - [mrpt-base] The iterator returned by end() in all MRPT vectors and
matrices (based on Eigen) pointed to the last element, not to the (now correct)
next position after the last element - <a
href="http://code.google.com/p/mrpt/source/detail?r=2941" >r2941</a>
      - [mrpt-base] mrpt::dynamicsize_vector::resize() performed a memory
reallocation even if given the current size, due to an inherited behavior from
Eigen. It is not the expected behavior, so it has been fixed. - <a
href="http://code.google.com/p/mrpt/source/detail?r=3003" >r3003</a>
      - [mrpt-base] Wrong computation of normPDF() values for the
multidimensional cases. Closes <a
href="http://code.google.com/p/mrpt/issues/detail?id=46" >#46</a> - <a
href="http://code.google.com/p/mrpt/source/detail?r=3068" >r3068</a>
      - [mrpt-base] mrpt::poses::CPoint::asString() confused the 2D and 3D
cases (Thanks Cipri!)
      - [mrpt-base] Fixed errors in de-serialization of
mrpt::utils::CPointPDFSOG and mrpt::maps::CReflectivityGridMap2D
      - [mrpt-base] mrpt::math::KDTreeCapable::kdTreeRadiusSearch2D()
always returned 0 matched.
      - [mrpt-graphs] Fixed bug in RecursiveSpectralPartition (Thanks to
Edu!) - <a href="http://code.google.com/p/mrpt/source/detail?r=3026" >r3026</a>
      - [mrpt-hwdrivers] Fixed potential SEGFAULT in
mrpt::hwdrivers::CGPSInterface (Thanks K.Miyawaki for <a
href="http://www.mrpt.org/node/2474" >reporting</a>)
      - [mrpt-hwdrivers] Fixed communications to LMS 1xx scanners (Thanks
Henry! See http://code.google.com/p/mrpt/issues/detail?id=49 )
      - [mrpt-maps] mrpt::maps::COccupancyGridMap2D::getAs3DObject()
returned cells with an occupancy of exactly "0" as transparent - <a
href="http://code.google.com/p/mrpt/source/detail?r=2957" >r2957</a>
      - [mrpt-maps] Fixed saving the correct point colors in
mrpt::maps::CColouredPointsMap::savePCDFile() (Thanks Mariano!) - <a
href="http://code.google.com/p/mrpt/source/detail?r=3090" >r3090</a>
      - [mrpt-maps] In CPointsMap::computeMatchingWith3D. Fixed matching
two 3D point clouds as each correspondence was inserted twice into the output
vector. (By Paco) - <a href="http://code.google.com/p/mrpt/source/detail?r=3162"
>r3162</a>
      - [mrpt-opengl] Fixed a potential bug: after deserializing an object
based on a display-list (most of them), it won't update in the opengl view.
      - [mrpt-reactivenav] Class mrpt::reactivenav::CHolonomicVFF was not
exported in Windows DLL's (Thanks Mariano for noticing!).
      - [mrpt-reactivenav] Fixed wrong computation of obstacles force
fields in mrpt::reactivenav::CHolonomicVFF (Thanks Mariano for noticing!) - <a
href="http://code.google.com/p/mrpt/source/detail?r=2953" >r2953</a>
      - [mrpt-reactivenav] Precomputed collision grids could be loaded in
mrpt::reactivenav::CParameterizedTrajectoryGenerator even for different robot
parameters/shape: now it correctly detects such situations and recompute when
needed - <a href="http://code.google.com/p/mrpt/source/detail?r=2939" >r2939</a>
- Closes <a href="http://code.google.com/p/mrpt/issues/detail?id=33" >#33</a>
      - [mrpt-reactivenav] ND algorithm: Fixed bugs of "last gap is never
evaluated" and wrong composition of representative direction for some gaps (By
Mariano) - <a href="http://code.google.com/p/mrpt/source/detail?r=3056"
>r3056</a>


 <br>
 <hr>
 <a name="0.9.6">
  <h2>Version 0.9.6 - (Version 1.0.0-Release_Candidate_4): Released 30-MAY-2012
(SVN 2930) </h2></a>
  - New applications:
    - <a
href="http://www.mrpt.org/list-of-mrpt-apps/application-kinect-stereo-calibrate"
>kinect-stereo-calibrate</a>: A GUI tool for calibrating RGB+D and/or stereo
cameras, including live Kinect capturing.
  - Removed applications:
    - stereo-calib-gui: it's now superseded by kinect-stereo-gui. The old
command line tool is still useful, so it's still there as the example
"stereo-calib-opencv".
  - Changes in applications:
    - <a href="http://www.mrpt.org/list-of-mrpt-apps/application-icp-slam"
>icp-slam</a>:
      - Added a new option (SHOW_LASER_SCANS_3D in config files) to draw
laser scans in the live 3D view - <a
href="http://code.google.com/p/mrpt/source/detail?r=2881" >r2881</a>
    - <a
href="http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-edit"
>rawlog-edit</a>:
      - Operation "--camera-params" now also handles stereo observations.
      - New operation "--stereo-rectify" for batch rectifying datasets
with stereo images.
      - New operation "--rename-externals".
    - <a
href="http://www.mrpt.org/list-of-mrpt-apps/application-SceneViewer"
>SceneViewer3D</a>:
      - New menu for generating high-resolution renders of any scene
directly to imag files - <a
href="http://code.google.com/p/mrpt/source/detail?r=2775" >r2775</a>
      - Many new menus for selective selecting objects and applying
operations on them - <a
href="http://code.google.com/p/mrpt/source/detail?r=2776" >r2776</a>
    - stereo-calib-gui: Now generates a report with detailed and clear
results from stereo calibration and allows the user to change most parameters
interactively - <a href="http://code.google.com/p/mrpt/source/detail?r=2801"
>r2801</a>
    - <a
href="http://www.mrpt.org/list-of-mrpt-apps/application-kinect-3d-view"
>kinect-3d-view</a>: New key command: press '9' to grab selected snapshots to
disk  - <a href="http://code.google.com/p/mrpt/source/detail?r=2890" >r2890</a>
  - Kinect stuff:
    - [mrpt-hwdrivers]
      - mrpt::hwdrivers::CKinect now decodes Bayer color using OpenCV
instead of default freenect - <a
href="http://code.google.com/p/mrpt/source/detail?r=2721" >r2721</a>, <a
href="http://code.google.com/p/mrpt/source/detail?r=2762" >r2762</a>
      - mrpt::hwdrivers::CKinect no longer forces a horizontal tilt at
start up by default, what may be annoying (if required, set
"initial_tilt_angle") - <a
href="http://code.google.com/p/mrpt/source/detail?r=2722" >r2722</a>
      - mrpt::hwdrivers::CKinect now loads Kinect calibration files in a
format compatible with stereo cameras. See
http://www.mrpt.org/Kinect_calibration
    - [mrpt-obs]
      - New method mrpt::obs::CObservation3DRangeScan::convertTo2DScan()
allows simulating a "fake 2D laser scanner" from a Kinect. See the example:
http://www.mrpt.org/Example_Kinect_To_2D_laser_scan
    - [mrpt-vision]
      - New function mrpt::vision::checkerBoardStereoCalibration() to
calibrate stereo and RGB+D cameras. See also the program <a
href="http://www.mrpt.org/list-of-mrpt-apps/application-kinect-stereo-calibrate"
>kinect-stereo-calibrate</a>:
  - New classes:
    - [mrpt-gui]
      - New event generated by GUI windows:
mrpt::gui::mrptEventWindowClosed
    - [mrpt-hwdrivers]
      - mrpt::hwdrivers::CRaePID: A new interface to PID gas sensing
devices (by Emil Khatib, University of Malaga) - <a
href="http://code.google.com/p/mrpt/source/detail?r=2841" >r2841</a>
    - [mrpt-opengl]
      - New classes for representing confidence intervals (ellipsoids) in
transformed spaces - <a
href="http://code.google.com/p/mrpt/source/detail?r=2783" >r2783</a>
        - mrpt::opengl::CGeneralizedEllipsoidTemplate<>
        - mrpt::opengl::CEllipsoidRangeBearing2D
        - mrpt::opengl::CEllipsoidInverseDepth2D
        - mrpt::opengl::CEllipsoidInverseDepth3D
      - mrpt::opengl::CFrustum to easily render these geometric figures
      - New struct mrpt::opengl::TFontParams result of a code refactoring
    - [mrpt-vision]
      - mrpt::vision::TSIFTDescriptorsKDTreeIndex,
TSURFDescriptorsKDTreeIndex  - <a
href="http://code.google.com/p/mrpt/source/detail?r=2799" >2799</a>
      - mrpt::vision::CStereoRectifyMap - See tutorial online:
http://www.mrpt.org/Rectifying_stereo_

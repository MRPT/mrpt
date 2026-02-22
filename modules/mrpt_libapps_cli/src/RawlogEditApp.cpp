/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

// ===========================================================================
//  Program: rawlog-edit
//
//  Intention: A generic rawlog (dataset) files manipulation program,
//   much like the GUI program RawlogViewer but allowing command-line
//   operations.
//  See the "--help" output for list of supported operations and further
//   instructions.
//
//  About integration with bash/.BAT scripts: The program will return 0 upon
//   successful execution. To avoid any information display to stdout invoke
//   it with the -q (or --quiet) flag. Upon error, it will return -1.
//
//  Started: JLBC @ Jul-2010
// ===========================================================================

#include <mrpt/apps_cli/RawlogEditApp.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <memory>

#include "rawlog-edit-declarations.h"

using TOperationFunctor =
    void (*)(mrpt::io::CCompressedInputStream& in_rawlog, CLI::App& cmdline, bool verbose);

using namespace mrpt;
using namespace mrpt::img;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace std;
using namespace mrpt::io;

// Frwd. decl:
DECLARE_OP_FUNCTION(op_camera_params);
DECLARE_OP_FUNCTION(op_cut);
DECLARE_OP_FUNCTION(op_deexternalize);

DECLARE_OP_FUNCTION(op_describe);

DECLARE_OP_FUNCTION(op_export_gps_all);
DECLARE_OP_FUNCTION(op_export_gps_gas_kml);
DECLARE_OP_FUNCTION(op_export_gps_kml);
DECLARE_OP_FUNCTION(op_export_enose_txt);
DECLARE_OP_FUNCTION(op_export_gps_txt);
DECLARE_OP_FUNCTION(op_export_rawdaq_txt);

DECLARE_OP_FUNCTION(op_export_txt);
// op_export_txt is a generic replacement of all these:
DECLARE_OP_FUNCTION(op_export_2d_scans_txt);
DECLARE_OP_FUNCTION(op_export_anemometer_txt);
DECLARE_OP_FUNCTION(op_export_imu_txt);
DECLARE_OP_FUNCTION(op_export_odometry_txt);
// ^^^

DECLARE_OP_FUNCTION(op_externalize);
DECLARE_OP_FUNCTION(op_generate_3d_pointclouds);
DECLARE_OP_FUNCTION(op_info);
DECLARE_OP_FUNCTION(op_keep_label);
DECLARE_OP_FUNCTION(op_list_images);
DECLARE_OP_FUNCTION(op_list_poses);
DECLARE_OP_FUNCTION(op_list_rangebearing);
DECLARE_OP_FUNCTION(op_list_timestamps);
DECLARE_OP_FUNCTION(op_recalc_odometry);
DECLARE_OP_FUNCTION(op_remap_timestamps);
DECLARE_OP_FUNCTION(op_remove_label);
DECLARE_OP_FUNCTION(op_rename_externals);
DECLARE_OP_FUNCTION(op_sensors_pose);
DECLARE_OP_FUNCTION(op_stereo_rectify);
DECLARE_OP_FUNCTION(op_undistort);

// Declare the supported command line switches ===========
CLI::App cmd{"rawlog-edit"};

std::string val_input_file;
std::string val_output_file;
std::string val_plugins;
std::string val_outdir = ".";
std::string val_image_format = "png";
std::string val_externals_filename_fmt = "${type}_${label}_%.06%f";
bool val_txt_externals = false;
std::string val_img_size;
bool val_rectify_centers = false;
std::string val_out_text_file = "out.txt";
uint64_t val_from_index = 0;
uint64_t val_to_index = 0;
double val_from_time = 0;
double val_to_time = 0;
double val_odo_KL = 0;
double val_odo_KR = 0;
double val_odo_D = 0;
bool val_overwrite = false;
std::string val_select_label;
bool val_quiet = false;

// Storage for value-type operations:
std::string val_op_remap_timestamps;
std::string val_op_remove_label;
std::string val_op_keep_label;
std::string val_op_sensors_pose;
std::string val_op_camera_params;
std::string val_op_stereo_rectify;

void RawlogEditApp::run(int argc, const char** argv)
{
  cmd.set_version_flag(
      "--version", mrpt::format(
                       "%s - Sources timestamp: %s\n", MRPT_getVersion().c_str(),
                       MRPT_getCompilationDate().c_str()));

  // Global options:
  cmd.add_option("-i,--input", val_input_file, "Input dataset (required) (*.rawlog)")->required();
  cmd.add_option("-o,--output", val_output_file, "Output dataset (*.rawlog)");
  cmd.add_option(
      "-p,--plugins", val_plugins,
      "Single or comma-separated list of .so/.dll plugins to load for additional "
      "user-supplied classes");
  cmd.add_option("--out-dir", val_outdir, "Output directory (used by some commands only)");
  cmd.add_option("--image-format", val_image_format, "External image format");
  cmd.add_option(
      "--externals-filename-format", val_externals_filename_fmt,
      "Format string for the command --rename-externals."
      "(Default: \"${type}_${label}_%.06%f\"). Refer to docs for "
      "mrpt::obs::format_externals_filename().");
  cmd.add_flag(
      "--txt-externals", val_txt_externals,
      "When externalizing CObservation3DRangeScan objects, switched from binary "
      "files (default) to plain text.");
  cmd.add_option("--image-size", val_img_size, "Resize output images");
  cmd.add_flag(
      "--rectify-centers-coincide", val_rectify_centers,
      "In stereo rectification, force that both image centers after coincide "
      "after rectifying.");
  cmd.add_option("--text-file-output", val_out_text_file, "Output for a text file");
  cmd.add_option("--from-index", val_from_index, "Starting index for --cut");
  cmd.add_option("--to-index", val_to_index, "End index for --cut");
  cmd.add_option(
      "--from-time", val_from_time,
      "Starting time for --cut, as UNIX timestamp, optionally with fractions of seconds.");
  cmd.add_option(
      "--to-time", val_to_time,
      "End time for --cut, as UNIX timestamp, optionally with fractions of seconds.");
  cmd.add_option(
      "--odo-KL", val_odo_KL,
      "Constant from encoder ticks to meters (left wheel), used in --recalc-odometry.");
  cmd.add_option(
      "--odo-KR", val_odo_KR,
      "Constant from encoder ticks to meters (right wheel), used in --recalc-odometry.");
  cmd.add_option(
      "--odo-D", val_odo_D,
      "Distance between left-right wheels (meters), used in --recalc-odometry.");
  cmd.add_flag("-w,--overwrite", val_overwrite, "Force overwrite target file without prompting.");
  cmd.add_option(
      "--select-label", val_select_label,
      "Select one sensor label on which to apply the operation.\n"
      "Several labels can be provided separated by commas.\n"
      "Only for those ops that mention --select-label as optional.");
  cmd.add_flag("-q,--quiet", val_quiet, "Terse output");

  // --------------- List of possible operations ---------------
  struct OpEntry
  {
    std::string name;
    TOperationFunctor functor;
    CLI::Option* opt;
  };
  std::vector<OpEntry> ops;
  map<string, TOperationFunctor> ops_functors;

  auto addSwitchOp = [&](const std::string& name, TOperationFunctor func, const std::string& desc)
  {
    auto* opt = cmd.add_flag("--" + name, desc);
    ops.push_back({name, func, opt});
    ops_functors[name] = func;
  };

  auto addValueOp = [&](const std::string& name, std::string& storage, TOperationFunctor func,
                        const std::string& desc)
  {
    auto* opt = cmd.add_option("--" + name, storage, desc);
    ops.push_back({name, func, opt});
    ops_functors[name] = func;
  };

  addSwitchOp(
      "externalize", &op_externalize,
      "Op: convert to external storage.\n"
      "Requires: -o (or --output)\n"
      "Optional: --image-format, --txt-externals");

  addSwitchOp(
      "de-externalize", &op_deexternalize,
      "Op: the opposite that --externalize: generates a monolitic rawlog "
      "file with all external files integrated in one.\n"
      "Requires: -o (or --output)");

  addSwitchOp("info", &op_info, "Op: parse input file and dump information and statistics.");

  addSwitchOp(
      "list-images", &op_list_images,
      "Op: dump a list of all external image files in the dataset.\n"
      "Optionally the output text file can be changed with --text-file-output.");

  addSwitchOp(
      "list-poses", &op_list_poses,
      "Op: dump a list of all the poses of the observations in the dataset.\n"
      "Optionally the output text file can be changed with --text-file-output.");

  addSwitchOp(
      "list-timestamps", &op_list_timestamps,
      "Op: generates a list with all the observations' timestamp, "
      "sensor label and C++ class name.\n"
      "Optionally the output text file can be changed with --text-file-output.");

  addValueOp(
      "remap-timestamps", val_op_remap_timestamps, &op_remap_timestamps,
      "Op: Change all timestamps t replacing it with the linear map 'a*t+b'."
      "The parameters 'a' and 'b' must be given separated with a semicolon.\n"
      "Optional: --select-label LABEL1[,LABEL2] to limit the operation to "
      "those sensors only.\n"
      "Requires: -o (or --output)");

  addSwitchOp(
      "list-range-bearing", &op_list_rangebearing,
      "Op: dump a list of all landmark observations of type range-bearing.\n"
      "Optionally the output text file can be changed with --text-file-output.");

  addValueOp(
      "remove-label", val_op_remove_label, &op_remove_label,
      "Op: Remove all observation matching the given sensor label(s)."
      "Several labels can be provided separated by commas.\n"
      "Requires: -o (or --output)");

  addValueOp(
      "keep-label", val_op_keep_label, &op_keep_label,
      "Op: Remove all observations not matching the given sensor label(s)."
      "Several labels can be provided separated by commas.\n"
      "Requires: -o (or --output)");

  addSwitchOp(
      "export-gps-kml", &op_export_gps_kml,
      "Op: Export GPS paths to Google Earth KML files.\n"
      "Generates one .kml file with different sections for each "
      "different sensor label of GPS observations in the dataset. "
      "The generated .kml files will be saved in the same path than "
      "the input rawlog, with the same filename + each sensorLabel.");

  addSwitchOp(
      "export-gps-gas-kml", &op_export_gps_gas_kml,
      "Op: Export GPS paths to Google Earth KML files coloured by the gas concentration.\n"
      "Generates one .kml file with different sections for each "
      "different sensor label of GPS observations in the dataset. "
      "The generated .kml files will be saved in the same path than "
      "the input rawlog, with the same filename + each sensorLabel.");

  addSwitchOp(
      "export-gps-txt", &op_export_gps_txt,
      "Op: Export GPS GPGGA messages to TXT files.\n"
      "Generates one .txt file for each different sensor label of "
      "GPS observations in the dataset. "
      "The generated .txt files will be saved in the same path than "
      "the input rawlog, with the same filename + each sensorLabel.");

  addSwitchOp(
      "export-gps-all", &op_export_gps_all,
      "Op: Generic export all kinds of GPS/GNSS messages to separate TXT files.\n"
      "Generates one .txt file for each different sensor label and for each "
      "message type in the dataset, with a first header line describing each field.");

  addSwitchOp(
      "export-imu-txt", &op_export_imu_txt,
      "Op: Export IMU readings to TXT files.\n"
      "Generates one .txt file for each different sensor label of an "
      "IMU observation in the dataset. "
      "The generated .txt files will be saved in the same path than "
      "the input rawlog, with the same filename + each sensorLabel.");

  addSwitchOp(
      "export-odometry-txt", &op_export_odometry_txt,
      "Op: Export absolute odometry readings to TXT files.\n"
      "Generates one .txt file for each different sensor label of an "
      "odometry observation in the dataset. "
      "The generated .txt files will be saved in the same path than "
      "the input rawlog, with the same filename + each sensorLabel.");

  addSwitchOp(
      "export-enose-txt", &op_export_enose_txt,
      "Op: Export e-nose readigns to TXT files.\n"
      "Generates one .txt file for each different sensor label of an "
      "e-nose observation in the dataset. "
      "The generated .txt files will be saved in the same path than "
      "the input rawlog, with the same filename + each sensorLabel.");

  addSwitchOp(
      "export-anemometer-txt", &op_export_anemometer_txt,
      "Op: Export anemometer readigns to TXT files.\n"
      "Generates one .txt file for each different sensor label of an "
      "anemometer observation in the dataset. "
      "The generated .txt files will be saved in the same path than "
      "the input rawlog, with the same filename + each sensorLabel.");

  addSwitchOp(
      "recalc-odometry", &op_recalc_odometry,
      "Op: Recomputes odometry increments from new encoder-to-odometry constants.\n"
      "Requires: -o (or --output)\n"
      "Requires: --odo-KL, --odo-KR and --odo-D.");

  addSwitchOp(
      "export-rawdaq-txt", &op_export_rawdaq_txt,
      "Op: Export raw DAQ readings to TXT files.\n"
      "Generates one .txt file for each different sensor label + DAQ task. "
      "The generated .txt files will be saved in the same path than the input rawlog.");

  addSwitchOp(
      "export-txt", &op_export_txt,
      "Op: Generic export observations to TXT/CSV files.\n"
      "Generates one .txt file for each different sensor label of "
      "all observation classes that supports the export-to-txt API.\n"
      "The generated .txt files will be saved in the same path than "
      "the input rawlog, as `<rawlog_filename>_<sensorLabel>.txt`.");

  addSwitchOp(
      "export-2d-scans-txt", &op_export_2d_scans_txt,
      "Op: Export 2D scans to TXT files.\n"
      "Generates two .txt files for each different sensor label of "
      "2D scan observations, one with the timestamps and the other with range data.\n"
      "The generated .txt files will be saved in the same path than "
      "the input rawlog, with the same filename + each sensorLabel.");

  addSwitchOp(
      "cut", &op_cut,
      "Op: Cut a part of the input rawlog.\n"
      "Requires: -o (or --output)\n"
      "Requires: At least one of --from-index, --from-time, "
      "--to-index, --to-time. Use only one of the --from-* and --to-* at once.\n"
      "If only a --from-* is given, the rawlog will be saved up to its end. "
      "If only a --to-* is given, the rawlog will be saved from its beginning.");

  addSwitchOp(
      "generate-3d-pointclouds", &op_generate_3d_pointclouds,
      "Op: (re)generate the 3D pointclouds within "
      "CObservation3DRangeScan objects that have range data.\n"
      "Requires: -o (or --output)");

  addValueOp(
      "sensors-pose", val_op_sensors_pose, &op_sensors_pose,
      "Op: batch change the poses of sensors from a "
      "rawlog-grabber-like configuration file that specifies the "
      "pose of sensors by their sensorLabel names.\n"
      "Requires: -o (or --output)");

  addValueOp(
      "camera-params", val_op_camera_params, &op_camera_params,
      "Op: change the camera intrinsic parameters of all CObservationImage "
      "with the given SENSOR_LABEL, with new params loaded from the "
      "given file, section '[CAMERA_PARAMS]' "
      "for monocular cameras, or '[CAMERA_PARAMS_LEFT]' and "
      "'[CAMERA_PARAMS_RIGHT]' for CObservationStereoImage, "
      "or '[DEPTH_CAM_PARAMS]' and '[INTENSITY_CAM_PARAMS]' for "
      "CObservation3DRangeScan.\n"
      "Requires: -o (or --output)");

  addValueOp(
      "stereo-rectify", val_op_stereo_rectify, &op_stereo_rectify,
      "Op: creates a new set of external images for all "
      "CObservationStereoImages with the given SENSOR_LABEL, using "
      "the camera parameters stored in the "
      "observations (which must be a valid calibration) and with the "
      "given alpha value. Alpha can be -1 for auto, or otherwise be "
      "in the range [0,1] (see OpenCV's docs for cvStereoRectify).\n"
      "Requires: -o (or --output)\n"
      "Optional: --image-format to set image format (default=jpg), \n"
      "          --image-size to resize output images (example: --image-size 640x480)");

  addSwitchOp(
      "rename-externals", &op_rename_externals,
      "Op: Renames all the external storage file names within the "
      "rawlog (it doesn't change the external files, which may even not exist).");

  addSwitchOp("undistort", &op_undistort, "Op: Undistort all images in the rawlog.");

  addSwitchOp(
      "describe", &op_describe,
      "Op: Prints a human-readable description for *all* objects in the dataset.");

  // --------------- End of list of possible operations --------

  // Parse arguments:
  try
  {
    cmd.parse(argc, argv);
  }
  catch (const CLI::ParseError& e)
  {
    int ret = cmd.exit(e);
    if (ret == 0) return;  // --help or --version
    throw std::runtime_error("CLI argument parsing error.");
  }

  // sanity check: one and only one operation is allowed:
  unsigned int nOps = 0;
  for (auto& op : ops)
    if (op.opt->count() > 0) nOps++;
  if (nOps != 1)
  {
    THROW_EXCEPTION_FMT(
        "One and only one operation must be selected by command line "
        "arguments, but %u provided. Use --help for further details.",
        nOps);
  }

  string input_rawlog = val_input_file;
  const bool verbose = !val_quiet;

  // Plugins:
  if (!val_plugins.empty()) mrpt::system::loadPluginModules(val_plugins);

  // Check the selected operation:
  string selected_op;
  for (auto& op : ops)
    if (op.opt->count() > 0)
    {
      selected_op = op.name;
      break;
    }

  ASSERTMSG_(!selected_op.empty(), "Internal error: no operation selected.");

  VERBOSE_COUT << "Operation to perform: " << selected_op << "\n";

  // This will be done for any operation: open the input rawlog
  // ------------------------------------------------------------
  if (!mrpt::system::fileExists(input_rawlog))
    throw runtime_error(format("Input file doesn't exist: '%s'", input_rawlog.c_str()));

  // Open input rawlog:
  CCompressedInputStream fil_input;
  VERBOSE_COUT << "Opening '" << input_rawlog << "'...\n";
  fil_input.open(input_rawlog);
  VERBOSE_COUT << "Open OK.\n";

  // External storage directory?
  CImage::setImagesPathBase(CRawlog::detectImagesDirectory(input_rawlog));
  if (mrpt::system::directoryExists(CImage::getImagesPathBase()))
  {
    VERBOSE_COUT << "Found external storage directory: " << CImage::getImagesPathBase() << "\n";
  }
  else
  {
    VERBOSE_COUT << "Warning: No external storage directory was found "
                    "(not an issue if the rawlog does not contain "
                    "delayed-load images).\n";
  }

  // ------------------------------------
  //  EXECUTE THE REQUESTED OPERATION
  // ------------------------------------
  ASSERTMSG_(
      ops_functors.find(selected_op) != ops_functors.end(),
      "Internal error: Unknown operation functor!");

  // Call the selected functor:
  ops_functors[selected_op](fil_input, cmd, verbose);

  // successful end of program.
}

// ======================================================================
//   See TOutputRawlogCreator declaration
// ======================================================================
TOutputRawlogCreator::TOutputRawlogCreator()
{
  if (val_output_file.empty())
    throw runtime_error(
        "This operation requires an output file. Use '-o file' or "
        "'--output file'.");

  out_rawlog_filename = val_output_file;
  if (fileExists(out_rawlog_filename) && !val_overwrite)
    throw runtime_error(
        string("*ABORTING*: Output file already exists: ") + out_rawlog_filename +
        string("\n. Select a different output path, remove the file or "
               "force overwrite with '-w' or '--overwrite'."));

  if (!out_rawlog_io.open(out_rawlog_filename))
    throw runtime_error(string("*ABORTING*: Cannot open output file: ") + out_rawlog_filename);
  out_rawlog =
      std::make_unique<mrpt::serialization::CArchiveStreamBase<mrpt::io::CCompressedOutputStream>>(
          out_rawlog_io);
}

bool isFlagSet(CLI::App& app, const std::string& arg_name)
{
  auto* opt = app.get_option_no_throw("--" + arg_name);
  return opt != nullptr && opt->count() > 0;
}

template <typename T>
bool getArgValue(CLI::App& app, const std::string& arg_name, T& out_val)
{
  auto* opt = app.get_option_no_throw("--" + arg_name);
  if (!opt || opt->count() == 0) return false;
  out_val = opt->as<T>();
  return true;
}

// Explicit instantations:
template bool getArgValue<>(CLI::App& app, const std::string& arg_name, std::string& out_val);
template bool getArgValue<>(CLI::App& app, const std::string& arg_name, double& out_val);
template bool getArgValue<>(CLI::App& app, const std::string& arg_name, size_t& out_val);
template bool getArgValue<>(CLI::App& app, const std::string& arg_name, int& out_val);

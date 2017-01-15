/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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

#include "rawlog-edit-declarations.h"

#include <mrpt/otherlibs/tclap/CmdLine.h>

typedef void (*TOperationFunctor)(mrpt::utils::CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose);


using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;

// Frwd. decl:
DECLARE_OP_FUNCTION(op_externalize);
DECLARE_OP_FUNCTION(op_info);
DECLARE_OP_FUNCTION(op_list_images);
DECLARE_OP_FUNCTION(op_list_poses);
DECLARE_OP_FUNCTION(op_list_rangebearing);
DECLARE_OP_FUNCTION(op_remove_label);
DECLARE_OP_FUNCTION(op_keep_label);
DECLARE_OP_FUNCTION(op_cut);
DECLARE_OP_FUNCTION(op_export_gps_kml);
DECLARE_OP_FUNCTION(op_export_gps_gas_kml);
DECLARE_OP_FUNCTION(op_export_gps_txt);
DECLARE_OP_FUNCTION(op_export_gps_all);
DECLARE_OP_FUNCTION(op_export_imu_txt);
DECLARE_OP_FUNCTION(op_export_odometry_txt);
DECLARE_OP_FUNCTION(op_export_enose_txt);
DECLARE_OP_FUNCTION(op_export_anemometer_txt);
DECLARE_OP_FUNCTION(op_recalc_odometry);
DECLARE_OP_FUNCTION(op_export_rawdaq_txt);
DECLARE_OP_FUNCTION(op_export_2d_scans_txt);
DECLARE_OP_FUNCTION(op_sensors_pose);
DECLARE_OP_FUNCTION(op_camera_params);
DECLARE_OP_FUNCTION(op_generate_3d_pointclouds);
DECLARE_OP_FUNCTION(op_generate_pcd);
DECLARE_OP_FUNCTION(op_stereo_rectify);
DECLARE_OP_FUNCTION(op_rename_externals);
DECLARE_OP_FUNCTION(op_list_timestamps);
DECLARE_OP_FUNCTION(op_remap_timestamps);

// Declare the supported command line switches ===========
TCLAP::CmdLine cmd("rawlog-edit", ' ', mrpt::format("%s - Sources timestamp: %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str()) );

TCLAP::ValueArg<std::string> arg_input_file ("i","input","Input dataset (required) (*.rawlog)",true,"","dataset.rawlog",cmd);
TCLAP::ValueArg<std::string> arg_output_file("o","output","Output dataset (*.rawlog)",false,"","dataset_out.rawlog",cmd);

TCLAP::ValueArg<std::string> arg_outdir ("","out-dir","Output directory (used by some commands only)",false,".",".",cmd);

TCLAP::ValueArg<std::string> arg_external_img_extension("","image-format","External image format",false,"jpg","jpg,png,pgm,...",cmd);
TCLAP::SwitchArg             arg_txt_externals("","txt-externals","When externalizing CObservation3DRangeScan objects, switched from binary files (default) to plain text.",cmd, false);
TCLAP::ValueArg<std::string> arg_img_size("","image-size","Resize output images",false,"","COLSxROWS",cmd);
TCLAP::SwitchArg             arg_rectify_centers("","rectify-centers-coincide","In stereo rectification, force that both image centers after coincide after rectifying.",cmd, false);

TCLAP::ValueArg<std::string> arg_out_text_file("","text-file-output","Output for a text file",false,"out.txt","out.txt",cmd);

TCLAP::ValueArg<uint64_t> arg_from_index("","from-index","Starting index for --cut",false,0,"N0",cmd);
TCLAP::ValueArg<uint64_t> arg_to_index  ("","to-index",  "End index for --cut",false,0,"N1",cmd);

TCLAP::ValueArg<double> arg_from_time("","from-time","Starting time for --cut, as UNIX timestamp, optionally with fractions of seconds.",false,0,"T0",cmd);
TCLAP::ValueArg<double> arg_to_time  ("","to-time",  "End time for --cut, as UNIX timestamp, optionally with fractions of seconds.",false,0,"T1",cmd);

TCLAP::ValueArg<double> arg_odo_KL  ("","odo-KL",  "Constant from encoder ticks to meters (left wheel), used in --recalc-odometry.",false,0,"KL",cmd);
TCLAP::ValueArg<double> arg_odo_KR  ("","odo-KR",  "Constant from encoder ticks to meters (right wheel), used in --recalc-odometry.",false,0,"KR",cmd);
TCLAP::ValueArg<double> arg_odo_D   ("","odo-D",   "Distance between left-right wheels (meters), used in --recalc-odometry.",false,0,"D",cmd);

TCLAP::SwitchArg arg_overwrite("w","overwrite","Force overwrite target file without prompting.",cmd, false);

TCLAP::SwitchArg arg_quiet("q","quiet","Terse output",cmd, false);



// ======================================================================
//     main() of rawlog-edit
// ======================================================================
int main(int argc, char **argv)
{
	vector<TCLAP::Arg*> arg_ops;  // to be destroyed on exit.
	int ret_val = 0;

	try
	{
		// --------------- List of possible operations ---------------
		map<string,TOperationFunctor>  ops_functors;

		arg_ops.push_back(new TCLAP::SwitchArg("","externalize",
			"Op: convert to external storage.\n"
			"Requires: -o (or --output)\n"
			"Optional: --image-format, --txt-externals",cmd, false) );
		ops_functors["externalize"] = &op_externalize;

		arg_ops.push_back(new TCLAP::SwitchArg("","info",
			"Op: parse input file and dump information and statistics.",cmd, false) );
		ops_functors["info"] = &op_info;

		arg_ops.push_back(new TCLAP::SwitchArg("","list-images",
			"Op: dump a list of all external image files in the dataset.\n"
			"Optionally the output text file can be changed with --text-file-output."
			,cmd, false) );
		ops_functors["list-images"] = &op_list_images;

		arg_ops.push_back(new TCLAP::SwitchArg("","list-poses",
			"Op: dump a list of all the poses of the observations in the dataset.\n"
			"Optionally the output text file can be changed with --text-file-output."
			,cmd, false) );
		ops_functors["list-poses"] = &op_list_poses;

		arg_ops.push_back(new TCLAP::SwitchArg("","list-timestamps",
			"Op: generates a list with all the observations' timestamp, sensor label and C++ class name.\n"
			"Optionally the output text file can be changed with --text-file-output."
			,cmd, false) );
		ops_functors["list-timestamps"] = &op_list_timestamps;

		arg_ops.push_back(new TCLAP::ValueArg<std::string>("","remap-timestamps",
			"Op: Change all timestamps t replacing it with the linear map 'a*t+b'."
			"The parameters 'a' and 'b' must be given separated with a semicolon.\n"
			"Requires: -o (or --output)",false,"","a;b",cmd) );
		ops_functors["remap-timestamps"] = &op_remap_timestamps;

		arg_ops.push_back(new TCLAP::SwitchArg("","list-range-bearing",
			"Op: dump a list of all landmark observations of type range-bearing.\n"
			"Optionally the output text file can be changed with --text-file-output."
			,cmd, false) );
		ops_functors["list-range-bearing"] = &op_list_rangebearing;

		arg_ops.push_back(new TCLAP::ValueArg<std::string>("","remove-label",
			"Op: Remove all observation matching the given sensor label(s)."
			"Several labels can be provided separated by commas.\n"
			"Requires: -o (or --output)",false,"","label[,label...]",cmd) );
		ops_functors["remove-label"] = &op_remove_label;

		arg_ops.push_back(new TCLAP::ValueArg<std::string>("","keep-label",
			"Op: Remove all observations not matching the given sensor label(s)."
			"Several labels can be provided separated by commas.\n"
			"Requires: -o (or --output)",false,"","label[,label...]",cmd) );
		ops_functors["keep-label"] = &op_keep_label;

		arg_ops.push_back(new TCLAP::SwitchArg("","export-gps-kml",
			"Op: Export GPS paths to Google Earth KML files.\n"
			"Generates one .kml file with different sections for each different sensor label of GPS observations in the dataset. "
			"The generated .kml files will be saved in the same path than the input rawlog, with the same "
			"filename + each sensorLabel."
			,cmd,false) );
		ops_functors["export-gps-kml"] = &op_export_gps_kml;

		arg_ops.push_back(new TCLAP::SwitchArg("","export-gps-gas-kml",
			"Op: Export GPS paths to Google Earth KML files coloured by the gas concentration.\n"
			"Generates one .kml file with different sections for each different sensor label of GPS observations in the dataset. "
			"The generated .kml files will be saved in the same path than the input rawlog, with the same "
			"filename + each sensorLabel."
			,cmd,false) );
		ops_functors["export-gps-gas-kml"] = &op_export_gps_gas_kml;

		arg_ops.push_back(new TCLAP::SwitchArg("","export-gps-txt",
			"Op: Export GPS GPGGA messages to TXT files.\n"
			"Generates one .txt file for each different sensor label of GPS observations in the dataset. "
			"The generated .txt files will be saved in the same path than the input rawlog, with the same "
			"filename + each sensorLabel."
			,cmd,false) );
		ops_functors["export-gps-txt"] = &op_export_gps_txt;

		arg_ops.push_back(new TCLAP::SwitchArg("","export-gps-all",
			"Op: Generic export all kinds of GPS/GNSS messages to separate TXT files.\n"
			"Generates one .txt file for each different sensor label and for each "
			"message type in the dataset, with a first header line describing each field."
			,cmd,false) );
		ops_functors["export-gps-all"] = &op_export_gps_all;

		arg_ops.push_back(new TCLAP::SwitchArg("","export-imu-txt",
			"Op: Export IMU readings to TXT files.\n"
			"Generates one .txt file for each different sensor label of an IMU observation in the dataset. "
			"The generated .txt files will be saved in the same path than the input rawlog, with the same "
			"filename + each sensorLabel."
			,cmd,false) );
		ops_functors["export-imu-txt"] = &op_export_imu_txt;

		arg_ops.push_back(new TCLAP::SwitchArg("","export-odometry-txt",
			"Op: Export absolute odometry readings to TXT files.\n"
			"Generates one .txt file for each different sensor label of an odometry observation in the dataset. "
			"The generated .txt files will be saved in the same path than the input rawlog, with the same "
			"filename + each sensorLabel."
			,cmd,false) );
		ops_functors["export-odometry-txt"] = &op_export_odometry_txt;

		arg_ops.push_back(new TCLAP::SwitchArg("", "export-enose-txt",
			"Op: Export e-nose readigns to TXT files.\n"
			"Generates one .txt file for each different sensor label of an e-nose observation in the dataset. "
			"The generated .txt files will be saved in the same path than the input rawlog, with the same "
			"filename + each sensorLabel."
			, cmd, false));
		ops_functors["export-enose-txt"] = &op_export_enose_txt;

		arg_ops.push_back(new TCLAP::SwitchArg("", "export-anemometer-txt",
			"Op: Export anemometer readigns to TXT files.\n"
			"Generates one .txt file for each different sensor label of an anemometer observation in the dataset. "
			"The generated .txt files will be saved in the same path than the input rawlog, with the same "
			"filename + each sensorLabel."
			, cmd, false));
		ops_functors["export-anemometer-txt"] = &op_export_anemometer_txt;

		arg_ops.push_back(new TCLAP::SwitchArg("","recalc-odometry",
			"Op: Recomputes odometry increments from new encoder-to-odometry constants.\n"
			"Requires: -o (or --output)\n"
			"Requires: --odo-KL, --odo-KR and --odo-D.\n"
			,cmd,false) );
		ops_functors["recalc-odometry"] = &op_recalc_odometry;


		arg_ops.push_back(new TCLAP::SwitchArg("","export-rawdaq-txt",
			"Op: Export raw DAQ readings to TXT files.\n"
			"Generates one .txt file for each different sensor label + DAQ task. "
			"The generated .txt files will be saved in the same path than the input rawlog."
			,cmd,false) );
		ops_functors["export-rawdaq-txt"] = &op_export_rawdaq_txt;
	
		arg_ops.push_back(new TCLAP::SwitchArg("","export-2d-scans-txt",
			"Op: Export 2D scans to TXT files.\n"
			"Generates two .txt files for each different sensor label of 2D scan observations, one with "
			"the timestamps and the other with range data.\n"
			"The generated .txt files will be saved in the same path than the input rawlog, with the same "
			"filename + each sensorLabel."
			,cmd,false) );
		ops_functors["export-2d-scans-txt"] = &op_export_2d_scans_txt;

		arg_ops.push_back(new TCLAP::SwitchArg("","cut",
			"Op: Cut a part of the input rawlog.\n"
			"Requires: -o (or --output)\n"
			"Requires: At least one of --from-index, --from-time, --to-index, --to-time. Use only one of the --from-* and --to-* at once.\n"
			"If only a --from-* is given, the rawlog will be saved up to its end. If only a --to-* is given, the rawlog will be saved from its beginning.\n"
			,cmd,false) );
		ops_functors["cut"] = &op_cut;

		arg_ops.push_back(new TCLAP::SwitchArg("","generate-3d-pointclouds",
			"Op: (re)generate the 3D pointclouds within CObservation3DRangeScan objects that have range data.\n"
			"Requires: -o (or --output)\n"
			,cmd,false));
		ops_functors["generate-3d-pointclouds"] = &op_generate_3d_pointclouds;

		arg_ops.push_back(new TCLAP::SwitchArg("","generate-pcd",
			"Op: Generate a PointCloud Library (PCL) PCD file with the point cloud for each sensor observation that can be converted into"
			" this representation: laser scans, 3D camera images, etc.\n"
			"Optional: --out-dir to change the output directory (default: \"./\")\n"
			,cmd,false));
		ops_functors["generate-pcd"] = &op_generate_pcd;

		arg_ops.push_back(new TCLAP::ValueArg<std::string>("","sensors-pose",
			"Op: batch change the poses of sensors from a rawlog-grabber-like configuration file that specifies the pose of sensors by their sensorLabel names.\n"
			"Requires: -o (or --output)\n",
			false,"","file.ini",cmd) );
		ops_functors["sensors-pose"] = &op_sensors_pose;

		arg_ops.push_back(new TCLAP::ValueArg<std::string>("","camera-params",
			"Op: change the camera parameters of all CObservationImage's with the given SENSOR_LABEL, with new params loaded from the given file, section '[CAMERA_PARAMS]' "
			"for monocular cameras, or '[CAMERA_PARAMS_LEFT]' and '[CAMERA_PARAMS_RIGHT]' for stereo.\n"
			"Requires: -o (or --output)\n"
			,false,"","SENSOR_LABEL,file.ini",cmd) );
		ops_functors["camera-params"] = &op_camera_params;

		arg_ops.push_back(new TCLAP::ValueArg<std::string>("","stereo-rectify",
			"Op: creates a new set of external images for all CObservationStereoImages with the given SENSOR_LABEL, using the camera parameters stored in the "
			"observations (which must be a valid calibration) and with the given alpha value. Alpha can be -1 for auto, or otherwise be in the range [0,1] (see OpenCV's docs for cvStereoRectify).\n"
			"Requires: -o (or --output)\n"
			"Optional: --image-format to set image format (default=jpg), \n"
			"          --image-size to resize output images (example: --image-size 640x480) \n"
			,false,"","SENSOR_LABEL,0.5",cmd) );
		ops_functors["stereo-rectify"] = &op_stereo_rectify;

		arg_ops.push_back(new TCLAP::SwitchArg("","rename-externals",
			"Op: Renames all the external storage file names within the rawlog (it doesn't change the external files, which may even not exist).\n"
			,cmd,false));
		ops_functors["rename-externals"] = &op_rename_externals;



		// --------------- End of list of possible operations --------

		// Parse arguments:
		if (!cmd.parse( argc, argv ))
			throw std::runtime_error(""); // should exit.

		string input_rawlog  = arg_input_file.getValue();
		const bool verbose = !arg_quiet.getValue();

		// Check the selected operation:
		//  Only one of the ops should be selected:
		string selected_op;
		for (size_t i=0;i<arg_ops.size();i++)
			if (arg_ops[i]->isSet())
			{
				if (selected_op.empty())
				{
					selected_op = arg_ops[i]->getName();
				}
				else	throw std::runtime_error(
					"Exactly one operation must be indicated on command line.\n"
					"Use --help to see the list of possible operations.");
			}

		if (selected_op.empty())
		{
			throw std::runtime_error(
				"Don't know what to do: No operation was indicated.\n"
				"Use --help to see the list of possible operations.");
		}

		VERBOSE_COUT << "Operation to perform: " << selected_op << endl;

		// This will be done for any operation: open the input rawlog
		// ------------------------------------------------------------
		if (!mrpt::system::fileExists(input_rawlog))
			throw runtime_error(format("Input file doesn't exist: '%s'",input_rawlog.c_str()));

		// Open input rawlog:
		CFileGZInputStream  fil_input;
		VERBOSE_COUT << "Opening '" << input_rawlog << "'...\n";
		fil_input.open(input_rawlog);
		VERBOSE_COUT << "Open OK.\n";

		// External storage directory?
		CImage::IMAGES_PATH_BASE = CRawlog::detectImagesDirectory(input_rawlog);
		if (mrpt::system::directoryExists(CImage::IMAGES_PATH_BASE)) {
			VERBOSE_COUT << "Found external storage directory: " << CImage::IMAGES_PATH_BASE << "\n";
		}
		else {
			VERBOSE_COUT << "Warning: No external storage directory was found (not an issue if the rawlog does not contain delayed-load images).\n";
		}


		// ------------------------------------
		//  EXECUTE THE REQUESTED OPERATION
		// ------------------------------------
		ASSERTMSG_(ops_functors.find(selected_op)!=ops_functors.end(), "Internal error: Unknown operation functor!")

		// Call the selected functor:
		ops_functors[selected_op](fil_input,cmd,verbose);

		// successful end of program.
		ret_val = 0;
	}
	catch(std::exception &e)
	{
		if (strlen(e.what())) std::cerr << e.what() << std::endl;
		ret_val = -1;
	}

	// Free mem:
	for (size_t i=0;i<arg_ops.size();i++)
		delete arg_ops[i];

	// end:
	return ret_val;
} // end of main()



// ======================================================================
//   See TOutputRawlogCreator declaration
// ======================================================================
TOutputRawlogCreator::TOutputRawlogCreator()
{
	if (!arg_output_file.isSet())
		throw runtime_error("This operation requires an output file. Use '-o file' or '--output file'.");

	out_rawlog_filename = arg_output_file.getValue();
	if (fileExists(out_rawlog_filename) && !arg_overwrite.getValue() )
		throw runtime_error(string("*ABORTING*: Output file already exists: ") + out_rawlog_filename + string("\n. Select a different output path, remove the file or force overwrite with '-w' or '--overwrite'.") );

	if (!out_rawlog.open(out_rawlog_filename))
		throw runtime_error(string("*ABORTING*: Cannot open output file: ") + out_rawlog_filename );
}

bool isFlagSet(TCLAP::CmdLine &cmdline, const std::string &arg_name)
{
	using namespace TCLAP;

	std::list<Arg*>& args = cmdline.getArgList();
	for (std::list<Arg*>::iterator it=args.begin();it!=args.end();++it)
		if ( (*it)->getName() == arg_name)
			return (*it)->isSet();
	return false;
}


template <typename T>
bool getArgValue(TCLAP::CmdLine &cmdline, const std::string &arg_name, T &out_val)
{
	using namespace TCLAP;

	std::list<Arg*>& args = cmdline.getArgList();
	for (std::list<Arg*>::iterator it=args.begin();it!=args.end();++it)
	{
		if ( (*it)->getName() == arg_name)
		{
			// Is it set? Return the default value anyway:
			TCLAP::ValueArg<T> *arg = static_cast<TCLAP::ValueArg<T> *>(*it);
			out_val = arg->getValue();
			return (*it)->isSet();
		}
	}
	return false;
}

// Explicit instantations:
template bool getArgValue<>(TCLAP::CmdLine &cmdline, const std::string &arg_name, std::string &out_val);
template bool getArgValue<>(TCLAP::CmdLine &cmdline, const std::string &arg_name, double &out_val);
template bool getArgValue<>(TCLAP::CmdLine &cmdline, const std::string &arg_name, size_t &out_val);
template bool getArgValue<>(TCLAP::CmdLine &cmdline, const std::string &arg_name, int &out_val);


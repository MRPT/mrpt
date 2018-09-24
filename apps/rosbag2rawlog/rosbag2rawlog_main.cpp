/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

// ===========================================================================
//  Program: rosbag2rawlog
//  Intention: Parse bag files, save
//             as a RawLog file, easily readable by MRPT C++ programs.
//
//  Started: JLBC @ Feb-2016
// ===========================================================================

#include <mrpt/system/filesystem.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/os.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/buffer.h>

using namespace mrpt;
//using namespace mrpt::obs;
using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::system;
using namespace std;

// Declare the supported command line switches ===========
TCLAP::CmdLine cmd("rosbag2rawlog", ' ', MRPT_getVersion().c_str());

TCLAP::UnlabeledMultiArg<std::string> arg_input_files(
	"bags", "Input bag files (required) (*.bag)", true, "log.bag", cmd);

TCLAP::ValueArg<std::string> arg_output_file(
	"o", "output", "Output dataset (*.rawlog)", true, "", "dataset_out.rawlog",
	cmd);

TCLAP::SwitchArg arg_overwrite(
	"w", "overwrite", "Force overwrite target file without prompting.", cmd,
	false);

TCLAP::ValueArg<std::string> arg_world_frame(
	"f", "frame", "World TF frame", true, "", "world",
	cmd);


class Transcriber
{
	public:
	Transcriber(std::string_view rootFrame) : m_rootFrame(rootFrame) {}
	mrpt::serialization::CSerializable::Ptr toMrpt(const rosbag::MessageInstance &rosmsg)
	{
		mrpt::serialization::CSerializable::Ptr ptr;
		auto data = rosmsg.getDataType();
		auto topic = rosmsg.getTopic();
		if(data == "tf2_msgs/TFMessage")
		{
			auto tfs = rosmsg.instantiate<tf2_msgs::TFMessage>();
			for(auto & tf : tfs->transforms)
			{
				bool isStatic = topic == "/tf_static";	
				try
				{
					m_tfBuffer.setTransform(tf, "bagfile", isStatic);
				}
				catch (tf2::TransformException& ex)
				{
					std::cerr << ex.what() << std::endl;
				}
			}
		}
		else if(data == "sensor_msgs/Image")
		{
			auto image = rosmsg.instantiate<sensor_msgs::Image>();
			auto cv_ptr = cv_bridge::toCvShare(image);
			
			MRPT_TODO("Create a config file to explicity map ros topics to MRPT types");

			//For now we are just assuming this is a range image
			if(cv_ptr->encoding == "32FC1")
			{
				try
				{
					//Convert pose
					auto t = m_tfBuffer.lookupTransform(image->header.frame_id, m_rootFrame, ros::Time(0));
				
					auto &translate = t.transform.translation;
					double x = translate.x;
					double y = translate.y;
					double z = translate.z;

					auto &q = t.transform.rotation;

					mrpt::math::CQuaternion quat(q.w, q.x, q.y, q.z);

					double yaw, pitch, roll;

					quat.rpy(roll,pitch,yaw);

					mrpt::poses::CPose3D pose(x,y,z, yaw, pitch, roll); 

					auto rangeScan = mrpt::obs::CObservation3DRangeScan::Create();

					mrpt::Clock::duration time = 
						std::chrono::duration_cast<mrpt::Clock::duration>(
							std::chrono::seconds(image->header.stamp.sec) +
							std::chrono::nanoseconds(image->header.stamp.nsec) +
							std::chrono::seconds(11644473600));

					rangeScan->sensorLabel = topic;
					rangeScan->timestamp = TTimeStamp(time);
					rangeScan->setSensorPose(pose);

					rangeScan->hasRangeImage = true;
					rangeScan->rangeImage_setSize(cv_ptr->image.rows, cv_ptr->image.cols);

					rangeScan->cameraParams.nrows = cv_ptr->image.rows;
					rangeScan->cameraParams.ncols = cv_ptr->image.cols;

					rangeScan->cameraParams.dist[0] = 0;
					rangeScan->cameraParams.dist[1] = 0;
					rangeScan->cameraParams.dist[2] = 0;
					rangeScan->cameraParams.dist[3] = 0;
					rangeScan->cameraParams.dist[4] = 0;

					size_t rows = cv_ptr->image.rows;
					size_t cols = cv_ptr->image.cols;
					//Need to implement something like ros synchronization
					MRPT_TODO("This should come from range sensor_msgs/CameraInfo");
					const double init[] = {50,0,cols/2,0,50,rows/2,0,0,1};
					rangeScan->cameraParams.intrinsicParams = mrpt::math::CMatrixDouble33(init);

					for(size_t i = 0; i < rows; i++)
					{
						for(size_t j = 0; j < cols; j++)
						{
							rangeScan->rangeImage(i,j) = cv_ptr->image.at<float>(i,j);
						}
					}

					ptr = rangeScan;
				}
				catch (tf2::TransformException& ex)
				{
					std::cerr << ex.what() << std::endl;
				}
			}
		}
		return ptr;
	};
	private:
	tf2::BufferCore m_tfBuffer;
	std::string m_rootFrame;
};

int main(int argc, char** argv)
{
	try
	{
		printf(" rosbag2rawlog - Part of the MRPT\n");
		printf(
			" MRPT C++ Library: %s - Sources timestamp: %s\n",
			MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());

		// Parse arguments:
		if (!cmd.parse(argc, argv))
			throw std::runtime_error("");  // should exit.

		auto input_bag_files = arg_input_files.getValue();
		string output_rawlog_file = arg_output_file.getValue();


		// Open input ros bag:
		std::vector<std::shared_ptr<rosbag::Bag>> bags;
		rosbag::View full_view;
		for(auto & file: input_bag_files)
		{
			ASSERT_FILE_EXISTS_(file);
			std::cout << "Opening: " << file << std::endl;
			auto bag = make_shared<rosbag::Bag>();
			bag->open(file);
			bags.push_back(bag);
			full_view.addQuery(*bag);
		}


		// Open output:
		if (mrpt::system::fileExists(output_rawlog_file) &&
			!arg_overwrite.isSet())
		{
			cout << "Output file already exists: `" << output_rawlog_file
				 << "`, aborting. Use `-w` flag to overwrite.\n";
			return 1;
		}

		CFileGZOutputStream fil_out;
		cout << "Opening for writing: '" << output_rawlog_file << "'...\n";
		if (!fil_out.open(output_rawlog_file))
			throw std::runtime_error("Error writing file!");

		auto arch = archiveFrom(fil_out);
		Transcriber t(arg_world_frame.getValue());
		for(const auto &m: full_view)
		{
			auto ptr = t.toMrpt(m);
			if(ptr)
			{
				arch << ptr;
			}
		}

		for(auto & bag: bags)
		{
			bag->close();
		}

		// successful end of program.
		return 0;
	}
	catch (std::exception& e)
	{
		if (strlen(e.what())) std::cerr << e.what() << std::endl;
		return 1;
	}
}  // end of main()

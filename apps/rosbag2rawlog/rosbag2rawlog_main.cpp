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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>

#include <yaml-cpp/yaml.h>

#include <memory>

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

TCLAP::ValueArg<std::string> arg_config_file(
	"c", "config", "Config yaml file (*.yml)", true, "", "config.yml",
	cmd);

TCLAP::SwitchArg arg_overwrite(
	"w", "overwrite", "Force overwrite target file without prompting.", cmd,
	false);

TCLAP::ValueArg<std::string> arg_world_frame(
	"f", "frame", "World TF frame", true, "", "world",
	cmd);


using CallbackFunction = std::function<mrpt::serialization::CSerializable::Ptr(const rosbag::MessageInstance &)>;

template <typename ... Args>
class RosSynchronizer : public std::enable_shared_from_this<RosSynchronizer<Args...>>
{
public:
	using Tuple = std::tuple<boost::shared_ptr<Args>...>;

	using Callback = std::function<mrpt::serialization::CSerializable::Ptr(const boost::shared_ptr<Args> &...)>;

	RosSynchronizer(const Callback &callback) : m_callback(callback)
	{
	}


	template <std::size_t ... N>
	mrpt::serialization::CSerializable::Ptr signal(std::index_sequence<N...>)
	{
		auto ptr = m_callback(std::get<N>(m_cache)...);
		m_cache = {};
		return ptr;
	}

	mrpt::serialization::CSerializable::Ptr signal()
	{
		return signal(std::make_index_sequence<sizeof...(Args)>{});
	}

	template <std::size_t... N>
	bool check(std::index_sequence<N...>)
	{
		return (std::get<N>(m_cache) && ...);
	}

	bool check()
	{
		return check(std::make_index_sequence<sizeof...(Args)>{});
	}

	template <size_t i>
	CallbackFunction bind()
	{
		std::shared_ptr<RosSynchronizer> ptr = this->shared_from_this();
		return [=](const rosbag::MessageInstance &rosmsg)
		{
			if(!std::get<i>(ptr->m_cache))
			{
				std::get<i>(ptr->m_cache) = 
					rosmsg.instantiate<typename std::tuple_element<i,Tuple>::type::element_type>();
			}
			if(ptr->check())
			{
				return ptr->signal();
			}
			return mrpt::serialization::CSerializable::Ptr();
		};
	}
private:
	Tuple m_cache;
	Callback m_callback;
};


mrpt::serialization::CSerializable::Ptr toRangeImage(std::string_view msg, const std::string &rootFrame, const tf2::BufferCore &tfBuffer, const sensor_msgs::Image::Ptr &image, const sensor_msgs::CameraInfo::Ptr &cameraInfo, bool rangeIsDepth)
{
	auto cv_ptr = cv_bridge::toCvShare(image);
	
	//For now we are just assuming this is a range image
	if(cv_ptr->encoding == "32FC1")
	{
		try
		{
			//Convert pose
			auto t = tfBuffer.lookupTransform(image->header.frame_id, rootFrame, ros::Time(0));
		
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

			rangeScan->sensorLabel = msg;
			rangeScan->timestamp = TTimeStamp(time);
			rangeScan->setSensorPose(pose);

			rangeScan->hasRangeImage = true;
			rangeScan->rangeImage_setSize(cv_ptr->image.rows, cv_ptr->image.cols);

			rangeScan->cameraParams.nrows = cv_ptr->image.rows;
			rangeScan->cameraParams.ncols = cv_ptr->image.cols;

			std::copy(cameraInfo->D.begin(), cameraInfo->D.end(), rangeScan->cameraParams.dist.begin());

			size_t rows = cv_ptr->image.rows;
			size_t cols = cv_ptr->image.cols;
			std::copy(cameraInfo->K.begin(), cameraInfo->K.end(), rangeScan->cameraParams.intrinsicParams.begin());

			for(size_t i = 0; i < rows; i++)
			{
				for(size_t j = 0; j < cols; j++)
				{
					rangeScan->rangeImage(i,j) = cv_ptr->image.at<float>(i,j);
				}
			}

			rangeScan->range_is_depth = rangeIsDepth;
			return rangeScan;
		}
		catch (tf2::TransformException& ex)
		{
			std::cerr << ex.what() << std::endl;
		}
	}
	return {};
}

class Transcriber
{
public:
	Transcriber(std::string_view rootFrame, const YAML::Node &config) : m_rootFrame(rootFrame)
	{
		m_lookup["/tf"].emplace_back([=](const rosbag::MessageInstance &rosmsg) {
			toTf<false>(rosmsg);
			return mrpt::serialization::CSerializable::Ptr();
		});
		m_lookup["/tf_static"].emplace_back([=](const rosbag::MessageInstance &rosmsg) {
			toTf<true>(rosmsg);
			return mrpt::serialization::CSerializable::Ptr();
		});

		for(auto &sensorNode: config["sensors"])
		{
			auto &sensorName = sensorNode.first.as<std::string>();
			auto &sensor = sensorNode.second;
			if(sensor["type"].as<std::string>() == "CObservation3DRangeScan")
			{
				bool rangeIsDepth = sensor["rangeIsDepth"].as<bool>(true);
				auto callback = [=](const sensor_msgs::Image::Ptr &image, const sensor_msgs::CameraInfo::Ptr &info)
				{
					return toRangeImage(sensorName, m_rootFrame, m_tfBuffer, image, info, rangeIsDepth);
				};
				using Synchronizer = RosSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>;
				auto sync = std::make_shared<Synchronizer>(callback);
				m_lookup[sensor["depth"].as<std::string>()].emplace_back(sync->bind<0>());
				m_lookup[sensor["cameraInfo"].as<std::string>()].emplace_back(sync->bind<1>());
			}
		}

	}
	template <bool isStatic>
	mrpt::serialization::CSerializable::Ptr toTf(const rosbag::MessageInstance &rosmsg)
	{
		if(rosmsg.getDataType() == "tf2_msgs/TFMessage")
		{
			auto tfs = rosmsg.instantiate<tf2_msgs::TFMessage>();
			for(auto & tf : tfs->transforms)
			{
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
		return {};
	}


	std::vector<mrpt::serialization::CSerializable::Ptr> toMrpt(const rosbag::MessageInstance &rosmsg)
	{
		std::vector<mrpt::serialization::CSerializable::Ptr> rets;
		auto topic = rosmsg.getTopic();
		auto search = m_lookup.find(topic);
		if(search != m_lookup.end())
		{
			for(const auto &found : std::get<1>(*search))
			{
				auto obs = found(rosmsg);
				if(obs)
				{
					rets.push_back(obs);
				}
			}
		}
		return rets;
	};
private:
	tf2::BufferCore m_tfBuffer;
	std::string m_rootFrame;
	std::map<std::string, std::vector<CallbackFunction>> m_lookup; 
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

		YAML::Node config = YAML::LoadFile(arg_config_file.getValue());

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
		Transcriber t(arg_world_frame.getValue(), config);
		for(const auto &m: full_view)
		{
			auto ptrs = t.toMrpt(m);
			for(auto &ptr : ptrs)
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

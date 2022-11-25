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
//  Started: HL @ SEPT-2018
// ===========================================================================

#include <cv_bridge/cv_bridge.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/ros1bridge/imu.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>
#include <rosbag/bag.h>	 // rosbag_storage C++ lib
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2_msgs/TFMessage.h>

#include <memory>

using namespace mrpt;
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
	"c", "config", "Config yaml file (*.yml)", true, "", "config.yml", cmd);

TCLAP::SwitchArg arg_overwrite(
	"w", "overwrite", "Force overwrite target file without prompting.", cmd,
	false);

TCLAP::ValueArg<std::string> arg_world_frame(
	"f", "frame", "World TF frame", true, "", "world", cmd);

using Obs = std::list<mrpt::serialization::CSerializable::Ptr>;

using CallbackFunction = std::function<Obs(const rosbag::MessageInstance&)>;

template <typename... Args>
class RosSynchronizer
	: public std::enable_shared_from_this<RosSynchronizer<Args...>>
{
   public:
	using Tuple = std::tuple<boost::shared_ptr<Args>...>;

	using Callback = std::function<Obs(const boost::shared_ptr<Args>&...)>;

	RosSynchronizer(
		std::string_view rootFrame, std::shared_ptr<tf2::BufferCore> tfBuffer,
		const Callback& callback)
		: m_rootFrame(rootFrame),
		  m_tfBuffer(std::move(tfBuffer)),
		  m_callback(callback)
	{
	}

	template <std::size_t... N>
	Obs signal(std::index_sequence<N...>)
	{
		auto ptr = m_callback(std::get<N>(m_cache)...);
		m_cache = {};
		return ptr;
	}

	Obs signal()
	{
		auto& frame = std::get<0>(m_cache)->header.frame_id;
		auto& stamp = std::get<0>(m_cache)->header.stamp;
		if (m_tfBuffer->canTransform(m_rootFrame, frame, stamp))
		{
			auto t = m_tfBuffer->lookupTransform(m_rootFrame, frame, stamp);
			mrpt::obs::CActionRobotMovement3D odom_move;

			auto& translate = t.transform.translation;
			double x = translate.x;
			double y = translate.y;
			double z = translate.z;

			auto& q = t.transform.rotation;
			mrpt::math::CQuaternion<double> quat{q.w, q.x, q.y, q.z};

			mrpt::poses::CPose3DQuat poseQuat(x, y, z, quat);
			mrpt::poses::CPose3D currentPose(poseQuat);

			mrpt::poses::CPose3D incOdoPose(0, 0, 0, 0, 0, 0);
			if (m_poseValid) { incOdoPose = currentPose - m_lastPose; }
			m_lastPose = currentPose;
			m_poseValid = true;
			mrpt::obs::CActionRobotMovement3D::TMotionModelOptions model;
			odom_move.computeFromOdometry(incOdoPose, model);

			auto acts = mrpt::obs::CActionCollection::Create();
			acts->insert(odom_move);

			auto obs = signal(std::make_index_sequence<sizeof...(Args)>{});
			obs.push_front(acts);
			return obs;
		}
		return {};
	}

	template <std::size_t... N>
	bool check(std::index_sequence<N...>)
	{
		return (std::get<N>(m_cache) && ...);
	}

	Obs checkAndSignal()
	{
		if (check(std::make_index_sequence<sizeof...(Args)>{}))
		{
			return signal();
		}
		return {};
	}

	template <size_t i>
	CallbackFunction bind()
	{
		std::shared_ptr<RosSynchronizer> ptr = this->shared_from_this();
		return [=](const rosbag::MessageInstance& rosmsg) {
			if (!std::get<i>(ptr->m_cache))
			{
				std::get<i>(ptr->m_cache) =
					rosmsg.instantiate<typename std::tuple_element<
						i, Tuple>::type::element_type>();
				return ptr->checkAndSignal();
			}
			return Obs();
		};
	}

	CallbackFunction bindTfSync()
	{
		std::shared_ptr<RosSynchronizer> ptr = this->shared_from_this();
		return [=](const rosbag::MessageInstance& rosmsg) {
			return ptr->checkAndSignal();
		};
	}

   private:
	std::string m_rootFrame;
	std::shared_ptr<tf2::BufferCore> m_tfBuffer;
	Tuple m_cache;
	bool m_poseValid = false;
	mrpt::poses::CPose3D m_lastPose;
	Callback m_callback;
};

Obs toPointCloud2(std::string_view msg, const rosbag::MessageInstance& rosmsg)
{
	auto pts = rosmsg.instantiate<sensor_msgs::PointCloud2>();

	auto ptsObs = mrpt::obs::CObservationPointCloud::Create();
	ptsObs->sensorLabel = msg;
	ptsObs->timestamp = mrpt::ros1bridge::fromROS(pts->header.stamp);

	// Convert points:
	std::set<std::string> fields = mrpt::ros1bridge::extractFields(*pts);

	// We need X Y Z:
	if (!fields.count("x") || !fields.count("y") || !fields.count("z"))
		return {};

	if (fields.count("intensity"))
	{
		// XYZI
		auto mrptPts = mrpt::maps::CPointsMapXYZI::Create();
		ptsObs->pointcloud = mrptPts;

		if (!mrpt::ros1bridge::fromROS(*pts, *mrptPts))
		{
			thread_local bool warn1st = false;
			if (!warn1st)
			{
				warn1st = true;
				std::cerr << "Could not convert pointcloud from ROS to "
							 "CPointsMapXYZI. Trying another format.\n";
			}
		}
	}

	{
		// XYZ
		auto mrptPts = mrpt::maps::CSimplePointsMap::Create();
		ptsObs->pointcloud = mrptPts;
		if (!mrpt::ros1bridge::fromROS(*pts, *mrptPts))
			THROW_EXCEPTION(
				"Could not convert pointcloud from ROS to "
				"CSimplePointsMap");
	}

	return {ptsObs};
}

Obs toRotatingScan(std::string_view msg, const rosbag::MessageInstance& rosmsg)
{
	auto pts = rosmsg.instantiate<sensor_msgs::PointCloud2>();

	// Convert points:
	std::set<std::string> fields = mrpt::ros1bridge::extractFields(*pts);

	// We need X Y Z:
	if (!fields.count("x") || !fields.count("y") || !fields.count("z") ||
		!fields.count("ring"))
		return {};

	// As a structured 2D range images, if we have ring numbers:
	auto obsRotScan = mrpt::obs::CObservationRotatingScan::Create();
	MRPT_TODO("Extract sensor pose from tf frames");
	const mrpt::poses::CPose3D sensorPose;

	if (!mrpt::ros1bridge::fromROS(*pts, *obsRotScan, sensorPose))
	{
		THROW_EXCEPTION(
			"Could not convert pointcloud from ROS to "
			"CObservationRotatingScan. Trying another format.");
	}

	obsRotScan->sensorLabel = msg;
	obsRotScan->timestamp = mrpt::ros1bridge::fromROS(pts->header.stamp);

	return {obsRotScan};
}

Obs toIMU(std::string_view msg, const rosbag::MessageInstance& rosmsg)
{
	auto pts = rosmsg.instantiate<sensor_msgs::Imu>();

	auto mrptObs = mrpt::obs::CObservationIMU::Create();

	mrptObs->sensorLabel = msg;
	mrptObs->timestamp = mrpt::ros1bridge::fromROS(pts->header.stamp);

	// Convert data:
	mrpt::ros1bridge::fromROS(*pts, *mrptObs);

	return {mrptObs};
}

Obs toRangeImage(
	std::string_view msg, const sensor_msgs::Image::Ptr& image,
	const sensor_msgs::CameraInfo::Ptr& cameraInfo, bool rangeIsDepth)
{
	auto cv_ptr = cv_bridge::toCvShare(image);

	// For now we are just assuming this is a range image
	if (cv_ptr->encoding == "32FC1")
	{
		auto rangeScan = mrpt::obs::CObservation3DRangeScan::Create();

		// MRPT assumes the image plane is parallel to the YZ plane, so the
		// camera is pointed in the X direction ROS assumes the image plane
		// is parallel to XY plane, so the camera is pointed in the Z
		// direction Apply a rotation to convert between these conventions.
		mrpt::math::CQuaternion<double> rot{0.5, 0.5, -0.5, 0.5};
		mrpt::poses::CPose3DQuat poseQuat(0, 0, 0, rot);
		mrpt::poses::CPose3D pose(poseQuat);
		rangeScan->setSensorPose(pose);

		rangeScan->sensorLabel = msg;
		rangeScan->timestamp = mrpt::ros1bridge::fromROS(image->header.stamp);

		rangeScan->hasRangeImage = true;
		rangeScan->rangeImage_setSize(cv_ptr->image.rows, cv_ptr->image.cols);

		rangeScan->cameraParams.nrows = cv_ptr->image.rows;
		rangeScan->cameraParams.ncols = cv_ptr->image.cols;

		std::copy(
			cameraInfo->D.begin(), cameraInfo->D.end(),
			rangeScan->cameraParams.dist.begin());

		size_t rows = cv_ptr->image.rows;
		size_t cols = cv_ptr->image.cols;
		std::copy(
			cameraInfo->K.begin(), cameraInfo->K.end(),
			rangeScan->cameraParams.intrinsicParams.begin());

		rangeScan->rangeUnits = 1e-3;
		const float inv_unit = 1.0f / rangeScan->rangeUnits;

		for (size_t i = 0; i < rows; i++)
			for (size_t j = 0; j < cols; j++)
				rangeScan->rangeImage(i, j) = static_cast<uint16_t>(
					inv_unit * cv_ptr->image.at<float>(i, j));

		rangeScan->range_is_depth = rangeIsDepth;

		return {rangeScan};
	}
	return {};
}

Obs toImage(std::string_view msg, const rosbag::MessageInstance& rosmsg)
{
	auto image = rosmsg.instantiate<sensor_msgs::Image>();

	auto cv_ptr = cv_bridge::toCvShare(image);

	auto imgObs = mrpt::obs::CObservationImage::Create();

	imgObs->sensorLabel = msg;
	imgObs->timestamp = mrpt::ros1bridge::fromROS(image->header.stamp);

	imgObs->image = mrpt::img::CImage(cv_ptr->image, mrpt::img::SHALLOW_COPY);

	return {imgObs};
}

template <bool isStatic>
Obs toTf(tf2::BufferCore& tfBuffer, const rosbag::MessageInstance& rosmsg)
{
	if (rosmsg.getDataType() == "tf2_msgs/TFMessage")
	{
		auto tfs = rosmsg.instantiate<tf2_msgs::TFMessage>();
		for (auto& tf : tfs->transforms)
		{
			try
			{
				tfBuffer.setTransform(tf, "bagfile", isStatic);
			}
			catch (const tf2::TransformException& ex)
			{
				std::cerr << ex.what() << std::endl;
			}
		}
	}
	return {};
}

class Transcriber
{
   public:
	Transcriber(
		std::string_view rootFrame, const mrpt::containers::yaml& config)
		: m_rootFrame(rootFrame)
	{
		auto tfBuffer = std::make_shared<tf2::BufferCore>();

		m_lookup["/tf"].emplace_back(
			[=](const rosbag::MessageInstance& rosmsg) {
				return toTf<false>(*tfBuffer, rosmsg);
			});
		m_lookup["/tf_static"].emplace_back(
			[=](const rosbag::MessageInstance& rosmsg) {
				return toTf<true>(*tfBuffer, rosmsg);
			});

		for (auto& sensorNode : config["sensors"].asMap())
		{
			auto sensorName = sensorNode.first.as<std::string>();
			auto& sensor = sensorNode.second.asMap();
			const auto sensorType = sensor.at("type").as<std::string>();

			if (sensorType == "CObservation3DRangeScan")
			{
				bool rangeIsDepth = sensor.count("rangeIsDepth")
					? sensor.at("rangeIsDepth").as<bool>()
					: true;
				auto callback = [=](const sensor_msgs::Image::Ptr& image,
									const sensor_msgs::CameraInfo::Ptr& info) {
					return toRangeImage(sensorName, image, info, rangeIsDepth);
				};
				using Synchronizer = RosSynchronizer<
					sensor_msgs::Image, sensor_msgs::CameraInfo>;
				auto sync = std::make_shared<Synchronizer>(
					rootFrame, tfBuffer, callback);
				m_lookup[sensor.at("depth").as<std::string>()].emplace_back(
					sync->bind<0>());
				m_lookup[sensor.at("cameraInfo").as<std::string>()]
					.emplace_back(sync->bind<1>());
				m_lookup["/tf"].emplace_back(sync->bindTfSync());
			}
			else if (sensorType == "CObservationImage")
			{
				auto callback = [=](const rosbag::MessageInstance& m) {
					return toImage(sensorName, m);
				};
				m_lookup[sensor.at("image_topic").as<std::string>()]
					.emplace_back(callback);
			}
			else if (sensorType == "CObservationPointCloud")
			{
				auto callback = [=](const rosbag::MessageInstance& m) {
					return toPointCloud2(sensorName, m);
				};
				m_lookup[sensor.at("topic").as<std::string>()].emplace_back(
					callback);
				// m_lookup["/tf"].emplace_back(sync->bindTfSync());
			}
			else if (sensorType == "CObservationRotatingScan")
			{
				auto callback = [=](const rosbag::MessageInstance& m) {
					return toRotatingScan(sensorName, m);
				};
				m_lookup[sensor.at("topic").as<std::string>()].emplace_back(
					callback);
			}
			else if (sensorType == "CObservationIMU")
			{
				auto callback = [=](const rosbag::MessageInstance& m) {
					return toIMU(sensorName, m);
				};
				m_lookup[sensor.at("topic").as<std::string>()].emplace_back(
					callback);
				// m_lookup["/tf"].emplace_back(sync->bindTfSync());
			}
			// TODO: Handle more cases?
		}
	}

	Obs toMrpt(const rosbag::MessageInstance& rosmsg)
	{
		Obs rets;
		auto topic = rosmsg.getTopic();

		if (auto search = m_lookup.find(topic); search != m_lookup.end())
		{
			for (const auto& callback : search->second)
			{
				auto obs = callback(rosmsg);
				rets.insert(rets.end(), obs.begin(), obs.end());
			}
		}
		else
		{
			if (m_unhandledTopics.count(topic) == 0)
			{
				m_unhandledTopics.insert(topic);
				std::cout << "Warning: unhandled topic '" << topic << "'"
						  << std::endl;
			}
		}
		return rets;
	};

   private:
	std::string m_rootFrame;
	std::map<std::string, std::vector<CallbackFunction>> m_lookup;
	std::set<std::string> m_unhandledTopics;
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

		auto config =
			mrpt::containers::yaml::FromFile(arg_config_file.getValue());

		auto input_bag_files = arg_input_files.getValue();
		string output_rawlog_file = arg_output_file.getValue();

		// Open input ros bag:
		std::vector<std::shared_ptr<rosbag::Bag>> bags;
		rosbag::View full_view;
		for (auto& file : input_bag_files)
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
		const auto nEntries = full_view.size();
		size_t curEntry = 0, showProgressCnt = 0;
		for (const auto& m : full_view)
		{
			auto ptrs = t.toMrpt(m);
			for (auto& ptr : ptrs)
			{
				arch << ptr;
			}

			curEntry++;

			if (++showProgressCnt > 100)
			{
				const double pr = (1.0 * curEntry) / nEntries;

				printf(
					"Progress: %u/%u %s %.03f%%        \r",
					static_cast<unsigned int>(curEntry),
					static_cast<unsigned int>(nEntries),
					mrpt::system::progress(pr, 50).c_str(), 100.0 * pr);
				fflush(stdout);
				showProgressCnt = 0;
			}
		}
		printf("\n");

		for (auto& bag : bags)
		{
			bag->close();
		}

		// successful end of program.
		return 0;
	}
	catch (std::exception& e)
	{
		if (strlen(e.what()))
			std::cerr << mrpt::exception_to_str(e) << std::endl;
		return 1;
	}
}  // end of main()

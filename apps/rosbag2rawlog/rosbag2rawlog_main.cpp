/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

// ===========================================================================
//  Program: rosbag2rawlog
//  Intention: Parse bag files, save
//             as a RawLog file, easily readable by MRPT C++ programs.
//
//  Started: Hunter Laux @ SEPT-2018.
//  Maintained: JLBC @ 2018-2024
// ===========================================================================

#include <cv_bridge/cv_bridge.h>  // this header is obsolete in ros2-I but as long as this app is only built for ros1 we are ok
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/ros1bridge/imu.h>
#include <mrpt/ros1bridge/laser_scan.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/pose.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>  // rosbag_storage C++ lib
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // needed by tf2::fromMsg()
#include <tf2_msgs/TFMessage.h>

#include <memory>

using namespace mrpt;
using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::system;
using namespace std;

// Declare the supported command line switches ===========
TCLAP::CmdLine cmd("rosbag2rawlog (ROS 1)", ' ', MRPT_getVersion().c_str());

TCLAP::UnlabeledMultiArg<std::string> arg_input_files(
    "bags", "Input bag files (required) (*.bag)", true, "log.bag", cmd);

TCLAP::ValueArg<std::string> arg_output_file(
    "o", "output", "Output dataset (*.rawlog)", true, "", "dataset_out.rawlog", cmd);

TCLAP::ValueArg<std::string> arg_config_file(
    "c", "config", "Config yaml file (*.yml)", true, "", "config.yml", cmd);

TCLAP::SwitchArg arg_overwrite(
    "w", "overwrite", "Force overwrite target file without prompting.", cmd, false);

TCLAP::ValueArg<std::string> arg_base_link_frame(
    "b",
    "base-link",
    "Reference /tf frame for the robot frame (Default: 'base_link')",
    false,
    "base_link",
    "base_link",
    cmd);

using Obs = std::list<mrpt::serialization::CSerializable::Ptr>;

using CallbackFunction = std::function<Obs(const rosbag::MessageInstance&)>;

template <typename... Args>
class RosSynchronizer : public std::enable_shared_from_this<RosSynchronizer<Args...>>
{
 public:
  using Tuple = std::tuple<boost::shared_ptr<Args>...>;

  using Callback = std::function<Obs(const boost::shared_ptr<Args>&...)>;

  RosSynchronizer(std::shared_ptr<tf2::BufferCore> tfBuffer, const Callback& callback) :
      m_tfBuffer(std::move(tfBuffer)), m_callback(callback)
  {
  }

  template <std::size_t... N>
  Obs signal(std::index_sequence<N...>)
  {
    auto ptr = m_callback(std::get<N>(m_cache)...);
    m_cache = {};
    return ptr;
  }

  Obs signal() { return {}; }

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
    return [=](const rosbag::MessageInstance& rosmsg)
    {
      if (!std::get<i>(ptr->m_cache))
      {
        std::get<i>(ptr->m_cache) =
            rosmsg.instantiate<typename std::tuple_element<i, Tuple>::type::element_type>();
        return ptr->checkAndSignal();
      }
      return Obs();
    };
  }

  CallbackFunction bindTfSync()
  {
    std::shared_ptr<RosSynchronizer> ptr = this->shared_from_this();
    return [=](const rosbag::MessageInstance& rosmsg) { return ptr->checkAndSignal(); };
  }

 private:
  std::shared_ptr<tf2::BufferCore> m_tfBuffer;
  Tuple m_cache;
  bool m_poseValid = false;
  mrpt::poses::CPose3D m_lastPose;
  Callback m_callback;
};

std::shared_ptr<tf2::BufferCore> tfBuffer;

std::set<std::string> known_tf_frames;

void removeTrailingSlash(std::string& s)
{
  ASSERT_(!s.empty());
  if (s.at(0) == '/') s = s.substr(1);
}

void addTfFrameAsKnown(std::string s)
{
  removeTrailingSlash(s);
  known_tf_frames.insert(s);
}

bool findOutSensorPose(
    mrpt::poses::CPose3D& des,
    std::string referenceFrame,
    std::string frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  if (fixedSensorPose)
  {
    des = fixedSensorPose.value();
    return true;
  }

  // TF1 old frames started with "/foo", forbidden in TF2.
  // Handle this since this program is for importing old ROS1 bags:
  removeTrailingSlash(referenceFrame);
  removeTrailingSlash(frame);

  try
  {
    ASSERT_(tfBuffer);

    geometry_msgs::TransformStamped ref_to_trgFrame =
        tfBuffer->lookupTransform(frame, referenceFrame, {} /*latest value*/);

    tf2::Transform tf;
    tf2::fromMsg(ref_to_trgFrame.transform, tf);
    des = mrpt::ros1bridge::fromROS(tf);

#if 0
		std::cout << mrpt::format(
			"[findOutSensorPose] Found pose %s -> %s: %s\n",
			referenceFrame.c_str(), frame.c_str(), des.asString().c_str());
#endif

    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    std::cerr << "findOutSensorPose: " << ex.what() << std::endl << "\nKnown TF frames: ";
    for (const auto& f : known_tf_frames) std::cerr << "'" << f << "',";
    std::cerr << std::endl;

    return false;
  }
}

Obs toPointCloud2(
    std::string_view msg,
    const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  auto pts = rosmsg.instantiate<sensor_msgs::PointCloud2>();

  auto ptsObs = mrpt::obs::CObservationPointCloud::Create();
  ptsObs->sensorLabel = msg;
  ptsObs->timestamp = mrpt::ros1bridge::fromROS(pts->header.stamp);

  bool sensorPoseOK = findOutSensorPose(
      ptsObs->sensorPose, pts->header.frame_id, arg_base_link_frame.getValue(), fixedSensorPose);
  if (!sensorPoseOK)
  {
    std::cerr << "Warning: dropping one observation of type '" << msg
              << "' due to missing /tf data.\n";
    return {};
  }

  // Convert points:
  std::set<std::string> fields = mrpt::ros1bridge::extractFields(*pts);

  // We need X Y Z:
  if (!fields.count("x") || !fields.count("y") || !fields.count("z")) return {};

  if (fields.count("ring") || fields.count("time"))
  {
    // XYZIRT
    auto mrptPts = mrpt::maps::CPointsMapXYZIRT::Create();
    ptsObs->pointcloud = mrptPts;

    if (!mrpt::ros1bridge::fromROS(*pts, *mrptPts))
    {
      THROW_EXCEPTION("Could not convert pointcloud from ROS to CPointsMapXYZIRT");
    }
    else
    {  // converted ok:
      return {ptsObs};
    }
  }

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
                     "CPointsMapXYZI. Trying with XYZ.\n";
      }
    }
    else
    {  // converted ok:
      return {ptsObs};
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

Obs toLidar2D(
    std::string_view msg,
    const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  auto scan = rosmsg.instantiate<sensor_msgs::LaserScan>();

  auto scanObs = mrpt::obs::CObservation2DRangeScan::Create();

  // Extract sensor pose from tf frames, if enabled:
  mrpt::poses::CPose3D sensorPose;
  mrpt::ros1bridge::fromROS(*scan, sensorPose, *scanObs);

  scanObs->sensorLabel = msg;
  scanObs->timestamp = mrpt::ros1bridge::fromROS(scan->header.stamp);

  bool sensorPoseOK = findOutSensorPose(
      scanObs->sensorPose, scan->header.frame_id, arg_base_link_frame.getValue(), fixedSensorPose);
  if (!sensorPoseOK)
  {
    std::cerr << "Warning: dropping one observation of type '" << msg
              << "' due to missing /tf data.\n";
    return {};
  }

  return {scanObs};
}

Obs toRotatingScan(
    std::string_view msg,
    const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  auto pts = rosmsg.instantiate<sensor_msgs::PointCloud2>();

  // Convert points:
  std::set<std::string> fields = mrpt::ros1bridge::extractFields(*pts);

  // We need X Y Z:
  if (!fields.count("x") || !fields.count("y") || !fields.count("z") || !fields.count("ring"))
    return {};

  // As a structured 2D range images, if we have ring numbers:
  auto obsRotScan = mrpt::obs::CObservationRotatingScan::Create();
  const mrpt::poses::CPose3D sensorPose;

  if (!mrpt::ros1bridge::fromROS(*pts, *obsRotScan, sensorPose))
  {
    THROW_EXCEPTION(
        "Could not convert pointcloud from ROS to "
        "CObservationRotatingScan. Trying another format.");
  }

  obsRotScan->sensorLabel = msg;
  obsRotScan->timestamp = mrpt::ros1bridge::fromROS(pts->header.stamp);

  bool sensorPoseOK = findOutSensorPose(
      obsRotScan->sensorPose, pts->header.frame_id, arg_base_link_frame.getValue(),
      fixedSensorPose);
  if (!sensorPoseOK)
  {
    std::cerr << "Warning: dropping one observation of type '" << msg
              << "' due to missing /tf data.\n";
    return {};
  }

  return {obsRotScan};
}

Obs toIMU(
    std::string_view msg,
    const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  auto imu = rosmsg.instantiate<sensor_msgs::Imu>();

  auto mrptObs = mrpt::obs::CObservationIMU::Create();

  mrptObs->sensorLabel = msg;
  mrptObs->timestamp = mrpt::ros1bridge::fromROS(imu->header.stamp);

  // Convert data:
  mrpt::ros1bridge::fromROS(*imu, *mrptObs);

  bool sensorPoseOK = findOutSensorPose(
      mrptObs->sensorPose, imu->header.frame_id, arg_base_link_frame.getValue(), fixedSensorPose);
  if (!sensorPoseOK)
  {
    std::cerr << "Warning: dropping one observation of type '" << msg
              << "' due to missing /tf data.\n";
    return {};
  }

  return {mrptObs};
}

Obs toOdometry(std::string_view msg, const rosbag::MessageInstance& rosmsg)
{
  auto odo = rosmsg.instantiate<nav_msgs::Odometry>();

  auto mrptObs = mrpt::obs::CObservationOdometry::Create();

  mrptObs->sensorLabel = msg;
  mrptObs->timestamp = mrpt::ros1bridge::fromROS(odo->header.stamp);

  // Convert data:
  const auto pose = mrpt::ros1bridge::fromROS(odo->pose);
  mrptObs->odometry = {pose.mean.x(), pose.mean.y(), pose.mean.yaw()};

  mrptObs->hasVelocities = true;
  mrptObs->velocityLocal.vx = odo->twist.twist.linear.x;
  mrptObs->velocityLocal.vy = odo->twist.twist.linear.y;
  mrptObs->velocityLocal.omega = odo->twist.twist.angular.z;

  return {mrptObs};
}

Obs toImage(
    std::string_view msg,
    const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  cv_bridge::CvImagePtr cv_ptr;
  mrpt::Clock::time_point timeStamp;
  std::string frame_id;

  if (auto image = rosmsg.instantiate<sensor_msgs::Image>(); image)
  {
    cv_ptr = cv_bridge::toCvCopy(image);
    timeStamp = mrpt::ros1bridge::fromROS(image->header.stamp);
    frame_id = image->header.frame_id;
  }
  else if (auto imC = rosmsg.instantiate<sensor_msgs::CompressedImage>(); imC)
  {
    cv_ptr = cv_bridge::toCvCopy(imC);
    timeStamp = mrpt::ros1bridge::fromROS(imC->header.stamp);
    frame_id = imC->header.frame_id;
  }
  else
  {
    THROW_EXCEPTION_FMT(
        "Unhandled ROS topic '%s' type: images are expected to be either "
        "sensor_msgs::Image or sensor_msgs::CompressedImage",
        std::string(msg).c_str());
  }

  auto imgObs = mrpt::obs::CObservationImage::Create();

  imgObs->sensorLabel = msg;
  imgObs->timestamp = timeStamp;

  imgObs->image = mrpt::img::CImage(cv_ptr->image, mrpt::img::SHALLOW_COPY);

  bool sensorPoseOK = findOutSensorPose(
      imgObs->cameraPose, frame_id, arg_base_link_frame.getValue(), fixedSensorPose);
  if (!sensorPoseOK)
  {
    std::cerr << "Warning: dropping one observation of type '" << msg
              << "' due to missing /tf data.\n";
    return {};
  }

  return {imgObs};
}

Obs toRangeImage(
    std::string_view msg,
    const sensor_msgs::Image::Ptr& image,
    const sensor_msgs::CameraInfo::Ptr& cameraInfo,
    bool rangeIsDepth,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  auto cv_ptr = cv_bridge::toCvShare(image);

  // For now we are just assuming this is a range image
  if (cv_ptr->encoding == "32FC1")
  {
    auto rangeScan = mrpt::obs::CObservation3DRangeScan::Create();

    rangeScan->sensorLabel = msg;
    rangeScan->timestamp = mrpt::ros1bridge::fromROS(image->header.stamp);

    rangeScan->hasRangeImage = true;
    rangeScan->rangeImage_setSize(cv_ptr->image.rows, cv_ptr->image.cols);

    rangeScan->cameraParams.nrows = cv_ptr->image.rows;
    rangeScan->cameraParams.ncols = cv_ptr->image.cols;

    std::copy(cameraInfo->D.begin(), cameraInfo->D.end(), rangeScan->cameraParams.dist.begin());

    size_t rows = cv_ptr->image.rows;
    size_t cols = cv_ptr->image.cols;
    std::copy(
        cameraInfo->K.begin(), cameraInfo->K.end(),
        rangeScan->cameraParams.intrinsicParams.begin());

    rangeScan->rangeUnits = 1e-3;
    const float inv_unit = 1.0f / rangeScan->rangeUnits;

    for (size_t i = 0; i < rows; i++)
      for (size_t j = 0; j < cols; j++)
        rangeScan->rangeImage(i, j) =
            static_cast<uint16_t>(inv_unit * cv_ptr->image.at<float>(i, j));

    rangeScan->range_is_depth = rangeIsDepth;

    bool sensorPoseOK = findOutSensorPose(
        rangeScan->sensorPose, image->header.frame_id, arg_base_link_frame.getValue(),
        fixedSensorPose);
    if (!sensorPoseOK)
    {
      std::cerr << "Warning: dropping one observation of type '" << msg
                << "' due to missing /tf data.\n";
      return {};
    }

#if 0  // JLBC: not needed anymore?
	   // MRPT assumes the image plane is parallel to the YZ plane, so the
	   // camera is pointed in the X direction ROS assumes the image plane
	   // is parallel to XY plane, so the camera is pointed in the Z
	   // direction Apply a rotation to convert between these conventions.
		mrpt::math::CQuaternion<double> rot{0.5, 0.5, -0.5, 0.5};
		mrpt::poses::CPose3DQuat poseQuat(0, 0, 0, rot);
		mrpt::poses::CPose3D pose(poseQuat);
		rangeScan->setSensorPose(pose);
#endif

    return {rangeScan};
  }
  return {};
}

template <bool isStatic>
Obs toTf(tf2::BufferCore& tfBuf, const rosbag::MessageInstance& rosmsg)
{
  if (rosmsg.getDataType() == "tf2_msgs/TFMessage")
  {
    auto tfs = rosmsg.instantiate<tf2_msgs::TFMessage>();
    for (auto& tf : tfs->transforms)
    {
      try
      {
        tfBuf.setTransform(tf, "bagfile", isStatic);

        addTfFrameAsKnown(tf.child_frame_id);
        addTfFrameAsKnown(tf.header.frame_id);
#if 0
				std::cout << "tf: " << tf.child_frame_id
						  << " <= " << tf.header.frame_id << "\n";
#endif
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
  Transcriber(const mrpt::containers::yaml& config)
  {
    tfBuffer = std::make_shared<tf2::BufferCore>();

    m_lookup["/tf"].emplace_back([=](const rosbag::MessageInstance& rosmsg)
                                 { return toTf<false>(*tfBuffer, rosmsg); });
    m_lookup["/tf_static"].emplace_back([=](const rosbag::MessageInstance& rosmsg)
                                        { return toTf<true>(*tfBuffer, rosmsg); });

    for (auto& sensorNode : config["sensors"].asMap())
    {
      auto sensorName = sensorNode.first.as<std::string>();
      auto& sensor = sensorNode.second.asMap();
      const auto sensorType = sensor.at("type").as<std::string>();

      // Optional: fixed sensorPose (then ignores/don't need "tf" data):
      std::optional<mrpt::poses::CPose3D> fixedSensorPose;
      if (sensor.count("fixed_sensor_pose") != 0)
      {
        fixedSensorPose = mrpt::poses::CPose3D::FromString(
            "["s + sensor.at("fixed_sensor_pose").as<std::string>() + "]"s);
      }

#if 0
			if (sensorType == "CObservation3DRangeScan")
			{
				bool rangeIsDepth = sensor.count("rangeIsDepth")
					? sensor.at("rangeIsDepth").as<bool>()
					: true;
				auto callback = [=](const sensor_msgs::Image::Ptr& image,
									const sensor_msgs::CameraInfo::Ptr& info) {
					return toRangeImage(
						sensorName, image, info, rangeIsDepth, fixedSensorPose);
				};
				using Synchronizer = RosSynchronizer<
					sensor_msgs::Image, sensor_msgs::CameraInfo>;
				auto sync = std::make_shared<Synchronizer>(tfBuffer, callback);
				m_lookup[sensor.at("depth").as<std::string>()].emplace_back(
					sync->bind<0>());
				m_lookup[sensor.at("cameraInfo").as<std::string>()]
					.emplace_back(sync->bind<1>());
				m_lookup["/tf"].emplace_back(sync->bindTfSync());
			}
			else
#endif
      if (sensorType == "CObservationImage")
      {
        auto callback = [=](const rosbag::MessageInstance& m)
        { return toImage(sensorName, m, fixedSensorPose); };
        ASSERT_(sensor.count("image_topic") != 0);
        m_lookup[sensor.at("image_topic").as<std::string>()].emplace_back(callback);
      }
      else if (sensorType == "CObservationPointCloud")
      {
        auto callback = [=](const rosbag::MessageInstance& m)
        { return toPointCloud2(sensorName, m, fixedSensorPose); };
        m_lookup[sensor.at("topic").as<std::string>()].emplace_back(callback);
      }
      else if (sensorType == "CObservation2DRangeScan")
      {
        auto callback = [=](const rosbag::MessageInstance& m)
        { return toLidar2D(sensorName, m, fixedSensorPose); };
        m_lookup[sensor.at("topic").as<std::string>()].emplace_back(callback);
      }
      else if (sensorType == "CObservationRotatingScan")
      {
        auto callback = [=](const rosbag::MessageInstance& m)
        { return toRotatingScan(sensorName, m, fixedSensorPose); };
        m_lookup[sensor.at("topic").as<std::string>()].emplace_back(callback);
      }
      else if (sensorType == "CObservationIMU")
      {
        auto callback = [=](const rosbag::MessageInstance& m)
        { return toIMU(sensorName, m, fixedSensorPose); };
        m_lookup[sensor.at("topic").as<std::string>()].emplace_back(callback);
      }
      else if (sensorType == "CObservationOdometry")
      {
        auto callback = [=](const rosbag::MessageInstance& m) { return toOdometry(sensorName, m); };
        m_lookup[sensor.at("topic").as<std::string>()].emplace_back(callback);
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

        std::cout << "Warning: unhandled topic '" << topic << "' [" << rosmsg.getDataType() << "]"
                  << std::endl;
      }
    }
    return rets;
  };

 private:
  std::map<std::string, std::vector<CallbackFunction>> m_lookup;
  std::set<std::string> m_unhandledTopics;
};

int main(int argc, char** argv)
{
  try
  {
    printf(" rosbag2rawlog (ROS 1) - Part of the MRPT\n");
    printf(
        " MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(),
        MRPT_getCompilationDate().c_str());

    // Parse arguments:
    if (!cmd.parse(argc, argv)) throw std::runtime_error("");  // should exit.

    auto config = mrpt::containers::yaml::FromFile(arg_config_file.getValue());

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
    if (mrpt::system::fileExists(output_rawlog_file) && !arg_overwrite.isSet())
    {
      cout << "Output file already exists: `" << output_rawlog_file
           << "`, aborting. Use `-w` flag to overwrite.\n";
      return 1;
    }

    CFileGZOutputStream fil_out;
    cout << "Opening for writing: '" << output_rawlog_file << "'...\n";
    if (!fil_out.open(output_rawlog_file)) throw std::runtime_error("Error writing file!");

    auto arch = archiveFrom(fil_out);
    Transcriber t(config);
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
            "Progress: %u/%u %s %.03f%%        \r", static_cast<unsigned int>(curEntry),
            static_cast<unsigned int>(nEntries), mrpt::system::progress(pr, 50).c_str(),
            100.0 * pr);
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
    if (strlen(e.what())) std::cerr << e.what() << std::endl;
    return 1;
  }
}  // end of main()

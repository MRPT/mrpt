/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers

#include <mrpt/apps/RawlogGrabberApp.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/round.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/img/CImage.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CRateTimer.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <thread>

using namespace mrpt::apps;

RawlogGrabberApp::RawlogGrabberApp()
	: mrpt::system::COutputLogger("RawlogGrabberApp")
{
}

void RawlogGrabberApp::initialize(int argc, const char** argv)
{
	MRPT_START

	if ((getenv("MRPT_HWDRIVERS_VERBOSE") != nullptr) &&
		atoi(getenv("MRPT_HWDRIVERS_VERBOSE")) != 0)
	{
		this->setMinLoggingLevel(mrpt::system::LVL_DEBUG);
	}

	MRPT_LOG_INFO(" rawlog-grabber - Part of the MRPT");
	MRPT_LOG_INFO_FMT(
		" MRPT C++ Library: %s - Sources timestamp: %s\n",
		mrpt::system::MRPT_getVersion().c_str(),
		mrpt::system::MRPT_getCompilationDate().c_str());

	// Process arguments:
	if (argc < 2)
		THROW_EXCEPTION_FMT("Usage: %s <config_file.ini>\n\n", argv[0]);

	{
		std::string INI_FILENAME(argv[1]);
		ASSERT_FILE_EXISTS_(INI_FILENAME);
		std::vector<std::string> cfgLines;
		mrpt::io::loadTextFile(cfgLines, INI_FILENAME);
		params.setContent(cfgLines);
	}

	MRPT_END
}

void RawlogGrabberApp::run()
{
	MRPT_START

	using namespace mrpt;
	using namespace mrpt::system;
	using namespace mrpt::hwdrivers;
	using namespace mrpt::config;
	using namespace mrpt::serialization;
	using namespace mrpt::img;
	using namespace mrpt::obs;
	using namespace mrpt::poses;
	using namespace std;

	const std::string GLOBAL_SECT = "global";

	// ------------------------------------------
	//			Load config from file:
	// ------------------------------------------
	string rawlog_prefix = "dataset";
	int time_between_launches = 300;
	bool use_sensoryframes = false;
	int GRABBER_PERIOD_MS = 1000;
	int rawlog_GZ_compress_level = 1;  // 0: No compress, 1-9: compress level

	MRPT_LOAD_CONFIG_VAR(rawlog_prefix, string, params, GLOBAL_SECT);
	MRPT_LOAD_CONFIG_VAR(time_between_launches, int, params, GLOBAL_SECT);
	MRPT_LOAD_CONFIG_VAR(SF_max_time_span, float, params, GLOBAL_SECT);
	MRPT_LOAD_CONFIG_VAR(use_sensoryframes, bool, params, GLOBAL_SECT);
	MRPT_LOAD_CONFIG_VAR(GRABBER_PERIOD_MS, int, params, GLOBAL_SECT);

	MRPT_LOAD_CONFIG_VAR(rawlog_GZ_compress_level, int, params, GLOBAL_SECT);

	// Build full rawlog file name:
	string rawlog_postfix = "_";

	// rawlog_postfix += dateTimeToString( now() );
	mrpt::system::TTimeParts parts;
	mrpt::system::timestampToParts(now(), parts, true);
	rawlog_postfix += format(
		"%04u-%02u-%02u_%02uh%02um%02us", (unsigned int)parts.year,
		(unsigned int)parts.month, (unsigned int)parts.day,
		(unsigned int)parts.hour, (unsigned int)parts.minute,
		(unsigned int)parts.second);

	rawlog_postfix = mrpt::system::fileNameStripInvalidChars(rawlog_postfix);

	// Only set this if we want externally stored images:
	m_rawlog_ext_imgs_dir =
		rawlog_prefix +
		fileNameStripInvalidChars(rawlog_postfix + string("_Images"));

	// Also, set the path in CImage to enable online visualization in a GUI
	// window:
	CImage::setImagesPathBase(m_rawlog_ext_imgs_dir);

	rawlog_postfix += string(".rawlog");
	rawlog_postfix = fileNameStripInvalidChars(rawlog_postfix);

	rawlog_filename = rawlog_prefix + rawlog_postfix;

	MRPT_LOG_INFO_STREAM("Output rawlog filename: " << rawlog_filename);
	MRPT_LOG_INFO_STREAM("External image storage: " << m_rawlog_ext_imgs_dir);

	// ----------------------------------------------
	// Launch threads:
	// ----------------------------------------------
	std::vector<std::string> sections;
	params.getAllSections(sections);

	vector<std::thread> lstThreads;

	for (auto& section : sections)
	{
		if (section == GLOBAL_SECT || section.empty() ||
			params.read_bool(section, "rawlog-grabber-ignore", false, false))
			continue;  // This is not a sensor:

		lstThreads.emplace_back(&RawlogGrabberApp::SensorThread, this, section);

		std::this_thread::sleep_for(
			std::chrono::milliseconds(time_between_launches));
	}

	// ----------------------------------------------
	// Run:
	// ----------------------------------------------
	mrpt::io::CFileGZOutputStream out_file;
	auto out_arch_obj = archiveFrom(out_file);
	m_out_arch_ptr = &out_arch_obj;

	out_file.open(rawlog_filename, rawlog_GZ_compress_level);

	CGenericSensor::TListObservations copy_of_m_global_list_obs;

	MRPT_LOG_INFO_STREAM("Press any key to exit program");

	mrpt::system::CTicTac run_timer;
	run_timer.Tic();

	while (!os::kbhit() && !allThreadsMustExit)
	{
		// Check "run for X seconds" flag:
		if (run_for_seconds > 0 && run_timer.Tac() > run_for_seconds) break;

		// See if we have observations and process them:
		{
			std::lock_guard<std::mutex> lock(cs_m_global_list_obs);
			copy_of_m_global_list_obs.clear();

			if (!m_global_list_obs.empty())
			{
				auto itEnd = m_global_list_obs.begin();
				std::advance(itEnd, m_global_list_obs.size() / 2);
				copy_of_m_global_list_obs.insert(
					m_global_list_obs.begin(), itEnd);
				m_global_list_obs.erase(m_global_list_obs.begin(), itEnd);
			}
		}  // End of cs lock

		if (use_sensoryframes)
		{
			process_observations_for_sf(copy_of_m_global_list_obs);
		}
		else
		{
			process_observations_for_nonsf(copy_of_m_global_list_obs);
		}

		std::this_thread::sleep_for(
			std::chrono::milliseconds(GRABBER_PERIOD_MS));
	}

	if (allThreadsMustExit)
	{
		MRPT_LOG_ERROR(
			"[main thread] Ended due to other thread signal to exit "
			"application.");
	}

	// Flush file to disk:
	out_file.close();

	// Wait all threads:
	// ----------------------------
	allThreadsMustExit = true;
	std::this_thread::sleep_for(300ms);

	MRPT_LOG_INFO("Waiting for all threads to close...");

	for (auto& lstThread : lstThreads) lstThread.join();

	MRPT_END
}

void RawlogGrabberApp::dump_verbose_info(
	const mrpt::serialization::CSerializable::Ptr& o) const
{
	// Show GPS mode:
	if (getMinLoggingLevel() != mrpt::system::LVL_DEBUG) return;

	static auto last_t = mrpt::Clock::now();
	const auto t_now = mrpt::Clock::now();
	if (mrpt::system::timeDifference(last_t, t_now) < 1.0) return;
	last_t = t_now;

	if (auto gps = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(o);
		gps)
	{
		dump_GPS_mode_info(*gps);
	}
	else if (auto imu =
				 std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(o);
			 imu)
	{
		dump_IMU_info(*imu);
	}
}

void RawlogGrabberApp::dump_verbose_info(
	const mrpt::obs::CSensoryFrame& sf) const
{
	if (getMinLoggingLevel() != mrpt::system::LVL_DEBUG) return;

	// Show GPS mode:
	mrpt::obs::CObservationGPS::Ptr gps;
	std::size_t idx = 0;
	do
	{
		gps = sf.getObservationByClass<mrpt::obs::CObservationGPS>(idx++);
		if (gps) dump_GPS_mode_info(*gps);
	} while (gps);

	// Show IMU angles:
	auto imu = sf.getObservationByClass<mrpt::obs::CObservationIMU>();
	if (imu) dump_IMU_info(*imu);
}

void RawlogGrabberApp::dump_GPS_mode_info(
	const mrpt::obs::CObservationGPS& o) const
{
	if (o.has_GGA_datum())
	{
		using mrpt::obs::gnss::Message_NMEA_GGA;

		const auto fq = static_cast<int>(
			o.getMsgByClass<Message_NMEA_GGA>().fields.fix_quality);
		MRPT_LOG_DEBUG_STREAM(
			"  GPS mode: " << fq << " label: " << o.sensorLabel);
	}
	{
		std::stringstream ss;
		o.getDescriptionAsText(ss);
		MRPT_LOG_DEBUG_STREAM(ss.str());
	}
}

void RawlogGrabberApp::dump_IMU_info(const mrpt::obs::CObservationIMU& o) const
{
	// Show IMU angles:
	MRPT_LOG_DEBUG_FMT(
		"   IMU angles (degrees): "
		"(yaw,pitch,roll)=(%.06f, %.06f, %.06f)",
		mrpt::RAD2DEG(o.rawMeasurements[mrpt::obs::IMU_YAW]),
		mrpt::RAD2DEG(o.rawMeasurements[mrpt::obs::IMU_PITCH]),
		mrpt::RAD2DEG(o.rawMeasurements[mrpt::obs::IMU_ROLL]));
}

// ------------------------------------------------------
//				SensorThread
// ------------------------------------------------------
void RawlogGrabberApp::SensorThread(std::string sensor_label)
{
	try
	{
		std::string driver_name =
			params.read_string(sensor_label, "driver", "", true);

		auto sensor =
			mrpt::hwdrivers::CGenericSensor::createSensorPtr(driver_name);

		if (!sensor)
			throw std::runtime_error(
				std::string("Class name not recognized: ") + driver_name);

		// Load common & sensor specific parameters:
		sensor->loadConfig(params, sensor_label);

		MRPT_LOG_INFO_STREAM(
			"[thread_" << sensor_label << "] Starting at "
					   << sensor->getProcessRate() << " Hz");

		ASSERT_ABOVE_(sensor->getProcessRate(), 0);

		// For imaging sensors, set external storage directory:
		sensor->setPathForExternalImages(m_rawlog_ext_imgs_dir);

		// Init device:
		sensor->initialize();

		mrpt::system::CRateTimer rate;
		rate.setRate(sensor->getProcessRate());

		while (!allThreadsMustExit)
		{
			// Process
			sensor->doProcess();

			// Get new observations
			mrpt::hwdrivers::CGenericSensor::TListObservations lstObjs;
			sensor->getObservations(lstObjs);

			{
				std::lock_guard<std::mutex> lock(cs_m_global_list_obs);
				m_global_list_obs.insert(lstObjs.begin(), lstObjs.end());
			}

			lstObjs.clear();

			// wait for the process period:
			rate.sleep();
		}

		sensor.reset();

		MRPT_LOG_INFO_FMT("[thread_%s] Closing...", sensor_label.c_str());
	}
	catch (const std::exception& e)
	{
		if (show_sensor_thread_exceptions)
		{
			MRPT_LOG_ERROR_STREAM(
				"Exception in SensorThread:\n"
				<< mrpt::exception_to_str(e));
		}
		allThreadsMustExit = true;
	}
	catch (...)
	{
		if (show_sensor_thread_exceptions)
		{
			MRPT_LOG_ERROR("Untyped exception in SensorThread.");
		}
		allThreadsMustExit = true;
	}
}

void RawlogGrabberApp::process_observations_for_sf(
	const RawlogGrabberApp::TListObservations& list_obs)
{
	using namespace mrpt::obs;

	// -----------------------
	// USE SENSORY-FRAMES
	// -----------------------
	for (auto it = list_obs.begin(); it != list_obs.end(); ++it)
	{
		// If we have an action, save the SF and start a new one:
		if (IS_DERIVED(*it->second, CAction))
		{
			CAction::Ptr act = std::dynamic_pointer_cast<CAction>(it->second);

			(*m_out_arch_ptr) << m_curSF;
			MRPT_LOG_INFO_STREAM(
				"Saved SF with " << m_curSF.size() << " objects.");
			m_curSF.clear();

			CActionCollection acts;
			acts.insert(*act);
			act.reset();

			(*m_out_arch_ptr) << acts;
		}
		else if (IS_CLASS(*it->second, CObservationOdometry))
		{
			CObservationOdometry::Ptr odom =
				std::dynamic_pointer_cast<CObservationOdometry>(it->second);

			auto act = CActionRobotMovement2D::Create();
			act->timestamp = odom->timestamp;

			// Compute the increment since the last reading:
			static CActionRobotMovement2D::TMotionModelOptions odomOpts;
			static CObservationOdometry last_odo;
			static bool last_odo_first = true;

			mrpt::poses::CPose2D odo_incr;
			int64_t lticks_incr, rticks_incr;

			if (last_odo_first)
			{
				last_odo_first = false;
				odo_incr = mrpt::poses::CPose2D(0, 0, 0);
				lticks_incr = rticks_incr = 0;
			}
			else
			{
				odo_incr = odom->odometry - last_odo.odometry;
				lticks_incr =
					odom->encoderLeftTicks - last_odo.encoderLeftTicks;
				rticks_incr =
					odom->encoderRightTicks - last_odo.encoderRightTicks;

				last_odo = *odom;
			}

			// Save as action & dump to file:
			act->computeFromOdometry(odo_incr, odomOpts);

			act->hasEncodersInfo = true;
			act->encoderLeftTicks = lticks_incr;
			act->encoderRightTicks = rticks_incr;

			act->hasVelocities = true;
			act->velocityLocal = odom->velocityLocal;

			(*m_out_arch_ptr) << m_curSF;
			rawlog_saved_objects++;

			MRPT_LOG_INFO_STREAM(
				"Saved SF with " << m_curSF.size() << " objects.");
			m_curSF.clear();

			CActionCollection acts;
			acts.insert(*act);
			act.reset();

			(*m_out_arch_ptr) << acts;
			rawlog_saved_objects++;
		}
		else if (IS_DERIVED(*it->second, CObservation))
		{
			CObservation::Ptr obs =
				std::dynamic_pointer_cast<CObservation>(it->second);

			// First, check if inserting this OBS into the SF would
			// overflow "SF_max_time_span":
			if (m_curSF.size() != 0 &&
				mrpt::system::timeDifference(
					m_curSF.getObservationByIndex(0)->timestamp,
					obs->timestamp) > SF_max_time_span)
			{
				dump_verbose_info(m_curSF);

				// Save and start a new one:
				(*m_out_arch_ptr) << m_curSF;
				rawlog_saved_objects++;

				MRPT_LOG_INFO_STREAM(
					"Saved SF with " << m_curSF.size() << " objects.");
				m_curSF.clear();
			}

			// Now, insert the observation in the SF:
			m_curSF.insert(obs);
		}
		else
			THROW_EXCEPTION(
				"*** ERROR *** Class is not an action or an "
				"observation");
	}
}

void RawlogGrabberApp::process_observations_for_nonsf(
	const RawlogGrabberApp::TListObservations& list_obs)
{
	// ---------------------------
	//  DO NOT USE SENSORY-FRAMES
	// ---------------------------
	for (auto& ob : list_obs)
	{
		auto& obj_ptr = ob.second;
		(*m_out_arch_ptr) << *obj_ptr;
		rawlog_saved_objects++;

		dump_verbose_info(obj_ptr);
	}

	if (!list_obs.empty())
	{
		MRPT_LOG_INFO_STREAM("Saved " << list_obs.size() << " objects.");
	}
}

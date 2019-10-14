/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/system/COutputLogger.h>

namespace mrpt::apps
{
/** RawlogGrabber application wrapper class.
 *
 * \note If the environment variable `MRPT_HWDRIVERS_VERBOSE=1` is defined
 * before calling initialize(), verbosity level will be changed to LVL_DEBUG.
 *
 * \ingroup mrpt_apps_grp
 */
class RawlogGrabberApp : public mrpt::system::COutputLogger
{
   public:
	RawlogGrabberApp();

	/** @name Main API
	 * @{ */

	/** Initializes the application from CLI parameters. Refer to the manpage of
	 * rawlog-grabber. Throws on errors.
	 */
	void initialize(int argc, const char** argv);

	inline void initialize(int argc, char** argv)
	{
		initialize(argc, const_cast<const char**>(argv));
	}

	/** Runs with the current parameter set. Throws on errors. */
	void run();

	/** @} */

	/** @name Parameters and options. See: initialize()
	 * @{ */

	/** If >0, run() will return after this period (in seconds) */
	double run_for_seconds = 0;

	/** Populated in initialize(). Can be replaced or manipulated by the user
	 * after that and before run() to change the parameters loaded from INI
	 * file. */
	mrpt::config::CConfigFileMemory params;

	/** @} */

	/** @name Outputs and result variables
	 * @{ */

	std::string rawlog_filename;  //!< The generated .rawlog file
	std::size_t rawlog_saved_objects;  //!< Counter of saved objects

	/** @} */

	void SensorThread(std::string sensor_label);

   private:
	using TListObservations =
		mrpt::hwdrivers::CGenericSensor::TListObservations;

	void dump_verbose_info(
		const mrpt::serialization::CSerializable::Ptr& o) const;
	void dump_verbose_info(const mrpt::obs::CSensoryFrame& sf) const;
	void dump_GPS_mode_info(const mrpt::obs::CObservationGPS& o) const;
	void dump_IMU_info(const mrpt::obs::CObservationIMU& o) const;

	void process_observations_for_sf(const TListObservations& list_obs);
	void process_observations_for_nonsf(const TListObservations& list_obs);

	TListObservations m_global_list_obs;
	std::mutex cs_m_global_list_obs;

	bool allThreadsMustExit = false;

	/** Directory where to save externally stored images, only for
	 * CCameraSensor's. */
	std::string m_rawlog_ext_imgs_dir;

	mrpt::serialization::CArchive* m_out_arch_ptr = nullptr;

	mrpt::obs::CSensoryFrame m_curSF;
	double SF_max_time_span = 0.25;  // Seconds
};

}  // namespace mrpt::apps

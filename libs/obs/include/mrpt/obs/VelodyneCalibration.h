/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>
#include <vector>

namespace mrpt::obs
{
/** Velodyne calibration data, for usage in mrpt::obs::CObservationVelodyneScan
 *
 * It is mandatory to use some calibration data to convert Velodyne scans into
 * 3D point clouds. Users should
 * normally use the XML files provided by the manufacturer with each scanner,
 * but default calibration files can be
 * loaded with \a VelodyneCalibration::LoadDefaultCalibration().
 *
 * \note New in MRPT 1.4.0
 * \sa CObservationVelodyneScan, CVelodyneScanner
 * \ingroup mrpt_obs_grp
 */
struct VelodyneCalibration
{
	/** Default ctor (leaves all empty) */
	VelodyneCalibration();

	/** Returns true if no calibration has been loaded yet */
	bool empty() const;
	/** Clear all previous contents */
	void clear();

	/** Loads default calibration files for common LIDAR models.
	 * \param[in] lidar_model Valid model names are: `VLP16`, `HDL32`
	 * \return It always return a calibration structure, but it may be empty if
	 * the model name is unknown. See \a empty()
	 * \note Default files can be inspected in `[MRPT_SRC or
	 * /usr]/share/mrpt/config_files/rawlog-grabber/velodyne_default_calib_{*}.xml`
	 */
	static const VelodyneCalibration& LoadDefaultCalibration(
		const std::string& lidar_model);

	/** Loads calibration from file, in the format supplied by the manufacturer.
	 * \return false on any error, true on success */
	bool loadFromXMLFile(const std::string& velodyne_calibration_xml_filename);
	/** Loads calibration from a string containing an entire XML calibration
	 * file. \sa loadFromXMLFile \return false on any error, true on success */
	bool loadFromXMLText(const std::string& xml_file_contents);

// Pragma to ensure we can safely serialize some of these structures
#pragma pack(push, 1)
	struct PerLaserCalib
	{
		double azimuthCorrection{.0}, verticalCorrection{.0},
			distanceCorrection{.0};
		double verticalOffsetCorrection{.0}, horizontalOffsetCorrection{.0};
		double sinVertCorrection{.0}, cosVertCorrection{1.0};
		double sinVertOffsetCorrection{.0}, cosVertOffsetCorrection{1.0};

		PerLaserCalib();
	};
#pragma pack(pop)

	std::vector<PerLaserCalib> laser_corrections;

   private:
	bool internal_loadFromXMLNode(void* node);
};
}  // namespace mrpt::obs

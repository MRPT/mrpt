/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef VelodyneCalibration_H
#define VelodyneCalibration_H

#include <mrpt/utils/core_defs.h>
#include <mrpt/obs/link_pragmas.h>
#include <string>
#include <vector>

namespace mrpt {
namespace obs 
{
	/** Velodyne calibration data, for usage in mrpt::obs::CObservationVelodyneScan
	  *
	  * It is mandatory to use some calibration data to convert Velodyne scans into 3D point clouds. Users should 
	  * normally use the XML files provided by the manufacturer with each scanner, but default calibration files can be 
	  * loaded with \a VelodyneCalibration::LoadDefaultCalibration().
	  *
	  * \note New in MRPT 1.4.0
	  * \sa CObservationVelodyneScan, CVelodyneScanner
	  * \ingroup mrpt_obs_grp
	  */
	struct OBS_IMPEXP VelodyneCalibration
	{
		VelodyneCalibration(); //!< Default ctor (leaves all empty)
		
		bool empty() const; //!< Returns true if no calibration has been loaded yet
		void clear(); //!< Clear all previous contents

		/** Loads default calibration files for common LIDAR models.
		  * \param[in] lidar_model Valid model names are: `VLP16`, `HDL32`
		  * \return It always return a calibration structure, but it may be empty if the model name is unknown. See \a empty()
		  * \note Default files can be inspected in `[MRPT_SRC or /usr]/share/mrpt/config_files/rawlog-grabber/velodyne_default_calib_{*}.xml`
		  */
		static const VelodyneCalibration & LoadDefaultCalibration(const std::string & lidar_model);
		
		/** Loads calibration from file, in the format supplied by the manufacturer. \return false on any error, true on success */
		bool loadFromXMLFile(const std::string & velodyne_calibration_xml_filename);
		/** Loads calibration from a string containing an entire XML calibration file. \sa loadFromXMLFile \return false on any error, true on success */
		bool loadFromXMLText(const std::string & xml_file_contents);

// Pragma to ensure we can safely serialize some of these structures
#pragma pack(push,1)
		struct OBS_IMPEXP PerLaserCalib
		{
			double azimuthCorrection, verticalCorrection, distanceCorrection;
			double verticalOffsetCorrection, horizontalOffsetCorrection;
			double sinVertCorrection, cosVertCorrection;
			double sinVertOffsetCorrection, cosVertOffsetCorrection;
			
			PerLaserCalib();
		};
#pragma pack(pop)

		std::vector<PerLaserCalib> laser_corrections;
	private:
		bool internal_loadFromXMLNode(void *node);

	};
} // End of namespace
} // End of namespace

#endif

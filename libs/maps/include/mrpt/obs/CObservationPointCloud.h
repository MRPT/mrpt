/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservation.h>

namespace mrpt::obs
{
class CObservation3DRangeScan;

/** An observation from any sensor that can be summarized as a pointcloud.
 * The cloud can comprise plain XYZ points, or can include intensity, or RGB
 * data; in particular, the point cloud can be any of the derived classes of
 * mrpt::maps::CPointsMap.
 *
 * The pointcloud has as frame of coordinates the `sensorPose` field, which
 * may match the origin of the vehicle/robot or not.
 *
 * \note This is a mrpt::obs::CObservation class, but it is defined in the
 * mrpt_maps_grp library, so it can use mrpt::maps::CPointsMap.
 *
 * \sa CObservation, mrpt::maps::CPointsMap
 * \ingroup mrpt_maps_grp
 */
class CObservationPointCloud : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationPointCloud, mrpt::obs)

   public:
	enum class ExternalStorageFormat : uint8_t
	{
		None = 0,  //!< is always stored in memory
		MRPT_Serialization,	 //!< Uses mrpt-serialization binary file
		KittiBinFile,  //!< Uses Kitti .bin file format
		PlainTextFile  //!< Plain text, each line has "x y z [i]" coords
	};

   protected:
	ExternalStorageFormat m_externally_stored{ExternalStorageFormat::None};
	std::string m_external_file;

   public:
	CObservationPointCloud() = default;

	explicit CObservationPointCloud(const CObservation3DRangeScan& o);

	/** The pointcloud */
	mrpt::maps::CPointsMap::Ptr pointcloud;

	/** Sensor placement wrt the vehicle/robot.
	 * i.e. A point at (0,0,0) in \a pointcloud is at \a sensorPose wrt the
	 * vehicle.
	 */
	mrpt::poses::CPose3D sensorPose;

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override;
	void setSensorPose(const mrpt::poses::CPose3D& p) override;
	void getDescriptionAsText(std::ostream& o) const override;

	/** @name Delayed-load manual control methods.
		@{ */
	// See base class docs.
	void load() const override;
	void unload() const override;
	/** @} */

	/** \name Point cloud external storage functions
	 * @{ */
	inline bool isExternallyStored() const
	{
		return m_externally_stored != ExternalStorageFormat::None;
	}
	inline const std::string& getExternalStorageFile() const
	{
		return m_external_file;
	}
	void setAsExternalStorage(
		const std::string& fileName, const ExternalStorageFormat fmt);

	void overrideExternalStorageFormatFlag(const ExternalStorageFormat fmt)
	{
		m_externally_stored = fmt;
	}
	/** @} */

};	// End of class def.

}  // namespace mrpt::obs

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

namespace mrpt::hwdrivers
{
/** Open parameters for CMyntEyeCamera
 * \sa CMyntEyeCamera
 * \ingroup mrpt_hwdrivers_grp
 */
struct TMyntEyeCameraParameters : public mrpt::config::CLoadableOptions
{
	std::uint8_t ir_intensity = 4;	//!< IR (Infrared), range [0,10], default 0.

	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& source,
		const std::string& section) override;
};

/** Wrapper on MYNT-EYE-D cameras. Requires MYNT-EYE SDK.
 *
 * \sa mrpt::hwdrivers::CCameraSensor
 * \sa The most generic camera grabber in MRPT: mrpt::hwdrivers::CCameraSensor
 * \ingroup mrpt_hwdrivers_grp
 */
class CMyntEyeCamera
{
   public:
	CMyntEyeCamera(const TMyntEyeCameraParameters& params);
	virtual ~CMyntEyeCamera();

	/** Check whether the camera has been open successfully. */
	bool isOpen() const { return m_bInitialized; }

	/** Grab an image from the opened camera.
	 * \param out_observation The object to be filled with sensed data.
	 *
	 * \return false on any error, true if all go fine.
	 */
	bool getObservation(mrpt::obs::CObservation3DRangeScan& out);

   protected:
	/** Set to false if we could not initialize the camera.
	 */
	bool m_bInitialized = false;

	struct Impl;
	mrpt::pimpl<Impl> m_capture;

	mrpt::img::TCamera m_intrinsics_left, m_intrinsics_right;

};	// End of class

}  // namespace mrpt::hwdrivers

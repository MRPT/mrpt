/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>

#include <mrpt/config.h>

namespace mrpt::hwdrivers
{
/** Grabs from a "Bumblebee" or "Bumblebee2" stereo camera using raw access to
 * the libdc1394 library.
 * Only raw, unrectified images can be captured with this class, which can be
 * manually rectified given
 * correct calibration parameters.
 *
 * See mrpt::hwdrivers::CStereoGrabber_Bumblebee for another class capable of
 * live capture of rectified images using
 * the vendor (PointGreyResearch) Triclops API.
 *
 * Once connected to a camera, you can call `getStereoObservation()` to
 * retrieve the stereo images.
 *
 * \sa You'll probably want to use instead the most generic camera grabber in
 * MRPT: mrpt::hwdrivers::CCameraSensor
 * \ingroup mrpt_hwdrivers_grp
 */
class CStereoGrabber_Bumblebee_libdc1394
{
   public:
	/** Constructor. Parameters have the same meaning as in
	 * CImageGrabber_dc1394::CImageGrabber_dc1394() */
	CStereoGrabber_Bumblebee_libdc1394(
		uint64_t cameraGUID, uint16_t cameraUnit, double frameRate);

	CStereoGrabber_Bumblebee_libdc1394(
		const CStereoGrabber_Bumblebee_libdc1394&) = delete;
	CStereoGrabber_Bumblebee_libdc1394& operator=(
		const CStereoGrabber_Bumblebee_libdc1394&) = delete;

	/** Destructor */
	virtual ~CStereoGrabber_Bumblebee_libdc1394();

	/** Grab stereo images, and return the pair of rectified images.
	 * \param out_observation The object to be filled with sensed data.
	 *
	 * \note The member "CObservationStereoImages::refCameraPose" must be set on
	 * the return of
	 *  this method by the user, since we don't know here the robot physical
	 * structure.
	 *
	 * \return false on any error, true if all go fine.
	 */
	bool getStereoObservation(
		mrpt::obs::CObservationStereoImages& out_observation);

   protected:
	/** The actual capture object used in Linux / Mac. */
	mrpt::hwdrivers::CImageGrabber_dc1394* m_firewire_capture;

	/** If this has been correctly initiated */
	bool m_bInitialized;
};  // End of class
static_assert(
	!std::is_copy_constructible_v<CStereoGrabber_Bumblebee_libdc1394> &&
		!std::is_copy_assignable_v<CStereoGrabber_Bumblebee_libdc1394>,
	"Copy Check");
}  // namespace mrpt::hwdrivers

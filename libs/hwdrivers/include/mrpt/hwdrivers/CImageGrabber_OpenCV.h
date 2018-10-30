/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config.h>

#include <mrpt/obs/CObservationImage.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mrpt::hwdrivers
{
/** These capture types are like their OpenCV equivalents. */
enum TCameraType
{
	CAMERA_CV_AUTODETECT = 0,
	CAMERA_CV_DC1394,
	CAMERA_CV_VFL,
	CAMERA_CV_VFW,
	CAMERA_CV_MIL,
	CAMERA_CV_DSHOW
};

/** Options used when creating an OpenCV capture object
 *  Some options apply to IEEE1394 cameras only.
 * \sa CImageGrabber_OpenCV
 * \ingroup mrpt_hwdrivers_grp
 */
struct TCaptureCVOptions
{
	TCaptureCVOptions() = default;

	/** (All cameras) Capture resolution (0: Leave the default) */
	int frame_width{0}, frame_height{0};
	/** (All cameras) Camera gain (0: Leave the default) */
	double gain{0};
	/** (IEEE1394 cameras) Frame rate for the capture (0: Leave the default). */
	double ieee1394_fps{0};
	/** (IEEE1394 cameras) Whether to grab grayscale images (Default=false). */
	bool ieee1394_grayscale{false};
};

/** A class for grabing images from a "OpenCV"-compatible camera, or from an AVI
 * video file.
 *   See the constructor for the options when opening the camera.
 *
 *  Unless input from AVI files is required, it is recommended to use the more
 * generic class
 *   mrpt::hwdrivers::CCameraSensor.
 *
 * \note This class is only available when compiling MRPT with the flag
 * "MRPT_HAS_OPENCV" defined.
 * \note Some code is based on the class CaptureCamera from the Orocos project.
 * \sa mrpt::hwdrivers::CCameraSensor, CImageGrabber_dc1394
 * \sa The most generic camera grabber in MRPT: mrpt::hwdrivers::CCameraSensor
 * \ingroup mrpt_hwdrivers_grp
 */
class CImageGrabber_OpenCV
{
   protected:
	/** Set to false if we could not initialize the camera.
	 */
	bool m_bInitialized;

	struct Impl;
	mrpt::pimpl<Impl> m_capture;

   public:
	/** Constructor for cameras:
	 * \param cameraIndex Set the camera index, or -1 if it does not matter and
	 * you select AUTODETECT as cameraType.
	 * \param cameraType Can be any value of TCameraType, or
	 * CAMERA_CV_AUTODETECT if there is only one camera.
	 * \param options Capture options, defined in
	 * mrpt::hwdrivers::TCaptureCVOptions. If not provided, all the default
	 * options will be used.
	 */
	CImageGrabber_OpenCV(
		int cameraIndex = -1, TCameraType cameraType = CAMERA_CV_AUTODETECT,
		const TCaptureCVOptions& options = TCaptureCVOptions());

	/** Constructor for AVI files:
	 */
	CImageGrabber_OpenCV(const std::string& AVI_fileName);

	/** Destructor
	 */
	virtual ~CImageGrabber_OpenCV();

	/** Check whether the camera has been open successfully. */
	bool isOpen() const { return m_bInitialized; }
	/** Grab an image from the opened camera.
	 * \param out_observation The object to be filled with sensed data.
	 *
	 * \return false on any error, true if all go fine.
	 */
	bool getObservation(mrpt::obs::CObservationImage& out_observation);

};  // End of class

}  // namespace mrpt::hwdrivers
MRPT_ENUM_TYPE_BEGIN(mrpt::hwdrivers::TCameraType)
MRPT_FILL_ENUM_MEMBER(mrpt::hwdrivers, CAMERA_CV_AUTODETECT);
MRPT_FILL_ENUM_MEMBER(mrpt::hwdrivers, CAMERA_CV_DC1394);
MRPT_FILL_ENUM_MEMBER(mrpt::hwdrivers, CAMERA_CV_VFL);
MRPT_FILL_ENUM_MEMBER(mrpt::hwdrivers, CAMERA_CV_VFW);
MRPT_FILL_ENUM_MEMBER(mrpt::hwdrivers, CAMERA_CV_MIL);
MRPT_FILL_ENUM_MEMBER(mrpt::hwdrivers, CAMERA_CV_DSHOW);
MRPT_ENUM_TYPE_END()

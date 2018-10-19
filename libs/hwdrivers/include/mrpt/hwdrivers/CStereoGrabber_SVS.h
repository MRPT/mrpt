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

namespace mrpt::hwdrivers
{
/** Options used when creating a STOC Videre Design camera capture object
 * \ingroup mrpt_hwdrivers_grp
 */
struct TCaptureOptions_SVS
{
	TCaptureOptions_SVS(
		int _frame_width = 640, int _frame_height = 480, double _framerate = 30,
		int _NDisp = 64, int _Corrsize = 15, int _LR = false, int _Thresh = 10,
		int _Unique = 13, int _Horopter = 0, int _SpeckleSize = 100,
		bool _procesOnChip = true, bool _calDisparity = true);

	/** Capture resolution (Default: 640x480) */
	int frame_width, frame_height;

	/** Indicates if the STOC camera must capture rectified images (Default:
	 * true -> rectified) */
	bool getRectified;
	/** STOC camera frame rate (Default: 30 fps) */
	double framerate;
	/** number of STOC's disparities (Default: 64 ) */
	int m_NDisp;
	int m_Corrsize;  // correlation window size
	int m_LR;  // no left-right check, not available
	int m_Thresh;  // texture filter
	int m_Unique;  // uniqueness filter
	int m_Horopter;
	int m_SpeckleSize;
	bool m_procesOnChip;
	bool m_calDisparity;
};

/** A class for grabing stereo images from a STOC camera of Videre Design
 * NOTE:
 *		- Windows:
 *			- This class is not available.
 *
 *		- Linux:
 *			- This class is only available when compiling MRPT with
 *"MRPT_HAS_SVS".
 *			- You must have the videre design's library.
 *			- Capture will be made in grayscale.
 * 			- The grabber must be launch in root.
 *
 * Once connected to a camera, you can call "getStereoObservation" to retrieve
 *the Disparity images.
 *
 * \sa You'll probably want to use instead the most generic camera grabber in
 *MRPT: mrpt::hwdrivers::CCameraSensor
 * \ingroup mrpt_hwdrivers_grp
 */
class CStereoGrabber_SVS
{
   protected:
	/** If this has been correctly initiated */
	bool m_bInitialized{false};

	void* m_videoObject{nullptr};  // svsVideoImages*
	void* m_stereoImage{nullptr};  // svsStereoImage*
	void* m_disparityParams{nullptr};  // svsDisparityParams*
	void* m_processObject;  // svsStereoProcess
	unsigned int m_resolutionX;
	unsigned int m_resolutionY;

	unsigned char* m_ptrMat;

	bool m_status;
	bool m_initialized;
	bool m_procesOnChip;
	bool m_calDisparity;

   private:
   public:
	TCaptureOptions_SVS m_options;

	/** Constructor: */
	CStereoGrabber_SVS(
		int cameraIndex = 0,
		const TCaptureOptions_SVS& options = TCaptureOptions_SVS());

	CStereoGrabber_SVS(const CStereoGrabber_SVS&) = delete;
	CStereoGrabber_SVS& operator=(const CStereoGrabber_SVS&) = delete;

	/** Destructor */
	virtual ~CStereoGrabber_SVS();

	/** Grab stereo images, and return the pair of rectified images.
	 * \param out_observation The object to be filled with sensed data.
	 *
	 *  NOTICE: (1) That the member "CObservationStereoImages::refCameraPose"
	 * must be
	 *                set on the return of this method, since we don't know here
	 * the robot physical structure.
	 *          (2) The images are already rectified.
	 *
	 * \return false on any error, true if all go fine.
	 */
	bool getStereoObservation(
		mrpt::obs::CObservationStereoImages& out_observation);

};  // End of class
static_assert(
	!std::is_copy_constructible_v<CStereoGrabber_SVS> &&
		!std::is_copy_assignable_v<CStereoGrabber_SVS>,
	"Copy Check");
}  // namespace mrpt::hwdrivers

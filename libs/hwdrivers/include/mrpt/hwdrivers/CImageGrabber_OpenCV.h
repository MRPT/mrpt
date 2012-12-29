/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CImageGrabber_OpenCV_H
#define CImageGrabber_OpenCV_H

#include <mrpt/config.h>
#include <mrpt/hwdrivers/link_pragmas.h>

#include <mrpt/slam/CObservationImage.h>
#include <mrpt/utils/safe_pointers.h>


namespace mrpt
{
	namespace hwdrivers
	{
		/** These capture types are like their OpenCV equivalents. */
		enum TCameraType
		{
			CAMERA_CV_AUTODETECT = 0,
			CAMERA_CV_DC1394,
			CAMERA_CV_VFL,
			CAMERA_CV_VFW,
			CAMERA_CV_MIL,
			CAMERA_CV_DSHOW //!< Valid only with OpenCV >= 1.1.0
		};

		/** Options used when creating an OpenCV capture object
		  *  Some options apply to IEEE1394 cameras only.
		  * \sa CImageGrabber_OpenCV
		  * \ingroup mrpt_hwdrivers_grp
		  */
		struct TCaptureCVOptions
		{
			TCaptureCVOptions() :
				frame_width(0), frame_height(0),
				gain(0),
				ieee1394_fps(0),
				ieee1394_grayscale(false)
			{}

			int		frame_width,frame_height;	//!< (All cameras) Capture resolution (0: Leave the default)
			double  gain;			//!< (All cameras) Camera gain (0: Leave the default)
			double 	ieee1394_fps;	//!< (IEEE1394 cameras) Frame rate for the capture (0: Leave the default).
			bool    ieee1394_grayscale;	//!< (IEEE1394 cameras) Whether to grab grayscale images (Default=false).
		};

		/** A class for grabing images from a "OpenCV"-compatible camera, or from an AVI video file.
		  *   See the constructor for the options when opening the camera.
		  *
		  *  Unless input from AVI files is required, it is recommended to use the more generic class
		  *   mrpt::hwdrivers::CCameraSensor.
		  *
		  * \note This class is only available when compiling MRPT with the flag "MRPT_HAS_OPENCV" defined.
		  * \note Some code is based on the class CaptureCamera from the Orocos project.
		  * \sa mrpt::hwdrivers::CCameraSensor, CImageGrabber_dc1394
		  * \sa The most generic camera grabber in MRPT: mrpt::hwdrivers::CCameraSensor
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CImageGrabber_OpenCV
		{
		protected:
			/** Set to false if we could not initialize the camera.
			  */
			bool	m_bInitialized;

			/** Internal use:
			  */
			mrpt::utils::void_ptr_noncopy /*CvCapture*/ m_capture;

		public:
			/** Constructor for cameras:
			 * \param cameraIndex Set the camera index, or -1 if it does not matter and you select AUTODETECT as cameraType.
			 * \param cameraType Can be any value of TCameraType, or CAMERA_CV_AUTODETECT if there is only one camera.
			 * \param options Capture options, defined in mrpt::hwdrivers::TCaptureCVOptions. If not provided, all the default options will be used.
			 */
			CImageGrabber_OpenCV(
				int cameraIndex = -1,
				TCameraType cameraType = CAMERA_CV_AUTODETECT,
				const TCaptureCVOptions &options = TCaptureCVOptions()
				 );

			/** Constructor for AVI files:
			*/
			CImageGrabber_OpenCV( const std::string &AVI_fileName );

			/** Destructor
			*/
			virtual ~CImageGrabber_OpenCV( );

			/** Check whether the camera has been open successfully. */
			bool isOpen() const
			{
				return m_bInitialized;
			}

			/** Grab an image from the opened camera.
			 * \param out_observation The object to be filled with sensed data.
			 *
			 * \return false on any error, true if all go fine.
			*/
			bool  getObservation( mrpt::slam::CObservationImage &out_observation);


		};	// End of class

	} // End of NS
} // End of NS


#endif

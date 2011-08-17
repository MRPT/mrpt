/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CStereoGrabber_SVS_H
#define CStereoGrabber_SVS_H

#include <mrpt/slam/CObservationStereoImages.h>
#include <mrpt/hwdrivers/link_pragmas.h>


namespace mrpt
{
	namespace hwdrivers
        {


		/** Options used when creating a STOC Videre Design camera capture object
		  * \ingroup mrpt_hwdrivers_grp
		  */
		struct HWDRIVERS_IMPEXP TCaptureOptions_SVS
		{
			TCaptureOptions_SVS(int _frame_width=640, int _frame_height=480 , double _framerate = 30, int _NDisp= 64,
                                                                int _Corrsize=15, int _LR = false, int _Thresh = 10, int _Unique = 13, int _Horopter = 0,int _SpeckleSize = 100,bool _procesOnChip = true,bool _calDisparity = true);

			int	frame_width, frame_height;	//!< Capture resolution (Default: 640x480)

			bool getRectified;				//!< Indicates if the STOC camera must capture rectified images (Default: true -> rectified)
			double framerate;                               //!< STOC camera frame rate (Default: 30 fps)
			int                                 m_NDisp;	//!< number of STOC's disparities (Default: 64 )
			int                                 m_Corrsize; // correlation window size
			int                                 m_LR;	// no left-right check, not available
			int                                 m_Thresh;	// texture filter
			int                                 m_Unique;	// uniqueness filter
			int                                 m_Horopter;
			int                                 m_SpeckleSize;
                        bool                                m_procesOnChip;
                        bool                                m_calDisparity;

		};

		/** A class for grabing stereo images from a STOC camera of Videre Design
		  * NOTE:
		  *		- Windows:
		  *			- This class is not available.
		  *
		  *		- Linux:
		  *			- This class is only available when compiling MRPT with "MRPT_HAS_SVS".
		  *			- You must have the videre design's library.
		  *			- Capture will be made in grayscale.
		  * 			- The grabber must be launch in root.
		  *
		  * Once connected to a camera, you can call "getStereoObservation" to retrieve the Disparity images.
		  *
		  * \sa You'll probably want to use instead the most generic camera grabber in MRPT: mrpt::hwdrivers::CCameraSensor
		  * \ingroup mrpt_hwdrivers_grp
		  */
        class HWDRIVERS_IMPEXP  CStereoGrabber_SVS : public mrpt::utils::CUncopiable
		{
		protected:
			bool			m_bInitialized;					//!< If this has been correctly initiated

			void                     *m_videoObject;	// svsVideoImages*
			void                     *m_stereoImage;	// svsStereoImage*
			void                 *m_disparityParams;	// svsDisparityParams*
                        void                   *m_processObject;        // svsStereoProcess
			unsigned int                        m_resolutionX;
			unsigned int                        m_resolutionY;

			unsigned char                      *m_ptrMat;

			bool                                m_status;
			bool                                m_initialized;
                        bool                                m_procesOnChip;
                        bool                                m_calDisparity;


		private:

		public:

			TCaptureOptions_SVS	m_options;

			/** Constructor: */
			CStereoGrabber_SVS( int cameraIndex = 0, const TCaptureOptions_SVS &options = TCaptureOptions_SVS() );

			/** Destructor */
			virtual ~CStereoGrabber_SVS(void);

			/** Grab stereo images, and return the pair of rectified images.
			 * \param out_observation The object to be filled with sensed data.
			 *
			 *  NOTICE: (1) That the member "CObservationStereoImages::refCameraPose" must be
			 *                set on the return of this method, since we don't know here the robot physical structure.
			 *          (2) The images are already rectified.
			 *
			 * \return false on any error, true if all go fine.
			*/
			bool  getStereoObservation( mrpt::slam::CObservationStereoImages &out_observation );

		};	// End of class

	} // End of NS
} // End of NS


#endif

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
#ifndef CCImageGrabber_dc1394
#define CCImageGrabber_dc1394

#include <mrpt/config.h>

#include <mrpt/slam/CObservationImage.h>
#include <mrpt/slam/CObservationStereoImages.h>

#include <mrpt/hwdrivers/link_pragmas.h>

namespace mrpt
{
	namespace hwdrivers
	{

		typedef enum {
			FRAMERATE_1_875= 32,
			FRAMERATE_3_75,
			FRAMERATE_7_5,
			FRAMERATE_15,
			FRAMERATE_30,
			FRAMERATE_60,
			FRAMERATE_120,
			FRAMERATE_240
		} grabber_dc1394_framerate_t;

		typedef enum {
			COLOR_CODING_MONO8= 352,
			COLOR_CODING_YUV411,
			COLOR_CODING_YUV422,
			COLOR_CODING_YUV444,
			COLOR_CODING_RGB8,
			COLOR_CODING_MONO16
		} grabber_dc1394_color_coding_t;


		/** Options used when creating an dc1394 capture object
		  *   All but the frame size, framerate, and color_coding can be changed dynamically by CImageGrabber_dc1394::changeCaptureOptions
		  * \sa CImageGrabber_dc1394
		  * \ingroup mrpt_hwdrivers_grp
		  */
		struct TCaptureOptions_dc1394
		{
			TCaptureOptions_dc1394() :
				frame_width		(640),
				frame_height	(480),
				framerate		(FRAMERATE_15),
				color_coding	(COLOR_CODING_YUV422),
				mode7			(-1),
				shutter			(-1),
				gain			(-1),
				gamma			(-1),
				brightness		(-1),
				exposure		(-1),
				sharpness		(-1),
				white_balance	(-1),
				deinterlace_stereo(false)
			{}

			int		frame_width,frame_height;	//!< Capture resolution (Default: 640x480)
			grabber_dc1394_framerate_t		framerate;
			grabber_dc1394_color_coding_t	color_coding;

			int		mode7;			//!< -1: Normal mode, i>=0: use MODE7_i, then frame_width/height and color_coding are ignored.

			int		shutter;		//!< Shutter, -1=default:Do not change
			int		gain;			//!< Gain, -1=default:Do not change
			int		gamma;			//!< Gamma, -1=default:Do not change
			int		brightness;		//!< Brightness, -1=default:Do not change
			int		exposure;		//!< Exposure, -1=default:Do not change
			int		sharpness;		//!< Sharpness, -1=default:Do not change
			int		white_balance;	//!< White balance, -1=default:Do not change
			bool  	deinterlace_stereo;	//!< For stereo cameras (eg PR Bumblebee)
		};

		/** A class for grabing images from a IEEE1394 (Firewire) camera using the libdc1394-2 library.
		  *   See the constructor for the options when opening the camera. Notice that you may have
		  *    to carefully set the resolution, framerate and color_mode. See the verbose parameter of
		  *    the constructor, which can display a list of supported modes in your camera.
		  *
		  *  This class is able to manage any Firewire cameras, including Stereo or multi-cameras in general,
		  *    so this can be used to open the Bumblebee camera (not tested yet).
		  *
		  * A static method (CImageGrabber_dc1394::enumerateCameras) is provided to enumerate all existing cameras and their properties. It can be used
		  *  to find the GUID of the desired camera, then open it at the constructor.
		  *
		  * \note This class requires MRPT compiled with "libdc1394-2" (Only works under Linux for now) and "opencv".
		  * \note In Linux you may need to execute "chmod 666 /dev/video1394/ * " and "chmod 666 /dev/raw1394" for allowing any user R/W access to firewire cameras.
		  * \sa The most generic camera grabber in MRPT: mrpt::hwdrivers::CCameraSensor
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CImageGrabber_dc1394
		{
		protected:
			/** Set to false if we could not initialize the camera.
			  */
			bool	m_bInitialized;

			/** Internal use: */
			void /* dc1394_t * */ 		*m_dc1394_lib_context;
			void /* dc1394camera_t* */ 	*m_dc1394camera;
			int							m_desired_mode;


			TCaptureOptions_dc1394		m_options;

		public:
			/** Constructor: open an ieee1394 camera.
			 * \param cameraGUID Set the camera GUID to open, or 0 to open the first found camera.
			 * \param cameraUnit (Ignored if cameraGUID=0). The number of camera to open within the device with the given GUID: In a stereo camera this may be 0 or 1. Normally this is 0.
			 * \param options Capture options, defined in mrpt::hwdrivers::TCaptureOptions_dc1394.
			 * \param verbose  Displays a lot of information about the camera to be open and its valid video modes.
			 */
			CImageGrabber_dc1394(
				uint64_t	cameraGUID = 0,
				uint16_t	cameraUnit = 0,
				const TCaptureOptions_dc1394 &options = TCaptureOptions_dc1394(),
				bool  verbose = false
				);

			/** Destructor
			*/
			virtual ~CImageGrabber_dc1394( );

			/** Check whether the camera has been open successfully. */
			bool isOpen() const
			{
				return m_bInitialized;
			}

			/** Changes the capture properties (brightness, gain, shutter, etc)
			  * The frame size, framerate, and color_coding fields in options are ignored since they can be only set at construction time.
			  * \return false on error
			  */
			bool changeCaptureOptions( const TCaptureOptions_dc1394 &options  );


			/** Grab an image from the opened camera (for monocular cameras).
			 * \param out_observation The object to be filled with sensed data.
			 *
			 * \return false on any error, true if all go fine.
			*/
			bool  getObservation( mrpt::slam::CObservationImage &out_observation);

			/** Grab an image from the opened camera (for stereo cameras).
			 * \param out_observation The object to be filled with sensed data.
			 *
			 * \return false on any error, true if all go fine.
			*/
			bool  getObservation( mrpt::slam::CObservationStereoImages &out_observation);

			/** Used in enumerateCameras */
			struct TCameraInfo
			{
				uint64_t             guid;
				int                  unit;
				uint32_t             unit_spec_ID;
				uint32_t             unit_sw_version;
				uint32_t             unit_sub_sw_version;
				uint32_t             command_registers_base;
				uint32_t             unit_directory;
				uint32_t             unit_dependent_directory;
				uint64_t             advanced_features_csr;
				uint64_t             PIO_control_csr;
				uint64_t             SIO_control_csr;
				uint64_t             strobe_control_csr;
				uint64_t             format7_csr[16];
				int                  iidc_version;
				std::string          vendor;
				std::string          model;
				uint32_t             vendor_id;
				uint32_t             model_id;
				bool                 bmode_capable;
				bool                 one_shot_capable;
				bool                 multi_shot_capable;
				bool                 can_switch_on_off;
				bool                 has_vmode_error_status;
				bool                 has_feature_error_status;
				int                  max_mem_channel;
			};

			typedef std::list<TCameraInfo> TCameraInfoList;

			/** Generates a list with the information on all the existing (Firewire) cameras in the system.
			  * \exception std::runtime_error On any error calling libdc1394.
			  */
			static void enumerateCameras( TCameraInfoList &out_list );


		};	// End of class

	} // End of NS
} // End of NS


#endif

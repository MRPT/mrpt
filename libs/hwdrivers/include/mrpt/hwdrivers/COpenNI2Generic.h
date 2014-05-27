/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef mrpt_COpenNI2Generic_H
#define mrpt_COpenNI2Generic_H

#include <mrpt/slam/CObservation3DRangeScan.h>

#include <mrpt/hwdrivers/link_pragmas.h>


namespace mrpt
{
	namespace hwdrivers
	{
		/** An abstract class for accessing OpenNI2 compatible sensors.
		  * This class permits to access several sensors simultaneously. The same options (resolution, fps, etc.) are used for every sensor.
		  *
		  *  More references to read:
		  *		- http://http://www.openni.org/
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP  COpenNI2Generic
		{

		 public:

			COpenNI2Generic();	 //!< Default ctor
			~COpenNI2Generic();	 //!< Default ctor

      /** Get a list of the connected OpenNI2 sensors.
      */
      int getConnectedDevices();

			/** Kill the OpenNI2 driver
			  */
      void kill();

			/** The main data retrieving function, to be called after calling loadConfig() and initialize().
			  *  \param out_img The output retrieved RGB image (only if there_is_obs=true).
			  *  \param timestamp The timestamp of the capture (only if there_is_obs=true).
			  *  \param there_is_obs If set to false, there was no new observation.
			  *  \param hardware_error True on hardware/comms error.
			  *  \param sensor_id The index of the sensor accessed.
			  *
			  */
			void getNextFrameRGB(
				mrpt::utils::CImage &rgb_img,
        uint64_t &timestamp,
				bool &there_is_obs,
				bool &hardware_error ,
				unsigned sensor_id = 0);

			/** The main data retrieving function, to be called after calling loadConfig() and initialize().
			  *  \param depth_img The output retrieved depth image (only if there_is_obs=true).
			  *  \param timestamp The timestamp of the capture (only if there_is_obs=true).
			  *  \param there_is_obs If set to false, there was no new observation.
			  *  \param hardware_error True on hardware/comms error.
			  *  \param sensor_id The index of the sensor accessed.
			  *
			  */
			void getNextFrameD(
				mrpt::math::CMatrix &depth_img,
        uint64_t &timestamp,
				bool &there_is_obs,
				bool &hardware_error ,
				unsigned sensor_id = 0);

			/** The main data retrieving function, to be called after calling loadConfig() and initialize().
			  *  \param out_obs The output retrieved observation (only if there_is_obs=true).
			  *  \param there_is_obs If set to false, there was no new observation.
			  *  \param hardware_error True on hardware/comms error.
			  *  \param sensor_id The index of the sensor accessed.
			  *
			  * \sa doProcess
			  */
			void getNextFrameRGBD(
				mrpt::slam::CObservation3DRangeScan &out_obs,
				bool &there_is_obs,
				bool &hardware_error ,
				unsigned sensor_id = 0);

			/** @name Sensor parameters (alternative to \a loadConfig ) and manual control
			    @{ */

//			/** Set the stream mode (resolution, fps and pixel format) for the OpenNI2 device */
//      bool setONI2StreamMode(openni::VideoStream& stream, int w, int h, int fps, openni::PixelFormat format);
//
			/** Set the RGB stream mode (resolution, fps and pixel format) for the OpenNI2 device */
      bool initONI2RGBStream(unsigned sensor_id, int w, int h, int fps, void* pFormat);

			/** Set the Depth stream mode (resolution, fps and pixel format) for the OpenNI2 device */
      bool initONI2DepthStream(unsigned sensor_id, int w, int h, int fps, void* pFormat);

			/** Try to open the camera (set all the parameters before calling this) - users may also call initialize(), which in turn calls this method.
			  *  Raises an exception upon error.
			  * \exception std::exception A textual description of the error.
			  */
			void open(unsigned sensor_id = 0);

			bool isOpen(const unsigned sensor_id) const; //!< Whether there is a working connection to the sensor

			/** Close the conection to the sensor (not need to call it manually unless desired for some reason,
			  * since it's called at destructor) */
			void close(unsigned sensor_id = 0);

			/** The amount of available devices at initialization */
			unsigned numDevices;

			/** @} */

		protected:

			/** List the number of devices connected */
			void* deviceListPtr;  // Opaque pointer to "openni::Array<openni::DeviceInfo>"

			/** The index of the chosen devices */
			static std::vector<unsigned> vOpenDevices;
//			static std::vector<COpenNI2Generic*> vOpenDevices;

			/** A vector with pointers to the available devices */
			std::vector<void*>	vp_devices; // Opaque pointer to "openni::Device"

			/** The same options (width, height and fps) are set for all the sensors. (This could be changed if necessary) */
      int width, height;
      float fps;
      void *rgb_pFormat, *depth_pFormat; // These parameters will be necesary if several pixel formats are used by this class.

			/** A vector with pointers to the rgb streams of the available devices */
			std::vector<void*> vp_depth_stream; // Opaque pointer to "openni::VideoStream"
			std::vector<void*> vp_rgb_stream; // Opaque pointer to "openni::VideoStream"

			/** A vector with pointers to the frame output structures */
			std::vector<void*> vp_frame_depth, vp_frame_rgb;	// Opaque pointers to "openni::VideoFrameRef"

			/** Check whether the OpenNI2 device has RGB camera or not */
			bool  m_has_color;

			bool  m_grab_image, m_grab_depth, m_grab_3D_points ; //!< Default: all true

		};	// End of class

	} // End of NS

} // End of NS

#endif

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef mrpt_COpenNI2Generic_H
#define mrpt_COpenNI2Generic_H

#include <mrpt/obs/CObservation3DRangeScan.h>

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
			COpenNI2Generic();	 //!< Default ctor (width=640, height=480, fps=30)
			COpenNI2Generic(int width, int height, float fps = 30.0f, bool open_streams_now = true); //!< Ctor. \sa start()

			~COpenNI2Generic();	 //!< Default ctor

			/** The main data retrieving function, to be called after calling loadConfig() and initialize().
			  *  \param out_img The output retrieved RGB image (only if there_is_obs=true).
			  *  \param timestamp The timestamp of the capture (only if there_is_obs=true).
			  *  \param there_is_obs If set to false, there was no new observation.
			  *  \param hardware_error True on hardware/comms error.
			  *  \param sensor_id The index of the sensor accessed.  */
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
			  *  \param sensor_id The index of the sensor accessed. */
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
				mrpt::obs::CObservation3DRangeScan &out_obs,
				bool &there_is_obs,
				bool &hardware_error ,
				unsigned sensor_id = 0);

			/** @name Open/Close device methods
			    @{ */
			/** Try to open the camera (all the parameters [resolution,fps,...] must be set before calling this) - users may also call initialize(), which in turn calls this method.
			  *  Raises an exception upon error.
			  * \exception std::exception A textual description of the error.
			  */
			void open(unsigned sensor_id = 0);

			/** Open a set of RGBD devices specified by their serial number. Raises an exception when the demanded serial numbers
			*  are not among the connected devices. This function also fills a vector with the serial numbers of the connected
			*  OpenNI2 sensors (this requires openning the sensors which are still closed to read their serial)
			*/
			unsigned int openDevicesBySerialNum(const std::set<unsigned>& vSerialRequired);

			/** Open a RGBD device specified by its serial number. This method is a wrapper for
			*  openDevicesBySerialNum(const std::set<unsigned>& vSerialRequired)
			*  This method requires to open the sensors which are still closed to read their serial.
			*/
			unsigned int openDeviceBySerial(const unsigned int SerialRequired);

			bool getDeviceIDFromSerialNum(const unsigned int SerialRequired, int& sensor_id) const; //!< Get the ID of the device corresponding to 'SerialRequired'.
			bool start(); //!< Open all sensor streams (normally called automatically at constructor, no need to call it manually). \return false on error \sa kill() to close
			void kill(); //!< Kill the OpenNI2 driver \sa start()
			bool isOpen(const unsigned sensor_id) const; //!< Whether there is a working connection to the sensor
			void close(unsigned sensor_id = 0); //!< Close the connection to the sensor (no need to call it manually unless desired for some reason, since it's called at destructor
			int getNumDevices()const; //!< The number of available devices at initialization
			int getConnectedDevices(); //!< Get a list of the connected OpenNI2 sensors.

			/** @} */

			void setVerbose(bool verbose);
			bool isVerbose() const;

			bool  getColorSensorParam(mrpt::utils::TCamera& param, unsigned sensor_id = 0) const;
			bool  getDepthSensorParam(mrpt::utils::TCamera& param, unsigned sensor_id = 0) const;

		protected:

			/** The list of available devices */
			class HWDRIVERS_IMPEXP CDevice;
			static std::vector<stlplus::smart_ptr<CDevice> > vDevices;
			static int                        numInstances;

			/** A vector with the serial numbers of the available devices */
			std::vector<int>	vSerialNums;

			/** The same options (width, height and fps) are set for all the sensors. (This could be changed if necessary) */
			int   m_width, m_height;
			float m_fps;
			int   m_rgb_format, m_depth_format;
			bool  m_verbose;
			void  showLog(const std::string& message)const;
			/** The data that the RGBD sensors can return */
			bool  m_grab_image, m_grab_depth, m_grab_3D_points ; //!< Default: all true
		};	// End of class
	} // End of NS
} // End of NS
#endif

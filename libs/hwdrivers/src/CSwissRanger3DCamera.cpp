/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CSwissRanger3DCamera.h>

#include <mrpt/system/string_utils.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CSwissRanger3DCamera,mrpt::hwdrivers)


#if MRPT_HAS_SWISSRANGE
	#ifdef MRPT_OS_WINDOWS
		#define WIN32_LEAN_AND_MEAN
		#include <windows.h>
	#endif

	#include <libMesaSR.h>

	#ifdef MRPT_OS_LINUX
		#include <termios.h>
		#include <stdio.h>
		#include <unistd.h>

		#include <linux/sockios.h>
		#include <asm/ioctls.h>
		#include <sys/select.h>
	#endif
#endif



/*-------------------------------------------------------------
		ctor
 -------------------------------------------------------------*/
CSwissRanger3DCamera::CSwissRanger3DCamera()  :
	m_sensorPoseOnRobot	(),
	m_save_3d		(true),
	m_save_range_img(true),
	m_save_intensity_img(true),
	m_save_confidence(false),

	m_enable_img_hist_equal(false),
	m_enable_median_filter (true),
	m_enable_mediancross_filter(false),
	m_enable_conv_gray (false),
	m_enable_denoise_anf (true),

	m_open_from_usb		(true),
	m_usb_serial		(0),
	m_ip_address		("192.168.2.14"),
	m_rows			(0),
	m_cols			(0),
	m_cam_serial_num(0),
	m_maxRange		(5),
	m_preview_window(false)
{
	m_sensorLabel = "3DCAM";

	// Default params: Obtained from a SR4000 with 0.004px avr reprojection error.
	m_cameraParams.ncols = 176;
	m_cameraParams.nrows = 144;
	m_cameraParams.intrinsicParams(0,0) = 262.9201; // fx
	m_cameraParams.intrinsicParams(1,1) = 262.9218; // fy
	m_cameraParams.intrinsicParams(0,2) = 87.99958; // cx
	m_cameraParams.intrinsicParams(1,2) = 68.99957; // cy
	m_cameraParams.dist[0] = -8.258543e-01;
	m_cameraParams.dist[1] =  6.561022e-01;
	m_cameraParams.dist[2] =  2.699818e-06;
	m_cameraParams.dist[3] = -3.263559e-05;
	m_cameraParams.dist[4] = 0;

#if !MRPT_HAS_SWISSRANGE
	THROW_EXCEPTION("MRPT was compiled without support for SwissRanger 3D cameras! Rebuild it.")
#endif
}

/*-------------------------------------------------------------
			dtor
 -------------------------------------------------------------*/
CSwissRanger3DCamera::~CSwissRanger3DCamera()
{
	this->close();
}


/*-------------------------------------------------------------
	Modified A-law compression algorithm for uint16_t -> uint8_t
	 The original method uses signed int16_t. It's being tuned for
	 what we want here...
 -------------------------------------------------------------*/
static char ALawCompressTable[128] =
{
     1,1,2,2,3,3,3,3,
     4,4,4,4,4,4,4,4,
     5,5,5,5,5,5,5,5,
     5,5,5,5,5,5,5,5,
     6,6,6,6,6,6,6,6,
     6,6,6,6,6,6,6,6,
     6,6,6,6,6,6,6,6,
     6,6,6,6,6,6,6,6,
     7,7,7,7,7,7,7,7,
     7,7,7,7,7,7,7,7,
     7,7,7,7,7,7,7,7,
     7,7,7,7,7,7,7,7,
     7,7,7,7,7,7,7,7,
     7,7,7,7,7,7,7,7,
     7,7,7,7,7,7,7,7,
     7,7,7,7,7,7,7,7
};

uint8_t table_16u_to_8u [0x10000];
bool    table_16u_to_8u_init = false;

unsigned char LinearToALawSample(uint16_t sample)
{
     if (sample >= 0x200)
     {
          int exponent =  ALawCompressTable[(sample >> 9) & 0x7F];
          int mantissa = (sample >> (exponent + 3) ) & 0x1F;
          return ((exponent << 5) | mantissa);
     }
     else
     {
          return (sample >> 4);
     }
}

void do_init_table_16u_to_8u()
{
	for (unsigned int i=0;i<0x10000;i++)
		table_16u_to_8u[i] = LinearToALawSample(i);
}


/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void CSwissRanger3DCamera::initialize()
{
	if (!open())
		THROW_EXCEPTION("Error opening SwissRanger 3D camera.")
}

/** This method will be invoked at a minimum rate of "process_rate" (Hz)
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void CSwissRanger3DCamera::doProcess()
{
	using namespace mrpt::obs;

	bool	thereIs, hwError;

	CObservation3DRangeScanPtr newObs = CObservation3DRangeScan::Create();

	getNextObservation( *newObs, thereIs, hwError );

	if (hwError)
	{
		m_state = ssError;
	    THROW_EXCEPTION("Couldn't communicate to the SwissRanger 3D camera!");
	}

	if (thereIs)
	{
		m_state = ssWorking;

		appendObservation( newObs );
	}
}

/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
*  \exception This method must throw an exception with a descriptive message if some critical parameter is missing or has an invalid value.
*/
void  CSwissRanger3DCamera::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
	using mrpt::utils::DEG2RAD;

	m_sensorPoseOnRobot.setFromValues(
		configSource.read_float(iniSection,"pose_x",0),
		configSource.read_float(iniSection,"pose_y",0),
		configSource.read_float(iniSection,"pose_z",0),
		DEG2RAD( configSource.read_float(iniSection,"pose_yaw",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_pitch",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_roll",0) )
		);

	m_preview_window = configSource.read_bool(iniSection,"preview_window",m_preview_window);

	m_save_3d = configSource.read_bool(iniSection,"save_3d",m_save_3d);
	m_save_range_img= configSource.read_bool(iniSection,"save_range_img",m_save_range_img);
	m_save_intensity_img= configSource.read_bool(iniSection,"save_intensity_img",m_save_intensity_img);
	m_save_confidence= configSource.read_bool(iniSection,"save_confidence",m_save_confidence);

	m_enable_img_hist_equal = configSource.read_bool(iniSection,"enable_img_hist_equal",m_enable_img_hist_equal);
	m_enable_median_filter = configSource.read_bool(iniSection,"enable_median_filter",m_enable_median_filter);
	m_enable_mediancross_filter = configSource.read_bool(iniSection,"enable_mediancross_filter",m_enable_mediancross_filter);
	m_enable_conv_gray = configSource.read_bool(iniSection,"enable_conv_gray",m_enable_conv_gray);
	m_enable_denoise_anf = configSource.read_bool(iniSection,"enable_denoise_anf",m_enable_denoise_anf);

	m_open_from_usb = configSource.read_bool(iniSection,"open_from_usb",m_open_from_usb);
	m_usb_serial = configSource.read_uint64_t(iniSection,"usb_serial",m_usb_serial);
	m_ip_address = configSource.read_string(iniSection,"ip_address",m_ip_address);

	m_external_images_format = mrpt::system::trim( configSource.read_string( iniSection, "external_images_format", m_external_images_format ) );
	m_external_images_jpeg_quality = configSource.read_int( iniSection, "external_images_jpeg_quality", m_external_images_jpeg_quality );

	try
	{
		m_cameraParams.loadFromConfigFile(iniSection, configSource);
	}
	catch(std::exception &)
	{
		// If there's some missing field, just keep the default values.
	}
}



bool CSwissRanger3DCamera::getMesaLibVersion(std::string &out_version) const
{
#if MRPT_HAS_SWISSRANGE
	unsigned short version[4];
	if (0!=SR_GetVersion(version))
		return false;

	out_version = format("%d.%d.%d.%d",version[3],version[2],version[1],version[0]);
	return true;
#else
	MRPT_UNUSED_PARAM(out_version);
	return false;
#endif
}

bool CSwissRanger3DCamera::isOpen() const
{
	return m_cam != NULL;
}

bool CSwissRanger3DCamera::open()
{
#if MRPT_HAS_SWISSRANGE
	if (isOpen())
		close();

	SRCAM cam;
	if (m_open_from_usb)
	{
		if (SR_OpenUSB(&cam, this->m_usb_serial)<=0)
			return false;
	}
	else
	{
		if (SR_OpenETH(&cam, this->m_ip_address.c_str() )<=0)
			return false;
	}
	m_cam = cam;

	// Initialization code:
	m_rows = SR_GetRows(cam);
	m_cols = SR_GetCols(cam);

	m_cam_serial_num = SR_ReadSerial(cam);

	// Deduce max range from frequency:
	const ModulationFrq fr = SR_GetModulationFrequency(cam);
	switch(fr)
	{
		case MF_40MHz: m_maxRange = 3.75; break;
		case MF_30MHz: m_maxRange = 5; break;
		case MF_21MHz: m_maxRange = 7.14; break;
		case MF_20MHz: m_maxRange = 7.5; break;
		case MF_19MHz: m_maxRange = 7.89; break;
		case MF_60MHz: m_maxRange = 2.5; break;
		case MF_15MHz: m_maxRange = 10; break;
		case MF_10MHz: m_maxRange = 15; break;
		case MF_29MHz: m_maxRange = 5.17; break;
		case MF_31MHz: m_maxRange = 4.84; break;
		case MF_14_5MHz: m_maxRange = 10.34; break;
		case MF_15_5MHz: m_maxRange = 9.68; break;

		default:  m_maxRange = 5.0; break;
	}

	SR_SetTimeout(cam, 1000 /* ms */);

	internal_resendParamsToCamera();

	return true;
#else
	return false;
#endif
}

void CSwissRanger3DCamera::close()
{
#if MRPT_HAS_SWISSRANGE
	if (m_cam)
		SR_Close(SRCAM(m_cam));
	m_cam = NULL;
#endif
}

void CSwissRanger3DCamera::internal_resendParamsToCamera() const
{
#if MRPT_HAS_SWISSRANGE
	if (!isOpen()) return;

	SR_SetMode(SRCAM(m_cam),
		AM_COR_FIX_PTRN  | // turns on fix pattern noise correction <b>this should always be enabled for good distance measurement</b>
		(m_enable_median_filter ? AM_MEDIAN : 0 ) |
		(m_enable_conv_gray ? AM_CONV_GRAY : 0 ) |
		(m_enable_denoise_anf ? AM_DENOISE_ANF : 0 ) |
		(m_save_confidence ? AM_CONF_MAP : 0 ) |
		(m_enable_mediancross_filter ? AM_MEDIANCROSS : 0 )
		);
#endif
}


/** The main data retrieving function, to be called after calling loadConfig() and initialize().
  *  \param out_obs The output retrieved observation (only if there_is_obs=true).
  *  \param there_is_obs If set to false, there was no new observation.
  *  \param hardware_error True on hardware/comms error.
  *
  * \sa doProcess
  */
void CSwissRanger3DCamera::getNextObservation(
	mrpt::obs::CObservation3DRangeScan &_out_obs,
	bool &there_is_obs,
	bool &hardware_error )
{
	there_is_obs=false;
	hardware_error = false;
#if MRPT_HAS_SWISSRANGE

	int bytesRx = SR_Acquire( SRCAM(m_cam) );
	if (!bytesRx)
	{
		cerr << "[CSwissRanger3DCamera] Zero bytes read from the camera." << endl;
		hardware_error = true;
		return;
	}

	// Extract images:
	ImgEntry *imgEntryArray;
	const int nImgs=SR_GetImageList( SRCAM(m_cam),&imgEntryArray);

	if (!nImgs)
	{
		cerr << "[CSwissRanger3DCamera] Error: no images in image list." << endl;
		hardware_error = true;
		return;
	}

	// Initialize the output observation:
	obs::CObservation3DRangeScan obs;
	obs.sensorLabel     = m_sensorLabel;
	obs.sensorPose		= m_sensorPoseOnRobot;
	obs.maxRange  = m_maxRange;
	obs.stdError  = 0.01f;

	// Process each of the images:
	for (int i=0;i<nImgs;i++)
	{
		const ImgEntry *img = imgEntryArray+i;
		switch (img->imgType)
		{
			// Ranges:
			case ImgEntry::IT_DISTANCE:
			{
				if (this->m_save_range_img)
				{
					ASSERT_(img->dataType==ImgEntry::DT_USHORT)
					obs.hasRangeImage = true;
					obs.range_is_depth = false;

					// Convert data from uint16_t to float ranges:
					//  (0x0000, 0xFFFF)  -> (0m, 5m)
					const float K = obs.maxRange / 0xFFFF;
					obs.rangeImage.setSize(img->height, img->width);

					const uint16_t *data_ptr = reinterpret_cast<const uint16_t *>(img->data);

					for (size_t y=0;y<img->height;y++)
						for (size_t x=0;x<img->width;x++)
							obs.rangeImage.set_unsafe(y,x, K * (*data_ptr++) );
				}

				if (this->m_save_3d)
				{
					ASSERT_(img->dataType==ImgEntry::DT_USHORT)
					obs.hasPoints3D  = true;

					const size_t N = img->height * img->width;
					obs.points3D_x.resize(N);
					obs.points3D_y.resize(N);
					obs.points3D_z.resize(N);

					// Swap XYZ order, so:
					//  SwissRange   -> MRPT
					//      Z            X
					//      X            Y
					//      Y            Z
					SR_CoordTrfFlt(SRCAM(m_cam),
						&obs.points3D_y[0],  // X
						&obs.points3D_z[0],  // Y
						&obs.points3D_x[0],  // Z
						sizeof(float), sizeof(float), sizeof(float));
				}
			}
			break;

			// Intensity:
			case ImgEntry::IT_AMPLITUDE:
			{
				if (this->m_save_intensity_img)
				{
					ASSERT_(img->dataType==ImgEntry::DT_USHORT)
					obs.hasIntensityImage = true;

					// Make sure the camera params are there:
					m_cameraParams.scaleToResolution(img->width,img->height);
					obs.cameraParams = m_cameraParams;

					// make sure the modified A-law Look Up Table is up-to-date:
					if (!table_16u_to_8u_init)
					{
						table_16u_to_8u_init = true;
						do_init_table_16u_to_8u();
					}

					obs.intensityImage.resize(img->width,img->height,1, true);

					const uint16_t *data_ptr = reinterpret_cast<const uint16_t *>(img->data);
					for (size_t y=0;y<img->height;y++)
					{
						uint8_t *row = obs.intensityImage.get_unsafe(0,y,0);
						for (size_t x=0;x<img->width;x++)
							// Convert 16u -> 8u
							(*row++) = table_16u_to_8u[ *data_ptr++ ];
					}

					if (m_enable_img_hist_equal)
						obs.intensityImage.equalizeHistInPlace();

					// Save as external image file??
					if (!m_path_for_external_images.empty())
					{
						const string filName = fileNameStripInvalidChars( trim(m_sensorLabel) ) + format( "_INT_%f.%s", (double)timestampTotime_t( obs.timestamp ), m_external_images_format.c_str() );
						obs.intensityImage.saveToFile( m_path_for_external_images + string("/") +filName, m_external_images_jpeg_quality );
						obs.intensityImage.setExternalStorage( filName );
					}
				}
			}
			break;

			// Confidence:
			case ImgEntry::IT_CONF_MAP:
			{
				if (this->m_save_confidence)
				{
					ASSERT_(img->dataType==ImgEntry::DT_USHORT)
					obs.hasConfidenceImage  = true;

					obs.confidenceImage.resize(img->width,img->height,1, true);

					const uint16_t *data_ptr = reinterpret_cast<const uint16_t *>(img->data);
					for (size_t y=0;y<img->height;y++)
					{
						uint8_t *row = obs.confidenceImage.get_unsafe(0,y,0);
						for (size_t x=0;x<img->width;x++)
							(*row++) = (*data_ptr++) >> 8;	// Convert 16u -> 8u
					}

					// Save as external image file??
					if (!m_path_for_external_images.empty())
					{
						const string filName = fileNameStripInvalidChars( trim(m_sensorLabel) ) + format( "_CONF_%f.%s", (double)timestampTotime_t( obs.timestamp ), m_external_images_format.c_str() );
						obs.confidenceImage.saveToFile( m_path_for_external_images + string("/") +filName, m_external_images_jpeg_quality );
						obs.confidenceImage.setExternalStorage( filName );
					}

				}
			}
			break;

			default:
				break;
		}
	}

	// Save the observation to the user's object:
	_out_obs.swap(obs);

	there_is_obs = true;

	// preview in real-time?
	if (m_preview_window)
	{
		if ( _out_obs.hasRangeImage )
		{
			static int decim = 0;
			if (++decim>10)
			{
				decim=0;
				if (!m_win_range)	{ m_win_range = mrpt::gui::CDisplayWindow::Create("Preview RANGE"); m_win_range->setPos(5,5); }

				mrpt::utils::CImage  img;
				// Normalize the image
				math::CMatrixFloat  range2D = _out_obs.rangeImage;
				range2D*= 1.0/m_maxRange;
				img.setFromMatrix(range2D);
				m_win_range->showImage(img);
			}
		}
		if ( _out_obs.hasIntensityImage )
		{
			static int decim = 0;
			if (++decim>10)
			{
				decim=0;
				if (!m_win_int)		{ m_win_int = mrpt::gui::CDisplayWindow::Create("Preview INTENSITY"); m_win_int->setPos(300,5); }
				m_win_int->showImage(_out_obs.intensityImage );
			}
		}
	}
	else
	{
		if (m_win_range) m_win_range.clear();
		if (m_win_int) m_win_int.clear();
	}

	return;
#else
	MRPT_UNUSED_PARAM(_out_obs);
	MRPT_UNUSED_PARAM(there_is_obs);
	MRPT_UNUSED_PARAM(hardware_error);
#endif
}


/* -----------------------------------------------------
				setPathForExternalImages
----------------------------------------------------- */
void CSwissRanger3DCamera::setPathForExternalImages( const std::string &directory )
{
	return;
	// Ignore for now. It seems performance is better grabbing everything
	// to a single big file than creating hundreds of smaller files per second...

	if (!mrpt::system::createDirectory( directory ))
	{
		THROW_EXCEPTION_CUSTOM_MSG1("Error: Cannot create the directory for externally saved images: %s",directory.c_str() )
	}
	m_path_for_external_images = directory;
}


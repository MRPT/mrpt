/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/hwdrivers/CDUO3DCamera.h>

#include <mrpt/otherlibs/do_opencv_includes.h> 

// We must define & store OpenCV-specific data like this in the .cpp, we don't want to force users to need opencv headers:
struct TDUOParams
{
#if MRPT_HAS_OPENCV
	cv::Mat m_rectify_map_left_x;
	cv::Mat m_rectify_map_left_y;
	cv::Mat m_rectify_map_right_x;
	cv::Mat m_rectify_map_right_y;
#endif
};
std::map<const mrpt::hwdrivers::TCaptureOptions_DUO3D*,TDUOParams> duo_params;

// duo3d header files
#if MRPT_HAS_DUO3D
#	include <DUOLib.h>
//#	ifdef _MSC_VER  // this pragma only works for MSVC:
//#		pragma comment(lib, "DUOLib.lib")
//#	endif
#endif
MRPT_TODO("FIXME: Try to remove the pragma lib above ==> TARGET_LINK_LIBRARIES() in hwdrivers/CMakeLists.txt, so it can work seamlessly on Linux in the future")

// m_duo: Opaque pointer to DUO3D's "DUOInstance":
#define M_DUO_PTR    ( reinterpret_cast<DUOInstance*>(m_duo))
#define M_DUO_VALUE  (*M_DUO_PTR)


using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;

// opencv header files and namespaces
#if MRPT_HAS_OPENCV
	using namespace cv;
#endif

TCaptureOptions_DUO3D::TCaptureOptions_DUO3D() :
	m_img_width(640),
	m_img_height(480),
	m_fps(30),
	m_exposure(50),
	m_led(25),
	m_gain(0),
	m_capture_imu(false),
	m_capture_rectified(false),
	m_calibration_from_file(true),
	m_rectify_map_filename(""),
	m_intrinsic_filename(""),
	m_extrinsic_filename(""),
	m_stereo_camera(TStereoCamera())
{
	duo_params[this]; // Create
}

TCaptureOptions_DUO3D::~TCaptureOptions_DUO3D()
{
	duo_params.erase(this);// Remove entry
}

TCaptureOptions_DUO3D::TYMLReadResult TCaptureOptions_DUO3D::m_rectify_map_from_yml( const string & _file_name )
{
#if MRPT_HAS_OPENCV
	const string file_name = _file_name.empty() ? m_rectify_map_filename : _file_name;

	TDUOParams & dp = duo_params[this];

	string aux = mrpt::system::extractFileName( file_name );
	const size_t found = aux.find( mrpt::format("_R%dx%d_",this->m_img_width,this->m_img_height) );
	if( found == std::string::npos )
	{
		dp.m_rectify_map_left_x  =
		dp.m_rectify_map_left_y  =
		dp.m_rectify_map_right_x =
		dp.m_rectify_map_right_y = cv::Mat();
		return yrr_NAME_NON_CONSISTENT;
	}
	// read file
	FileStorage fs( file_name , FileStorage::READ);
	fs["R0X"] >> dp.m_rectify_map_left_x;
	fs["R0Y"] >> dp.m_rectify_map_left_y;
	fs["R1X"] >> dp.m_rectify_map_right_x;
	fs["R1Y"] >> dp.m_rectify_map_right_y;

	if( dp.m_rectify_map_left_x.size() == Size(0,0)  || dp.m_rectify_map_left_y.size() == Size(0,0) ||
		dp.m_rectify_map_right_x.size() == Size(0,0) || dp.m_rectify_map_right_y.size() == Size(0,0) )
	return yrr_EMPTY;

	return yrr_OK;
#else
    THROW_EXCEPTION("This function requires building with OpenCV support")
#endif
}

TCaptureOptions_DUO3D::TYMLReadResult TCaptureOptions_DUO3D::m_camera_ext_params_from_yml( const string & _file_name )
{
#if MRPT_HAS_OPENCV
	const string file_name = _file_name.empty() ? m_extrinsic_filename : _file_name;

	// this will look for R and t matrixes
	cv::Mat aux_mat;
	bool empty = false;
	string aux = mrpt::system::extractFileName( file_name );
	const size_t found = aux.find( mrpt::format("_R%dx%d_",this->m_img_width,this->m_img_height) );
	if( found == std::string::npos )
	{
		m_stereo_camera.rightCameraPose = CPose3DQuat();
		return yrr_NAME_NON_CONSISTENT;
	}
	// read file
	FileStorage fs( file_name , FileStorage::READ);
	CMatrixDouble33 M;
	CMatrixDouble13 t;
	CMatrixDouble44 M2;

	// rotation matrix
	fs["R"] >> aux_mat;
	if( aux_mat.size() == Size(3,3) )
	{
		for(size_t k1 = 0; k1 < 3; ++k1)
			for(size_t k2 = 0; k2 < 3; ++k2)
				M(k1,k2) = aux_mat.at<double>(k1,k2);
	}
	else
	{
		empty = true;
		m_stereo_camera.rightCameraPose = CPose3DQuat();
	}

	// translation
	fs["T"] >> aux_mat;
	if( aux_mat.size() == Size(1,3) )
	{
		t(0,0) = aux_mat.at<double>(0,0)/1000.0;
		t(0,1) = aux_mat.at<double>(1,0)/1000.0;
		t(0,2) = aux_mat.at<double>(2,0)/1000.0;
	}
	else
	{
		empty = true;
		m_stereo_camera.rightCameraPose = CPose3DQuat();
	}

	if( empty ) return yrr_EMPTY;

	m_stereo_camera.rightCameraPose = CPose3DQuat( CPose3D(M,t) );
	return yrr_OK;
#else
    THROW_EXCEPTION("This function requires building with OpenCV support")
#endif
}

TCaptureOptions_DUO3D::TYMLReadResult TCaptureOptions_DUO3D::m_camera_int_params_from_yml( const string & _file_name )
{
#if MRPT_HAS_OPENCV
	const string file_name = _file_name.empty() ? m_intrinsic_filename : _file_name;

	// this will look for M1, D1, M2 and D2 matrixes
	cv::Mat aux_mat;
	bool empty = false;
	string aux = mrpt::system::extractFileName( file_name );
	const size_t found = aux.find( mrpt::format("_R%dx%d_",this->m_img_width,this->m_img_height) );
	if( found == std::string::npos )
	{
		m_stereo_camera.leftCamera.intrinsicParams.zeros();
		m_stereo_camera.leftCamera.dist.zeros();
		m_stereo_camera.rightCamera.intrinsicParams.zeros();
		m_stereo_camera.rightCamera.dist.zeros();

		return yrr_NAME_NON_CONSISTENT;
	}
	// read file
	FileStorage fs( file_name , FileStorage::READ);

	// left camera
	fs["M1"] >> aux_mat;
	if( aux_mat.size() == Size(0,0) )
	{
		empty = true;
		m_stereo_camera.leftCamera.intrinsicParams.zeros();
	}
	m_stereo_camera.leftCamera.setIntrinsicParamsFromValues( aux_mat.at<double>(0,0), aux_mat.at<double>(1,1), aux_mat.at<double>(0,2), aux_mat.at<double>(1,2) );

	fs["D1"] >> aux_mat;
	if( aux_mat.size() == Size(0,0) )
	{
		empty = true;
		m_stereo_camera.leftCamera.dist.zeros();
	}
	m_stereo_camera.leftCamera.setDistortionParamsFromValues( aux_mat.at<double>(0,0), aux_mat.at<double>(0,1), aux_mat.at<double>(0,2), aux_mat.at<double>(0,3), aux_mat.at<double>(0,4) );

	fs["M2"] >> aux_mat;
	if( aux_mat.size() == Size(0,0) )
	{
		empty = true;
		m_stereo_camera.rightCamera.intrinsicParams.zeros();
	}
	m_stereo_camera.rightCamera.setIntrinsicParamsFromValues( aux_mat.at<double>(0,0), aux_mat.at<double>(1,1), aux_mat.at<double>(0,2), aux_mat.at<double>(1,2) );

	fs["D2"] >> aux_mat;
	if( aux_mat.size() == Size(0,0) )
	{
		empty = true;
		m_stereo_camera.rightCamera.dist.zeros();
	}
	m_stereo_camera.rightCamera.setDistortionParamsFromValues( aux_mat.at<double>(0,0), aux_mat.at<double>(0,1), aux_mat.at<double>(0,2), aux_mat.at<double>(0,3), aux_mat.at<double>(0,4) );

	return empty ? yrr_EMPTY : yrr_OK;
#else
    THROW_EXCEPTION("This function requires building with OpenCV support")
#endif
}

void TCaptureOptions_DUO3D::loadOptionsFrom(
	const mrpt::utils::CConfigFileBase & configSource,
	const std::string & iniSection,
	const std::string & prefix )
{
	m_img_width				= configSource.read_int(iniSection,"image_width",m_img_width);
	m_img_height			= configSource.read_int(iniSection,"image_height",m_img_height);

	m_fps					= configSource.read_float(iniSection,"fps",m_fps);
	m_exposure				= configSource.read_float(iniSection,"exposure",m_exposure);
	m_led					= configSource.read_float(iniSection,"led",m_led);
	m_gain					= configSource.read_float(iniSection,"gain",m_gain);

	m_capture_rectified		= configSource.read_bool(iniSection,"capture_rectified",m_capture_rectified);
	m_capture_imu			= configSource.read_bool(iniSection,"capture_imu",m_capture_imu);
	m_calibration_from_file = configSource.read_bool(iniSection,"calibration_from_file",m_calibration_from_file);

	if( m_calibration_from_file )
	{
		m_intrinsic_filename = configSource.read_string(iniSection,"intrinsic_filename", m_intrinsic_filename);
		m_extrinsic_filename = configSource.read_string(iniSection,"extrinsic_filename", m_extrinsic_filename);
		m_stereo_camera.leftCamera.ncols = m_stereo_camera.rightCamera.ncols = m_img_width;
		m_stereo_camera.leftCamera.nrows = m_stereo_camera.rightCamera.nrows = m_img_height;
	}
	else
		m_stereo_camera.loadFromConfigFile( "DUO3D", configSource );

	if( m_capture_rectified )
	{
		m_rectify_map_filename = configSource.read_string(iniSection,"rectify_map_filename", m_rectify_map_filename);
	} // end-capture-rectified
}

#if MRPT_HAS_DUO3D
static void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{
	CDUO3DCamera* obj = static_cast<CDUO3DCamera*>(pUserData);
	obj->setDataFrame( reinterpret_cast<void*>(pFrameData) );
	SetEvent( reinterpret_cast<HANDLE>( obj->getEvent() ) );
}
#endif

/** Default constructor. */
CDUO3DCamera::CDUO3DCamera() :
	m_options(TCaptureOptions_DUO3D()),
	m_duo(NULL)
{
#if MRPT_HAS_DUO3D
	m_duo = new DUOInstance[1];
	M_DUO_VALUE = NULL;  // m_duo = NULL;

	m_pframe_data = NULL;
	m_evFrame = CreateEvent(NULL, FALSE, FALSE, NULL);
#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_DUO3D'=OFF, so this class cannot be used.");
#endif
} // end-constructor

/** Custom initialization and start grabbing constructor. */
CDUO3DCamera::CDUO3DCamera( const TCaptureOptions_DUO3D & options ) :
	m_duo(NULL)
{
#if MRPT_HAS_DUO3D
	m_duo = new DUOInstance[1];
	M_DUO_VALUE = NULL;  // m_duo = NULL;

	m_pframe_data = NULL;
	m_evFrame = CreateEvent(NULL, FALSE, FALSE, NULL);
	this->open( options );
#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_DUO3D'=OFF, so this class cannot be used.");
#endif
} // end-constructor

/** Destructor */
CDUO3DCamera::~CDUO3DCamera()
{
#if MRPT_HAS_DUO3D
	this->close();
	if (m_duo) { delete[] M_DUO_PTR; m_duo=NULL;}
#endif
} // end-destructor

/** Tries to open the camera with the given options. Raises an exception on error. \sa close() */
void CDUO3DCamera::open( const TCaptureOptions_DUO3D & options, const bool startCapture )
{
#if MRPT_HAS_DUO3D
	if( M_DUO_VALUE ) this->close();
	this->m_options = options;

	if( this->m_options.m_calibration_from_file )
	{
		// get intrinsic parameters
		TCaptureOptions_DUO3D::TYMLReadResult res = this->m_options.m_camera_int_params_from_yml();
		if( res == TCaptureOptions_DUO3D::yrr_EMPTY )
			cout << "[CDUO3DCamera] Warning: Some of the intrinsic params could not be read (size=0). Check file content." << endl;
		else if( res == TCaptureOptions_DUO3D::yrr_NAME_NON_CONSISTENT )
			cout << "[CDUO3DCamera] Warning: Intrinsic params filename is not consistent with image size. Are you using the correct calibration?. All params set to zero." << endl;

		// get extrinsic parameters
		res = this->m_options.m_camera_ext_params_from_yml();
		if( res == TCaptureOptions_DUO3D::yrr_EMPTY )
			cout << "[CDUO3DCamera] Warning: Some of the extrinsic params could not be read (size!=3x3). Check file content." << endl;
		else if( res == TCaptureOptions_DUO3D::yrr_NAME_NON_CONSISTENT )
			cout << "[CDUO3DCamera] Warning: Extrinsic params filename is not consistent with image size. Are you using the correct calibration?. All params set to zero." << endl;

		if( this->m_options.m_capture_rectified )
		{
			if( !this->m_options.m_rectify_map_filename.empty() )
			{
				// read "rectify_map"
				res = this->m_options.m_rectify_map_from_yml();
					if( res == TCaptureOptions_DUO3D::yrr_EMPTY )
						cout << "[CDUO3DCamera] Warning: Rectification map could not be read (size==0). Check file content." << endl;
					else if( res == TCaptureOptions_DUO3D::yrr_NAME_NON_CONSISTENT )
						cout << "[CDUO3DCamera] Warning: Rectification map filename is not consistent with image size. Are you using the correct calibration?. Rectification map set to zero." << endl;

				this->m_options.m_capture_rectified = res == TCaptureOptions_DUO3D::yrr_OK;

				//const size_t area = this->m_options.dp.m_rectify_map_left_x.size().area();
				TDUOParams & dp = duo_params[&(this->m_options)];
				const size_t area = dp.m_rectify_map_left_x.size().area();
				vector<int16_t> v_left_x(area), v_right_x(area);
				vector<uint16_t> v_left_y(area), v_right_y(area);

				for( size_t k = 0; k < area; ++k )
				{
					v_left_x[k] = dp.m_rectify_map_left_x.at<int16_t>(k);
					v_left_y[k] = dp.m_rectify_map_left_y.at<uint16_t>(k);
					v_right_x[k] = dp.m_rectify_map_right_x.at<int16_t>(k);
					v_right_y[k] = dp.m_rectify_map_right_y.at<uint16_t>(k);

					//v_left_x[k] = this->m_options.dp.m_rectify_map_left_x.at<int16_t>(k);
					//v_left_y[k] = this->m_options.dp.m_rectify_map_left_y.at<uint16_t>(k);
					//v_right_x[k] = this->m_options.dp.m_rectify_map_right_x.at<int16_t>(k);
					//v_right_y[k] = this->m_options.dp.m_rectify_map_right_y.at<uint16_t>(k);
				}
				m_rectify_map.setFromCamParams( this->m_options.m_stereo_camera );
				//m_rectify_map.setRectifyMaps( v_left_x, v_left_y, v_right_x, v_right_y );
			}
			else
			{
				cout << "[CDUO3DCamera] Warning: Calibration information is set to be read from a file, but the file was not specified. Unrectified images will be grabbed." << endl;
			}
		} // end-if
	} // end-if
	else if( this->m_options.m_capture_rectified )
	{
		m_rectify_map.setFromCamParams( this->m_options.m_stereo_camera );
	}

	// Find optimal binning parameters for given (width, height)
	// This maximizes sensor imaging area for given resolution
	int binning = DUO_BIN_NONE;
	if(this->m_options.m_img_width <= 752/2)
		binning += DUO_BIN_HORIZONTAL2;
	if(this->m_options.m_img_height <= 480/4)
		binning += DUO_BIN_VERTICAL4;
	else if(this->m_options.m_img_height <= 480/2)
		binning += DUO_BIN_VERTICAL2;

	// Check if we support given resolution (width, height, binning, fps)
	DUOResolutionInfo ri;
	if(!EnumerateResolutions(&ri, 1, this->m_options.m_img_width, this->m_options.m_img_height, binning, this->m_options.m_fps))
		THROW_EXCEPTION( "[CDUO3DCamera] Error: Resolution not supported." )

	if(!OpenDUO(& M_DUO_VALUE ))  // was: m_duo
		THROW_EXCEPTION( "[CDUO3DCamera] Error: Camera could not be opened." )

	// Get and print some DUO parameter values
	char name[260], version[260];
	GetDUODeviceName(M_DUO_VALUE,name);
	GetDUOFirmwareVersion(M_DUO_VALUE,version);
	cout << "[CDUO3DCamera::open] DUO3DCamera name: " << name << " (v" << version << ")" << endl;

	// Set selected resolution
	SetDUOResolutionInfo(M_DUO_VALUE,ri);

	// Set selected camera settings
	SetDUOExposure(M_DUO_VALUE,m_options.m_exposure);
	SetDUOGain(M_DUO_VALUE,m_options.m_gain);
	SetDUOLedPWM(M_DUO_VALUE,m_options.m_led);

	// Start capture
	if( startCapture )
	{
		if(!StartDUO(M_DUO_VALUE, DUOCallback, reinterpret_cast<void*>(this)))
			THROW_EXCEPTION( "[CDUO3DCamera] Error: Camera could not be started." )
	}

#endif
} // end-open

/*-------------------------------------------------------------
						getObservations
-------------------------------------------------------------*/
void  CDUO3DCamera::getObservations(
	CObservationStereoImages		& outObservation_img,
	CObservationIMU					& outObservation_imu,
	bool							& there_is_img,
	bool							& there_is_imu )
{
#if MRPT_HAS_DUO3D
	there_is_img	= false;
	there_is_imu	= false;

	m_pframe_data = m_get_duo_frame();
    if(!m_pframe_data) return;

	// -----------------------------------------------
	//   Extract the observation:
	// -----------------------------------------------
	outObservation_img.timestamp = outObservation_imu.timestamp = mrpt::system::now();

	outObservation_img.setStereoCameraParams(m_options.m_stereo_camera);
	outObservation_img.imageLeft.loadFromMemoryBuffer(
			m_options.m_img_width,
			m_options.m_img_height,
			false,
			(unsigned char*)reinterpret_cast<PDUOFrame>(m_pframe_data)->leftData);

	outObservation_img.imageRight.loadFromMemoryBuffer(
			m_options.m_img_width,
			m_options.m_img_height,
			false,
			(unsigned char*)reinterpret_cast<PDUOFrame>(m_pframe_data)->rightData);

	if( this->m_options.m_capture_rectified )
		m_rectify_map.rectify( outObservation_img );

	there_is_img = true;

	if( this->m_options.m_capture_imu )
	{
		if( !reinterpret_cast<PDUOFrame>(m_pframe_data)->accelerometerPresent )
		{
			cout << "[CDUO3DCamera] Warning: This device does not provide IMU data. No IMU observations will be created." << endl;
			this->m_options.m_capture_imu = false;
		}
		else
		{
			// Accelerometer data
			for(size_t k = 0; k < 3; ++k)
			{
				outObservation_imu.rawMeasurements[k] = reinterpret_cast<PDUOFrame>(m_pframe_data)->accelData[k];
				outObservation_imu.dataIsPresent[k] = true;
			}

			// Gyroscopes data
			for(size_t k = 0; k < 3; ++k)
			{
				outObservation_imu.rawMeasurements[k+3] = reinterpret_cast<PDUOFrame>(m_pframe_data)->gyroData[k];
				outObservation_imu.dataIsPresent[k+3] = true;
			}
			there_is_imu = true;
		}// end else
	} // end-imu-info
#endif
}

/** Closes DUO camera */
void CDUO3DCamera::close()
{
#if MRPT_HAS_DUO3D
	if( !M_DUO_VALUE ) return;
	StopDUO( M_DUO_VALUE );
	CloseDUO( M_DUO_VALUE );
	M_DUO_VALUE = NULL;
#endif
} // end-close

// Waits until the new DUO frame is ready and returns it
void* CDUO3DCamera::m_get_duo_frame()
{
#if MRPT_HAS_DUO3D
	if( M_DUO_VALUE == NULL ) return 0;
	if(WaitForSingleObject( m_evFrame, 1000 ) == WAIT_OBJECT_0) return m_pframe_data;
	else return NULL;
#else
	return NULL; // return something to silent compiler warnings.
#endif
}

void CDUO3DCamera::m_set_exposure(float value)
{
#if MRPT_HAS_DUO3D
	if( M_DUO_VALUE == NULL ) return;
	SetDUOExposure( M_DUO_VALUE, value );
#endif
}

void CDUO3DCamera::m_set_gain(float value)
{
#if MRPT_HAS_DUO3D
	if( M_DUO_VALUE == NULL ) return;
	SetDUOGain( M_DUO_VALUE, value );
#endif
}

void CDUO3DCamera::m_set_led(float value)
{
#if MRPT_HAS_DUO3D
	if( M_DUO_VALUE == NULL ) return;
	SetDUOLedPWM( M_DUO_VALUE, value );
#endif
}

/** Queries the DUO3D Camera firmware version */
bool CDUO3DCamera::queryVersion(std::string version, bool printOutVersion)
{
#if MRPT_HAS_DUO3D
	version = std::string(GetLibVersion());
	if( printOutVersion ) std::cout << "DUO3D Camera library version: " << version << std::endl;
	return true;
#else
	return false;
#endif
}

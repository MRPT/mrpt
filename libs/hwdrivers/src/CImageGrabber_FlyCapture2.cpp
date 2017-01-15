/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CImageGrabber_FlyCapture2.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/datetime.h>

#if MRPT_HAS_FLYCAPTURE2
	#include <FlyCapture2.h>
	using namespace FlyCapture2;
#endif
#if MRPT_HAS_TRICLOPS
	#include <triclops.h>
	#include <fc2triclops.h>
	using namespace Fc2Triclops;
#endif

#if MRPT_HAS_OPENCV
	#include <opencv2/core/core.hpp>
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/imgproc/imgproc.hpp>
	#include <opencv2/imgproc/imgproc_c.h>
#endif

#define CHECK_FC2_ERROR(_err) { if (_err != PGRERROR_OK) { THROW_EXCEPTION_CUSTOM_MSG1("FlyCapture2 error:\n%s",_err.GetDescription()) } }
#define CHECK_TRICLOPS_ERROR(_err)	\
{  if( _err != TriclopsErrorOk ) \
	{ THROW_EXCEPTION_CUSTOM_MSG1("Triclops Error:\n'%s'",triclopsErrorToString( _err )) }	\
}
#define FC2_CAM  reinterpret_cast<FlyCapture2::Camera*>(m_camera)
#define FC2_CAM_INFO  reinterpret_cast<FlyCapture2::CameraInfo*>(m_camera_info)
#define FC2_BUF_IMG   reinterpret_cast<FlyCapture2::Image*>(m_img_buffer)
#define TRI_CONTEXT	reinterpret_cast<TriclopsContext*>(m_triclops)

using namespace mrpt::hwdrivers;
using namespace std;

#if MRPT_HAS_FLYCAPTURE2
// Declare tables to convert strings to their #define values:
template <typename T>
struct fc2_str_val {
	const char* str;
	T val;
};

const fc2_str_val<VideoMode> fc2_VideoMode_table[] = {
	{ "VIDEOMODE_160x120YUV444",VIDEOMODE_160x120YUV444},
	{ "VIDEOMODE_320x240YUV422",VIDEOMODE_320x240YUV422},
	{ "VIDEOMODE_640x480YUV411",VIDEOMODE_640x480YUV411},
	{ "VIDEOMODE_640x480YUV422",VIDEOMODE_640x480YUV422},
	{ "VIDEOMODE_640x480RGB",VIDEOMODE_640x480RGB},
	{ "VIDEOMODE_640x480Y8",VIDEOMODE_640x480Y8},
	{ "VIDEOMODE_640x480Y16",VIDEOMODE_640x480Y16},
	{ "VIDEOMODE_800x600YUV422",VIDEOMODE_800x600YUV422},
	{ "VIDEOMODE_800x600RGB",VIDEOMODE_800x600RGB},
	{ "VIDEOMODE_800x600Y8",VIDEOMODE_800x600Y8},
	{ "VIDEOMODE_800x600Y16",VIDEOMODE_800x600Y16},
	{ "VIDEOMODE_1024x768YUV422",VIDEOMODE_1024x768YUV422},
	{ "VIDEOMODE_1024x768RGB",VIDEOMODE_1024x768RGB},
	{ "VIDEOMODE_1024x768Y8",VIDEOMODE_1024x768Y8},
	{ "VIDEOMODE_1024x768Y16",VIDEOMODE_1024x768Y16},
	{ "VIDEOMODE_1280x960YUV422",VIDEOMODE_1280x960YUV422},
	{ "VIDEOMODE_1280x960RGB",VIDEOMODE_1280x960RGB},
	{ "VIDEOMODE_1280x960Y8",VIDEOMODE_1280x960Y8},
	{ "VIDEOMODE_1280x960Y16",VIDEOMODE_1280x960Y16},
	{ "VIDEOMODE_1600x1200YUV422",VIDEOMODE_1600x1200YUV422},
	{ "VIDEOMODE_1600x1200RGB",VIDEOMODE_1600x1200RGB},
	{ "VIDEOMODE_1600x1200Y8",VIDEOMODE_1600x1200Y8},
	{ "VIDEOMODE_1600x1200Y16",VIDEOMODE_1600x1200Y16},
	{ "VIDEOMODE_FORMAT7", VIDEOMODE_FORMAT7}
};
fc2_str_val<FrameRate> fc2_FrameRate_table[] = {
	{ "FRAMERATE_1_875",FlyCapture2::FRAMERATE_1_875},
	{ "FRAMERATE_3_75",FlyCapture2::FRAMERATE_3_75},
	{ "FRAMERATE_7_5",FlyCapture2::FRAMERATE_7_5},
	{ "FRAMERATE_15",FlyCapture2::FRAMERATE_15},
	{ "FRAMERATE_30",FlyCapture2::FRAMERATE_30},
	{ "FRAMERATE_60",FlyCapture2::FRAMERATE_60},
	{ "FRAMERATE_120",FlyCapture2::FRAMERATE_120},
	{ "FRAMERATE_240",FlyCapture2::FRAMERATE_240},
	{ "FRAMERATE_FORMAT7",FlyCapture2::FRAMERATE_FORMAT7}
};
fc2_str_val<GrabMode> fc2_GrabMode_table[] = {
	{ "DROP_FRAMES",DROP_FRAMES},
	{ "BUFFER_FRAMES",BUFFER_FRAMES}
};

#define GET_CONV_TABLE(type)  \
	vector< fc2_str_val<type> > fc2_vals_gen( type ) {  \
	size_t n = sizeof( fc2_##type##_table ) / sizeof( fc2_##type##_table[0] );  \
	vector< fc2_str_val<type> > vec( &fc2_##type##_table[0], &fc2_##type##_table[n] );  \
	return vec; }
GET_CONV_TABLE(VideoMode)
GET_CONV_TABLE(FrameRate)
GET_CONV_TABLE(GrabMode)

template <typename T>
T fc2_defstr2num(const std::string &str)
{
	vector< fc2_str_val<T> > fc2_vals = fc2_vals_gen( T() );
	const std::string s = mrpt::system::trim(str);
	for (size_t i=0;i<fc2_vals.size();i++)
	{
		if (mrpt::system::strCmpI(fc2_vals[i].str,s.c_str()))
			return fc2_vals[i].val;
	}
	THROW_EXCEPTION_CUSTOM_MSG1("Error: Unknown FlyCapture2 constant: %s",s.c_str())
}


template <typename T>
const char* fc2_defnum2str(const T &val)
{
    vector< fc2_str_val<T> > fc2_vals = fc2_vals_gen( T() );
	 size_t i = static_cast<int>(val);
	 if (i < fc2_vals.size())
		  return fc2_vals[i].str;
	 else
		  THROW_EXCEPTION_CUSTOM_MSG1("Error: Unknown FlyCapture2 enum: %i",static_cast<int>(val))
}
#endif


//  Options: TCaptureOptions_bumblebee
// -------------------------------------------------------------
TCaptureOptions_FlyCapture2::TCaptureOptions_FlyCapture2() :
	camera_index (0),
	open_by_guid (false),
	videomode(), //("VIDEOMODE_640x480Y8"),
	framerate(), // ("FRAMERATE_30"),
	grabmode("BUFFER_FRAMES"),
	numBuffers(30),
	grabTimeout(-1),
	trigger_enabled(false),
	trigger_polarity(0),
	trigger_source(0),
	trigger_mode(0),
	strobe_enabled(false),
	strobe_source(0),
	strobe_polarity(0),
	strobe_delay(0.0f),
	strobe_duration(1.0f),
    autoexposure_auto(true),
    autoexposure_onOff(true),
    autoexposure_abs(true),
    autoexposure_EV(0.0f),
	shutter_auto(true),
    shutter_abs(true),
	shutter_time_ms(4.0f),
    gain_auto(true),
    gain_abs(true),
    gain_dB(0.0f),
	stereo_mode(false),
	get_rectified(false),
	rect_width(640),
	rect_height(480)
{
	memset(camera_guid,0,4*sizeof(camera_guid[0]));
}

void TCaptureOptions_FlyCapture2::loadOptionsFrom(
	const mrpt::utils::CConfigFileBase & cfg,
	const std::string & sect,
	const std::string & prefix )
{
	camera_index = cfg.read_int(sect, prefix+string("camera_index"), camera_index);
	open_by_guid = cfg.read_bool(sect, prefix+string("open_by_guid"), open_by_guid);

	if (open_by_guid)
	{
		string sGUID = cfg.read_string(sect, prefix+string("camera_guid"), "",  true );
		vector<string> sGUIDparts;
		mrpt::system::tokenize(sGUID,"- \t\r\n",sGUIDparts);
		ASSERTMSG_(sGUIDparts.size()==4, "GUID format error: must have four blocks like XXX-XXX-XXX-XXX")

		for (int i=0;i<4;i++)
			sscanf(sGUIDparts[i].c_str(),"%X", &camera_guid[i]);
	}

	videomode = cfg.read_string(sect, prefix+string("videomode"), videomode);
	framerate = cfg.read_string(sect, prefix+string("framerate"), framerate);
	grabmode = cfg.read_string(sect, prefix+string("grabmode"), grabmode);
	numBuffers = cfg.read_uint64_t(sect, prefix+string("numBuffers"), numBuffers);
	grabTimeout = cfg.read_int(sect, prefix+string("grabTimeout"), grabTimeout);

	trigger_enabled = cfg.read_bool(sect, prefix+string("trigger_enabled"), trigger_enabled);
	trigger_polarity = cfg.read_int(sect, prefix+string("trigger_polarity"), trigger_polarity);
	trigger_source = cfg.read_int(sect, prefix+string("trigger_source"), trigger_source);
	trigger_mode = cfg.read_int(sect, prefix+string("trigger_mode"), trigger_mode);

	strobe_enabled = cfg.read_bool(sect, prefix+string("strobe_enabled"), strobe_enabled);
	strobe_source = cfg.read_int(sect, prefix+string("strobe_source"), strobe_source);
	strobe_polarity = cfg.read_int(sect, prefix+string("strobe_polarity"), strobe_polarity);
	strobe_delay = cfg.read_float(sect, prefix+string("strobe_delay"), strobe_delay);
	strobe_duration = cfg.read_float(sect, prefix+string("strobe_duration"), strobe_duration);

    autoexposure_auto = cfg.read_bool(sect, prefix+string("autoexposure_auto"), autoexposure_auto);
    autoexposure_onOff = cfg.read_bool(sect, prefix+string("autoexposure_onOFf"), autoexposure_onOff);
    autoexposure_abs = cfg.read_bool(sect, prefix+string("autoexposure_abs"), autoexposure_abs);
    autoexposure_EV = cfg.read_float(sect, prefix+string("autoexposure_EV"), autoexposure_EV);

	shutter_auto = cfg.read_bool(sect, prefix+string("shutter_auto"), shutter_auto);
    shutter_abs = cfg.read_bool(sect, prefix+string("shutter_abs"), shutter_abs);
	shutter_time_ms = cfg.read_float(sect, prefix+string("shutter_time_ms"), shutter_time_ms);

    gain_auto = cfg.read_bool(sect, prefix+string("gain_auto"), gain_auto);
    gain_abs = cfg.read_bool(sect, prefix+string("gain_abs"), gain_abs);
    gain_dB = cfg.read_float(sect, prefix+string("gain_dB"), gain_dB);

	stereo_mode = cfg.read_bool(sect, prefix+string("stereo_mode"), stereo_mode);
	get_rectified = cfg.read_bool(sect, prefix+string("get_rectified"), get_rectified);
	rect_width = cfg.read_uint64_t(sect, prefix+string("rect_width"), rect_width);
	rect_height = cfg.read_uint64_t(sect, prefix+string("rect_height"), rect_height);
}


// ---------------------------------------------------------------
/** Default constructor */
CImageGrabber_FlyCapture2::CImageGrabber_FlyCapture2() :
	m_camera(NULL),
	m_camera_info(NULL),
	m_img_buffer(NULL),
	m_triclops(NULL)
{
#if MRPT_HAS_FLYCAPTURE2
	m_img_buffer = new FlyCapture2::Image();
#endif
}

/** Constructor + open */
CImageGrabber_FlyCapture2::CImageGrabber_FlyCapture2( const TCaptureOptions_FlyCapture2 &options ) :
	m_camera(NULL),
	m_camera_info(NULL),
	m_img_buffer(NULL),
	m_triclops(NULL)
{
#if MRPT_HAS_FLYCAPTURE2
	m_img_buffer = new FlyCapture2::Image();
#endif
	this->open(options);
}

/** Destructor */
CImageGrabber_FlyCapture2::~CImageGrabber_FlyCapture2()
{
#if MRPT_HAS_FLYCAPTURE2
	this->close();
	delete FC2_BUF_IMG; m_img_buffer = NULL;
#endif
}


/** Tries to open the camera with the given options. Raises an exception on error. \sa close() */
void CImageGrabber_FlyCapture2::open( const TCaptureOptions_FlyCapture2 &options, const bool startCapture )
{
#if MRPT_HAS_FLYCAPTURE2
	FlyCapture2::Error fe;

	cout << "[CImageGrabber_FlyCapture2::open] FlyCapture2 version: " << CImageGrabber_FlyCapture2::getFC2version() << std::endl;

	this->close();
	this->m_options = options;

	// Determine camera to open:
	// -----------------------------------
	PGRGuid guid;
	if (m_options.open_by_guid)
	{
		// Open by GUID:
		for (int i=0;i<4;i++)
			guid.value[i] = m_options.camera_guid[i];
	}
	else
	{
		// Open by camera index:
		BusManager busMgr;
		unsigned int numCameras;
		fe = busMgr.GetNumOfCameras(&numCameras);
        CHECK_FC2_ERROR(fe)

		if (m_options.camera_index>=numCameras)
			THROW_EXCEPTION(mrpt::format("Error: camera_index to open is '%u', but only '%u' cameras were detected in the system.",m_options.camera_index,numCameras))

		fe = busMgr.GetCameraFromIndex(m_options.camera_index, &guid);
		CHECK_FC2_ERROR(fe)
	}

	// Connect to camera:
	m_camera = new FlyCapture2::Camera();
	m_camera_info = new FlyCapture2::CameraInfo();

	cout << mrpt::format("[CImageGrabber_FlyCapture2::open] Opening camera with GUID= %08X-%08X-%08X-%08X...\n", guid.value[0],guid.value[1],guid.value[2],guid.value[3]);
	fe = FC2_CAM->Connect(&guid);
	CHECK_FC2_ERROR(fe)
	fe = FC2_CAM->GetCameraInfo(FC2_CAM_INFO);
	CHECK_FC2_ERROR(fe)

	const FlyCapture2::CameraInfo *ci = FC2_CAM_INFO;

	cout << mrpt::format(
	"[CImageGrabber_FlyCapture2::open] Camera connected ok:\n"
		" Serial number - %u\n"
		" Camera model - %s\n"
		" Camera vendor - %s\n"
		" Sensor - %s\n"
		" Resolution - %s\n"
		" Firmware version - %s\n"
		" Firmware build time - %s\n\n",
		ci->serialNumber,
		ci->modelName,
		ci->vendorName,
		ci->sensorInfo,
		ci->sensorResolution,
		ci->firmwareVersion,
		ci->firmwareBuildTime );

	// Set camera config:
	if (!m_options.videomode.empty() && !m_options.framerate.empty())
	{
		bool isSupported = false;

		if (!m_options.stereo_mode)
		{
			FlyCapture2::VideoMode vidMode = fc2_defstr2num<FlyCapture2::VideoMode>(m_options.videomode);
			FlyCapture2::FrameRate vidRate = fc2_defstr2num<FlyCapture2::FrameRate>(m_options.framerate);

			fe = FC2_CAM->GetVideoModeAndFrameRateInfo(vidMode,vidRate, &isSupported);
			CHECK_FC2_ERROR(fe)

					if (!isSupported)
			{
				FlyCapture2::VideoMode curVidMode;
				FlyCapture2::FrameRate curVidRate;
				fe = FC2_CAM->GetVideoModeAndFrameRate(&curVidMode,&curVidRate);

				THROW_EXCEPTION(mrpt::format("Camera mode '%s' + '%s' is not supported by this camera. Current mode is %d, current rate is %d.",m_options.videomode.c_str(),m_options.framerate.c_str(),static_cast<int>(curVidMode),static_cast<int>(curVidRate) ))
			}

			fe = FC2_CAM->SetVideoModeAndFrameRate(vidMode,vidRate);
			CHECK_FC2_ERROR(fe)
		}
		else
		{
#if MRPT_HAS_TRICLOPS
			Fc2Triclops::ErrorType fte;
			// Configure camera for Stereo mode
			StereoCameraMode mode = TWO_CAMERA;
			fte = setStereoMode( *(FC2_CAM), mode );
			if ( fte )
				handleFc2TriclopsError(fte, "setStereoMode");

			//Generate Triclops context

			m_triclops= new TriclopsContext;
			fte = getContextFromCamera( FC2_CAM_INFO->serialNumber,
													TRI_CONTEXT );
			if (fte != ERRORTYPE_OK)
				handleFc2TriclopsError(fte, "getContextFromCamera");

			//  ------------------------------------------------------
			//   TRICLOPS CONFIGURATION
			//  ------------------------------------------------------
			// Current Format7 settings
			/*
			Format7ImageSettings f7settings;
			unsigned int f7PacketSize;
			float f7Percentage;
			fe = FC2_CAM->GetFormat7Configuration(&f7settings, &f7PacketSize, &f7Percentage);
			CHECK_FC2_ERROR(fe)
			*/

			TriclopsError te;
			// Set rectified resolution
			te = triclopsSetResolution( *(TRI_CONTEXT), m_options.rect_height, m_options.rect_width );
			CHECK_TRICLOPS_ERROR( te );
			// Retrieve camera parameters
			te = triclopsGetBaseline( *(TRI_CONTEXT), &m_baseline );
			CHECK_TRICLOPS_ERROR( te );
			te = triclopsGetFocalLength( *(TRI_CONTEXT), &m_focalLength );
			CHECK_TRICLOPS_ERROR( te );
			te = triclopsGetImageCenter( *(TRI_CONTEXT), &m_centerRow, &m_centerCol);
			CHECK_TRICLOPS_ERROR( te );
#else
			THROW_EXCEPTION("MRPT compiled without support for Triclops")
#endif
		}
	}

	{
		FlyCapture2::VideoMode curVidMode;
		FlyCapture2::FrameRate curVidRate;
		fe = FC2_CAM->GetVideoModeAndFrameRate(&curVidMode,&curVidRate);
		if (fe==PGRERROR_OK)
			cout << mrpt::format("[CImageGrabber_FlyCapture2::open] Current camera mode is %s, current rate is %s.\n",
										fc2_defnum2str<FlyCapture2::VideoMode>(curVidMode),
										fc2_defnum2str<FlyCapture2::FrameRate>(curVidRate));
	}


	// Set trigger:
	FlyCapture2::TriggerModeInfo trigInfo;
	FC2_CAM->GetTriggerModeInfo(&trigInfo);

	FlyCapture2::TriggerMode trig;
	trig.onOff = m_options.trigger_enabled;
	if ( m_options.trigger_enabled )
	{
		trig.mode  = m_options.trigger_mode;
		trig.polarity = m_options.trigger_polarity;
		trig.source   = m_options.trigger_source;
	}
	fe = FC2_CAM->SetTriggerMode(&trig);
	CHECK_FC2_ERROR(fe)

	// Strobe:
	if (m_options.strobe_enabled)
	{
		FlyCapture2::StrobeControl strobe;

		strobe.onOff = m_options.strobe_enabled;
		strobe.delay = m_options.strobe_delay;
		strobe.duration = m_options.strobe_duration;
		strobe.polarity = m_options.strobe_polarity;
		strobe.source = m_options.strobe_source;

		fe = FC2_CAM->SetStrobe(&strobe);
		CHECK_FC2_ERROR(fe)
	}

	// Set configs:
	FlyCapture2::FC2Config fc2conf;
	FC2_CAM->GetConfiguration(&fc2conf);
	CHECK_FC2_ERROR(fe)

	fc2conf.grabMode = fc2_defstr2num<FlyCapture2::GrabMode>(m_options.grabmode);
	if (m_options.grabTimeout>=0)
		fc2conf.grabTimeout = m_options.grabTimeout;

	fc2conf.numBuffers = m_options.numBuffers;

	fe = FC2_CAM->SetConfiguration( &fc2conf);
	CHECK_FC2_ERROR(fe)

	// Autoexposure:
	{
		FlyCapture2::Property p;
		p.type = FlyCapture2::AUTO_EXPOSURE;
		FlyCapture2::Error error = FC2_CAM->GetProperty( &p );
		CHECK_FC2_ERROR(error)
		p.autoManualMode = m_options.autoexposure_auto; // true=auto
		p.onOff = m_options.autoexposure_onOff; // true=on
		p.absControl = m_options.autoexposure_abs; // true=abs
		p.absValue = m_options.autoexposure_EV; // abs value in Exposure Value (EV)
		fe = FC2_CAM->SetProperty (&p);
		CHECK_FC2_ERROR(fe)
	}

	// Brightness:
	{
		FlyCapture2::Property p;
		p.type = FlyCapture2::BRIGHTNESS;
		FlyCapture2::Error error = FC2_CAM->GetProperty( &p );
		CHECK_FC2_ERROR(error)
		p.autoManualMode = true; // true=auto
		//p.absControl = true;
		//p.absValue = Brightness;
		fe = FC2_CAM->SetProperty (&p);
		CHECK_FC2_ERROR(fe)
	}

	// Shutter:
	{
		FlyCapture2::Property p;
		p.type = FlyCapture2::SHUTTER;
		FlyCapture2::Error error = FC2_CAM->GetProperty( &p );
		CHECK_FC2_ERROR(error)
		p.autoManualMode = m_options.shutter_auto; // true=auto
		p.absControl = m_options.shutter_abs; // true=abs
		p.absValue = m_options.shutter_time_ms;
		//p.onOff = false;
		fe = FC2_CAM->SetProperty (&p);
		CHECK_FC2_ERROR(fe)
	}

	// Gain:
	{
		FlyCapture2::Property p;
		p.type = FlyCapture2::GAIN;
		FlyCapture2::Error error = FC2_CAM->GetProperty( &p );
		CHECK_FC2_ERROR(error)
		p.autoManualMode = m_options.gain_auto; // true=auto
		p.absControl = m_options.gain_abs; // true=abs
		p.absValue = m_options.gain_dB; // abs value in dB (decibeles)
		//p.onOff = false;
		fe = FC2_CAM->SetProperty (&p);
		CHECK_FC2_ERROR(fe)
	}

	// Framecounter:
	EmbeddedImageInfo eii;
	fe = FC2_CAM->GetEmbeddedImageInfo(&eii);
	if (fe == PGRERROR_OK)
	{
		if (eii.frameCounter.available) eii.frameCounter.onOff = true;
		if (eii.timestamp.available)    eii.timestamp.onOff = true;
		if (eii.exposure.available)     eii.exposure.onOff = true;
		if (eii.brightness.available)   eii.brightness.onOff = true;

		// Enable all:
		FC2_CAM->SetEmbeddedImageInfo(&eii);
	}

	// Start:
	if (startCapture)
		this->startCapture();
#else
	THROW_EXCEPTION("MRPT compiled without support for FlyCapture2")
#endif
}

/** Start the actual image capture of the camera. Must be called after open(), only when "startCapture" was set to false. */
void CImageGrabber_FlyCapture2::startCapture()
{
#if MRPT_HAS_FLYCAPTURE2
	if (!m_camera) { THROW_EXCEPTION("Camera is not opened. Call open() first.") }

	FlyCapture2::Error error = FC2_CAM->StartCapture();
	CHECK_FC2_ERROR(error)

#else
	THROW_EXCEPTION("MRPT compiled without support for FlyCapture2")
#endif
}

/** Starts a synchronous capture of several cameras, which must have been already opened. */
void CImageGrabber_FlyCapture2::startSyncCapture( int numCameras, const CImageGrabber_FlyCapture2 **cameras_array )
{
#if MRPT_HAS_FLYCAPTURE2

	std::vector<const FlyCapture2::Camera*> cam_ptrs(numCameras);

	for (int i=0;i<numCameras;i++)
	{
		const CImageGrabber_FlyCapture2 *obj = cameras_array[i];
		if (!obj->m_camera) { THROW_EXCEPTION_CUSTOM_MSG1("Camera #%i in list is not opened. Call open() first.",i) }

		FlyCapture2::Camera *cam = reinterpret_cast<FlyCapture2::Camera*>(obj->m_camera);
		cam_ptrs[i] = cam;
	}

	if (!cam_ptrs.empty())
	{
		FlyCapture2::Error error = FlyCapture2::Camera::StartSyncCapture(cam_ptrs.size(), &cam_ptrs[0]);
		CHECK_FC2_ERROR(error)
	}
#else
	THROW_EXCEPTION("MRPT compiled without support for FlyCapture2")
#endif
}

/** Stop capture. */
void CImageGrabber_FlyCapture2::stopCapture()
{
#if MRPT_HAS_FLYCAPTURE2
	if (m_camera)
	{
		Error error;

		// Stop grabbing:
		error = FC2_CAM->StopCapture();
		CHECK_FC2_ERROR(error)
	}
#else
	THROW_EXCEPTION("MRPT compiled without support for FlyCapture2")
#endif
}


/** Closes the opened camera, if any. Called automatically on object destruction. */
void CImageGrabber_FlyCapture2::close()
{
#if MRPT_HAS_FLYCAPTURE2
	try {
		this->stopCapture();
	}
	catch (...) { }

	// Disconnect the camera
	try {
		if (m_camera) FC2_CAM->Disconnect();
	}
	catch (...) { }

	// Delete objects:
	try { if (m_camera) delete FC2_CAM;  } catch (...) {}
	try { if (m_camera_info) delete FC2_CAM_INFO; } catch (...) {}
#if MRPT_HAS_TRICLOPS
	try { if (m_triclops) delete TRI_CONTEXT; } catch (...) {}
#endif

	m_camera=NULL;
	m_camera_info=NULL;

#else
	THROW_EXCEPTION("MRPT compiled without support for FlyCapture2")
#endif
}


/** Returns the PGR FlyCapture2 library version */
std::string CImageGrabber_FlyCapture2::getFC2version()
{
#if MRPT_HAS_FLYCAPTURE2
    FlyCapture2::FC2Version fc2Version;
    FlyCapture2::Utilities::GetLibraryVersion( &fc2Version );
    return mrpt::format("%d.%d.%d.%d", fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build);
#else
	THROW_EXCEPTION("MRPT compiled without support for FlyCapture2")
#endif
}

/*-------------------------------------------------------------
					get the image - MONO
 -------------------------------------------------------------*/
// Grab image from the camera. This method blocks until the next frame is captured.
// return: false on any error.
bool CImageGrabber_FlyCapture2::getObservation( mrpt::obs::CObservationImage &out_observation )
{
#if MRPT_HAS_FLYCAPTURE2
if (!m_camera) {
std::cerr << "[CImageGrabber_FlyCapture2::getObservation] Camera is not opened. Call open() first.\n";
return false;
}
try
{
FlyCapture2::Error error;
FlyCapture2::Image image;
error = FC2_CAM->RetrieveBuffer( &image );
CHECK_FC2_ERROR(error)
FlyCapture2::TimeStamp timestamp = image.GetTimeStamp();
// White balance, etc.
//FlyCapture2::ImageMetadata imd = image.GetMetadata();
// Determine if it's B/W or color:
FlyCapture2::PixelFormat pf = image.GetPixelFormat();
const bool is_color =
pf==PIXEL_FORMAT_RGB8 || pf==PIXEL_FORMAT_RGB16 || pf==PIXEL_FORMAT_S_RGB16 ||
pf==PIXEL_FORMAT_RAW8 || pf==PIXEL_FORMAT_RAW16 || pf==PIXEL_FORMAT_RAW12 ||
pf==PIXEL_FORMAT_BGR || pf==PIXEL_FORMAT_BGRU || pf==PIXEL_FORMAT_RGBU ||
pf==PIXEL_FORMAT_BGR16 || pf==PIXEL_FORMAT_BGRU16 || pf==PIXEL_FORMAT_422YUV8_JPEG;
// Decode image:
error = image.Convert(is_color ? PIXEL_FORMAT_BGR : PIXEL_FORMAT_MONO8, FC2_BUF_IMG);
CHECK_FC2_ERROR(error)
// Convert PGR FlyCapture2 image ==> OpenCV format:
unsigned int img_rows, img_cols, img_stride;
FC2_BUF_IMG->GetDimensions( &img_rows, &img_cols, &img_stride);
out_observation.image.loadFromMemoryBuffer(img_cols,img_rows, is_color, FC2_BUF_IMG->GetData() );
// It seems timestamp is not always correctly filled in the incoming imgs:
if (timestamp.seconds!=0)
out_observation.timestamp = mrpt::system::time_tToTimestamp( timestamp.seconds + 1e-6*timestamp.microSeconds );
else out_observation.timestamp = mrpt::system::now();
return true;
}
catch( std::exception &e)
{
std::cerr << "[CImageGrabber_FlyCapture2::getObservation] Error:\n" << e.what() << std::endl;
return false;
}
#else
THROW_EXCEPTION("MRPT compiled without support for FlyCapture2")
#endif
}

/*-------------------------------------------------------------
					get the image - STEREO
 -------------------------------------------------------------*/
// Grab image from the camera. This method blocks until the next frame is captured.
// return: false on any error.
bool CImageGrabber_FlyCapture2::getObservation( mrpt::obs::CObservationStereoImages &out_observation )
{
#if MRPT_HAS_FLYCAPTURE2 && MRPT_HAS_TRICLOPS && MRPT_HAS_OPENCV
	if (!m_camera) {
		std::cerr << "[CImageGrabber_FlyCapture2::getObservation] Camera is not opened. Call open() first.\n";
		return false;
	}

	try
	{
		FlyCapture2::Error ferr;
		Fc2Triclops::ErrorType	fterr;
		TriclopsError te;
		FlyCapture2::Image image;
		ferr = FC2_CAM->RetrieveBuffer( &image );
		CHECK_FC2_ERROR(ferr)
		mrpt::system::TTimeStamp ts_retrieved = mrpt::system::now();
		FlyCapture2::TimeStamp timestamp = image.GetTimeStamp();

		// White balance, etc.
		//FlyCapture2::ImageMetadata imd = image.GetMetadata();

		// ------------------------------------------
		// Extract images from common interleaved image:
		// ------------------------------------------
		IplImage*	imageIpl[2];	// Output pair of images
		FlyCapture2::Image rawImage[2];

		// Convert the pixel interleaved raw data to de-interleaved raw data
		fterr = Fc2Triclops::unpackUnprocessedRawOrMono16Image(
					image,
					true,
					rawImage[0],	// Right image
					rawImage[1] );	// Left image
		if (fterr != Fc2Triclops::ERRORTYPE_OK)
		{
			Fc2Triclops::handleFc2TriclopsError(fterr, "unprocessedRawOrMono16Image()");
			return false;
		}

		// Convert each raw image to RGBU image (for color images)
		unsigned int img_rows, img_cols, img_stride;
		for ( int i = 0; i < 2; ++i )
		{
			FlyCapture2::Image rgbuImage;
			ferr = rawImage[i].SetColorProcessing(FlyCapture2::HQ_LINEAR);
			CHECK_FC2_ERROR(ferr)
			ferr = rawImage[i].Convert(PIXEL_FORMAT_BGRU, &rgbuImage);
			CHECK_FC2_ERROR(ferr)

			unsigned char* dataPtr; // To store Ipl converted image pointer
			if(m_options.get_rectified) // If rectified
			{
				// Use the rgbu single image to build up a packed (rbgu) TriclopsInput.
				// A packed triclops input will contain a single image with 32 bpp.
				TriclopsInput triclopsColorInput;
				te = triclopsBuildPackedTriclopsInput(
							rgbuImage.GetCols(),
							rgbuImage.GetRows(),
							rgbuImage.GetStride(),
							(unsigned long)image.GetTimeStamp().seconds,
							(unsigned long)image.GetTimeStamp().microSeconds,
							rgbuImage.GetData(),
							&triclopsColorInput );

				// Do rectification
				TriclopsPackedColorImage rectPackColImg;
				te = triclopsRectifyPackedColorImage(
					*(TRI_CONTEXT),
					i==0 ? TriCam_RIGHT : TriCam_LEFT,
					const_cast<TriclopsInput *>(&triclopsColorInput),
					&rectPackColImg );
				CHECK_TRICLOPS_ERROR( te )

				// Set image properties for reallocation
				img_rows = rectPackColImg.nrows;
				img_cols = rectPackColImg.ncols;
				img_stride = rectPackColImg.rowinc;
				dataPtr = (unsigned char*)rectPackColImg.data;
			}
			else // If not rectified
			{
				rgbuImage.GetDimensions(&img_rows,&img_cols,&img_stride);
				dataPtr = rgbuImage.GetData();
			}
			// Convert PGR image ==> OpenCV format:
			IplImage *tmpImage = cvCreateImage( cvSize( img_cols, img_rows ), IPL_DEPTH_8U, 4 );

			// Copy image data
			memcpy( tmpImage->imageData, dataPtr, img_rows*img_stride );
			tmpImage->widthStep = img_stride;
			// Convert images to BGR (3 channels) and set origins
			imageIpl[i] = cvCreateImage( cvSize( img_cols, img_rows ), IPL_DEPTH_8U, 3 );
			cvCvtColor( tmpImage, imageIpl[i], CV_BGRA2BGR );
			imageIpl[i]->origin = tmpImage->origin;
			// Release temp images
			cvReleaseImage( &tmpImage );
		}

		/*-------------------------------------------------------------
							Fill output stereo observation
		 -------------------------------------------------------------*/
		out_observation.imageRight.setFromIplImage( imageIpl[0] ); // Right cam.
		out_observation.imageLeft.setFromIplImage ( imageIpl[1] ); // Left cam.

		// It seems timestamp is not always correctly filled in the incoming imgs:
		if (timestamp.seconds!=0)
			out_observation.timestamp = mrpt::system::time_tToTimestamp( timestamp.seconds + 1e-6*timestamp.microSeconds );
		else out_observation.timestamp = ts_retrieved;

		out_observation.rightCameraPose.x( m_baseline );
		out_observation.rightCameraPose.y( 0 );
		out_observation.rightCameraPose.z( 0 );

		out_observation.rightCameraPose.quat().r( 1 );
		out_observation.rightCameraPose.quat().x( 0 );
		out_observation.rightCameraPose.quat().y( 0 );
		out_observation.rightCameraPose.quat().z( 0 );

		out_observation.cameraPose.x( 0 );
		out_observation.cameraPose.y( 0 );
		out_observation.cameraPose.z( 0 );

		out_observation.cameraPose.quat().r( 1 );
		out_observation.cameraPose.quat().x( 0 );
		out_observation.cameraPose.quat().y( 0 );
		out_observation.cameraPose.quat().z( 0 );

		out_observation.leftCamera.setIntrinsicParamsFromValues ( m_focalLength, m_focalLength, m_centerCol, m_centerRow );
		out_observation.rightCamera.setIntrinsicParamsFromValues( m_focalLength, m_focalLength, m_centerCol, m_centerRow );
		return true;
	}
	catch( std::exception &e)
	{
		std::cerr << "[CImageGrabber_FlyCapture2::getObservation] Error:\n" << e.what() << std::endl;
		return false;
	}
#else
	THROW_EXCEPTION("MRPT compiled without support for FlyCapture2, Triclops or OpenCV")
#endif
}

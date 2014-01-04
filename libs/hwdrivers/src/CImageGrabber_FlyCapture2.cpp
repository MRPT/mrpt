/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers.h> // Precompiled header

#if MRPT_HAS_FLYCAPTURE2
	#include <FlyCapture2.h>
#endif

#define CHECK_FC2_ERROR(_err) { if (_err != PGRERROR_OK) { THROW_EXCEPTION_CUSTOM_MSG1("FlyCapture2 error:\n%s",_err.GetDescription()) } }
#define FC2_CAM  reinterpret_cast<FlyCapture2::Camera*>(m_camera)
#define FC2_CAM_INFO  reinterpret_cast<FlyCapture2::CameraInfo*>(m_camera_info)

using namespace mrpt::hwdrivers;

//  Options: TCaptureOptions_bumblebee 
// -------------------------------------------------------------
TCaptureOptions_FlyCapture2::TCaptureOptions_FlyCapture2() :
	camera_index (0),
	open_by_guid (false)
{
	memset(camera_guid,0,4*sizeof(camera_guid[0]));
}

// ---------------------------------------------------------------
/** Default constructor */
CImageGrabber_FlyCapture2::CImageGrabber_FlyCapture2() : 
	m_camera(NULL),
	m_camera_info(NULL)
{
}

/** Constructor + open */
CImageGrabber_FlyCapture2::CImageGrabber_FlyCapture2( const TCaptureOptions_FlyCapture2 &options ) : 
	m_camera(NULL),
	m_camera_info(NULL)
{
	this->open(options);
}

/** Destructor */
CImageGrabber_FlyCapture2::~CImageGrabber_FlyCapture2()
{
	this->close();
}


/** Tries to open the camera with the given options. Raises an exception on error. \sa close() */
void CImageGrabber_FlyCapture2::open( const TCaptureOptions_FlyCapture2 &options )
{
#if MRPT_HAS_FLYCAPTURE2
	using namespace FlyCapture2;
    Error error;

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
		error = busMgr.GetNumOfCameras(&numCameras);
		CHECK_FC2_ERROR(error)

		if (m_options.camera_index>=numCameras)
			THROW_EXCEPTION(mrpt::format("Error: camera_index to open is '%u', but only '%u' cameras were detected in the system.",m_options.camera_index,numCameras))

		error = busMgr.GetCameraFromIndex(m_options.camera_index, &guid);
		CHECK_FC2_ERROR(error)
	}

	// Connect to camera:
	m_camera = new FlyCapture2::Camera();
	m_camera_info = new FlyCapture2::CameraInfo();

	cout << mrpt::format("[CImageGrabber_FlyCapture2::open] Opening camera with GUID= %04X%04X%04X%04X...\n", guid.value[0],guid.value[1],guid.value[2],guid.value[3]);
    error = FC2_CAM->Connect(&guid);
	CHECK_FC2_ERROR(error)
	error=FC2_CAM->GetCameraInfo(FC2_CAM_INFO);
	CHECK_FC2_ERROR(error)

	cout << mrpt::format(
	"[CImageGrabber_FlyCapture2::open] Camera connected ok:\n"
		" Serial number - %u\n"
		" Camera model - %s\n"
		" Camera vendor - %s\n"
		" Sensor - %s\n"
		" Resolution - %s\n"
		" Firmware version - %s\n"
		" Firmware build time - %s\n\n",
		FC2_CAM_INFO->serialNumber,
		FC2_CAM_INFO->modelName,
		FC2_CAM_INFO->vendorName,
		FC2_CAM_INFO->sensorInfo,
		FC2_CAM_INFO->sensorResolution,
		FC2_CAM_INFO->firmwareVersion,
		FC2_CAM_INFO->firmwareBuildTime );



#else
	THROW_EXCEPTION("MRPT compiled without support for FlyCapture2")
#endif
}


/** Closes the opened camera, if any. Called automatically on object destruction. */
void CImageGrabber_FlyCapture2::close()
{
#if MRPT_HAS_FLYCAPTURE2
	using namespace FlyCapture2;

	try
	{
		if (m_camera)
		{
			Error error;
			// Stop grabbing:
			error = FC2_CAM->StopCapture();
			CHECK_FC2_ERROR(error)

			// Disconnect the camera
			error = FC2_CAM->Disconnect();
			CHECK_FC2_ERROR(error)
		}

		// Delete objects:
		if (m_camera) { delete FC2_CAM; m_camera=NULL; }
		if (m_camera_info) { delete FC2_CAM_INFO; m_camera_info=NULL; }
	}
	catch (std::exception &)
	{
		// At least, always delete objects:
		if (m_camera) { delete FC2_CAM; m_camera=NULL; }
		if (m_camera_info) { delete FC2_CAM_INFO; m_camera_info=NULL; }

		throw; // rethrow
	}

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


bool CImageGrabber_FlyCapture2::getObservation( mrpt::slam::CObservationImage &out_observation )
{
#if MRPT_HAS_FLYCAPTURE2
	return false;
#else
	THROW_EXCEPTION("MRPT compiled without support for FlyCapture2")
#endif
}




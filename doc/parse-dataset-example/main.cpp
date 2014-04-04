/* +---------------------------------------------------------------------------+
   |          Rawlog (dataset) parsing example code                            |
   |                                                                           |
   |  This file is released to the public domain.                              |
   |  Jose Luis Blanco, 2013 - University of Almeria                           |
   |                                                                           |
   |                                                                           |
   | Program: parse-dataset-example                                            |
   | Purpose: Displays a dataset content as it is read.                        |
   | Usage:                                                                    |
   |  ./parse-dataset-example DATASET_FILE.rawlog                              |
   |  ./parse-dataset-example DATASET_FILE.rawlog [optional_start_timestamp]   |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

// This program requires the MRPT libraries ( http://www.mrpt.org/ )
#include <mrpt/base.h>   // Serialization, etc.
#include <mrpt/obs.h>    // Sensor observations classes
#include <mrpt/maps.h>   // Point clouds, etc.
#include <mrpt/gui.h>    // GUI windows
#include <mrpt/topography.h>   // GPS coordinates processing

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace std;

int main(int argc, char **argv)
{
	CDisplayWindow3D  win("Data set preview",608,544);

	try
	{
		if (argc!=2 && argc!=3)
		{
			cerr << "Usage:\n"
			     <<  argv[0] << " <RAWLOG_FILE> [optional_start_timestamp]\n\n";
			return 1;
		}

		const string rawlog_file = string(argv[1]);
		ASSERT_(mrpt::system::fileExists(rawlog_file))

		const TTimeStamp start_timestamp =
			(argc==3) ?
			mrpt::system::time_tToTimestamp(atof(argv[2])) :
			TTimeStamp(0);

		if (argc==3)
			cout << "Using starting timestamp = " << mrpt::system::dateTimeLocalToString(start_timestamp) << endl;

		// External images: autodetect the directory for images
		CImage::IMAGES_PATH_BASE = CRawlog::detectImagesDirectory(rawlog_file);

		// GZ-compressed input stream:
		CFileGZInputStream  fil(rawlog_file);

		size_t nEntry = 0;
		CActionCollectionPtr  acts;
		CSensoryFramePtr      SF;
		CObservationPtr       obs;

		cout << "Parsing rawlog...\n";

		bool waiting_to_first = false;
		topography::TGeodeticCoords  coords_ref; // GPS reference point (=first GPS reading)

		while (CRawlog::getActionObservationPairOrObservation(fil,acts,SF,obs,nEntry) && win.isOpen() )
		{
			if (!obs || obs->timestamp<start_timestamp)
			{
				waiting_to_first = true;
				continue;
			}

			if (waiting_to_first)
			{
				waiting_to_first = false;
				cout << "Reached desired starting point...\n";
			}

			// Process GPS entries:
			if (IS_CLASS(obs,CObservationGPS))
			{
				CObservationGPSPtr o = CObservationGPSPtr(obs);
				if (o->has_GGA_datum && o->GGA_datum.fix_quality>=1)
				{
					topography::TGeodeticCoords  coord = o->GGA_datum.getAsStruct<topography::TGeodeticCoords>();
					if (coords_ref.isClear())
						coords_ref = coord;

					TPoint3D P;
					topography::geodeticToENU_WGS84(coord,P,coords_ref);

					cout << "GPS: ENU coords=" << P
					     << " lat: " << coord.lat.getAsString()
					     << " lon: "<< coord.lon.getAsString() << endl;
				}
			}
			else
			if (IS_CLASS(obs,CObservation2DRangeScan))
			{
				CObservation2DRangeScanPtr o = CObservation2DRangeScanPtr(obs);
				// o->...
			}
			else
			if (IS_CLASS(obs,CObservationStereoImages))
			{
			    CObservationStereoImagesPtr o = CObservationStereoImagesPtr(obs);
			    win.setImageView( o->imageLeft ); // Use a 3D window to display a 2D image (exploits OpenGL acceleration)
			    win.repaint();

			    // Internally, mrpt::utils::CImage are stored in OpenCV's IPL format, so you can efficiently get
			    // them as "IplImage*" and call OpenCV APIs:
			    //
			    // const IplImage * img_left  = o->imageLeft.getAs<IplImage>();
			    // const IplImage * img_right = o->imageRight.getAs<IplImage>();
			}
			else
			if (IS_CLASS(obs,CObservationIMU))
			{
				CObservationIMUPtr o = CObservationIMUPtr(obs);
				//cout << "IMU: yaw vel.=" << o->rawMeasurements[IMU_YAW_VEL] <<" rad/s" << endl;
			}


		}; // end while

		cout << "\nAll done, close the window to quit.\n";
		win.waitForKey();

		return 0;
	} catch (exception &e)
	{
		cerr << "Exception: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		cerr << "Untyped excepcion!!";
		return -1;
	}
}


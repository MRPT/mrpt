/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/* ------------------------------------------------------
	              rgbd2rawlog
    A small tool to translate RGB+D datasets from TUM:
    http://cvpr.in.tum.de/data/datasets/rgbd-dataset/

    into MRPT rawlog binary format, ready to be parsed
    by RawLogViewer or user programs.

Usage:
      rgbd_dataset2rawlog  [PATH_TO_RGBD_DATASET] [OUTPUT_NAME]

Where expected input files are:
    - PATH_TO_RGBD_DATASET/accelerometer.txt
    - PATH_TO_RGBD_DATASET/depth.txt
    - PATH_TO_RGBD_DATASET/rgb.txt
    - PATH_TO_RGBD_DATASET/rgb/... (Images, 3x8u)
    - PATH_TO_RGBD_DATASET/depth/...  (Images, 1x16u)

Output files:
	- OUTPUT_NAME.rawlog: The output rawlog file.
	- OUTPUT_NAME_Images/...: External RGB images.

   ------------------------------------------------------ */


#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CTextFileLinesParser.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::obs;

const double KINECT_FPS = 30.0;

// Based on: http://stackoverflow.com/questions/218488/finding-best-matching-key-for-a-given-key-in-a-sorted-stl-container
template <class CONTAINER>
typename CONTAINER::const_iterator find_closest(
	const CONTAINER &data,
	const typename CONTAINER::key_type &searchkey )
{
    typename CONTAINER::const_iterator upper = data.lower_bound(searchkey);
    if (upper == data.begin() || upper->first==searchkey)
        return upper;
    typename CONTAINER::const_iterator lower = upper;
    --lower;
    if (upper == data.end() ||
            (searchkey - lower->first) < (upper->first - searchkey))
        return lower;
    return upper;
}


void rgbd2rawlog(const string &src_path, const string &out_name)
{
	const string in_fil_acc   = src_path+string("/accelerometer.txt");
	const string in_fil_depth = mrpt::system::fileExists(src_path+string("/ir.txt")) ? src_path+string("/ir.txt") : src_path+string("/depth.txt");
	const string in_fil_rgb = src_path+string("/rgb.txt");

	//ASSERT_FILE_EXISTS_(in_fil_acc)
	//ASSERT_FILE_EXISTS_(in_fil_depth)
	//ASSERT_FILE_EXISTS_(in_fil_rgb)

	// make a list with RGB & DEPTH files ------------------------
	map<double,string>  list_rgb, list_depth;
	map<double,vector<double> >  list_acc;
	std::istringstream line;
	if (mrpt::system::fileExists(in_fil_rgb))
	{
		mrpt::utils::CTextFileLinesParser  fparser(in_fil_rgb);
		while (fparser.getNextLine(line))
		{
			double tim;
			string f;
			if ( line >> tim >> f )
				list_rgb[tim]=f;
		}
	}
	if (mrpt::system::fileExists(in_fil_depth))
	{
		mrpt::utils::CTextFileLinesParser  fparser(in_fil_depth);
		while (fparser.getNextLine(line))
		{
			double tim;
			string f;
			if ( line >> tim >> f )
				list_depth[tim]=f;
		}
	}
	if (mrpt::system::fileExists(in_fil_acc))
	{
		mrpt::utils::CTextFileLinesParser  fparser(in_fil_acc);
		while (fparser.getNextLine(line))
		{
			double tim;
			double ax,ay,az;
			if ( line >> tim >> ax >> ay >> az )
			{
				vector<double> &v = list_acc[tim];
				v.resize(3);
				v[0] = ax;
				v[1] = ay;
				v[2] = az;
			}
		}
	}
	cout << "Parsed: " << list_depth.size() << " / " << list_rgb.size() << " / " << list_acc.size() << " depth/rgb/acc entries.\n";

	const bool only_ir  = list_depth.size()>10 && list_rgb.empty();
	const bool only_rgb = list_depth.empty() && list_rgb.size()>10;


	// Create output directory for images ------------------------------
	const string  out_img_dir = out_name + string("_Images");

	cout << "Creating images directory: " << out_img_dir << endl;
	mrpt::system::createDirectory(out_img_dir);

	// Create rawlog file ----------------------------------------------
	const string  out_rawlog_fil = out_name + string(".rawlog");
	cout << "Creating rawlog: " << out_rawlog_fil << endl;
	mrpt::utils::CFileGZOutputStream  f_out(out_rawlog_fil);

	// Fill out the common field to all entries:
	CObservation3DRangeScan  obs;
	obs.sensorLabel = "KINECT";
	obs.range_is_depth = true; // Kinect style: ranges are actually depth values, not Euclidean distances.

	// Range & RGB images are already registered and warped to remove distortion:
	const double FOCAL = 525.0;
	obs.cameraParams.nrows = 640; obs.cameraParams.ncols = 480;
	obs.cameraParams.fx(FOCAL);   obs.cameraParams.fy(FOCAL);
	obs.cameraParams.cx(319.5);   obs.cameraParams.cy(239.5);
	obs.cameraParamsIntensity = obs.cameraParams;
	obs.relativePoseIntensityWRTDepth = mrpt::poses::CPose3D(0,0,0,mrpt::utils::DEG2RAD(-90),0,mrpt::utils::DEG2RAD(-90));  // No translation between rgb & range cameras, and rotation only due to XYZ axes conventions.

	CObservationIMU obs_imu;
	obs_imu.sensorLabel = "KINECT_ACC";
	obs_imu.dataIsPresent[IMU_X_ACC] = true;
	obs_imu.dataIsPresent[IMU_Y_ACC] = true;
	obs_imu.dataIsPresent[IMU_Z_ACC] = true;

	// Go thru the data:
	unsigned int counter = 0;
	if (!only_ir && !only_rgb)
	{
		for (map<double,string>::const_iterator it_list_rgb=list_rgb.begin();it_list_rgb!=list_rgb.end();++it_list_rgb)
		{
			cout << "Processing image " << counter<< "\r"; cout.flush();
			counter++;

			// This is not the most efficient solution in the world, but...come on! it's just a rawlog converter tool!
			map<double,string>::const_iterator it_list_depth = find_closest( list_depth, it_list_rgb->first );

			const double At = std::abs( it_list_rgb->first - it_list_depth->first );
			if (At> (1./KINECT_FPS)*.5 )
			{
				cout << "\nWarning: Discarding observation for too separated RGB/D timestamps: " << At*1e3 << " ms\n";
			}
			else
			{
				// OK, we accept this RGB-DEPTH pair:
				const double avrg_time = .5* (it_list_rgb->first + it_list_depth->first);
				obs.timestamp   = mrpt::system::time_tToTimestamp(avrg_time);

				// RGB img:
				obs.hasIntensityImage = true;
				obs.intensityImage.loadFromFile( src_path + string("/") + it_list_rgb->second );
				const string sRGBfile = mrpt::format("%.06f_rgb.png", avrg_time );
				obs.intensityImage.saveToFile( out_img_dir + string("/") + sRGBfile );
				obs.intensityImage.setExternalStorage(sRGBfile);

				// Depth:
				obs.hasRangeImage = true;
				obs.rangeImage_forceResetExternalStorage();
				mrpt::utils::CImage depth_img;
				if (!depth_img.loadFromFile(src_path + string("/") + it_list_depth->second ))
					throw std::runtime_error(string("Error loading depth image!: ") + it_list_depth->second);

				const unsigned int w = depth_img.getWidth();
				const unsigned int h = depth_img.getHeight();
				obs.rangeImage_setSize(h,w);

				for (unsigned int row=0;row<h;row++)
				{
					const uint16_t *ptr = reinterpret_cast<const uint16_t *>( depth_img.get_unsafe(0,row) );
					for (unsigned int col=0;col<w;col++)
						obs.rangeImage(row,col) = (*ptr++) * (1./5000);
				}

				const string sDepthfile = mrpt::format("%.06f_depth.bin", avrg_time );
				obs.rangeImage_convertToExternalStorage( sDepthfile, out_img_dir + string("/") );

				// save:
				f_out << obs;

				// Search for acc data:
				map<double,vector<double> >::const_iterator it_list_acc = find_closest(list_acc ,avrg_time);
				const double At_acc = std::abs( it_list_rgb->first - it_list_acc->first );
				if (At_acc<1e-2)
				{
					obs_imu.timestamp = mrpt::system::time_tToTimestamp(avrg_time);

					obs_imu.rawMeasurements[IMU_X_ACC] = it_list_acc->second[0];
					obs_imu.rawMeasurements[IMU_Y_ACC] = it_list_acc->second[1];
					obs_imu.rawMeasurements[IMU_Z_ACC] = it_list_acc->second[2];

					f_out << obs_imu;
				}
			}
		}
	}
	else if (only_ir)
	{
		for (map<double,string>::const_iterator it_list_depth=list_depth.begin();it_list_depth!=list_depth.end();++it_list_depth)
		{
			cout << "Processing image " << counter<< "\r"; cout.flush();
			counter++;

			const double avrg_time = it_list_depth->first;
			obs.timestamp   = mrpt::system::time_tToTimestamp(it_list_depth->first);

			// Depth:
			obs.hasRangeImage = true;
			obs.rangeImage_forceResetExternalStorage();
			mrpt::utils::CImage depth_img;
			if (!depth_img.loadFromFile(src_path + string("/") + it_list_depth->second ))
				throw std::runtime_error(string("Error loading depth image!: ") + it_list_depth->second);

			const unsigned int w = depth_img.getWidth();
			const unsigned int h = depth_img.getHeight();
			obs.rangeImage_setSize(h,w);

			for (unsigned int row=0;row<h;row++)
			{
				const uint16_t *ptr = reinterpret_cast<const uint16_t *>( depth_img.get_unsafe(0,row) );
				for (unsigned int col=0;col<w;col++)
					obs.rangeImage(row,col) = (*ptr++) * (1./5000);
			}

			const string sDepthfile = mrpt::format("%.06f_depth.bin", avrg_time );
			obs.rangeImage_convertToExternalStorage( sDepthfile, out_img_dir + string("/") );

			// save:
			f_out << obs;
		}
	}
	else if (only_rgb)
	{
		for (map<double,string>::const_iterator it_list_rgb=list_rgb.begin();it_list_rgb!=list_rgb.end();++it_list_rgb)
		{
			cout << "Processing image " << counter<< "\r"; cout.flush();
			counter++;

			const double avrg_time = it_list_rgb->first;
			obs.timestamp   = mrpt::system::time_tToTimestamp(it_list_rgb->first);

			// RGB img:
			obs.hasIntensityImage = true;
			obs.intensityImage.loadFromFile( src_path + string("/") + it_list_rgb->second );
			const string sRGBfile = mrpt::format("%.06f_rgb.png", avrg_time );
			obs.intensityImage.saveToFile( out_img_dir + string("/") + sRGBfile );
			obs.intensityImage.setExternalStorage(sRGBfile);

			// save:
			f_out << obs;
		}
	}

	cout << "\nAll done!\n";
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char**argv)
{
	try
	{
		if (argc!=3)
		{
			cerr << "Usage: " << argv[0] << " [PATH_TO_RGBD_DATASET] [OUTPUT_NAME]\n";
			return 1;
		}

		rgbd2rawlog(argv[1],argv[2]);

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}

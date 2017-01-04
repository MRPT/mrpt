/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/* ------------------------------------------------------
	              kitti_dataset2rawlog
    A small tool to translate the KITTI datasets and 
	the karlsruhe sequences
    http://www.cvlibs.net/datasets/karlsruhe_sequences/

    into MRPT rawlog binary format, ready to be parsed
    by RawLogViewer or user programs.

Usage:
 * Karlsruhe seqs:
        kitti_dataset2rawlog  [PATH_TO_DIR_WITH_IMAGES] [CALIB_FILE] [OUTPUT_NAME]
 
 * KITTI seqs:
        kitti_dataset2rawlog  [PATH_TO_IMAGE_00] [PATH_TO_IMAGE_01] [CALIB_FILE] [OUTPUT_NAME]

Output files:
	- OUTPUT_NAME.rawlog: The output rawlog file.

  ------------------------------------------------------ */

#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/TCamera.h>
#include <mrpt/utils/CTextFileLinesParser.h>
#include <mrpt/obs/CObservationStereoImages.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::utils;

const double STEREO_FPS = 10.0;


void stereo2rawlog(bool is_kitti_dataset, const string &src_path0,const string &src_path1, const string &calib_file,  const string &out_name)
{
	/* Camera params:
	Left (1):
	6.790081e+02 0.000000e+00 6.598034e+02 0.000000e+00 
	0.000000e+00 6.790081e+02 1.865724e+02 0.000000e+00 
	0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00

	Right (2):

	6.790081e+02 0.000000e+00 6.598034e+02 -3.887481e+02 
	0.000000e+00 6.790081e+02 1.865724e+02 0.000000e+00 
	0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00

	base = 3.887481e+02  / 6.790081e+02 = 0.57252351 m
	*/

	// Parse calib file:
	mrpt::math::CMatrixDouble P1_roi, P2_roi;
	bool p1_ok=false, p2_ok=false;

	const string name_P0 = is_kitti_dataset ? "P0:" : "P1_roi:";
	const string name_P1 = is_kitti_dataset ? "P1:" : "P2_roi:";

	std::istringstream  ss;
	mrpt::utils::CTextFileLinesParser fp;
	fp.open(calib_file);
	while (fp.getNextLine(ss))
	{
		string sStart; 
		ss >> sStart;
		if (mrpt::system::strStartsI(sStart,name_P0)) {
			P1_roi.loadFromTextFile(ss);
			ASSERT_( P1_roi.cols()==12 && P1_roi.rows()==1 )
			p1_ok=true;
		}
		if (mrpt::system::strStartsI(sStart,name_P1)) {
			P2_roi.loadFromTextFile(ss);
			ASSERT_( P2_roi.cols()==12 && P2_roi.rows()==1 )
			p2_ok=true;
		}
	}

	if (!p1_ok || !p2_ok) throw std::runtime_error("Couldn't load P*_ROI calib matrices!");

	mrpt::utils::TCamera cam_params_l;
	cam_params_l.ncols = 1344;
	cam_params_l.nrows = 391;
	cam_params_l.fx( P2_roi(0,0) );
	cam_params_l.fy( P2_roi(0,5) );
	cam_params_l.cx( P2_roi(0,2) );
	cam_params_l.cy( P2_roi(0,6) );

	mrpt::utils::TCamera cam_params_r = cam_params_l;

	// base = -P2_roi(1,4)/P2_roi(1,1)
	const double baseline = -P2_roi(0,3) / P2_roi(0,0);
	const mrpt::poses::CPose3DQuat l2r_pose(baseline,0.0,0.0, mrpt::math::CQuaternionDouble() );

	cout << "baseline: " <<baseline << endl;
	

	// Create rawlog file ----------------------------------------------
	const string  out_rawlog_fil = out_name + string(".rawlog");
	const string  out_imgs_dir = out_name + string("_Images");
	cout << "Creating rawlog: " << out_rawlog_fil << endl;
	mrpt::utils::CFileGZOutputStream  f_out(out_rawlog_fil);

	if (is_kitti_dataset)
	{
		cout << "Creating imgs dir: " << out_imgs_dir << endl;
		mrpt::system::createDirectory(out_imgs_dir);
	}

	mrpt::system::TTimeStamp tim0 = mrpt::system::now();

	// For each image:
	for (int i=0;  ; i++)
	{
		string sImgFile_L, sImgFile_R;
		string sImg_L, sImg_R;
		string sTrgImgFile_L, sTrgImgFile_R;
		string sTrgImg_L, sTrgImg_R; // If !="", will copy the file.

		if (is_kitti_dataset)
		{
			// 0000000000.png
			// 1234567890
			sImgFile_L = mrpt::format("%010i.png",i);
			sImgFile_R = mrpt::format("%010i.png",i);
			sImg_L = mrpt::format("%s/%s",src_path0.c_str(),sImgFile_L.c_str());
			sImg_R = mrpt::format("%s/%s",src_path1.c_str(),sImgFile_R.c_str());

			sTrgImgFile_L = mrpt::format("I0_%s",sImgFile_L.c_str());
			sTrgImgFile_R = mrpt::format("I1_%s",sImgFile_R.c_str());

			sTrgImg_L = mrpt::format("%s/%s",out_imgs_dir.c_str(),sTrgImgFile_L.c_str());
			sTrgImg_R = mrpt::format("%s/%s",out_imgs_dir.c_str(),sTrgImgFile_R.c_str());
		}
		else
		{
			// I1_000000.png
			// I2_002570.png
			sImgFile_L = mrpt::format("I1_%06i.png",i);
			sImgFile_R = mrpt::format("I2_%06i.png",i);
			sTrgImgFile_L = sImgFile_L;
			sTrgImgFile_R = sImgFile_R;
			sImg_L = mrpt::format("%s/%s",src_path0.c_str(),sImgFile_L.c_str());
			sImg_R = mrpt::format("%s/%s",src_path0.c_str(),sImgFile_R.c_str());
		}

		if (!mrpt::system::fileExists(sImg_L) || !mrpt::system::fileExists(sImg_R) )
		{
			cout << "Couldn't detect image pair "<< sImg_L << " | " << sImg_R <<"  -> ending.\n";
			break;
		}
		
		CObservationStereoImages obs;
		obs.timestamp = mrpt::system::time_tToTimestamp( mrpt::system::timestampTotime_t( tim0 ) + i/STEREO_FPS );
		obs.cameraPose =  mrpt::poses::CPose3DQuat( mrpt::poses::CPose3D(0,0,0, DEG2RAD(-90.0), DEG2RAD(0.0), DEG2RAD(-90.0-4.6) ) );

		obs.leftCamera  = cam_params_l;
		obs.rightCamera = cam_params_r;
		obs.rightCameraPose = l2r_pose;

		obs.imageLeft.setExternalStorage(sTrgImgFile_L);
		obs.imageRight.setExternalStorage(sTrgImgFile_R);

		obs.sensorLabel = "CAMERA1";

		// save:
		f_out << obs;

		// Copy img:
		if (is_kitti_dataset)
		{
			if (!mrpt::system::copyFile(sImg_L,sTrgImg_L)) {
				cerr << "Error copying file: " << sImg_L << " => " << sTrgImg_L << endl;
				return;
			}
			if (!mrpt::system::copyFile(sImg_R,sTrgImg_R)) {
				cerr << "Error copying file: " << sImg_R << " => " << sTrgImg_R << endl;
				return;
			}
		}

		//cout << "Writing entry #" << i << endl;
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
		if (argc!=4 && argc!=5)
		{
			cerr << "Usage:\n"
				"Karlsruhe seqs:\n"
				" kitti_dataset2rawlog  [PATH_TO_DIR_WITH_IMAGES] [CALIB_FILE] [OUTPUT_NAME]\n"
				"KITTI seqs:\n"
				" kitti_dataset2rawlog  [PATH_TO_IMAGE_00] [PATH_TO_IMAGE_01] [CALIB_FILE] [OUTPUT_NAME]\n";
			return 1;
		}
		
		if (argc==4)
		{
			// "Karlsruhe seqs":
			stereo2rawlog(false, argv[1],"",argv[2],argv[3]);
		}
		else
		{
			// "KITTI":
			stereo2rawlog(true, argv[1],argv[2],argv[3],argv[4]);
		}

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

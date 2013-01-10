

#include "demo-simul.h"
#include <mrpt/utils.h> // We use a lot of classes from here
#include <mrpt/system/filesystem.h>  // for ASSERT_FILE_EXISTS_
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>

int main(int argc, char**argv)
{
	try
	{
		return run_demo_simul(argc,argv);
	}
	catch (std::exception &e)
	{
		std::cerr << "**EXCEPTION**:\n" << e.what() << std::endl;
		return 1;
	}
}


#include <sys/types.h>
#include <sys/stat.h>

time_t getFileModificationTime(const std::string &filename)
{
	struct stat fS;
	if (0!=stat( filename.c_str(), &fS)) return 0;
	else return fS.st_mtime;
}


size_t load_simulated_dataset(
	const std::string & FILE_PATH_AND_PREFIX,
	mrpt::math::CMatrixD   &OBS,
	mrpt::aligned_containers<mrpt::poses::CPose3DQuat>::vector_t  &GT_path,
	mrpt::slam::CSimplePointsMap  & GT_MAP,
	mrpt::utils::TCamera          & CAM_CALIB,
	const int verbose_level  )
{
	using namespace std;

	// Load simulated sensor observations
	// ------------------------------------------------------
	// Try with binary cached version (much faster to load!)
	bool obs_cache_found = false;

	const std::string sFil_OBSbin = FILE_PATH_AND_PREFIX + std::string("_SENSOR.bin");
	const std::string sFil_OBS = FILE_PATH_AND_PREFIX + std::string("_SENSOR.txt");


	if (mrpt::system::fileExists(sFil_OBSbin) && mrpt::system::fileExists(sFil_OBS) &&
		getFileModificationTime(sFil_OBSbin)>getFileModificationTime(sFil_OBS) )
	{
		try
		{

			mrpt::utils::CFileGZInputStream f( sFil_OBSbin ); // will throw if file not found

			if (verbose_level>=1) { cout << "Loading binary cache version of dataset...\n"; cout.flush(); }
			f >> OBS;

			obs_cache_found=true;
		}
		catch (std::exception &)
		{
			// Cache not found or corrupt, etc.
		}
	}

	// OBS: Columns are: FRAME_ID   FEAT_ID PIXEL.X  PIXEL.Y
	if (!obs_cache_found)
	{
		// Cache not found: load from text:
		ASSERT_FILE_EXISTS_(sFil_OBS)

		if (verbose_level>=1) { cout << "Loading dataset file: \n -> "<<sFil_OBS<<" ...\n"; cout.flush();}
		OBS.loadFromTextFile(sFil_OBS);

		ASSERT_(OBS.getColCount()==4 || OBS.getColCount()==5)  // pixel (x,y) or pixel (x,y)+range
		ASSERT_ABOVE_(OBS.getRowCount(),2)
	}
	const size_t nTotalObs = OBS.getRowCount();
	if (verbose_level>=1) { cout << "Loaded " << nTotalObs << " observations.\n";}

	if (!obs_cache_found)
	{
		// Save cache:
		try
		{
			mrpt::utils::CFileGZOutputStream f( sFil_OBSbin );
			if (verbose_level>=1) { cout << "Saving binary cache version of dataset...\n"; cout.flush();}
			f << OBS;
			if (verbose_level>=1) { cout << "done.\n"; cout.flush();}
		}
		catch(std::exception &)
		{
			cerr << "Warning: Ignoring error writing binary cache version of dataset.\n";
		}
	}



	// Load GT map of landmarks (so we can compute relative LMs poses)
	// -----------------------------------------------------------------
	bool map_cache_found = false;

	const std::string sFil_MAPbin = FILE_PATH_AND_PREFIX + std::string("_GT_MAP.bin");
	const std::string sFil_MAP    = FILE_PATH_AND_PREFIX + std::string("_GT_MAP.txt");

	if (mrpt::system::fileExists(sFil_MAPbin) && mrpt::system::fileExists(sFil_MAP) &&
		getFileModificationTime(sFil_MAPbin)>getFileModificationTime(sFil_MAP) )
	{
		try
		{
			mrpt::utils::CFileGZInputStream f( sFil_MAPbin ); // will throw if file not found

			if (verbose_level>=1) { cout << "Loading binary cache version of map...\n"; cout.flush();}
			f >> GT_MAP;

			map_cache_found=true;
		}
		catch (std::exception &)
		{
			// Cache not found or corrupt, etc.
		}
	}

	if (!map_cache_found)
	{
		ASSERT_FILE_EXISTS_(sFil_MAP)

		if (verbose_level>=1) { cout << "Loading dataset file: \n -> "<<sFil_MAP<<" ...\n"; cout.flush();}
		GT_MAP.load3D_from_text_file(sFil_MAP);
		ASSERT_ABOVE_(GT_MAP.size(),0)
	}
	const size_t nTotalLMs = GT_MAP.size();
	if (verbose_level>=1) { cout << "Loaded " << nTotalLMs << " landmarks (ground truth map).\n";}

	if (!map_cache_found)
	{
		// Save cache:
		try
		{
			mrpt::utils::CFileGZOutputStream f( sFil_MAPbin );
			if (verbose_level>=1) { cout << "Saving binary cache version of map...\n"; cout.flush();}
			f << GT_MAP;
			if (verbose_level>=1) { cout << "done.\n"; cout.flush();}
		}
		catch(std::exception &)
		{
			cerr << "Warning: Ignoring error writing binary cache version of map.\n";
		}
	}

	// Load GT poses (so we can compute relative LMs poses)
	// -----------------------------------------------------------------
	bool path_cache_found = false;

	const std::string sFil_PATHbin = FILE_PATH_AND_PREFIX + std::string("_GT_PATH.bin");
	const std::string sFil_PATH    = FILE_PATH_AND_PREFIX + std::string("_GT_PATH.txt");

	if (mrpt::system::fileExists(sFil_PATHbin) && mrpt::system::fileExists(sFil_PATH) &&
		getFileModificationTime(sFil_PATHbin)>getFileModificationTime(sFil_PATH) )
	{
		try
		{
			mrpt::utils::CFileGZInputStream f( sFil_PATHbin ); // will throw if file not found

			if (verbose_level>=1) { cout << "Loading binary cache version of path...\n"; cout.flush();}
			f >> GT_path;

			path_cache_found=true;
		}
		catch (std::exception &)
		{
			// Cache not found or corrupt, etc.
		}
	}

	if (!path_cache_found)
	{
		ASSERT_FILE_EXISTS_(sFil_PATH)

		mrpt::utils::CTextFileLinesParser flp(sFil_PATH);
		std::istringstream ss;
		while (flp.getNextLine(ss))
		{
			unsigned int idx;
			double x,y,z,qr,qx,qy,qz;
			if ( (ss >> idx >> x >> y >> z >> qr >> qx >> qy >> qz) )
			{
				if (idx!=GT_path.size())
					THROW_EXCEPTION("Reading _GT_PATH file: Pose IDs expected in ascending order and starting at 0.")

				GT_path.resize(GT_path.size()+1);
				*GT_path.rbegin() = mrpt::poses::CPose3DQuat(x,y,z, mrpt::math::CQuaternionDouble(qr,qx,qy,qz) ) ;
			}
		}
	}
	const size_t nTotalPoses = GT_path.size();
	if (verbose_level>=1) { cout << "Loaded " << nTotalPoses << " trajectory poses (ground truth path).\n";}

	if (!path_cache_found)
	{
		// Save cache:
		try
		{
			mrpt::utils::CFileGZOutputStream f( sFil_PATHbin );
			if (verbose_level>=1) { cout << "Saving binary cache version of path...\n"; cout.flush();}
			f << GT_path;
			if (verbose_level>=1) { cout << "done.\n"; cout.flush();}
		}
		catch(std::exception &)
		{
			cerr << "Warning: Ignoring error writing binary cache version of path.\n";
		}
	}



	// Load Camera calibration:
	// -----------------------------------------------------------------
	const std::string sFil_CAMCALIB = FILE_PATH_AND_PREFIX + std::string("_CAMCALIB.txt");
	ASSERT_FILE_EXISTS_(sFil_CAMCALIB)
	CAM_CALIB.loadFromConfigFile("CAMERA",mrpt::utils::CConfigFile(sFil_CAMCALIB));

	return nTotalObs;
}


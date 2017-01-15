/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */
#include <mrpt/pbmap.h> // precomp. hdr

#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace mrpt::system;
using namespace mrpt::pbmap;

string path("/home/edu/Libraries/mrpt-svn/share/mrpt/datasets/pbmap-demos/");

void printHelp()
{
  cout << "./pbmap_test reads a set of pairs pointCloud+Pose to build a PbMap\n";
}

void testPbMapConstruction(const string &config_file)
{
  printHelp();

  // Reconstructed PointCloud
  pcl::PointCloud<PointT> globalCloud;

  PbMapMaker pbmap_maker(config_file);

  // Read in the cloud data
  pcl::PCDReader reader;

  unsigned N = 3; // Read the 5 sample pairs pointClouds+Pose
  string cloudFile, poseFile;
  for(unsigned i=0; i <= N; i++)
  {
    // Read frame
    frameRGBDandPose cloudAndPose;
    cloudAndPose.cloudPtr.reset(new pcl::PointCloud<PointT>);
    cloudFile = mrpt::format("pointcloud%i.pcd", i);
    ASSERT_FILE_EXISTS_(path + cloudFile)
    reader.read (path + cloudFile, *cloudAndPose.cloudPtr);

    // Read pose
    mrpt::utils::CFileGZInputStream serialized_pose;
    poseFile = path + mrpt::format("pose%i.mat", i);

    if (serialized_pose.open(poseFile))
    {
      serialized_pose.ReadBufferFixEndianness<Eigen::Matrix4f::Scalar>(&cloudAndPose.pose(0),16);
    }
    else
      cout << "Error: cannot open " << poseFile << "\n";
    serialized_pose.close();

    // Detect planes and build PbMap
    pbmap_maker.frameQueue.push_back(cloudAndPose);

    pcl::PointCloud<PointT>::Ptr alignedCloudPtr(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloudAndPose.cloudPtr,*alignedCloudPtr,cloudAndPose.pose);
    globalCloud += *alignedCloudPtr;

    mrpt::system::sleep(1000); // sleep to visualize the map creation from the keyframes in slow motion
  }

  // Serialize PbMap
  mrpt::utils::CFileGZOutputStream serialize_pbmap("test.pbmap");
  serialize_pbmap << pbmap_maker.getPbMap();
  serialize_pbmap.close();

  // Save reconstructed point cloud
  pcl::io::savePCDFile("reconstructed_cloud.pcd", globalCloud);

  double total_area = 0.0;
  for(unsigned i=0; i < pbmap_maker.getPbMap().vPlanes.size(); i++)
    total_area += pbmap_maker.getPbMap().vPlanes[i].areaHull;
  cout << "This PbMap contains " << pbmap_maker.getPbMap().vPlanes.size() << " planes, covering a total area of " << total_area << " m2" << endl;

  mrpt::system::sleep(10000);
}

int main(int argc, char **argv)
{

  try
  {
		bool showHelp    = argc>1 && !os::_strcmp(argv[1],"--help");

		// Process arguments:
		if (argc<2 || showHelp )
		{
			printf("Usage: %s <config_file.ini>\n\n",argv[0]);
			if (!showHelp)
			{
				mrpt::system::pause();
				return -1;
			}
			else	return 0;
		}

    const string INI_FILENAME = string( argv[1] );

    testPbMapConstruction(INI_FILENAME);

    return 0;

  } catch (exception &e)
  {
    cout << "MRPT exception caught: " << e.what() << endl;
    return -1;
  }
  catch (...)
  {
    printf("Another exception!!");
    return -1;
  }
}

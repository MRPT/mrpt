/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs.h>    // For loading from the rawlog
#include <mrpt/maps.h>   // For converting into point maps
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system.h>
#include <mrpt/synch.h>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


size_t rawlogEntry=0;


struct ThreadData
{
	ThreadData() : new_timestamp(INVALID_TIMESTAMP) { }

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  new_cloud;
	mrpt::system::TTimeStamp  new_timestamp;
};

ThreadData                    td;
mrpt::synch::CCriticalSection td_cs;


void viewerUpdate(pcl::visualization::PCLVisualizer& viewer)
{
    std::stringstream ss;
    ss << "Rawlog entry: " << rawlogEntry;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 10,50, "text", 0);

    static mrpt::system::TTimeStamp last_time = INVALID_TIMESTAMP;

    {  // Mutex protected
    	mrpt::synch::CCriticalSectionLocker lock(&td_cs);
    	if (td.new_timestamp!=last_time)
    	{
    		last_time = td.new_timestamp;
			viewer.removePointCloud("cloud", 0);
			viewer.addPointCloud (td.new_cloud,"cloud",0);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0);

			const size_t N = td.new_cloud->size();
			std::cout << "Showing new point cloud of size=" << N << std::endl;

			static bool first = true;
			if (N && first)
			{
				first = false;
			//viewer.resetCameraViewpoint("cloud");
			}

#if 0
			std::cout << mrpt::format(
				"camera: clip = %f %f\n"
				"        focal = %f %f %f\n"
				"        pos   = %f %f %f\n"
				"        view  = %f %f %f\n",
				viewer.camera_.clip[0],viewer.camera_.clip[1],
				viewer.camera_.focal[0],viewer.camera_.focal[1],viewer.camera_.focal[2],
				viewer.camera_.pos[0],viewer.camera_.pos[1],viewer.camera_.pos[2],
				viewer.camera_.view[0],viewer.camera_.view[1],viewer.camera_.view[2]);
#endif
    	}
    }
}

void  viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0.3, 0.3, 0.3);
    viewer.addCoordinateSystem(1.0, 0);
    viewer.initCameraParameters();
    viewer.camera_.pos[2] = 30;
	viewer.updateCamera();
}

int main(int argc, char**argv)
{
	try
	{
		if (argc!=2)
		{
			std::cerr << "Usage: " << argv[0] << " <DATASET.rawlog>\n";
			return 1;
		}


		std::cout << "Opening: " << argv[1] << std::endl;
		mrpt::utils::CFileGZInputStream  fil(argv[1]);
		bool rawlog_eof = false;

		pcl::visualization::CloudViewer viewer("Cloud Viewer from MRPT's rawlog");

		//This will only get called once
		viewer.runOnVisualizationThreadOnce (viewerOneOff);

		//This will get called once per visualization iteration
		viewer.runOnVisualizationThread (viewerUpdate);

		while (!viewer.wasStopped ())
		{
			mrpt::slam::CActionCollectionPtr	actions;
			mrpt::slam::CSensoryFramePtr		SF;
			mrpt::slam::CObservationPtr			obs;

			if (!rawlog_eof)
			{
				if (!mrpt::slam::CRawlog::getActionObservationPairOrObservation(fil, actions, SF, obs,rawlogEntry))
				{
					rawlog_eof = true;
					std::cerr << "End of rawlog file!! Close the window to exit\n";
				}
				else
				{
					// Can generate a point cloud from this data?
					// TODO: Process Kinect observations differently to extract RGB data.
					mrpt::slam::CPointsMapPtr  new_map;
					if (SF)
					{
						new_map = mrpt::slam::CSimplePointsMap::Create();
						// new_map->insertionOptions.minDistBetweenLaserPoints = 0;
						SF->insertObservationsInto(new_map);
					}
					else if (obs)
					{
						new_map = mrpt::slam::CSimplePointsMap::Create();
						// new_map->insertionOptions.minDistBetweenLaserPoints = 0;
						new_map->insertObservation(obs.pointer());
					}

					if (new_map)
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

						// Convert MRPT point maps -> PCL point cloud.
						new_map->getPCLPointCloud(*cloud);

						{  // Mutex protected
							mrpt::synch::CCriticalSectionLocker lock(&td_cs);
							td.new_timestamp = mrpt::system::now();
							td.new_cloud = cloud;
						}

						mrpt::system::sleep(30);  // Delay to allow the point cloud to show up.
					}

				}
			}

			mrpt::system::sleep(1);
		}
		return 0;
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
}

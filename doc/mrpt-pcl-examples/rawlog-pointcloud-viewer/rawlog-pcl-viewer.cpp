/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs.h>    // For loading from the rawlog
#include <mrpt/maps.h>   // For converting into point maps
#include <mrpt/io/CFileGZInputStream.h>
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
std::mutex td_cs;


void viewerUpdate(pcl::visualization::PCLVisualizer& viewer)
{
    std::stringstream ss;
    ss << "Rawlog entry: " << rawlogEntry;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 10,50, "text", 0);

    static mrpt::system::TTimeStamp last_time = INVALID_TIMESTAMP;

    {  // Mutex protected
    	std::lock_guard<std::mutex> lock(td_cs);
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
		mrpt::io::CFileGZInputStream  fil(argv[1]);
		bool rawlog_eof = false;

		pcl::visualization::CloudViewer viewer("Cloud Viewer from MRPT's rawlog");

		//This will only get called once
		viewer.runOnVisualizationThreadOnce (viewerOneOff);

		//This will get called once per visualization iteration
		viewer.runOnVisualizationThread (viewerUpdate);

		while (!viewer.wasStopped ())
		{
			mrpt::obs::CActionCollection::Ptr	actions;
			mrpt::obs::CSensoryFrame::Ptr		SF;
			mrpt::obs::CObservation::Ptr			obs;

			if (!rawlog_eof)
			{
				if (!mrpt::obs::CRawlog::getActionObservationPairOrObservation(fil, actions, SF, obs,rawlogEntry))
				{
					rawlog_eof = true;
					std::cerr << "End of rawlog file!! Close the window to exit\n";
				}
				else
				{
					// Can generate a point cloud from this data?
					// TODO: Process Kinect observations differently to extract RGB data.
					mrpt::maps::CPointsMap::Ptr  new_map;
					if (SF)
					{
						new_map = mrpt::make_aligned_shared<mrpt::maps::CSimplePointsMap>();
						// new_map->insertionOptions.minDistBetweenLaserPoints = 0;
						SF->insertObservationsInto(new_map);
					}
					else if (obs)
					{
						new_map = mrpt::make_aligned_shared<mrpt::maps::CSimplePointsMap>();
						// new_map->insertionOptions.minDistBetweenLaserPoints = 0;
						new_map->insertObservation(obs.get());
					}

					if (new_map)
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

						// Convert MRPT point maps -> PCL point cloud.
						new_map->getPCLPointCloud(*cloud);

						{  // Mutex protected
							std::lock_guard<std::mutex> lock(td_cs);
							td.new_timestamp = mrpt::system::now();
							td.new_cloud = cloud;
						}

						std::this_thread::sleep_for(30ms);  // Delay to allow the point cloud to show up.
					}

				}
			}

			std::this_thread::sleep_for(1ms);
		}
		return 0;
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
}

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

#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::random;
using namespace mrpt::gui;
using namespace std;



#include <mrpt/examples_config.h>
const string   sample_simplemap_file = MRPT_EXAMPLES_BASE_DIRECTORY + string("../share/mrpt/datasets/localization_demo.simplemap.gz");

// ------------------------------------------------------
//				TestVoronoi
// ------------------------------------------------------
void TestVoronoi()
{
	if (!mrpt::system::fileExists(sample_simplemap_file))
	{
		cerr << "Error: file doesn't exist: " << sample_simplemap_file << endl;
		return;
	}

	// Load simplemap:
	cout << "Loading simplemap: " << sample_simplemap_file << endl;

	CSimpleMap  simplemap;
	simplemap.loadFromFile(sample_simplemap_file);

	// Load a grid map:
	cout << "Building gridmap...\n";

	COccupancyGridMap2D  gridmap(-5,5,  -5,5,  0.10);
	gridmap.loadFromSimpleMap(simplemap);

	// Build voronoi:
	cout << "Building Voronoi diagram...\n";

	gridmap.buildVoronoiDiagram(0.5,0.3);


	// Show results:
	CImage img_grid;
	gridmap.getAsImage(img_grid);

	CImage img_voronoi;
	CMatrixDouble mat_voronoi;
	gridmap.getVoronoiDiagram().getAsMatrix(mat_voronoi);
	img_voronoi.setFromMatrix(mat_voronoi, false /* do normalization */ );

	// Show results:
	CDisplayWindow   win1("Grid map");
	win1.showImage(img_grid);

	CDisplayWindow   win2("Voronoi map");
	win2.showImage(img_voronoi);

	mrpt::system::pause();
}

int main(int argc, char **argv)
{
	try
	{
		TestVoronoi();
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


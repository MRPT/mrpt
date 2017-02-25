/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/system/filesystem.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::maps;
using namespace mrpt::random;
using namespace mrpt::gui;
using namespace mrpt::math;
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


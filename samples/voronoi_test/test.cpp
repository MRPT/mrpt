/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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


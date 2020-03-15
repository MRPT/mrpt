/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/system/os.h>
#include <iostream>

//#define HAS_SYSTEM_OCTOMAP

#ifdef HAS_SYSTEM_OCTOMAP
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#endif

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace std;

// ------------------------------------------------------
//				TestOctoMap
// ------------------------------------------------------
void TestOctoMap()
{
	COctoMap map(0.2);

	if (false)
	{
		// Manually update voxels:
		map.updateVoxel(1, 1, 1, true);  // integrate 'occupied' measurement

		map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement
		map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement
		map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement

		map.updateVoxel(-1, -1, 1, false);  // integrate 'occupied' measurement

		double occup;
		bool is_mapped;
		mrpt::math::TPoint3D pt;

		pt = mrpt::math::TPoint3D(1, 1, 1);
		is_mapped = map.getPointOccupancy(pt.x, pt.y, pt.z, occup);
		cout << "pt: " << pt << " is mapped?: " << (is_mapped ? "YES" : "NO")
			 << " occupancy: " << occup << endl;

		pt = mrpt::math::TPoint3D(-1, -1, 1);
		is_mapped = map.getPointOccupancy(pt.x, pt.y, pt.z, occup);
		cout << "pt: " << pt << " is mapped?: " << (is_mapped ? "YES" : "NO")
			 << " occupancy: " << occup << endl;
	}

	// Insert 2D scan:
	{
		CObservation2DRangeScan scan1;
		stock_observations::example2DRangeScan(scan1);
		map.insertObservation(scan1);
	}

	mrpt::gui::CDisplayWindow3D win("OctoMap demo", 640, 480);

	mrpt::opengl::COctoMapVoxels::Ptr gl_map =
		mrpt::opengl::COctoMapVoxels::Create();

	{
		mrpt::opengl::COpenGLScene::Ptr& scene = win.get3DSceneAndLock();

		{
			mrpt::opengl::CGridPlaneXY::Ptr gl_grid =
				mrpt::opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
			gl_grid->setColor_u8(mrpt::img::TColor(0x80, 0x80, 0x80));
			scene->insert(gl_grid);
		}

		map.renderingOptions.generateGridLines = true;
		map.getAsOctoMapVoxels(*gl_map);

		gl_map->showGridLines(false);
		gl_map->showVoxels(VOXEL_SET_OCCUPIED, true);
		gl_map->showVoxels(VOXEL_SET_FREESPACE, true);
		scene->insert(gl_map);

		win.unlockAccess3DScene();
	}

// Go through voxels:
#ifdef HAS_SYSTEM_OCTOMAP
	{
		const auto& om = map.getOctomap<octomap::OcTree>();
		for (auto it = om.begin_leafs(); it != om.end_leafs(); ++it)
		{
			const octomap::point3d pt = it.getCoordinate();
			cout << "pt: " << pt << " -> occupancy = " << it->getOccupancy()
				 << endl;
		}
	}
#endif

	cout << "Close the window to exit" << endl;

	bool update_msg = true;

	while (win.isOpen())
	{
		if (win.keyHit())
		{
			const unsigned int k = win.getPushedKey();

			switch (k)
			{
				case 'g':
				case 'G':
					gl_map->showGridLines(!gl_map->areGridLinesVisible());
					break;
				case 'f':
				case 'F':
					gl_map->showVoxels(
						VOXEL_SET_FREESPACE,
						!gl_map->areVoxelsVisible(VOXEL_SET_FREESPACE));
					break;
				case 'o':
				case 'O':
					gl_map->showVoxels(
						VOXEL_SET_OCCUPIED,
						!gl_map->areVoxelsVisible(VOXEL_SET_OCCUPIED));
					break;
				case 'l':
				case 'L':
					gl_map->enableLights(!gl_map->areLightsEnabled());
					break;
			};
			update_msg = true;
		}

		if (update_msg)
		{
			update_msg = false;

			win.addTextMessage(
				5, 5,
				mrpt::format(
					"Commands: 'g' (grids=%s) | 'f' (freespace=%s) | 'o' "
					"(occupied=%s) | 'l' (lights=%s)",
					gl_map->areGridLinesVisible() ? "YES" : "NO",
					gl_map->areVoxelsVisible(VOXEL_SET_FREESPACE) ? "YES"
																  : "NO",
					gl_map->areVoxelsVisible(VOXEL_SET_OCCUPIED) ? "YES" : "NO",
					gl_map->areLightsEnabled() ? "YES" : "NO"));

			win.repaint();
		}

		std::this_thread::sleep_for(10ms);
	};
}

int main(int argc, char** argv)
{
	try
	{
		TestOctoMap();
		return 0;
	}
	catch (exception& e)
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

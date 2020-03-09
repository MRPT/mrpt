/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/system/os.h>
#include <iostream>

// ------------------------------------------------------
//				TestGridMap3D
// ------------------------------------------------------
void TestGridMap3D()
{
	mrpt::maps::COccupancyGridMap3D map;
	const float resolution = 0.10;  // [meters]
	map.setSize(
		mrpt::math::TPoint3D(-3, -5, -2), mrpt::math::TPoint3D(10, 5, 2),
		resolution);

	// Insert 2D scan:
	{
		mrpt::obs::CObservation2DRangeScan scan1;
		mrpt::obs::stock_observations::example2DRangeScan(scan1);
		map.insertObservation(scan1);
	}

	mrpt::gui::CDisplayWindow3D win("GridMap3D demo", 640, 480);

	auto gl_map = mrpt::opengl::COctoMapVoxels::Create();

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
		gl_map->showVoxels(mrpt::opengl::VOXEL_SET_OCCUPIED, true);
		gl_map->showVoxels(mrpt::opengl::VOXEL_SET_FREESPACE, true);
		scene->insert(gl_map);

		win.unlockAccess3DScene();
	}

	std::cout << "Close the window to exit" << std::endl;

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
						mrpt::opengl::VOXEL_SET_FREESPACE,
						!gl_map->areVoxelsVisible(
							mrpt::opengl::VOXEL_SET_FREESPACE));
					break;
				case 'o':
				case 'O':
					gl_map->showVoxels(
						mrpt::opengl::VOXEL_SET_OCCUPIED,
						!gl_map->areVoxelsVisible(
							mrpt::opengl::VOXEL_SET_OCCUPIED));
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
					gl_map->areVoxelsVisible(mrpt::opengl::VOXEL_SET_FREESPACE)
						? "YES"
						: "NO",
					gl_map->areVoxelsVisible(mrpt::opengl::VOXEL_SET_OCCUPIED)
						? "YES"
						: "NO",
					gl_map->areLightsEnabled() ? "YES" : "NO"));

			win.repaint();
		}

		using namespace std::chrono_literals;
		std::this_thread::sleep_for(10ms);
	};
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
	try
	{
		TestGridMap3D();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}

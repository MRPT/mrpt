/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/maps/COctoMap.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>  // for sleep()
#include <mrpt/gui/CDisplayWindow3D.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace std;

#define SCAN_SIZE 361

float SCAN_RANGES_1[] = {0.910f,0.900f,0.910f,0.900f,0.900f,0.890f,0.890f,0.880f,0.890f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.870f,0.880f,0.870f,0.870f,0.870f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.890f,0.880f,0.880f,0.880f,0.890f,0.880f,0.890f,0.890f,0.880f,0.890f,0.890f,0.880f,0.890f,0.890f,0.890f,0.890f,0.890f,0.890f,0.900f,0.900f,0.900f,0.900f,0.900f,0.910f,0.910f,0.910f,0.910f,0.920f,0.920f,0.920f,0.920f,0.920f,0.930f,0.930f,0.930f,0.930f,0.940f,0.940f,0.950f,0.950f,0.950f,0.950f,0.960f,0.960f,0.970f,0.970f,0.970f,0.980f,0.980f,0.990f,1.000f,1.000f,1.000f,1.010f,1.010f,1.020f,1.030f,1.030f,1.030f,1.040f,1.050f,1.060f,1.050f,1.060f,1.070f,1.070f,1.080f,1.080f,1.090f,1.100f,1.110f,1.120f,1.120f,1.130f,1.140f,1.140f,1.160f,1.170f,1.180f,1.180f,1.190f,1.200f,1.220f,1.220f,1.230f,1.230f,1.240f,1.250f,1.270f,1.280f,1.290f,1.300f,1.320f,1.320f,1.350f,1.360f,1.370f,1.390f,1.410f,1.410f,1.420f,1.430f,1.450f,1.470f,1.490f,1.500f,1.520f,1.530f,1.560f,1.580f,1.600f,1.620f,1.650f,1.670f,1.700f,1.730f,1.750f,1.780f,1.800f,1.830f,1.850f,1.880f,1.910f,1.940f,1.980f,2.010f,2.060f,2.090f,2.130f,2.180f,2.220f,2.250f,2.300f,2.350f,2.410f,2.460f,2.520f,2.570f,2.640f,2.700f,2.780f,2.850f,2.930f,3.010f,3.100f,3.200f,3.300f,3.390f,3.500f,3.620f,3.770f,3.920f,4.070f,4.230f,4.430f,4.610f,4.820f,5.040f,5.290f,5.520f,8.970f,8.960f,8.950f,8.930f,8.940f,8.930f,9.050f,9.970f,9.960f,10.110f,13.960f,18.870f,19.290f,81.910f,20.890f,48.750f,48.840f,48.840f,19.970f,19.980f,19.990f,15.410f,20.010f,19.740f,17.650f,17.400f,14.360f,12.860f,11.260f,11.230f,8.550f,8.630f,9.120f,9.120f,8.670f,8.570f,7.230f,7.080f,7.040f,6.980f,6.970f,5.260f,5.030f,4.830f,4.620f,4.440f,4.390f,4.410f,4.410f,4.410f,4.430f,4.440f,4.460f,4.460f,4.490f,4.510f,4.540f,3.970f,3.820f,3.730f,3.640f,3.550f,3.460f,3.400f,3.320f,3.300f,3.320f,3.320f,3.340f,2.790f,2.640f,2.600f,2.570f,2.540f,2.530f,2.510f,2.490f,2.490f,2.480f,2.470f,2.460f,2.460f,2.460f,2.450f,2.450f,2.450f,2.460f,2.460f,2.470f,2.480f,2.490f,2.490f,2.520f,2.510f,2.550f,2.570f,2.610f,2.640f,2.980f,3.040f,3.010f,2.980f,2.940f,2.920f,2.890f,2.870f,2.830f,2.810f,2.780f,2.760f,2.740f,2.720f,2.690f,2.670f,2.650f,2.630f,2.620f,2.610f,2.590f,2.560f,2.550f,2.530f,2.510f,2.500f,2.480f,2.460f,2.450f,2.430f,2.420f,2.400f,2.390f,2.380f,2.360f,2.350f,2.340f,2.330f,2.310f,2.300f,2.290f,2.280f,2.270f,2.260f,2.250f,2.240f,2.230f,2.230f,2.220f,2.210f,2.200f,2.190f,2.180f,2.170f,1.320f,1.140f,1.130f,1.130f,1.120f,1.120f,1.110f,1.110f,1.110f,1.110f,1.100f,1.110f,1.100f};
char  SCAN_VALID_1[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

// ------------------------------------------------------
//				TestOctoMap
// ------------------------------------------------------
void TestOctoMap()
{
	COctoMap  map(0.2);

	if (0)
	{
		// Manually update voxels:
		map.updateVoxel(1,1,1, true); // integrate 'occupied' measurement

		map.updateVoxel(1.5,1,1, true); // integrate 'occupied' measurement
		map.updateVoxel(1.5,1,1, true); // integrate 'occupied' measurement
		map.updateVoxel(1.5,1,1, true); // integrate 'occupied' measurement

		map.updateVoxel(-1,-1,1, false); // integrate 'occupied' measurement

		double occup;
		bool   is_mapped;
		mrpt::math::TPoint3D  pt;

		pt = mrpt::math::TPoint3D(1,1,1);
		is_mapped = map.getPointOccupancy(pt.x,pt.y,pt.z, occup);
		cout << "pt: " << pt << " is mapped?: "<< (is_mapped ? "YES":"NO") <<" occupancy: " << occup << endl;

		pt = mrpt::math::TPoint3D(-1,-1,1);
		is_mapped = map.getPointOccupancy(pt.x,pt.y,pt.z, occup);
		cout << "pt: " << pt << " is mapped?: "<< (is_mapped ? "YES":"NO") <<" occupancy: " << occup << endl;
	}

	// Insert 2D scan:
	{
		CObservation2DRangeScan	scan1;
		scan1.aperture = M_PIf;
		scan1.rightToLeft = true;
        ASSERT_( sizeof(SCAN_RANGES_1) == sizeof(float)*SCAN_SIZE );
        scan1.loadFromVectors( SCAN_SIZE, SCAN_RANGES_1, SCAN_VALID_1);
		map.insertObservation( &scan1 );
	}


	mrpt::gui::CDisplayWindow3D  win("OctoMap demo", 640,480);

	mrpt::opengl::COctoMapVoxelsPtr gl_map = mrpt::opengl::COctoMapVoxels::Create();

	{
		mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();

		{
			mrpt::opengl::CGridPlaneXYPtr gl_grid = mrpt::opengl::CGridPlaneXY::Create(-20,20, -20,20, 0, 1);
			gl_grid->setColor_u8(mrpt::utils::TColor(0x80,0x80,0x80));
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
	if (1)
	{
		const COctoMap::octree_t &om = map.getOctomap();

		for (COctoMap::octree_t::leaf_iterator it=om.begin_leafs();it!=om.end_leafs(); ++it)
		{
			const octomap::point3d pt = it.getCoordinate();
			cout << "pt: " << pt << " -> occupancy = " << it->getOccupancy() << endl;
		}
	}


	cout << "Close the window to exit" << endl;

	bool update_msg = true;

	while (win.isOpen())
	{
		if (win.keyHit())
		{
			const unsigned int k = win.getPushedKey();

			switch(k)
			{
			case 'g':
			case 'G':
				gl_map->showGridLines( !gl_map->areGridLinesVisible() );
				break;
			case 'f':
			case 'F':
				gl_map->showVoxels(VOXEL_SET_FREESPACE, !gl_map->areVoxelsVisible(VOXEL_SET_FREESPACE) );
				break;
			case 'o':
			case 'O':
				gl_map->showVoxels(VOXEL_SET_OCCUPIED, !gl_map->areVoxelsVisible(VOXEL_SET_OCCUPIED) );
				break;
			case 'l':
			case 'L':
				gl_map->enableLights( !gl_map->areLightsEnabled() );
				break;
			};
			update_msg = true;

		}

		if (update_msg)
		{
			update_msg= false;

			win.addTextMessage(
				5,5,mrpt::format("Commands: 'g' (grids=%s) | 'f' (freespace=%s) | 'o' (occupied=%s) | 'l' (lights=%s)",
					gl_map->areGridLinesVisible() ? "YES":"NO",
					gl_map->areVoxelsVisible(VOXEL_SET_FREESPACE) ? "YES":"NO",
					gl_map->areVoxelsVisible(VOXEL_SET_OCCUPIED) ? "YES":"NO",
					gl_map->areLightsEnabled() ? "YES":"NO"
					),
				TColorf(1,1,1),"sans",15);

			win.repaint();
		}

		mrpt::system::sleep(10);
	};


}

int main(int argc, char **argv)
{
	try
	{
		TestOctoMap();
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

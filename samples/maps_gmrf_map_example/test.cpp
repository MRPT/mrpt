/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/gui.h>
#include <mrpt/random.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CPointCloud.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

// Example of custom connectivity pattern:
struct MyConnectivityVisitor
	: public mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor
{
	/** Implement the check of whether node i=(icx,icy) is connected with node
	 * j=(jcx,jcy).
	 * This visitor method will be called only for immediate neighbors.
	 * \return true if connected (and the "information" value should be also
	 * updated in out_edge_information), false otherwise.
	 */
	bool getEdgeInformation(
		/** The parent map on which we are running */
		const CRandomFieldGridMap2D* parent,
		/** (cx,cy) for node "i" */
		size_t icx, size_t icy,
		/** (cx,cy) for node "j" */
		size_t jcx, size_t jcy,
		/** Must output here the inverse of the variance of the constraint
		   edge. */
		double& out_edge_information) override
	{
		out_edge_information = 1.0 / (1.0 + icx + icy);
		return true;
	}
};

void Example_GMRF()
{
	const double X_SIZE = 10.0;
	const double Y_SIZE = 10.0;
	const double RESOLUTION = 0.5;

	mrpt::maps::CGasConcentrationGridMap2D gasmap(
		CRandomFieldGridMap2D::mrGMRF_SD /*map type*/, 0, X_SIZE, 0, Y_SIZE,
		RESOLUTION /* resolution */
	);

	mrpt::maps::CGasConcentrationGridMap2D::ConnectivityDescriptor::Ptr conn =
		mrpt::maps::CGasConcentrationGridMap2D::ConnectivityDescriptor::Ptr(
			new MyConnectivityVisitor);
	gasmap.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
	gasmap.setCellsConnectivity(conn);
	gasmap.clear();  // for the connectivity to be taken into account.

	mrpt::opengl::CPointCloud::Ptr gl_data =
		mrpt::make_aligned_shared<mrpt::opengl::CPointCloud>();
	gl_data->setPointSize(3.0f);

	for (int i = 0; i < 20; i++)
	{
		const double value = getRandomGenerator().drawUniform(0.01, 0.99);
		const double x = getRandomGenerator().drawUniform(0.1, 0.95 * X_SIZE);
		const double y = getRandomGenerator().drawUniform(0.1, 0.95 * Y_SIZE);

		printf(
			"Observation: (x,y)=(%6.02f,%6.02f,)  => value: %6.03f\n", x, y,
			value);
		gl_data->insertPoint(x, y, value);

		gasmap.insertIndividualReading(
			value, TPoint2D(x, y), false /*dont update map now*/);
	}

	// Update only once now:
	gasmap.updateMapEstimation();

	// 3D view:
	mrpt::opengl::CSetOfObjects::Ptr glObj =
		mrpt::make_aligned_shared<mrpt::opengl::CSetOfObjects>();
	gasmap.getAs3DObject(glObj);

	mrpt::gui::CDisplayWindow3D win("Map", 640, 480);

	mrpt::opengl::COpenGLScene::Ptr& scene = win.get3DSceneAndLock();
	scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 4.0f));
	scene->insert(gl_data);
	scene->insert(glObj);
	win.unlockAccess3DScene();
	win.repaint();

	win.waitForKey();
}

int main(int argc, char** argv)
{
	try
	{
		Example_GMRF();
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

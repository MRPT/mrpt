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
#include <mrpt/maps.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>


using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::slam;


//Increase this values to get more precision. It will also increase run time.
const size_t HOW_MANY_YAWS=120;
const size_t HOW_MANY_PITCHS=120;


// The scans of the 3D object, taken from 2 different places:
vector<CObservation2DRangeScan> sequence_scans1, sequence_scans2;

// The two origins for the 3D scans
CPose3D viewpoint1(-0.3,0.7,3, DEG2RAD(5),DEG2RAD(80),DEG2RAD(3));
CPose3D	viewpoint2(0.5,-0.2,2.6, DEG2RAD(-5),DEG2RAD(100),DEG2RAD(-7));

CPose3D SCAN2_POSE_ERROR (0.15,-0.07,0.10, -0.03, 0.1, 0.1 );


void generateObjects(CSetOfObjectsPtr &world)
{
	CSpherePtr sph=CSphere::Create(0.5);
	sph->setLocation(0,0,0);
	sph->setColor(1,0,0);
	world->insert(sph);

	CDiskPtr pln= opengl::CDisk::Create();
	pln->setDiskRadius(2);
	pln->setPose(CPose3D(0,0,0,0,DEG2RAD(5),DEG2RAD(5)));
	pln->setColor(0.8,0,0);
	world->insert(pln);

	{
		CDiskPtr pln= opengl::CDisk::Create();
		pln->setDiskRadius(2);
		pln->setPose(CPose3D(0,0,0,DEG2RAD(30),DEG2RAD(-20),DEG2RAD(-2)));
		pln->setColor(0.9,0,0);
		world->insert(pln);
	}
}

void test_icp3D()
{
	// Create the reference objects:
	COpenGLScenePtr scene1=COpenGLScene::Create();
	COpenGLScenePtr scene2=COpenGLScene::Create();
	COpenGLScenePtr scene3=COpenGLScene::Create();

	opengl::CGridPlaneXYPtr plane1=CGridPlaneXY::Create(-20,20,-20,20,0,1);
	plane1->setColor(0.3,0.3,0.3);
	scene1->insert(plane1);
	scene2->insert(plane1);
	scene3->insert(plane1);

	CSetOfObjectsPtr world=CSetOfObjects::Create();
	generateObjects(world);
	scene1->insert(world);

	// Perform the 3D scans:
	CAngularObservationMeshPtr aom1=CAngularObservationMesh::Create();
	CAngularObservationMeshPtr aom2=CAngularObservationMesh::Create();

	cout << "Performing ray-tracing..." << endl;
	CAngularObservationMesh::trace2DSetOfRays(scene1,viewpoint1,aom1,CAngularObservationMesh::TDoubleRange::CreateFromAperture(M_PI,HOW_MANY_PITCHS),CAngularObservationMesh::TDoubleRange::CreateFromAperture(M_PI,HOW_MANY_YAWS));
	CAngularObservationMesh::trace2DSetOfRays(scene1,viewpoint2,aom2,CAngularObservationMesh::TDoubleRange::CreateFromAperture(M_PI,HOW_MANY_PITCHS),CAngularObservationMesh::TDoubleRange::CreateFromAperture(M_PI,HOW_MANY_YAWS));
	cout << "Ray-tracing done" << endl;



	// Put the viewpoints origins:
	{
		CSetOfObjectsPtr origin1= opengl::stock_objects::CornerXYZ();
		origin1->setPose(viewpoint1);
		origin1->setScale(0.6);
		scene1->insert( origin1 );
		scene2->insert( origin1 );
	}
	{
		CSetOfObjectsPtr origin2= opengl::stock_objects::CornerXYZ();
		origin2->setPose(viewpoint2);
		origin2->setScale(0.6);
		scene1->insert( origin2 );
		scene2->insert( origin2 );
	}


	// Show the scanned points:
	CSimplePointsMap	M1,M2;

	aom1->generatePointCloud(&M1);
	aom2->generatePointCloud(&M2);

	// Create the wrongly-localized M2:
	CSimplePointsMap	M2_noisy;
	M2_noisy = M2;
	M2_noisy.changeCoordinatesReference( SCAN2_POSE_ERROR );


	CSetOfObjectsPtr  PTNS1 = CSetOfObjects::Create();
	CSetOfObjectsPtr  PTNS2 = CSetOfObjects::Create();

	CPointsMap::COLOR_3DSCENE_R = 1;
	CPointsMap::COLOR_3DSCENE_G = 0;
	CPointsMap::COLOR_3DSCENE_B = 0;
	M1.getAs3DObject(PTNS1);

	CPointsMap::COLOR_3DSCENE_R = 0;
	CPointsMap::COLOR_3DSCENE_G = 0;
	CPointsMap::COLOR_3DSCENE_B = 1;
	M2_noisy.getAs3DObject(PTNS2);

	scene2->insert( PTNS1 );
	scene2->insert( PTNS2 );

	// --------------------------------------
	// Do the ICP-3D
	// --------------------------------------
	float run_time;
	CICP	icp;
	CICP::TReturnInfo	icp_info;

	icp.options.thresholdDist = 0.40;
	icp.options.thresholdAng = 0;

	CPose3DPDFPtr pdf= icp.Align3D(
		&M2_noisy,    // Map to align
		&M1,          // Reference map
		CPose3D(),    // Initial gross estimate
		&run_time,
		&icp_info);

	CPose3D  mean = pdf->getMeanVal();

	cout << "ICP run took " << run_time << " secs." << endl;
	cout << "Goodness: " << 100*icp_info.goodness << "% , # of iterations= " << icp_info.nIterations << endl;
	cout << "ICP output: mean= " << mean << endl;
	cout << "Real displacement: " << SCAN2_POSE_ERROR  << endl;


	// Aligned maps:
	CSetOfObjectsPtr  PTNS2_ALIGN = CSetOfObjects::Create();

	M2_noisy.changeCoordinatesReference( CPose3D()-mean );
	M2_noisy.getAs3DObject(PTNS2_ALIGN);

	scene3->insert( PTNS1 );
	scene3->insert( PTNS2_ALIGN );


	// Show in Windows:
	CDisplayWindow3D window("ICP-3D demo: scene",500,500);
	CDisplayWindow3D window2("ICP-3D demo: UNALIGNED scans",500,500);
	CDisplayWindow3D window3("ICP-3D demo: ICP-ALIGNED scans",500,500);

	window.setPos(10,10);
	window2.setPos(530,10);
	window3.setPos(10,520);

	window.get3DSceneAndLock()=scene1;
	window.unlockAccess3DScene();

	window2.get3DSceneAndLock()=scene2;
	window2.unlockAccess3DScene();

	window3.get3DSceneAndLock()=scene3;
	window3.unlockAccess3DScene();


	mrpt::system::sleep(20);
	window.forceRepaint();
	window2.forceRepaint();

	window.setCameraElevationDeg(15);
	window.setCameraAzimuthDeg(90);
	window.setCameraZoom(15);

	window2.setCameraElevationDeg(15);
	window2.setCameraAzimuthDeg(90);
	window2.setCameraZoom(15);

	window3.setCameraElevationDeg(15);
	window3.setCameraAzimuthDeg(90);
	window3.setCameraZoom(15);

	cout << "Press any key to exit..." << endl;
	window.waitForKey();
}

int main()
{
	try	{
		test_icp3D();
		return 0;
	}	catch (exception &e)	{
		cout<<"Error: "<<e.what()<<'.'<<endl;
		return -1;
	}	catch (...)	{
		cout<<"Unknown Error.\n";
		return -1;
	}
}


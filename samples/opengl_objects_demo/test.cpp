/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/random.h>

#include <mrpt/opengl.h>
#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;

// ------------------------------------------------------
//				TestOpenGLObjects
// ------------------------------------------------------
void TestOpenGLObjects()
{
	mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE(10000);

	CDisplayWindow3D win("Demo of MRPT's OpenGL objects", 640, 480);

	COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();

	// Lights:
	// theScene->getViewport()->setNumberOfLights(1);
	// mrpt::opengl::CLight & light0 = theScene->getViewport()->getLight(0);
	// light0.light_ID = 0;
	// light0.setPosition(1,1,0,0);

	// Objects:
	double off_x = 0;
	const double off_y_label = 20;
	const double STEP_X = 25;

	// XY Grid
	/**
	 * Define each one of the objects in its own scope and just attach it to the
	 * scene by using insert(obj) method call.
	 */
	{
		// using mrpt smart pointers so that obj survives outside this scope.
		opengl::CGridPlaneXY::Ptr obj =
			mrpt::make_aligned_shared<opengl::CGridPlaneXY>(-7, 7, -7, 7, 0, 1);
		obj->setColor(0.7, 0.7, 0.7);
		obj->setLocation(off_x, 0, 0);
		theScene->insert(obj);

		opengl::CGridPlaneXY::Ptr obj2 =
			mrpt::make_aligned_shared<opengl::CGridPlaneXY>(-7, 7, -7, 7, 0, 1);
		obj2->setColor(0.7, 0.7, 0.7, 0.99);
		obj2->setLocation(off_x, 15, 0);
		obj2->enableAntiAliasing();
		theScene->insert(obj2);

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CGridPlaneXY");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// XZ Grid
	{
		opengl::CGridPlaneXZ::Ptr obj =
			mrpt::make_aligned_shared<opengl::CGridPlaneXZ>(-7, 7, -7, 7, 0, 1);
		obj->setColor(0.7, 0.7, 0.7);
		obj->setLocation(off_x, 0, 0);
		theScene->insert(obj);

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CGridPlaneXZ");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// Arrow
	{
		opengl::CArrow::Ptr obj = mrpt::make_aligned_shared<opengl::CArrow>(
			0, 0, 0, 3, 0, 0, 0.2f, 0.1f, 0.2f, 0, 0, 0);
		obj->setLocation(off_x, 0, 0);
		obj->setColor(1, 0, 0);
		theScene->insert(obj);

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CArrow");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// Axis
	{
		opengl::CAxis::Ptr obj = mrpt::make_aligned_shared<opengl::CAxis>(
			-6, -6, -6, 6, 6, 6, 2, 2, true);
		obj->setLocation(off_x, 0, 0);
		theScene->insert(obj);

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CAxis");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// Box
	{
		opengl::CBox::Ptr obj = mrpt::make_aligned_shared<opengl::CBox>(
			TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), true, 3.0);
		obj->setLocation(off_x, 0, 0);
		theScene->insert(obj);

		opengl::CBox::Ptr obj2 = mrpt::make_aligned_shared<opengl::CBox>(
			TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), false);
		obj2->setLocation(off_x, 4, 0);
		theScene->insert(obj2);

		opengl::CBox::Ptr obj3 = mrpt::make_aligned_shared<opengl::CBox>(
			TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), false);
		obj3->enableBoxBorder(true);
		obj3->setLineWidth(3);
		obj3->setLocation(off_x, 8, 0);
		theScene->insert(obj3);

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CBox");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// Frustum
	{
		opengl::CFrustum::Ptr obj = mrpt::make_aligned_shared<opengl::CFrustum>(
			1, 5, 60, 45, 1.5f, true, false);
		obj->setLocation(off_x, 0, 0);
		theScene->insert(obj);

		opengl::CFrustum::Ptr obj2 =
			mrpt::make_aligned_shared<opengl::CFrustum>(
				1, 5, 60, 45, 2.5f, true, true);
		obj2->setLocation(off_x, 6, 0);
		theScene->insert(obj2);

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CFrustum");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// Cylinder
	{
		opengl::CCylinder::Ptr obj =
			mrpt::make_aligned_shared<opengl::CCylinder>(2, 2, 4, 20, 10);
		obj->setLocation(off_x, 0, 0);
		obj->setColor(0, 0, 0.8);
		theScene->insert(obj);

		opengl::CCylinder::Ptr obj2 =
			mrpt::make_aligned_shared<opengl::CCylinder>(2, 1, 4, 20, 10);
		obj2->setLocation(off_x, 6, 0);
		obj2->setColor(0, 0, 0.8);
		theScene->insert(obj2);

		opengl::CCylinder::Ptr obj3 =
			mrpt::make_aligned_shared<opengl::CCylinder>(2, 0, 4, 20, 10);
		obj3->setLocation(off_x, -6, 0);
		obj3->setColor(0, 0, 0.8);
		theScene->insert(obj3);

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CCylinder");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CDisk
	{
		{
			opengl::CDisk::Ptr obj =
				mrpt::make_aligned_shared<opengl::CDisk>(2.0f, 1.8f, 50);
			obj->setLocation(off_x, 0, 0);
			obj->setColor(0.8, 0, 0);
			theScene->insert(obj);
		}

		{
			opengl::CDisk::Ptr obj =
				mrpt::make_aligned_shared<opengl::CDisk>(2.0f, 0, 50);
			obj->setLocation(off_x, 5, 0);
			obj->setColor(0.8, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CDisk");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CEllipsoid
	{
		const double cov3d_dat[] = {0.9,  0.7,  -0.4, 0.7, 1.6,
									-0.6, -0.4, -0.6, 1.5};
		const double cov2d_dat[] = {0.9, 0.7, 0.7, 1.6};
		mrpt::math::CMatrixDouble22 cov2d(cov2d_dat);
		mrpt::math::CMatrixDouble33 cov3d(cov3d_dat);

		{
			opengl::CEllipsoid::Ptr obj =
				mrpt::make_aligned_shared<opengl::CEllipsoid>();
			obj->setCovMatrix(cov2d);
			obj->setLocation(off_x, 6, 0);
			obj->setQuantiles(2.0);
			theScene->insert(obj);
		}
		{
			opengl::CEllipsoid::Ptr obj =
				mrpt::make_aligned_shared<opengl::CEllipsoid>();
			obj->setCovMatrix(cov3d);
			obj->setQuantiles(2.0);
			obj->enableDrawSolid3D(false);
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}
		{
			opengl::CEllipsoid::Ptr obj =
				mrpt::make_aligned_shared<opengl::CEllipsoid>();
			obj->setCovMatrix(cov3d);
			obj->setQuantiles(2.0);
			obj->enableDrawSolid3D(true);
			obj->setLocation(off_x, -6, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CEllipsoid");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CEllipsoidRangeBearing2D
	{  // (range,bearing) -> (x,y)
		const double cov_params_dat[] = {0.2, 0, 0, 0.1};
		const double mean_params_dat[] = {3.0, 0.5};
		mrpt::math::CMatrixFixedNumeric<double, 2, 2> cov_params(
			cov_params_dat);
		mrpt::math::CMatrixFixedNumeric<double, 2, 1> mean_params(
			mean_params_dat);

		{
			opengl::CEllipsoidRangeBearing2D::Ptr obj =
				mrpt::make_aligned_shared<opengl::CEllipsoidRangeBearing2D>();
			obj->setCovMatrixAndMean(cov_params, mean_params);
			obj->setLocation(off_x, 6, 0);
			obj->setQuantiles(2.0f);
			// obj->setNumberOfSegments(50);
			theScene->insert(obj);

			opengl::CSetOfObjects::Ptr obj_corner =
				opengl::stock_objects::CornerXYSimple(1, 3);
			obj_corner->setLocation(off_x, 6, 0);
			theScene->insert(obj_corner);
		}
	}
	{  // (range,bearing) -> (x,y)
		const double cov_params_dat[] = {0.2, 0.09, 0.09, 0.1};
		const double mean_params_dat[] = {5.0, -0.5};
		mrpt::math::CMatrixFixedNumeric<double, 2, 2> cov_params(
			cov_params_dat);
		mrpt::math::CMatrixFixedNumeric<double, 2, 1> mean_params(
			mean_params_dat);

		{
			opengl::CEllipsoidRangeBearing2D::Ptr obj =
				mrpt::make_aligned_shared<opengl::CEllipsoidRangeBearing2D>();
			obj->setCovMatrixAndMean(cov_params, mean_params);
			obj->setLocation(off_x, 0, 0);
			obj->setQuantiles(2.0f);
			// obj->setNumberOfSegments(50);
			theScene->insert(obj);

			opengl::CSetOfObjects::Ptr obj_corner =
				opengl::stock_objects::CornerXYSimple(1, 3);
			obj_corner->setLocation(off_x, 0, 0);
			theScene->insert(obj_corner);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"CEllipsoidRangeBearing2D");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CEllipsoidInverseDepth2D
	{  // (inv_range,yaw) -> (x,y)
		// Formula from our book (ch8) for confidence intervals of 3sigmas:
		const double max_dist = 1e4;
		const double min_dist = 1;
		const double rho_mean = 0.5 * (1. / min_dist + 1. / max_dist);
		const double rho_std = (1. / 6.) * (1. / min_dist - 1. / max_dist);

		const double cov_params_dat[] = {square(rho_std), 0, 0,
										 square(DEG2RAD(2))};
		const double mean_params_dat[] = {rho_mean, DEG2RAD(70)};
		mrpt::math::CMatrixFixedNumeric<double, 2, 2> cov_params(
			cov_params_dat);
		mrpt::math::CMatrixFixedNumeric<double, 2, 1> mean_params(
			mean_params_dat);

		{
			opengl::CEllipsoidInverseDepth2D::Ptr obj =
				mrpt::make_aligned_shared<opengl::CEllipsoidInverseDepth2D>();
			obj->setCovMatrixAndMean(cov_params, mean_params);
			obj->setLocation(off_x, 6, 0);
			obj->setQuantiles(3.f);
			obj->setNumberOfSegments(100);
			theScene->insert(obj);

			opengl::CSetOfObjects::Ptr obj_corner =
				opengl::stock_objects::CornerXYSimple(1, 3);
			obj_corner->setLocation(off_x, 6, 0);
			theScene->insert(obj_corner);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"CEllipsoidInverseDepth2D");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CEllipsoidInverseDepth3D
	{  // (inv_range,yaw,pitch) -> (x,y,z)
		// Formula from our book (ch8) for confidence intervals of 3sigmas:
		const double max_dist = 1e2;
		const double min_dist = 1;
		const double rho_mean = 0.5 * (1. / min_dist + 1. / max_dist);
		const double rho_std = (1. / 6.) * (1. / min_dist - 1. / max_dist);

		const double cov_params_dat[] = {square(rho_std),	0, 0, 0,
										 square(DEG2RAD(2)), 0, 0, 0,
										 square(DEG2RAD(2))};
		const double mean_params_dat[] = {rho_mean, DEG2RAD(30), DEG2RAD(-45)};
		mrpt::math::CMatrixFixedNumeric<double, 3, 3> cov_params(
			cov_params_dat);
		mrpt::math::CMatrixFixedNumeric<double, 3, 1> mean_params(
			mean_params_dat);

		{
			opengl::CEllipsoidInverseDepth3D::Ptr obj =
				mrpt::make_aligned_shared<opengl::CEllipsoidInverseDepth3D>();
			obj->setCovMatrixAndMean(cov_params, mean_params);
			obj->setLocation(off_x, 0, 0);
			obj->setQuantiles(3.f);
			// obj->setNumberOfSegments(50);
			theScene->insert(obj);

			opengl::CSetOfObjects::Ptr obj_corner =
				opengl::stock_objects::CornerXYZSimple(1, 3);
			obj_corner->setLocation(off_x, 0, 0);
			theScene->insert(obj_corner);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"CEllipsoidInverseDepth3D");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CMesh
	{
		opengl::CMesh::Ptr obj = mrpt::make_aligned_shared<opengl::CMesh>();
		obj->setLocation(off_x, 0, 0);

		mrpt::math::CMatrixFloat Zs(40, 40);  // Height
		//		mrpt::math::CMatrixFloat  Us,Vs; // Texture
		//		Us.resize(Zs.size());
		//		Vs.resize(Zs.size());

		for (size_t i = 0; i < Zs.rows(); i++)
			for (size_t j = 0; j < Zs.cols(); j++)
			{
				double x = i * 0.25;
				double y = j * 0.25;
				Zs(i, j) = 4 * cos((x + y) * 0.6) +
						   sin((x - 0.5) * (y + 1.2) * 0.3) * 3;
			}

		obj->setGridLimits(-10, 10, -10, 10);
		obj->setZ(Zs);
		// obj->enableWireFrame(true);
		obj->enableColorFromZ(true, mrpt::img::cmJET);
		theScene->insert(obj);

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CMesh");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CMesh3D
	{
		opengl::CMesh3D::Ptr obj = mrpt::make_aligned_shared<opengl::CMesh3D>();
		obj->enableShowEdges(false);
		obj->enableShowFaces(true);
		obj->enableShowVertices(false);
		obj->setLocation(off_x, 0, 0);

		const unsigned int rows = 200, cols = 200;
		const unsigned int num_verts = (rows + 1) * (cols + 1);
		const unsigned int num_faces = rows * cols;
		int* vert_per_face = new int[num_faces];
		int* face_verts = new int[4 * num_faces];
		float* vert_coords = new float[3 * num_verts];

		// Assign vertices to faces and set them to be Quads
		unsigned int first_ind = 0;
		for (unsigned int u = 0; u < cols; u++)
		{
			for (unsigned int v = 0; v < rows; v++)
			{
				const unsigned int face_ind = v + u * rows;
				vert_per_face[face_ind] = 4;

				face_verts[4 * face_ind] = first_ind;
				face_verts[4 * face_ind + 1] = first_ind + 1;
				face_verts[4 * face_ind + 2] = first_ind + rows + 2;
				face_verts[4 * face_ind + 3] = first_ind + rows + 1;
				first_ind++;
			}

			first_ind++;
		}

		// Create vert coords
		for (unsigned int u = 0; u <= cols; u++)
			for (unsigned int v = 0; v <= rows; v++)
			{
				const unsigned int vert_ind = v + u * (rows + 1);

				vert_coords[3 * vert_ind] = (2.f - 0.01f * u) *
											(2.f + cos(0.01f * M_PI * v)) *
											cos(0.01f * M_PI * u);
				vert_coords[3 * vert_ind + 1] = (2.f - 0.01f * u) *
												(2.f + cos(0.01f * M_PI * v)) *
												sin(0.01f * M_PI * u);
				vert_coords[3 * vert_ind + 2] =
					3.f * 0.01f * u + (2.f - 0.01f * u) * sin(0.01f * M_PI * v);
			}

		obj->loadMesh(
			num_verts, num_faces, vert_per_face, face_verts, vert_coords);
		theScene->insert(obj);

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CMesh3D");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CPointCloud
	{
		opengl::CPointCloud::Ptr obj =
			mrpt::make_aligned_shared<opengl::CPointCloud>();
		obj->setLocation(off_x, 0, 0);
		theScene->insert(obj);

		obj->setPointSize(3.0);
		obj->enablePointSmooth();
		obj->enableColorFromY();

		for (int i = 0; i < 100000; i++)
			obj->insertPoint(
				mrpt::random::getRandomGenerator().drawUniform(-5, 5),
				mrpt::random::getRandomGenerator().drawUniform(-5, 5),
				mrpt::random::getRandomGenerator().drawUniform(-5, 5));

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CPointCloud");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CPointCloudColoured
	{
		opengl::CPointCloudColoured::Ptr obj =
			mrpt::make_aligned_shared<opengl::CPointCloudColoured>();
		obj->setLocation(off_x, 0, 0);
		theScene->insert(obj);

		obj->setPointSize(3.0);
		obj->enablePointSmooth();

		for (int i = 0; i < 200; i++)
			obj->push_back(
				mrpt::random::getRandomGenerator().drawUniform(-5, 5),
				mrpt::random::getRandomGenerator().drawUniform(-5, 5),
				mrpt::random::getRandomGenerator().drawUniform(-5, 5),
				mrpt::random::getRandomGenerator().drawUniform(0, 1),
				mrpt::random::getRandomGenerator().drawUniform(0, 1),
				mrpt::random::getRandomGenerator().drawUniform(0, 1));

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CPointCloudColoured");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CPolyhedron
	{
		{
			opengl::CPolyhedron::Ptr obj =
				opengl::CPolyhedron::CreateCuboctahedron(1.0);
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}
		{
			opengl::CPolyhedron::Ptr obj =
				opengl::CPolyhedron::CreateDodecahedron(1.0);
			obj->setLocation(off_x, -5, 0);
			theScene->insert(obj);
		}
		{
			opengl::CPolyhedron::Ptr obj =
				opengl::CPolyhedron::CreateIcosahedron(1.0);
			obj->setLocation(off_x, 5, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CPolyhedron");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CSphere
	{
		{
			opengl::CSphere::Ptr obj =
				mrpt::make_aligned_shared<opengl::CSphere>(3.0);
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CSphere");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

#if 1
	// CText
	{
		{
			opengl::CText::Ptr obj = mrpt::make_aligned_shared<opengl::CText>(
				"This is a CText example! My size is invariant to "
				"eye-distance");
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CText");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CText3D
	{
		{
			opengl::CText3D::Ptr obj =
				mrpt::make_aligned_shared<opengl::CText3D>(
					"I'm a cool CText3D!");
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CText3D");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CColorMap
	{
		{
			opengl::CColorBar::Ptr obj =
				mrpt::make_aligned_shared<opengl::CColorBar>(
					mrpt::img::cmHOT, 0.2, 1.0, 0.0, 1.0, -50.0, 100.0,
					"%7.02f m/s");
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CColorBar");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CSetOfLines
	{
		{
			opengl::CSetOfLines::Ptr obj =
				mrpt::make_aligned_shared<opengl::CSetOfLines>();
			obj->setLocation(off_x, 0, 0);

			for (int i = 0; i < 15; i++)
				obj->appendLine(
					mrpt::random::getRandomGenerator().drawUniform(-5, 5),
					mrpt::random::getRandomGenerator().drawUniform(-5, 5),
					mrpt::random::getRandomGenerator().drawUniform(-5, 5),
					mrpt::random::getRandomGenerator().drawUniform(-5, 5),
					mrpt::random::getRandomGenerator().drawUniform(-5, 5),
					mrpt::random::getRandomGenerator().drawUniform(-5, 5));

			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CSetOfLines");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CSimpleLine
	{
		{
			opengl::CSimpleLine::Ptr obj =
				mrpt::make_aligned_shared<opengl::CSimpleLine>();
			obj->setLocation(off_x, 0, 0);

			obj->setLineCoords(
				mrpt::random::getRandomGenerator().drawUniform(-5, 5),
				mrpt::random::getRandomGenerator().drawUniform(-5, 5),
				mrpt::random::getRandomGenerator().drawUniform(-5, 5),
				mrpt::random::getRandomGenerator().drawUniform(-5, 5),
				mrpt::random::getRandomGenerator().drawUniform(-5, 5),
				mrpt::random::getRandomGenerator().drawUniform(-5, 5));

			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CSimpleLine");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CVectorField2D
	{
		{
			opengl::CVectorField2D::Ptr obj =
				mrpt::make_aligned_shared<opengl::CVectorField2D>();
			obj->setLocation(off_x, 0, 0);

			CMatrixFloat x(16, 16), y(16, 16);
			for (unsigned int i = 0; i < x.rows(); i++)
				for (unsigned int j = 0; j < x.cols(); j++)
				{
					x(i, j) = sin(0.3 * i);
					y(i, j) = cos(0.3 * i);
				}
			obj->setVectorField(x, y);
			obj->setPointColor(1, 0.3f, 0);
			obj->setVectorFieldColor(0, 0, 1);
			obj->setPointSize(3.0);
			obj->setLineWidth(2.0);
			obj->setGridCenterAndCellSize(0, 0, 1.2f, 1.2f);
			obj->adjustVectorFieldToGrid();
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CVectorField2D");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// CVectorField3D
	{
		{
			const unsigned int num = 20;
			const float scale = 0.8 * STEP_X / num;
			opengl::CVectorField3D::Ptr obj =
				mrpt::make_aligned_shared<opengl::CVectorField3D>();
			obj->setLocation(off_x, -0.5 * scale * num, 0);  //

			CMatrixFloat x(num, num), y(num, num), z(num, num);
			CMatrixFloat vx(num, num), vy(num, num), vz(num, num);
			for (unsigned int i = 0; i < x.rows(); i++)
				for (unsigned int j = 0; j < x.cols(); j++)
				{
					x(i, j) = (i - 0.5 * num) * scale;
					y(i, j) = j * scale;
					z(i, j) = 3 * sin(0.3 * i) * cos(0.3 * j);
					vx(i, j) = 0.4 * sin(0.3 * i);
					vy(i, j) = 0.8 * cos(0.3 * i);
					vz(i, j) = 0.01 * i * j;
				}
			obj->setPointCoordinates(x, y, z);
			obj->setVectorField(vx, vy, vz);
			obj->setPointColor(1, 0.3f, 0);
			obj->setVectorFieldColor(0, 0, 1);
			obj->setPointSize(3.0);
			obj->setLineWidth(2.0);
			obj->enableColorFromModule();
			obj->setMaxSpeedForColor(3.0);
			obj->setMotionFieldColormap(0, 0, 1, 1, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt =
			mrpt::make_aligned_shared<opengl::CText>("CVectorField3D");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// stock_objects::BumblebeeCamera
	{
		{
			opengl::CSetOfObjects::Ptr obj =
				opengl::stock_objects::BumblebeeCamera();
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"stock_objects::BumblebeeCamera()");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// stock_objects::CornerXYSimple
	{
		{
			opengl::CSetOfObjects::Ptr obj =
				opengl::stock_objects::CornerXYSimple(1, 3);
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"stock_objects::CornerXYSimple()");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// stock_objects::CornerXYZSimple
	{
		{
			opengl::CSetOfObjects::Ptr obj =
				opengl::stock_objects::CornerXYZSimple(1, 3);
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"stock_objects::CornerXYZSimple()");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// stock_objects::CornerXYZ
	{
		{
			opengl::CSetOfObjects::Ptr obj = opengl::stock_objects::CornerXYZ();
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"stock_objects::CornerXYZ()");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// stock_objects::RobotPioneer
	{
		{
			opengl::CSetOfObjects::Ptr obj =
				opengl::stock_objects::RobotPioneer();
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"stock_objects::RobotPioneer()");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// stock_objects::Hokuyo_URG
	{
		{
			opengl::CSetOfObjects::Ptr obj =
				opengl::stock_objects::Hokuyo_URG();
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"stock_objects::Hokuyo_URG()");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// stock_objects::Hokuyo_UTM
	{
		{
			opengl::CSetOfObjects::Ptr obj =
				opengl::stock_objects::Hokuyo_UTM();
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"stock_objects::Hokuyo_UTM()");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// stock_objects::Househam_Sprayer
	{
		{
			opengl::CSetOfObjects::Ptr obj =
				opengl::stock_objects::Househam_Sprayer();
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"stock_objects::Househam_Sprayer()");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

	// stock_objects::RobotRhodon
	{
		{
			opengl::CSetOfObjects::Ptr obj =
				opengl::stock_objects::RobotRhodon();
			obj->setLocation(off_x, 0, 0);
			theScene->insert(obj);
		}

		opengl::CText::Ptr gl_txt = mrpt::make_aligned_shared<opengl::CText>(
			"stock_objects::RobotRhodon()");
		gl_txt->setLocation(off_x, off_y_label, 0);
		theScene->insert(gl_txt);
	}
	off_x += STEP_X;

#endif

	win.setCameraZoom(150);

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();
	win.repaint();

	cout << "Close the window to end.\n";
	while (win.isOpen())
	{
		win.addTextMessage(5, 5, format("%.02fFPS", win.getRenderingFPS()));
		std::this_thread::sleep_for(2ms);
		win.repaint();
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestOpenGLObjects();

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}

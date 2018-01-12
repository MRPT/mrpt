/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "bindings.h"

/* MRPT */
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/COpenGLScene.h>

#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

#include <mrpt/math/CMatrix.h>

// #include <mrpt/img/TColor.h>

/* namespaces */
using namespace boost::python;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::serialization;
using namespace mrpt::io;

// CRenderizable
void CRenderizable_setPose(CRenderizable& self, mrpt::poses::CPose3D pose)
{
	self.setPose(pose);
}

void CRenderizable_setLocation1(CRenderizable& self, mrpt::poses::CPoint3D pose)
{
	self.setLocation(pose.asTPoint());
}

void CRenderizable_setLocation2(CRenderizable& self, float x, float y, float z)
{
	self.setLocation(x, y, z);
}

void CRenderizable_setColor1(CRenderizable& self, mrpt::img::TColorf& c)
{
	self.setColor(c);
}

void CRenderizable_setColor2(
	CRenderizable& self, double r, double g, double b, double alpha = 1.0)
{
	self.setColor(r, g, b, alpha);
}

BOOST_PYTHON_FUNCTION_OVERLOADS(
	CRenderizable_setColor2_overloads, CRenderizable_setColor2, 4, 5)
// end of CRenderizable

// COpenGLScene
void COpenGLScene_insert(
	COpenGLScene& self, const CRenderizable::Ptr& newObject,
	const std::string& viewportName = std::string("main"))
{
	self.insert(newObject, viewportName);
}

BOOST_PYTHON_FUNCTION_OVERLOADS(
	COpenGLScene_insert_overloads, COpenGLScene_insert, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	COpenGLScene_clear_overloads, clear, 0, 1)
// end of COpenGLScene

// CSetOfLines
void CSetOfLines_appendLine(
	CSetOfLines& self, float x0, float y0, float z0, float x1, float y1,
	float z1)
{
	self.appendLine(x0, y0, z0, x1, y1, z1);
}

CSetOfLines::Ptr CSetOfLines_Create()
{
	return mrpt::make_aligned_shared<CSetOfLines>();
}
// end of CSetOfLines

// CEllipsoid
CEllipsoid::Ptr CEllipsoid_Create()
{
	return mrpt::make_aligned_shared<CEllipsoid>();
}
void CEllipsoid_setFromPosePDF(CEllipsoid& self, CPose3DPDF& posePDF)
{
	CPose3D meanPose;
	CMatrixDouble66 COV;
	posePDF.getCovarianceAndMean(COV, meanPose);
	CMatrixDouble33 COV3 = COV.block(0, 0, 3, 3);
	self.setLocation(meanPose.x(), meanPose.y(), meanPose.z() + 0.001);
	self.setCovMatrix(COV3, COV3(2, 2) == 0 ? 2 : 3);
}
// end of CEllipsoid

// CGridPlaneXY
CGridPlaneXY::Ptr CGridPlaneXY_Create(
	float xMin = -10.0, float xMax = 10.0, float yMin = -10.0,
	float yMax = 10.0, float z = 0.0, float frequency = 1.0)
{
	return mrpt::make_aligned_shared<CGridPlaneXY>(
		xMin, xMax, yMin, yMax, z, frequency);
}

BOOST_PYTHON_FUNCTION_OVERLOADS(
	CGridPlaneXY_Create_overloads, CGridPlaneXY_Create, 0, 6)
// end of CGridPlaneXY

// smart pointer contents
MAKE_PTR_CTX(COpenGLScene)
MAKE_PTR_CTX(CRenderizable)
MAKE_PTR_CTX(CSetOfObjects)
MAKE_PTR_CTX(CSetOfLines)
MAKE_PTR_CTX(CEllipsoid)
MAKE_PTR_CTX(CGridPlaneXY)

// exporter
void export_opengl()
{
	// map namespace to be submodule of mrpt package
	MAKE_SUBMODULE(opengl)

	// CRenderizable
	{
		MAKE_PTR(CRenderizable)

		class_<CRenderizable, boost::noncopyable>(
			"CRenderizable",
			"The base class of 3D objects that can be directly rendered "
			"through OpenGL.",
			no_init)
			.def(
				"setPose", &CRenderizable_setPose, args("o"),
				"Set the 3D pose from a mrpt::poses::CPose3D object.")
			.def(
				"setLocation", &CRenderizable_setLocation1, args("point3D"),
				"Changes the location of the object, keeping untouched the "
				"orientation.")
			.def(
				"setLocation", &CRenderizable_setLocation2, args("x", "y", "z"),
				"Changes the location of the object, keeping untouched the "
				"orientation.")
			.def(
				"getPoseX", &CRenderizable::getPoseX,
				"Translation relative to parent coordinate origin.")
			.def(
				"getPoseY", &CRenderizable::getPoseY,
				"Translation relative to parent coordinate origin.")
			.def(
				"getPoseZ", &CRenderizable::getPoseZ,
				"Translation relative to parent coordinate origin.")
			.def(
				"getPoseYaw", &CRenderizable::getPoseYaw,
				"Rotation relative to parent coordinate origin, in "
				"**DEGREES**.")
			.def(
				"getPosePitch", &CRenderizable::getPosePitch,
				"Rotation relative to parent coordinate origin, in "
				"**DEGREES**.")
			.def(
				"getPoseRoll", &CRenderizable::getPoseRoll,
				"Rotation relative to parent coordinate origin, in "
				"**DEGREES**.")
			.def(
				"setColor", &CRenderizable_setColor1,
				"Set the color of this object.")
			.def(
				"setColor", &CRenderizable_setColor2,
				CRenderizable_setColor2_overloads())  // "Set the color
			// components of this
			// object (R,G,B,Alpha, in
			// the range 0-1).")
			;
	}

	// CGridPlaneXY
	{
		MAKE_PTR(CGridPlaneXY)

		class_<CGridPlaneXY, boost::noncopyable, bases<CRenderizable>>(
			"CGridPlaneXY", "A grid of lines over the XY plane.", no_init)
			.def(
				"Create", &CGridPlaneXY_Create, CGridPlaneXY_Create_overloads())
			.staticmethod("Create");
	}

	// CSetOfObjects
	{
		MAKE_PTR(CSetOfObjects)

		class_<CSetOfObjects, boost::noncopyable, bases<CRenderizable>>(
			"CSetOfObjects",
			"A set of objects, which are referenced to the coordinates "
			"framework established in this object.",
			no_init) MAKE_CREATE(CSetOfObjects);
	}

	// CSetOfLines
	{
		MAKE_PTR(CSetOfLines)

		class_<CSetOfLines, boost::noncopyable, bases<CRenderizable>>(
			"CSetOfLines",
			"A set of independent lines (or segments), one line with its own "
			"start and end positions (X,Y,Z).",
			no_init)
			.def(
				"Create", &CSetOfLines_Create,
				"Create smart pointer from class.")
			.staticmethod("Create")
			.def(
				"appendLine", &CSetOfLines_appendLine,
				args("x0", "y0", "z0", "x1", "y1", "z1"),
				"Appends a line to the set, given the coordinates of its "
				"bounds.");
	}

	// CEllipsoid
	{
		MAKE_PTR(CEllipsoid)

		class_<CEllipsoid, boost::noncopyable, bases<CRenderizable>>(
			"CEllipsoid",
			"A 2D ellipse or 3D ellipsoid, depending on the size of the m_cov "
			"matrix (2x2 or 3x3).",
			no_init)
			.def(
				"Create", &CEllipsoid_Create,
				"Create smart pointer from class.")
			.staticmethod("Create")
			.def("setFromPosePDF", CEllipsoid_setFromPosePDF);
	}

	// COpenGLScene
	{
		MAKE_PTR(COpenGLScene)

		class_<COpenGLScene>(
			"COpenGLScene",
			"This class allows the user to create, load, save, and render 3D "
			"scenes using OpenGL primitives.",
			init<>("Constructor."))
			.def(
				"insert", &COpenGLScene_insert,
				COpenGLScene_insert_overloads())  //, "Insert a new object into
			// the scene, in the given
			// viewport (by default, into
			// the \"main\" viewport).")
			.def(
				"clear", &COpenGLScene::clear,
				COpenGLScene_clear_overloads())  //, "Clear the list of objects
			// and viewports in the scene,
			// deleting objects' memory, and
			// leaving just the default
			// viewport with the default
			// values.")
			MAKE_CREATE(COpenGLScene);
	}
}

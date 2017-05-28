/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

namespace mrpt {
namespace opengl {
#define FWRD_DECL(_CLASS) class _CLASS; struct _CLASS##Ptr

	FWRD_DECL(C3DSScene);
	FWRD_DECL(CArrow);
	FWRD_DECL(CAssimpModel);
	FWRD_DECL(CAxis);
	FWRD_DECL(CBox);
	FWRD_DECL(CCamera);
	FWRD_DECL(CColorBar);
	FWRD_DECL(CCylinder);
	FWRD_DECL(CDisk);
	FWRD_DECL(CEllipsoid);
	FWRD_DECL(CEllipsoidInverseDepth2D);
	FWRD_DECL(CEllipsoidInverseDepth3D);
	FWRD_DECL(CEllipsoidRangeBearing2D);
	FWRD_DECL(CFrustum);
	FWRD_DECL(CGeneralizedCylinder);
	FWRD_DECL(CGridPlaneXY);
	FWRD_DECL(CGridPlaneXZ);
	FWRD_DECL(CMesh);
	FWRD_DECL(CMesh3D);
	FWRD_DECL(CMeshFast);
	FWRD_DECL(COctoMapVoxels);
	FWRD_DECL(COpenGLScene);
	FWRD_DECL(COpenGLViewport);
	FWRD_DECL(CPointCloud);
	FWRD_DECL(CPointCloudColoured);
	FWRD_DECL(CPolyhedron);
	FWRD_DECL(CSetOfLines);
	FWRD_DECL(CSetOfObjects);
	FWRD_DECL(CSetOfTriangles);
	FWRD_DECL(CSimpleLine);
	FWRD_DECL(CSphere);
	FWRD_DECL(CText);
	FWRD_DECL(CText3D);
	FWRD_DECL(CTexturedPlane);
	FWRD_DECL(CVectorField2D);
	FWRD_DECL(CVectorField3D);

#undef FWRD_DECL
} // end namespace
} // End of namespace

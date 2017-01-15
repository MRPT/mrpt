/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/opengl.h>
#include <mrpt/utils/initializer.h>

using namespace mrpt::opengl;
using namespace mrpt::utils;

MRPT_INITIALIZER(registerAllClasses_mrpt_opengl)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	// Opengl classes:
	registerClass( CLASS_ID( CRenderizable ) );
	registerClass( CLASS_ID( C3DSScene ) );
	registerClass( CLASS_ID( CAssimpModel ) );
	registerClass( CLASS_ID( CAxis ) );
	registerClass( CLASS_ID( CBox ) );
	registerClass( CLASS_ID( CFrustum ) );
	registerClass( CLASS_ID( CDisk ) );
	registerClass( CLASS_ID( CGridPlaneXY ) );
	registerClass( CLASS_ID( CMesh ) );
	registerClass( CLASS_ID( CMesh3D ) );
	registerClass( CLASS_ID( CMeshFast ) );
	registerClass( CLASS_ID( COpenGLViewport ) );
	registerClass( CLASS_ID( CPointCloud ) );
	registerClass( CLASS_ID( CPointCloudColoured ) );
	registerClass( CLASS_ID( CSetOfLines ) );
	registerClass( CLASS_ID( CSetOfTriangles ) );
	registerClass( CLASS_ID( CSphere ) );
	registerClass( CLASS_ID( CCylinder ) );
	registerClass( CLASS_ID( CGeneralizedCylinder ) );
	registerClass( CLASS_ID( CPolyhedron ) );
	registerClass( CLASS_ID( CTexturedPlane ) );
	registerClass( CLASS_ID( CArrow ) );
	registerClass( CLASS_ID( CCamera ) );
	registerClass( CLASS_ID( CEllipsoid  ) );
	registerClass( CLASS_ID( CGridPlaneXZ ) );
	registerClass( CLASS_ID( COpenGLScene ) );
	registerClass( CLASS_ID( CSetOfObjects ) );
	registerClass( CLASS_ID( CSimpleLine ) );
	registerClass( CLASS_ID( CText ) );
	registerClass( CLASS_ID( CText3D ) );
	registerClass( CLASS_ID( CEllipsoidInverseDepth2D ) );
	registerClass( CLASS_ID( CEllipsoidInverseDepth3D ) );
	registerClass( CLASS_ID( CEllipsoidRangeBearing2D ) );
	registerClass( CLASS_ID( COctoMapVoxels ) );
	registerClass( CLASS_ID( CVectorField2D ) );
	registerClass( CLASS_ID( CVectorField3D ) );

	// These ones are in the lib: mrpt-obsmaps
	//registerClass( CLASS_ID( CPlanarLaserScan ) );
	//registerClass( CLASS_ID( CAngularObservationMesh ) );
#endif
}


/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

#include <mrpt/opengl.h>

#ifndef MRPT_ENABLE_PRECOMPILED_HDRS
#	define MRPT_ALWAYS_INCLUDE_ALL_HEADERS
#	undef _mrpt_opengl_H
#	include <mrpt/opengl.h>
#endif

#include <mrpt/utils/CStartUpClassesRegister.h>

using namespace mrpt::opengl;
using namespace mrpt::utils;

void registerAllClasses_mrpt_opengl();

CStartUpClassesRegister  mrpt_opengl_class_reg(&registerAllClasses_mrpt_opengl);

/*---------------------------------------------------------------
					registerAllClasses_mrpt_opengl
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_opengl()
{
	// Opengl classes:
	registerClass( CLASS_ID( CRenderizable ) );
	registerClass( CLASS_ID( C3DSScene ) );
	registerClass( CLASS_ID( CAxis ) );
	registerClass( CLASS_ID( CBox ) );
	registerClass( CLASS_ID( CFrustum ) );
	registerClass( CLASS_ID( CDisk ) );
	registerClass( CLASS_ID( CGridPlaneXY ) );
	registerClass( CLASS_ID( CMesh ) );
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

	// These ones are in the lib: mrpt-obsmaps
	//registerClass( CLASS_ID( CPlanarLaserScan ) );
	//registerClass( CLASS_ID( CAngularObservationMesh ) );
}


/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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

	// These ones are in the lib: mrpt-obsmaps
	//registerClass( CLASS_ID( CPlanarLaserScan ) );
	//registerClass( CLASS_ID( CAngularObservationMesh ) );
}


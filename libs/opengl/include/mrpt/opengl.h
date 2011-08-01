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

#ifndef _mrpt_opengl_H
#define _mrpt_opengl_H

#include <mrpt/config.h>

// Only really include all headers if we come from a user program (anything
//  not defining mrpt_*_EXPORTS) or MRPT is being built with precompiled headers.
#if !defined(mrpt_opengl_EXPORTS) || MRPT_ENABLE_PRECOMPILED_HDRS || defined(MRPT_ALWAYS_INCLUDE_ALL_HEADERS)

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/COpenGLViewport.h>

#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CGridPlaneXZ.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/opengl/CSetOfTexturedTriangles.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CText3D.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/C3DSScene.h>
//#include <mrpt/opengl/CPlanarLaserScan.h>			// It's in the lib mrpt-maps now
//#include <mrpt/opengl/CAngularObservationMesh.h>	// It's in the lib mrpt-maps now
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/opengl/CGeneralizedCylinder.h>
#include <mrpt/opengl/COpenGLStandardObject.h>
#include <mrpt/opengl/CFBORender.h>

#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/pose_pdfs.h>
#include <mrpt/opengl/graph_tools.h>

#endif // end precomp.headers


#endif

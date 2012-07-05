/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
#include <mrpt/opengl/CFrustum.h>
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
#include <mrpt/opengl/CEllipsoidInverseDepth2D.h>
#include <mrpt/opengl/CEllipsoidInverseDepth3D.h>
#include <mrpt/opengl/CEllipsoidRangeBearing2D.h>

#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/pose_pdfs.h>
#include <mrpt/opengl/graph_tools.h>

#endif // end precomp.headers


#endif

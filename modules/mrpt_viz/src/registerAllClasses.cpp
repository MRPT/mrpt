/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/initializer.h>

// My classes:
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/CAssimpModel.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CColorBar.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CEllipsoid2D.h>
#include <mrpt/viz/CEllipsoid3D.h>
#include <mrpt/viz/CEllipsoidInverseDepth2D.h>
#include <mrpt/viz/CEllipsoidInverseDepth3D.h>
#include <mrpt/viz/CEllipsoidRangeBearing2D.h>
#include <mrpt/viz/CFrustum.h>
#include <mrpt/viz/CGeneralizedEllipsoidTemplate.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CGridPlaneXZ.h>
#include <mrpt/viz/CMesh.h>
#include <mrpt/viz/CMesh3D.h>
#include <mrpt/viz/CMeshFast.h>
#include <mrpt/viz/COctoMapVoxels.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CPolyhedron.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSetOfTexturedTriangles.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CSimpleLine.h>
#include <mrpt/viz/CSkyBox.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/CText3D.h>
#include <mrpt/viz/CTextMessageCapable.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/CUBE_TEXTURE_FACE.h>
#include <mrpt/viz/CVectorField2D.h>
#include <mrpt/viz/CVectorField3D.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/TLightParameters.h>
#include <mrpt/viz/TTriangle.h>
#include <mrpt/viz/Viewport.h>
#include <mrpt/viz/Visualizable.h>
#include <mrpt/viz/registerAllClasses.h>
#include <mrpt/viz/stock_objects.h>

// deps:
#include <mrpt/img/registerAllClasses.h>
#include <mrpt/math/registerAllClasses.h>
#include <mrpt/poses/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_viz)
{
  using namespace mrpt::viz;

#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)

// Register the actual new mrpt 3.0 mrpt::viz class, plus the legacy 2.x versions for backwards
// compatibility:
#define DO_REGISTER(CLASS_NAME)        \
  registerClass(CLASS_ID(CLASS_NAME)); \
  registerClassCustomName("mrpt::opengl::" #CLASS_NAME, CLASS_ID(CLASS_NAME));

  // mrpt::viz classes:
  DO_REGISTER(Scene);
  DO_REGISTER(Viewport);
  DO_REGISTER(CArrow);
  DO_REGISTER(CAssimpModel);
  DO_REGISTER(CAxis);
  DO_REGISTER(CBox);
  DO_REGISTER(CCamera);
  DO_REGISTER(CColorBar);
  DO_REGISTER(CCylinder);
  DO_REGISTER(CDisk);
  DO_REGISTER(CEllipsoid2D);
  DO_REGISTER(CEllipsoid3D);
  DO_REGISTER(CEllipsoidInverseDepth2D);
  DO_REGISTER(CEllipsoidInverseDepth3D);
  DO_REGISTER(CEllipsoidRangeBearing2D);
  DO_REGISTER(CFrustum);
  DO_REGISTER(CGridPlaneXY);
  DO_REGISTER(CGridPlaneXZ);
  DO_REGISTER(CMesh);
  DO_REGISTER(CMesh3D);
  DO_REGISTER(CMeshFast);
  DO_REGISTER(COctoMapVoxels);
  DO_REGISTER(CPointCloud);
  DO_REGISTER(CPointCloudColoured);
  DO_REGISTER(CPolyhedron);
  DO_REGISTER(CSetOfLines);
  DO_REGISTER(CSetOfObjects);
  DO_REGISTER(CSetOfTriangles);
  DO_REGISTER(CSetOfTexturedTriangles);
  DO_REGISTER(CSkyBox);
  DO_REGISTER(CSimpleLine);
  DO_REGISTER(CSphere);
  DO_REGISTER(CText);
  DO_REGISTER(CText3D);
  DO_REGISTER(CTexturedPlane);
  DO_REGISTER(CVectorField2D);
  DO_REGISTER(CVectorField3D);

  // backwards compatibility de-serialization:
  registerClassCustomName("mrpt::opengl::COpenGLScene", CLASS_ID(Scene));
  registerClassCustomName("mrpt::opengl::COpenGLViewport", CLASS_ID(Viewport));

// These ones are in the lib: mrpt-obsmaps
// registerClass( CLASS_ID( CPlanarLaserScan ) );
// registerClass( CLASS_ID( CAngularObservationMesh ) );
#endif
}

void mrpt::viz::registerAllClasses_mrpt_viz()
{
  ::registerAllClasses_mrpt_viz();
  // deps:
  mrpt::img::registerAllClasses_mrpt_img();
  mrpt::poses::registerAllClasses_mrpt_poses();
}

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
  // Opengl classes:
  registerClass(CLASS_ID(Scene));
  registerClass(CLASS_ID(Viewport));
  registerClass(CLASS_ID(CArrow));
  registerClass(CLASS_ID(CAssimpModel));
  registerClass(CLASS_ID(CAxis));
  registerClass(CLASS_ID(CBox));
  registerClass(CLASS_ID(CCamera));
  registerClass(CLASS_ID(CColorBar));
  registerClass(CLASS_ID(CCylinder));
  registerClass(CLASS_ID(CDisk));
  registerClass(CLASS_ID(CEllipsoid2D));
  registerClass(CLASS_ID(CEllipsoid3D));
  registerClass(CLASS_ID(CEllipsoidInverseDepth2D));
  registerClass(CLASS_ID(CEllipsoidInverseDepth3D));
  registerClass(CLASS_ID(CEllipsoidRangeBearing2D));
  registerClass(CLASS_ID(CFrustum));
  registerClass(CLASS_ID(CGridPlaneXY));
  registerClass(CLASS_ID(CGridPlaneXZ));
  registerClass(CLASS_ID(CMesh));
  registerClass(CLASS_ID(CMesh3D));
  registerClass(CLASS_ID(CMeshFast));
  registerClass(CLASS_ID(COctoMapVoxels));
  registerClass(CLASS_ID(CPointCloud));
  registerClass(CLASS_ID(CPointCloudColoured));
  registerClass(CLASS_ID(CPolyhedron));
  registerClass(CLASS_ID(CSetOfLines));
  registerClass(CLASS_ID(CSetOfObjects));
  registerClass(CLASS_ID(CSetOfTriangles));
  registerClass(CLASS_ID(CSetOfTexturedTriangles));
  registerClass(CLASS_ID(CSkyBox));
  registerClass(CLASS_ID(CSimpleLine));
  registerClass(CLASS_ID(CSphere));
  registerClass(CLASS_ID(CText));
  registerClass(CLASS_ID(CText3D));
  registerClass(CLASS_ID(CTexturedPlane));
  registerClass(CLASS_ID(CVectorField2D));
  registerClass(CLASS_ID(CVectorField3D));

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

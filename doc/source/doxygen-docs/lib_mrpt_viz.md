\defgroup mrpt_viz_grp [mrpt-viz]

3-D scene graph: visual objects, scenes, viewports, cameras, and stock objects.

[TOC]

# Library mrpt-viz

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-viz-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

## Overview

`mrpt-viz` contains the **scene-graph API** for building, manipulating, and
serializing 3-D scenes. In MRPT 3.0 this API moved from the former
`mrpt::opengl` namespace to **mrpt::viz** (see \ref porting_mrpt3 for
migration details).

Key entry points:
- mrpt::viz::CVisualObject — base class for all 3-D visual objects
  (renamed from the former `CRenderizable`).
- mrpt::viz::Scene — a complete 3-D scene (renamed from `COpenGLScene`).
- mrpt::viz::Viewport — a sub-region of a scene with its own camera
  (renamed from `COpenGLViewport`).
- mrpt::viz::CCamera — camera parameters for a viewport.
- mrpt::viz::stock_objects — convenience factory functions for common
  objects (coordinate axes, robot models, etc.).

To actually **render** a scene on screen, use mrpt::gui::CDisplayWindow3D
or mrpt::gui::CDisplayWindowGUI (library mrpt-gui). For off-screen
rendering, use mrpt::opengl::CFBORender (library mrpt-opengl).

See the tutorial: \ref tutorial_3D_scenes

## Rendering primitives

Below is a list of the available rendering primitive classes.

- mrpt::viz::CArrow: ![CArrow](_static/preview_CArrow.png)
- mrpt::viz::CAssimpModel: ![CAssimpModel](_static/preview_CAssimpModel.png)
- mrpt::viz::CAxis: ![CAxis](_static/preview_CAxis.png)
- mrpt::viz::CBox: ![CBox](_static/preview_CBox.png)
- mrpt::viz::CFrustum: ![CFrustum](_static/preview_CFrustum.png)
- mrpt::viz::CCylinder: ![CCylinder](_static/preview_CCylinder.png)
- mrpt::viz::CDisk: ![CDisk](_static/preview_CDisk.png)
- mrpt::viz::CEllipsoid3D: ![CEllipsoid3D](_static/preview_CEllipsoid.png)
- mrpt::viz::CGridPlaneXY: ![CGridPlaneXY](_static/preview_CGridPlaneXY.png)
- mrpt::viz::CGridPlaneXZ: ![CGridPlaneXZ](_static/preview_CGridPlaneXZ.png)
- mrpt::viz::CMesh: ![CMesh](_static/preview_CMesh.png)
- mrpt::viz::CMesh3D: ![CMesh3D](_static/preview_CMesh3D.png)
- mrpt::viz::CMeshFast: ![CMeshFast](_static/preview_CMeshFast.png)
- mrpt::viz::CPointCloud: ![CPointCloud](_static/preview_CPointCloud.png)
- mrpt::viz::CPointCloudColoured: ![CPointCloudColoured](_static/preview_CPointCloudColoured.png)
- mrpt::viz::CPolyhedron: ![CPolyhedron](_static/preview_CPolyhedron.png)
- mrpt::viz::CSetOfLines: ![CSetOfLines](_static/preview_CSetOfLines.png)
- mrpt::viz::CSphere: ![CSphere](_static/preview_CSphere.png)
- mrpt::viz::CText: ![CText](_static/preview_CText.png)
- mrpt::viz::CText3D: ![CText3D](_static/preview_CText3D.png)
- mrpt::viz::CEllipsoidRangeBearing2D: ![CEllipsoidRangeBearing2D](_static/preview_CEllipsoidRangeBearing2D.png)
- mrpt::viz::CEllipsoidInverseDepth2D: ![CEllipsoidInverseDepth2D](_static/preview_CEllipsoidInverseDepth2D.png)
- mrpt::viz::CEllipsoidInverseDepth3D: ![CEllipsoidInverseDepth3D](_static/preview_CEllipsoidInverseDepth3D.png)
- mrpt::viz::COctoMapVoxels: ![COctoMapVoxels](_static/preview_COctoMapVoxels.png)
- mrpt::viz::CVectorField2D: ![CVectorField2D](_static/preview_CVectorField2D.png)
- mrpt::viz::CVectorField3D: ![CVectorField3D](_static/preview_CVectorField3D.png)
- mrpt::viz::stock_objects::BumblebeeCamera(): ![BumblebeeCamera](_static/preview_stock_objects_BumblebeeCamera.png)
- mrpt::viz::stock_objects::CornerXYSimple(): ![CornerXYSimple](_static/preview_stock_objects_CornerXYSimple.png)
- mrpt::viz::stock_objects::CornerXYZSimple(): ![CornerXYZSimple](_static/preview_stock_objects_CornerXYSimple.png)
- mrpt::viz::stock_objects::CornerXYZ(): ![CornerXYZ](_static/preview_stock_objects_CornerXYZ.png)
- mrpt::viz::stock_objects::RobotPioneer(): ![RobotPioneer](_static/preview_stock_objects_RobotPioneer.png)
- mrpt::viz::stock_objects::RobotRhodon(): ![RobotRhodon](_static/preview_stock_objects_RobotRhodon.png)
- mrpt::viz::stock_objects::Hokuyo_URG(): ![Hokuyo_URG](_static/preview_stock_objects_Hokuyo_URG.png)
- mrpt::viz::stock_objects::Hokuyo_UTM(): ![Hokuyo_UTM](_static/preview_stock_objects_Hokuyo_UTM.png)
- mrpt::viz::stock_objects::Househam_Sprayer(): ![Househam_Sprayer](_static/preview_stock_objects_Househam_Sprayer.png)

Pose PDF classes can be converted into visual objects with
mrpt::viz::CSetOfObjects::posePDF2opengl():
- mrpt::viz::CSetOfObjects::posePDF2opengl() for mrpt::poses::CPosePDFParticles: ![pose PDF particles](_static/preview_CPosePDFParticles_as_opengl.png)

Note: The following extra visual classes are provided by other libraries:
- mrpt::viz::CPlanarLaserScan (in mrpt-maps): ![CPlanarLaserScan](_static/preview_CPlanarLaserScan.png)

## Multi-light API

MRPT 3.0 supports up to 8 simultaneous lights via
mrpt::viz::TLightParameters. See \ref porting_mrpt3 section 10 for details.

# Library contents

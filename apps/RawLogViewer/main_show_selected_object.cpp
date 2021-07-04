/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CMyRedirector.h>
#include <mrpt/math/ops_matrices.h>	 // << ops
#include <mrpt/math/ops_vectors.h>	// << ops
#include <mrpt/math/wrap2pi.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/datetime.h>

#include "ParametersView3DPoints.h"
#include "xRawLogViewerMain.h"

#define MRPT_NO_WARN_BIG_HDR  // It's ok to include ALL hdrs here.
#include <mrpt/gui/WxUtils.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs.h>
#include <mrpt/obs/CObservationRotatingScan.h>	// not included in obs.h since it's in mrpt-maps
#include <mrpt/poses/CPosePDFParticles.h>

#include <iomanip>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::maps;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::serialization;
using namespace mrpt::poses;
using namespace mrpt::rtti;
using namespace std;

void recolorize3Dpc(
	const mrpt::opengl::CPointCloudColoured::Ptr& pnts,
	const ParametersView3DPoints& p)
{
	const auto bb = pnts->getBoundingBox();

	switch (p.colorizeByAxis)
	{
		case 0:
			pnts->recolorizeByCoordinate(
				p.invertColorMapping ? bb.max.x : bb.min.x,
				p.invertColorMapping ? bb.min.x : bb.max.x, 0 /* x */,
				p.colorMap);
			break;
		case 1:
			pnts->recolorizeByCoordinate(
				p.invertColorMapping ? bb.max.y : bb.min.y,
				p.invertColorMapping ? bb.min.y : bb.max.y, 1 /* y */,
				p.colorMap);
			break;
		case 2:
			pnts->recolorizeByCoordinate(
				p.invertColorMapping ? bb.max.z : bb.min.z,
				p.invertColorMapping ? bb.min.z : bb.max.z, 2 /* z */,
				p.colorMap);
			break;
		default: break;
	}
}

void add_common_to_viz(
	const CObservation& obs, const ParametersView3DPoints& p,
	mrpt::opengl::CSetOfObjects& out)
{
	if (p.showAxis)
	{
		const float L = p.axisLimits;
		auto gl_axis = mrpt::opengl::CAxis::Create(
			-L, -L, -L, L, L, L, p.axisTickFrequency, 2, true);
		gl_axis->setTextScale(p.axisTickTextSize);
		gl_axis->setColor_u8(0xa0, 0xa0, 0xa0, 0x80);
		out.insert(gl_axis);

		// Show axis labels such that they can be read from the IIIrd-IVth
		// quadrant:
		const float yawIncrs[3] = {180.f, -90.f, 180.f};
		for (int axis = 0; axis < 3; axis++)
		{
			float yaw_deg, pitch_deg, roll_deg;
			gl_axis->getTextLabelOrientation(
				axis, yaw_deg, pitch_deg, roll_deg);
			yaw_deg += yawIncrs[axis];
			gl_axis->setTextLabelOrientation(
				axis, yaw_deg, pitch_deg, roll_deg);
		}
	}

	if (p.drawSensorPose)
	{
		const auto glCorner =
			mrpt::opengl::stock_objects::CornerXYZSimple(p.sensorPoseScale);
		glCorner->setPose(obs.sensorPose());
		out.insert(glCorner);
	}
}

void obs3Dscan_to_viz(
	const CObservation3DRangeScan::Ptr& obs, const ParametersView3DPoints& p,
	mrpt::opengl::CSetOfObjects& out)
{
	out.clear();

	// Generate/load 3D points
	// ----------------------
	mrpt::maps::CPointsMap::Ptr pointMap;
	mrpt::maps::CColouredPointsMap::Ptr pointMapCol;
	mrpt::obs::T3DPointsProjectionParams pp;
	pp.takeIntoAccountSensorPoseOnRobot = true;

	// Color from intensity image?
	if (p.colorFromRGBimage && obs->hasRangeImage && obs->hasIntensityImage)
	{
		pointMapCol = mrpt::maps::CColouredPointsMap::Create();
		pointMapCol->colorScheme.scheme =
			CColouredPointsMap::cmFromIntensityImage;

		obs->unprojectInto(*pointMapCol, pp);
		pointMap = pointMapCol;
	}
	else
	{
		// Empty point set, or load from XYZ in observation:
		pointMap = mrpt::maps::CSimplePointsMap::Create();
		if (obs->hasPoints3D)
		{
			for (size_t i = 0; i < obs->points3D_x.size(); i++)
				pointMap->insertPoint(
					obs->points3D_x[i], obs->points3D_y[i], obs->points3D_z[i]);
		}
		else if (obs->hasRangeImage)
		{
			obs->unprojectInto(*pointMap, pp);
		}
	}

	add_common_to_viz(*obs, p, out);

	auto gl_pnts = mrpt::opengl::CPointCloudColoured::Create();
	// Load as RGB or grayscale points:
	if (pointMapCol) gl_pnts->loadFromPointsMap(pointMapCol.get());
	else
	{
		gl_pnts->loadFromPointsMap(pointMap.get());
		recolorize3Dpc(gl_pnts, p);
	}

	// No need to further transform 3D points
	gl_pnts->setPose(mrpt::poses::CPose3D());
	gl_pnts->setPointSize(p.pointSize);

	out.insert(gl_pnts);
}

void obsVelodyne_to_viz(
	const CObservationVelodyneScan::Ptr& obs, const ParametersView3DPoints& p,
	mrpt::opengl::CSetOfObjects& out)
{
	out.clear();

	add_common_to_viz(*obs, p, out);

	auto pnts = mrpt::opengl::CPointCloudColoured::Create();
	out.insert(pnts);

	CColouredPointsMap pntsMap;
	pntsMap.loadFromVelodyneScan(*obs);
	pnts->loadFromPointsMap(&pntsMap);
	pnts->setPointSize(p.pointSize);

	if (!p.colorFromRGBimage) recolorize3Dpc(pnts, p);
}

void obs2Dscan_to_viz(
	const CObservation2DRangeScan::Ptr& obs, const ParametersView3DPoints& p,
	mrpt::opengl::CSetOfObjects& out)
{
	out.clear();

	add_common_to_viz(*obs, p, out);

	auto pnts = mrpt::opengl::CPlanarLaserScan::Create();
	out.insert(pnts);

	pnts->setScan(*obs);
	pnts->setPointSize(p.pointSize);
	pnts->enableSurface(p.showSurfaceIn2Dscans);
	pnts->enablePoints(p.showPointsIn2Dscans);

	pnts->setSurfaceColor(
		mrpt::u8tof(p.surface2DscansColor.R),
		mrpt::u8tof(p.surface2DscansColor.G),
		mrpt::u8tof(p.surface2DscansColor.B),
		mrpt::u8tof(p.surface2DscansColor.A));
	pnts->setPointsColor(
		mrpt::u8tof(p.points2DscansColor.R),
		mrpt::u8tof(p.points2DscansColor.G),
		mrpt::u8tof(p.points2DscansColor.B),
		mrpt::u8tof(p.points2DscansColor.A));
}

// Update selected item display:
void xRawLogViewerFrame::SelectObjectInTreeView(
	const CSerializable::Ptr& sel_obj)
{
	WX_START_TRY

	if (!sel_obj)
	{
		Notebook1->ChangeSelection(0);

		memo->Freeze();	 // Freeze the window to prevent scrollbar jumping
		memo->Clear();
		{
			CMyRedirector myRedirector(memo);

			// Show comments of the rawlog:
			string s;
			rawlog.getCommentText(s);

			if (s.empty()) { cout << "(The rawlog has no comments)" << endl; }
			else
			{
				cout << s;
			}
		}
		// Set focus on the first line:
		memo->ShowPosition(0);
		memo->Thaw();  // Allow the window to redraw
		return;
	}

	// -----------------------------------------------------------
	// 	Write to the memo:
	// 	Redirect all the console output (cout) to the "memo":
	// ------------------------------------------------------------
	memo->Clear();

	auto myRedirector = std::make_unique<CMyRedirector>(memo);

	// For contextualize the popup menu, etc...
	curSelectedObject = sel_obj;

	// Select the panel by class (cannot make
	// "wxWindow::FindWindowByName" to run right!!! :-(
	//  And update the required data:
	const TRuntimeClassId* classID = sel_obj->GetRuntimeClass();

	// Default selection:
	Notebook1->ChangeSelection(0);

	// Common data:
	if (classID->derivedFrom(CLASS_ID(CObservation)))
	{
		CObservation::Ptr obs(std::dynamic_pointer_cast<CObservation>(sel_obj));
		obs->load();
		obs->getDescriptionAsText(cout);
		curSelectedObservation =
			std::dynamic_pointer_cast<CObservation>(sel_obj);
	}
	if (classID->derivedFrom(CLASS_ID(CAction)))
	{
		CAction::Ptr act(std::dynamic_pointer_cast<CAction>(sel_obj));
		cout << act->getDescriptionAsTextValue();
	}

	// Specific data:
	if (classID == CLASS_ID(CObservation2DRangeScan))
	{
		// ----------------------------------------------------------------------
		//              CObservation2DRangeScan
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(2);
		auto obs = std::dynamic_pointer_cast<CObservation2DRangeScan>(sel_obj);

		// Additional text description: This is not within
		// getDescriptionAsTextValue() because mrpt-maps is not available within
		// mrpt-obs:
		mrpt::maps::CSimplePointsMap pts;
		pts.insertionOptions.minDistBetweenLaserPoints = .0;

		pts.loadFromRangeScan(*obs);

		cout << "2D coordinates of valid points (wrt to "
				"robot/vehicle frame, "
			 << pts.size() << " points)\n";
		cout << "pts=[";
		const auto& xs = pts.getPointsBufferRef_x();
		const auto& ys = pts.getPointsBufferRef_y();
		for (size_t i = 0; i < xs.size(); i++)
			cout << format("%7.04f %7.04f;", xs[i], ys[i]);
		cout << "]\n\n";

		// The plot:
		mrpt::maps::CSimplePointsMap dummMap;
		dummMap.insertionOptions.minDistBetweenLaserPoints = 0;
		dummMap.insertObservation(*obs);

		vector<float> Xs, Ys;
		dummMap.getAllPoints(Xs, Ys);

		lyScan2D->SetData(Xs, Ys);
		plotScan2D->Fit();	// Update the window to show the new data fitted.
	}

	if (classID == CLASS_ID(CObservationImage))
	{
		// ----------------------------------------------------------------------
		//              CObservationImage
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(3);
		auto obs = std::dynamic_pointer_cast<CObservationImage>(sel_obj);

		// Get bitmap:
		// ----------------------
		wxImage* img = mrpt::gui::MRPTImage2wxImage(obs->image);
		bmpObsImage->SetBitmap(wxBitmap(*img));
		bmpObsImage->SetSize(img->GetWidth(), img->GetHeight());
		FlexGridSizerImg->FitInside(ScrolledWindow2);
		// bmpObsImage->FitInside();
		// ScrolledWindow2->SetVirtualSize(img->GetWidth(), img->GetHeight());
		ScrolledWindow2->SetScrollRate(1, 1);

		bmpObsImage->Refresh();
		delete img;
		obs->image.unload();  // For externally-stored datasets
	}

	if (classID == CLASS_ID(CObservationStereoImages))
	{
		// ----------------------------------------------------------------------
		//              CObservationStereoImages
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(4);
		auto obs = std::dynamic_pointer_cast<CObservationStereoImages>(sel_obj);

		// Images:
		// ----------------------
		wxImage* imgLeft = mrpt::gui::MRPTImage2wxImage(obs->imageLeft);
		bmpObsStereoLeft->SetBitmap(wxBitmap(*imgLeft));
		bmpObsStereoLeft->Refresh();
		delete imgLeft;

		if (obs->hasImageRight)
		{
			wxImage* imgRight = mrpt::gui::MRPTImage2wxImage(obs->imageRight);
			bmpObsStereoRight->SetBitmap(wxBitmap(*imgRight));
			bmpObsStereoRight->Refresh();
			delete imgRight;
		}

		if (obs->hasImageDisparity)
		{
			wxImage* imgDisp =
				mrpt::gui::MRPTImage2wxImage(obs->imageDisparity);
			bmpObsStereoDisp->SetBitmap(wxBitmap(*imgDisp));
			bmpObsStereoDisp->Refresh();
			delete imgDisp;
		}
	}

	if (classID == CLASS_ID(CActionRobotMovement2D))
	{
		// ----------------------------------------------------------------------
		//              CActionRobotMovement2D
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(1);

		auto act = std::dynamic_pointer_cast<CActionRobotMovement2D>(sel_obj);

		// Plot the 2D pose samples:
		unsigned int N = 1000;
		vector<CVectorDouble> samples;
		vector<float> xs(N), ys(N), ps(N), dumm(N, 0.1f);

		// Draw a set of random (x,y,phi) samples:
		act->poseChange->drawManySamples(N, samples);

		// Pass to vectors and draw them:
		for (unsigned int i = 0; i < N; i++)
		{
			xs[i] = samples[i][0];
			ys[i] = samples[i][1];
			ps[i] = RAD2DEG(samples[i][2]);
		}

		lyAction2D_XY->SetData(xs, ys);
		lyAction2D_PHI->SetData(ps, dumm);

		plotAct2D_XY->Fit();
		plotAct2D_PHI->Fit();
	}

	if (classID == CLASS_ID(CObservationBearingRange))
	{
		// ----------------------------------------------------------------------
		//              CObservationBearingRange
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(8);
		auto obs = std::dynamic_pointer_cast<CObservationBearingRange>(sel_obj);

		// The plot:
		size_t nPts = obs->sensedData.size();
		vector<float> Xs(nPts), Ys(nPts), Zs(nPts);
		for (size_t k = 0; k < nPts; k++)
		{
			float R = obs->sensedData[k].range;
			float yaw = obs->sensedData[k].yaw;
			float pitch = obs->sensedData[k].pitch;

			CPoint3D local(
				R * cos(yaw) * cos(pitch), R * sin(yaw) * cos(pitch),
				R * sin(pitch));
			CPoint3D relRobot(obs->sensorLocationOnRobot + local);
			Xs[k] = relRobot.x();
			Ys[k] = relRobot.y();
			Zs[k] = relRobot.z();
		}
		lyRangeBearingLandmarks->SetData(Xs, Ys);
		plotRangeBearing->LockAspect();
		plotRangeBearing
			->Fit();  // Update the window to show the new data fitted.
	}

	if (classID == CLASS_ID(CObservation3DRangeScan))
	{
		// ----------------------------------------------------------------------
		//              CObservation3DRangeScan
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(9);
		auto obs = std::dynamic_pointer_cast<CObservation3DRangeScan>(sel_obj);

		obs->load();  // Make sure the 3D point cloud, etc... are all
		// loaded in memory.

		// Render options
		// --------------------------------
		const auto& p = pnViewOptions->m_params;

		auto glPts = mrpt::opengl::CSetOfObjects::Create();

		obs3Dscan_to_viz(obs, p, *glPts);

// Update 3D view ==========
#if RAWLOGVIEWER_HAS_3D
		auto openGLSceneRef = m_gl3DRangeScan->getOpenGLSceneRef();
		openGLSceneRef->clear();
		openGLSceneRef->insert(glPts);

		m_gl3DRangeScan->Refresh();
#endif

		// Update intensity image ======
		{
			CImage im;
			if (obs->hasIntensityImage) im = obs->intensityImage;
			else
				im.resize(1, 1, CH_GRAY);
			wxImage* img = mrpt::gui::MRPTImage2wxImage(im);
			if (img->IsOk()) bmp3Dobs_int->SetBitmap(wxBitmap(*img));
			bmp3Dobs_int->Refresh();
			delete img;
			obs->intensityImage.unload();  // For externally-stored datasets
		}
		// Update depth image ======
		{
			CImage auxImg;
			if (obs->hasRangeImage)
				auxImg =
					obs->rangeImage_getAsImage(mrpt::img::TColormap::cmHOT);
			else
				auxImg.resize(1, 1, CH_GRAY);

			wxImage* img = mrpt::gui::MRPTImage2wxImage(auxImg);
			if (img->IsOk()) bmp3Dobs_depth->SetBitmap(wxBitmap(*img));
			bmp3Dobs_depth->Refresh();
			delete img;
		}
		// Update confidence image ======
		{
			wxImage* img;
			if (obs->hasConfidenceImage)
				img = mrpt::gui::MRPTImage2wxImage(obs->confidenceImage);
			else
			{
				mrpt::img::CImage dumm(1, 1);
				img = mrpt::gui::MRPTImage2wxImage(dumm);
			}
			if (img->IsOk()) bmp3Dobs_conf->SetBitmap(wxBitmap(*img));
			bmp3Dobs_conf->Refresh();
			delete img;
			obs->confidenceImage.unload();	// For externally-stored datasets
		}
		obs->unload();
	}

	if (classID == CLASS_ID(CObservationVelodyneScan))
	{
		// ----------------------------------------------------------------------
		//              CObservationVelodyneScan
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(9);
		auto obs = std::dynamic_pointer_cast<CObservationVelodyneScan>(sel_obj);

		obs->generatePointCloud();
		const auto& p = pnViewOptions->m_params;

		auto glPts = mrpt::opengl::CSetOfObjects::Create();
		obsVelodyne_to_viz(obs, p, *glPts);

// Update 3D view ==========
#if RAWLOGVIEWER_HAS_3D
		auto openGLSceneRef = m_gl3DRangeScan->getOpenGLSceneRef();
		openGLSceneRef->clear();
		openGLSceneRef->insert(glPts);

		this->m_gl3DRangeScan->Refresh();

		// Free memory:
		obs->point_cloud.clear_deep();
#endif
	}

	if (classID == CLASS_ID(CObservation3DScene))
	{
		// ----------------------------------------------------------------------
		//              CObservation3DScene
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(9);
		CObservation3DScene::Ptr obs =
			std::dynamic_pointer_cast<CObservation3DScene>(sel_obj);

// Update 3D view ==========
#if RAWLOGVIEWER_HAS_3D
		auto openGLSceneRef = m_gl3DRangeScan->getOpenGLSceneRef();
		if (obs->scene) *openGLSceneRef = *obs->scene;
		else
			openGLSceneRef->clear();

		this->m_gl3DRangeScan->Refresh();
#endif
	}

	if (classID == CLASS_ID(CObservationRotatingScan))
	{
		// ----------------------------------------------------------------------
		//              CObservationRotatingScan
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(4);
		auto obs = std::dynamic_pointer_cast<CObservationRotatingScan>(sel_obj);

		// Get range image as bitmap:
		// ---------------------------
		mrpt::img::CImage img_range;
		img_range.setFromMatrix(obs->rangeImage, false);

		auto imgL =
			std::unique_ptr<wxBitmap>(mrpt::gui::MRPTImage2wxBitmap(img_range));
		bmpObsStereoLeft->SetBitmap(*imgL);
		bmpObsStereoLeft->Refresh();

		mrpt::img::CImage img_intensity;
		img_intensity.setFromMatrix(obs->intensityImage, false);

		auto imgR = std::unique_ptr<wxBitmap>(
			mrpt::gui::MRPTImage2wxBitmap(img_intensity));
		bmpObsStereoRight->SetBitmap(*imgR);
		bmpObsStereoRight->Refresh();
	}

	if (classID->derivedFrom(CLASS_ID(CObservation)))
	{
		CObservation::Ptr obs(std::dynamic_pointer_cast<CObservation>(sel_obj));
		obs->unload();
	}

	myRedirector.reset();  // ensures cout is redirected to text box
	memo->ShowPosition(0);
}
catch (CExceptionExternalImageNotFound& e)
{
	wxMessageBox(
		mrpt::exception_to_str(e), _("Error with a delayed load image"), wxOK,
		this);

	if (wxYES ==
		wxMessageBox(
			(format(
				 "The current directory for relative images is:\n%s\n\nDo "
				 "you want to set it to a different one?",
				 CImage::getImagesPathBase().c_str())
				 .c_str()),
			_("Error with delayed loading image"), wxYES_NO, this))
	{
		// Change CImage::getImagesPathBase()
		wxDirDialog dirDialog(
			this, _("Choose the base directory for relative image paths"),
			CImage::getImagesPathBase().c_str(), 0, wxDefaultPosition);
		if (dirDialog.ShowModal() == wxID_OK)
		{
			CImage::setImagesPathBase(string(dirDialog.GetPath().mb_str()));
		}
	}
}
catch (const std::exception& e)
{
	wxMessageBox(mrpt::exception_to_str(e), wxT("Exception"), wxOK, this);
}
}

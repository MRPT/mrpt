/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CMyRedirector.h>
#include <mrpt/math/ops_matrices.h>	 // << ops
#include <mrpt/math/ops_vectors.h>	// << ops
#include <mrpt/math/wrap2pi.h>
#include <mrpt/system/datetime.h>

#include "ParametersView3DPoints.h"
#include "xRawLogViewerMain.h"

#define MRPT_NO_WARN_BIG_HDR  // It's ok to include ALL hdrs here.
#include <mrpt/gui/WxUtils.h>
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

static void showImageInGLView(CMyGLCanvas& canvas, const mrpt::img::CImage& im)
{
	auto scene = canvas.getOpenGLSceneRef();
	scene->getViewport()->setImageView(im);
	canvas.Refresh();
}

// Update selected item display:
void xRawLogViewerFrame::SelectObjectInTreeView(
	const CSerializable::Ptr& sel_obj)
{
	WX_START_TRY

	edSelectedTimeInfo->SetValue("(Timestamp information of selected object)");

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
	bool textDescriptionDone = false;

	// Default selection:
	Notebook1->ChangeSelection(0);

	std::optional<mrpt::Clock::time_point> obsStamp;

	// Common data:
	if (auto obs = std::dynamic_pointer_cast<CObservation>(sel_obj); obs)
	{
		try
		{
			obs->load();
			obs->getDescriptionAsText(cout);
			textDescriptionDone = true;

			obsStamp = obs->timestamp;
		}
		catch (const mrpt::img::CExceptionExternalImageNotFound& e)
		{
			std::cout << "Error with lazy-load object:\n" << e.what() << "\n";
		}

		m_selectedObj = std::dynamic_pointer_cast<CObservation>(sel_obj);
	}
	else if (auto act = std::dynamic_pointer_cast<CAction>(sel_obj); act)
	{
		std::cout << act->getDescriptionAsTextValue();
		textDescriptionDone = true;
		obsStamp = act->timestamp;
	}

	// Handle timestamp info:
	if (obsStamp.has_value() && obsStamp.value() != INVALID_TIMESTAMP)
	{
		auto s = wxString::Format(
			"Timestamp (UTC): %s\n"
			"        (local): %s\n"
			"    (as time_t): %.09f\n",
			mrpt::system::dateTimeToString(*obsStamp).c_str(),
			mrpt::system::dateTimeLocalToString(*obsStamp).c_str(),
			mrpt::Clock::toDouble(*obsStamp));

		if (m_treeView->getFirstTimestamp() != INVALID_TIMESTAMP)
		{
			const double posInSeconds = mrpt::system::timeDifference(
				m_treeView->getFirstTimestamp(), *obsStamp);

			s += wxString::Format(
				"Since rawlog beginning: %.03f [s]\n", posInSeconds);
		}

		edSelectedTimeInfo->SetValue(s);
	}

	// Specific data:
	if (classID == CLASS_ID(CObservation2DRangeScan))
	{
		// ----------------------------------------------------------------------
		//              CObservation2DRangeScan
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(8);
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
		obs->load();
		const auto& p = pnViewOptions->m_params;
		auto glPts = mrpt::opengl::CSetOfObjects::Create();
		obs2Dscan_to_viz(obs, p, *glPts);

// Update 3D view ==========
#if RAWLOGVIEWER_HAS_3D
		auto openGLSceneRef = m_gl3DRangeScan->getOpenGLSceneRef();
		openGLSceneRef->clear();
		openGLSceneRef->insert(glPts);

		m_gl3DRangeScan->Refresh();
#endif
	}

	if (classID == CLASS_ID(CObservationImage))
	{
		// ----------------------------------------------------------------------
		//              CObservationImage
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(2);
		auto obs = std::dynamic_pointer_cast<CObservationImage>(sel_obj);

		// Get bitmap:
		// ----------------------
		bool loadOk = false;
		try
		{
			obs->load();
			loadOk = true;
		}
		catch (const std::exception& e)
		{
			std::cout << "Error with lazy-load image:\n" << e.what() << "\n";
		}

		if (loadOk)	 //
			showImageInGLView(*bmpObsImage, obs->image);
		else
			showImageInGLView(*bmpObsImage, mrpt::img::CImage());

		FlexGridSizerImg->FitInside(ScrolledWindow2);
		ScrolledWindow2->SetScrollRate(1, 1);

		bmpObsImage->Refresh();
		obs->image.unload();  // For externally-stored datasets
	}

	if (classID == CLASS_ID(CObservationStereoImages))
	{
		// ----------------------------------------------------------------------
		//              CObservationStereoImages
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(3);
		auto obs = std::dynamic_pointer_cast<CObservationStereoImages>(sel_obj);

		bool loadOk = false;
		try
		{
			obs->load();
			loadOk = true;
		}
		catch (const std::exception& e)
		{
			std::cout << "Error with lazy-load image:\n" << e.what() << "\n";
		}

		// Images:
		// ----------------------
		if (loadOk)
		{
			showImageInGLView(*bmpObsStereoLeft, obs->imageLeft);
			obs->imageLeft.unload();  // For externally-stored datasets
		}

		if (obs->hasImageRight && loadOk)
		{
			showImageInGLView(*bmpObsStereoRight, obs->imageRight);
			obs->imageRight.unload();  // For externally-stored datasets
		}

		if (obs->hasImageDisparity)
		{
			showImageInGLView(*bmpObsStereoDisp, obs->imageDisparity);

			obs->imageDisparity.unload();  // For externally-stored datasets
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
		Notebook1->ChangeSelection(7);
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
		Notebook1->ChangeSelection(8);
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

			showImageInGLView(*bmp3Dobs_int, im);

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

			showImageInGLView(*bmp3Dobs_depth, auxImg);
		}
		// Update confidence image ======
		{
			if (obs->hasConfidenceImage)
				showImageInGLView(*bmp3Dobs_conf, obs->confidenceImage);
			else
				showImageInGLView(*bmp3Dobs_conf, {});

			obs->confidenceImage.unload();	// For externally-stored datasets
		}
		obs->unload();
	}

	if (classID == CLASS_ID(CObservationVelodyneScan))
	{
		// ----------------------------------------------------------------------
		//              CObservationVelodyneScan
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(8);
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

	if (classID == CLASS_ID(CObservationPointCloud))
	{
		// ----------------------------------------------------------------------
		//              CObservationPointCloud
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(8);
		auto obs = std::dynamic_pointer_cast<CObservationPointCloud>(sel_obj);

		const auto& p = pnViewOptions->m_params;

		auto glPts = mrpt::opengl::CSetOfObjects::Create();
		obsPointCloud_to_viz(obs, p, *glPts);

// Update 3D view ==========
#if RAWLOGVIEWER_HAS_3D
		auto openGLSceneRef = m_gl3DRangeScan->getOpenGLSceneRef();
		openGLSceneRef->clear();
		openGLSceneRef->insert(glPts);

		this->m_gl3DRangeScan->Refresh();
#endif
	}

	// Generic visualizable object:
	if (auto viz =
			std::dynamic_pointer_cast<mrpt::opengl::Visualizable>(sel_obj);
		viz)
	{
		// ----------------------------------------------------------------------
		//              Generic visualizable object:
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(8);

// Update 3D view ==========
#if RAWLOGVIEWER_HAS_3D
		auto openGLSceneRef = m_gl3DRangeScan->getOpenGLSceneRef();

		openGLSceneRef->clear();
		openGLSceneRef->insert(viz->getVisualization());

		this->m_gl3DRangeScan->Refresh();
#endif
	}

	if (classID == CLASS_ID(CObservationRotatingScan))
	{
		// ----------------------------------------------------------------------
		//              CObservationRotatingScan
		// ----------------------------------------------------------------------
		Notebook1->ChangeSelection(3);
		auto obs = std::dynamic_pointer_cast<CObservationRotatingScan>(sel_obj);

		// Get range image as bitmap:
		// ---------------------------
		mrpt::img::CImage img_range;
		img_range.setFromMatrix(obs->rangeImage, false);

		showImageInGLView(*bmpObsStereoLeft, img_range);

		mrpt::img::CImage img_intensity;
		img_intensity.setFromMatrix(obs->intensityImage, false);

		showImageInGLView(*bmpObsStereoRight, img_intensity);
	}

	if (classID->derivedFrom(CLASS_ID(CObservation)))
	{
		CObservation::Ptr obs(std::dynamic_pointer_cast<CObservation>(sel_obj));
		obs->unload();
	}

	// Stringifyable Interface as a fallback:
	// ------------------------------------------
	if (auto s = dynamic_cast<const mrpt::Stringifyable*>(sel_obj.get());
		s != nullptr && !textDescriptionDone)
	{
		std::cout << "Generic object textual description from "
					 "mrpt::Stringifyable::asString():\n\n"
				  << s->asString() << std::endl;
	}

	myRedirector.reset();  // ensures cout is redirected to text box
	wxTheApp->Yield(true);	// Let the app. process messages

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
		{ CImage::setImagesPathBase(string(dirDialog.GetPath().mb_str())); }
	}
}
catch (const std::exception& e)
{
	wxMessageBox(mrpt::exception_to_str(e), wxT("Exception"), wxOK, this);
}
}

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "xRawLogViewerMain.h"

#include <mrpt/system/datetime.h>
#include <mrpt/math/ops_matrices.h> // << ops
#include <mrpt/math/ops_vectors.h> // << ops
#include <mrpt/math/wrap2pi.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CPointCloudColoured.h>

#include <mrpt/gui/CMyRedirector.h>

#define MRPT_NO_WARN_BIG_HDR // It's ok to include ALL hdrs here.
#include <mrpt/obs.h>

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/poses/CPosePDFParticles.h>

#include <iomanip>

#include <mrpt/gui/WxUtils.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;


// Update selected item display:
void xRawLogViewerFrame::SelectObjectInTreeView( const CSerializablePtr & sel_obj )
{
	WX_START_TRY

	if (sel_obj.present())
	{
		// -----------------------------------------------------------
		// 	Write to the memo:
		// 	Redirect all the console output (cout) to the "memo":
		// ------------------------------------------------------------
		memo->Clear();

		{ // scope for redirector
		CMyRedirector myRedirector( memo );

		// For contextualize the popup menu, etc...
		curSelectedObject = sel_obj;

		// Select the panel by class (cannot make "wxWindow::FindWindowByName" to run right!!! :-(
		//  And update the required data:
		const TRuntimeClassId *classID = sel_obj->GetRuntimeClass();

		// Default selection:
		Notebook1->ChangeSelection( 0 );

			// Common data:
		if ( classID->derivedFrom( CLASS_ID( CObservation ) ) )
		{
			CObservationPtr obs( sel_obj );
			obs->load();
			obs->getDescriptionAsText(cout);

			// Special cases:
			if ( IS_CLASS(sel_obj, CObservation2DRangeScan) )
			{
				CObservation2DRangeScanPtr obs_scan2d = CObservation2DRangeScanPtr(sel_obj);

				mrpt::maps::CSimplePointsMap pts;
				pts.insertionOptions.minDistBetweenLaserPoints = .0;

				pts.loadFromRangeScan(*obs_scan2d);

				cout << "2D coordinates of valid points (wrt to robot/vehicle frame, " << pts.size() << " points)\n";
				cout << "pts=[";
				const std::vector<float> & xs = pts.getPointsBufferRef_x();
				const std::vector<float> & ys = pts.getPointsBufferRef_y();
				for (size_t i=0;i<xs.size();i++) cout << format("%7.04f %7.04f;", xs[i],ys[i] );
				cout << "]\n\n";
			}

			curSelectedObservation = CObservationPtr( sel_obj );
		}
		if ( classID->derivedFrom( CLASS_ID( CAction ) ) )
		{
			CActionPtr obs( sel_obj );
			cout << "Timestamp (UTC): " << dateTimeToString(obs->timestamp) << endl;
		}

		// Specific data:
		if ( classID  == CLASS_ID(CObservation2DRangeScan) )
		{
			// ----------------------------------------------------------------------
			//              CObservation2DRangeScan
			// ----------------------------------------------------------------------
			Notebook1->ChangeSelection( 2 );
			CObservation2DRangeScanPtr obs = CObservation2DRangeScanPtr( sel_obj );

			// The plot:
			mrpt::maps::CSimplePointsMap  dummMap;
			dummMap.insertionOptions.minDistBetweenLaserPoints = 0;
			dummMap.insertObservation( obs.pointer() );

			vector<float>    Xs,Ys;
			dummMap.getAllPoints(Xs,Ys);

			lyScan2D->SetData(Xs,Ys);
			plotScan2D->Fit();      // Update the window to show the new data fitted.
		}

		if ( classID  == CLASS_ID(CObservationImage) )
		{
			// ----------------------------------------------------------------------
			//              CObservationImage
			// ----------------------------------------------------------------------
			Notebook1->ChangeSelection( 3 );
			CObservationImagePtr obs = CObservationImagePtr( sel_obj );
				
			// Get bitmap:
			// ----------------------
			wxImage *img = mrpt::gui::MRPTImage2wxImage( obs->image );
			bmpObsImage->SetBitmap( wxBitmap(*img) );
			bmpObsImage->Refresh();
			delete img;
			obs->image.unload(); // For externally-stored datasets
		}

		if ( classID  == CLASS_ID(CObservationStereoImages) )
		{
			// ----------------------------------------------------------------------
			//              CObservationStereoImages
			// ----------------------------------------------------------------------
			Notebook1->ChangeSelection( 4);
			CObservationStereoImagesPtr obs = CObservationStereoImagesPtr(sel_obj);

			// Images:
			// ----------------------
			wxImage *imgLeft = mrpt::gui::MRPTImage2wxImage( obs->imageLeft );
			bmpObsStereoLeft->SetBitmap( wxBitmap(*imgLeft) );
			bmpObsStereoLeft->Refresh();
			delete imgLeft;

			wxImage *imgRight = mrpt::gui::MRPTImage2wxImage( obs->imageRight );
			bmpObsStereoRight->SetBitmap( wxBitmap(*imgRight) );
			bmpObsStereoRight->Refresh();
			delete imgRight;

			wxImage *imgDisp = mrpt::gui::MRPTImage2wxImage( obs->imageDisparity );
			bmpObsStereoDisp->SetBitmap( wxBitmap(*imgDisp) );
			bmpObsStereoDisp->Refresh();
			delete imgDisp;
		}

		if ( classID == CLASS_ID(CActionRobotMovement2D) )
		{
			// ----------------------------------------------------------------------
			//              CActionRobotMovement2D
			// ----------------------------------------------------------------------
			Notebook1->ChangeSelection( 1 );

			CActionRobotMovement2DPtr act = CActionRobotMovement2DPtr( sel_obj );

			CPose2D			Ap;
			CMatrixDouble33 mat;
			act->poseChange->getCovarianceAndMean(mat,Ap);

			cout << "Robot Movement (as a gaussian pose change):\n";
			cout << " Mean = " << Ap << endl;

			cout << format(" Covariance:     DET=%e\n", mat.det());

			cout << format("      %e %e %e\n", mat(0,0), mat(0,1), mat(0,2) );
			cout << format("      %e %e %e\n", mat(1,0), mat(1,1), mat(1,2) );
			cout << format("      %e %e %e\n", mat(2,0), mat(2,1), mat(2,2) );

			cout << endl;

			cout << " Actual reading from the odometry increment = " << act->rawOdometryIncrementReading << endl;

			cout << format("Actual PDF class is: '%s'\n",
							act->poseChange->GetRuntimeClass()->className );

			if (act->poseChange->GetRuntimeClass()==CLASS_ID(CPosePDFParticles))
			{
				CPosePDFParticlesPtr aux = CPosePDFParticlesPtr( act->poseChange.get_ptr() );
				cout << format (" (Particle count = %u)\n", (unsigned)aux->m_particles.size() );
			}
			cout << endl;

			cout << "Estimation method: ";
			switch (act->estimationMethod)
			{
			case CActionRobotMovement2D::emOdometry:
				cout << "emOdometry\n";
				break;
			case CActionRobotMovement2D::emScan2DMatching:
				cout << "emScan2DMatching\n";
				break;
			default:
				cout <<  "(Unknown ID!)\n";
				break;
			};

			// Additional data:
			if (act->hasEncodersInfo)
			{
				cout << format(" Encoder info: deltaL=%i deltaR=%i\n", act->encoderLeftTicks, act->encoderRightTicks );
			}
			else    cout << "Encoder info: Not available!\n";

			if (act->hasVelocities)
			{
				cout << format(" Velocity info: v=%s\n", act->velocityLocal.asString().c_str());
			}
			else    cout << "Velocity info: Not available!\n";


			// Plot the 2D pose samples:
			unsigned int                    N = 1000;
			vector<CVectorDouble>       samples;
			vector<float>                    xs(N),ys(N),ps(N),dumm(N,0.1f);

			// Draw a set of random (x,y,phi) samples:
			act->poseChange->drawManySamples( N, samples );

			// Pass to vectors and draw them:
			for (unsigned int i=0;i<N;i++)
			{
				xs[i] = samples[i][0];
				ys[i] = samples[i][1];
				ps[i] = RAD2DEG(samples[i][2]);
			}

			lyAction2D_XY->SetData(xs,ys);
			lyAction2D_PHI->SetData(ps,dumm);

			plotAct2D_XY->Fit();
			plotAct2D_PHI->Fit();
		}

		if ( classID  == CLASS_ID(CObservationBearingRange) )
		{
			// ----------------------------------------------------------------------
			//              CObservationBearingRange
			// ----------------------------------------------------------------------
			Notebook1->ChangeSelection( 8 );
			CObservationBearingRangePtr obs = CObservationBearingRangePtr( sel_obj );

			// The plot:
			size_t nPts = obs->sensedData.size();
			vector<float>    Xs(nPts),Ys(nPts),Zs(nPts);
			for (size_t k=0;k<nPts;k++)
			{
				float R      = obs->sensedData[k].range;
				float yaw    = obs->sensedData[k].yaw;
				float pitch  = obs->sensedData[k].pitch;

				CPoint3D   local(
					R * cos(yaw) * cos(pitch),
					R * sin(yaw) * cos(pitch),
					R * sin(pitch) );
				CPoint3D relRobot( obs->sensorLocationOnRobot + local);
				Xs[k] = relRobot.x();
				Ys[k] = relRobot.y();
				Zs[k] = relRobot.z();
			}
			lyRangeBearingLandmarks->SetData(Xs,Ys);
			plotRangeBearing->LockAspect();
			plotRangeBearing->Fit();      // Update the window to show the new data fitted.
		}

		if ( classID  == CLASS_ID(CActionRobotMovement3D) )
		{
			// ----------------------------------------------------------------------
			//              CActionRobotMovement3D
			// ----------------------------------------------------------------------
			//Notebook1->ChangeSelection( 1 );
			CActionRobotMovement3DPtr act = CActionRobotMovement3DPtr( sel_obj );
			cout << "Robot Movement (as a gaussian pose change):\n";
			cout << act->poseChange << endl;
		}

		if ( classID  == CLASS_ID(CObservation3DRangeScan) )
		{
			// ----------------------------------------------------------------------
			//              CObservation3DRangeScan
			// ----------------------------------------------------------------------
			Notebook1->ChangeSelection( 9 );
			CObservation3DRangeScanPtr obs = CObservation3DRangeScanPtr( sel_obj );

			obs->load(); // Make sure the 3D point cloud, etc... are all loaded in memory.

			const bool generate3Donthefly = !obs->hasPoints3D && mnuItemEnable3DCamAutoGenPoints->IsChecked();
			if (generate3Donthefly) {
				mrpt::obs::T3DPointsProjectionParams pp;
				pp.takeIntoAccountSensorPoseOnRobot = false;
				obs->project3DPointsFromDepthImageInto(*obs, pp );
			}

			if (generate3Donthefly)
				cout << "NOTICE: The stored observation didn't contain 3D points, but these have been generated on-the-fly just for visualization purposes.\n"
				"(You can disable this behavior from the menu Sensors->3D depth cameras\n\n";

			// Update 3D view ==========
#if RAWLOGVIEWER_HAS_3D
			this->m_gl3DRangeScan->m_openGLScene->clear();
			//this->m_gl3DRangeScan->m_openGLScene->insert( mrpt::opengl::stock_objects::CornerXYZ() );
			this->m_gl3DRangeScan->m_openGLScene->insert( mrpt::opengl::CAxis::Create(-20,-20,-20,20,20,20,1,2,true ));

			mrpt::opengl::CPointCloudColouredPtr pnts = mrpt::opengl::CPointCloudColoured::Create();
			CColouredPointsMap  pointMap;
			pointMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;

			if (obs->hasPoints3D)
			{
				// Assign only those points above a certain threshold:
				const int confThreshold =   obs->hasConfidenceImage ? slid3DcamConf->GetValue() : 0;

				if (confThreshold==0) // This includes when there is no confidence image.
				{
					pointMap.insertionOptions.minDistBetweenLaserPoints = 0; // don't drop any point
					pointMap.insertObservation(obs.pointer());  // This transform points into vehicle-frame
					pnts->loadFromPointsMap(&pointMap);

					pnts->setPose( mrpt::poses::CPose3D() ); // No need to further transform 3D points
				}
				else
				{
					pnts->clear();

					const vector<float>  &obs_xs = obs->points3D_x;
					const vector<float>  &obs_ys = obs->points3D_y;
					const vector<float>  &obs_zs = obs->points3D_z;

					size_t i=0;

					const size_t W = obs->confidenceImage.getWidth();
					const size_t H = obs->confidenceImage.getHeight();

					ASSERT_(obs->confidenceImage.isColor()==false)
					ASSERT_(obs_xs.size() == H*W)

					for(size_t r=0;r<H;r++)
					{
						unsigned char const * ptr_lin = obs->confidenceImage.get_unsafe(0,r,0);
						for(size_t c=0;c<W;c++,  i++ )
						{
							unsigned char conf = *ptr_lin++;
							if (conf>=confThreshold)
								pnts->push_back(obs_xs[i],obs_ys[i],obs_zs[i],1,1,1);
						}
					}
					// Translate the 3D cloud since sensed points are relative to the camera, but the camera may be translated wrt the robot (our 0,0,0 here):
					pnts->setPose( obs->sensorPose );
				}

				pnts->setPointSize(4.0);

			}
			this->m_gl3DRangeScan->m_openGLScene->insert(pnts);
			this->m_gl3DRangeScan->Refresh();

			// Free memory:
			if (generate3Donthefly)
			{
				obs->hasPoints3D = false;
				obs->resizePoints3DVectors(0);
			}
#endif

			// Update intensity image ======
			{
				CImage im;
				if (obs->hasIntensityImage)
						im = obs->intensityImage;
				else im.resize(10,10,CH_GRAY, true);
				wxImage *img = mrpt::gui::MRPTImage2wxImage( im );
				if (img->IsOk())
					bmp3Dobs_int->SetBitmap( wxBitmap(*img) );
				bmp3Dobs_int->Refresh();
				delete img;
				obs->intensityImage.unload(); // For externally-stored datasets
			}
			// Update depth image ======
			{
				CImage  auxImg;
				if (obs->hasRangeImage)
				{
					// Convert to range [0,255]
					mrpt::math::CMatrix normalized_range = obs->rangeImage;
					const float max_rang = std::max(obs->maxRange, normalized_range.maximum() );
					if (max_rang>0) normalized_range *= 255./max_rang;
					auxImg.setFromMatrix(normalized_range, false /* it's in range [0,255] */);
				}
				else auxImg.resize(10,10, CH_GRAY, true );

				wxImage *img = mrpt::gui::MRPTImage2wxImage( auxImg );
				if (img->IsOk())
					bmp3Dobs_depth->SetBitmap( wxBitmap(*img) );
				bmp3Dobs_depth->Refresh();
				delete img;
			}
			// Update confidence image ======
			{
				wxImage *img;
				if (obs->hasConfidenceImage)
					img = mrpt::gui::MRPTImage2wxImage( obs->confidenceImage );
				else
				{
					mrpt::utils::CImage dumm(10,10);
					img = mrpt::gui::MRPTImage2wxImage( dumm );
				}
				if (img->IsOk())
					bmp3Dobs_conf->SetBitmap( wxBitmap(*img) );
				bmp3Dobs_conf->Refresh();
				delete img;
				obs->confidenceImage.unload(); // For externally-stored datasets
			}
			obs->unload();
		}

		if ( classID  == CLASS_ID(CObservationVelodyneScan) )
		{
			// ----------------------------------------------------------------------
			//              CObservationVelodyneScan
			// ----------------------------------------------------------------------
			Notebook1->ChangeSelection( 9 );
			CObservationVelodyneScanPtr obs = CObservationVelodyneScanPtr( sel_obj );
			
			obs->generatePointCloud();
			// Update 3D view ==========
#if RAWLOGVIEWER_HAS_3D
			this->m_gl3DRangeScan->m_openGLScene->clear();
			//this->m_gl3DRangeScan->m_openGLScene->insert( mrpt::opengl::stock_objects::CornerXYZ() );
			this->m_gl3DRangeScan->m_openGLScene->insert( mrpt::opengl::CAxis::Create(-20,-20,-20,20,20,20,1,2,true ));

			mrpt::opengl::CPointCloudColouredPtr pnts = mrpt::opengl::CPointCloudColoured::Create();

			CColouredPointsMap pntsMap;
			pntsMap.loadFromVelodyneScan(*obs);
			pnts->loadFromPointsMap(&pntsMap);
			pnts->setPointSize(4.0);

			this->m_gl3DRangeScan->m_openGLScene->insert(pnts);
			this->m_gl3DRangeScan->Refresh();

			// Free memory:
			obs->point_cloud.clear_deep();
#endif
		}

		if ( classID->derivedFrom( CLASS_ID( CObservation ) ) ) {
			CObservationPtr obs( sel_obj );
			obs->unload();
		}
		} // scope for redirector
		memo->ShowPosition(0);
	}
	else
	{
		Notebook1->ChangeSelection( 0 );

		memo->Freeze();                 // Freeze the window to prevent scrollbar jumping
		memo->Clear();
		{
			CMyRedirector myRedirector( memo );

			// Show comments of the rawlog:
			string s;
			rawlog.getCommentText( s );

			if (s.empty())
			{
				cout << "(The rawlog has no comments)" << endl;
			}
			else
			{
				cout << s;
			}
		}
		// Set focus on the first line:
		memo->ShowPosition(0);
		memo->Thaw();  // Allow the window to redraw
	}

  }
  catch( utils::CExceptionExternalImageNotFound &e )
  {
	  wxMessageBox( _U(e.what()), _("Error with a delayed load image"), wxOK, this );

	  if (wxYES==wxMessageBox(
		  _U(format( "The current directory for relative images is:\n%s\n\nDo you want to set it to a different one?", CImage::IMAGES_PATH_BASE.c_str() ).c_str()),
		_("Error with delayed loading image"), wxYES_NO, this) )
	  {
			// Change CImage::IMAGES_PATH_BASE
			wxDirDialog dirDialog(
				this,
				_("Choose the base directory for relative image paths"),
				_U(CImage::IMAGES_PATH_BASE.c_str()), 0, wxDefaultPosition );
			if (dirDialog.ShowModal()==wxID_OK)
			{
				CImage::IMAGES_PATH_BASE = string( dirDialog.GetPath().mb_str() );
			}
	  }
  }
  catch(std::exception &e)
  {
		wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Exception"), wxOK, this);
  }
}


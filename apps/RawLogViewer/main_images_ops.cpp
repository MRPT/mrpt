/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "xRawLogViewerMain.h"

#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/app.h>
#include <wx/progdlg.h>
#include <wx/imaglist.h>
#include <wx/busyinfo.h>
#include <wx/log.h>
#include <wx/textdlg.h>
#include <wx/choicdlg.h>
#include <wx/dirdlg.h>

// General global variables:
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::poses;
using namespace std;

void xRawLogViewerFrame::OnGenerateSeqImgs(wxCommandEvent& event)
{
	WX_START_TRY

	// ask for the output directory:
	wxDirDialog dirDialog(
		this, _("Choose the output directory for the images"), _("."), 0,
		wxDefaultPosition);

	if (dirDialog.ShowModal() != wxID_OK) return;
	string outDir(dirDialog.GetPath().mb_str());

	// Let the user choose the image format:
	string imgFileExtension = AskForImageFileFormat();
	if (imgFileExtension.empty()) return;

	wxBusyCursor waitCursor;
	unsigned int nEntries = (int)rawlog.size();

	wxString auxStr;
	wxProgressDialog progDia(
		wxT("Progress"), wxT("Parsing rawlog..."),
		nEntries,  // range
		this,  // parent
		wxPD_CAN_ABORT | wxPD_APP_MODAL | wxPD_SMOOTH | wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	int imgSaved = 0;
	string errorMsg;

	for (unsigned int countLoop = 0; countLoop < nEntries; countLoop++)
	{
		if (countLoop % 5 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"), countLoop);
			if (!progDia.Update(countLoop, auxStr)) break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			switch (rawlog.getType(countLoop))
			{
				case CRawlog::etSensoryFrame:
				{
					CSensoryFrame::Ptr SF = rawlog.getAsObservations(countLoop);

					for (unsigned int k = 0; k < SF->size(); k++)
					{
						if (SF->getObservationByIndex(k)->GetRuntimeClass() ==
							CLASS_ID(CObservationStereoImages))
						{
							auto obsSt = SF->getObservationByIndexAs<
								CObservationStereoImages::Ptr>(k);
							obsSt->imageLeft.saveToFile(format(
								"%s/img_stereo_%u_left_%05u.%s", outDir.c_str(),
								k, imgSaved, imgFileExtension.c_str()));

							obsSt->imageRight.saveToFile(format(
								"%s/img_stereo_%u_right_%05u.%s",
								outDir.c_str(), k, imgSaved,
								imgFileExtension.c_str()));
							imgSaved++;
						}
						if (SF->getObservationByIndex(k)->GetRuntimeClass() ==
							CLASS_ID(CObservationImage))
						{
							auto obsIm = SF->getObservationByIndexAs<
								CObservationImage::Ptr>(k);
							obsIm->image.saveToFile(format(
								"%s/img_monocular_%u_%05u.%s", outDir.c_str(),
								k, imgSaved, imgFileExtension.c_str()));
							imgSaved++;
						}
					}
				}
				break;

				case CRawlog::etObservation:
				{
					CObservation::Ptr o = rawlog.getAsObservation(countLoop);

					if (IS_CLASS(o, CObservationStereoImages))
					{
						CObservationStereoImages::Ptr obsSt =
							std::dynamic_pointer_cast<CObservationStereoImages>(
								o);
						obsSt->imageLeft.saveToFile(format(
							"%s/img_stereo_%s_left_%05u.%s", outDir.c_str(),
							obsSt->sensorLabel.c_str(), imgSaved,
							imgFileExtension.c_str()));

						obsSt->imageRight.saveToFile(format(
							"%s/img_stereo_%s_right_%05u.%s", outDir.c_str(),
							obsSt->sensorLabel.c_str(), imgSaved,
							imgFileExtension.c_str()));
						imgSaved++;
					}
					else if (IS_CLASS(o, CObservationImage))
					{
						CObservationImage::Ptr obsIm =
							std::dynamic_pointer_cast<CObservationImage>(o);
						obsIm->image.saveToFile(format(
							"%s/img_monocular_%s_%05u.%s", outDir.c_str(),
							obsIm->sensorLabel.c_str(), imgSaved,
							imgFileExtension.c_str()));
						imgSaved++;
					}
				}
				break;

				default:
					break;
			}  // end for each entry
		}
		catch (exception& e)
		{
			errorMsg = mrpt::exception_to_str(e);
			break;
		}
		catch (...)
		{
			break;
		}
	}  // end while keep loading

	progDia.Update(nEntries);

	// Set error msg:
	wxMessageBox(
		(format("Images saved: %i", imgSaved).c_str()), _("Done"), wxOK, this);

	WX_END_TRY
}

// Convert pairs of mono images to stereo:
void xRawLogViewerFrame::OnMenuMono2Stereo(wxCommandEvent& event)
{
	WX_START_TRY

	string lb_left = AskForObservationByLabel("Select the LEFT camera");
	if (lb_left.empty()) return;

	string lb_right = AskForObservationByLabel("Select the RIGHT camera");
	if (lb_right.empty()) return;

	wxString sNewLabel = wxGetTextFromUser(
		_("New stereo observation label:"), _("Stereo observations"),
		_("STEREO_CAM"), this);
	string lb_stereo = string(sNewLabel.mb_str());

	wxBusyCursor waitCursor;
	auto nEntries = (unsigned int)rawlog.size();

	wxProgressDialog progDia(
		wxT("Progress"), wxT("Parsing rawlog..."),
		nEntries,  // range
		this,  // parent
		wxPD_CAN_ABORT | wxPD_APP_MODAL | wxPD_SMOOTH | wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	for (unsigned int countLoop = 0; countLoop < nEntries; countLoop++)
	{
		if (countLoop % 20 == 0)
		{
			if (!progDia.Update(
					countLoop,
					wxString::Format(
						wxT("Parsing rawlog... %u objects"), countLoop)))
			{
				return;
			}
			wxTheApp->Yield();  // Let the app. process messages
		}

		switch (rawlog.getType(countLoop))
		{
			case CRawlog::etSensoryFrame:
			{
				CSensoryFrame::Ptr sf = rawlog.getAsObservations(countLoop);

				CObservation::Ptr obs_l =
					sf->getObservationBySensorLabel(lb_left);
				CObservation::Ptr obs_r =
					sf->getObservationBySensorLabel(lb_right);

				if (obs_l && obs_r)
				{
					CObservationImage::Ptr o_l =
						std::dynamic_pointer_cast<CObservationImage>(obs_l);
					CObservationImage::Ptr o_r =
						std::dynamic_pointer_cast<CObservationImage>(obs_r);

					CObservationStereoImages::Ptr new_obs =
						mrpt::make_aligned_shared<CObservationStereoImages>();

					new_obs->timestamp =
						mrpt::Clock::time_point(mrpt::Clock::duration(
							(o_l->timestamp.time_since_epoch().count() >> 1) +
							(o_r->timestamp.time_since_epoch().count() >> 1)));

					new_obs->sensorLabel = lb_stereo;

					new_obs->cameraPose = CPose3DQuat(o_l->cameraPose);
					new_obs->rightCameraPose =
						CPose3DQuat(o_r->cameraPose - o_l->cameraPose);
					new_obs->imageLeft = o_l->image;
					new_obs->imageRight = o_r->image;

					// new_obs->focalLength_meters = o_l->focalLength_meters;

					// Delete old ones, add new one:
					sf->eraseByLabel(lb_left);
					sf->eraseByLabel(lb_right);

					sf->insert(new_obs);
				}
			}
			break;

			case CRawlog::etObservation:
			{
				// CObservation::Ptr o = rawlog.getAsObservation(countLoop);
				THROW_EXCEPTION(
					"Operation not implemented for observations-only rawlogs. "
					"Please convert into SensoryFrame-based first.");
			}
			break;

			default:
				break;
		}  // end for each entry

	}  // end while keep loading

	progDia.Update(nEntries);

	// Update:
	rebuildTreeView();

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuRectifyImages(wxCommandEvent& event)
{
	WX_START_TRY

	// ask for the output directory:
	/*wxDirDialog dirDialog( this, _("Choose the output directory for the
	images"),
						   _("."), 0, wxDefaultPosition );

	if (dirDialog.ShowModal()!=wxID_OK) return;
	string outDir( dirDialog.GetPath().mb_str() );

	// Let the user choose the image format:
	string imgFileExtension = AskForImageFileFormat();
	if (imgFileExtension.empty()) return;
	*/
	wxBusyCursor waitCursor;
	unsigned int nEntries = (int)rawlog.size();

	wxString auxStr;
	wxProgressDialog progDia(
		wxT("Progress"), wxT("Parsing rawlog..."),
		nEntries,  // range
		this,  // parent
		wxPD_CAN_ABORT | wxPD_APP_MODAL | wxPD_SMOOTH | wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	int N = 0;
	string errorMsg;

	for (unsigned int countLoop = 0; countLoop < nEntries; countLoop++)
	{
		if (countLoop % 5 == 0)
		{
			auxStr.sprintf(wxT("Rectifying images... %u images"), countLoop);
			if (!progDia.Update(countLoop, auxStr)) break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			switch (rawlog.getType(countLoop))
			{
				case CRawlog::etSensoryFrame:
				{
					CSensoryFrame::Ptr SF = rawlog.getAsObservations(countLoop);

					for (unsigned int k = 0; k < SF->size(); k++)
					{
						if (SF->getObservationByIndex(k)->GetRuntimeClass() ==
							CLASS_ID(CObservationImage))
						{
							auto obsIm = SF->getObservationByIndexAs<
								CObservationImage::Ptr>(k);

							if (obsIm->cameraParams.k1() != 0 ||
								obsIm->cameraParams.k2() != 0 ||
								obsIm->cameraParams.p1() != 0 ||
								obsIm->cameraParams.p2() != 0)
							{
								string p;
								obsIm->image.getExternalStorageFileAbsolutePath(
									p);

								obsIm->image.rectifyImageInPlace(
									obsIm->cameraParams);

								// Set distortion parameters to zero ->
								// indicating that the image is now rectified
								obsIm->cameraParams
									.setDistortionParamsFromValues(0, 0, 0, 0);

								// Save image to file and free memory
								if (obsIm->image.isExternallyStored())
								{
									obsIm->image.saveToFile(p);
									obsIm->image.unload();
								}  // end if
							}  // end if image is not undistorted
							N++;
						}  // end if CObservationImage
					}  // end for k
				}  // end case etSensoryFrame
				break;

				case CRawlog::etObservation:
				{
					CObservation::Ptr o = rawlog.getAsObservation(countLoop);

					if (IS_CLASS(o, CObservationImage))
					{
						CObservationImage::Ptr obsIm =
							std::dynamic_pointer_cast<CObservationImage>(o);

						if (obsIm->cameraParams.k1() != 0 ||
							obsIm->cameraParams.k2() != 0 ||
							obsIm->cameraParams.p1() != 0 ||
							obsIm->cameraParams.p2() != 0)
						{
							string p;
							obsIm->image.getExternalStorageFileAbsolutePath(p);

							obsIm->image.rectifyImageInPlace(
								obsIm->cameraParams);

							// Set distortion parameters to zero -> indicating
							// that the image is now rectified
							obsIm->cameraParams.setDistortionParamsFromValues(
								0, 0, 0, 0);

							// Save image to file and free memory
							if (obsIm->image.isExternallyStored())
							{
								obsIm->image.saveToFile(p);
								obsIm->image.unload();
							}
						}  // end if image is not undistorted
						N++;  // Increment of the number of rectified images
					}  // end if CObservationImage
				}  // end case etObservation
				break;

				default:
					break;
			}  // end for each entry
		}
		catch (exception& e)
		{
			errorMsg = mrpt::exception_to_str(e);
			break;
		}
		catch (...)
		{
			break;
		}
	}  // end while keep loading

	progDia.Update(nEntries);

	// Set error msg:
	wxMessageBox(
		(format("Images rectified: %i", N).c_str()), _("Done"), wxOK, this);

	WX_END_TRY
}

void renameExternalImageFile(CObservationImage::Ptr o)
{
	if (!o->image.isExternallyStored()) return;

	string img_file;
	o->image.getExternalStorageFileAbsolutePath(img_file);

	bool imgFileExistsNow = mrpt::system::fileExists(img_file);

	string new_img_file =
		o->sensorLabel +
		format(
			"_%.06f.%s", (double)timestampTotime_t(o->timestamp),
			mrpt::system::extractFileExtension(img_file).c_str());
	string new_img_fullpath =
		mrpt::system::extractFileDirectory(img_file) + "/" + new_img_file;

	if (imgFileExistsNow)
	{  // Rename the actual file:
		string strErr;
		if (!mrpt::system::renameFile(img_file, new_img_fullpath, &strErr))
			THROW_EXCEPTION(strErr);
	}

	// Anyway, rename its reference in the image:
	o->image.setExternalStorage(new_img_file);
}

void renameExternalStereoImageFile(CObservationStereoImages::Ptr o)
{
	if (o->imageLeft.isExternallyStored())
	{
		string img_file;
		o->imageLeft.getExternalStorageFileAbsolutePath(img_file);

		bool imgFileExistsNow = mrpt::system::fileExists(img_file);

		string new_img_file =
			o->sensorLabel +
			format(
				"_L_%.06f.%s", (double)timestampTotime_t(o->timestamp),
				mrpt::system::extractFileExtension(img_file).c_str());
		string new_img_fullpath =
			mrpt::system::extractFileDirectory(img_file) + "/" + new_img_file;

		if (imgFileExistsNow)
		{  // Rename the actual file:
			string strErr;
			if (!mrpt::system::renameFile(img_file, new_img_fullpath, &strErr))
				THROW_EXCEPTION(strErr);
		}

		// Anyway, rename its reference in the image:
		o->imageLeft.setExternalStorage(new_img_file);
	}

	if (o->imageRight.isExternallyStored())
	{
		string img_file;
		o->imageRight.getExternalStorageFileAbsolutePath(img_file);

		bool imgFileExistsNow = mrpt::system::fileExists(img_file);

		string new_img_file =
			o->sensorLabel +
			format(
				"_R_%.06f.%s", (double)timestampTotime_t(o->timestamp),
				mrpt::system::extractFileExtension(img_file).c_str());
		string new_img_fullpath =
			mrpt::system::extractFileDirectory(img_file) + "/" + new_img_file;

		if (imgFileExistsNow)
		{  // Rename the actual file:
			string strErr;
			if (!mrpt::system::renameFile(img_file, new_img_fullpath, &strErr))
				THROW_EXCEPTION(strErr);
		}

		// Anyway, rename its reference in the image:
		o->imageRight.setExternalStorage(new_img_file);
	}
}

void xRawLogViewerFrame::OnMenuRenameImageFiles(wxCommandEvent& event)
{
	WX_START_TRY

	if (wxYES !=
		wxMessageBox(
			_("All externally stored image files will be RENAMED according\nto "
			  "the format <SENSOR_LABEL>_<TIMESTAMP>.<ext>\n Are you sure?"),
			_("Confirm"), wxYES_NO, this))
		return;

	wxBusyCursor waitCursor;
	unsigned int nEntries = rawlog.size();

	wxString auxStr;
	wxProgressDialog progDia(
		wxT("Progress"), wxT("Processing rawlog..."),
		nEntries,  // range
		this,  // parent
		wxPD_CAN_ABORT | wxPD_APP_MODAL | wxPD_SMOOTH | wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	int N = 0;
	string errorMsg;

	for (unsigned int countLoop = 0; countLoop < nEntries; countLoop++)
	{
		if (countLoop % 50 == 0)
		{
			auxStr.sprintf(wxT("Renaming images... %u images"), countLoop);
			if (!progDia.Update(countLoop, auxStr)) break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			switch (rawlog.getType(countLoop))
			{
				case CRawlog::etSensoryFrame:
				{
					CSensoryFrame::Ptr SF = rawlog.getAsObservations(countLoop);

					for (unsigned int k = 0; k < SF->size(); k++)
					{
						if (IS_CLASS(
								SF->getObservationByIndex(k),
								CObservationImage))
						{
							auto obsIm = SF->getObservationByIndexAs<
								CObservationImage::Ptr>(k);
							renameExternalImageFile(obsIm);
							N++;
						}  // end if CObservationImage
						else if (IS_CLASS(
									 SF->getObservationByIndex(k),
									 CObservationStereoImages))
						{
							auto obsIm = SF->getObservationByIndexAs<
								CObservationStereoImages::Ptr>(k);
							renameExternalStereoImageFile(obsIm);
							N++;
						}  // end if CObservationImage
					}  // end for k
				}  // end case etSensoryFrame
				break;

				case CRawlog::etObservation:
				{
					CObservation::Ptr o = rawlog.getAsObservation(countLoop);

					if (IS_CLASS(o, CObservationImage))
					{
						CObservationImage::Ptr obsIm =
							std::dynamic_pointer_cast<CObservationImage>(o);
						renameExternalImageFile(obsIm);
						N++;
					}  // end if CObservationImage
					else if (IS_CLASS(o, CObservationStereoImages))
					{
						CObservationStereoImages::Ptr obsIm =
							std::dynamic_pointer_cast<CObservationStereoImages>(
								o);
						renameExternalStereoImageFile(obsIm);
						N++;
					}  // end if CObservationImage
				}  // end case etObservation
				break;

				default:
					break;
			}  // end for each entry
		}
		catch (exception& e)
		{
			errorMsg = mrpt::exception_to_str(e);
			break;
		}
		catch (...)
		{
			break;
		}
	}  // end while keep loading

	progDia.Update(nEntries);

	WX_END_TRY
}

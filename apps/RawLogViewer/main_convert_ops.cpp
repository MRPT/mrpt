/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "xRawLogViewerMain.h"

#include <wx/choicdlg.h>
#include <wx/progdlg.h>
#include <wx/app.h>
#include <wx/msgdlg.h>
#include <wx/textdlg.h>

// General global variables:
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::io;
using namespace mrpt::opengl;
using namespace mrpt::serialization;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::poses;
using namespace std;

void xRawLogViewerFrame::OnMenuCompactRawlog(wxCommandEvent& event)
{
	WX_START_TRY

	bool onlyOnePerLabel =
		(wxYES == wxMessageBox(
					  _("Keep only one observation of each label within each "
						"sensoryframe?"),
					  _("Compact rawlog"), wxYES_NO, this));

	int progress_N = static_cast<int>(rawlog.size());
	int progress_i = progress_N;

	wxProgressDialog progDia(
		wxT("Compacting rawlog"), wxT("Processing..."),
		progress_N,  // range
		this,  // parent
		wxPD_CAN_ABORT | wxPD_APP_MODAL | wxPD_SMOOTH | wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	CActionRobotMovement2D::Ptr lastAct;
	CSensoryFrame::Ptr lastSF;  //  = nullptr;

	unsigned counter_loops = 0;
	unsigned nActionsDel = 0;
	unsigned nEmptySFDel = 0;

	CRawlog::iterator it = rawlog.begin();
	for (progress_i = 0; it != rawlog.end(); progress_i--)
	{
		if (counter_loops++ % 50 == 0)
		{
			if (!progDia.Update(progress_N - progress_i)) break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		bool deleteThis = false;

		if (it.getType() == CRawlog::etActionCollection)
		{
			// Is this a part of multiple actions?
			if (lastAct)
			{
				// Remove this one and add it to the first in the series:
				CActionRobotMovement2D::Ptr act =
					std::dynamic_pointer_cast<CActionCollection>(*it)
						->getMovementEstimationByType(
							CActionRobotMovement2D::emOdometry);
				ASSERT_(act);
				lastAct->computeFromOdometry(
					lastAct->rawOdometryIncrementReading +
						act->rawOdometryIncrementReading,
					lastAct->motionModelConfiguration);

				deleteThis = true;
				nActionsDel++;
			}
			else
			{
				// This is the first one:
				lastAct = std::dynamic_pointer_cast<CActionCollection>(*it)
							  ->getMovementEstimationByType(
								  CActionRobotMovement2D::emOdometry);
				ASSERT_(lastAct);

				// Before leaving the last SF, leave only one observation for
				// each sensorLabel:
				if (onlyOnePerLabel && lastSF)
				{
					CSensoryFrame::Ptr newSF =
						mrpt::make_aligned_shared<CSensoryFrame>();
					set<string> knownLabels;

					for (auto o = lastSF->begin(); o != lastSF->end(); ++o)
					{
						if (knownLabels.find((*o)->sensorLabel) ==
							knownLabels.end())
							newSF->insert(*o);
						knownLabels.insert((*o)->sensorLabel);
					}
					*lastSF = *newSF;
				}
				// Ok, done:
				lastSF.reset();
			}
		}
		else if (it.getType() == CRawlog::etSensoryFrame)
		{
			// Is this a part of a series?
			if (lastSF)
			{
				// remove this one and accumulate in the first in the serie:
				lastSF->moveFrom(
					*std::dynamic_pointer_cast<CSensoryFrame>(*it));

				deleteThis = true;
				nEmptySFDel++;
			}
			else
			{
				// This is the first SF:
				CSensoryFrame::Ptr sf =
					std::dynamic_pointer_cast<CSensoryFrame>(*it);

				// Only take into account if not empty!
				if (sf->size())
				{
					lastSF = sf;
					ASSERT_(lastSF);
					lastAct.reset();
				}
				else
				{
					deleteThis = true;
					nEmptySFDel++;
				}
			}
		}
		else
			THROW_EXCEPTION("Unexpected class found!");

		if (deleteThis)
		{
			it = rawlog.erase(it);
			progress_i--;  // Extra count
		}
		else
			it++;
	}

	progDia.Update(progress_N);

	string str = format(
		"%u actions deleted\n%u sensory frames deleted", nActionsDel,
		nEmptySFDel);
	::wxMessageBox(str.c_str());

	rebuildTreeView();

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuLossLessDecimate(wxCommandEvent& event)
{
	WX_START_TRY

	CRawlog newRawLog;
	newRawLog.setCommentText(rawlog.getCommentText());

	wxString strDecimation = wxGetTextFromUser(
		_("The number of observations will be decimated (only 1 out of M will "
		  "be kept). Enter the decimation ratio M:"),
		_("Decimation"), _("1"));
	long DECIMATE_RATIO;
	strDecimation.ToLong(&DECIMATE_RATIO);

	ASSERT_(DECIMATE_RATIO >= 1);

	wxBusyCursor busyCursor;
	wxTheApp->Yield();  // Let the app. process messages

	size_t i, N = rawlog.size();

	// ------------------------------------------------------------------------------
	// METHOD TO BE MEMORY EFFICIENT:
	//  To free the memory of the current rawlog entries as we create the new
	//  one,
	//  then call "clearWithoutDelete" at the end.
	// ------------------------------------------------------------------------------
	CSensoryFrame::Ptr accum_sf;
	CActionRobotMovement2D::TMotionModelOptions odometryOptions;
	bool cummMovementInit = false;
	long SF_counter = 0;

	// Reset cummulative pose change:
	CPose2D accumMovement(0, 0, 0);

	// For each entry:
	for (i = 0; i < N; i++)
	{
		CSerializable::Ptr obj = rawlog.getAsGeneric(i);

		if (rawlog.getType(i) == CRawlog::etActionCollection)
		{
			// Accumulate Actions
			// ----------------------
			CActionCollection::Ptr curActs =
				std::dynamic_pointer_cast<CActionCollection>(obj);
			CActionRobotMovement2D::Ptr mov =
				curActs->getBestMovementEstimation();
			if (mov)
			{
				CPose2D incrPose = mov->poseChange->getMeanVal();

				// If we have previous observations, shift them according to
				// this new increment:
				if (accum_sf)
				{
					CPose3D inv_incrPose3D(
						CPose3D(0, 0, 0) - CPose3D(incrPose));

					for (auto& it : *accum_sf)
					{
						CPose3D tmpPose;

						it->getSensorPose(tmpPose);
						tmpPose = inv_incrPose3D + tmpPose;
						it->setSensorPose(tmpPose);
					}
				}

				// Accumulate from odometry:
				accumMovement = accumMovement + incrPose;

				// Copy the probabilistic options from the first entry we find:
				if (!cummMovementInit)
				{
					odometryOptions = mov->motionModelConfiguration;
					cummMovementInit = true;
				}
			}
		}
		else if (rawlog.getType(i) == CRawlog::etSensoryFrame)
		{
			// Decimate Observations
			// ---------------------------

			// Add observations to the accum. SF:
			if (!accum_sf)
				accum_sf = mrpt::make_aligned_shared<CSensoryFrame>();

			// Copy pointers to observations only (fast):
			accum_sf->moveFrom(*std::dynamic_pointer_cast<CSensoryFrame>(obj));

			if (++SF_counter >= DECIMATE_RATIO)
			{
				SF_counter = 0;

				// INSERT OBSERVATIONS:
				newRawLog.addObservationsMemoryReference(accum_sf);
				accum_sf.reset();

				// INSERT ACTIONS:
				CActionCollection actsCol;
				if (cummMovementInit)
				{
					CActionRobotMovement2D cummMovement;
					cummMovement.computeFromOdometry(
						accumMovement, odometryOptions);
					actsCol.insert(cummMovement);
					// Reset odometry accumulation:
					accumMovement = CPose2D(0, 0, 0);
				}
				newRawLog.addActions(actsCol);
			}
		}
		else
			THROW_EXCEPTION(
				"This operation implemented for SF-based rawlogs only.");

		// Delete object
		obj.reset();

	}  // end for i each entry

	// Clear the list only (objects already deleted)
	rawlog.clear();

	// Copy as new log:
	rawlog = newRawLog;

	rebuildTreeView();

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenCompactFILE(wxCommandEvent& event)
{
	WX_START_TRY

	THROW_EXCEPTION("Sorry, compact not implemented for files yet.");

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuLossLessDecFILE(wxCommandEvent& event)
{
	WX_START_TRY

	string filToOpen;
	if (!AskForOpenRawlog(filToOpen)) return;

	wxString strDecimation = wxGetTextFromUser(
		_("The number of observations will be decimated (only 1 out of M will "
		  "be kept). Enter the decimation ratio M:"),
		_("Decimation"), _("1"));
	long DECIMATE_RATIO;
	strDecimation.ToLong(&DECIMATE_RATIO);

	ASSERT_(DECIMATE_RATIO >= 1);

	string filToSave;
	AskForSaveRawlog(filToSave);

	CFileGZInputStream fil(filToOpen);
	CFileGZOutputStream f_out(filToSave);

	wxBusyCursor waitCursor;
	auto filSize = (unsigned int)fil.getTotalBytesCount();

	wxString auxStr;
	wxProgressDialog progDia(
		wxT("Progress"), wxT("Parsing file..."),
		filSize,  // range
		this,  // parent
		wxPD_CAN_ABORT | wxPD_APP_MODAL | wxPD_SMOOTH | wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	unsigned int countLoop = 0;
	int entryIndex = 0;
	bool keepLoading = true;
	string errorMsg;

	// ------------------------------------------------------------------------------
	// METHOD TO BE MEMORY EFFICIENT:
	//  To free the memory of the current rawlog entries as we create the new
	//  one,
	//  then call "clearWithoutDelete" at the end.
	// ------------------------------------------------------------------------------
	CSensoryFrame::Ptr accum_sf;
	CActionRobotMovement2D::TMotionModelOptions odometryOptions;
	bool cummMovementInit = false;
	long SF_counter = 0;

	// Reset cummulative pose change:
	CPose2D accumMovement(0, 0, 0);

	TTimeStamp timestamp_lastAction = INVALID_TIMESTAMP;

	while (keepLoading)
	{
		if (countLoop++ % 100 == 0)
		{
			auxStr.sprintf(wxT("Parsing file... %u objects"), entryIndex);
			if (!progDia.Update((int)fil.getPosition(), auxStr))
				keepLoading = false;
			wxTheApp->Yield();  // Let the app. process messages
		}

		CSerializable::Ptr newObj;
		try
		{
			archiveFrom(fil) >> newObj;
			entryIndex++;

			// Check type:
			if (newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
			{
				// Decimate Observations
				// ---------------------------

				// Add observations to the accum. SF:
				if (!accum_sf)
					accum_sf = mrpt::make_aligned_shared<CSensoryFrame>();

				// Copy pointers to observations only (fast):
				accum_sf->moveFrom(
					*std::dynamic_pointer_cast<CSensoryFrame>(newObj));

				if (++SF_counter >= DECIMATE_RATIO)
				{
					SF_counter = 0;

					// INSERT OBSERVATIONS:
					archiveFrom(f_out) << *accum_sf;
					accum_sf.reset();

					// INSERT ACTIONS:
					CActionCollection::Ptr actsCol =
						mrpt::make_aligned_shared<CActionCollection>();
					if (cummMovementInit)
					{
						CActionRobotMovement2D cummMovement;
						cummMovement.computeFromOdometry(
							accumMovement, odometryOptions);
						cummMovement.timestamp = timestamp_lastAction;
						actsCol->insert(cummMovement);
						// Reset odometry accumulation:
						accumMovement = CPose2D(0, 0, 0);
					}
					archiveFrom(f_out) << *actsCol;
					actsCol.reset();
				}
			}
			else if (newObj->GetRuntimeClass() == CLASS_ID(CActionCollection))
			{
				// Accumulate Actions
				// ----------------------
				CActionCollection::Ptr curActs =
					std::dynamic_pointer_cast<CActionCollection>(newObj);
				CActionRobotMovement2D::Ptr mov =
					curActs->getBestMovementEstimation();
				if (mov)
				{
					timestamp_lastAction = mov->timestamp;
					CPose2D incrPose = mov->poseChange->getMeanVal();

					// If we have previous observations, shift them according to
					// this new increment:
					if (accum_sf)
					{
						CPose3D inv_incrPose3D(
							CPose3D(0, 0, 0) - CPose3D(incrPose));

						for (auto& it : *accum_sf)
						{
							CPose3D tmpPose;

							it->getSensorPose(tmpPose);
							tmpPose = inv_incrPose3D + tmpPose;
							it->setSensorPose(tmpPose);
						}
					}

					// Accumulate from odometry:
					accumMovement = accumMovement + incrPose;

					// Copy the probabilistic options from the first entry we
					// find:
					if (!cummMovementInit)
					{
						odometryOptions = mov->motionModelConfiguration;
						cummMovementInit = true;
					}
				}
			}
			else
			{
				// Unknown class:
				THROW_EXCEPTION("Unknown class found in the file!");
			}
		}
		catch (exception& e)
		{
			errorMsg = mrpt::exception_to_str(e);
			keepLoading = false;
		}
		catch (...)
		{
			keepLoading = false;
		}
	}  // end while keep loading

	progDia.Update(filSize);

	wxTheApp->Yield();  // Let the app. process messages

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuConvertExternallyStored(wxCommandEvent& event)
{
	WX_START_TRY

	wxMessageBox(_("Select the rawlog file with embedded images."));

	string str;
	if (!AskForOpenRawlog(str)) return;

	wxMessageBox(_("Select the target file where to save the new rawlog."));
	string filToSave;
	if (!AskForSaveRawlog(filToSave)) return;

	// Create the default "/Images" directory.
	string outDir = extractFileDirectory(filToSave) + string("/") +
					extractFileName(filToSave) + string("_Images");
	if (fileExists(outDir))
	{
		wxMessageBox(
			(format(
				 "*ABORTING*: Output directory for images already exists. "
				 "Select a different output path or remove the "
				 "directory:\n%s",
				 outDir.c_str())
				 .c_str()));
		return;
	}

	createDirectory(outDir);
	if (!fileExists(outDir))
	{
		wxMessageBox(
			(format("*ABORTING*: Cannot create directory:\n%s", outDir.c_str())
				 .c_str()));
		return;
	}

	// Add the final /
	outDir += "/";

	// Let the user choose the image format:
	string imgFileExtension = AskForImageFileFormat();
	if (imgFileExtension.empty()) return;

	wxBusyCursor waitCursor;
	CFileGZInputStream fil(str);
	auto filSize = (unsigned int)fil.getTotalBytesCount();

	CFileGZOutputStream f_out(filToSave);

	wxString auxStr;
	wxProgressDialog progDia(
		wxT("Progress"), wxT("Parsing file..."),
		filSize,  // range
		this,  // parent
		wxPD_CAN_ABORT | wxPD_APP_MODAL | wxPD_SMOOTH | wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	unsigned int countLoop = 0;
	int imgSaved = 0;
	bool keepLoading = true;
	string errorMsg;

	while (keepLoading)
	{
		if (countLoop++ % 5 == 0)
		{
			auxStr.sprintf(wxT("Parsing file... %u objects"), countLoop);
			if (!progDia.Update((int)fil.getPosition(), auxStr))
				keepLoading = false;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			CSerializable::Ptr newObj;
			archiveFrom(fil) >> newObj;

			// Check type:
			if (newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
			{
				CSensoryFrame::Ptr SF(
					std::dynamic_pointer_cast<CSensoryFrame>(newObj));

				for (unsigned k = 0; k < SF->size(); k++)
				{
					if (SF->getObservationByIndex(k)->GetRuntimeClass() ==
						CLASS_ID(CObservationStereoImages))
					{
						auto obsSt = SF->getObservationByIndexAs<
							CObservationStereoImages::Ptr>(k);

						// save image to file & convert into external storage:
						string fileName = format(
							"img_stereo_%u_left_%05u.%s", k, imgSaved,
							imgFileExtension.c_str());
						obsSt->imageLeft.saveToFile(outDir + fileName);
						obsSt->imageLeft.setExternalStorage(fileName);

						imgSaved++;

						// save image to file & convert into external storage:
						fileName = format(
							"img_stereo_%u_right_%05u.%s", k, imgSaved,
							imgFileExtension.c_str());
						obsSt->imageRight.saveToFile(outDir + fileName);
						obsSt->imageRight.setExternalStorage(fileName);

						imgSaved++;
					}
					if (SF->getObservationByIndex(k)->GetRuntimeClass() ==
						CLASS_ID(CObservationImage))
					{
						auto obsIm =
							SF->getObservationByIndexAs<CObservationImage::Ptr>(
								k);

						// save image to file & convert into external storage:
						string fileName = format(
							"img_monocular_%u_%05u.%s", k, imgSaved,
							imgFileExtension.c_str());
						obsIm->image.saveToFile(outDir + fileName);
						obsIm->image.setExternalStorage(fileName);

						imgSaved++;
					}
				}
			}
			else if (
				newObj->GetRuntimeClass() == CLASS_ID(CActionCollection) ||
				newObj->GetRuntimeClass() == CLASS_ID(CPose2D))
			{
			}
			else
			{
				// Unknown class:
				THROW_EXCEPTION("Unknown class found in the file!");
			}

			// Dump to the new file:
			archiveFrom(f_out) << *newObj;

			// Free memory:
			newObj.reset();
		}
		catch (exception& e)
		{
			errorMsg = mrpt::exception_to_str(e);
			keepLoading = false;
		}
		catch (...)
		{
			keepLoading = false;
		}
	}  // end while keep loading

	progDia.Update(filSize);

	// Set error msg:
	wxMessageBox(
		(format("Images saved: %i", imgSaved).c_str()), _("Done"), wxOK, this);

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuConvertObservationOnly(wxCommandEvent& event)
{
	WX_START_TRY

	wxMessageBox(_("Select the rawlog file to convert..."));

	string str;
	if (!AskForOpenRawlog(str)) return;

	wxMessageBox(_("Select the target file where to save the new rawlog."));
	string filToSave;
	if (!AskForSaveRawlog(filToSave)) return;

	wxBusyCursor waitCursor;
	CFileGZInputStream fil(str);
	auto filSize = (unsigned int)fil.getTotalBytesCount();

	CFileGZOutputStream f_out(filToSave);

	wxString auxStr;
	wxProgressDialog progDia(
		wxT("Progress"), wxT("Parsing file..."),
		filSize,  // range
		this,  // parent
		wxPD_CAN_ABORT | wxPD_APP_MODAL | wxPD_SMOOTH | wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	unsigned int countLoop = 0;
	bool keepLoading = true;
	string errorMsg;

	CPose2D odometry_accum;

	// We'll save here all the individual observations ordered in time:
	TListTimeAndObservations time_ordered_list_observation;

	mrpt::system::TTimeStamp lastValidObsTime = INVALID_TIMESTAMP;

	while (keepLoading)
	{
		if (countLoop++ % 5 == 0)
		{
			auxStr.sprintf(wxT("Parsing file... %u objects"), countLoop);
			if (!progDia.Update((int)fil.getPosition(), auxStr))
				keepLoading = false;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			CSerializable::Ptr newObj;
			archiveFrom(fil) >> newObj;

			// Check type:
			if (newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
			{
				CSensoryFrame::Ptr SF(
					std::dynamic_pointer_cast<CSensoryFrame>(newObj));
				for (auto& it : *SF)
				{
					time_ordered_list_observation.insert(
						TTimeObservationPair(it->timestamp, it));
					lastValidObsTime = it->timestamp;
				}
			}
			else if (newObj->GetRuntimeClass() == CLASS_ID(CActionCollection))
			{
				// Replace "odometry action" with "odometry observation":
				CActionCollection::Ptr acts =
					std::dynamic_pointer_cast<CActionCollection>(newObj);
				// Get odometry:
				CActionRobotMovement2D::Ptr actOdom =
					acts->getBestMovementEstimation();
				if (actOdom)
				{
					odometry_accum =
						odometry_accum + actOdom->poseChange->getMeanVal();

					// Generate "odometry obs":
					CObservationOdometry::Ptr newO =
						mrpt::make_aligned_shared<CObservationOdometry>();
					newO->sensorLabel = "odometry";
					newO->timestamp = actOdom->timestamp != INVALID_TIMESTAMP
										  ? actOdom->timestamp
										  : lastValidObsTime;
					newO->odometry = odometry_accum;

					time_ordered_list_observation.insert(
						TTimeObservationPair(newO->timestamp, newO));
				}
			}
			else if (newObj->GetRuntimeClass()->derivedFrom(
						 CLASS_ID(CObservation)))
			{
				CObservation::Ptr o =
					std::dynamic_pointer_cast<CObservation>(newObj);
				time_ordered_list_observation.insert(
					TTimeObservationPair(o->timestamp, o));
			}

			// Dump to the new file: Only the oldest one:
			// --------------------------------------------------
			if (time_ordered_list_observation.size() > 30)
			{
				// Save a few of the oldest and continue:
				for (unsigned i = 0; i < 15; i++)
				{
					archiveFrom(f_out)
						<< *(time_ordered_list_observation.begin()->second);
					time_ordered_list_observation.erase(
						time_ordered_list_observation.begin());
				}
			}

			// Free memory:
			newObj.reset();
		}
		catch (exception& e)
		{
			errorMsg = mrpt::exception_to_str(e);
			keepLoading = false;
		}
		catch (...)
		{
			keepLoading = false;
		}
	}  // end while keep loading

	// Save the rest to the out file:
	while (!time_ordered_list_observation.empty())
	{
		archiveFrom(f_out) << *(time_ordered_list_observation.begin()->second);
		time_ordered_list_observation.erase(
			time_ordered_list_observation.begin());
	}

	progDia.Update(filSize);

	// Set error msg:
	wxMessageBox(_("Done"));

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuResortByTimestamp(wxCommandEvent& event)
{
	WX_START_TRY

	bool useSensorTimestamp =
		(wxYES == wxMessageBox(
					  _("Yes: use sensor-based UTC timestamp. No: use "
						"computer-based timestamp, when available."),
					  _("Which timestamp to use?"), wxYES_NO, this));

	wxBusyCursor waitCursor;

	// First, build an ordered list of "times"->"indexes":
	// ------------------------------------------------------
	std::multimap<TTimeStamp, size_t> ordered_times;
	size_t i, n = rawlog.size();

	for (i = 0; i < n; i++)
	{
		switch (rawlog.getType(i))
		{
			default:
				wxMessageBox(
					_("Error: this command is for rawlogs without sensory "
					  "frames."));
				return;
				break;

			case CRawlog::etObservation:
			{
				CObservation::Ptr o = rawlog.getAsObservation(i);

				TTimeStamp tim = useSensorTimestamp
									 ? o->getTimeStamp()
									 : o->getOriginalReceivedTimeStamp();

				if (tim == INVALID_TIMESTAMP)
				{
					wxMessageBox(wxString::Format(
						_("Error: Element %u does not have a valid "
						  "timestamp."),
						(unsigned int)i));
					return;
				}

				ordered_times.insert(
					multimap<TTimeStamp, size_t>::value_type(tim, i));
			}
			break;
		}  // end switch type
	}  // end for i

	// Now create the new ordered rawlog
	// ------------------------------------------------------
	CRawlog temp_rawlog;
	temp_rawlog.setCommentText(rawlog.getCommentText());

	for (auto& ordered_time : ordered_times)
	{
		size_t idx = ordered_time.second;
		temp_rawlog.addObservationMemoryReference(rawlog.getAsObservation(idx));
	}

	rawlog = std::move(temp_rawlog);

	// Update the views:
	rebuildTreeView();

	tree_view->Refresh();

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuShiftTimestampsByLabel(wxCommandEvent& event)
{
	WX_START_TRY

	wxMessageBox(
		_("The timestamps of all the observations of a given sensor label will "
		  "be shifted a given number of seconds. Press OK to continue."));

	std::vector<std::string> the_labels =
		AskForObservationByLabelMultiple("Choose the sensor(s):");
	if (the_labels.empty()) return;

	wxString s = wxGetTextFromUser(
		_("Enter the number of seconds to shift (can have fractions, be "
		  "positive or negative)"),
		_("Timestamp shift"), _("0.0"), this);

	if (s.IsEmpty()) return;

	double delta_time_secs;
	if (!s.ToDouble(&delta_time_secs))
	{
		wxMessageBox(_("Invalid number"));
		return;
	}

	size_t i, n = rawlog.size();
	unsigned int nChanges = 0;

	auto DeltaTime =
		std::chrono::microseconds(static_cast<uint64_t>(delta_time_secs * 1e6));

	for (i = 0; i < n; i++)
	{
		switch (rawlog.getType(i))
		{
			case CRawlog::etSensoryFrame:
			{
				CSensoryFrame::Ptr sf = rawlog.getAsObservations(i);
				CObservation::Ptr o;
				for (const auto& the_label : the_labels)
				{
					size_t idx = 0;
					while (
						(o = sf->getObservationBySensorLabel(the_label, idx++)))
					{
						o->timestamp += DeltaTime;
						nChanges++;
					}
				}
			}
			break;

			case CRawlog::etObservation:
			{
				CObservation::Ptr o = rawlog.getAsObservation(i);

				for (const auto& the_label : the_labels)
					if (o->sensorLabel == the_label)
					{
						o->timestamp += DeltaTime;
						nChanges++;
					}
			}
			break;

			default:
				break;
		}  // end switch type

	}  // end for

	if (wxYES == wxMessageBox(
					 wxString::Format(
						 _("%u changes. Do you want to re-order by timestamp?"),
						 nChanges),
					 _("Done"), wxYES_NO, this))
	{
		OnMenuResortByTimestamp(event);
	}

	WX_END_TRY
}

// Convert from observations only to actions-SF format:
void xRawLogViewerFrame::OnMenuConvertSF(wxCommandEvent& event)
{
	WX_START_TRY

	bool onlyOnePerLabel =
		(wxYES == wxMessageBox(
					  _("Keep only one observation of each label within each "
						"sensoryframe?"),
					  _("Convert to sensoryframe's"), wxYES_NO, this));

	wxString strMaxL = wxGetTextFromUser(
		_("Maximum length of each sensoryframe (seconds):"),
		_("Convert to sensoryframe's"), _("1.0"));
	double maxLengthSF;
	strMaxL.ToDouble(&maxLengthSF);

	// Process:
	CRawlog new_rawlog;
	new_rawlog.setCommentText(rawlog.getCommentText());

	wxBusyCursor waitCursor;
	auto nEntries = (unsigned int)rawlog.size();

	wxProgressDialog progDia(
		wxT("Progress"), wxT("Parsing rawlog..."),
		nEntries,  // range
		this,  // parent
		wxPD_CAN_ABORT | wxPD_APP_MODAL | wxPD_SMOOTH | wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	CSensoryFrame SF_new;
	set<string> SF_new_labels;
	TTimeStamp SF_new_first_t = INVALID_TIMESTAMP;
	CObservationOdometry::Ptr last_sf_odo, cur_sf_odo;

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
			case CRawlog::etActionCollection:
			{
				THROW_EXCEPTION(
					"The rawlog already has sensory frames and/or actions!");
			}
			break;

			case CRawlog::etObservation:
			{
				CObservation::Ptr o = rawlog.getAsObservation(countLoop);

				// Update stats:
				bool label_existed =
					SF_new_labels.find(o->sensorLabel) != SF_new_labels.end();
				double SF_len =
					SF_new_first_t == INVALID_TIMESTAMP
						? 0
						: timeDifference(SF_new_first_t, o->timestamp);

				// Decide:
				// End SF and start a new one?
				if (SF_len > maxLengthSF && SF_new.size() != 0)
				{
					new_rawlog.addObservations(SF_new);

					// Odometry increments:
					CActionCollection acts;
					if (last_sf_odo && cur_sf_odo)
					{
						CActionRobotMovement2D act;
						act.timestamp = cur_sf_odo->timestamp;
						CActionRobotMovement2D::TMotionModelOptions opts;
						act.computeFromOdometry(
							cur_sf_odo->odometry - last_sf_odo->odometry, opts);
						acts.insert(act);
					}
					new_rawlog.addActions(acts);

					last_sf_odo = cur_sf_odo;
					cur_sf_odo.reset();

					SF_new.clear();
					SF_new_labels.clear();
					SF_new_first_t = INVALID_TIMESTAMP;
				}

				// Insert into SF:
				if (!onlyOnePerLabel || !label_existed)
				{
					SF_new.insert(o);
					SF_new_labels.insert(o->sensorLabel);
				}
				if (SF_new_first_t == INVALID_TIMESTAMP)
					SF_new_first_t = o->timestamp;

				if (o->GetRuntimeClass() == CLASS_ID(CObservationOdometry))
				{
					cur_sf_odo =
						std::dynamic_pointer_cast<CObservationOdometry>(o);
				}
			}
			break;

			default:
				break;
		}  // end for each entry

	}  // end while keep loading

	// Remaining obs:
	if (SF_new.size())
	{
		new_rawlog.addObservations(SF_new);
		SF_new.clear();
	}

	progDia.Update(nEntries);

	// Update:
	rawlog = std::move(new_rawlog);
	rebuildTreeView();

	WX_END_TRY
}

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "xRawLogViewerMain.h"

#include <mrpt/gui/CMyRedirector.h>

#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/busyinfo.h>
#include <wx/textdlg.h>
#include <wx/dirdlg.h>
#include <wx/image.h>


// General global variables:
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/os.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/utils/CTextFileLinesParser.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/obs/CObservationRFID.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;


extern TTimeStamp		rawlog_first_timestamp;

void parseGeneralVector(
	const 	 std::string &str,
	size_t   idx,
	vector<float> &outVector);

void goToNextToken(char *&str);
void goToTheLastToken(char *&str);



void goToNextToken(char *&str)
{
	while (str[0] && str[0]!=' ') str++;
	if (str[0]==' ') str++;
}

void goToTheLastToken(char *&str)
{
	while (str[0]) str++;
	str--;
	while (str[0] && str[0]!=' ') str--;
	if (str[0]==' ') str++;
}


void xRawLogViewerFrame::OnImportCARMEN(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Import a CARMEN log...");
	wxString wildcard = wxT("CARMEN log files (*.log)|*.log|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _("*.log");
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_OPEN |  wxFD_FILE_MUST_EXIST );

	if (dialog.ShowModal() != wxID_OK)
		return;

	// Save the path
	WX_START_TRY
	iniFile->write(iniFileSect,"LastDir",string(dialog.GetDirectory().mb_str()));
	WX_END_TRY


	wxString fileName = dialog.GetPath();
	string   fil( fileName.mbc_str() );

	loadedFileName = fil + string(".rawlog");

	CActionRobotMovement2D		actionMovement;
	CSensoryFrame				sf;
	CObservation2DRangeScanPtr	obsScan;
	CActionCollection			actions;
	CPose2D						frontLaserPose(0,0,0);
	CPose2D						rearLaserPose(0,0,M_PIf);
	CPose2D						thisOdoReading,lastOdoReading,Apose;
	bool						isTheFirstOdo = true;

	wxString strMaxLaserRange = wxGetTextFromUser(
									_("Maximum valid laser range:"),
									_("Enter parameter:"),
									_("30"));
	double						maxValidLaserRange;
	strMaxLaserRange.ToDouble( &maxValidLaserRange );

	float						thisTimestamp;

	unsigned int				i,n;

	CStringList     sl;
	sl.loadFromFile( fil );
	n = (int)sl.size();

	rawlog.clear();
	wxProgressDialog    progDia(
		_("Importing rawlog..."),
		_("Processing..."),
		n, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);


	i = 0;
	bool end = false;
	while (i<n && !end)
	{

		// Find the line type:
		string line( sl(i) ); //= sl->Strings[i];

		// Find the line type:
		// ----------------------------------------------------
		bool	isFrontLaser = line.find("FLASER ")==0;
		bool	isRearLaser = line.find("RLASER ")==0;
		if ( isFrontLaser || isRearLaser )
		{
			obsScan = CObservation2DRangeScan::Create();
			obsScan->aperture = M_PIf;
			obsScan->rightToLeft = true;
			obsScan->maxRange = maxValidLaserRange;
			obsScan->sensorPose = isFrontLaser ? CPose3D( frontLaserPose ):CPose3D( rearLaserPose );

			// Load readings from the string:
			char	*str = &line[7];
			int		scanSize = atoi(str);
			goToNextToken(str);

			obsScan->resizeScanAndAssign(scanSize,0, true);

			for (int q =0;q<scanSize;q++)
			{
				obsScan->setScanRange(q, atof(str));
				goToNextToken(str);
				obsScan->setScanRangeValidity(q, obsScan->scan[q] < maxValidLaserRange );
			}

			// Read odometry:
			thisOdoReading.x( atof(str) );
			goToNextToken(str);
			thisOdoReading.y( atof(str));
			goToNextToken(str);
			thisOdoReading.phi( atof(str));
			goToNextToken(str);

			if (isRearLaser)
				thisOdoReading.phi( -thisOdoReading.phi() );


			// Read timestamp:
			goToTheLastToken(str);
			thisTimestamp = atof(str);
			// Insert timestamp from Carmen logfile, adds UNIX 1970-01-01 offset
			obsScan->timestamp = time_tToTimestamp(static_cast<double>(thisTimestamp));

			//thisOdoReading
			if (isTheFirstOdo)
			{
				isTheFirstOdo = false;
				Apose = CPose2D(0,0,0);
			}
			else
			{
				Apose = thisOdoReading - lastOdoReading;
			}

			// For the next step:
			//lastTimestamp = thisTimestamp;
			lastOdoReading = thisOdoReading;


			// Add the ODO to the log:
			CActionRobotMovement2D::TMotionModelOptions	opts;

			actionMovement.computeFromOdometry( Apose, opts );

			actions.clear();
			actions.insert(actionMovement);

			rawlog.addActions( actions );

			// Add the SCAN to the log:
			sf.clear();
			sf.push_back(obsScan);

			rawlog.addObservations(sf);
		}

		if ((i++ % 30) == 0)
		{
			if (!progDia.Update( i )) end=true; // Exit the loop
			wxTheApp->Yield();  // Let the app. process messages
		}
	}

	progDia.Update(n);

	// Time to erase the progress window.
	wxTheApp->Yield();

	// Update the views:
	rebuildTreeView();

	WX_END_TRY
}


// Import a sequence of images as a new rawlog
void xRawLogViewerFrame::OnImportSequenceOfImages(wxCommandEvent& event)
{
	WX_START_TRY

	if (rawlog.size())
		if (wxYES!=wxMessageBox(_("This will overwrite your currently loaded rawlog. Proceed anyway?"),_("Import rawlog"),wxYES_NO,this))
			return;

	// Select directory:
	wxMessageBox(_("Please, select a directory which contain the images (*.bmp,*.jpg,*.tif,*.png,...) to be imported\n They will be loaded by filename ascending order."),_("Import rawlog"),wxOK,this);

	wxDirDialog dirDialog( this, _("Choose the directory that contains the images"),
						   _("."), 0, wxDefaultPosition );

	if (dirDialog.ShowModal()!=wxID_OK) return;
	string inDir( dirDialog.GetPath().mb_str() );

	// Explore ir:
	CDirectoryExplorer::TFileInfoList	lstFiles;
	CDirectoryExplorer::explore( inDir, FILE_ATTRIB_ARCHIVE, lstFiles );

	// Order by name:
	CDirectoryExplorer::sortByName(lstFiles);

	wxString strFPS = wxGetTextFromUser(
						  _("Provide frame rate (fps, or Hz):"),
						  _("To generate fake timestamps:"),
						  _("10"));
	double FPS;
	strFPS.ToDouble( &FPS );

	ASSERT_(FPS>0);

	// Compute the Timestamp increments (of 100ns):
	TTimeStamp At =(TTimeStamp )((1/FPS) * 10000000.0);


	wxBusyCursor        waitCursor;
	int				nEntries = (int)lstFiles.size();
	size_t				insertCount = 0;

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing files..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	string		errorMsg;
	rawlog.clear();

	TTimeStamp		fakeTimeStamp = getCurrentTime();


	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 5 == 0)
		{
			auxStr.sprintf(wxT("Parsed %u files"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			string filName=lstFiles[countLoop].wholePath;
			string filExt= extractFileExtension( filName );
			if ( !os::_strcmpi( "pgm",filExt.c_str() ) ||
					!os::_strcmpi( "jpg",filExt.c_str() ) ||
					!os::_strcmpi( "bmp",filExt.c_str() ) ||
					!os::_strcmpi( "png",filExt.c_str() ) ||
					!os::_strcmpi( "jpeg",filExt.c_str() ) ||
					!os::_strcmpi( "tif",filExt.c_str() ) ||
					!os::_strcmpi( "tiff",filExt.c_str() ) ||
					!os::_strcmpi( "dib",filExt.c_str() ) ||
					!os::_strcmpi( "jpe",filExt.c_str() ) ||
					!os::_strcmpi( "pbm",filExt.c_str() ) ||
					!os::_strcmpi( "ppm",filExt.c_str() ) ||
					!os::_strcmpi( "sr",filExt.c_str() ) ||
					!os::_strcmpi( "ras",filExt.c_str() ) )
			{
				// Add SF:
				CSensoryFramePtr sf=CSensoryFrame::Create();
				CObservationImagePtr  im = CObservationImage::Create();
				im->cameraPose=CPose3D(0,0,0);
				im->image.loadFromFile(filName);
				im->timestamp = fakeTimeStamp;

				// Default camera parameters:
				im->cameraParams.dist.assign(0);
				im->cameraParams.intrinsicParams.zeros();
				im->cameraParams.intrinsicParams(0,0) = 300; // fx
				im->cameraParams.intrinsicParams(1,1) = 300; // fy
				im->cameraParams.intrinsicParams(0,2) = im->image.getWidth()/2;
				im->cameraParams.intrinsicParams(1,2) = im->image.getHeight()/2;
				im->cameraParams.intrinsicParams(2,2) = 1;

				sf->push_back(im);

				rawlog.addObservationsMemoryReference( sf );

				// Add emppty action:
				CActionCollectionPtr acts=CActionCollection::Create();
				rawlog.addActionsMemoryReference( acts );

				// for the next step:
				fakeTimeStamp += At;
				insertCount++;
			}



		}
		catch (exception &e)
		{
			errorMsg = e.what();
			break;
		}
		catch (...)
		{
			break;
		}
	} // end while keep loading

	progDia.Update( nEntries );

	wxMessageBox(_U( format("%u images have been imported to rawlog format", (unsigned)insertCount ).c_str() ),_("Done"),wxOK,this);

	// Update the views:
	rebuildTreeView();

	WX_END_TRY
}

// Export rawlog as a MOOS' alog file:
void xRawLogViewerFrame::OnMenuExportALOG(wxCommandEvent& event)
{
	wxString caption = wxT("Export as ALOG...");
	wxString wildcard = wxT("ALOG files (*.alog)|*.alog|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( (loadedFileName+string(".alog")).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dialog.ShowModal() == wxID_OK)
	{
		// Ask for the robot name:
		wxString strRobot = wxGetTextFromUser(
								_("Enter the name of the robot to use in ALOG:"),
								_("Robot name:"),
								_("ROBOT"));

		string robot_name = string(strRobot.mb_str());


		wxString fil = dialog.GetPath();
		wxString filePath = dialog.GetDirectory();

		// Save the path
		WX_START_TRY

		iniFile->write(iniFileSect,"LastDir",string(filePath.mb_str()));

		// Save the file:
		string fileName =  string(fil.mb_str());

		FILE *f = os::fopen(fileName.c_str(),"wt");
		if (!f)
			THROW_EXCEPTION("Cannot open output file for write.");

		int          countLoop=0, i, n = (int)rawlog.size();

		CPose2D		 globalOdometry(0,0,0);
		TTimeStamp   firstTime = INVALID_TIMESTAMP;
		TTimeStamp   lastTime  = INVALID_TIMESTAMP;

		wxBusyCursor    waitCursor;

		wxProgressDialog    progDia(
			wxT("Exporting rawlog to ALOG"),
			wxT("Saving..."),
			n, // range
			this, // parent
			wxPD_CAN_ABORT |
			wxPD_APP_MODAL |
			wxPD_SMOOTH |
			wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME |
			wxPD_ESTIMATED_TIME |
			wxPD_REMAINING_TIME);

		wxTheApp->Yield();  // Let the app. process messages

		for (i=0;i<n;i++)
		{
			if (countLoop++ % 100 == 0)
			{
				if (!progDia.Update( i )) continue; // Exit the loop
				wxTheApp->Yield();  // Let the app. process messages
			}

			// EXPORT
			// ---------------------
			if ( rawlog.getType(i)==CRawlog::etActionCollection && lastTime!=INVALID_TIMESTAMP )
			{
				CActionCollectionPtr acts  = rawlog.getAsAction(i);

				// Have we odometry here?
				CActionRobotMovement2DPtr act = acts->getMovementEstimationByType( CActionRobotMovement2D::emOdometry );
				if ( act )
				{
					globalOdometry = globalOdometry  + act->poseChange->getMeanVal();

					// MOOS uses a different coordinate system:
					CPose2D  MOOS_odometry( -globalOdometry.y(), globalOdometry.x(), globalOdometry.phi() );

					double time_odo;
					if (act->timestamp != INVALID_TIMESTAMP )
						time_odo = timeDifference(firstTime, act->timestamp );
					else  // We must approximate it... :-S  ugly
						time_odo = timeDifference(firstTime,lastTime) + 0.01;

					::fprintf(f,"%f   %s_ODOMETRY   iPlatform    time=%f,Pose=[3x1]{%.03f,%.03f,%.03f}\n",
							time_odo,
							robot_name.c_str(),
							time_odo,
							MOOS_odometry.x(),
							MOOS_odometry.y(),
							MOOS_odometry.phi() );
				}
			}
			else if ( rawlog.getType(i)==CRawlog::etSensoryFrame )
			{
				CSensoryFramePtr SF = rawlog.getAsObservations(i);

				// Any laser scan?
				CObservation2DRangeScanPtr obs = SF->getObservationByClass<CObservation2DRangeScan>();
				if (obs)
				{
					lastTime = obs->timestamp;
					if (firstTime==INVALID_TIMESTAMP) firstTime = lastTime;
					double time_obs = timeDifference(firstTime,lastTime);
					unsigned int M_real = static_cast<unsigned int>( obs->scan.size() );
					ASSERT_(M_real==361 || M_real==181);

					::fprintf(f,"%f    LMS_LASER_2D     LMS2D  time=%f,elevation=0.000000,endElevation=0.000000,angRes=1.00,offset=0.00,subRange=0,minAngle=%.02f,maxAngle=%.02f,range=[%u]{",
							time_obs,
							time_obs,
							RAD2DEG(-obs->aperture/2),
							RAD2DEG(obs->aperture/2),
							181);

					if (M_real==361)
					{
						for (unsigned int j=0;j<181;j++)
						{
							unsigned int idx;
							if (j==90)
								idx = 180;
							else if (j<90)
								idx = 1+2*j;
							else
								idx = 2*j;

							float val = obs->validRange[idx] ? obs->scan[idx] : 0;
							if (j<(181-1))
								::fprintf(f,"%.03f,",val);
							else	::fprintf(f,"%.03f}\n",val);
						}
					}
					else
					{
						for (unsigned int idx=0;idx<181;idx++)
						{
							float val = obs->validRange[idx] ? obs->scan[idx] : 0;
							if (idx<(181-1))
								::fprintf(f,"%.03f,",val);
							else	::fprintf(f,"%.03f}\n",val);
						}
					}





				}
			} else THROW_EXCEPTION("Export to ALOG only implemented for SF-based rawlogs.");

		} // end for i

		progDia.Update( n );

		os::fclose(f);

		WX_END_TRY
	}

}


void xRawLogViewerFrame::OnImportRTL(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Import a RTL log file...");
	wxString wildcard = wxT("RTL files (*.rtl)|*.rtl|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _("*.rtl");
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_OPEN |  wxFD_FILE_MUST_EXIST );

	if (dialog.ShowModal() != wxID_OK)
		return;

	// Save the path
	WX_START_TRY
	iniFile->write(iniFileSect,"LastDir",string(dialog.GetDirectory().mb_str()));
	WX_END_TRY

	wxString fileName = dialog.GetPath();
	string   fil( fileName.mbc_str() );

	// 1) Parse the entire file:
	size_t			nLines = 0;
	size_t			nLasers=0;
	// Time -> data
	std::map<double,TAlogRecord>	theAlog;

	{
		wxBusyCursor    waitCursor;
		ifstream		f(fil.c_str());
		string			strLine;

		double init_timestamp = timestampTotime_t ( getCurrentTime() );
		//float	max_laser_range=49.5f;

		while (!f.eof())
		{
			std::getline(f,strLine);
			nLines++;

			double timestamp=0;

			// Determine type:
			TAlogRecord		newRecord;
			newRecord.type = -1;

			if (strLine.size()>3)
			{
				if ( strLine[0] == 'P' )
				{
//					size_t idx;
//					if ((idx = strLine.find("Max_scan_distance"))!=strLine.npos )
//					{
//						max_laser_range = atof( strLine.c_str() + idx + strlen("Max_scan_distance") + 1 );
//					}
					//else if
				}
				else if ( strLine[0] == 'O' )
				{
					vector<float>  rawdata;
					parseGeneralVector(strLine,2, rawdata);
					if (rawdata.size()==4)
					{
						newRecord.type = 0;  // OK, valid odometry

						timestamp = init_timestamp + rawdata[0];

						newRecord.data.resize(6);
						newRecord.data[0] = rawdata[1];
						newRecord.data[1] = rawdata[2];
						newRecord.data[2] = rawdata[3]-M_PI/2;
					}
				}
				else if ( strLine[0] == 'L' )
				{
					newRecord.endElev = 0;
					newRecord.startElev = 0;

					vector<float>  rawdata;
					parseGeneralVector(strLine,2, rawdata);

					if (rawdata.size()>10)
					{
						timestamp = init_timestamp + rawdata[0];

						size_t N = (rawdata.size()-1)/4;

						newRecord.data.clear();
						newRecord.data.reserve(N);

						for (size_t i=0;i<N;i++)
						{
							float rng = sqrt( square( rawdata[1+i*4+2]-rawdata[1+i*4+0]) + square(rawdata[1+i*4+3]-rawdata[1+i*4+1]) );
							newRecord.data.push_back( rng );
						}

						newRecord.type =  1;
						newRecord.label  = "LASER_2D";

						nLasers++;
					}
				}

				// Add:
				if (newRecord.type>=0)
				{
					// DO NOT OVERWRITE OTHER RECORDS! SHIFT TIMESTAMP A LITTLE
					std::map<double,TAlogRecord>::const_iterator itPrev = theAlog.begin();
					while (itPrev!=theAlog.end())
					{
						timestamp+=0.0002;
						itPrev = theAlog.find(timestamp);
					}
					theAlog[timestamp] = newRecord;
				}
			}
		}
	}

	// Show summary:
	string summary;
	summary = format("%u lines parsed, %u entries recognized\n",(unsigned)nLines, (unsigned)theAlog.size());
//	summary += format("%u images\n", (unsigned) nImgs);
	summary += format("%u laser scans\n", (unsigned) nLasers);
	summary += string("The rawlog is being saved to a file now.");
	::wxMessageBox(_U( summary.c_str() ) );


	// Ask for target file:
	string  target_filename;
	{
		caption = wxT("Save as...");
		wildcard = wxT("RawLog files (*.rawlog,*.rawlog.gz)|*.rawlog;*.rawlog.gz|All files (*.*)|*.*");
		defaultDir= _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() );
		defaultFilename = _( "IMPORTED.rawlog" );
		wxFileDialog  dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

		if (dialog.ShowModal() != wxID_OK)
			return;

		target_filename = string( dialog.GetPath().mbc_str() );
	}

	string  dir_for_images(".");

	// Create rawlog:
	saveImportedLogToRawlog(target_filename, theAlog, dir_for_images);

	WX_END_TRY
}

// [N]={X1,X2,...}
void parseMOOSVector(
	const 	 std::string &str,
	size_t   idx,
	vector<float> &outVector)
{
	ASSERT_( str[idx]=='[' );

	unsigned N = atoi( str.c_str()+idx+1 );

	idx = str.find("]{",idx);
	ASSERT_(idx!=string::npos);

	outVector.resize(N);

	idx+=2;  // idx points to the first number
	for (unsigned i=0;i<N;i++)
	{
		outVector[i] = atof( str.c_str()+idx );

		if (i< (N-1)  )
		{
			idx = str.find(",",idx+1) + 1;
			ASSERT_(idx!=string::npos);
		}
	}
}

// "X1,X2,..."
void parseGeneralVector(
	const 	 std::string &str,
	size_t   idx,
	vector<float> &outVector)
{
	outVector.clear();
	outVector.reserve(400);

	do
	{
		float v = atof( str.c_str()+idx );
		outVector.push_back(v);

		idx = str.find(" ",idx+1);
		if (idx!=str.npos)
			idx++;
	}
	while (idx!=str.npos);
}


void xRawLogViewerFrame::OnMenuImportALOG(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Import a MOOS alog file...");
	wxString wildcard = wxT("MOOS alog files (*.alog)|*.alog|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _("*.alog");
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_OPEN |  wxFD_FILE_MUST_EXIST );

	if (dialog.ShowModal() != wxID_OK)
		return;

	// Save the path
	WX_START_TRY
	iniFile->write(iniFileSect,"LastDir",string(dialog.GetDirectory().mb_str()));
	WX_END_TRY

	wxString fileName = dialog.GetPath();
	string   fil( fileName.mbc_str() );

	// 1) Parse the entire file:
	size_t			nLines = 0;
	size_t			nImgs  = 0, nLasers=0;
	// Time -> data
	std::map<double,TAlogRecord>	theAlog;

	{
		wxBusyCursor    waitCursor;
		ifstream		f(fil.c_str());
		string			strLine;


		while (!f.eof())
		{
			std::getline(f,strLine);
			nLines++;

			// Look for the real timestamp: "time=XXXX.XXX"
			double timestamp;
			size_t idx;
			idx = strLine.find("time=");
			// Determine type:
			TAlogRecord		newRecord;
			newRecord.type = -1;

			if (idx!=string::npos)
			{
				timestamp = atof( strLine.c_str() + idx + 5);

				if ( string::npos != (idx = strLine.find("_ODOMETRY") ))
				{
					if ( string::npos != (idx = strLine.find("Pose=[3x1]{") ))
					{
						newRecord.data.resize(6);
						if (3==sscanf(strLine.c_str()+idx+11, "%f,%f,%f", &newRecord.data[0],&newRecord.data[1],&newRecord.data[2] ))
						{
							if ( string::npos != (idx = strLine.find("Vel=[3x1]{") ))
							{
								if (3==sscanf(strLine.c_str()+idx+10, "%f,%f,%f", &newRecord.data[3],&newRecord.data[4],&newRecord.data[5] ))
								{
									newRecord.type = 0;  // OK, valid odometry
								}
							}
						}
					}
				}
				else if ( string::npos != strLine.find("LMS_LASER_2D") || string::npos != strLine.find("LMS_LASER_3D")  )
				{
					if ( string::npos != (idx = strLine.find("endElevation=") ))
					{
						newRecord.endElev = atof( strLine.c_str()+idx+13 );

						if ( string::npos != (idx = strLine.find("elevation=") ))
						{
							newRecord.startElev = atof( strLine.c_str()+idx+10 );

							if ( string::npos != (idx = strLine.find("range=") ))
							{
								parseMOOSVector( strLine, idx+6,  newRecord.data );
								if (string::npos != strLine.find("LMS_LASER_2D"))
								{
									newRecord.type =  1;
									newRecord.label  = "LASER_2D";
								}
								else
								{
									newRecord.type =  2;
									newRecord.label  = "LASER_3D";
								}
								nLasers++;
							}
						}
					}
				}

				// Add:
				if (newRecord.type>=0)
				{
					// DO NOT OVERWRITE OTHER RECORDS! SHIFT TIMESTAMP A LITTLE
					std::map<double,TAlogRecord>::const_iterator itPrev = theAlog.begin();
					while (itPrev!=theAlog.end())
					{
						timestamp+=0.0002;
						itPrev = theAlog.find(timestamp);
					}
					theAlog[timestamp] = newRecord;
				}
			}

			// Images are different:
			if ( string::npos != strLine.find("CAMERA_GRAB") )
			{
				if ( string::npos != (idx = strLine.find(".jpg") ) )
				{
					size_t  idx_slash = strLine.rfind("/",idx);
					if (idx_slash != string::npos )
					{
						//  /XXXXXXX.jpg
						//  ^       ^
						newRecord.imgFile = strLine.substr( idx_slash+1, idx+3-idx_slash );
						newRecord.type =  3;

						// Time:
						size_t idx_time = strLine.rfind("_",idx) + 1;
						if (string::npos != idx_time)
						{
							timestamp = atof( strLine.c_str() + idx_time );

							// Pan:
							size_t idx_pan = strLine.find( "Pan=" );
							if (idx_pan!=string::npos)
							{
								newRecord.data.resize(1);
								newRecord.data[0] = DEG2RAD( atof( strLine.c_str() + idx_pan + 4 ) );

								// DO NOT OVERWRITE OTHER RECORDS! SHIFT TIMESTAMP A LITTLE
								std::map<double,TAlogRecord>::const_iterator itPrev = theAlog.begin();
								while (itPrev!=theAlog.end())
								{
									timestamp+=0.0002;
									itPrev = theAlog.find(timestamp);
								}

								newRecord.label  = "CAMERA";

								theAlog[timestamp] = newRecord;
								nImgs++;
							}
						}
					}
				}
			}

		}

	}




	// Show summary:
	string summary;
	summary = format("%u lines parsed, %u entries recognized\n",(unsigned)nLines, (unsigned)theAlog.size());
	summary += format("%u images\n", (unsigned) nImgs);
	summary += format("%u laser scans\n", (unsigned) nLasers);
	summary += string("The rawlog is being saved to a file now.");
	::wxMessageBox(_U( summary.c_str() ) );


	// Ask for target file:
	string  target_filename;
	{
		caption = wxT("Save as...");
		wildcard = wxT("RawLog files (*.rawlog,*.rawlog.gz)|*.rawlog;*.rawlog.gz|All files (*.*)|*.*");
		defaultDir= _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() );
		defaultFilename = _( "IMPORTED.rawlog" );
		wxFileDialog  dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

		if (dialog.ShowModal() != wxID_OK)
			return;

		target_filename = string( dialog.GetPath().mbc_str() );
	}


	// Ask for the directory of images:
	string  dir_for_images(".");
	if ( nImgs )
	{
		::wxMessageBox(_("The alog contains images. Please pick next the directory containing all the images.") );
		wxDirDialog dirDialog(
			this,
			_("Choose the directory containing the image files from the log"),
			_U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ),
			0,
			wxDefaultPosition );

		if (dirDialog.ShowModal()!=wxID_OK)
			return;

		dir_for_images = string( dirDialog.GetPath().mb_str() );
	}
	else
	{
	    // No images, but perhaps they are in a separate directory??
        if (wxYES == wxMessageBox(_("No images found in the alog. Do you want to provide an additional directory with image files?"),_("Additional images?"),wxYES_NO ) )
        {
            wxDirDialog dirDialog(
                this,
                _("Choose the directory containing the image files from the log"),
                _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ),
                0,
                wxDefaultPosition );

            if (dirDialog.ShowModal()==wxID_OK)
            {
                dir_for_images = string( dirDialog.GetPath().mb_str() );

                // Look for image files:
                CDirectoryExplorer::TFileInfoList    lstFiles;
                CDirectoryExplorer::explore( dir_for_images, FILE_ATTRIB_ARCHIVE, lstFiles );

                CDirectoryExplorer::filterByExtension(lstFiles,"jpg");
                CDirectoryExplorer::sortByName(lstFiles);

                for (CDirectoryExplorer::TFileInfoList::iterator it=lstFiles.begin();it!=lstFiles.end();++it)
                {
                    //string strLine( it->wholePath );
                    TAlogRecord	newRecord;
                    newRecord.imgFile = it->name;
                    newRecord.type =  3;

                    // Time:
                    size_t idx_time = it->name.rfind("_") + 1;
                    if (string::npos != idx_time)
                    {
                        double timestamp = atof( it->name.c_str() + idx_time );

                        // Pan:
                        size_t idx_pan = it->name.find( "Motion" );
                        if (idx_pan!=string::npos)
                        {
                            newRecord.data.resize(1);
                            newRecord.data[0] = DEG2RAD( atof( it->name.c_str() + idx_pan + 6 ) );

                            // DO NOT OVERWRITE OTHER RECORDS! SHIFT TIMESTAMP A LITTLE
                            std::map<double,TAlogRecord>::const_iterator itPrev = theAlog.begin();
                            while (itPrev!=theAlog.end())
                            {
                                timestamp+=0.0002;
                                itPrev = theAlog.find(timestamp);
                            }

                            newRecord.label  = "CAMERA";

                            theAlog[timestamp] = newRecord;
                            nImgs++;
                        }
                    }
                }
            }
        }
	}


	// Create rawlog:
	saveImportedLogToRawlog(target_filename, theAlog, dir_for_images);

	WX_END_TRY
}

void xRawLogViewerFrame::saveImportedLogToRawlog(
	const std::string &target_filename,
	const std::map<double,TAlogRecord>	&theAlog,
	const std::string &dir_for_images )
{
	WX_START_TRY

	wxBusyCursor    waitCursor;

	CFileGZOutputStream	filOut(target_filename);



	CPose2D		lastOdometry;
	bool		firstOdo = true;
	TTimeStamp  firstSFtime = INVALID_TIMESTAMP;

	CActionRobotMovement2D::TMotionModelOptions		odoOpts;

	CSensoryFrame	sfAccum;

	size_t  progress_i, progress_N = theAlog.size();

	wxProgressDialog    progDia(
		wxT("Importing rawlog"),
		wxT("Importing..."),
		progress_N, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	std::map<double,TAlogRecord>::const_iterator  it;

	for (progress_i=0, it=theAlog.begin();it!=theAlog.end();it++)
	{
		if (progress_i++ % 50 == 0)
		{
			if (!progDia.Update( (int)progress_i  ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}


		TTimeStamp tim = time_tToTimestamp(it->first);

		if (it->second.type != 0)
		{
			// is an observation:
			CObservationPtr	 newObs;

			// Create object:
			switch (it->second.type)
			{
			case 1:
			case 2:
			{
				// ----------------
				// 2D/3D LASER
				// ----------------
				CObservation2DRangeScanPtr obs = CObservation2DRangeScan::Create();
				newObs = obs;

				obs->aperture = M_PIf;
				obs->rightToLeft = true;
				obs->resizeScan(it->second.data.size());
				for (size_t i=0;i<it->second.data.size();i++) {
					float v = it->second.data[i];
					bool valid = true;
					if (v < 0 || fabs(v-81.19)<0.01 || fabs(v-8.191)<0.01)
					{
						v=.0f;
						valid=false;
					}
					obs->setScanRange(i, v);
					obs->setScanRangeValidity(i,valid);
				}
				obs->deltaPitch = -DEG2RAD(it->second.endElev - it->second.startElev);

				obs->sensorPose.setFromValues(
					0,0,0,
					0,
					-DEG2RAD(it->second.startElev) ,
					0
				);
			}
			break;

			case 3:
			{
				// ----------------
				//  Img:
				// ----------------
				string img_file = dir_for_images + string("/") + it->second.imgFile;

				if ( fileExists( img_file ) )
				{

					CObservationImagePtr obs = CObservationImage::Create();
					newObs = obs;

					obs->cameraPose.setFromValues(
						0,0,0,
						it->second.data[0],
						0,0);

					//obs->image.loadFromFile( img_file );
					obs->image.setExternalStorage( it->second.imgFile  );
				}
				else
					cerr << "Image file does not exist: " << img_file << endl;

			} break;

			};


			// Add it:
			if (newObs)
			{
				newObs->timestamp = tim;
				newObs->sensorLabel = it->second.label;

				if (!sfAccum.size())
					firstSFtime = tim;

				sfAccum.insert( newObs );

				// Too many consecutive observations?
				if (  firstSFtime!=INVALID_TIMESTAMP &&
						timeDifference( firstSFtime, tim ) > 0.3 )
				{
					filOut << sfAccum;
					sfAccum.clear();
					firstSFtime=INVALID_TIMESTAMP;
				}
			}
		}
		else
		{
			// First, flush observations:
			if (sfAccum.size())
			{
				filOut << sfAccum;
				sfAccum.clear();
				firstSFtime=INVALID_TIMESTAMP;
			}

			// Is odometry
			CActionRobotMovement2D		act;
			CPose2D			Aodo(0,0,0);

			CPose2D  curOdo(
				it->second.data[0],
				it->second.data[1],
				it->second.data[2] + M_PI/2
			);

			if (firstOdo)
			{
				firstOdo = false;
			}
			else
			{
				Aodo = curOdo - lastOdometry;
			}

			act.computeFromOdometry( Aodo , odoOpts);
			act.timestamp = tim;

			CActionCollection	acts;
			acts.insert(act);

			filOut << acts;

			lastOdometry = curOdo;
		}
	} // end for each alog entry

	progDia.Update( progress_N );

	WX_END_TRY
}




void xRawLogViewerFrame::OnGenGasTxt(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Save as...");
	wxString wildcard = wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( (loadedFileName+string("_gasSensors.txt")).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dialog.ShowModal() == wxID_OK)
	{
		wxString fileName = dialog.GetPath();
		string        fil( fileName.mbc_str() );


		size_t          i, M = 0,  n = rawlog.size();
		FILE            *f = os::fopen( fil.c_str(), "wt");
		if (!f)
			THROW_EXCEPTION("Cannot open output file for write.");

		bool	genTimes = rawlog_first_timestamp != INVALID_TIMESTAMP;
		if (!genTimes)
			::wxMessageBox(_("It seems that there are no valid timestamps in the rawlog. Timestamps will be zero."));

		for (i=0;i<n;i++)
		{
			CObservationGasSensorsPtr obs;

			switch ( rawlog.getType(i) )
			{

			case CRawlog::etSensoryFrame:
				{
					CSensoryFramePtr sf = rawlog.getAsObservations(i);
					obs = sf->getObservationByClass<CObservationGasSensors>();
				}
				break;

			case CRawlog::etObservation:
				{
					CObservationPtr o = rawlog.getAsObservation(i);		//get the observation
					if (IS_CLASS(o,CObservationGasSensors))
					{
						obs = CObservationGasSensorsPtr(o);	//Get the GAS observation
					}
				}
				break;

			default: continue;
			}//end-case

			//If we have a GAS obs, then process it:
			if (obs)
			{
				// Timestamp:
				double t = 0;
				if (genTimes)
				{
					if (obs->timestamp!=INVALID_TIMESTAMP)
						t = mrpt::system::timeDifference(rawlog_first_timestamp, obs->timestamp);
				}

				::fprintf(f,"%f ", t );

				size_t lineCount = 0;

				//run each Enose (if more than one)
				for (size_t j=0;j<obs->m_readings.size();j++)
				{
					//Temperature
					if (obs->m_readings[j].hasTemperature== true)
					{
						float temp=obs->m_readings[j].temperature;
						::fprintf(f,"%f ",temp);
					}

					//Run each sensor on Enose
					for (vector<float>::iterator it=obs->m_readings[j].readingsVoltage.begin();it!=obs->m_readings[j].readingsVoltage.end();++it)
					{
						::fprintf(f,"%f ",*it);
						lineCount++;
					}
				}


				// Fill:
				while (lineCount<32)
				{
					::fprintf(f,"0 ");
					lineCount++;
				}

				::fprintf(f,"\n");
					M++;
			}//end-if(obs)

		}//end-for

		os::fclose(f);

		char auxStr[100];
		os::sprintf(auxStr,sizeof(auxStr),"%u entries saved!",(unsigned)M);
		wxMessageBox(_U(auxStr),_("Done"),wxOK,this);
	}//end-if

	WX_END_TRY
}


void xRawLogViewerFrame::OnGenWifiTxt(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Save as...");
	wxString wildcard = wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( (loadedFileName+string("_wifiSensors.txt")).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dialog.ShowModal() == wxID_OK)
	{
		wxString fileName = dialog.GetPath();
		string        fil( fileName.mbc_str() );


		size_t          i, M = 0,  n = rawlog.size();
		FILE            *f = os::fopen( fil.c_str(), "wt");
		if (!f)
			THROW_EXCEPTION("Cannot open output file for write.");

		bool	genTimes = rawlog_first_timestamp != INVALID_TIMESTAMP;
		if (!genTimes)
			::wxMessageBox(_("It seems that there are no valid timestamps in the rawlog. Timestamps will be zero."));

		for (i=0;i<n;i++)
		{
			CObservationWirelessPowerPtr obs;

			switch ( rawlog.getType(i) )
			{

			case CRawlog::etSensoryFrame:
				{
					CSensoryFramePtr sf = rawlog.getAsObservations(i);
					obs = sf->getObservationByClass<CObservationWirelessPower>();
				}
				break;

			case CRawlog::etObservation:
				{
					CObservationPtr o = rawlog.getAsObservation(i);		//get the observation
					if (IS_CLASS(o,CObservationWirelessPower))
					{
						obs = CObservationWirelessPowerPtr(o);	//Get the GAS observation
					}
				}
				break;

			default: continue;
			}//end-case


			//If we have a WIFI obs, then process it:
			if (obs)
			{
				// Timestamp:
				double t = 0;
				if (genTimes)
				{
					if (obs->timestamp!=INVALID_TIMESTAMP)
						t = mrpt::system::timeDifference(rawlog_first_timestamp, obs->timestamp);
				}

				::fprintf(f,"%f ", t );

				size_t lineCount = 0;

				//run each Enose (if more than one)
			/*	for (size_t j=0;j<obs->m_readings.size();j++)
				{
					//Temperature
					if (obs->m_readings[j].hasTemperature== true)
					{
						float temp=obs->m_readings[j].temperature;
						::fprintf(f,"%f ",temp);
					}

					//Run each sensor on Enose
					for (vector<float>::iterator it=obs->m_readings[j].readingsVoltage.begin();it!=obs->m_readings[j].readingsVoltage.end();++it)
					{
						::fprintf(f,"%f ",*it);
						lineCount++;
					}
				}*/


				// Fill:
				while (lineCount<32)
				{
					::fprintf(f,"0 ");
					lineCount++;
				}

				::fprintf(f,"\n");
					M++;
			}//end-if(obs)

		}//end-for

		os::fclose(f);

		char auxStr[100];
		os::sprintf(auxStr,sizeof(auxStr),"%u entries saved!",(unsigned)M);
		wxMessageBox(_U(auxStr),_("Done"),wxOK,this);
	}//end-if

	WX_END_TRY
}


void xRawLogViewerFrame::OnGenRFIDTxt(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Save as...");
	wxString wildcard = wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( (loadedFileName+string("_RFIDSensors.txt")).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dialog.ShowModal() == wxID_OK)
	{
		wxString fileName = dialog.GetPath();
		string        fil( fileName.mbc_str() );


		size_t          i, M = 0,  n = rawlog.size();
		FILE            *f = os::fopen( fil.c_str(), "wt");
		if (!f)
			THROW_EXCEPTION("Cannot open output file for write.");

		bool	genTimes = rawlog_first_timestamp != INVALID_TIMESTAMP;
		if (!genTimes)
			::wxMessageBox(_("It seems that there are no valid timestamps in the rawlog. Timestamps will be zero."));

		for (i=0;i<n;i++)
		{
			CObservationRFIDPtr obs;

			switch ( rawlog.getType(i) )
			{

			case CRawlog::etSensoryFrame:
				{
					CSensoryFramePtr sf = rawlog.getAsObservations(i);
					obs = sf->getObservationByClass<CObservationRFID>();
				}
				break;

			case CRawlog::etObservation:
				{
					CObservationPtr o = rawlog.getAsObservation(i);		//get the observation
					if (IS_CLASS(o,CObservationRFID))
					{
						obs = CObservationRFIDPtr(o);	//Get the GAS observation
					}
				}
				break;

			default: continue;
			}//end-case


			//If we have a RFID obs, then process it:
			if (obs)
			{
				// Timestamp:
				double t = 0;
				if (genTimes)
				{
					if (obs->timestamp!=INVALID_TIMESTAMP)
						t = mrpt::system::timeDifference(rawlog_first_timestamp, obs->timestamp);
				}

				::fprintf(f,"%f ", t );

				size_t lineCount = 0;

				//run each Enose (if more than one)
			/*	for (size_t j=0;j<obs->m_readings.size();j++)
				{
					//Temperature
					if (obs->m_readings[j].hasTemperature== true)
					{
						float temp=obs->m_readings[j].temperature;
						::fprintf(f,"%f ",temp);
					}

					//Run each sensor on Enose
					for (vector<float>::iterator it=obs->m_readings[j].readingsVoltage.begin();it!=obs->m_readings[j].readingsVoltage.end();++it)
					{
						::fprintf(f,"%f ",*it);
						lineCount++;
					}
				}*/


				// Fill:
				while (lineCount<32)
				{
					::fprintf(f,"0 ");
					lineCount++;
				}

				::fprintf(f,"\n");
					M++;
			}//end-if(obs)

		}//end-for

		os::fclose(f);

		char auxStr[100];
		os::sprintf(auxStr,sizeof(auxStr),"%u entries saved!",(unsigned)M);
		wxMessageBox(_U(auxStr),_("Done"),wxOK,this);
	}//end-if

	WX_END_TRY
}








// --------------------------------------------------------------------------------
// Datasets and docs from these nice guys:
//  http://www.informatik.uni-bremen.de/agebv/en/DlrSpatialCognitionDataSet
// --------------------------------------------------------------------------------
void xRawLogViewerFrame::OnMenuItemImportBremenDLRLog(wxCommandEvent& event)
{
	WX_START_TRY

	if (rawlog.size())
		if (wxYES!=wxMessageBox(_("This will overwrite your currently loaded rawlog. Proceed anyway?"),_("Import rawlog"),wxYES_NO,this))
			return;


	wxString caption = wxT("Import a uni-bremen DLR dataset...");
	wxString wildcard = wxT("Uni-bremen DLR circles dataset (*.circles)|*.circles|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _("*.circles");
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_OPEN |  wxFD_FILE_MUST_EXIST );
	if (dialog.ShowModal() != wxID_OK)
		return;
	// Save the path
	WX_START_TRY
	iniFile->write(iniFileSect,"LastDir",string(dialog.GetDirectory().mb_str()));
	WX_END_TRY

	// File to import:
	const wxString sFile = dialog.GetPath();
	string  import_filename( sFile.mbc_str() );

/* --------------------------------------------------------------------------------
   As I say above, see: http://www.informatik.uni-bremen.de/agebv/en/DlrSpatialCognitionDataSet

   Sample lines:

  CIRCLEMARKDETECTOR dlr-spatial_cognition-c.cam nohalf 0.10000 0.50000
  CIRCLEMARKDETECTOR <cameraCalibrationFile> <useHalfImages> <radius> <threshold>

Defines the parameters for the circular fiducial detector.
   <cameraCalibrationFile> is the file containing the camera calibration
   with respect to the floor plane that is used for finding the fiducials
   and converting the image location to a metrical location.

   <useHalfImages> If it is 'half' the camera calibration refers to a full
   frame image and it's y resolution must be divided by two to match the images.

   <radius> metrical radius of the fiducials (meter)

   <threshold> the visual fiducial detector assigns a quality measure between [0..1]
   to every fiducial found. Only those above <threshold> are passed to the SLAM algorithm.

  STEP dlr-spatial_cognition-c.0000 -0.03752 -0.10467 -0.10807 0.000014396673 -0.000000184001 0.000013237216 0.000015157225 -0.000001814284 0.000246181200
  STEP <image> <dX> <dY> <dTheta>, <cXX>, <cXY>, <cYY>, <cXTheta>, <cYTheta>, <cThetaTheta>

Defines a single step of the robot trajectory, that is an odometry measurement plus an image. Such
a line defines, that the robot first moved by <dX>, <dY>, <dTheta> relative to it's previous pose and
then took the image saved in <image>. <cXX>...<cThetaTheta> are the covariances of <dX>, <dY>, <dTheta>.
Units are m and radian.

   -------------------------------------------------------------------------------- */

	// Create a new dataset:
	CRawlog  newRawlog;

	const bool use_SF_format =
		(wxYES==wxMessageBox(_("Use Actions-SensoryFrames format (YES) or the Observation-only format (NO)?"),_("Import rawlog"),wxYES_NO,this));

	const bool use_ground_truth_IDs =
		(wxYES==wxMessageBox(_("Employ landmark IDs in file (YES) or make the sensor unable to identify any landmarks (NO)?"),_("Import rawlog"),wxYES_NO,this));

	// Parse line by line:
	mrpt::utils::CTextFileLinesParser  fileParser(import_filename);
	std::string line;

	mrpt::system::TTimeStamp cur_timestamp = mrpt::system::now();
	const mrpt::system::TTimeStamp time_steps = mrpt::system::secondsToTimestamp(0.25); // why not? ;-)

	loadedFileName = import_filename + string( use_SF_format ? ".rawlog" : ".obs.rawlog" );

	wxBusyCursor  wait;

	CSensoryFrame  set_of_obs; // Set of observations after the last odometry; saved BEFORE processing the next odometry.

	// For use only in Observations-only format:
	CObservationOdometry  obs_odo;
	obs_odo.sensorLabel = "ODOMETRY";
	obs_odo.odometry = CPose2D(0,0,0);

	// For stats on covariance transforms:
	CVectorDouble stats_stdRanges,stats_stdYaw;
	CVectorDouble stats_stdXs,stats_stdYs;


	int32_t next_outlier_ID = 10000; // In DLR datasets no real landmark has such a high ID... use these numbers for outliers.


	while (fileParser.getNextLine(line))
	{
		std::vector<std::string> words;
		mrpt::system::tokenize(line," \t",words);
		if (words.empty()) continue;

		if (words[0]=="STEP")
		{
			cur_timestamp+=time_steps; // Increment time counter

			// First, do we have some queued obs?
			if (!set_of_obs.empty())
			{
				if (use_SF_format)
				{
					newRawlog.addObservations(set_of_obs);
				}
				else
				{
					for (size_t i=0;i<set_of_obs.size();i++)
						newRawlog.addObservationMemoryReference( set_of_obs.getObservationByIndex(i) );
				}
				set_of_obs.clear();
			}

			if (set_of_obs.empty())
			{
				// Always create an range-bearing observation, for the cases of images without any detected landmark
				//  so we have the observation, even if it's empty:
				// Create upon first landmark:
				CObservationBearingRangePtr obs = CObservationBearingRange::Create();
				obs->sensorLabel = "CIRCLE_DETECTOR";
				obs->timestamp = cur_timestamp;
				obs->minSensorDistance = 0;
				obs->maxSensorDistance = 100;
				obs->sensor_std_yaw = 1e-4f;
				obs->sensor_std_range = 1e-2f;
				obs->sensor_std_pitch = 0; // Is a 2D sensor
				obs->fieldOfView_pitch = 0;
				obs->fieldOfView_yaw = DEG2RAD(180);
				obs->validCovariances = true;  // Each observation has its own cov. matrix.

				set_of_obs.insert(obs);
			}

			ASSERT_ABOVEEQ_(words.size(), 11)
			// Process STEP entries (odometry increments)
			//  STEP dlr-spatial_cognition-c.0000 -0.03752 -0.10467 -0.10807 0.000014396673 -0.000000184001 0.000013237216 0.000015157225 -0.000001814284 0.000246181200
			//  STEP <image> <dX> <dY> <dTheta>, <cXX>, <cXY>, <cYY>, <cXTheta>, <cYTheta>, <cThetaTheta>

			// Add the image to be inserted before the next odometry entry:
			CObservationImagePtr newImg = CObservationImage::Create();
			newImg->timestamp = cur_timestamp;
			newImg->sensorLabel = "CAMERA";
			newImg->image.setExternalStorage(words[1] + std::string(".jpg"));
			//MRPT_TODO("Camera params, ...")
			// newImg->cameraParams
			set_of_obs.insert( newImg );

			const double odo_dx   = atof(words[2].c_str());
			const double odo_dy   = atof(words[3].c_str());
			const double odo_dphi = atof(words[4].c_str());

			const CPose2D odoIncr( -odo_dy, odo_dx, odo_dphi );

			CActionRobotMovement2D  act_mov;
			CActionRobotMovement2D::TMotionModelOptions odoParams;
			odoParams.modelSelection = CActionRobotMovement2D::mmGaussian;
			odoParams.gaussianModel.a1 =
			odoParams.gaussianModel.a2 =
			odoParams.gaussianModel.a3 =
			odoParams.gaussianModel.a4 = 0;
			odoParams.gaussianModel.minStdXY  = std::sqrt( atof(words[5].c_str())+atof(words[7].c_str()) );
			odoParams.gaussianModel.minStdPHI = std::sqrt( atof(words[10].c_str()) );

			act_mov.computeFromOdometry(odoIncr,odoParams);
			act_mov.timestamp = cur_timestamp;

			if (use_SF_format)
			{
				CActionCollection  acts;
				acts.insert(act_mov);
				newRawlog.addActions(acts);
			}
			else
			{
				obs_odo.timestamp = cur_timestamp;
				obs_odo.odometry += odoIncr;
				newRawlog.addObservationMemoryReference( CObservationOdometryPtr( new CObservationOdometry( obs_odo )));
			}
		}
		else
		if (words[0]=="LANDMARK_C")
		{
			ASSERT_ABOVEEQ_(words.size(), 8)
			// LANDMARK_C 0.67972 -2.87676 0.600235 0.00176143 -0.000314459 0.00334762 -1
			// #  LANDMARK <pX> <pY> <quality> <cXX> <cXY> <cYY> <ID>

			// Just append this measure to the observation within "set_of_obs":

			CObservationBearingRangePtr obs = set_of_obs.getObservationByClass<CObservationBearingRange>();
			ASSERT_(obs)

			// <pX> <pY> <quality> <cXX> <cXY> <cYY> <ID>
			const double lm_x = -atof(words[2].c_str());  // Ey! Yes, I flipped the coord. system to match MRPT's standard
			const double lm_y =  atof(words[1].c_str());  //  of +X pointing fordward, +Y to the left.

			const double r = std::sqrt( square(lm_x)+square(lm_y) );
			const double a = atan2(lm_y,lm_x);

			/* Transform covariance from (x,y) -> (r,a):
			 *   Jacobian:
			 *   [ x/(x^2 + y^2)^(1/2), y/(x^2 + y^2)^(1/2)]  // range
			 *   [      -y/(x^2 + y^2),       x/(x^2 + y^2)]  // yaw
			 */
			const double Cxy_xx = atof(words[4].c_str());
			const double Cxy_xy = atof(words[5].c_str());
			const double Cxy_yy = atof(words[6].c_str());

			const double dat_C_xy[3*3]= {
				Cxy_xx, Cxy_xy, 0,
				Cxy_xy, Cxy_yy, 0,
				0,      0,      0
				};
			const CMatrixDouble33  C_xy(dat_C_xy);

			// Jacobian:
			const double dat_H[3*3]= {
				lm_x / r,    lm_y / r,    0,
				-lm_y/(r*r), lm_x/(r*r),  0,
				0,           0,           0
				};
			const CMatrixDouble33 H(dat_H);

			// Transform covariance with Jacobian:
			const CMatrixDouble33 C_ypr = H * C_xy * H.transpose();

			// Create obs:
			CObservationBearingRange::TMeasurement meas;
			if (use_ground_truth_IDs)
			{
				const int32_t ID_in_file = atoi(words[7].c_str());
				if (ID_in_file<0)
				{
					// If the ground-truth says this landmark is an outlier (ID=-1), then
					//  assign a unique ID in a range that will not collide with real landmarks:
					meas.landmarkID = next_outlier_ID++;
				}
				else
				{
					meas.landmarkID = ID_in_file;
				}
			}
			else
			{
				meas.landmarkID = INVALID_LANDMARK_ID; // Unknown IDs
			}

			meas.range  = r;
			meas.yaw    = a;
			meas.pitch  = 0;
			meas.covariance = C_ypr;

			obs->sensedData.push_back(meas);

			// gather stats:
			stats_stdRanges.push_back( std::sqrt(C_ypr(0,0)));
			stats_stdYaw.push_back   ( std::sqrt(C_ypr(1,1)));
			stats_stdXs.push_back( std::sqrt(C_xy(0,0)));
			stats_stdYs.push_back( std::sqrt(C_xy(1,1)));
		}
	};

	// finally, we have to repeat this part not to lost the last few observations:
	if (!set_of_obs.empty())
	{
		if (use_SF_format)
		{
			newRawlog.addObservations(set_of_obs);
		}
		else
		{
			for (size_t i=0;i<set_of_obs.size();i++)
				newRawlog.addObservationMemoryReference( set_of_obs.getObservationByIndex(i) );
		}
	}

	wxTheApp->Yield();
	// Update the views:
	rawlog = newRawlog; // Load into GUI
	rebuildTreeView();


	// Extra info, to the console:
	if (stats_stdRanges.size()!=0)
	{
		cout << "[ImportBremenDLRLog] Stats for: " << import_filename << endl;

		double stdRange_mean , stdRange_std;
		double stdYaw_mean   , stdYaw_std;
		mrpt::math::meanAndStd(stats_stdRanges,stdRange_mean , stdRange_std);
		mrpt::math::meanAndStd(stats_stdYaw   ,stdYaw_mean   , stdYaw_std);
		cout << "[ImportBremenDLRLog] Computed stdRange (m): mean= " << stdRange_mean << " std= " << stdRange_std << endl;
		cout << "[ImportBremenDLRLog] Computed stdYaw (deg): mean= " << RAD2DEG(stdYaw_mean) << " std= " << RAD2DEG(stdYaw_std) << endl;

		double stdX_mean , stdX_std;
		double stdY_mean   , stdY_std;
		mrpt::math::meanAndStd(stats_stdXs, stdX_mean , stdX_std);
		mrpt::math::meanAndStd(stats_stdYs, stdY_mean , stdY_std);
		cout << "[ImportBremenDLRLog] Computed stdX (m): mean= " << stdX_mean << " std= " << stdX_std << endl;
		cout << "[ImportBremenDLRLog] Computed stdY (m): mean= " << stdY_mean << " std= " << stdY_std << endl;
	}

	WX_END_TRY
}


void xRawLogViewerFrame::OnGenerateIMUTextFile(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Save as...");
	wxString wildcard = wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( (loadedFileName+string("_IMU.txt")).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if ( dialog.ShowModal() == wxID_OK )
	{
		wxString fileName = dialog.GetPath();
		string        fil( fileName.mbc_str() );

		int             i, M = 0,  n = (int)rawlog.size();
		FILE            *f = os::fopen( fil.c_str(), "wt");
		if (!f)
			THROW_EXCEPTION("Cannot open output file for write.");

		::fprintf(f,
		"%% TIMESTAMP IMU_X_ACC IMU_Y_ACC IMU_Z_ACC IMU_YAW_VEL IMU_PITCH_VEL IMU_ROLL_VEL IMU_X_VEL IMU_Y_VEL IMU_Z_VEL IMU_YAW IMU_PITCH IMU_ROLL IMU_X IMU_Y IMU_Z MAG_X MAG_Y MAG_Z PRESS ALTIT TEMP\n"
		"%% ---------------------------------------------------------------------------------------------------------------------------------------------------------\n");

		for (i=0;i<n;i++)
		{
			switch (rawlog.getType(i))
			{
				default:
					break;

				case CRawlog::etSensoryFrame:
				{
					CSensoryFramePtr sf = rawlog.getAsObservations(i);
					size_t k = 0;
					CObservationIMUPtr obs;
					do {
						obs =sf->getObservationByClass<CObservationIMU>(k++);
						if (obs)
						{
							// For each entry in this sequence:
							if (obs->rawMeasurements.size()>0)
							{
								ASSERT_( obs->dataIsPresent.size()==obs->rawMeasurements.size() );
								size_t nValuesPerRow = obs->dataIsPresent.size();

								// For each entry in this sequence: Compute the timestamp and save all 15 values:
								ASSERT_(obs->timestamp!=INVALID_TIMESTAMP);
								TTimeStamp	t  = obs->timestamp;

								//double 	sampleTime = timeDifference( rawlog_first_timestamp ,t);
								double 	sampleTime = timestampTotime_t(t);

								// Time:
								::fprintf(f,"%f ",sampleTime);
								ASSERT_( obs->rawMeasurements.size()==obs->rawMeasurements.size() );
								for (size_t idx=0;idx<nValuesPerRow;idx++)
									::fprintf(f,"%f ",obs->dataIsPresent[idx] ? obs->rawMeasurements[idx] : 0);
								::fprintf(f,"\n");
								M++;
							} // end if
						} // end if
					} while(obs);
				}
				break;

				case CRawlog::etObservation:
				{
					CObservationPtr oo = rawlog.getAsObservation(i);

					if ( oo->GetRuntimeClass() == CLASS_ID(CObservationIMU) )
					{
						CObservationIMUPtr obs = CObservationIMUPtr(oo);
						// For each entry in this sequence:
						if (obs->rawMeasurements.size()>0)
						{
							ASSERT_( obs->dataIsPresent.size()==obs->rawMeasurements.size() );
							size_t nValuesPerRow = obs->dataIsPresent.size();

							// For each entry in this sequence: Compute the timestamp and save all 15 values:
							ASSERT_(obs->timestamp!=INVALID_TIMESTAMP);
							TTimeStamp	t  = obs->timestamp;

							double 	sampleTime = timestampTotime_t(t); //timeDifference(rawlog_first_timestamp,t);

							// Time:
							::fprintf(f,"%f ",sampleTime);
							ASSERT_( obs->rawMeasurements.size()==obs->rawMeasurements.size() );
							for (size_t idx=0;idx<nValuesPerRow;idx++)
								::fprintf(f,"%f ",obs->dataIsPresent[idx] ? obs->rawMeasurements[idx] : 0);
							::fprintf(f,"\n");
							M++;
						} // end if
					}
				}
				break;

			} // end switch.

		}

		os::fclose(f);

		wxMessageBox(_U( format("%u IMU data entries saved!",M).c_str() ),_("Done"),wxOK,this);
	}

	WX_END_TRY
}

void xRawLogViewerFrame::OnGenerateTextFileRangeBearing(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Save as...");
	wxString wildcard = wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( (loadedFileName+string("_RANGE_BEARING.txt")).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if ( dialog.ShowModal() == wxID_OK )
	{
		wxString fileName = dialog.GetPath();
		string        fil( fileName.mbc_str() );

		int             i, M = 0,  n = (int)rawlog.size();
		FILE            *f = os::fopen( fil.c_str(), "wt");
		if (!f)
			THROW_EXCEPTION("Cannot open output file for write.");

		for (i=0;i<n;i++)
		{
			if ( rawlog.getType(i)==CRawlog::etSensoryFrame )
			{
				CSensoryFramePtr sf = rawlog.getAsObservations(i);
				CObservationBearingRangePtr obs = sf->getObservationByClass<CObservationBearingRange>();
				if (obs)
				{
					// For each entry in this sequence:
					for (size_t q=0;q<obs->sensedData.size();q++)
					{
						M++;
						::fprintf(f,"%u %i %f %f %f\n",
								i,
								obs->sensedData[q].landmarkID,
								obs->sensedData[q].range,
								obs->sensedData[q].yaw,
								obs->sensedData[q].pitch);
					}
				}
			}
			else if ( rawlog.getType(i)==CRawlog::etObservation )
			{
				CObservationPtr o = rawlog.getAsObservation(i);

				if (IS_CLASS(o,CObservationBearingRange))
				{
					CObservationBearingRangePtr obs = CObservationBearingRangePtr(o);
					// For each entry in this sequence:
					for (size_t q=0;q<obs->sensedData.size();q++)
					{
						M++;
						::fprintf(f,"%u %i %f %f %f\n",
								i,
								obs->sensedData[q].landmarkID,
								obs->sensedData[q].range,
								obs->sensedData[q].yaw,
								obs->sensedData[q].pitch);
					}
				}
			}
		}

		os::fclose(f);

		wxMessageBox(_U( format("%u bearing-range data entries saved!",M).c_str() ),_("Done"),wxOK,this);
	}

	WX_END_TRY
}


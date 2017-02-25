/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "xRawLogViewerMain.h"
#include "CFormEdit.h"

#include <wx/choicdlg.h>
#include <wx/msgdlg.h>
#include <wx/textdlg.h>
#include <wx/dirdlg.h>
#include <wx/filedlg.h>

// General global variables:
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/system/os.h>
#include <mrpt/topography.h>
#include <mrpt/utils/stl_containers_utils.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/geometry.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::topography;
using namespace mrpt::poses;
using namespace std;


// Show GPS path in a window:
void xRawLogViewerFrame::OnMenuDrawGPSPath(wxCommandEvent& event)
{
	WX_START_TRY

	string  the_label = AskForObservationByLabel("Choose the GPS to use:");

	size_t          i, M = 0,  n = rawlog.size();

	TGeodeticCoords  ref;
	bool			 ref_valid = false;

	// Ask the user for the reference?
	if (wxYES!=wxMessageBox(_("Do you want to take the GPS reference automatically from the first found entry?"),_("GPS path"),wxYES_NO ))
	{
		wxString s = wxGetTextFromUser(
			_("Reference Latitude (degrees):"),
			_("GPS reference"),
			_("0.0"), this );
		if (s.IsEmpty()) return;
		if (!s.ToDouble(&ref.lat.decimal_value)) { wxMessageBox(_("Invalid number")); return; }

		s = wxGetTextFromUser(
			_("Reference Longitude (degrees):"),
			_("GPS reference"),
			_("0.0"), this );
		if (s.IsEmpty()) return;
		if (!s.ToDouble(&ref.lon.decimal_value)) { wxMessageBox(_("Invalid number")); return; }

		s = wxGetTextFromUser(
			_("Reference Height (meters):"),
			_("GPS reference"),
			_("0.0"), this );
		if (s.IsEmpty()) return;
		if (!s.ToDouble(&ref.height)) { wxMessageBox(_("Invalid number")); return; }

		ref_valid=true;
	}

	// Only RTK fixed?
	bool only_rtk =  wxYES==wxMessageBox(_("Take into account 'rtk' (modes 4-5) readings only?"),_("GPS path"),wxYES_NO );

	vector<float>  xs,ys,zs;
	double  overall_distance = 0;

	for (i=0;i<n;i++)
	{
		CObservationGPSPtr obs;

		switch ( rawlog.getType(i) )
		{
		case CRawlog::etSensoryFrame:
			{
				CSensoryFramePtr sf = rawlog.getAsObservations(i);

				CObservationPtr o= sf->getObservationBySensorLabel(the_label);
				if (o && IS_CLASS(o,CObservationGPS))
				{
					obs = CObservationGPSPtr(o);
				}
			}
			break;

		case CRawlog::etObservation:
			{
				CObservationPtr o = rawlog.getAsObservation(i);

				if ( !os::_strcmpi(o->sensorLabel.c_str(), the_label.c_str()) && IS_CLASS(o,CObservationGPS))
				{
					obs = CObservationGPSPtr(o);
				}
			}
			break;

		default: break;
		}

		// If we had a GPS obs, process it:
		const mrpt::obs::gnss::Message_NMEA_GGA *gga = NULL;
		if (obs && obs->hasMsgClass<mrpt::obs::gnss::Message_NMEA_GGA>()) {
			gga = &obs->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
		}

		if (gga && (!only_rtk || gga->fields.fix_quality==4 || gga->fields.fix_quality==5) )
		{
			TPoint3D  X_ENU;		// Transformed coordinates

			const TGeodeticCoords obsCoords = gga->getAsStruct<TGeodeticCoords>();

			// The first gps datum?
			if (!ref_valid)
			{
				ref_valid=true;
				ref = obsCoords;
			}

			// Local XYZ coordinates transform:
			geodeticToENU_WGS84( obsCoords, X_ENU, ref );

			// Geocentric XYZ:
			TPoint3D  X_geo;
			geodeticToGeocentric_WGS84( obsCoords, X_geo);

			if (!xs.empty())
				overall_distance+=sqrt( square(X_ENU.x-*xs.rbegin())+square(X_ENU.y-*ys.rbegin())+square(X_ENU.z-*zs.rbegin()) );

			xs.push_back(X_ENU.x);
			ys.push_back(X_ENU.y);
			zs.push_back(X_ENU.z);

			M++;
		}
	}

	// Window 3d:
	winGPSPath = CDisplayWindow3D::Create(format("GPS path, %i points (%s) %.03f meters length", int(M), the_label.c_str(), overall_distance ) );

	COpenGLScene scene;
	CPointCloudPtr  gl_path = CPointCloud::Create();
	gl_path->setAllPoints(xs,ys,zs);
	gl_path->setColor(0,0,1);

	gl_path->setPointSize(3);

	scene.insert( gl_path );
	scene.insert( CGridPlaneXYPtr( CGridPlaneXY::Create(-300,300,-300,300,0,10)));
	scene.insert( CAxisPtr( CAxis::Create(-300,-300,-50, 300,300,50, 1.0, 3, true  ) ) );

	COpenGLScenePtr the_scene = winGPSPath->get3DSceneAndLock();
	*the_scene = scene;
	winGPSPath->unlockAccess3DScene();
	winGPSPath->repaint();


	// 2D wins:
	winGPSPath2D_xy = CDisplayWindowPlots::Create( format("GPS path - XY (%s)", the_label.c_str() ) );
	winGPSPath2D_xy->plot(xs,ys,"b");
	winGPSPath2D_xy->axis_fit(true);

	winGPSPath2D_xz = CDisplayWindowPlots::Create( format("GPS path - XZ (%s)", the_label.c_str() ) );
	winGPSPath2D_xz->plot(xs,zs,"b");
	winGPSPath2D_xz->axis_fit(true);


	WX_END_TRY
}



void fixGPStimestamp(CObservationGPSPtr &obs, CVectorDouble &time_changes, std::map<std::string,double> &DeltaTimes )
{
	if (!obs->has_GGA_datum && !obs->has_RMC_datum) return;

	CObservationGPS::TUTCTime	theTime;
	bool  hasTime=false;

	const gnss::Message_NMEA_GGA *gga = NULL;
	if (obs && obs->hasMsgClass<gnss::Message_NMEA_GGA>()) {
		gga = &obs->getMsgByClass<gnss::Message_NMEA_GGA>();
	}
	const gnss::Message_NMEA_RMC *rmc = NULL;
	if (obs && obs->hasMsgClass<gnss::Message_NMEA_RMC>()) {
		rmc = &obs->getMsgByClass<gnss::Message_NMEA_RMC>();
	}

	if (gga && gga->fields.fix_quality>0)
	{
		theTime = gga->fields.UTCTime;
		hasTime = true;
	}
	else
	if (rmc && rmc->fields.validity_char=='A' )
	{
		theTime = rmc->fields.UTCTime;
		hasTime = true;
	}

	 // The last known delta_time for this sensor name
	if (DeltaTimes.find( obs->sensorLabel )==DeltaTimes.end())
		DeltaTimes[obs->sensorLabel] = 0;

	double &DeltaTime = DeltaTimes[obs->sensorLabel];

	if ( hasTime )
	{
		TTimeParts	timparts;
		mrpt::system::timestampToParts( obs->timestamp, timparts);

		DeltaTime  = 3600*theTime.hour + 60*theTime.minute  + theTime.sec;
		DeltaTime -= 3600*timparts.hour + 60*timparts.minute + timparts.second;

		if (theTime.hour < timparts.hour-2)
		{
			// The GPS time is one day ahead the "timestamp"
			DeltaTime += 3600*24;
		}
		else if (timparts.hour > theTime.hour+2)
		{
			// The "timstamp" is one day ahead the GPS time:
			DeltaTime -= 3600*24;
		}

		// Instead of delta, just replace:
		timparts.hour 		= theTime.hour;
		timparts.minute 	= theTime.minute;
		timparts.second 	= theTime.sec;

		obs->timestamp = buildTimestampFromParts(timparts);
	}
	else
	{
		// Use last delta
		obs->timestamp += mrpt::system::secondsToTimestamp(DeltaTime);
	}

	// Fix timestamp:
	time_changes.push_back( DeltaTime );
}

void xRawLogViewerFrame::OnMenuRegenerateGPSTimestamps(wxCommandEvent& event)
{
	WX_START_TRY

	wxBusyCursor        waitCursor;

	vector_string  the_labels = AskForObservationByLabelMultiple("Choose the GPS(s) to consider:");

	CVectorDouble time_changes;	// all the shifts

	std::map<std::string,double> DeltaTimes;


	// First, build an ordered list of "times"->"indexes":
	// ------------------------------------------------------
	std::map<TTimeStamp,size_t>	ordered_times;
    size_t  i, n = rawlog.size();

    for (i=0;i<n;i++)
    {
        switch ( rawlog.getType(i) )
        {
        default:
			wxMessageBox(_("Error: this command is for rawlogs without sensory frames."));
			return;
            break;

		case CRawlog::etSensoryFrame:
			{
				CSensoryFramePtr sf = rawlog.getAsObservations(i);

				for (CSensoryFrame::iterator it=sf->begin();it!=sf->end();++it)
				{
					if ( (*it)->GetRuntimeClass()==CLASS_ID(CObservationGPS) && find_in_vector( (*it)->sensorLabel, the_labels)!=string::npos )
					{
						CObservationGPSPtr obs = CObservationGPSPtr(*it);
						fixGPStimestamp(obs, time_changes, DeltaTimes);
					}
				}
			}
			break;

        case CRawlog::etObservation:
            {
                CObservationPtr o = rawlog.getAsObservation(i);

				if (IS_CLASS(o,CObservationGPS) && find_in_vector( o->sensorLabel, the_labels)!=string::npos)
				{
					CObservationGPSPtr obs = CObservationGPSPtr(o);
					fixGPStimestamp(obs, time_changes, DeltaTimes);
				}
            }
            break;
        } // end switch type
    } // end for i

    unsigned int nChanges = time_changes.size();
    double average_time_change = 0, std_time_change = 0;
    mrpt::math::meanAndStd(time_changes,average_time_change, std_time_change );

	if (wxYES==wxMessageBox( wxString::Format(_("%u changes, average/std time shift is: %f/%f sec. Do you want to re-order by timestamp?"),nChanges, average_time_change, std_time_change  ), _("Done"),wxYES_NO, this ))
	{
		OnMenuResortByTimestamp(event);
	}


	WX_END_TRY
}



void xRawLogViewerFrame::OnMenuDistanceBtwGPSs(wxCommandEvent& event)
{
	WX_START_TRY

	wxMessageBox(_("It will be measured the distance between two GPSs, assuming they are fixed on the vehicle,\n and using only RTK fixed observations."));

	if (listOfSensorLabels.empty())
	{
	    wxMessageBox(_("No sensors were found with proper sensor labels. Labels are required for this operation."));
	    return;
	}

    // List of labels:
	wxArrayString lstLabels;
    for (std::map<std::string,TInfoPerSensorLabel>::iterator i=listOfSensorLabels.begin();i!=listOfSensorLabels.end();++i)
        lstLabels.Add( _U( i->first.c_str() ) );

	wxString ret = wxGetSingleChoice(
		_("Choose the first GPS:"),
		_("Sensor Labels"),
		lstLabels,
		this );
	if (ret.IsEmpty()) return;

	string  gps1 = string(ret.mb_str());

	ret = wxGetSingleChoice(
		_("Choose the second GPS:"),
		_("Sensor Labels"),
		lstLabels,
		this );
	if (ret.IsEmpty()) return;
	string  gps2 = string(ret.mb_str());


    size_t  i, n = rawlog.size();

    // Look for the 2 observations:
    CObservationGPSPtr last_GPS1, last_GPS2;

    CVectorDouble   dists;

	TGeodeticCoords refCoords(0,0,0);

	// Load configuration block:
	CConfigFileMemory	memFil;
	rawlog.getCommentTextAsConfigFile(memFil);

	refCoords.lat = memFil.read_double("GPS_ORIGIN","lat_deg",0);
	refCoords.lon = memFil.read_double("GPS_ORIGIN","lon_deg",0);
	refCoords.height = memFil.read_double("GPS_ORIGIN","height",0);

	bool ref_valid = !refCoords.isClear();


    for (i=0;i<n;i++)
    {
        switch ( rawlog.getType(i) )
        {
        case CRawlog::etSensoryFrame:
            {
                CSensoryFramePtr sf = rawlog.getAsObservations(i);

                if (!ref_valid)
                {
                	CObservationGPSPtr o = sf->getObservationByClass<CObservationGPS>();
                	if (o && o->has_GGA_datum)
                	{
						refCoords = o->getMsgByClass<gnss::Message_NMEA_GGA>().getAsStruct<TGeodeticCoords>();
                		ref_valid = true;
                	}
                }

                CObservationPtr o1 = sf->getObservationBySensorLabel(gps1);
                CObservationPtr o2 = sf->getObservationBySensorLabel(gps2);

                if (o1)
                {
                    ASSERT_(o1->GetRuntimeClass()==CLASS_ID(CObservationGPS));
                    CObservationGPSPtr obs = CObservationGPSPtr(o1);
                    if (obs->has_GGA_datum && obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality==4)
                        last_GPS1 = obs;
                }
                if (o2)
                {
                    ASSERT_(o2->GetRuntimeClass()==CLASS_ID(CObservationGPS));
                    CObservationGPSPtr obs = CObservationGPSPtr(o2);
                    if (obs->has_GGA_datum && obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality==4)
                        last_GPS2 = obs;
                }
            }
            break;

        case CRawlog::etObservation:
            {
                CObservationPtr o = rawlog.getAsObservation(i);

                if (!ref_valid && IS_CLASS(o,CObservationGPS))
                {
                	CObservationGPSPtr ob = CObservationGPSPtr(o);
                	if (ob && ob->has_GGA_datum)
                	{
                		refCoords = ob->getMsgByClass<gnss::Message_NMEA_GGA>().getAsStruct<TGeodeticCoords>();
                		ref_valid = true;
                	}
                }


                if (o->sensorLabel == gps1)
                {
                    ASSERT_(IS_CLASS(o,CObservationGPS));
                    CObservationGPSPtr obs = CObservationGPSPtr(o);
                    if (obs->has_GGA_datum && obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality==4)
                        last_GPS1 = obs;
                }

                if (o->sensorLabel == gps2)
                {
                    ASSERT_(IS_CLASS(o,CObservationGPS));
                    CObservationGPSPtr obs = CObservationGPSPtr(o);
                    if (obs->has_GGA_datum && obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality==4)
                        last_GPS2 = obs;
                }
            }
            break;

            default:
                break;
        } // end switch type

        // Now check if we have 2 gps with the same time stamp:
        if (last_GPS1 && last_GPS2)
        {
            if (last_GPS1->getMsgByClass<gnss::Message_NMEA_GGA>().fields.UTCTime == last_GPS2->getMsgByClass<gnss::Message_NMEA_GGA>().fields.UTCTime)
            {
                // Compute distance:
                TPoint3D  p1;
                mrpt::topography::geodeticToENU_WGS84(
                    last_GPS1->getMsgByClass<gnss::Message_NMEA_GGA>().getAsStruct<TGeodeticCoords>(),
					p1,
					refCoords);

                TPoint3D  p2;
                mrpt::topography::geodeticToENU_WGS84(
                    last_GPS2->getMsgByClass<gnss::Message_NMEA_GGA>().getAsStruct<TGeodeticCoords>(),
					p2,
					refCoords);

                // Fix offset:
                p1.x += memFil.read_double( string("OFFSET_")+last_GPS1->sensorLabel, "x", 0 );
                p1.y += memFil.read_double( string("OFFSET_")+last_GPS1->sensorLabel, "y", 0 );
                p1.z += memFil.read_double( string("OFFSET_")+last_GPS1->sensorLabel, "z", 0 );

                p2.x += memFil.read_double( string("OFFSET_")+last_GPS2->sensorLabel, "x", 0 );
                p2.y += memFil.read_double( string("OFFSET_")+last_GPS2->sensorLabel, "y", 0 );
                p2.z += memFil.read_double( string("OFFSET_")+last_GPS2->sensorLabel, "z", 0 );

                double d = mrpt::math::distance(p1,p2);

                dists.push_back(d);

                last_GPS1.clear_unique();
                last_GPS2.clear_unique();
            }
        }
    } // end for


    if (dists.empty())
    {
        wxMessageBox(_("No valid GPS observations were found."),_("Done"),wxOK,this);
    }
    else
    {
        double d_mean,d_std;
        mrpt::math::meanAndStd(dists,d_mean,d_std);

        wxMessageBox(_U(
            format("The distance between GPS sensors is %.04fm, with\n a sigma=%.04fm, average from %u entries.",
            d_mean,d_std, (unsigned)dists.size()).c_str() ),_("Done"),wxOK,this);
    }

	WX_END_TRY
}


void xRawLogViewerFrame::OnSummaryGPS(wxCommandEvent& event)
{
	WX_START_TRY

	size_t             	i, n = rawlog.size();
	std::vector<int>	histogramGPSModes(9,0);


	for (i=0;i<n;i++)
	{

		switch ( rawlog.getType(i) )
		{
		default: break;

		case CRawlog::etSensoryFrame:
			{
			CSensoryFramePtr sf = rawlog.getAsObservations(i);
			CObservationGPSPtr obs =sf->getObservationByClass<CObservationGPS>();
			if (obs)
				if (obs->has_GGA_datum)
				{
					ASSERT_(obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality<=8);
					histogramGPSModes[ obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality ] ++;
				}
			}
			break;

		case CRawlog::etObservation:
			{
			CObservationPtr o= rawlog.getAsObservation(i);
			if (IS_CLASS(o,CObservationGPS))
			{
				CObservationGPSPtr obs = CObservationGPSPtr(o);
				if (obs)
					if (obs->has_GGA_datum)
					{
						ASSERT_(obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality<=8);
						histogramGPSModes[ obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality ] ++;
					}
			}
			}
			break;
		}
	}

	string s( "Number of GPS readings for each fix_quality value:\n");
	static const char *gpsModes[9]=
	{
		"Invalid",
		"GPS fix (SPS)",
		"DGPS",
		"PPS fix",
		"RTK",
		"Float RTK",
		"Dead reckoning",
		"Manual input",
		"Simulation"
	};
	for (i=0;i<9;i++)
		s = s + format("Mode %u : %u readings (Mode: '%s')\n", (unsigned)i, histogramGPSModes[i],gpsModes[i]);

	wxMessageBox(_U(s.c_str()),_("GPS data summary"),wxOK,this);

	WX_END_TRY
}


void xRawLogViewerFrame::OnGenGPSTxt(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Save as...");
	wxString wildcard = wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( ( loadedFileName+string("_GPS.txt")).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dialog.ShowModal() == wxID_OK)
	{
		wxString fileName = dialog.GetPath();
		string        fil( fileName.mbc_str() );

		size_t          i, M = 0,  n = rawlog.size();

		map<string, FILE*>  lstFiles;

		TGeodeticCoords refCoords(0,0,0);
		bool			ref_valid = false;

		// Load configuration block:
		CConfigFileMemory	memFil;
		rawlog.getCommentTextAsConfigFile(memFil);

		refCoords.lat = memFil.read_double("GPS_ORIGIN","lat_deg",0);
		refCoords.lon = memFil.read_double("GPS_ORIGIN","lon_deg",0);
		refCoords.height = memFil.read_double("GPS_ORIGIN","height",0);

		ref_valid = !refCoords.isClear();

		CPose3D local_ENU;

		if (ref_valid)
		{
			wxMessageBox(_("GPS origin coordinates taken from rawlog configuration block"),_("Export GPS data"));
		}

		// Ask the user for the reference?
		if (!ref_valid && wxYES!=wxMessageBox(_("Do you want to take the GPS reference automatically from the first found entry?"),_("Export GPS data"),wxYES_NO ))
		{
            wxString s = wxGetTextFromUser(
                _("Reference Latitude (degrees):"),
                _("GPS reference"),
                _("0.0"), this );
            if (s.IsEmpty()) return;
            if (!s.ToDouble(&refCoords.lat.decimal_value)) { wxMessageBox(_("Invalid number")); return; }

            s = wxGetTextFromUser(
                _("Reference Longitude (degrees):"),
                _("GPS reference"),
                _("0.0"), this );
            if (s.IsEmpty()) return;
            if (!s.ToDouble(&refCoords.lon.decimal_value)) { wxMessageBox(_("Invalid number")); return; }

            s = wxGetTextFromUser(
                _("Reference Height (meters):"),
                _("GPS reference"),
                _("0.0"), this );
            if (s.IsEmpty()) return;
            if (!s.ToDouble(&refCoords.height)) { wxMessageBox(_("Invalid number")); return; }

            ref_valid=true;

			// Local coordinates reference:
			TPose3D _local_ENU;
			mrpt::topography::ENU_axes_from_WGS84(
				refCoords.lon, refCoords.lat, refCoords.height,
				_local_ENU,
				true);
			local_ENU = _local_ENU;
		}

		// All gps data:
		map< TTimeStamp, map<string,CPoint3D> > 	lstXYZallGPS;
		set< string > lstAllGPSlabels;


		for (i=0;i<n;i++)
		{
			switch ( rawlog.getType(i) )
			{
			case CRawlog::etSensoryFrame:
				{
					CSensoryFramePtr sf = rawlog.getAsObservations(i);

					size_t  ith_obs = 0;
					CObservationGPSPtr obs;
					do
					{
						obs = sf->getObservationByClass<CObservationGPS>(ith_obs++);
						if (obs)
						{
							map<string, FILE*>::const_iterator  it = lstFiles.find( obs->sensorLabel );

							FILE *f_this;

							if ( it==lstFiles.end() )	// A new fiile for this sensorlabel??
							{
								f_this = lstFiles[ obs->sensorLabel ] = os::fopen(
									format("%s_%s.txt",
										fil.c_str(),
										fileNameStripInvalidChars( obs->sensorLabel ).c_str()
										).c_str(),"wt");

								if (!f_this)
									THROW_EXCEPTION("Cannot open output file for write.");
							}
							else
								f_this = it->second;

							if (obs->has_GGA_datum) // && obs->has_RMC_datum )
							{
								TPoint3D p;		// Transformed coordinates

								// The first gps datum?
								if (!ref_valid)
								{
									ref_valid=true;
									refCoords = obs->getMsgByClass<gnss::Message_NMEA_GGA>().getAsStruct<TGeodeticCoords>();

									// Local coordinates reference:
									TPose3D _local_ENU;
									mrpt::topography::ENU_axes_from_WGS84(
										refCoords,
										_local_ENU,
										true);
									local_ENU = _local_ENU;
								}

								// Local XYZ coordinates transform:
								mrpt::topography::geodeticToENU_WGS84(
									obs->getMsgByClass<gnss::Message_NMEA_GGA>().getAsStruct<TGeodeticCoords>(),
									p,
									refCoords );

								// Geocentric XYZ:
								TPoint3D geo;
								mrpt::topography::geodeticToGeocentric_WGS84(
									obs->getMsgByClass<gnss::Message_NMEA_GGA>().getAsStruct<TGeodeticCoords>(),
									geo);

								// Save file:
								double 	tim = mrpt::system::timestampTotime_t(obs->timestamp);
								/*  obs->GGA_datum.UTCTime.hour * 3600 +
											  obs->GGA_datum.UTCTime.minute * 60 +
											  obs->GGA_datum.UTCTime.sec;*/

								::fprintf(f_this,"%.4f %.16f %.16f %f %u %u %f %f %.16f %.16f %f %i %.4f %.4f %.4f\n",
										tim,
										DEG2RAD(obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.latitude_degrees),
										DEG2RAD(obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.longitude_degrees),
										obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.altitude_meters,
										obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality,
										obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.satellitesUsed,
										obs->has_RMC_datum ? obs->getMsgByClass<gnss::Message_NMEA_RMC>().fields.speed_knots : 0.0,
										obs->has_RMC_datum ? DEG2RAD(obs->getMsgByClass<gnss::Message_NMEA_RMC>().fields.direction_degrees) : 0.0,
										p.x,p.y,p.z,
										(int)i,  // rawlog index
										geo.x, geo.y, geo.z
									   );
								M++;

								if (obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality==4)
								{
									lstXYZallGPS[obs->timestamp][obs->sensorLabel] = CPoint3D(p);
									lstAllGPSlabels.insert( obs->sensorLabel );
								}
							}
						}

					} while (obs);
				}
				break;

			case CRawlog::etObservation:
				{
					CObservationPtr o = rawlog.getAsObservation(i);

					if (IS_CLASS(o,CObservationGPS))
					{
						CObservationGPSPtr obs = CObservationGPSPtr(o);
						if (obs)
						{
							map<string, FILE*>::const_iterator  it = lstFiles.find( obs->sensorLabel );

							FILE *f_this;

							if ( it==lstFiles.end() )	// A new fiile for this sensorlabel??
							{
								std::string temp = format("%s_%s.txt",
										fil.c_str(),
										fileNameStripInvalidChars( obs->sensorLabel ).c_str()
										);
								f_this = lstFiles[ obs->sensorLabel ] = os::fopen( temp.c_str(), "wt");

								if (!f_this)
									THROW_EXCEPTION("Cannot open output file for write.");

								// The first line is a description of the columns:
								::fprintf(f_this,
									"%% "
									"%14s "				// Time
									"%23s %23s %23s "	// lat lon alt
									"%4s %4s %11s %11s "		// fix #sats speed dir
									"%23s %23s %23s "	// X Y Z local
									"%6s "				// rawlog index
									"%21s %21s %21s "	// X Y Z geocentric
									"%21s %21s %21s "	// X Y Z Cartessian (GPS)
									"%21s %21s %21s "	// VX VY VZ Cartessian (GPS)
									"%21s %21s %21s "	// VX VY VZ Cartessian (Local)
									"%14s "				// SAT Time
									"\n"
									,
									"Time",
									"Lat","Lon","Alt",
									"fix","#sats", "speed","dir",
									"Local X","Local Y","Local Z",
									"rawlog ID",
									"Geocen X","Geocen Y","Geocen Z",
									"GPS X","GPS Y","GPS Z",
									"GPS VX","GPS VY","GPS VZ",
									"Local VX","Local VY","Local VZ",
									"SAT Time"
									);
							}
							else
								f_this = it->second;

							if (obs->has_GGA_datum) // && obs->has_RMC_datum )
							{
								TPoint3D p;		// Transformed coordinates

								// The first gps datum?
								if (!ref_valid)
								{
									ref_valid=true;
									refCoords.lon = obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.longitude_degrees;
									refCoords.lat = obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.latitude_degrees;
									refCoords.height = obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.altitude_meters;

									// Local coordinates reference:
									TPose3D _local_ENU;
									mrpt::topography::ENU_axes_from_WGS84(
										refCoords.lon, refCoords.lat, refCoords.height,
										_local_ENU,
										true);
									local_ENU = _local_ENU;
								}

								// Local XYZ coordinates transform:
								mrpt::topography::geodeticToENU_WGS84(
									obs->getMsgByClass<gnss::Message_NMEA_GGA>().getAsStruct<TGeodeticCoords>(),
									p,
									refCoords);

								// Geocentric XYZ:
								TPoint3D geo;
								mrpt::topography::geodeticToGeocentric_WGS84(
									obs->getMsgByClass<gnss::Message_NMEA_GGA>().getAsStruct<TGeodeticCoords>(),
									geo );

								// Save file:
								double 	tim = mrpt::system::timestampTotime_t(obs->timestamp);

								// If available, Cartessian X Y Z, VX VY VZ, as supplied by the GPS itself:
								TPoint3D  cart_pos(0,0,0), cart_vel(0,0,0);
								TPoint3D  cart_vel_local(0,0,0);
								if (obs->has_PZS_datum && obs->getMsgByClass<gnss::Message_TOPCON_PZS>().hasCartesianPosVel)
								{
									const gnss::Message_TOPCON_PZS & pzs = obs->getMsgByClass<gnss::Message_TOPCON_PZS>();
									cart_pos.x = pzs.cartesian_x;
									cart_pos.y = pzs.cartesian_y;
									cart_pos.z = pzs.cartesian_z;

									cart_vel.x = pzs.cartesian_vx;
									cart_vel.y = pzs.cartesian_vy;
									cart_vel.z = pzs.cartesian_vz;

									cart_vel_local = TPoint3D( CPoint3D(cart_vel) - local_ENU );
								}

								::fprintf(f_this,
									"%14.4f "				// Time
									"%23.16f %23.16f %23.6f "	// lat lon alt
									"%4u %4u %11.6f %11.6f "		// fix #sats speed dir
									"%23.16f %23.16f %23.16f "	// X Y Z local
									"%6i "				// rawlog index
									"%21.16f %21.16f %21.16f "	// X Y Z geocentric
									"%21.16f %21.16f %21.16f "	// X Y Z Cartessian (GPS)
									"%21.16f %21.16f %21.16f "	// VX VY VZ Cartessian (GPS)
									"%21.16f %21.16f %21.16f "	// VX VY VZ Cartessian (Local)
									"%14.4f "				// SAT Time
									"\n",
										tim,
										DEG2RAD(obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.latitude_degrees),
										DEG2RAD(obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.longitude_degrees),
										obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.altitude_meters,
										obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality,
										obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.satellitesUsed,
										obs->has_RMC_datum ? obs->getMsgByClass<gnss::Message_NMEA_RMC>().fields.speed_knots : 0.0,
										obs->has_RMC_datum ? DEG2RAD(obs->getMsgByClass<gnss::Message_NMEA_RMC>().fields.direction_degrees) : 0.0,
										p.x,p.y,p.z,
										(int)i,  // rawlog index
										geo.x, geo.y, geo.z,
										cart_pos.x,cart_pos.y,cart_pos.z,
										cart_vel.x,cart_vel.y,cart_vel.z,
										cart_vel_local.x,cart_vel_local.y,cart_vel_local.z,
										mrpt::system::timestampTotime_t( obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.UTCTime.getAsTimestamp( obs->timestamp ) )
									);
								M++;

								if (obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality==4)
								{
									lstXYZallGPS[obs->timestamp][obs->sensorLabel] = CPoint3D(p);
									lstAllGPSlabels.insert( obs->sensorLabel );
								}
							}
						}
					}
				}
				break;

				default:
					break;
			}
		}

		for (map<string, FILE*>::const_iterator  it=lstFiles.begin();it!=lstFiles.end();++it)
		{
			os::fclose(it->second);
		}
		lstFiles.clear();


		// Save the joint file:
		// -------------------------
		// Remove those entries with not all the GPSs:
		for (map< TTimeStamp, map<string,CPoint3D> >::iterator a = lstXYZallGPS.begin();a!=lstXYZallGPS.end(); )
		{
			if ( a->second.size()!=lstAllGPSlabels.size() )
			{
				map< TTimeStamp, map<string,CPoint3D> >::iterator b = a;
				b++;
				lstXYZallGPS.erase(a);
				a = b;
			}
			else 	++a;
		}
		cout << "# of gps entries with all the GPSs:" << lstXYZallGPS.size() << endl;

		CMatrixDouble	MAT( lstXYZallGPS.size(), 1+3*lstAllGPSlabels.size() );
		int 			nLabels = 0;
		for (map< TTimeStamp, map<string,CPoint3D> >::iterator a = lstXYZallGPS.begin();a!=lstXYZallGPS.end();++a, nLabels++ )
		{
			MAT(nLabels,0) = timestampTotime_t(a->first);
			map<string,CPoint3D>   &m = a->second;
			int k = 0;
			for (set< string >::iterator it=lstAllGPSlabels.begin();it!=lstAllGPSlabels.end();++it, k++)
			{
				MAT(nLabels,1 + 3*k + 0 ) = m[*it].x();
				MAT(nLabels,1 + 3*k + 1 ) = m[*it].y();
				MAT(nLabels,1 + 3*k + 2 ) = m[*it].z();
			}
		}

		// The name of the file:
		string joint_name;
		for (set< string >::iterator it=lstAllGPSlabels.begin();it!=lstAllGPSlabels.end();++it)
		{
			joint_name += *it;
		}


		MAT.saveToTextFile( format("%s_JOINT_%s.txt",fil.c_str(), joint_name.c_str() ) );

		CMatrixDouble MAT_REF(1,3);
		MAT_REF(0,0) = refCoords.lon;
		MAT_REF(0,1) = refCoords.lat;
		MAT_REF(0,2) = refCoords.height;
		MAT_REF.saveToTextFile( format("%s_JOINTREF_%s.txt",fil.c_str(), joint_name.c_str() ), MATRIX_FORMAT_FIXED );


		wxMessageBox(_U( format("%u entries saved!",(unsigned)M).c_str() ),_("Done"),wxOK,this);
	}


	WX_END_TRY
}


// Delete rawlog entries with GPS observations with lat/lon/height being Not-A-Number
//  (useful for some vendor-specific devices...)
void filter_delGPSNan(
    mrpt::obs::CActionCollection *acts,
    mrpt::obs::CSensoryFrame *SF,
    int &changesCount  )
{
    if (SF)
    {
        for (CSensoryFrame::iterator it=SF->begin();it!=SF->end();  )
        {
        	bool del = false;
            if (IS_CLASS(*it,CObservationGPS) )
            {
            	CObservationGPSPtr o = CObservationGPSPtr(*it);
            	if (o->has_GGA_datum &&
					(mrpt::math::isNaN(o->getMsgByClass<gnss::Message_NMEA_GGA>().fields.latitude_degrees) ||
					 mrpt::math::isNaN(o->getMsgByClass<gnss::Message_NMEA_GGA>().fields.longitude_degrees) ||
					 mrpt::math::isNaN(o->getMsgByClass<gnss::Message_NMEA_GGA>().fields.altitude_meters) ) )
				{
					it = SF->erase(it);
					changesCount++;
					del = true;
				}
            }
            if (!del) ++it;
        }
    }
}

void xRawLogViewerFrame::OnMenuGPSDeleteNaN(wxCommandEvent& event)
{
	WX_START_TRY

	CFormEdit  dlgEdit(this);

	dlgEdit.rbLoaded->SetValue(true); // Apply to Rawlog in memory.
	dlgEdit.spinFirst->SetValue(0);
	dlgEdit.spinLast->SetRange(0,rawlog.size()-1);
	dlgEdit.spinLast->SetValue(rawlog.size()-1);

	dlgEdit.executeOperationOnRawlog(filter_delGPSNan,"GPS deleted with NaN values:");

	rebuildTreeView();

	WX_END_TRY
}

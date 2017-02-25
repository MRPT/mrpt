/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "topography-precomp.h"  // Precompiled headers

#include <mrpt/tfest/se3.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/topography/data_types.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/topography/path_from_rtk_gps.h>

#if MRPT_HAS_WXWIDGETS
	#include <wx/app.h>
	#include <wx/msgdlg.h>
	#include <wx/string.h>
	#include <wx/progdlg.h>
	#include <wx/busyinfo.h>
	#include <wx/log.h>
#endif   // MRPT_HAS_WXWIDGETS


using namespace std;
using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::topography;

template <class T>
std::set<T> make_set( const T& v0, const T& v1 )
{
	std::set<T> s;
	s.insert(v0);
	s.insert(v1);
	return s;
}

/*---------------------------------------------------------------
					path_from_rtk_gps
 ---------------------------------------------------------------*/
void  mrpt::topography::path_from_rtk_gps(
	mrpt::poses::CPose3DInterpolator	&robot_path,
	const mrpt::obs::CRawlog			&rawlog,
	size_t 								first,
	size_t 								last,
	bool								isGUI,
	bool								disableGPSInterp,
	int									PATH_SMOOTH_FILTER ,
	TPathFromRTKInfo					*outInfo
	)
{
	MRPT_START

#if MRPT_HAS_WXWIDGETS
	// Use a smart pointer so we are safe against exceptions:
	stlplus::smart_ptr<wxBusyCursor>	waitCursorPtr;
	if (isGUI)
		waitCursorPtr = stlplus::smart_ptr<wxBusyCursor>( new  wxBusyCursor() );
#else
	MRPT_UNUSED_PARAM(isGUI);
#endif

    // Go: generate the map:
    size_t      i;

    ASSERT_(first<=last);
    ASSERT_(last<=rawlog.size()-1);


	set<string>	lstGPSLabels;

    size_t count = 0;

	robot_path.clear();
	robot_path.setMaxTimeInterpolation(3.0);	// Max. seconds of GPS blackout not to interpolate.
	robot_path.setInterpolationMethod( CPose3DInterpolator::imSSLSLL );

	TPathFromRTKInfo	outInfoTemp;
	if (outInfo) *outInfo = outInfoTemp;

	map<string, map<TTimeStamp,TPoint3D> >	gps_paths;	// label -> (time -> 3D local coords)

    bool		abort = false;
	bool		ref_valid = false;

	// Load configuration block:
	CConfigFileMemory	memFil;
	rawlog.getCommentTextAsConfigFile(memFil);

	TGeodeticCoords ref(
		memFil.read_double("GPS_ORIGIN","lat_deg",0),
		memFil.read_double("GPS_ORIGIN","lon_deg",0),
		memFil.read_double("GPS_ORIGIN","height",0) );

	ref_valid = !ref.isClear();

	// Do we have info for the consistency test?
	const double std_0 = memFil.read_double("CONSISTENCY_TEST","std_0",0);
	bool  doConsistencyCheck = std_0>0;

	// Do we have the "reference uncertainty" matrix W^\star ??
	memFil.read_matrix("UNCERTAINTY","W_star",outInfoTemp.W_star);
	const bool doUncertaintyCovs = size(outInfoTemp.W_star,1)!=0;
	if (doUncertaintyCovs && (size(outInfoTemp.W_star,1)!=6 || size(outInfoTemp.W_star,2)!=6))
		THROW_EXCEPTION("ERROR: W_star matrix for uncertainty estimation is provided but it's not a 6x6 matrix.");

	// ------------------------------------------
	// Look for the 2 observations:
	// ------------------------------------------
#if MRPT_HAS_WXWIDGETS
	wxProgressDialog    *progDia=NULL;
	if (isGUI)
	{
		progDia = new wxProgressDialog(
			wxT("Building map"),
			wxT("Getting GPS observations..."),
			(int)(last-first+1), // range
			NULL, // parent
			wxPD_CAN_ABORT |
			wxPD_APP_MODAL |
			wxPD_SMOOTH |
			wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME |
			wxPD_ESTIMATED_TIME |
			wxPD_REMAINING_TIME);
	}
#endif

	// The list with all time ordered gps's in valid RTK mode
	typedef std::map< mrpt::system::TTimeStamp, std::map<std::string,CObservationGPSPtr> > TListGPSs;
	TListGPSs	list_gps_obs;

	map<string,size_t>  GPS_RTK_reads;	// label-># of RTK readings
	map<string,TPoint3D> GPS_local_coords_on_vehicle;  // label -> local pose on the vehicle

    for (i=first;!abort && i<=last;i++)
    {
        switch ( rawlog.getType(i) )
		{
		default:
			break;

        case CRawlog::etObservation:
            {
                CObservationPtr o = rawlog.getAsObservation(i);

				if (o->GetRuntimeClass()==CLASS_ID(CObservationGPS))
				{
					CObservationGPSPtr obs = CObservationGPSPtr(o);

					if (obs->has_GGA_datum && obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality==4)
					{
						// Add to the list:
						list_gps_obs[obs->timestamp][obs->sensorLabel] = obs;

						lstGPSLabels.insert(obs->sensorLabel);
					}

					// Save to GPS paths:
					if (obs->has_GGA_datum && (obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality==4 || obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.fix_quality==5))
					{
						GPS_RTK_reads[obs->sensorLabel]++;

						// map<string,TPoint3D> GPS_local_coords_on_vehicle;  // label -> local pose on the vehicle
						if (GPS_local_coords_on_vehicle.find(obs->sensorLabel)==GPS_local_coords_on_vehicle.end())
							GPS_local_coords_on_vehicle[obs->sensorLabel] = TPoint3D( obs->sensorPose );

						//map<string, map<TTimeStamp,TPoint3D> >	gps_paths;	// label -> (time -> 3D local coords)
						gps_paths[obs->sensorLabel][obs->timestamp] = TPoint3D(
							obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.longitude_degrees,
							obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.latitude_degrees,
							obs->getMsgByClass<gnss::Message_NMEA_GGA>().fields.altitude_meters );
					}
                }
            }
            break;
        } // end switch type

		// Show progress:
        if ((count++ % 100)==0)
        {
#if MRPT_HAS_WXWIDGETS
        	if (progDia)
        	{
				if (!progDia->Update((int)(i-first)))
					abort = true;
				wxTheApp->Yield();
        	}
#endif
        }
    } // end for i

#if MRPT_HAS_WXWIDGETS
	if (progDia)
	{
		delete progDia;
		progDia=NULL;
	}
#endif

	// -----------------------------------------------------------
	// At this point we already have the sensor positions, thus
	//  we can estimate the covariance matrix D:
	//
	// TODO: Generalize equations for # of GPS > 3
	// -----------------------------------------------------------
	map< set<string>, double >  Ad_ij;  // InterGPS distances in 2D
	map< set<string>, double >  phi_ij; // Directions on XY of the lines between i-j
	map< string, size_t>  D_cov_indexes; // Sensor label-> index in the matrix (first=0, ...)
	map< size_t, string>  D_cov_rev_indexes; // Reverse of D_cov_indexes

	CMatrixDouble	  D_cov;   // square distances cov
	CMatrixDouble	  D_cov_1;   // square distances cov (inverse)
	CVectorDouble	  D_mean;  // square distances mean

	if (doConsistencyCheck && GPS_local_coords_on_vehicle.size()==3)
	{
		unsigned int cnt = 0;
		for(map<string,TPoint3D>::iterator i=GPS_local_coords_on_vehicle.begin();i!=GPS_local_coords_on_vehicle.end();++i)
		{
			// Index tables:
			D_cov_indexes[i->first] = cnt;
			D_cov_rev_indexes[cnt] = i->first;
			cnt++;

			for(map<string,TPoint3D>::iterator j=i;j!=GPS_local_coords_on_vehicle.end();++j)
			{
				if (i!=j)
				{
					const TPoint3D  &pi = i->second;
					const TPoint3D  &pj = j->second;
					Ad_ij[ make_set(i->first,j->first) ]  = pi.distanceTo( pj );
					phi_ij[ make_set(i->first,j->first) ] = atan2( pj.y-pi.y, pj.x-pi.x );
				}
			}
		}
		ASSERT_( D_cov_indexes.size()==3  && D_cov_rev_indexes.size()==3 );

		D_cov.setSize( D_cov_indexes.size(), D_cov_indexes.size() );
		D_mean.resize( D_cov_indexes.size() );

		// See paper for the formulas!
		// TODO: generalize for N>3

		D_cov(0,0) = 2*square( Ad_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[1])] );
		D_cov(1,1) = 2*square( Ad_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[2])] );
		D_cov(2,2) = 2*square( Ad_ij[ make_set(D_cov_rev_indexes[1],D_cov_rev_indexes[2])] );

		D_cov(1,0) = Ad_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[1])] * Ad_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[2])]
					 * cos( phi_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[1])] - phi_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[2])] );
		D_cov(0,1) = D_cov(1,0);

		D_cov(2,0) =-Ad_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[1])] * Ad_ij[ make_set(D_cov_rev_indexes[1],D_cov_rev_indexes[2])]
					 * cos( phi_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[1])] - phi_ij[ make_set(D_cov_rev_indexes[1],D_cov_rev_indexes[2])] );
		D_cov(0,2) = D_cov(2,0);

		D_cov(2,1) = Ad_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[2])] * Ad_ij[ make_set(D_cov_rev_indexes[1],D_cov_rev_indexes[2])]
					 * cos( phi_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[2])] - phi_ij[ make_set(D_cov_rev_indexes[1],D_cov_rev_indexes[2])] );
		D_cov(1,2) = D_cov(2,1);

		D_cov *= 4*square(std_0);

		D_cov_1 = D_cov.inv();

		//cout << D_cov.inMatlabFormat() << endl;

		D_mean[0] = square( Ad_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[1])] );
		D_mean[1] = square( Ad_ij[ make_set(D_cov_rev_indexes[0],D_cov_rev_indexes[2])] );
		D_mean[2] = square( Ad_ij[ make_set(D_cov_rev_indexes[1],D_cov_rev_indexes[2])] );

	}
	else doConsistencyCheck =false;


	// ------------------------------------------
	// Look for the 2 observations:
	// ------------------------------------------
	int N_GPSs = list_gps_obs.size();

	if (N_GPSs)
	{
		// loop interpolate 1-out-of-5: this solves the issue with JAVAD GPSs
		//  that skip some readings at some times .0 .2 .4 .6 .8
		if (list_gps_obs.size()>4)
		{
			TListGPSs::iterator F = list_gps_obs.begin();
			++F; ++F;
			TListGPSs::iterator E = list_gps_obs.end();
			--E; --E;

			for (TListGPSs::iterator i=F;i!=E;++i)
			{
				// Now check if we have 3 gps with the same time stamp:
				//const size_t N = i->second.size();
				std::map<std::string, CObservationGPSPtr> & GPS = i->second;

				// Check if any in "lstGPSLabels" is missing here:
				for (set<string>::iterator l=lstGPSLabels.begin();l!=lstGPSLabels.end();++l)
				{
					// For each GPS in the current timestamp:
					bool fnd = ( GPS.find(*l)!=GPS.end() );

					if (fnd) continue; // this one is present.

					// Ok, we have "*l" missing in the set "*i".
					// Try to interpolate from neighbors:
					TListGPSs::iterator	i_b1 = i; --i_b1;
					TListGPSs::iterator	i_a1 = i; ++i_a1;

					CObservationGPSPtr	GPS_b1, GPS_a1;

					if (i_b1->second.find( *l )!=i_b1->second.end())
						GPS_b1 = i_b1->second.find( *l )->second;

					if (i_a1->second.find( *l )!=i_a1->second.end())
						GPS_a1 = i_a1->second.find( *l )->second;

					if (!disableGPSInterp && GPS_a1 && GPS_b1)
					{
						if ( mrpt::system::timeDifference( GPS_b1->timestamp, GPS_a1->timestamp ) < 0.5 )
						{
							CObservationGPSPtr	new_gps = CObservationGPSPtr( new CObservationGPS(*GPS_a1) );
							new_gps->sensorLabel = *l;

							//cout << mrpt::system::timeLocalToString(GPS_b1->timestamp) << " " << mrpt::system::timeLocalToString(GPS_a1->timestamp) << " " << *l;
							//cout << endl;

							new_gps->getMsgByClass<gnss::Message_NMEA_GGA>().fields.longitude_degrees = 0.5 * ( GPS_a1->getMsgByClass<gnss::Message_NMEA_GGA>().fields.longitude_degrees + GPS_b1->getMsgByClass<gnss::Message_NMEA_GGA>().fields.longitude_degrees );
							new_gps->getMsgByClass<gnss::Message_NMEA_GGA>().fields.latitude_degrees  = 0.5 * ( GPS_a1->getMsgByClass<gnss::Message_NMEA_GGA>().fields.latitude_degrees + GPS_b1->getMsgByClass<gnss::Message_NMEA_GGA>().fields.latitude_degrees );
							new_gps->getMsgByClass<gnss::Message_NMEA_GGA>().fields.altitude_meters   = 0.5 * ( GPS_a1->getMsgByClass<gnss::Message_NMEA_GGA>().fields.altitude_meters + GPS_b1->getMsgByClass<gnss::Message_NMEA_GGA>().fields.altitude_meters );

							new_gps->timestamp = (GPS_a1->timestamp + GPS_b1->timestamp) / 2;

							i->second[new_gps->sensorLabel] = new_gps;
						}
					}
				}
			} // end loop interpolate 1-out-of-5
		}



#if MRPT_HAS_WXWIDGETS
	wxProgressDialog    *progDia3=NULL;
	if (isGUI)
	{
		progDia3 = new wxProgressDialog(
			wxT("Building map"),
			wxT("Estimating 6D path..."),
			N_GPSs, // range
			NULL, // parent
			wxPD_CAN_ABORT |
			wxPD_APP_MODAL |
			wxPD_SMOOTH |
			wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME |
			wxPD_ESTIMATED_TIME |
			wxPD_REMAINING_TIME);
	}
#endif

		int	idx_in_GPSs = 0;

		for (TListGPSs::iterator i=list_gps_obs.begin();i!=list_gps_obs.end();++i, idx_in_GPSs++)
		{
			// Now check if we have 3 gps with the same time stamp:
			if (i->second.size()>=3)
			{
				const size_t N = i->second.size();
				std::map<std::string, CObservationGPSPtr> & GPS = i->second;
				CVectorDouble X(N),Y(N),Z(N); 	// Global XYZ coordinates
				std::map<string,size_t> XYZidxs;  // Sensor label -> indices in X Y Z

				if (!ref_valid)	// get the reference lat/lon, if it's not set from rawlog configuration block.
				{
					ref_valid 	= true;
					ref = GPS.begin()->second->getMsgByClass<gnss::Message_NMEA_GGA>().getAsStruct<TGeodeticCoords>();
				}

				// Compute the XYZ coordinates of all sensors:
				TMatchingPairList corrs;
				unsigned int k;
				std::map<std::string, CObservationGPSPtr>::iterator g_it;

				for (k=0,g_it=GPS.begin();g_it!=GPS.end();++g_it,++k)
				{
					TPoint3D P;
					mrpt::topography::geodeticToENU_WGS84(
						g_it->second->getMsgByClass<gnss::Message_NMEA_GGA>().getAsStruct<TGeodeticCoords>(),
						P,
						ref );

					// Correction of offsets:
					const string sect = string("OFFSET_")+g_it->second->sensorLabel;
					P.x += memFil.read_double( sect, "x", 0 );
					P.y += memFil.read_double( sect, "y", 0 );
					P.z += memFil.read_double( sect, "z", 0 );

					XYZidxs[g_it->second->sensorLabel] = k; // Save index correspondence

					// Create the correspondence:
					corrs.push_back( TMatchingPair(
						k,k, 	// Indices
						P.x,P.y,P.z, // "This"/Global coords
						g_it->second->sensorPose.x(),g_it->second->sensorPose.y(),g_it->second->sensorPose.z() // "other"/local coordinates
						));

					X[k] = P.x;
					Y[k] = P.y;
					Z[k] = P.z;
				}

				if (doConsistencyCheck && GPS.size()==3)
				{
					// XYZ[k] have the k'd final coordinates of each GPS
					// GPS[k] are the CObservations:

					// Compute the inter-GPS square distances:
					CVectorDouble iGPSdist2(3);

					// [0]: sq dist between: D_cov_rev_indexes[0],D_cov_rev_indexes[1]
					TPoint3D   P0( X[XYZidxs[D_cov_rev_indexes[0]]], Y[XYZidxs[D_cov_rev_indexes[0]]], Z[XYZidxs[D_cov_rev_indexes[0]]] );
					TPoint3D   P1( X[XYZidxs[D_cov_rev_indexes[1]]], Y[XYZidxs[D_cov_rev_indexes[1]]], Z[XYZidxs[D_cov_rev_indexes[1]]] );
					TPoint3D   P2( X[XYZidxs[D_cov_rev_indexes[2]]], Y[XYZidxs[D_cov_rev_indexes[2]]], Z[XYZidxs[D_cov_rev_indexes[2]]] );

					iGPSdist2[0] = P0.sqrDistanceTo(P1);
					iGPSdist2[1] = P0.sqrDistanceTo(P2);
					iGPSdist2[2] = P1.sqrDistanceTo(P2);

					double mahaD = mrpt::math::mahalanobisDistance( iGPSdist2, D_mean, D_cov_1 );
					outInfoTemp.mahalabis_quality_measure[i->first] = mahaD;

					//cout << "x: " << iGPSdist2 << " MU: " <<  D_mean << " -> " << mahaD  << endl;
				} // end consistency

				// Use a 6D matching method to estimate the location of the vehicle:
				CPose3DQuat optimal_pose;
				double  optimal_scale;

				// "this" (reference map) -> GPS global coordinates
				// "other" -> GPS local coordinates on the vehicle
				mrpt::tfest::se3_l2( corrs,optimal_pose,optimal_scale, true ); // Force scale=1
				//cout << "optimal pose: " << optimal_pose << " " << optimal_scale << endl;
				MRPT_CHECK_NORMAL_NUMBER( optimal_pose.x() );
				MRPT_CHECK_NORMAL_NUMBER( optimal_pose.y() );
				MRPT_CHECK_NORMAL_NUMBER( optimal_pose.z() );
				MRPT_CHECK_NORMAL_NUMBER( optimal_pose.quat().x() );
				MRPT_CHECK_NORMAL_NUMBER( optimal_pose.quat().y() );
				MRPT_CHECK_NORMAL_NUMBER( optimal_pose.quat().z() );
				MRPT_CHECK_NORMAL_NUMBER( optimal_pose.quat().r() );

				// Final vehicle pose:
				const CPose3D veh_pose= optimal_pose;

				// Add to the interpolator:
				robot_path.insert( i->first, veh_pose );

				// If we have W_star, compute the pose uncertainty:
				if (doUncertaintyCovs)
				{
					CPose3DPDFGaussian	final_veh_uncert;
					final_veh_uncert.mean.setFromValues(0,0,0,0,0,0);
					final_veh_uncert.cov = outInfoTemp.W_star;

					// Rotate the covariance according to the real vehicle pose:
					final_veh_uncert.changeCoordinatesReference(veh_pose);

					outInfoTemp.vehicle_uncertainty[ i->first ] = final_veh_uncert.cov;
				}
			}

			// Show progress:
			if ((count++ % 100)==0)
			{
#if MRPT_HAS_WXWIDGETS
				if (progDia3)
				{
					if (!progDia3->Update(idx_in_GPSs))
						abort = true;
					wxTheApp->Yield();
				}
#endif
			}
		} // end for i

#if MRPT_HAS_WXWIDGETS
	if (progDia3)
	{
		delete progDia3;
		progDia3=NULL;
	}
#endif

		if (PATH_SMOOTH_FILTER>0 && robot_path.size()>1)
		{
			CPose3DInterpolator	filtered_robot_path = robot_path;

			// Do Angles smoother filter of the path:
			// ---------------------------------------------
			const double MAX_DIST_TO_FILTER = 4.0;

			for (CPose3DInterpolator::iterator i=robot_path.begin();i!=robot_path.end();++i)
			{
				CPose3D   p = i->second;

				CVectorDouble pitchs, rolls;	// The elements to average

				pitchs.push_back(p.pitch());
				rolls.push_back(p.roll());

				CPose3DInterpolator::iterator q=i;
				for (int k=0;k<PATH_SMOOTH_FILTER && q!=robot_path.begin();k++)
				{
					--q;
					if (abs( mrpt::system::timeDifference(q->first,i->first))<MAX_DIST_TO_FILTER )
					{
						pitchs.push_back( q->second.pitch() );
						rolls.push_back( q->second.roll() );
					}
				}
				q=i;
				for (int k=0;k<PATH_SMOOTH_FILTER && q!=(--robot_path.end()) ;k++)
				{
					++q;
					if (abs( mrpt::system::timeDifference(q->first,i->first))<MAX_DIST_TO_FILTER )
					{
						pitchs.push_back( q->second.pitch() );
						rolls.push_back( q->second.roll() );
					}
				}

				p.setYawPitchRoll(p.yaw(), mrpt::math::averageWrap2Pi(pitchs), mrpt::math::averageWrap2Pi(rolls) );

				// save in filtered path:
				filtered_robot_path.insert( i->first, p );
			}
			// Replace:
			robot_path = filtered_robot_path;

		} // end PATH_SMOOTH_FILTER

	} // end step generate 6D path


	// Here we can set best_gps_path  (that with the max. number of RTK fixed/foat readings):
	if (outInfo)
	{
		string bestLabel;
		size_t bestNum = 0;
		for (map<string,size_t>::iterator i=GPS_RTK_reads.begin();i!=GPS_RTK_reads.end();++i)
		{
			if (i->second>bestNum)
			{
				bestNum = i->second;
				bestLabel = i->first;
			}
		}
		outInfoTemp.best_gps_path = gps_paths[bestLabel];

		// and transform to XYZ:
		// Correction of offsets:
		const string sect = string("OFFSET_")+bestLabel;
		const double off_X = memFil.read_double( sect, "x", 0 );
		const double off_Y = memFil.read_double( sect, "y", 0 );
		const double off_Z = memFil.read_double( sect, "z", 0 );

		// map<TTimeStamp,TPoint3D> best_gps_path;		// time -> 3D local coords
		for (map<TTimeStamp,TPoint3D>::iterator i=outInfoTemp.best_gps_path.begin();i!=outInfoTemp.best_gps_path.end();++i)
		{
			TPoint3D P;
			TPoint3D &pl = i->second;
			mrpt::topography::geodeticToENU_WGS84(
				TGeodeticCoords(pl.x,pl.y,pl.z),  // i->second.x,i->second.y,i->second.z, // lat, lon, heigh
				P, // X Y Z
				ref );

			pl.x = P.x + off_X;
			pl.y = P.y + off_Y;
			pl.z = P.z + off_Z;
		}
	} // end best_gps_path

	if (outInfo) *outInfo = outInfoTemp;


    MRPT_END
}

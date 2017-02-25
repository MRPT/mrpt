/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/math/geometry.h>
#include <mrpt/random.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileOutputStream.h>

#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/CLandmark.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationVisualLandmarks.h>
#include <mrpt/system/os.h>

#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/COpenGLScene.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace std;
using mrpt::maps::internal::TSequenceLandmarks;


//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CLandmarksMap,landmarksMap", mrpt::maps::CLandmarksMap)

CLandmarksMap::TMapDefinition::TMapDefinition()
{
}

void CLandmarksMap::TMapDefinition::loadFromConfigFile_map_specific(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	const std::string sSectCreation = sectionNamePrefix+string("_creationOpts");
	this->initialBeacons.clear();
	const unsigned  int nBeacons = source.read_int(sSectCreation,"nBeacons",0);
	for (unsigned int q=1;q<=nBeacons;q++)
	{
		TPairIdBeacon newPair;
		newPair.second = source.read_int(sSectCreation,format("beacon_%03u_ID",q),0);

		newPair.first.x= source.read_float(sSectCreation,format("beacon_%03u_x",q),0);
		newPair.first.y= source.read_float(sSectCreation,format("beacon_%03u_y",q),0);
		newPair.first.z= source.read_float(sSectCreation,format("beacon_%03u_z",q),0);

		this->initialBeacons.push_back(newPair);
	}

	insertionOpts.loadFromConfigFile(source, sectionNamePrefix+string("_insertOpts") );
	likelihoodOpts.loadFromConfigFile(source, sectionNamePrefix+string("_likelihoodOpts") );
}

void CLandmarksMap::TMapDefinition::dumpToTextStream_map_specific(mrpt::utils::CStream &out) const
{
	out.printf("number of initial beacons               = %u\n",(int)initialBeacons.size());

	out.printf("      ID         (X,Y,Z)\n");
	out.printf("--------------------------------------------------------\n");
	for (std::deque<TPairIdBeacon>::const_iterator p=initialBeacons.begin();p!=initialBeacons.end();++p)
		out.printf("      %03u         (%8.03f,%8.03f,%8.03f)\n", p->second,p->first.x,p->first.y,p->first.z);

	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CLandmarksMap::internal_CreateFromMapDefinition(const mrpt::maps::TMetricMapInitializer &_def)
{
	const CLandmarksMap::TMapDefinition &def = *dynamic_cast<const CLandmarksMap::TMapDefinition*>(&_def);
	CLandmarksMap *obj = new CLandmarksMap();

	for (std::deque<CLandmarksMap::TMapDefinition::TPairIdBeacon>::const_iterator p = def.initialBeacons.begin();p!=def.initialBeacons.end();++p)
	{
		CLandmark	lm;

		lm.createOneFeature();
		lm.features[0]->type = featBeacon;

		lm.features[0]->ID = p->second;
		lm.ID = p->second;

		lm.pose_mean = p->first;

		lm.pose_cov_11=
		lm.pose_cov_22=
		lm.pose_cov_33=
		lm.pose_cov_12=
		lm.pose_cov_13=
		lm.pose_cov_23=square(0.01f);

		obj->landmarks.push_back( lm );
	}

	obj->insertionOptions  = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	return obj;
}
//  =========== End of Map definition Block =========


IMPLEMENTS_SERIALIZABLE(CLandmarksMap, CMetricMap,mrpt::maps)

/*---------------------------------------------------------------
				Static variables initialization
  ---------------------------------------------------------------*/
std::map<std::pair<mrpt::maps::CLandmark::TLandmarkID, mrpt::maps::CLandmark::TLandmarkID>, double> CLandmarksMap::_mEDD;
mrpt::maps::CLandmark::TLandmarkID CLandmarksMap::_mapMaxID;
bool CLandmarksMap::_maxIDUpdated = false;
/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CLandmarksMap::CLandmarksMap() :
	landmarks(),
	insertionOptions(),
	likelihoodOptions(),
	insertionResults(),
	fuseOptions()
{
	landmarks.clear();
	//_mEDD.clear();
	//_mapMaxID = 0;
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CLandmarksMap::~CLandmarksMap()
{
	landmarks.clear();
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CLandmarksMap::internal_clear()
{
	landmarks.clear();
}

/*---------------------------------------------------------------
						getLandmarksCount
  ---------------------------------------------------------------*/
size_t  CLandmarksMap::size() const
{
	return landmarks.size();
}

/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CLandmarksMap::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		uint32_t n = landmarks.size();

		// First, write the number of landmarks:
		out << n;

		// Write all landmarks:
		for (TSequenceLandmarks::const_iterator	it=landmarks.begin();it!=landmarks.end();++it)
			out << (*it);

	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  CLandmarksMap::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t					n,i;
			CLandmark					lm;

			// Delete previous content of map:
			// -------------------------------------
			landmarks.clear();

			// Load from stream:
			// -------------------------------------
			in >> n;

			landmarks.clear(); // resize(n);

			// Read all landmarks:
			for (i=0;i<n;i++)
			{
				in >> lm;
				landmarks.push_back(lm);
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/**** FAMD ****/
//void  CLandmarksMap::importMapMaxID( CLandmarksMap &sourceMap )
//{
//	CLandmarksMap::_mapMaxID = sourceMap._mapMaxID;
//}
/**** END FAMD ****/



/*---------------------------------------------------------------
					computeObservationLikelihood
  ---------------------------------------------------------------*/
double	 CLandmarksMap::internal_computeObservationLikelihood(
	const CObservation		*obs,
	const CPose3D			&robotPose3D )
{
	MRPT_START

	if ( CLASS_ID(CObservation2DRangeScan )==obs->GetRuntimeClass() &&
		   insertionOptions.insert_Landmarks_from_range_scans )
	{
		/********************************************************************
						OBSERVATION TYPE: CObservation2DRangeScan
			********************************************************************/
		const CObservation2DRangeScan 	*o = static_cast<const CObservation2DRangeScan *>( obs );
		CLandmarksMap				auxMap;
		CPose2D						sensorPose2D ( robotPose3D + o->sensorPose );

		auxMap.loadOccupancyFeaturesFrom2DRangeScan(	*o,
														&robotPose3D,
														likelihoodOptions.rangeScan2D_decimation);

		// And compute its likelihood:
		return computeLikelihood_RSLC_2007( &auxMap, sensorPose2D );
	} // end of likelihood of 2D range scan:
	else
	if ( CLASS_ID(CObservationStereoImages )==obs->GetRuntimeClass() )
	{
		/********************************************************************
						OBSERVATION TYPE: CObservationStereoImages
				Lik. between "this" and "auxMap";
			********************************************************************/
		const CObservationStereoImages 	*o = static_cast<const CObservationStereoImages *>( obs );

		CLandmarksMap	auxMap;
		auxMap.insertionOptions = insertionOptions;
		auxMap.loadSiftFeaturesFromStereoImageObservation( *o, CLandmarksMap::_mapMaxID, likelihoodOptions.SIFT_feat_options );
		auxMap.changeCoordinatesReference( robotPose3D );

		// ACCESS TO STATIC VARIABLE
		//std::cout << "_mapMaxID, from " << CLandmarksMap::_mapMaxID << " to ";
		if( !CLandmarksMap::_maxIDUpdated )
		{
			CLandmarksMap::_mapMaxID += auxMap.size();
			CLandmarksMap::_maxIDUpdated = true;
		} // end if

		//std::cout << CLandmarksMap::_mapMaxID <<std::endl;
		return computeLikelihood_SIFT_LandmarkMap( &auxMap );

	} // end of likelihood of Stereo Images scan:
	else
	if ( CLASS_ID(CObservationBeaconRanges)==obs->GetRuntimeClass() )
	{
		/********************************************************************

						OBSERVATION TYPE: CObservationBeaconRanges

				Lik. between "this" and "auxMap";

			********************************************************************/
		const CObservationBeaconRanges 	*o = static_cast<const CObservationBeaconRanges*>( obs );

		std::deque<CObservationBeaconRanges::TMeasurement>::const_iterator	it;
		TSequenceLandmarks::iterator									lm_it;
		CPointPDFGaussian												beaconPDF;
		CPoint3D														point3D,beacon3D;
		double															ret = 0; //300;

		for (it = o->sensedData.begin();it!=o->sensedData.end();it++)
		{
			// Look for the beacon in this map:
			unsigned int	sensedID = it->beaconID;
			bool			found = false;

			for (lm_it=landmarks.begin();!found && lm_it!=landmarks.end();lm_it++)
			{
				if ((lm_it->getType() == featBeacon )&&
					(lm_it->ID == sensedID)&&
					(!mrpt::math::isNaN(it->sensedDistance)))
				{
					lm_it->getPose( beaconPDF );
					beacon3D = beaconPDF.mean;

					// Compute the 3D position of the sensor:
					point3D = robotPose3D + it->sensorLocationOnRobot;

					const float	expectedRange = point3D.distanceTo( beacon3D );
					float	sensedDist    = it->sensedDistance;
					if (sensedDist<0) sensedDist=0;

					float sensorStd = likelihoodOptions.beaconRangesUseObservationStd ?
						o->stdError : likelihoodOptions.beaconRangesStd;
					ret += (-0.5f*square( ( expectedRange - sensedDist  ) / sensorStd ));
					found = true;
				}
			}

			// If not found, uniform distribution:
			if (!found)
			{
				if ( o->maxSensorDistance != o->minSensorDistance )
					ret+= log(1.0/ (o->maxSensorDistance - o->minSensorDistance));
			}

		} // for each sensed beacon "it"


		MRPT_CHECK_NORMAL_NUMBER(ret);
		return ret;

	} // end of likelihood of CObservationBeaconRanges
	else
	if ( CLASS_ID(CObservationRobotPose)==obs->GetRuntimeClass() )
	{
		/********************************************************************

				OBSERVATION TYPE: CObservationRobotPose

				Lik. between "this" and "robotPose";

		********************************************************************/
		const CObservationRobotPose 	*o = static_cast<const CObservationRobotPose*>( obs );

		// Compute the 3D position of the sensor:
		CPose3D sensorPose3D = robotPose3D + o->sensorPose;

		// Compute the likelihood according to mahalanobis distance between poses:
		CMatrixD dij(1,6), Cij(6,6), Cij_1;
		dij(0,0) =          o->pose.mean.x()     - sensorPose3D.x();
		dij(0,1) =          o->pose.mean.y()     - sensorPose3D.y();
		dij(0,2) =          o->pose.mean.z()     - sensorPose3D.z();
		dij(0,3) = wrapToPi(o->pose.mean.roll()  - sensorPose3D.roll());
		dij(0,4) = wrapToPi(o->pose.mean.pitch() - sensorPose3D.pitch());
		dij(0,5) = wrapToPi(o->pose.mean.yaw()   - sensorPose3D.yaw());

		// Equivalent covariance from "i" to "j":
		Cij = CMatrixDouble(o->pose.cov);
		Cij_1 = Cij.inv();

		double distMahaFlik2 =  dij.multiply_HCHt_scalar(Cij_1); //( dij * Cij_1 * (~dij) )(0,0);
		double ret = -0.5f * distMahaFlik2;

		MRPT_CHECK_NORMAL_NUMBER(ret);
		return ret;

	} // end of likelihood of CObservation
	else
	if ( CLASS_ID(CObservationGPS)==obs->GetRuntimeClass() )
	{
		/********************************************************************

						OBSERVATION TYPE: CObservationGPS

		********************************************************************/
		const CObservationGPS			*o = static_cast<const CObservationGPS*>( obs );
		// Compute the 3D position of the sensor:
		CPoint3D				point3D = CPoint3D(robotPose3D);
		CPoint3D				GPSpose;
		double					x,y;
		double					earth_radius=6378137;

		if ((o->has_RMC_datum)&&(likelihoodOptions.GPSOrigin.min_sat<=o->getMsgByClass<gnss::Message_NMEA_GGA>().fields.satellitesUsed))
		{
			//Compose GPS robot position

			x = DEG2RAD((o->getMsgByClass<gnss::Message_NMEA_GGA>().fields.longitude_degrees - likelihoodOptions.GPSOrigin.longitude))*earth_radius*1.03;
			y = DEG2RAD((o->getMsgByClass<gnss::Message_NMEA_GGA>().fields.latitude_degrees - likelihoodOptions.GPSOrigin.latitude))*earth_radius*1.15;
			GPSpose.x( (x*cos(likelihoodOptions.GPSOrigin.ang)+y*sin(likelihoodOptions.GPSOrigin.ang)+likelihoodOptions.GPSOrigin.x_shift) );
			GPSpose.y( (-x*sin(likelihoodOptions.GPSOrigin.ang)+y*cos(likelihoodOptions.GPSOrigin.ang)+likelihoodOptions.GPSOrigin.y_shift) );
			GPSpose.z( (o->getMsgByClass<gnss::Message_NMEA_GGA>().fields.altitude_meters- likelihoodOptions.GPSOrigin.altitude) );
			//std::cout<<"GPSpose calculo: "<<GPSpose.x<<","<<GPSpose.y<<"\n";

			//-------------------------------//
			//sigmaGPS = f(o->getMsgByClass<gnss::Message_NMEA_GGA>().fields.satellitesUsed) //funcion del numero de satelites
			//-------------------------------//

			//std::cout<<"datos de longitud y latitud: "<<o->getMsgByClass<gnss::Message_NMEA_GGA>().fields.longitude_degrees<<","<<o->getMsgByClass<gnss::Message_NMEA_GGA>().fields.latitude_degrees<<","<<"\n";
			//std::cout<<"x,y sin rotar: "<<x<<","<<y<<","<<"\n";
			//std::cout<<"angulo: "<<likelihoodOptions.GPSOrigin.ang<<"\n";
			//std::cout<<"desp x,y: "<<likelihoodOptions.GPSOrigin.x_shift<<","<<likelihoodOptions.GPSOrigin.y_shift<<"\n";
			//std::cout<<"GPS ORIGIN    : "<<likelihoodOptions.GPSOrigin.longitude<<","<<likelihoodOptions.GPSOrigin.latitude<<","<<likelihoodOptions.GPSOrigin.altitude<<"\n";
			//std::cout<<"POSE DEL ROBOT: "<<point3D.x<<","<<point3D.y<<","<<point3D.z<<"\n";
			//std::cout<<"POSE GPS      : "<<GPSpose.x<<","<<GPSpose.y<<","<<GPSpose.z<<"\n";
			//std::cin.get();

			float distance = GPSpose.distanceTo(point3D);

			//std::cout<<"likel gps:"<<-0.5f*square( ( distance )/likelihoodOptions.GPS_sigma)<<"\n";;
			double ret = -0.5f*square( ( distance )/likelihoodOptions.GPS_sigma);
			MRPT_CHECK_NORMAL_NUMBER(ret);
			return ret;
		}
		else
			return 0.5;
	} // end of likelihood of CObservationGPS
	else
	{
		/********************************************************************

					OBSERVATION TYPE: Unknown

		********************************************************************/
		return 0.5;
	}

	MRPT_END

}


/*---------------------------------------------------------------
						insertObservation
  ---------------------------------------------------------------*/
bool  CLandmarksMap::internal_insertObservation( const CObservation *obs, const CPose3D *robotPose)
{
	MRPT_START

	CPose2D		robotPose2D;
	CPose3D		robotPose3D;

	if (robotPose)
	{
		robotPose2D = CPose2D(*robotPose);
		robotPose3D = (*robotPose);
	}
	else
	{
		// Default values are (0,0,0)
	}

	if ( CLASS_ID(CObservationImage)==obs->GetRuntimeClass() &&
			insertionOptions.insert_SIFTs_from_monocular_images )
	{
		/********************************************************************

						OBSERVATION TYPE: CObservationImage

			********************************************************************/
		const CObservationImage	*o = static_cast<const CObservationImage*>(obs);
		CLandmarksMap		tempMap;


		// 1) Load the features in a temporary 3D landmarks map:
		tempMap.loadSiftFeaturesFromImageObservation( *o, insertionOptions.SIFT_feat_options );

		// 2) This temp. map must be moved to its real position on the global reference coordinates:
		tempMap.changeCoordinatesReference( robotPose3D );

		// 3) Fuse that map with the current contents of "this" one:
		fuseWith( tempMap );

		// DONE!!

		// Observation was successfully inserted into the map
		// --------------------------------------------------------
		return true;
	}
//	else
//	if ( CLASS_ID(CObservation2DRangeScan)==obs->GetRuntimeClass() &&
//		  insertionOptions.insert_Landmarks_from_range_scans)
//	{
		/********************************************************************

						OBSERVATION TYPE: CObservation2DRangeScan

			********************************************************************/
/*		CObservation2DRangeScan	*o = (CObservation2DRangeScan*) obs;
		CLandmarksMap			tempMap;

		// Load into the map:
		tempMap.loadOccupancyFeaturesFrom2DRangeScan(*o, robotPose);
		fuseWith( tempMap );

		// Observation was successfully inserted into the map
		// --------------------------------------------------------
		return true;
	}       				*/
	else
	if ( CLASS_ID(CObservationStereoImages)==obs->GetRuntimeClass() &&
		   insertionOptions.insert_SIFTs_from_stereo_images)
	{
		/********************************************************************
						OBSERVATION TYPE: CObservationStereoImages
			********************************************************************/
		const CObservationStereoImages	*o = static_cast<const CObservationStereoImages*>(obs);

		// Change coordinates ref:
		CLandmarksMap	auxMap;
		auxMap.insertionOptions = insertionOptions;
		auxMap.loadSiftFeaturesFromStereoImageObservation( *o, CLandmarksMap::_mapMaxID, insertionOptions.SIFT_feat_options );
		auxMap.changeCoordinatesReference( robotPose3D );

		fuseWith( auxMap );

		// Observation was successfully inserted into the map
		// --------------------------------------------------------
		return true;
	}
	else
	if ( CLASS_ID(CObservationVisualLandmarks)==obs->GetRuntimeClass() )
	{
		/********************************************************************

						OBSERVATION TYPE:  CObservationVisualLandmarks

			********************************************************************/
		const CObservationVisualLandmarks	*o = static_cast<const CObservationVisualLandmarks*>(obs);

		// Change coordinates ref:
		CLandmarksMap				auxMap;
		CPose3D						acumTransform( robotPose3D + o->refCameraPose);
		auxMap.changeCoordinatesReference( acumTransform, &o->landmarks );

		// Fuse with current:
		fuseWith( auxMap, true);


		// Observation was successfully inserted into the map
		// --------------------------------------------------------
		return true;
	}
	else
	{
	/********************************************************************
				OBSERVATION TYPE: Unknown
	********************************************************************/
		return false;
	}

	MRPT_END

}

/*---------------------------------------------------------------
				computeMatchingWith2D
  ---------------------------------------------------------------*/
void  CLandmarksMap::computeMatchingWith2D(
		const mrpt::maps::CMetricMap						*otherMap,
		const CPose2D							&otherMapPose,
		float									maxDistForCorrespondence,
		float									maxAngularDistForCorrespondence,
		const CPose2D							&angularDistPivotPoint,
		TMatchingPairList						&correspondences,
		float									&correspondencesRatio,
		float									*sumSqrDist	,
		bool									onlyKeepTheClosest,
		bool									onlyUniqueRobust ) const
{
	MRPT_UNUSED_PARAM(onlyKeepTheClosest);MRPT_UNUSED_PARAM(onlyUniqueRobust);MRPT_UNUSED_PARAM(sumSqrDist);
	MRPT_UNUSED_PARAM(angularDistPivotPoint);MRPT_UNUSED_PARAM(maxDistForCorrespondence);MRPT_UNUSED_PARAM(maxAngularDistForCorrespondence);

	MRPT_START


	CLandmarksMap	auxMap;
	CPose3D			otherMapPose3D(otherMapPose);

	correspondencesRatio = 0;

	// Check the other map class:
	ASSERT_( otherMap->GetRuntimeClass() == CLASS_ID(CLandmarksMap) );
	const CLandmarksMap		*otherMap2 = static_cast<const CLandmarksMap*>( otherMap );
	std::vector<bool>	otherCorrespondences;

	// Coordinates change:
	auxMap.changeCoordinatesReference( otherMapPose3D, otherMap2 );


	//// Use the 3D matching method:
	computeMatchingWith3DLandmarks( otherMap2,
									correspondences,
									correspondencesRatio,
									otherCorrespondences );

	MRPT_END

}

/*---------------------------------------------------------------
				loadSiftFeaturesFromImageObservation
  ---------------------------------------------------------------*/
void  CLandmarksMap::loadSiftFeaturesFromImageObservation(
	const CObservationImage	&obs,
	const mrpt::vision::CFeatureExtraction::TOptions & feat_options
	)
{
	CImage								corImg;
	CPointPDFGaussian						landmark3DPositionPDF;
	CPoint3D								dir;	// Unitary, director vector
	float									d = insertionOptions.SIFTsLoadDistanceOfTheMean;
	float									width = insertionOptions.SIFTsLoadEllipsoidWidth;
	CMatrixDouble33							P,D;
	CLandmark								lm;

	vision::CFeatureExtraction				fExt;
	vision::CFeatureList						siftList;			//vision::TSIFTFeatureList			siftList;
	vision::CFeatureList::iterator			sift;				//vision::TSIFTFeatureList::iterator	sift;

	// Timestamp:
	lm.timestampLastSeen	= obs.timestamp;
	lm.seenTimesCount		= 1;

	// Remove distortion:
	corImg = obs.image; //obs.image.correctDistortion(obs.intrinsicParams,obs.distortionParams);

	// Extract SIFT features:
	fExt.options = feat_options;
	fExt.detectFeatures( corImg, siftList );	//vision::computeSiftFeatures(corImg, siftList );

	// Save them as 3D landmarks:
	landmarks.clear(); //resize( siftList.size() );

	for (sift=siftList.begin();sift!=siftList.end();sift++)
	{
		// Find the 3D position from the pixels
		//  coordinates and the camera intrinsic matrix:
		dir = vision::pixelTo3D( TPixelCoordf( (*sift)->x,(*sift)->y) , obs.cameraParams.intrinsicParams );	//dir = vision::pixelTo3D( sift->x,sift->y, obs.intrinsicParams );

		// Compute the mean and covariance of the landmark gaussian 3D position,
		//  from the unitary direction vector and a given distance:
		// --------------------------------------------------------------------------
		landmark3DPositionPDF.mean = dir;	// The mean is easy :-)
		landmark3DPositionPDF.mean*= d;

		// The covariance:
		P = math::generateAxisBaseFromDirection( dir.x(),dir.y(),dir.z() );

		// Diagonal matrix (with the "size" of the ellipsoid)
		D(0,0) = square( 0.5 * d );
		D(1,1) = square( width );
		D(2,2) = square( width );

		// Finally, compute the covariance!
		landmark3DPositionPDF.cov = CMatrixDouble33( P * D * P.transpose() );

		// Save into the landmarks vector:
		// --------------------------------------------
		lm.features.resize(1);
		lm.features[0] = (*sift);

		CPoint3D	Normal = landmark3DPositionPDF.mean;
		Normal *= -1/Normal.norm();

		lm.normal = Normal;

		lm.pose_mean	= landmark3DPositionPDF.mean;

		lm.pose_cov_11	= landmark3DPositionPDF.cov(0,0);
		lm.pose_cov_22	= landmark3DPositionPDF.cov(1,1);
		lm.pose_cov_33	= landmark3DPositionPDF.cov(2,2);
		lm.pose_cov_12	= landmark3DPositionPDF.cov(0,1);
		lm.pose_cov_13	= landmark3DPositionPDF.cov(0,2);
		lm.pose_cov_23	= landmark3DPositionPDF.cov(1,2);

		landmarks.push_back( lm );
	}

} // end loadSiftFeaturesFromImageObservation

/*---------------------------------------------------------------
				loadSiftFeaturesFromStereoImagesObservation
  ---------------------------------------------------------------*/
void  CLandmarksMap::loadSiftFeaturesFromStereoImageObservation(
	const CObservationStereoImages	&obs,
	mrpt::maps::CLandmark::TLandmarkID fID,
	const mrpt::vision::CFeatureExtraction::TOptions & feat_options
	)
{
	MRPT_START

	vision::CFeatureExtraction				fExt;
	vision::CFeatureList					leftSiftList, rightSiftList;
	vision::CMatchedFeatureList				matchesList;
	vision::TMatchingOptions				matchingOptions;
	vision::TStereoSystemParams				stereoParams;

	CLandmarksMap							landmarkMap;

	// Extract SIFT features:
	fExt.options = feat_options;   // OLD: fExt.options.SIFTOptions.implementation = vision::CFeatureExtraction::Hess;

	// Default: Hess implementation
	fExt.detectFeatures( obs.imageLeft, leftSiftList, fID, insertionOptions.SIFTs_numberOfKLTKeypoints );
	fExt.detectFeatures( obs.imageRight, rightSiftList, fID, insertionOptions.SIFTs_numberOfKLTKeypoints );

	matchingOptions.matching_method = vision::TMatchingOptions::mmDescriptorSIFT;
	matchingOptions.epipolar_TH		= insertionOptions.SIFTs_epipolar_TH;
	matchingOptions.EDD_RATIO		= insertionOptions.SiftCorrRatioThreshold;
	matchingOptions.maxEDD_TH		= insertionOptions.SiftEDDThreshold;
	vision::matchFeatures( leftSiftList, rightSiftList, matchesList, matchingOptions );

	if( insertionOptions.PLOT_IMAGES )
	{
		std::cerr << "Warning: insertionOptions.PLOT_IMAGES has no effect since MRPT 0.9.1\n";
	}

	//obs.imageLeft.saveToFile("LImage.jpg");
	//obs.imageRight.saveToFile("RImage.jpg");
	//FILE *fmt;
	//fmt = os::fopen( "matchesRBPF.txt", "at" );
	//os::fprintf( fmt, "%d\n", (unsigned int)matchesList.size() );
	//os::fclose(fmt);
	//matchesList.saveToTextFile("matches.txt");

	// Feature Projection to 3D
	// Parameters of the stereo rig

	stereoParams.K = obs.leftCamera.intrinsicParams;
	stereoParams.stdPixel	= insertionOptions.SIFTs_stdXY;
	stereoParams.stdDisp	= insertionOptions.SIFTs_stdDisparity;
	stereoParams.baseline	= obs.rightCameraPose.x();
	stereoParams.minZ		= 0.0f;
	stereoParams.maxZ		= insertionOptions.SIFTs_stereo_maxDepth;

	size_t numM = matchesList.size();
	vision::projectMatchedFeatures( matchesList, stereoParams, *this );

	// Add Timestamp and the "Seen-Times" counter
	TCustomSequenceLandmarks::iterator	ii;
	for( ii = landmarks.begin(); ii != landmarks.end(); ii++ )
	{
		(*ii).timestampLastSeen		= obs.timestamp;
		(*ii).seenTimesCount		= 1;
	}

	printf("%u (out of %u) corrs!\n",static_cast<unsigned>(landmarks.size()), static_cast<unsigned>(numM) );

	//CLandmarksMap::_maxMapID = fID;

	// Project landmarks according to the ref. camera pose:
	changeCoordinatesReference( obs.cameraPose );

	MRPT_END
}

/*---------------------------------------------------------------
				changeCoordinatesReference
  ---------------------------------------------------------------*/
void  CLandmarksMap::changeCoordinatesReference( const CPose3D &newOrg )
{
	TSequenceLandmarks::iterator		lm;

	CMatrixDouble44 HM;
	newOrg.getHomogeneousMatrix(HM);

	// Build the rotation only transformation:
	double	R11		= HM.get_unsafe(0,0);
	double	R12		= HM.get_unsafe(0,1);
	double	R13		= HM.get_unsafe(0,2);
	double	R21		= HM.get_unsafe(1,0);
	double	R22		= HM.get_unsafe(1,1);
	double	R23		= HM.get_unsafe(1,2);
	double	R31		= HM.get_unsafe(2,0);
	double	R32		= HM.get_unsafe(2,1);
	double	R33		= HM.get_unsafe(2,2);

	double	c11,c22,c33,c12,c13,c23;

	// Change the reference of each individual landmark:
	// ----------------------------------------------------
	for (lm=landmarks.begin();lm!=landmarks.end();lm++)
	{
		// Transform the pose mean & covariance:
		// ---------------------------------------------------------
		newOrg.composePoint(lm->pose_mean, lm->pose_mean);

		float	C11 = lm->pose_cov_11;
		float	C22 = lm->pose_cov_22;
		float	C33 = lm->pose_cov_33;
		float	C12 = lm->pose_cov_12;
		float	C13 = lm->pose_cov_13;
		float	C23 = lm->pose_cov_23;

		// The covariance:  cov = M * cov * (~M);
		c11 = R11*(C11* R11 + C12 *R12 + C13 *R13) + R12 *(C12 *R11 + C22 *R12 + C23 *R13) + R13 *(C13 *R11 + C23 *R12 + C33 *R13);
		c12 = (C11 *R11 + C12 *R12 + C13 *R13) *R21 + (C12 *R11 + C22 *R12 + C23 *R13) *R22 + (C13 *R11 + C23 *R12 + C33 *R13) *R23;
		c13 = (C11 *R11 + C12 *R12 + C13 *R13) *R31 + (C12 *R11 + C22 *R12 + C23 *R13) *R32 + (C13 *R11 + C23 *R12 + C33 *R13) *R33;
		c22 = R21 *(C11 *R21 + C12 *R22 + C13 *R23) + R22 *(C12 *R21 + C22 *R22 + C23 *R23) + R23 *(C13 *R21 + C23 *R22 + C33 *R23);
		c23 = (C11 *R21 + C12 *R22 + C13 *R23) *R31 + (C12 *R21 + C22 *R22 + C23 *R23) *R32 + (C13 *R21 + C23 *R22 + C33 *R23) *R33;
		c33 = R31 *(C11 *R31 + C12 *R32 + C13 *R33) + R32 *(C12 *R31 + C22 *R32 + C23 *R33) +  R33 *(C13 *R31 + C23 *R32 + C33 *R33);

		// save into the landmark:
		lm->pose_cov_11 = c11;
		lm->pose_cov_22 = c22;
		lm->pose_cov_33 = c33;
		lm->pose_cov_12 = c12;
		lm->pose_cov_13 = c13;
		lm->pose_cov_23 = c23;

		// Rotate the normal:         lm->normal = rot + lm->normal;
		// ---------------------------------------------------------
		float	Nx = lm->normal.x;
		float	Ny = lm->normal.y;
		float	Nz = lm->normal.z;

		lm->normal.x =  Nx * R11 + Ny * R12 + Nz * R13;
		lm->normal.y =  Nx * R21 + Ny * R22 + Nz * R23;
		lm->normal.z =  Nx * R31 + Ny * R32 + Nz * R33;
	}

	// For updating the KD-Tree
	landmarks.hasBeenModifiedAll();
}

/*---------------------------------------------------------------
				changeCoordinatesReference
  ---------------------------------------------------------------*/
void  CLandmarksMap::changeCoordinatesReference( const CPose3D &newOrg, const mrpt::maps::CLandmarksMap *otherMap )
{
	TSequenceLandmarks::const_iterator		lm;
	CLandmark							newLandmark;

	CMatrixDouble44 HM;
	newOrg.getHomogeneousMatrix(HM);

	// Build the rotation only transformation:
	double	R11		= HM.get_unsafe(0,0);
	double	R12		= HM.get_unsafe(0,1);
	double	R13		= HM.get_unsafe(0,2);
	double	R21		= HM.get_unsafe(1,0);
	double	R22		= HM.get_unsafe(1,1);
	double	R23		= HM.get_unsafe(1,2);
	double	R31		= HM.get_unsafe(2,0);
	double	R32		= HM.get_unsafe(2,1);
	double	R33		= HM.get_unsafe(2,2);

	double	c11,c22,c33,c12,c13,c23;

	landmarks.clear();

	// Change the reference of each individual landmark:
	// ----------------------------------------------------
	for (lm=otherMap->landmarks.begin();lm!=otherMap->landmarks.end();lm++)
	{
		// Transform the pose mean & covariance:
		// ---------------------------------------------------------
		// The mean:     mean = newReferenceBase + mean;
		newOrg.composePoint(lm->pose_mean, newLandmark.pose_mean);

		float	C11 = lm->pose_cov_11;
		float	C22 = lm->pose_cov_22;
		float	C33 = lm->pose_cov_33;
		float	C12 = lm->pose_cov_12;
		float	C13 = lm->pose_cov_13;
		float	C23 = lm->pose_cov_23;

		// The covariance:  cov = M * cov * (~M);
		c11 = R11*(C11* R11 + C12 *R12 + C13 *R13) + R12 *(C12 *R11 + C22 *R12 + C23 *R13) + R13 *(C13 *R11 + C23 *R12 + C33 *R13);
		c12 = (C11 *R11 + C12 *R12 + C13 *R13) *R21 + (C12 *R11 + C22 *R12 + C23 *R13) *R22 + (C13 *R11 + C23 *R12 + C33 *R13) *R23;
		c13 = (C11 *R11 + C12 *R12 + C13 *R13) *R31 + (C12 *R11 + C22 *R12 + C23 *R13) *R32 + (C13 *R11 + C23 *R12 + C33 *R13) *R33;
		c22 = R21 *(C11 *R21 + C12 *R22 + C13 *R23) + R22 *(C12 *R21 + C22 *R22 + C23 *R23) + R23 *(C13 *R21 + C23 *R22 + C33 *R23);
		c23 = (C11 *R21 + C12 *R22 + C13 *R23) *R31 + (C12 *R21 + C22 *R22 + C23 *R23) *R32 + (C13 *R21 + C23 *R22 + C33 *R23) *R33;
		c33 = R31 *(C11 *R31 + C12 *R32 + C13 *R33) + R32 *(C12 *R31 + C22 *R32 + C23 *R33) +  R33 *(C13 *R31 + C23 *R32 + C33 *R33);

		// save into the landmark:
		newLandmark.pose_cov_11 = c11;
		newLandmark.pose_cov_22 = c22;
		newLandmark.pose_cov_33 = c33;
		newLandmark.pose_cov_12 = c12;
		newLandmark.pose_cov_13 = c13;
		newLandmark.pose_cov_23 = c23;

		// Rotate the normal:         lm->normal = rot + lm->normal;
		// ---------------------------------------------------------
		float	Nx = lm->normal.x;
		float	Ny = lm->normal.y;
		float	Nz = lm->normal.z;

		newLandmark.normal.x =  Nx * R11 + Ny * R12 + Nz * R13;
		newLandmark.normal.y =  Nx * R21 + Ny * R22 + Nz * R23;
		newLandmark.normal.z =  Nx * R31 + Ny * R32 + Nz * R33;

		newLandmark.ID		 = lm->ID;

		newLandmark.features = lm->features;

		newLandmark.timestampLastSeen = lm->timestampLastSeen;
		newLandmark.seenTimesCount	  = lm->seenTimesCount;

		landmarks.push_back( newLandmark );
	}

}


/*---------------------------------------------------------------
						fuseWith
  ---------------------------------------------------------------*/
void  CLandmarksMap::fuseWith( CLandmarksMap &other, bool justInsertAllOfThem )
{
	MRPT_START

	//std::cout << "Entrando en fuseWith" << std::endl;

	std::vector<bool>					otherCorrs(other.size(), false);
	TMatchingPairList					corrs;
	TMatchingPairList::iterator			corrIt;
	float								corrsRatio;
	CLandmark							*thisLM, *otherLM;
	int									i,n;
	bool								verbose = true; //false;
	TTimeStamp							lastestTime=0;
	unsigned int						nRemoved = 0;

	if (!justInsertAllOfThem )
	{
		// 1) Compute matching between the global and the new map:
		// ---------------------------------------------------------
		computeMatchingWith3DLandmarks( &other,
										corrs,
										corrsRatio,
										otherCorrs );

		// 2) Fuse correspondences
		// ---------------------------------------------------------
		for (corrIt=corrs.begin();corrIt!=corrs.end();corrIt++)
		{
			thisLM = landmarks.get(corrIt->this_idx );
			otherLM = other.landmarks.get( corrIt->other_idx );

			// Fuse their poses:
			CPointPDFGaussian		P,P1,P2;

			thisLM->getPose(P1);
			otherLM->getPose(P2);

			P.bayesianFusion( P1,P2 );

			landmarks.isToBeModified(corrIt->this_idx );
			thisLM->setPose( P );

			// Update "seen" data:
			thisLM->seenTimesCount++;
			thisLM->timestampLastSeen = otherLM->timestampLastSeen;

			landmarks.hasBeenModified(corrIt->this_idx );

		} // end foreach corrs
	}

	// 3) Add new landmarks from the other map:
	// ---------------------------------------------------------
	n = other.size();
	for (i=0;i<n;i++)
	{
		// Find the lastest time.
		lastestTime = max( lastestTime, other.landmarks.get(i)->timestampLastSeen );

		if ( !otherCorrs[i] )
		{
			// No corrs: A NEW LANDMARK!
			landmarks.push_back( *other.landmarks.get(i) );
		}
	} // end foreach other landmark


	if (!justInsertAllOfThem )
	{
		// 4) Remove landmarks that have been not seen the required
		//      number of times:
		// ---------------------------------------------------------
		n = landmarks.size();
		for (i=n-1;i>=0;i--)
		{
			if (landmarks.get(i)->getType()!=featNotDefined)  // Occupancy features
			{
				TTimeStamp	t = lastestTime - landmarks.get(i)->timestampLastSeen;
				double		tt = (int64_t)t * 0.0000001;    // (int64_t) required by MSVC6
				if ( tt > fuseOptions.ellapsedTime &&
					landmarks.get(i)->seenTimesCount<fuseOptions.minTimesSeen )
				{
					landmarks.erase(i);
					nRemoved++;
				}
			}
		}
	}

	if (verbose)
	{
		printf("[CLandmarksMap::fuseWith] %u fused/ %u new/ %u removed -> %u total\n",
            static_cast<unsigned int>(corrs.size()),
            static_cast<unsigned int>(other.size()-corrs.size()),
            static_cast<unsigned int>(nRemoved),
            static_cast<unsigned int>(landmarks.size()) );
		FILE *f = os::fopen("Fused.txt","at");
		fprintf(f, "%u\t%u\t%u\t%u\n",
            static_cast<unsigned int>(corrs.size()),
            static_cast<unsigned int>(other.size()-corrs.size()),
            static_cast<unsigned int>(nRemoved),
            static_cast<unsigned int>(landmarks.size()) );
		os::fclose(f);
	}

	MRPT_END

}


/*---------------------------------------------------------------
						computeMatchingWith3DLandmarks
  ---------------------------------------------------------------*/
void  CLandmarksMap::computeMatchingWith3DLandmarks(
		const mrpt::maps::CLandmarksMap				*anotherMap,
		TMatchingPairList						&correspondences,
		float									&correspondencesRatio,
		std::vector<bool>						&otherCorrespondences) const
{
	MRPT_START

	TSequenceLandmarks::const_iterator		thisIt,otherIt;
	unsigned int							nThis,nOther;
	int										maxIdx;
	float									desc;
	unsigned int							i,n,j,k;
	TMatchingPair							match;
	double									lik_dist, lik_desc, lik, maxLik;
	//double									maxLikDist = -1, maxLikDesc = -1;
	CPointPDFGaussian						pointPDF_k, pointPDF_j;
	std::vector<bool>						thisLandmarkAssigned;
	double									K_desc = 0.0;
	double									K_dist = 0.0;

//	FILE									*f = os::fopen( "flik.txt", "wt" );

	// Get the number of landmarks:
	nThis = landmarks.size();
	nOther = anotherMap->landmarks.size();

	// Initially no LM has a correspondence:
	thisLandmarkAssigned.resize( nThis, false );

	// Initially, set all landmarks without correspondences:
	correspondences.clear();
	otherCorrespondences.clear();
	otherCorrespondences.resize( nOther, false );
	correspondencesRatio = 0;

	// Method selection:
	// 0. Our Method.
	// 1. Sim, Elinas, Griffin, Little.

	switch ( insertionOptions.SIFTMatching3DMethod )
	{
	case 0:
		// Our method: Filter out by the likelihood of the 3D position and compute the likelihood of the Euclidean descriptor distance

		// "Constants" for the distance computation
		K_desc = - 0.5 / square(likelihoodOptions.SIFTs_sigma_descriptor_dist);
		K_dist = - 0.5 / square(likelihoodOptions.SIFTs_mahaDist_std);

		//CDynamicGrid<vector_int>		*gridLandmarks = landmarks.getGrid();
		//vector_int						closeLandmarksList;

		for (k=0,otherIt=anotherMap->landmarks.begin();otherIt!=anotherMap->landmarks.end();otherIt++,k++)
		{
			// Load into "pointPDF_k" the PDF of landmark "otherIt":
			otherIt->getPose( pointPDF_k );

			if (otherIt->getType()==featSIFT)
			{

				//minDist = minDist2 = 1e10f;
				maxLik  = -1;
				maxIdx  = -1;

				/*
				// Get the list of close landmarks:
				// ---------------------------------------------
				int	cx0 = gridLandmarks->x2idx( otherIt->pose_mean.x, otherIt->pose_mean.y );
				int	cx0 = gridLandmarks->x2idx( otherIt->pose_mean.x, otherIt->pose_mean.y );

				closeLandmarksList.clear();
				closeLandmarksList.reserve(300);
				...
				*/

				for (j=0,thisIt=landmarks.begin();thisIt!=landmarks.end();thisIt++,j++)
				{
					if (thisIt->getType()==featSIFT &&
						thisIt->features.size() == otherIt->features.size() &&
						!thisIt->features.empty() &&
						thisIt->features[0].present() && otherIt->features[0].present() &&
						thisIt->features[0]->descriptors.SIFT.size()==otherIt->features[0]->descriptors.SIFT.size()
						)
					{
						// Compute "coincidence probability":
						// --------------------------------------
						// Load into "pointPDF_j" the PDF of landmark "otherIt":
						thisIt->getPose( pointPDF_j );

						// Compute lik:
						//lik_dist = pointPDF_k.productIntegralNormalizedWith( &pointPDF_j );
						CMatrixDouble	dij(1,3), Cij(3,3), Cij_1;
						double		distMahaFlik2;

						// Distance between means:
						dij(0,0) = pointPDF_k.mean.x() - pointPDF_j.mean.x();
						dij(0,1) = pointPDF_k.mean.y() - pointPDF_j.mean.y();
						dij(0,2) = pointPDF_k.mean.z() - pointPDF_j.mean.z();

						// Equivalent covariance from "i" to "j":
						Cij = CMatrixDouble( pointPDF_k.cov + pointPDF_j.cov );
						Cij_1 = Cij.inv();

						distMahaFlik2 = dij.multiply_HCHt_scalar(Cij_1); //( dij * Cij_1 * (~dij) )(0,0);

						lik_dist = exp( K_dist * distMahaFlik2 );		// Likelihood regarding the spatial distance

						if( lik_dist > 1e-2 )
						{
							// Compute distance between descriptors:
							// --------------------------------------

							// MODIFICATION 19-SEPT-2007
							// ONLY COMPUTE THE EUCLIDEAN DISTANCE BETWEEN DESCRIPTORS IF IT HAS NOT BEEN COMPUTED BEFORE
							// Make the pair of points
							std::pair<mrpt::maps::CLandmark::TLandmarkID, mrpt::maps::CLandmark::TLandmarkID> mPair( thisIt->ID, otherIt->ID );

							if( CLandmarksMap::_mEDD[ mPair ] == 0 )
							{
								n = otherIt->features[0]->descriptors.SIFT.size();
								desc = 0;
								for (i=0;i<n;i++)
									desc += square( otherIt->features[0]->descriptors.SIFT[i] - thisIt->features[0]->descriptors.SIFT[i] );

								CLandmarksMap::_mEDD[ mPair ] = desc;
								//std::cout << "[fuseWith] - Nueva entrada! - (LIK): " << exp( K_desc * desc ) << " -> " << "(" << mPair.first << "," << mPair.second << ")" << std::endl;

							} // end if
							else
							{
								desc = CLandmarksMap::_mEDD[ mPair ];
								//std::cout << "[fuseWith] - Ya esta calculado!: " << "(" << mPair.first << "," << mPair.second << ")"  << ": " << desc << std::endl;
							}

							lik_desc = exp( K_desc * desc );			// Likelihood regarding the descriptor
						}
						else
						{
							lik_desc = 1e-3;
						}

						// Likelihood of the correspondence
						// --------------------------------------
						lik = lik_dist*lik_desc;

//						os::fprintf( f, "%i\t%i\t%f\t%f\t%f\n", k, j, lik_desc, lik_dist, lik );

						if( lik > maxLik )
						{
//							maxLikDist = lik_dist;
//							maxLikDesc = lik_desc;
							maxLik = lik;
							maxIdx = j;
						}

					} // end of this landmark is SIFT

				} // End of for each "this", j
				//os::fprintf(f, "%i\t%i\t%f\t%f\t%f\n", maxIdx, k, maxLikDist, maxLikDesc, maxLik);

				// Is it a correspondence?
				if ( maxLik > insertionOptions.SiftLikelihoodThreshold )
				{
					// TODO: Solve in a better way the multiple correspondences case!!!
					// ****************************************************************
					// If a previous correspondence for this LM was found, discard this one!
					if ( !thisLandmarkAssigned[ maxIdx ] )
					{
						thisLandmarkAssigned[ maxIdx ] = true;

						// OK: A correspondence found!!
						otherCorrespondences[ k ] = true;

						match.this_idx  = maxIdx;
						match.this_x	= landmarks.get(maxIdx)->pose_mean.x;
						match.this_y	= landmarks.get(maxIdx)->pose_mean.y;
						match.this_z	= landmarks.get(maxIdx)->pose_mean.z;

						match.other_idx = k;
						match.other_x	= anotherMap->landmarks.get(k)->pose_mean.x;
						match.other_y	= anotherMap->landmarks.get(k)->pose_mean.y;
						match.other_z	= anotherMap->landmarks.get(k)->pose_mean.z;

						correspondences.push_back( match );
					}
				}

			} // end of "otherIt" is SIFT

		} // end of other it., k

		// Compute the corrs ratio:
		correspondencesRatio = correspondences.size() / static_cast<float>(nOther);
//		os::fclose(f);

		break;

	case 1:

		// IMPLEMENTATION OF THE METHOD DESCRIBED IN [VISION-BASED SLAM USING THE RBPF][SIM, ELINAS, GRIFFIN, LITTLE]
		// 1. Compute Euclidean descriptor distance (EDD).
		// 2. Matching based on a Threshold.
		// 3. Compute likelihood based only on the position of the 3D landmarks.

		// 1.- COMPUTE EDD

		ASSERT_( !anotherMap->landmarks.begin()->features.empty() )
		ASSERT_( !landmarks.begin()->features.empty() )

		unsigned int	dLen = anotherMap->landmarks.begin()->features[0]->descriptors.SIFT.size();
		for (k = 0, otherIt = anotherMap->landmarks.begin(); otherIt != anotherMap->landmarks.end(); otherIt++, k++)
		{
			double			mEDD = -1.0;
			unsigned int	mEDDidx = 0;
			for (j = 0, thisIt = landmarks.begin(); thisIt != landmarks.end(); thisIt++, j++)
			{
				// Compute EDD
				double EDD = 0.0;
				for ( i = 0; i < dLen; i++ )
					EDD += square( otherIt->features[0]->descriptors.SIFT[i] - thisIt->features[0]->descriptors.SIFT[i] );

				EDD = sqrt(EDD);

				if ( EDD > mEDD )
				{
					mEDD	= EDD;
					mEDDidx = j;
				} // end if
			} // end for j

			if ( mEDD > insertionOptions.SiftEDDThreshold )
			{
				// There is a correspondence
				if ( !thisLandmarkAssigned[ mEDDidx ] ) // If there is not multiple correspondence
				{

					thisLandmarkAssigned[ mEDDidx ] = true;

					// OK: A correspondence found!!
					otherCorrespondences[k] = true;

					otherIt->getPose( pointPDF_k );
					thisIt->getPose( pointPDF_j );

					match.this_idx	= j;
					match.this_x	= landmarks.get(mEDDidx)->pose_mean.x;
					match.this_y	= landmarks.get(mEDDidx)->pose_mean.y;
					match.this_z	= landmarks.get(mEDDidx)->pose_mean.z;

					match.other_idx = k;
					match.other_x	= anotherMap->landmarks.get(k)->pose_mean.x;
					match.other_y	= anotherMap->landmarks.get(k)->pose_mean.y;
					match.other_z	= anotherMap->landmarks.get(k)->pose_mean.z;

					correspondences.push_back( match );

				} // end if multiple correspondence

			} // end if mEDD

		} // end for k

		correspondencesRatio = correspondences.size() / static_cast<float>(nOther);

		break;

	} // end switch


	MRPT_END

}

/*---------------------------------------------------------------
						saveToTextFile
  ---------------------------------------------------------------*/
bool  CLandmarksMap::saveToTextFile(std::string file)
{
	MRPT_START

	FILE	*f= os::fopen(file.c_str(),"wt");
	if (!f) return false;

	//os::fprintf(f,"%% Map of landmarks - file dumped by mrpt::maps::CLandmarksMap\n");
	//os::fprintf(f,"%%  Columns are: X Y Z TYPE(TFeatureType) TIMES_SEEN TIME_OF_LAST_OBSERVATION [SIFT DESCRIPTOR] ID\n");
	//os::fprintf(f,"%% -----------------------------------------------------------------------------------------------------\n");

	for (TSequenceLandmarks::iterator	it=landmarks.begin();it!=landmarks.end();++it)
	{
		os::fprintf(f,"%10f %10f %10f %4i %4u %10f",
			it->pose_mean.x,
			it->pose_mean.y,
			it->pose_mean.z,
			static_cast<int>(it->getType()),
			it->seenTimesCount,
			it->timestampLastSeen==INVALID_TIMESTAMP ? 0: mrpt::system::extractDayTimeFromTimestamp(it->timestampLastSeen )
			);

		if (it->getType() == featSIFT)
		{
			ASSERT_( !it->features.empty() && it->features[0].present() )

			for (unsigned int i=0;i<it->features[0]->descriptors.SIFT.size();i++)
				os::fprintf(f," %u ",it->features[0]->descriptors.SIFT[i]);
		}
		os::fprintf(f," %i ",(int)it->ID);

		os::fprintf(f,"\n");
	}

	os::fclose(f);

	return true;

	MRPT_END

}

/*---------------------------------------------------------------
						saveToMATLABScript3D
  ---------------------------------------------------------------*/
bool  CLandmarksMap::saveToMATLABScript3D(
	std::string		file,
	const char		*style,
	float			confInterval )const
{
	FILE	*f= os::fopen(file.c_str(),"wt");
	if (!f) return false;

	// Header:
	os::fprintf(f,"%%-------------------------------------------------------\n");
	os::fprintf(f,"%% File automatically generated using the MRPT method:\n");
	os::fprintf(f,"%%   'CLandmarksMap::saveToMATLABScript3D'\n");
	os::fprintf(f,"%%\n");
	os::fprintf(f,"%%                        ~ MRPT ~\n");
	os::fprintf(f,"%%  Jose Luis Blanco Claraco, University of Malaga @ 2006\n");
	os::fprintf(f,"%%  http://www.isa.uma.es/ \n");
	os::fprintf(f,"%%-------------------------------------------------------\n\n");

	// Main code:
	os::fprintf(f,"hold on;\n\n");

	for (TSequenceLandmarks::const_iterator	it=landmarks.begin();it!=landmarks.end();++it)
	{
		os::fprintf(f,"m=[%.4f %.4f %.4f];",it->pose_mean.x, it->pose_mean.y, it->pose_mean.z );
		os::fprintf(f,"c=[%.8f %.8f %.8f;%.8f %.8f %.8f;%.8f %.8f %.8f]; ",
			it->pose_cov_11,it->pose_cov_12,it->pose_cov_13,
			it->pose_cov_12,it->pose_cov_22,it->pose_cov_23,
			it->pose_cov_13,it->pose_cov_23,it->pose_cov_33 );

		os::fprintf(f,"error_ellipse(c,m,'conf',%f,'style','%s','numPointsEllipse',10);\n",confInterval,style);
	}

	os::fprintf(f,"axis equal;grid on;xlabel('x'); ylabel('y'); zlabel('z');");

	os::fclose(f);
	return true;
}
/**/

/*---------------------------------------------------------------
						saveToMATLABScript2D
  ---------------------------------------------------------------*/
bool  CLandmarksMap::saveToMATLABScript2D(
	std::string		file,
	const char			*style,
	float			stdCount  )
{
	FILE	*f= os::fopen(file.c_str(),"wt");
	if (!f) return false;

	const int						ELLIPSE_POINTS	= 30;
	std::vector<float>				X,Y,COS,SIN;
	std::vector<float>::iterator	x,y,Cos,Sin;
	double							ang;
	CMatrixD						cov(2,2),eigVal,eigVec,M;

	X.resize(ELLIPSE_POINTS);
	Y.resize(ELLIPSE_POINTS);
	COS.resize(ELLIPSE_POINTS);
	SIN.resize(ELLIPSE_POINTS);


	// Fill the angles:
	for (Cos=COS.begin(),Sin=SIN.begin(),ang=0;Cos!=COS.end();Cos++,Sin++, ang+= (M_2PI/(ELLIPSE_POINTS-1)) )
	{
		*Cos = cos(ang);
		*Sin = sin(ang);
	}

	// Header:
	os::fprintf(f,"%%-------------------------------------------------------\n");
	os::fprintf(f,"%% File automatically generated using the MRPT method:\n");
	os::fprintf(f,"%%   'CLandmarksMap::saveToMATLABScript2D'\n");
	os::fprintf(f,"%%\n");
	os::fprintf(f,"%%                        ~ MRPT ~\n");
	os::fprintf(f,"%%  Jose Luis Blanco Claraco, University of Malaga @ 2006\n");
	os::fprintf(f,"%%  http://www.isa.uma.es/ \n");
	os::fprintf(f,"%%-------------------------------------------------------\n\n");

	// Main code:
	os::fprintf(f,"hold on;\n\n");

	for (TSequenceLandmarks::iterator	it=landmarks.begin();it!=landmarks.end();++it)
	{
		// Compute the eigen-vectors & values:
		cov(0,0) = it->pose_cov_11;
		cov(1,1) = it->pose_cov_22;
		cov(0,1) = cov(1,0) = it->pose_cov_12;

		cov.eigenVectors(eigVec,eigVal);
		eigVal = eigVal.array().sqrt().matrix();
		M = eigVal * eigVec.transpose();

		// Compute the points of the ellipsoid:
		// ----------------------------------------------
		for (x=X.begin(), y=Y.begin(), Cos=COS.begin(),Sin=SIN.begin(); x!=X.end(); x++,y++,Cos++,Sin++)
		{
			*x = ( it->pose_mean.x + stdCount * (*Cos * M(0,0) + *Sin * M(1,0)) );
			*y = ( it->pose_mean.y + stdCount * (*Cos * M(0,1) + *Sin * M(1,1)) );
		}

		// Save the code to plot the ellipsoid:
		// ----------------------------------------------
		os::fprintf(f,"plot([ ");
		for (x=X.begin();x!=X.end();x++)
		{
			os::fprintf(f,"%.4f",*x);
			if (x!=(X.end()-1))	os::fprintf(f,",");
		}
		os::fprintf(f,"],[ ");
		for (y=Y.begin();y!=Y.end();y++)
		{
			os::fprintf(f,"%.4f",*y);
			if (y!=(Y.end()-1))	os::fprintf(f,",");
		}

		os::fprintf(f,"],'%s');\n",style);

		//os::fprintf(f,"error_ellipse(c,m,'conf',0.99,'style','%s','numPointsEllipse',10);\n",style);
	}

	os::fprintf(f,"\naxis equal;\n");
	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
						loadOccupancyFeaturesFrom2DRangeScan
  ---------------------------------------------------------------*/
void  CLandmarksMap::loadOccupancyFeaturesFrom2DRangeScan(
		const CObservation2DRangeScan	&obs,
		const CPose3D					*robotPose,
		unsigned int				downSampleFactor )
{
	unsigned int		i, n = obs.scan.size();
	double				Th,dTh;				// angle of the ray
	double				J11,J12,J21,J22;	// The jacobian elements.
	double				d;
	CPose3D				sensorGlobalPose;

	// Empty the map:
	this->clear();

	// Sensor pose in 3D:
	if (!robotPose)
			sensorGlobalPose = obs.sensorPose;
	else	sensorGlobalPose = *robotPose + obs.sensorPose;

	// Starting direction:
	if (obs.rightToLeft)
	{
		Th = - 0.5 * obs.aperture;
		dTh  = obs.aperture / n;
	}
	else
	{
		Th = + 0.5 * obs.aperture;
		dTh  = - obs.aperture / n;
	}

	// Measurement uncertainties:
	double				var_d  = square( 0.005 ); //square(obs.stdError);
	double				var_th = square( dTh / 10.0 );

	// For each range:
	for (i=0;i<n;i+= downSampleFactor,Th += downSampleFactor*dTh )
	{
		// If it is a valid ray:
		if (obs.validRange[i])
		{
			CLandmark		newLandmark;

			// Timestamp:
			newLandmark.timestampLastSeen	= obs.timestamp;
			newLandmark.seenTimesCount		= 1;

			newLandmark.createOneFeature();
			newLandmark.features[0]->type = featNotDefined;

			d = obs.scan[i];

			// Compute the landmark in 2D:
			// -----------------------------------------------
			// Descriptor:
			newLandmark.features[0]->orientation = Th;
			newLandmark.features[0]->scale = d;

			// Mean:
			newLandmark.pose_mean.x = (cos(Th) * d);
			newLandmark.pose_mean.y = (sin(Th) * d);
			newLandmark.pose_mean.z = 0;

			// Normal direction:
			newLandmark.normal = newLandmark.pose_mean;
			newLandmark.normal *= -1.0f / newLandmark.pose_mean.norm();

			// Jacobian:
			J11 = - d * sin(Th);  J12 = cos(Th);
			J21 =   d * cos(Th);  J22 = sin(Th);

			// Covariance matrix:
			newLandmark.pose_cov_11 = ( J11*J11*var_th + J12*J12*var_d );
			newLandmark.pose_cov_12 = ( J11*J21*var_th + J12*J22*var_d );
			newLandmark.pose_cov_22 = ( J21*J21*var_th + J22*J22*var_d );
			newLandmark.pose_cov_33 = ( square(0.005) ); //var_d;
			newLandmark.pose_cov_13 = newLandmark.pose_cov_23 =  0;

			// Append it:
			// -----------------------------------------------
			landmarks.push_back( newLandmark );

		} // end of valid ray.


	} // end for n

	// Take landmarks to 3D according to the robot & sensor 3D poses:
	// -----------------------------------------------
	changeCoordinatesReference( sensorGlobalPose );


}

/*---------------------------------------------------------------
				METHOD DESCRIBED IN PAPER
				-------------------------

		ICRA 2007,....

  ---------------------------------------------------------------*/
double  CLandmarksMap::computeLikelihood_RSLC_2007( const CLandmarksMap  *s, const CPose2D &sensorPose )
{
	MRPT_UNUSED_PARAM(sensorPose);
	MRPT_START

	double								lik = 1.0;
	TSequenceLandmarks::const_iterator		itOther;
	CLandmark							*lm; //*itClosest;
	double								corr;
	double								PrNoCorr;
	CPointPDFGaussian					poseThis,poseOther;
	//double								STD_THETA = DEG2RAD(0.15);
	//double								STD_DIST = 0.5f;
	double								nFoundCorrs = 0;
	vector_int							*corrs;
	unsigned int						cx,cy,cx_1,cx_2,cy_1,cy_2;

	//	s->saveToMATLABScript2D(std::string("ver_sensed.m")); 	saveToMATLABScript2D(std::string("ver_ref.m"),"r");

	CDynamicGrid<vector_int>*	grid = landmarks.getGrid();
	//grid->saveToTextFile( "debug_grid.txt" );

	// For each landmark in the observations:
	for (itOther = s->landmarks.begin(); itOther!=s->landmarks.end(); itOther++ )
	{
		itOther->getPose( poseOther );

		cx =
		cy = grid->y2idx( itOther->pose_mean.y );

		cx_1 = max(0, grid->x2idx( itOther->pose_mean.x - 0.10f ) );
		cx_2 = min(static_cast<int>(grid->getSizeX())-1, grid->x2idx( itOther->pose_mean.x + 0.10f ));
		cy_1 = max(0, grid->y2idx( itOther->pose_mean.y - 0.10f ) );
		cy_2 = min( static_cast<int>(grid->getSizeY())-1, grid->y2idx( itOther->pose_mean.y + 0.10f ) );

		// The likelihood of no correspondence starts at "1" and will be modified next:
		PrNoCorr = 1;

		// For each landmark in this map: Compute its correspondence likelihood
		//   and conditioned observation likelihood:
		//itClosest = NULL;

		// Look at landmarks in sourronding cells only:
		for (cx=cx_1;cx<=cx_2;cx++)
	    for (cy=cy_1;cy<=cy_2;cy++)
		{
			corrs = grid->cellByIndex( cx, cy );
			ASSERT_( corrs!=NULL );
			if (!corrs->empty())
			for (vector_int::iterator	it=corrs->begin();it!=corrs->end();++it)
			{
			lm = landmarks.get(*it);

			// Compute the "correspondence" in the range [0,1]:
			// -------------------------------------------------------------

			// Shortcut:
			if ( fabs( lm->pose_mean.x - itOther->pose_mean.x ) > 0.15f ||
					fabs( lm->pose_mean.y - itOther->pose_mean.y ) > 0.15f )
			{
				// Absolutely no correspondence, not need to compute it:
				corr = 0;
			}
			else
			{
				lm->getPose( poseThis );
				corr =  poseOther.productIntegralNormalizedWith2D( poseThis );
			}

			// The likelihood of no corresp. is proportional to the product of all "1-CORRij":
			// -------------------------------------------------------------
			PrNoCorr *= 1 - corr;

		} // end of each landmark in this map.
		}

		// Only consider this "point" if it has some real chance to has a correspondence:
		nFoundCorrs += 1 - PrNoCorr;

		/**** DEBUG **** /
		{
			//FILE	*f=os::fopen("debug_corrs.txt","wt");
			//for (Cij=v_Cij.begin(),pZj=v_pZj.begin(); pZj!=v_pZj.end(); Cij++,pZj++)
			//{
			//	os::fprintf(f,"%e %e\n", *Cij, *pZj);
			//}

			//os::fprintf(f,"\n INDIV LIK=%e\n lik=%e\n closestObstacleInLine=%e\n measured range=%e\n",indivLik,lik,closestObstacleInLine, itOther->descriptors.SIFT[1]);
			//os::fprintf(f," closestObstacleDirection=%e\n",closestObstacleDirection);
			//os::fclose(f);

			printf("\n lik=%e\n closestObstacleInLine=%e\n measured range=%e\n",lik,closestObstacleInLine, itOther->descriptors.SIFT[1]);
			if (itClosest)
					printf(" closest=(%.03f,%.03f)\n", itClosest->pose_mean.x, itClosest->pose_mean.y);
			else	printf(" closest=NULL\n");

			printf(" P(no corrs)=%e\n",	PrNoCorr );
			mrpt::system::pause();
		}
		/ ***************/

		lik *= 1 - PrNoCorr;

	} // enf for each landmark in the other map.

	lik = nFoundCorrs / static_cast<double>( s->size() );

	lik = log(lik);

	MRPT_CHECK_NORMAL_NUMBER( lik );
	return lik;

	MRPT_END
}

/*---------------------------------------------------------------

					TCustomSequenceLandmarks

  ---------------------------------------------------------------*/
CLandmarksMap::TCustomSequenceLandmarks::TCustomSequenceLandmarks() :
	m_landmarks(),
	m_grid( -10.0f,10.0f,-10.0f,10.f,0.20f ),
	m_largestDistanceFromOrigin(),
	m_largestDistanceFromOriginIsUpdated(false)
{
}

void  CLandmarksMap::TCustomSequenceLandmarks::clear()
{
	m_landmarks.clear();

	// Erase the grid:
	m_grid.clear();

	m_largestDistanceFromOriginIsUpdated = false;
}

void 	CLandmarksMap::TCustomSequenceLandmarks::push_back( const CLandmark	&l)
{
	// Resize grid if necesary:
	vector_int		dummyEmpty;

	m_grid.resize(	min( m_grid.getXMin(),l.pose_mean.x-0.1 ),
					max( m_grid.getXMax(),l.pose_mean.x+0.1 ),
					min( m_grid.getYMin(),l.pose_mean.y-0.1 ),
					max( m_grid.getYMax(),l.pose_mean.y+0.1 ),
					dummyEmpty );

	m_landmarks.push_back( l );

	// Add to the grid:
	vector_int	*cell = m_grid.cellByPos(l.pose_mean.x,l.pose_mean.y);
	ASSERT_(cell);
	cell->push_back( m_landmarks.size()-1 );

	m_largestDistanceFromOriginIsUpdated = false;
}

CLandmark* 	CLandmarksMap::TCustomSequenceLandmarks::get(unsigned int indx)
{
    return &m_landmarks[indx];
}

const CLandmark* 	CLandmarksMap::TCustomSequenceLandmarks::get(unsigned int indx) const
{
    return &m_landmarks[indx];
}

void 	CLandmarksMap::TCustomSequenceLandmarks::isToBeModified(unsigned int indx)
{
	vector_int	*cell = m_grid.cellByPos(m_landmarks[indx].pose_mean.x,m_landmarks[indx].pose_mean.y);

	vector_int::iterator	it;
	for (it=cell->begin();it!=cell->end();it++)
	{
		if (*it==static_cast<int>(indx))
		{
			cell->erase(it);
			return;
		}
	}

	m_largestDistanceFromOriginIsUpdated = false;
}

void 	CLandmarksMap::TCustomSequenceLandmarks::erase(unsigned int indx)
{
	m_landmarks.erase( m_landmarks.begin() + indx );
	m_largestDistanceFromOriginIsUpdated = false;
}

void 	CLandmarksMap::TCustomSequenceLandmarks::hasBeenModified(unsigned int indx)
{
	vector_int		dummyEmpty;

	// Resize grid if necesary:
	m_grid.resize(	min( m_grid.getXMin(),m_landmarks[indx].pose_mean.x ),
					max( m_grid.getXMax(),m_landmarks[indx].pose_mean.x ),
					min( m_grid.getYMin(),m_landmarks[indx].pose_mean.y ),
					max( m_grid.getYMax(),m_landmarks[indx].pose_mean.y ),
					dummyEmpty );

	// Add to the grid:
	vector_int	*cell = m_grid.cellByPos(m_landmarks[indx].pose_mean.x,m_landmarks[indx].pose_mean.y);
	cell->push_back( indx );
	m_largestDistanceFromOriginIsUpdated = false;
}

void 	CLandmarksMap::TCustomSequenceLandmarks::hasBeenModifiedAll()
{
	MRPT_START

	TSequenceLandmarks::iterator	it;
	unsigned int					idx;
	double    min_x=-10.0, max_x=10.0;
	double    min_y=-10.0, max_y=10.0;
	vector_int						dummyEmpty;

	// Clear cells:
	m_grid.clear();

	// Resize the grid to the outer limits of landmarks:
	for (idx=0,it=m_landmarks.begin();it!=m_landmarks.end();idx++,it++)
	{
		min_x = min(min_x, it->pose_mean.x);
		max_x = max(max_x, it->pose_mean.x);
		min_y = min(min_y, it->pose_mean.y);
		max_y = max(max_y, it->pose_mean.y);
	}
	m_grid.resize(min_x,max_x,min_y,max_y,dummyEmpty );

	// Add landmarks to cells:
	for (idx=0,it=m_landmarks.begin();it!=m_landmarks.end();idx++,it++)
	{
		vector_int	*cell = m_grid.cellByPos( it->pose_mean.x,it->pose_mean.y);
		cell->push_back( idx );
	}

	m_largestDistanceFromOriginIsUpdated = false;
	MRPT_END
}

/*---------------------------------------------------------------
						getLargestDistanceFromOrigin
---------------------------------------------------------------*/
float  CLandmarksMap::TCustomSequenceLandmarks::getLargestDistanceFromOrigin() const
{
	// Updated?
	if (!m_largestDistanceFromOriginIsUpdated)
	{
		// NO: Update it:
		float	maxDistSq = 0, d;
		for (const_iterator it = begin(); it!=end(); ++it)
		{
			d = square(it->pose_mean.x)+square(it->pose_mean.y)+square(it->pose_mean.z);
			maxDistSq = max( d, maxDistSq );
		}


		m_largestDistanceFromOrigin = sqrt( maxDistSq );
		m_largestDistanceFromOriginIsUpdated = true;
	}
	return m_largestDistanceFromOrigin;
}


/*---------------------------------------------------------------
					computeLikelihood_SIFT_LandmarkMap
  ---------------------------------------------------------------*/
double	 CLandmarksMap::computeLikelihood_SIFT_LandmarkMap( CLandmarksMap		*theMap,
															TMatchingPairList	*correspondences,
															std::vector<bool>	*otherCorrespondences )
//double	 CLandmarksMap::computeLikelihood_SIFT_LandmarkMap( CLandmarksMap *theMap )
{
	//std::cout << "Entrando en computeLikelihood -- MAPA_OTHER: " << theMap->landmarks.size() << std::endl;
	//std::cout << "Entrando en computeLikelihood -- MAPA_THIS: " << this->landmarks.size() << std::endl;
	double						lik = 0;			// For 'traditional'
	double						lik_i;
	unsigned long				distDesc;
	double						likByDist,likByDesc;

	//FILE						*f;

	//int							nFeaturesThis = this->size();
	//int							nFeaturesAuxMap = theMap->size();
	TSequenceLandmarks::iterator			lm1,lm2;
	std::vector<unsigned char>::iterator	it1,it2;

	// Fast look-up, precomputed, variables:
	//double						sigmaDist3 = 4.0 * likelihoodOptions.SIFTs_sigma_euclidean_dist;

	/**** FAMD ****/
	double						K_dist = - 0.5 / square( likelihoodOptions.SIFTs_mahaDist_std );
	//double						K_dist = - 0.5;
	/**************/
	double						K_desc = - 0.5 / square( likelihoodOptions.SIFTs_sigma_descriptor_dist );

	unsigned int				idx1,idx2;
	CPointPDFGaussian			lm1_pose, lm2_pose;
	CMatrixD					dij(1,3), Cij(3,3), Cij_1;
	double						distMahaFlik2;
	int							decimation = likelihoodOptions.SIFTs_decimation;

	// USE INSERTOPTIONS METHOD
	switch ( insertionOptions.SIFTLikelihoodMethod )
	{
	case 0: // Our method

		//lik = 1e-9;		// For consensus
		lik = 1.0;			// For traditional

		for (idx1=0,lm1 = theMap->landmarks.begin(); lm1 < theMap->landmarks.end(); lm1+=decimation,idx1+=decimation) // Other theMap LM1
		{
			if (lm1->getType() == featSIFT )
			{
				// Get the pose of lm1 as an object:
				lm1->getPose( lm1_pose );

				lik_i = 0;	// Counter

				for (idx2=0,lm2=landmarks.begin();lm2!=landmarks.end();lm2++,idx2++)							// This theMap LM2
				{
					if (lm2->getType() == featSIFT )
					{
						// Get the pose of lm2 as an object:
						lm2->getPose( lm2_pose );

						// Compute the likelihood according to mahalanobis distance:
						// Distance between means:
						dij(0,0) = lm1->pose_mean.x - lm2->pose_mean.x;
						dij(0,1) = lm1->pose_mean.y - lm2->pose_mean.y;
						dij(0,2) = lm1->pose_mean.z - lm2->pose_mean.z;

						//std::cout << "ED POSICION: " << sqrt( dij(0,0)*dij(0,0) + dij(0,1)*dij(0,1) + dij(0,2)*dij(0,2) ) << std::endl;
						// Equivalent covariance from "i" to "j":
						Cij = CMatrixDouble(lm1_pose.cov + lm2_pose.cov );
						Cij_1 = Cij.inv();

						distMahaFlik2 =  dij.multiply_HCHt_scalar(Cij_1); //( dij * Cij_1 * (~dij) )(0,0);

						likByDist = exp( K_dist * distMahaFlik2 );
						//std::cout << likByDist << std::endl;

						if ( likByDist > 1e-2 )
						{
							// If the EUCLIDEAN distance is not too large, we compute the Descriptor distance
							// Compute Likelihood by descriptor
							// Compute distance between descriptors

							// IF the EDD has been already computed, we skip this step!
							std::pair<mrpt::maps::CLandmark::TLandmarkID, mrpt::maps::CLandmark::TLandmarkID> mPair( lm2->ID, lm1->ID );
							//std::cout << "Par: (" << lm2->ID << "," << lm1->ID << ") -> ";

							if( CLandmarksMap::_mEDD[ mPair ] == 0 )
							{
								distDesc = 0;
								ASSERT_( !lm1->features.empty() && !lm2->features.empty() )
								ASSERT_( lm1->features[0].present() && lm2->features[0].present() )
								ASSERT_( lm1->features[0]->descriptors.SIFT.size() == lm2->features[0]->descriptors.SIFT.size() )

								for ( it1 = lm1->features[0]->descriptors.SIFT.begin(), it2 = lm2->features[0]->descriptors.SIFT.begin(); it1!=lm1->features[0]->descriptors.SIFT.end(); it1++, it2++)
									distDesc += square( *it1 - *it2 );

								// Insert into the vector of Euclidean distances
								CLandmarksMap::_mEDD[ mPair ] = distDesc;
								// std::cout << "Nueva entrada! - (LIK): " << exp( K_desc * distDesc ) << " -> " << "(" << mPair.first << "," << mPair.second << ")" << std::endl;

							} // end if
							else
							{
								distDesc = (unsigned long)CLandmarksMap::_mEDD[ mPair ];
								// std::cout << "Ya esta calculado!: " << "(" << mPair.first << "," << mPair.second << ")"  << ": " << distDesc << std::endl;
							}

							likByDesc = exp( K_desc * distDesc );

							//likByDesc = max(likByDesc, 1e-5);
							//printf("LIK (%d,%d) - (Dist) %f , (Desc) %f - %u\n", idx1, idx2, likByDist, likByDesc, distDesc );

							//if (likByDesc>0.9)
							//{
							//	printf("LM1: %d (%.3f,%.3f,%.3f), LM2: %d (%.3f,%.3f,%.3f)\n",	idx1, lm1->pose_mean.x, lm1->pose_mean.y, lm1->pose_mean.z,
							//																	idx2, lm2->pose_mean.x, lm2->pose_mean.y, lm2->pose_mean.z );
							//	mrpt::system::pause();
							//}

							lik_i += likByDist*likByDesc;	// Cumulative Likelihood
						}
						else
						{
							// If the EUCLIDEAN distance is too large, we assume that the cumulative likelihood is (almost) zero.
							lik_i += 1e-10f;
						}
					} // end if

				} // end for "lm2"
				//lik += (0.1 + 0.9*lik_i);		// (CONSENSUS) Total likelihood (assuming independent probabilities)
				//lik += lik_i;					// (CONSENSUS) Total likelihood (assuming independent probabilities)
				lik *= (0.1 + 0.9*lik_i);		// (TRADITIONAL) Total likelihood (assuming independent probabilities)
				// lik *= lik_i;				// (TRADITIONAL) Total likelihood (assuming independent probabilities)
			}
		} // end for "lm1"
		//std::cout << "LIK OBS:" << lik << std::endl;

		//f = os::fopen("likelihood","a+");
		//os::fprintf(f, "%e\n", lik);
		//os::fclose(f);
		//std::cout << "Fin del computeLikelihood: " << lik << std::endl;
		//system("pause");

		break;


	case 1:	// SIM, ELINAS, GRIFFIN, LITTLE
		double						dist;
		unsigned int				k;
		TMatchingPairList::iterator	itCorr;

		lik = 1.0f;

		// Check if the Matches are inserted into the function
		if ( correspondences == NULL )
			THROW_EXCEPTION( "When using this method with SIFTLikelihoodMethod = 1, TMatchingPairList *correspondence can not be NULL" );

		if ( otherCorrespondences == NULL )
			THROW_EXCEPTION( "When using this method with SIFTLikelihoodMethod = 1, std::vector<bool> *otherCorrespondences can not be NULL" );

		for ( itCorr = correspondences->begin(); itCorr != correspondences->end(); itCorr++ )
		{
			CLandmark *lm1 = theMap->landmarks.get( itCorr->other_idx );
			CLandmark *lm2 = landmarks.get( itCorr->this_idx );

			dij(0,0) = lm1->pose_mean.x - lm2->pose_mean.x;
			dij(0,1) = lm1->pose_mean.y - lm2->pose_mean.y;
			dij(0,2) = lm1->pose_mean.z - lm2->pose_mean.z;

			// Equivalent covariance from "i" to "j":
			Cij = CMatrixDouble( lm1_pose.cov + lm2_pose.cov );
			Cij_1 = Cij.inv();

			distMahaFlik2 =  dij.multiply_HCHt_scalar(Cij_1); // ( dij * Cij_1 * (~dij) )(0,0);

			dist = min( (double)likelihoodOptions.SIFTnullCorrespondenceDistance, distMahaFlik2 );

			lik *= exp( -0.5 * dist );

		} // end for correspondences

		// We complete the likelihood with the null correspondences
		for ( k = 1; k <= ( theMap->size() - correspondences->size() ); k++ )
			lik *= likelihoodOptions.SIFTnullCorrespondenceDistance;

		break;

	} // end switch


	/** /
	saveToMATLABScript3D(std::string(("ver_ref.m")),"b.");
	theMap->saveToMATLABScript3D(std::string(("ver_sensed.m")),"r.");
	theMap->saveToTextFile(std::string("debug_map1.txt"));
	this->saveToTextFile(std::string("debug_map2.txt"));
	/ **/

	lik = log(lik);
	MRPT_CHECK_NORMAL_NUMBER( lik );
	return lik;
}

/*---------------------------------------------------------------
					TInsertionOptions
  ---------------------------------------------------------------*/
CLandmarksMap::TInsertionOptions::TInsertionOptions() :
	insert_SIFTs_from_monocular_images	( true ),
	insert_SIFTs_from_stereo_images		( true ),
	insert_Landmarks_from_range_scans	( true ),
	SiftCorrRatioThreshold				( 0.4f ),
	SiftLikelihoodThreshold				( 0.5f ),

	SiftEDDThreshold					( 200.0f ),
	SIFTMatching3DMethod				( 0 ),
	SIFTLikelihoodMethod				( 0 ),

	SIFTsLoadDistanceOfTheMean			( 3.0f ),
	SIFTsLoadEllipsoidWidth				( 0.05f ),

	SIFTs_stdXY							( 2.0f ),
	SIFTs_stdDisparity					( 1.0f ),

	SIFTs_numberOfKLTKeypoints			( 60 ),
	SIFTs_stereo_maxDepth				( 15.0f ),

	SIFTs_epipolar_TH					( 1.5f ),
	PLOT_IMAGES							( false ),

	SIFT_feat_options					( vision::featSIFT )
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CLandmarksMap::TInsertionOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CLandmarksMap::TInsertionOptions] ------------ \n\n");

	out.printf("insert_SIFTs_from_monocular_images      = %c\n", insert_SIFTs_from_monocular_images ? 'Y':'N');
	out.printf("insert_SIFTs_from_stereo_images         = %c\n", insert_SIFTs_from_stereo_images ? 'Y':'N');
	out.printf("insert_Landmarks_from_range_scans       = %c\n", insert_Landmarks_from_range_scans ? 'Y':'N');
	out.printf("\n");
	out.printf("SiftCorrRatioThreshold                  = %f\n", SiftCorrRatioThreshold);
	out.printf("SiftLikelihoodThreshold                 = %f\n", SiftLikelihoodThreshold);
	out.printf("SiftEDDThreshold						= %f\n", SiftEDDThreshold);
	out.printf("SIFTMatching3DMethod					= %d\n", SIFTMatching3DMethod);
	out.printf("SIFTLikelihoodMethod					= %d\n", SIFTLikelihoodMethod);

	out.printf("SIFTsLoadDistanceOfTheMean              = %f\n", SIFTsLoadDistanceOfTheMean);
	out.printf("SIFTsLoadEllipsoidWidth                 = %f\n", SIFTsLoadEllipsoidWidth);
	out.printf("\n");
	out.printf("SIFTs_stdXY                             = %f\n", SIFTs_stdXY);
	out.printf("SIFTs_stdDisparity                      = %f\n", SIFTs_stdDisparity);
	out.printf("\n");
	out.printf("SIFTs_numberOfKLTKeypoints              = %i\n", SIFTs_numberOfKLTKeypoints);
	out.printf("SIFTs_stereo_maxDepth                   = %f\n", SIFTs_stereo_maxDepth);
	out.printf("SIFTs_epipolar_TH						= %f\n", SIFTs_epipolar_TH);
	out.printf("PLOT_IMAGES								= %c\n", PLOT_IMAGES ? 'Y':'N');

	SIFT_feat_options.dumpToTextStream(out);

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CLandmarksMap::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{

	insert_SIFTs_from_monocular_images	= iniFile.read_bool(section.c_str(),"insert_SIFTs_from_monocular_images",insert_SIFTs_from_monocular_images);
	insert_SIFTs_from_stereo_images		= iniFile.read_bool(section.c_str(),"insert_SIFTs_from_stereo_images",insert_SIFTs_from_stereo_images);
	insert_Landmarks_from_range_scans	= iniFile.read_bool(section.c_str(),"insert_Landmarks_from_range_scans",insert_Landmarks_from_range_scans);
	SiftCorrRatioThreshold				= iniFile.read_float(section.c_str(),"SiftCorrRatioThreshold",SiftCorrRatioThreshold);
	SiftLikelihoodThreshold				= iniFile.read_float(section.c_str(),"SiftLikelihoodThreshold",SiftLikelihoodThreshold);
	SiftEDDThreshold					= iniFile.read_float(section.c_str(),"SiftEDDThreshold",SiftEDDThreshold);
	SIFTMatching3DMethod				= iniFile.read_int(section.c_str(),"SIFTMatching3DMethod",SIFTMatching3DMethod);
	SIFTLikelihoodMethod				= iniFile.read_int(section.c_str(),"SIFTLikelihoodMethod",SIFTLikelihoodMethod);
	SIFTsLoadDistanceOfTheMean			= iniFile.read_float(section.c_str(),"SIFTsLoadDistanceOfTheMean",SIFTsLoadDistanceOfTheMean);
	SIFTsLoadEllipsoidWidth				= iniFile.read_float(section.c_str(),"SIFTsLoadEllipsoidWidth",SIFTsLoadEllipsoidWidth);
	SIFTs_stdXY							= iniFile.read_float(section.c_str(),"SIFTs_stdXY",SIFTs_stdXY);
	SIFTs_stdDisparity					= iniFile.read_float(section.c_str(),"SIFTs_stdDisparity",SIFTs_stdDisparity);
	SIFTs_numberOfKLTKeypoints			= iniFile.read_int(section.c_str(),"SIFTs_numberOfKLTKeypoints",SIFTs_numberOfKLTKeypoints);
	SIFTs_stereo_maxDepth				= iniFile.read_float(section.c_str(),"SIFTs_stereo_maxDepth",SIFTs_stereo_maxDepth);
	SIFTs_epipolar_TH					= iniFile.read_float(section.c_str(),"SIFTs_epipolar_TH",SIFTs_epipolar_TH);
	PLOT_IMAGES							= iniFile.read_bool(section.c_str(),"PLOT_IMAGES",PLOT_IMAGES);

	SIFT_feat_options.loadFromConfigFile(iniFile,section);
}

/*---------------------------------------------------------------
					TLikelihoodOptions
  ---------------------------------------------------------------*/
CLandmarksMap::TLikelihoodOptions::TLikelihoodOptions() :
	rangeScan2D_decimation			( 20 ),
	SIFTs_sigma_euclidean_dist		( 0.30f ),
	SIFTs_sigma_descriptor_dist		( 100.0f ),
	SIFTs_mahaDist_std				( 4.0f ),
	SIFTnullCorrespondenceDistance  ( 4.0f ),
	SIFTs_decimation				( 1 ),
	SIFT_feat_options				( vision::featSIFT ),
	beaconRangesStd					( 0.08f ),
	beaconRangesUseObservationStd	(false),
	GPSOrigin						(),
	GPS_sigma						( 1.0f )
{
}

CLandmarksMap::TLikelihoodOptions::TGPSOrigin::TGPSOrigin() :
	longitude ( -4.47763833333333 ),
	latitude ( 36.71559000000000 ),
	altitude ( 42.3 ),
	ang ( 0 ),
	x_shift ( 0 ),
	y_shift ( 0 ),
	min_sat ( 4 )
{}


/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CLandmarksMap::TLikelihoodOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CLandmarksMap::TLikelihoodOptions] ------------ \n\n");

	out.printf("rangeScan2D_decimation                  = %i\n",rangeScan2D_decimation);
	out.printf("SIFTs_sigma_euclidean_dist              = %f\n",SIFTs_sigma_euclidean_dist);
	out.printf("SIFTs_sigma_descriptor_dist             = %f\n",SIFTs_sigma_descriptor_dist);
	out.printf("SIFTs_mahaDist_std                      = %f\n",SIFTs_mahaDist_std);
	out.printf("SIFTs_decimation                        = %i\n",SIFTs_decimation);
	out.printf("SIFTnullCorrespondenceDistance          = %f\n",SIFTnullCorrespondenceDistance);
	out.printf("beaconRangesStd                         = %f\n",beaconRangesStd);
	out.printf("beaconRangesUseObservationStd           = %c\n",beaconRangesUseObservationStd ? 'Y':'N');


	out.printf("GPSOrigin:LATITUDE                      = %f\n",GPSOrigin.latitude);
	out.printf("GPSOrigin:LONGITUDE                     = %f\n",GPSOrigin.longitude);
	out.printf("GPSOrigin:ALTITUDE                      = %f\n",GPSOrigin.altitude);
	out.printf("GPSOrigin:Rotation_Angle                = %f\n",GPSOrigin.ang);
	out.printf("GPSOrigin:x_shift                       = %f\n",GPSOrigin.x_shift);
	out.printf("GPSOrigin:y_shift                       = %f\n",GPSOrigin.y_shift);
	out.printf("GPSOrigin:min_sat                       = %i\n",GPSOrigin.min_sat);

	out.printf("GPS_sigma                               = %f (m)\n",GPS_sigma);

	SIFT_feat_options.dumpToTextStream(out);

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CLandmarksMap::TLikelihoodOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&iniFile,
	const std::string &section)
{
	rangeScan2D_decimation			= iniFile.read_int(section.c_str(),"rangeScan2D_decimation",rangeScan2D_decimation);
	SIFTs_sigma_euclidean_dist		= iniFile.read_double(section.c_str(),"SIFTs_sigma_euclidean_dist",SIFTs_sigma_euclidean_dist);
	SIFTs_sigma_descriptor_dist		= iniFile.read_double(section.c_str(),"SIFTs_sigma_descriptor_dist",SIFTs_sigma_descriptor_dist);
	SIFTs_mahaDist_std				= iniFile.read_float(section.c_str(),"SIFTs_mahaDist_std",SIFTs_mahaDist_std);
	SIFTs_decimation				= iniFile.read_int(section.c_str(),"SIFTs_decimation",SIFTs_decimation);
	SIFTnullCorrespondenceDistance	= iniFile.read_float(section.c_str(),"SIFTnullCorrespondenceDistance",SIFTnullCorrespondenceDistance);

	GPSOrigin.latitude				= iniFile.read_double(section.c_str(),"GPSOriginLatitude",GPSOrigin.latitude);
	GPSOrigin.longitude				= iniFile.read_double(section.c_str(),"GPSOriginLongitude",GPSOrigin.longitude);
	GPSOrigin.altitude				= iniFile.read_double(section.c_str(),"GPSOriginAltitude",GPSOrigin.altitude);
	GPSOrigin.ang					= iniFile.read_double(section.c_str(),"GPSOriginAngle",GPSOrigin.ang)*M_PI/180;
	GPSOrigin.x_shift				= iniFile.read_double(section.c_str(),"GPSOriginXshift",GPSOrigin.x_shift);
	GPSOrigin.y_shift				= iniFile.read_double(section.c_str(),"GPSOriginYshift",GPSOrigin.y_shift);
	GPSOrigin.min_sat				= iniFile.read_int(section.c_str(),"GPSOriginMinSat",GPSOrigin.min_sat);

	GPS_sigma						= iniFile.read_float(section.c_str(),"GPSSigma",GPS_sigma);

	beaconRangesStd					= iniFile.read_float(section.c_str(),"beaconRangesStd",beaconRangesStd);
	beaconRangesUseObservationStd	= iniFile.read_bool(section.c_str(),"beaconRangesUseObservationStd",beaconRangesUseObservationStd);

	SIFT_feat_options.loadFromConfigFile(iniFile,section);
}

/*---------------------------------------------------------------
					TFuseOptions
  ---------------------------------------------------------------*/
CLandmarksMap::TFuseOptions::TFuseOptions() :
	minTimesSeen	( 2 ),
	ellapsedTime	( 4.0f )
{
}

/*---------------------------------------------------------------
					 isEmpty
  ---------------------------------------------------------------*/
bool  CLandmarksMap::isEmpty() const
{
	return size()==0;
}

/*---------------------------------------------------------------
					 simulateBeaconReadings
  ---------------------------------------------------------------*/
void  CLandmarksMap::simulateBeaconReadings(
    const CPose3D							&in_robotPose,
    const CPoint3D						&in_sensorLocationOnRobot,
	mrpt::obs::CObservationBeaconRanges  &out_Observations ) const
{
	TSequenceLandmarks::const_iterator				it;
	mrpt::obs::CObservationBeaconRanges::TMeasurement	newMeas;
	CPoint3D										point3D,beacon3D;
	CPointPDFGaussian								beaconPDF;

	// Compute the 3D position of the sensor:
	point3D = in_robotPose + in_sensorLocationOnRobot;


	// Clear output data:
	out_Observations.sensedData.clear();
	out_Observations.timestamp = mrpt::system::getCurrentTime();

	// For each BEACON landmark in the map:
	for (it=landmarks.begin();it!=landmarks.end();it++)
	{
		if (it->getType() == featBeacon)
		{
			// Get the 3D position of the beacon (just the mean value):
			it->getPose( beaconPDF );
			beacon3D = beaconPDF.mean;

			float	range = point3D.distanceTo( beacon3D );

            // Add noise:
            range += out_Observations.stdError * randomGenerator.drawGaussian1D_normalized();
            range=max(0.0f,range);

			if (range>=out_Observations.minSensorDistance && range<=out_Observations.maxSensorDistance)
			{
                // Fill out:
                newMeas.beaconID				= it->ID;
                newMeas.sensorLocationOnRobot	= in_sensorLocationOnRobot;
                newMeas.sensedDistance			= range;

                // Insert:
                out_Observations.sensedData.push_back( newMeas );
			}
		} // end if beacon
	} // end for it
	// Done!
}

/*---------------------------------------------------------------
					 saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void  CLandmarksMap::saveMetricMapRepresentationToFile( const std::string	&filNamePrefix ) const
{
	MRPT_START

	// Matlab:
	std::string		fil1( filNamePrefix + std::string("_3D.m") );
	saveToMATLABScript3D( fil1 );

	// 3D Scene:
	opengl::COpenGLScene				scene;
	mrpt::opengl::CSetOfObjectsPtr obj3D = mrpt::opengl::CSetOfObjects::Create();
	getAs3DObject( obj3D );

	opengl::CGridPlaneXYPtr	objGround = opengl::CGridPlaneXY::Create(-100,100,-100,100,0,1);

	scene.insert(obj3D);
	scene.insert(objGround);

	std::string		fil2( filNamePrefix + std::string("_3D.3Dscene") );
	CFileOutputStream	f(fil2.c_str());
	f << scene;

	MRPT_END
}

/*---------------------------------------------------------------
 						getAs3DObject
  ---------------------------------------------------------------*/
void  CLandmarksMap::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj )const
{
	if (!genericMapParams.enableSaveAs3DObject) return;

	// TODO: Generate patchs in 3D, etc...

	// Save 3D ellipsoids
	CPointPDFGaussian	pointGauss;
	for (TCustomSequenceLandmarks::const_iterator	it=landmarks.begin();it!=landmarks.end();++it)
	{
		opengl::CEllipsoidPtr ellip = opengl::CEllipsoid::Create();

		it->getPose( pointGauss );

		ellip->setPose(pointGauss.mean);
		ellip->setCovMatrix(pointGauss.cov);
		ellip->enableDrawSolid3D(false);
		ellip->setQuantiles(3.0);
		ellip->set3DsegmentsCount(10);
		ellip->setColor(0,0,1);
		ellip->setName(mrpt::format("LM.ID=%u",static_cast<unsigned int>(it->ID) ));
		ellip->enableShowName(true);

		outObj->insert( ellip );
	}

}
/**** FAMD ****/
mrpt::maps::CLandmark::TLandmarkID  CLandmarksMap::getMapMaxID()
{
	return _mapMaxID;
}
/**** END FAMD ****/

const CLandmark* 	CLandmarksMap::TCustomSequenceLandmarks::getByID( CLandmark::TLandmarkID ID ) const
{
	for(size_t indx = 0; indx < m_landmarks.size(); indx++)
	{
		if( m_landmarks[indx].ID == ID )
			return &m_landmarks[indx];
	}
	return NULL;
}

//CLandmark* 	CLandmarksMap::TCustomSequenceLandmarks::getByID( CLandmark::TLandmarkID ID )
//{
//	for(size_t indx = 0; indx < m_landmarks.size(); indx++)
//	{
//		if( m_landmarks[indx].ID == ID )
//			return &m_landmarks[indx];
//	}
//	return NULL;
//}

const CLandmark* 	CLandmarksMap::TCustomSequenceLandmarks::getByBeaconID( unsigned int ID ) const
{
	for(size_t indx = 0; indx < m_landmarks.size(); indx++)
	{
		if( m_landmarks[indx].ID == ID )
			return &m_landmarks[indx];
	}
	return NULL;
}

/*---------------------------------------------------------------
   Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
 * \param  otherMap					  [IN] The other map to compute the matching with.
 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
 * \param  maxDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
 * \param  maxMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
 *
 * \return The matching ratio [0,1]
 * \sa computeMatchingWith2D
 ----------------------------------------------------------------*/
float  CLandmarksMap::compute3DMatchingRatio(const mrpt::maps::CMetricMap *otherMap2, const mrpt::poses::CPose3D &otherMapPose, const TMatchingRatioParams &params) const
{
	MRPT_START

	// Compare to a similar map only:
	const CLandmarksMap	*otherMap = NULL;

	if ( otherMap2->GetRuntimeClass() == CLASS_ID(CLandmarksMap) )
		otherMap = static_cast<const CLandmarksMap*>( otherMap2 );

	if (!otherMap) return 0;

	TCustomSequenceLandmarks::const_iterator	    itThis,itOther;
	std::deque<CPointPDFGaussianPtr>				poses3DThis,poses3DOther;
	std::deque<CPointPDFGaussianPtr>::iterator	    itPoseThis,itPoseOther;
	CPointPDFGaussian							tempPose;
	size_t										nThis  = landmarks.size();
	size_t										nOther = otherMap->landmarks.size();
	size_t										otherLandmarkWithCorrespondence = 0;

	// Checks:
	if (!nThis)  return 0;
	if (!nOther) return 0;

	// The transformation:
	CMatrixDouble44	pose3DMatrix;
	otherMapPose.getHomogeneousMatrix(pose3DMatrix);
	float		Tx  = pose3DMatrix.get_unsafe(0,3);
	float		Ty  = pose3DMatrix.get_unsafe(1,3);
	float		Tz  = pose3DMatrix.get_unsafe(2,3);

	// ---------------------------------------------------------------------------------------------------------------
	// Is there any "contact" between the spheres that contain all the points from each map after translating them??
	// (Note that we can avoid computing the rotation of all the points if this test fail, with a great speed up!)
	// ---------------------------------------------------------------------------------------------------------------
	if ( sqrt( square(Tx) + square(Ty) + square(Tz) ) >= ( landmarks.getLargestDistanceFromOrigin()+otherMap->landmarks.getLargestDistanceFromOrigin() + 1.0f  ) )
		return 0;		// There is no contact!

	// Prepare:
	poses3DOther.resize( nOther );
	for (size_t i=0;i<nOther;i++) poses3DOther[i] = CPointPDFGaussian::Create();

	poses3DThis.resize( nThis );
	for (size_t i=0;i<nThis;i++) poses3DThis[i] = CPointPDFGaussian::Create();

	// Save 3D poses of the other map with transformed coordinates:
	for (itOther=otherMap->landmarks.begin(),itPoseOther=poses3DOther.begin();itOther!=otherMap->landmarks.end();itOther++,itPoseOther++)
	{
		itOther->getPose( **itPoseOther );
		(*itPoseOther)->changeCoordinatesReference(otherMapPose);
	}

	// And the 3D poses of "this" map:
	for (itThis=landmarks.begin(),itPoseThis=poses3DThis.begin();itThis!=landmarks.end();itThis++,itPoseThis++)
	{
		itThis->getPose( **itPoseThis );
	}

	// Check whether each "other"'s LM has a correspondence or not:
	for (itOther=otherMap->landmarks.begin(),itPoseOther=poses3DOther.begin();itOther!=otherMap->landmarks.end();itOther++,itPoseOther++)
	{
		for (itThis=landmarks.begin(),itPoseThis=poses3DThis.begin();itThis!=landmarks.end();itThis++,itPoseThis++)
		{
			CMatrixDouble		COV = CMatrixDouble((*itPoseThis)->cov + (*itPoseOther)->cov );
			TPoint3D	D( (*itPoseThis)->mean.x() - (*itPoseOther)->mean.x(), (*itPoseThis)->mean.y() - (*itPoseOther)->mean.y(), (*itPoseThis)->mean.z() - (*itPoseOther)->mean.z() );
			CMatrixDouble	d(1,3);
			d(0,0) = D.x;
			d(0,1) = D.y;
			d(0,2) = D.z;

			float	distMaha = sqrt(  d.multiply_HCHt_scalar(COV.inv()) );  //(d*COV.inv()*(~d))(0,0) );

			if (distMaha<params.maxMahaDistForCorr )
			{
				// Now test the SIFT descriptors:
				if ( !itThis->features.empty() && !itOther->features.empty() &&
					itThis->features[0]->descriptors.SIFT.size() == itOther->features[0]->descriptors.SIFT.size() )
				{
					unsigned long	descrDist = 0;
					std::vector<unsigned char>::const_iterator it1,it2;
					for (it1=itThis->features[0]->descriptors.SIFT.begin(), it2=itOther->features[0]->descriptors.SIFT.begin();it1!=itThis->features[0]->descriptors.SIFT.end(); it1++,it2++)
						descrDist+= square( *it1 - *it2 );

					float	descrDist_f = sqrt(static_cast<float>(descrDist)) / itThis->features[0]->descriptors.SIFT.size();

					if ( descrDist_f < 1.5f )
					{
						otherLandmarkWithCorrespondence++;
						break; // Go to the next "other" LM
					}
				}
			}
		} // for each in "this"

	} // for each in "other"


	return static_cast<float>(otherLandmarkWithCorrespondence) / nOther;

	MRPT_END
}


/*---------------------------------------------------------------
					auxParticleFilterCleanUp
 ---------------------------------------------------------------*/
void  CLandmarksMap::auxParticleFilterCleanUp()
{
	//std::cout << "mEDD:" << std::endl;
	//std::cout << "-----------------------" << std::endl;
	//std::map<std::pair<mrpt::maps::CLandmark::TLandmarkID, mrpt::maps::CLandmark::TLandmarkID>, unsigned long>::iterator itmEDD;
	//for(itmEDD = CLandmarksMap::_mEDD.begin(); itmEDD != CLandmarksMap::_mEDD.end(); itmEDD++)
	//	std::cout << "(" << itmEDD->first.first << "," << itmEDD->first.second << ")"  << ": " << itmEDD->second << std::endl;

	CLandmarksMap::_mEDD.clear();
	CLandmarksMap::_maxIDUpdated = false;
	// TODO: Paco...
}


/*---------------------------------------------------------------
					simulateRangeBearingReadings
 ---------------------------------------------------------------*/
void  CLandmarksMap::simulateRangeBearingReadings(
    const CPose3D					&in_robotPose,
    const CPose3D					&in_sensorLocationOnRobot,
    CObservationBearingRange		&out_Observations,
    bool                            sensorDetectsIDs,
    const float                     in_stdRange,
    const float                     in_stdYaw,
    const float                     in_stdPitch,
    vector_size_t 					*out_real_associations,
	const double                    spurious_count_mean,
	const double                    spurious_count_std
	) const
{
	TSequenceLandmarks::const_iterator				it;
	size_t  idx;
	mrpt::obs::CObservationBearingRange::TMeasurement	newMeas;
	CPoint3D									    beacon3D;
	CPointPDFGaussian								beaconPDF;

	if (out_real_associations) out_real_associations->clear();

	// Compute the 3D position of the sensor:
	const CPose3D point3D = in_robotPose + CPose3D(in_sensorLocationOnRobot);

	// Clear output data:
	out_Observations.validCovariances	= false;
	out_Observations.sensor_std_range	= in_stdRange;
	out_Observations.sensor_std_yaw		= in_stdYaw;
	out_Observations.sensor_std_pitch	= in_stdPitch;

	out_Observations.sensedData.clear();
	out_Observations.timestamp = mrpt::system::getCurrentTime();
	out_Observations.sensorLocationOnRobot	= in_sensorLocationOnRobot;

	// For each BEACON landmark in the map:
	// ------------------------------------------
	for (idx=0,it=landmarks.begin();it!=landmarks.end();++it,++idx)
	{
		// Get the 3D position of the beacon (just the mean value):
		it->getPose( beaconPDF );
		beacon3D = CPoint3D(beaconPDF.mean);

		// Compute yaw,pitch,range:
		double	range,yaw,pitch;
		point3D.sphericalCoordinates(beacon3D, range,yaw,pitch);

		// Add noises:
		range += in_stdRange * randomGenerator.drawGaussian1D_normalized();
		yaw   += in_stdYaw   * randomGenerator.drawGaussian1D_normalized();
		pitch += in_stdPitch * randomGenerator.drawGaussian1D_normalized();

		yaw = math::wrapToPi(yaw);
		range=max(0.0,range);


		if ( range>=out_Observations.minSensorDistance &&
			 range<=out_Observations.maxSensorDistance &&
			 fabs(yaw)<=0.5f*out_Observations.fieldOfView_yaw &&
			 fabs(pitch)<=0.5f*out_Observations.fieldOfView_pitch )
		{
			// Fill out:
			if (sensorDetectsIDs)
					newMeas.landmarkID = it->ID;
			else	newMeas.landmarkID = INVALID_LANDMARK_ID;
			newMeas.range = range;
			newMeas.yaw = yaw;
			newMeas.pitch = pitch;

			// Insert:
			out_Observations.sensedData.push_back( newMeas );

			if (out_real_associations) out_real_associations->push_back(idx); // Real indices.
		}
	} // end for it

	const double fSpurious = randomGenerator.drawGaussian1D(spurious_count_mean,spurious_count_std);
	size_t nSpurious = 0;
	if (spurious_count_std!=0 || spurious_count_mean!=0)
		nSpurious = static_cast<size_t>(mrpt::utils::round_long( std::max(0.0,fSpurious) ) );

	// For each spurios reading to generate:
	// ------------------------------------------
	for (size_t i=0;i<nSpurious;i++)
	{
		// Make up yaw,pitch,range out from thin air:
		// (the conditionals on yaw & pitch are to generate 2D observations if we are in 2D, which we learn from a null std.dev.)
		const double range = randomGenerator.drawUniform(out_Observations.minSensorDistance,out_Observations.maxSensorDistance);
		const double yaw   = (out_Observations.sensor_std_yaw==0) ? 0 : randomGenerator.drawUniform(-0.5f*out_Observations.fieldOfView_yaw,0.5f*out_Observations.fieldOfView_yaw);
		const double pitch = (out_Observations.sensor_std_pitch==0) ? 0 :  randomGenerator.drawUniform(-0.5f*out_Observations.fieldOfView_pitch,0.5f*out_Observations.fieldOfView_pitch);

		// Fill out:
		newMeas.landmarkID = INVALID_LANDMARK_ID;  // Always invalid ID since it's spurious
		newMeas.range = range;
		newMeas.yaw = yaw;
		newMeas.pitch = pitch;

		// Insert:
		out_Observations.sensedData.push_back( newMeas );

		if (out_real_associations) out_real_associations->push_back( std::string::npos ); // Real index: spurious
	} // end for it

	// Done!
}

/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/vision.h>  // Precompiled headers


#include <mrpt/slam/CLandmark.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/system/os.h>

using namespace mrpt::slam; using namespace mrpt::utils; using namespace mrpt::poses;

IMPLEMENTS_SERIALIZABLE(CLandmark, CSerializable,mrpt::slam)

// Initialization:
CLandmark::TLandmarkID	CLandmark::m_counterIDs= static_cast<CLandmark::TLandmarkID>(0);

/*---------------------------------------------------------------
						Default constructor
  ---------------------------------------------------------------*/
CLandmark::CLandmark( ) :
	features(),
	pose_mean(),
	normal(),
	pose_cov_11(),pose_cov_22(),pose_cov_33(),pose_cov_12(),pose_cov_13(),pose_cov_23(),
	ID( INVALID_LANDMARK_ID ),
    timestampLastSeen( INVALID_TIMESTAMP ),
	seenTimesCount(0)
{
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CLandmark::~CLandmark()
{

}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
void 	CLandmark::getPose( CPointPDFGaussian	&pose ) const
{
	pose.mean.x( pose_mean.x);
	pose.mean.y( pose_mean.y);
	pose.mean.z( pose_mean.z);

	pose.cov(0,0) = pose_cov_11;
	pose.cov(1,1) = pose_cov_22;
	pose.cov(2,2) = pose_cov_33;

	pose.cov(0,1) = pose.cov(1,0) = pose_cov_12;
	pose.cov(0,2) = pose.cov(2,0) = pose_cov_13;

	pose.cov(1,2) = pose.cov(2,1) = pose_cov_23;
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
void 	CLandmark::setPose( const CPointPDFGaussian	&pose )
{
	pose_mean.x = pose.mean.x();
	pose_mean.y = pose.mean.y();
	pose_mean.z = pose.mean.z();

	pose_cov_11 = pose.cov(0,0);
	pose_cov_22 = pose.cov(1,1);
	pose_cov_33 = pose.cov(2,2);
	pose_cov_12 = pose.cov(0,1);
	pose_cov_13 = pose.cov(0,2);
    pose_cov_23 = pose.cov(1,2);
}

/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CLandmark::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 4;
	else
	{
		out << features
			<< pose_mean
			<< normal
			<< pose_cov_11 << pose_cov_22 << pose_cov_33
			<< pose_cov_12 << pose_cov_13 << pose_cov_23
			<< ID
			<< timestampLastSeen
			<< seenTimesCount;
	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  CLandmark::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3: 
		THROW_EXCEPTION("Importing from this old version is not implemented");
		break;
	case 4:
		{
		in  >> features
			>> pose_mean
			>> normal
			>> pose_cov_11 >> pose_cov_22 >> pose_cov_33
			>> pose_cov_12 >> pose_cov_13 >> pose_cov_23
			>> ID
			>> timestampLastSeen
			>> seenTimesCount;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}



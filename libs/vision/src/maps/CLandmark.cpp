/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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



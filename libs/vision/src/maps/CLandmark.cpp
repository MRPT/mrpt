/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers

#include <mrpt/maps/CLandmark.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;

IMPLEMENTS_SERIALIZABLE(CLandmark, CSerializable, mrpt::maps)

// Initialization:
CLandmark::TLandmarkID CLandmark::m_counterIDs =
	static_cast<CLandmark::TLandmarkID>(0);

CLandmark::~CLandmark() = default;

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
void CLandmark::getPose(CPointPDFGaussian& pose) const
{
	pose.mean.x(pose_mean.x);
	pose.mean.y(pose_mean.y);
	pose.mean.z(pose_mean.z);

	pose.cov(0, 0) = pose_cov_11;
	pose.cov(1, 1) = pose_cov_22;
	pose.cov(2, 2) = pose_cov_33;

	pose.cov(0, 1) = pose.cov(1, 0) = pose_cov_12;
	pose.cov(0, 2) = pose.cov(2, 0) = pose_cov_13;

	pose.cov(1, 2) = pose.cov(2, 1) = pose_cov_23;
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
void CLandmark::setPose(const CPointPDFGaussian& pose)
{
	pose_mean.x = pose.mean.x();
	pose_mean.y = pose.mean.y();
	pose_mean.z = pose.mean.z();

	pose_cov_11 = d2f(pose.cov(0, 0));
	pose_cov_22 = d2f(pose.cov(1, 1));
	pose_cov_33 = d2f(pose.cov(2, 2));
	pose_cov_12 = d2f(pose.cov(0, 1));
	pose_cov_13 = d2f(pose.cov(0, 2));
	pose_cov_23 = d2f(pose.cov(1, 2));
}

uint8_t CLandmark::serializeGetVersion() const { return 4; }
void CLandmark::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << features << pose_mean << normal << pose_cov_11 << pose_cov_22
		<< pose_cov_33 << pose_cov_12 << pose_cov_13 << pose_cov_23 << ID
		<< timestampLastSeen << seenTimesCount;
}

void CLandmark::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
			THROW_EXCEPTION(
				"Importing from this old version is not implemented");
			break;
		case 4:
		{
			in >> features >> pose_mean >> normal >> pose_cov_11 >>
				pose_cov_22 >> pose_cov_33 >> pose_cov_12 >> pose_cov_13 >>
				pose_cov_23 >> ID >> timestampLastSeen >> seenTimesCount;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

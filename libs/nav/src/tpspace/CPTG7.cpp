/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header
#include <mrpt/nav/tpspace/CPTG7.h>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::system;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(CPTG7,CParameterizedTrajectoryGenerator,mrpt::nav)

void CPTG7::setParams(const mrpt::utils::TParameters<double> &params)
{
	CPTG_DiffDrive_CollisionGridBased::setParamsCommon(params);
}


void CPTG7::readFromStream(mrpt::utils::CStream &in, int version)
{
	CPTG_DiffDrive_CollisionGridBased::internal_readFromStream(in);

	switch (version)
	{
	case 0:
		MRPT_TODO("continue")
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPTG7::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version) 
	{
		*version = 0;
		return;
	}

	CPTG_DiffDrive_CollisionGridBased::internal_writeToStream(out);

//	out << V_MAX << W_MAX
	MRPT_TODO("continue")

}

std::string CPTG7::getDescription() const
{
	return std::string( format("Type#7PTG") );
}

void CPTG7::ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const
{
	MRPT_UNUSED_PARAM(phi); MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y); MRPT_UNUSED_PARAM(t);
	float	R  = V_MAX / W_MAX;
	float	Ty = 2*(alpha/M_PIf)*R;

	v = V_MAX;
	w = W_MAX;
	if (alpha<0) w*=-1;

	if (fabs(x)>fabs(0.5f*Ty)) w *= -1;
	if (x>=fabs(Ty)) w = 0;
}

bool CPTG7::PTG_IsIntoDomain( double x, double y ) const
{
	return !( (fabs(y)>M_PI*V_MAX) || (x<M_PI*V_MAX) );
}


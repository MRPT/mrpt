/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header
#include <mrpt/nav/tpspace/CPTG_DiffDrive_C.h>
#include <mrpt/math/wrap2pi.h>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::utils;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(CPTG_DiffDrive_C,CParameterizedTrajectoryGenerator,mrpt::nav)

void CPTG_DiffDrive_C::setParams(const mrpt::utils::TParameters<double> &params)
{
	this->K = params["K"];

	CPTG_DiffDrive_CollisionGridBased::setParamsCommon(params);
}

void CPTG_DiffDrive_C::readFromStream(mrpt::utils::CStream &in, int version)
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

void CPTG_DiffDrive_C::writeToStream(mrpt::utils::CStream &out, int *version) const
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



std::string CPTG_DiffDrive_C::getDescription() const
{
	return mrpt::format("Type#1PTG,circ.arcs,K=%i",(int)K);
}

void CPTG_DiffDrive_C::ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const
{
	MRPT_UNUSED_PARAM(t); MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y); MRPT_UNUSED_PARAM(phi);
    // (v,w)
    v = V_MAX * sign(K);
	// Use a linear mapping:  (Old was: w = tan( alpha/2 ) * W_MAX * sign(K))
    w = (alpha/M_PI) * W_MAX * sign(K); 
}

bool CPTG_DiffDrive_C::PTG_IsIntoDomain( double x, double y ) const
{
	MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
	return true;
}

bool CPTG_DiffDrive_C::inverseMap_WS2TP(double x, double y, int &k_out, double &d_out, double tolerance_dist) const
{
	MRPT_UNUSED_PARAM(tolerance_dist);
	bool is_exact = true;
	if (y!=0)
	{
		double R = (x*x+y*y)/(2*y);
		const double Rmin = std::abs(V_MAX/W_MAX);

		double theta;

		if (K>0)
		{
			if (y>0)
					theta = atan2( (double)x,fabs(R)-y );
			else	theta = atan2( (double)x,y+fabs(R) );
		}
		else
		{
			if (y>0)
					theta = atan2( -(double)x,fabs(R)-y );
			else	theta = atan2( -(double)x,y+fabs(R) );
		}

		// Arc length must be possitive [0,2*pi]
		mrpt::math::wrapTo2PiInPlace(theta);

		// Distance thru arc:
		d_out = (float)(theta * (fabs(R)+turningRadiusReference));

		if (std::abs(R)<Rmin)
		{
			is_exact=false;
			R=Rmin*mrpt::utils::sign(R);
		}
		
		//Was: a = 2*atan( V_MAX / (W_MAX*R) );
		const double a = M_PI* V_MAX / (W_MAX*R);
		k_out = alpha2index( (float)a );

	}
	else
	{
		if (sign(x)==sign(K))
		{
			k_out = alpha2index(0);
			d_out = x;
			is_exact=true;
		}
		else
		{
			k_out = alpha2index((float)M_PI);
			d_out = 1e+3;
			is_exact=false;
		}
	}

	// Normalize:
	d_out = d_out / refDistance;

	ASSERT_ABOVEEQ_(k_out,0)
	ASSERT_BELOW_(k_out,this->m_alphaValuesCount)

	return is_exact;
}

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header
#include <mrpt/nav/tpspace/CPTG_Holo_Blend.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/utils/CStream.h>
#include "poly34.h"

using namespace mrpt::nav;
using namespace mrpt::utils;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(CPTG_Holo_Blend,CParameterizedTrajectoryGenerator,mrpt::nav)

CPTG_Holo_Blend a;

/* ===============
equation for "t" values of closest approach to obstacle (x0,y0):

(4*k2^2 + 4*k4^2)*t^3 + (6*k1*k2 + 6*k3*k4)*t^2 + (2*k1^2 + 2*k3^2 - 4*k2*x0 - 4*k4*y0)*t - 2*k1*x0 - 2*k3*y0
 
k1=vxi;
k3=vyi;

k2=( vxf - vxi )/(2*T_ramp)
k4=( vyf - vyi )/(2*T_ramp)
 
>> pretty(ans)
     2       2   3                        2        2       2
(4 k2  + 4 k4 ) t  + (6 k1 k2 + 6 k3 k4) t  + (2 k1  + 2 k3  - 4 k2 x0 - 4 k4 y0) t - 2 k1 x0 - 2 k3 y0

================== */

CPTG_Holo_Blend::CPTG_Holo_Blend() : 
	T_ramp(-1.0),
	V_MAX(-1.0), 
	W_MAX(-1.0),
	turningRadiusReference(0.30)
{ 
}
void CPTG_Holo_Blend::setParams(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection,  const std::string &sKeyPrefix)
{
	this->T_ramp      = cfg.read_double  (sSection, sKeyPrefix+std::string("T_ramp"), .0, true );
	this->V_MAX   = cfg.read_double  (sSection, sKeyPrefix+std::string("v_max_mps"), .0, true );
	this->W_MAX   = mrpt::utils::DEG2RAD( cfg.read_double  (sSection, sKeyPrefix+std::string("w_max_dps"), .0, true ) );
	this->turningRadiusReference   = cfg.read_double  (sSection, sKeyPrefix+std::string("turningRadiusReference"), turningRadiusReference);
}

std::string CPTG_Holo_Blend::getDescription() const
{
	return mrpt::format("PTG_Holo_Blend_Tramp=%.03f",T_ramp);
}


void CPTG_Holo_Blend::readFromStream(mrpt::utils::CStream &in, int version)
{
	CParameterizedTrajectoryGenerator::internal_readFromStream(in);

	switch (version)
	{
	case 0:
		in >> T_ramp >> V_MAX >> W_MAX >> turningRadiusReference;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPTG_Holo_Blend::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version) 
	{
		*version = 0;
		return;
	}

	CParameterizedTrajectoryGenerator::internal_writeToStream(out);
	out << T_ramp << V_MAX << W_MAX << turningRadiusReference;
}

bool CPTG_Holo_Blend::inverseMap_WS2TP(double x, double y, int &out_k, double &out_d, double tolerance_dist) const
{
	THROW_EXCEPTION("todo");
	MRPT_TODO("impl");
	return false;
}

bool CPTG_Holo_Blend::PTG_IsIntoDomain(double x, double y ) const
{
	THROW_EXCEPTION("todo");
	MRPT_TODO("impl");
	return false;
}

void CPTG_Holo_Blend::initialize(const std::string & cacheFilename, const bool verbose )
{
	// No need to initialize anything, just do some params sanity checks:
	ASSERT_(T_ramp>0);
	ASSERT_(V_MAX>0);
	ASSERT_(W_MAX>0);
}

void CPTG_Holo_Blend::deinitialize()
{
	// Nothing to do in a closed-form PTG.
}

void CPTG_Holo_Blend::directionToMotionCommand( uint16_t k, std::vector<double> &out_action_cmd ) const
{
	THROW_EXCEPTION("todo");
	MRPT_TODO("impl");
}

void CPTG_Holo_Blend::renderPathAsSimpleLine(const uint16_t k,mrpt::opengl::CSetOfLines &gl_obj,const float decimate_distance,const float max_path_distance) const
{
	THROW_EXCEPTION("todo");
	MRPT_TODO("impl");
}

size_t CPTG_Holo_Blend::getPathStepCount(uint16_t k) const
{
	THROW_EXCEPTION("todo");
	MRPT_TODO("impl");
}

void CPTG_Holo_Blend::getPathPose(uint16_t k, uint16_t step, mrpt::math::TPose2D &p) const
{
	THROW_EXCEPTION("todo");
	MRPT_TODO("impl");
}

double CPTG_Holo_Blend::getPathDist(uint16_t k, uint16_t step) const
{
	THROW_EXCEPTION("todo");
	MRPT_TODO("impl");
}

bool CPTG_Holo_Blend::getPathStepForDist(uint16_t k, double dist, uint16_t &out_step) const
{
	THROW_EXCEPTION("todo");
	MRPT_TODO("impl");
}

void CPTG_Holo_Blend::updateTPObstacle(double ox, double oy, std::vector<double> &tp_obstacles) const
{
	THROW_EXCEPTION("todo");
	MRPT_TODO("impl");
}

void CPTG_Holo_Blend::internal_processNewRobotShape()
{
}


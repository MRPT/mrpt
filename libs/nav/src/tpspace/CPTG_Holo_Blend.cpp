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

/*
Closed-form PTG. Parameters: 
- Initial velocity vector (xip, yip)
- Target velocity vector depends on \alpha: xfp = V_MAX*cos(alpha), yfp = V_MAX*sin(alpha)
- T_ramp time for velocity interpolation (xip,yip) -> (xfp, yfp)
- W_MAX: Rotational velocity for robot heading forwards.

Number of steps "d" for each PTG path "k": 
- Step = time increment PATH_TIME_STEP


*/

const double PATH_TIME_STEP = 10e-3;   // 10 ms


CPTG_Holo_Blend::CPTG_Holo_Blend() : 
	T_ramp(-1.0),
	V_MAX(-1.0), 
	W_MAX(-1.0),
	turningRadiusReference(0.30),
	curVelLocal(0,0,0)
{ 
}

CPTG_Holo_Blend::CPTG_Holo_Blend(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection,  const std::string &sKeyPrefix) :
	turningRadiusReference(0.30),
	curVelLocal(0,0,0)
{
	setParams(cfg,sSection, sKeyPrefix);
}

void CPTG_Holo_Blend::updateCurrentRobotVel(const mrpt::math::TTwist2D &curVelLocal)
{
	this->curVelLocal = curVelLocal;
}

void CPTG_Holo_Blend::setParams(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection,  const std::string &sKeyPrefix)
{
	this->T_ramp      = cfg.read_double  (sSection, sKeyPrefix+std::string("T_ramp"), .0, true );
	this->V_MAX   = cfg.read_double  (sSection, sKeyPrefix+std::string("v_max_mps"), .0, true );
	this->W_MAX   = mrpt::utils::DEG2RAD( cfg.read_double  (sSection, sKeyPrefix+std::string("w_max_dps"), .0, true ) );
	this->turningRadiusReference   = cfg.read_double  (sSection, sKeyPrefix+std::string("turningRadiusReference"), turningRadiusReference);

	// Initial velocity, for debugging (in runtime this will be update to the current robot vel)
	curVelLocal.vx = cfg.read_double  (sSection, sKeyPrefix+std::string("vxi"), curVelLocal.vx);
	curVelLocal.vy = cfg.read_double  (sSection, sKeyPrefix+std::string("vyi"), curVelLocal.vy);

	CParameterizedTrajectoryGenerator::setParamsCommon(cfg,sSection,sKeyPrefix);
}

std::string CPTG_Holo_Blend::getDescription() const
{
	return mrpt::format("PTG_Holo_Blend_Tramp=%.03f_Vmax=%.03f_Wmax=%.03f",T_ramp,V_MAX,W_MAX);
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
	int k; 
	double d;
	return inverseMap_WS2TP(x,y,k,d);
}

void CPTG_Holo_Blend::initialize(const std::string & cacheFilename, const bool verbose )
{
	// No need to initialize anything, just do some params sanity checks:
	ASSERT_(T_ramp>0);
	ASSERT_(V_MAX>0);
	ASSERT_(W_MAX>0);
	ASSERT_(m_alphaValuesCount>0);

	// DEBUG:
	//debugDumpInFiles("1");

}

void CPTG_Holo_Blend::deinitialize()
{
	// Nothing to do in a closed-form PTG.
}

void CPTG_Holo_Blend::directionToMotionCommand( uint16_t k, std::vector<double> &cmd_vel ) const
{
	const double dir_local = CParameterizedTrajectoryGenerator::index2alpha(k);

	// cmd_vel=[vel dir_local ramp_time rot_speed]:
	cmd_vel.resize(4);
	cmd_vel[0] = V_MAX;
	cmd_vel[1] = dir_local;
	cmd_vel[2] = T_ramp;
	cmd_vel[3] = W_MAX;
}

size_t CPTG_Holo_Blend::getPathStepCount(uint16_t k) const
{
	MRPT_TODO("impl");
	return 600;
}

void CPTG_Holo_Blend::getPathPose(uint16_t k, uint16_t step, mrpt::math::TPose2D &p) const
{
	const double t = PATH_TIME_STEP*step;
	const double dir = CParameterizedTrajectoryGenerator::index2alpha(k);

	const double TR2_ = 1.0/(2*T_ramp);
	const double vxf = V_MAX * cos(dir), vyf = V_MAX * sin(dir);
	const double vxi = curVelLocal.vx, vyi = curVelLocal.vy;

	// Translational part:
	if (t<T_ramp)
	{
		p.x   = vxi * t + t*t * TR2_ * (vxf-vxi);
		p.y   = vyi * t + t*t * TR2_ * (vyf-vyi);
	}
	else
	{
		p.x   = T_ramp *0.5*(vxi+vxf) + (t-T_ramp) * vxf;
		p.y   = T_ramp *0.5*(vyi+vyf) + (t-T_ramp) * vyf;
	}

	// Rotational part:
	const double T_rot = std::abs(dir)/W_MAX;
	if (t<T_rot)
	     p.phi = t*dir/T_rot;
	else p.phi = dir;
}

double CPTG_Holo_Blend::getPathDist(uint16_t k, uint16_t step) const
{
	const double t = PATH_TIME_STEP*step;
	const double dir = CParameterizedTrajectoryGenerator::index2alpha(k);

	const double TR2_ = 1.0/(2*T_ramp);
	const double vxf = V_MAX * cos(dir), vyf = V_MAX * sin(dir);
	const double vxi = curVelLocal.vx, vyi = curVelLocal.vy;

	const double k2 = (vxf-vxi)*TR2_;
	const double k4 = (vyf-vyi)*TR2_;

	if (t<T_ramp)
	{
		const double num1 = vxi+2*t*k2;
		const double num2 = vyi+2*t*k4;
		const double dist_trans = std::sqrt( ( num1*num1*num1/k2 + num2*num2*num2/k4 )/6.0  );
		return dist_trans;
	}
	else
	{
		const double num1 = vxi+2*T_ramp*k2;
		const double num2 = vyi+2*T_ramp*k4;
		const double dist_trans = std::sqrt( ( num1*num1*num1/k2 + num2*num2*num2/k4 )/6.0  ) + (t-T_ramp) * V_MAX;
		return dist_trans;
	}
}

bool CPTG_Holo_Blend::getPathStepForDist(uint16_t k, double dist, uint16_t &out_step) const
{
	const double dir = CParameterizedTrajectoryGenerator::index2alpha(k);

	const double TR2_ = 1.0/(2*T_ramp);
	const double vxf = V_MAX * cos(dir), vyf = V_MAX * sin(dir);
	const double vxi = curVelLocal.vx, vyi = curVelLocal.vy;

	const double k2 = (vxf-vxi)*TR2_;
	const double k4 = (vyf-vyi)*TR2_;

	// 3 roots for possible solutions within 0<t<t_ramp:
	{
		double r[3];
		const double d = dist;
		r[0] = (pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0))*1.0/pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(-1.0/2.7E1)+sqrt(-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0),3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(1.0/2.7E1)+(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)-(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),2.0))-(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)+(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),1.0/3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(-1.0/2.7E1)+sqrt(-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0),3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(1.0/2.7E1)+(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)-(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),2.0))-(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)+(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),1.0/3.0)-((k2*k2)*k4*vxi*4.0+k2*(k4*k4)*vyi*4.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0);
		r[1] = (pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0))*1.0/pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(-1.0/2.7E1)+sqrt(-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0),3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(1.0/2.7E1)+(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)-(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),2.0))-(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)+(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),1.0/3.0)*(-1.0/2.0)-sqrt(3.0)*((pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0))*1.0/pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(-1.0/2.7E1)+sqrt(-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0),3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(1.0/2.7E1)+(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)-(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),2.0))-(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)+(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),1.0/3.0)-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(-1.0/2.7E1)+sqrt(-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0),3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(1.0/2.7E1)+(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)-(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),2.0))-(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)+(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),1.0/3.0))*5.0E-1*sqrt(-1.0)-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(-1.0/2.7E1)+sqrt(-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0),3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(1.0/2.7E1)+(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)-(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),2.0))-(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)+(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),1.0/3.0)*(1.0/2.0)-((k2*k2)*k4*vxi*4.0+k2*(k4*k4)*vyi*4.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0);
		r[2] = (pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0))*1.0/pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(-1.0/2.7E1)+sqrt(-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0),3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(1.0/2.7E1)+(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)-(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),2.0))-(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)+(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),1.0/3.0)*(-1.0/2.0)+sqrt(3.0)*((pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0))*1.0/pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(-1.0/2.7E1)+sqrt(-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0),3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(1.0/2.7E1)+(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)-(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),2.0))-(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)+(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),1.0/3.0)-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(-1.0/2.7E1)+sqrt(-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0),3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(1.0/2.7E1)+(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)-(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),2.0))-(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)+(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),1.0/3.0))*5.0E-1*sqrt(-1.0)-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(-1.0/2.7E1)+sqrt(-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0),3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(1.0/2.7E1)+(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)-(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),2.0))-(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0)+(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),1.0/3.0)*(1.0/2.0)-((k2*k2)*k4*vxi*4.0+k2*(k4*k4)*vyi*4.0)/(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0);
	}
 
	return 0;
}


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

void CPTG_Holo_Blend::updateTPObstacle(double ox, double oy, std::vector<double> &tp_obstacles) const
{
	THROW_EXCEPTION("todo");
	MRPT_TODO("impl");
}

void CPTG_Holo_Blend::internal_processNewRobotShape()
{
	// Nothing to do in a closed-form PTG.
}


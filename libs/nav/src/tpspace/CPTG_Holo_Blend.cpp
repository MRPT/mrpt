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
#include <mrpt/utils/round.h>
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
const double eps = 1e-5;               // epsilon for detecting 1/0 situation

// Axiliary function for computing the line-integral distance along the trajectory, handling special cases of 1/0:
inline double calc_trans_distance_t_below_Tramp(double k2, double k4, double vxi,double vyi, double t)
{
/*
dd = sqrt( (4*k2^2 + 4*k4^2)*t^2 + (4*k2*vxi + 4*k4*vyi)*t + vxi^2 + vyi^2 ) dt
a t^2 + b t + c 
*/
	const double c = (vxi*vxi+vyi*vyi);
	if (std::abs(k2)>eps || std::abs(k4)>eps)
	{
		const double a = ((k2*k2)*4.0+(k4*k4)*4.0);
		const double b = (k2*vxi*4.0+k4*vyi*4.0);
	
		const double d_t = (t*(1.0/2.0)+(b*(1.0/4.0))/a)*sqrt(c+b*t+a*(t*t))+1.0/pow(a,3.0/2.0)*log(1.0/sqrt(a)*(b*(1.0/2.0)+a*t)+sqrt(c+b*t+a*(t*t)))*(a*c-(b*b)*(1.0/4.0))*(1.0/2.0);
		const double log_arg = 1.0/sqrt(a)*(0.5*b)+sqrt(c);
		if (std::abs(log_arg)<eps)
			return d_t;
		else {
			const double d_0 = ((b*(1.0/4.0))/a)*sqrt(c)+1.0/pow(a,3.0/2.0)*log(log_arg)*(a*c-(b*b)*(1.0/4.0))*(1.0/2.0);
			return d_t-d_0;
		}
	}
	else
	{
		return std::sqrt(c)*t;
	}
}


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
	MRPT_UNUSED_PARAM(tolerance_dist);
	ASSERT_(x!=0 || y!=0);

	const double TIME_MISMATCH_TOLERANCE = 4.0*((2*M_PI/m_alphaValuesCount) * std::sqrt(x*x+y*y))/V_MAX;
	const double eps_distance            = 2.1*((2*M_PI/m_alphaValuesCount) * std::sqrt(x*x+y*y));

	double found_min_dist = std::numeric_limits<double>::max();
	int    found_k        = -1; // invalid

	const double TR2_ = 1.0/(2*T_ramp);
	const double vxi = curVelLocal.vx, vyi = curVelLocal.vy;

	// General idea: get the shortest path for all alpha values.
	for (uint16_t k=0;k<m_alphaValuesCount;k++)
	{
		const double dir = CParameterizedTrajectoryGenerator::index2alpha(k);
		const double vxf = V_MAX * cos(dir), vyf = V_MAX * sin(dir);

		const double k2 = (vxf-vxi)*TR2_;
		const double k4 = (vyf-vyi)*TR2_;

		double tx_solve, ty_solve;
		bool  tx_any = false, ty_any = false;

		//  Attempt to solve for t < T_ramp
		// -----------------------------------
		if (std::abs(vxf-vxi)<eps)
		{
			if (std::abs(vxi)<eps) {
				if (std::abs(x)<eps_distance)
				     tx_any = true;
				else tx_solve = -1.0;
			}
			else { // x = vxi * t  ->  t = x / vxi;
				tx_solve = x / vxi;
			}
		}
		else
		{
			const double discr_x = (vxf*x*2.0-vxi*x*2.0+T_ramp*(vxi*vxi))/T_ramp;
			if (discr_x<0) continue; // No solution
			const double sqrx = sqrt(discr_x);
			double tx_solved[2];
			tx_solved[0] = -(T_ramp*(vxi+sqrx))/(vxf-vxi);
			tx_solved[1] = -(T_ramp*(vxi-sqrx))/(vxf-vxi);
			tx_solve = tx_solved[0]>0 ? tx_solved[0] : tx_solved[1];
			if (!(tx_solve==tx_solve && mrpt::math::isFinite(tx_solve) && tx_solve>=0 && tx_solve<=T_ramp))
				tx_solve = -1.0;
		}

		if (std::abs(vyf-vyi)<eps) {
			if (std::abs(vyi)<eps) {
				if (std::abs(y)<eps_distance)
				     ty_any = true;
				else ty_solve = -1.0;
			}
			else { // y = vyi * t  ->  t = y / vyi;
				ty_solve = y / vyi;
			}
		}
		else
		{
			const double discr_y = (vyf*y*2.0-vyi*y*2.0+T_ramp*(vyi*vyi))/T_ramp;
			if (discr_y<0) continue; // No solution
			const double sqry = sqrt(discr_y);
			double ty_solved[2];
			ty_solved[0] = -(T_ramp*(vyi+sqry))/(vyf-vyi);
			ty_solved[1] = -(T_ramp*(vyi-sqry))/(vyf-vyi);
			ty_solve = ty_solved[0]>0 ? ty_solved[0] : ty_solved[1];
			if (!(ty_solve==ty_solve && mrpt::math::isFinite(ty_solve) && ty_solve>=0 && ty_solve<=T_ramp))
				ty_solve = -1.0;
		}
		
		if ((tx_any || (tx_solve>=0 && tx_solve<=T_ramp)) && 
			(ty_any || (ty_solve>=0 && ty_solve<=T_ramp)) )
		{
			// Good solution found for t < T_ramp
		}
		else
		{
			//  Attempt to solve for t > T_ramp
			// -----------------------------------
			if (std::abs(vxf)<eps)
			{
				const double final_x = vxi * T_ramp + T_ramp*T_ramp * TR2_ * (vxf-vxi);
				if (std::abs(x-final_x)<eps_distance)
					 tx_any = true;
				else tx_solve=-1.0; // No solution found for t>T_ramp
			}
			else {
				tx_solve = (x-T_ramp*(vxf+vxi)*(1.0/2.0)+T_ramp*vxf)/vxf;
			}

			if (std::abs(vyf)<eps)
			{
				const double final_y = vyi * T_ramp + T_ramp*T_ramp * TR2_ * (vyf-vyi);
				if (std::abs(y-final_y)<eps_distance)
					 ty_any = true;
				else ty_solve=-1.0; // No solution found for t>T_ramp
			}
			else ty_solve = (y-T_ramp*(vyf+vyi)*(1.0/2.0)+T_ramp*vyf)/vyf;
		}

		// Get the final solution:
		double t_solve;
		// The most common case:
		if (!tx_any && !ty_any) {
			if ( std::abs(tx_solve-ty_solve) > TIME_MISMATCH_TOLERANCE)
				continue; // No solution
			t_solve = tx_solve;
		}
		// Degenerate case: init = final velocity
		if (tx_any && ty_any) {
			t_solve = sqrt(x*x+y*y) / V_MAX;
		}
		// Special cases:
		if (!tx_any && ty_any) {
			t_solve = tx_solve;
		}
		if (tx_any && !ty_any) {
			t_solve = ty_solve;
		}

		// Good solution: save if better
		if (t_solve>=0)
		{
			const double dist_trans = calc_trans_distance_t_below_Tramp(k2,k4,vxi,vyi,t_solve);

			if (dist_trans<found_min_dist)
			{
				found_min_dist = dist_trans;
				found_k = k;
			}
		}
	} // end for each k

	if (found_k>=0)
	{
		out_d = found_min_dist / this->refDistance;
		out_k =  found_k;
		return true;
	}
	return false; // No solution found
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


	// DEBUG TESTS
#if 1
	//debugDumpInFiles("1");
	uint16_t step; 
	double d;
	
	getPathStepForDist(170,1.3,step);
	d = getPathDist(170,step);

	getPathStepForDist(170,0.3,step);
	d = getPathDist(170,step);
	{
		int k;
		double d;
		bool valid=inverseMap_WS2TP(1,5,k,d);
	}
#endif


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
	uint16_t step;
	if (!getPathStepForDist(k,this->refDistance,step)) {
		THROW_EXCEPTION_CUSTOM_MSG1("Could not solve closed-form distance for k=%u",static_cast<unsigned>(k));
	}
	return step;
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
		return calc_trans_distance_t_below_Tramp(k2,k4,vxi,vyi,t);
	}
	else
	{
		const double dist_trans = (t-T_ramp) * V_MAX + calc_trans_distance_t_below_Tramp(k2,k4,vxi,vyi,T_ramp);
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

	// Possible solutions within  t > T_ramp:
	{
		const double dist_trans_T_ramp = calc_trans_distance_t_below_Tramp(k2,k4,vxi,vyi,T_ramp);
		
		if (dist>dist_trans_T_ramp)
		{
			// Good solution:
			const double t_solved = T_ramp + (dist-dist_trans_T_ramp)/V_MAX;
			out_step = mrpt::utils::round( t_solved / PATH_TIME_STEP );
			return true;
		}
	}

	// 3 roots for possible solutions within t < T_ramp:
	MRPT_TODO("Re-solve these solutions for dd=sqrt(dx^2+dy^2)dt !!");
	double root1;
	{
		const double d = dist;

		const double z0 = ((k2*k2)*k4*vxi*4.0+k2*(k4*k4)*vyi*4.0);
		const double z4 = (k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0);
		const double z1 =z0/z4;
		const double z2 = (pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/z4);
		const double z3 = pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(-1.0/2.7E1)+sqrt(-pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,2.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/9.0)-(k2*k4*(vxi*vxi)*2.0+k2*k4*(vyi*vyi)*2.0)/z4,3.0)+pow(pow((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1,3.0)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,3.0)*(1.0/2.7E1)+(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/z4-(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),2.0))-(k4*(vxi*vxi*vxi)*(1.0/2.0)+k2*(vyi*vyi*vyi)*(1.0/2.0)-(d*d)*k2*k4*3.0)/z4+(k2*k4*(vxi*vxi)*6.0+k2*k4*(vyi*vyi)*6.0)*((k2*k2)*k4*vxi*1.2E1+k2*(k4*k4)*vyi*1.2E1)*1.0/pow(k2*(k4*k4*k4)*8.0+(k2*k2*k2)*k4*8.0,2.0)*(1.0/6.0),1.0/3.0);

		root1 = z2/z3+z3-z0/z4;
		// root 2 & 3 are always complex numbers: discarded
	}

	if (root1==root1 && // NAN
		mrpt::math::isFinite(root1) && // IND
		root1>=0 && 
		root1<=T_ramp )
	{
		// Good solution:
		out_step = mrpt::utils::round( root1 / PATH_TIME_STEP );
		return true;
	}


	return false;
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
	ox++;
	MRPT_TODO("impl");
}

void CPTG_Holo_Blend::internal_processNewRobotShape()
{
	// Nothing to do in a closed-form PTG.
}


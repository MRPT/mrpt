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
#include <mrpt/utils/types_math.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/round.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/math/poly_roots.h>

using namespace mrpt::nav;
using namespace mrpt::utils;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(CPTG_Holo_Blend,CParameterizedTrajectoryGenerator,mrpt::nav)

/*
Closed-form PTG. Parameters: 
- Initial velocity vector (xip, yip)
- Target velocity vector depends on \alpha: xfp = V_MAX*cos(alpha), yfp = V_MAX*sin(alpha)
- T_ramp_max max time for velocity interpolation (xip,yip) -> (xfp, yfp)
- W_MAX: Rotational velocity for robot heading forwards.

Number of steps "d" for each PTG path "k": 
- Step = time increment PATH_TIME_STEP

*/

// Uncomment only for benchmarking during development
//#define DO_PERFORMANCE_BENCHMARK

#ifdef DO_PERFORMANCE_BENCHMARK
	mrpt::utils::CTimeLogger tl;
	#define PERFORMANCE_BENCHMARK  CTimeLoggerEntry  tle(tl, __CURRENT_FUNCTION_NAME__);
#else 
	#define PERFORMANCE_BENCHMARK  
#endif

const double PATH_TIME_STEP = 10e-3;   // 10 ms
const double eps = 1e-8;               // epsilon for detecting 1/0 situation

// Axiliary function for calc_trans_distance_t_below_Tramp() and others:
double calc_trans_distance_t_below_Tramp_abc(double t, double a,double b, double c)
{
	ASSERT_(t>=0);
	if (t==0.0) return .0;

	double dist;
	// Handle special case: degenerate (a*t^2+b*t+c) =  (t-r)^2
	const double discr = b*b-4*a*c;
	if (std::abs(discr)<1e-6)
	{
		const double r = -b/2*a;
		// dist= definite integral [0,t] of: |t-r| dt
		if (r<0) {
			dist=+0.5*t*t-r*t;
		} else if (r>t) {
			dist=-0.5*t*t+r*t;
		} else {
			// r in [0,t]
			dist=0.5*(r*r+(t-r)*(t-r));
		}
	}
	else 
	{
		// General case:
		// Indefinite integral of sqrt(a*t^2+b*t+c):
		const double int_t = (t*(1.0/2.0)+(b*(1.0/4.0))/a)*sqrt(c+b*t+a*(t*t))+1.0/pow(a,3.0/2.0)*log(1.0/sqrt(a)*(b*(1.0/2.0)+a*t)+sqrt(c+b*t+a*(t*t)))*(a*c-(b*b)*(1.0/4.0))*(1.0/2.0);
		// Limit when t->0:
		const double int_t0 = (b*sqrt(c)*(1.0/4.0))/a+1.0/pow(a,3.0/2.0)*log(1.0/sqrt(a)*(b+sqrt(a)*sqrt(c)*2.0)*(1.0/2.0))*(a*c-(b*b)*(1.0/4.0))*(1.0/2.0);
		dist=int_t - int_t0;// Definite integral [0,t]
	}
#ifdef _DEBUG
	using namespace mrpt;
	MRPT_CHECK_NORMAL_NUMBER(dist);
	ASSERT_(dist>=.0);
#endif
	return dist;
}


// Axiliary function for computing the line-integral distance along the trajectory, handling special cases of 1/0:
double calc_trans_distance_t_below_Tramp(double k2, double k4, double vxi,double vyi, double t)
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

		// Numerically-ill case: b=c=0 (initial vel=0)
		if (std::abs(b)<eps && std::abs(c)<eps) {
			// Indefinite integral of simplified case: sqrt(a)*t
			const double int_t = sqrt(a)*(t*t)*0.5;
			return int_t; // Definite integral [0,t]
		}
		else {
			return calc_trans_distance_t_below_Tramp_abc(t,a,b,c);
		}
	}
	else {
		return std::sqrt(c)*t;
	}
}

inline double calc_T_ramp(const double T_ramp_max, double vxi, double vyi, double vxf, double vyf, double V_MAX)
{
	return T_ramp_max * std::max( std::abs(vxi-vxf), std::abs(vyi-vyf) ) / V_MAX;
}
inline double calc_T_ramp_dir(const double T_ramp_max, double vxi, double vyi, double dir, double V_MAX)
{
	return T_ramp_max * std::max( std::abs(vxi-V_MAX*cos(dir)), std::abs(vyi-V_MAX*sin(dir)) ) / V_MAX;
}


CPTG_Holo_Blend::CPTG_Holo_Blend() : 
	T_ramp_max(-1.0),
	V_MAX(-1.0), 
	W_MAX(-1.0),
	turningRadiusReference(0.30),
	curVelLocal(0,0,0),
	maxAllowedDirAngle(M_PI)
{ 
}

CPTG_Holo_Blend::CPTG_Holo_Blend(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) :
	turningRadiusReference(0.30),
	curVelLocal(0,0,0),
	maxAllowedDirAngle(M_PI)
{
	this->loadFromConfigFile(cfg,sSection);
}

void CPTG_Holo_Blend::updateCurrentRobotVel(const mrpt::math::TTwist2D &curVelLocal)
{
	this->curVelLocal = curVelLocal;
}

void CPTG_Holo_Blend::loadDefaultParams()
{
	CParameterizedTrajectoryGenerator::loadDefaultParams();
	CPTG_RobotShape_Circular::loadDefaultParams();

	m_alphaValuesCount = 100;
	T_ramp_max = 0.9;
	V_MAX = 1.0;
	W_MAX = mrpt::utils::DEG2RAD(120);
	maxAllowedDirAngle = M_PI;
}

void CPTG_Holo_Blend::loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection)
{
	CParameterizedTrajectoryGenerator::loadFromConfigFile(cfg,sSection);
	CPTG_RobotShape_Circular::loadShapeFromConfigFile(cfg,sSection);

	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(T_ramp_max ,double, T_ramp_max, cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(v_max_mps  ,double, V_MAX, cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(w_max_dps  ,double, W_MAX, cfg,sSection);
	MRPT_LOAD_CONFIG_VAR(turningRadiusReference  ,double, cfg,sSection);
	MRPT_LOAD_CONFIG_VAR_DEGREES(maxAllowedDirAngle, cfg,sSection);

	// For debugging only
	MRPT_LOAD_HERE_CONFIG_VAR(vxi  ,double, curVelLocal.vx, cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(vyi  ,double, curVelLocal.vy, cfg,sSection);
}
void CPTG_Holo_Blend::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const
{
	MRPT_START
	const int WN = 40, WV = 20;

	CParameterizedTrajectoryGenerator::saveToConfigFile(cfg,sSection);

	cfg.write(sSection,"T_ramp_max",T_ramp_max,   WN,WV, "Max duration of the velocity interpolation since a vel_cmd is issued [s].");
	cfg.write(sSection,"v_max_mps",V_MAX,   WN,WV, "Maximum linear velocity for trajectories [m/s].");
	cfg.write(sSection,"w_max_dps",mrpt::utils::RAD2DEG(W_MAX),   WN,WV, "Maximum angular velocity for trajectories [deg/s].");
	cfg.write(sSection,"turningRadiusReference",turningRadiusReference,   WN,WV, "An approximate dimension of the robot (not a critical parameter) [m].");
	cfg.write(sSection,"maxAllowedDirAngle",mrpt::utils::RAD2DEG(maxAllowedDirAngle),   WN,WV, "Maximum allowed angle between heading and motion direction [deg].");

	cfg.write(sSection,"vxi",curVelLocal.vx,   WN,WV, "(Only for debugging) Current robot velocity vx [m/s].");
	cfg.write(sSection,"vyi",curVelLocal.vy,   WN,WV, "(Only for debugging) Current robot velocity vy [m/s].");
	CPTG_RobotShape_Circular::saveToConfigFile(cfg,sSection);

	MRPT_END
}


std::string CPTG_Holo_Blend::getDescription() const
{
	return mrpt::format("PTG_Holo_Blend_Tramp=%.03f_Vmax=%.03f_Wmax=%.03f_MaxAng=%.02f",T_ramp_max,V_MAX,W_MAX,maxAllowedDirAngle);
}


void CPTG_Holo_Blend::readFromStream(mrpt::utils::CStream &in, int version)
{
	CParameterizedTrajectoryGenerator::internal_readFromStream(in);

	switch (version)
	{
	case 0:
	case 1:
	case 2:
		if (version>=1) {
			CPTG_RobotShape_Circular::internal_shape_loadFromStream(in);
		}

		in >> T_ramp_max >> V_MAX >> W_MAX >> turningRadiusReference;
		if (version>=2)
			in >> maxAllowedDirAngle;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPTG_Holo_Blend::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version) 
	{
		*version = 2;
		return;
	}

	CParameterizedTrajectoryGenerator::internal_writeToStream(out);
	CPTG_RobotShape_Circular::internal_shape_saveToStream(out);

	out << T_ramp_max << V_MAX << W_MAX << turningRadiusReference;
	out << maxAllowedDirAngle; // v2
}

bool CPTG_Holo_Blend::inverseMap_WS2TP(double x, double y, int &out_k, double &out_d, double tolerance_dist) const
{
	PERFORMANCE_BENCHMARK;

	MRPT_UNUSED_PARAM(tolerance_dist);
	ASSERT_(x!=0 || y!=0);
	
	const double err_threshold = 1e-3;

	const double vxi = curVelLocal.vx, vyi = curVelLocal.vy;
	const double V_MAXsq = V_MAX*V_MAX;

	// Use a Newton iterative non-linear optimizer to find the "exact" solution for (t,alpha) 
	// in each case: (1) t<T_ramp and (2) t>T_ramp

	// Initial value:
	Eigen::Vector3d  q; // [t vxf vyf]
	q[0]=T_ramp_max*1.1;
	q[1]=V_MAX*x/sqrt(x*x+y*y);
	q[2]=V_MAX*y/sqrt(x*x+y*y);

	// Iterate: case (2) t > T_ramp
	double err_mod=1e7;
	bool sol_found = false;
	for (int iters=0;!sol_found && iters<25;iters++)
	{
		const double T_ramp = calc_T_ramp(T_ramp_max, vxi,vyi, q[1], q[2], V_MAX);
		const double TR_ =  1.0/(T_ramp);
		const double TR2_ = 1.0/(2*T_ramp);

		// Eval residual:
		Eigen::Vector3d  r;
		if (q[0]>=T_ramp)
		{
			r[0] = 0.5*T_ramp *( vxi + q[1] ) + (q[0]-T_ramp)*q[1] - x;
			r[1] = 0.5*T_ramp *( vyi + q[2] ) + (q[0]-T_ramp)*q[2]   - y;
			r[2] = q[1]*q[1]+q[2]*q[2]   - V_MAXsq;
		}
		else
		{
			r[0] = vxi * q[0] + q[0]*q[0] * TR2_ * (q[1]-vxi)   - x;
			r[1] = vyi * q[0] + q[0]*q[0] * TR2_ * (q[2]-vyi)   - y;
			r[2] = q[1]*q[1]+q[2]*q[2]   - V_MAXsq;
		}

		// Jacobian:
		//  dx/dt  dx/dvxf  dx/dvyf
		//  dy/dt  dy/dvxf  dy/dvyf
		//  dVF/dt  dVF/dvxf  dVF/dvyf
		Eigen::Matrix3d J;
		if (q[0]>=T_ramp)
		{
			J(0,0) = q[1];  J(0,1) = 0.5*T_ramp+q[0]; J(0,2) = 0.0;
			J(1,0) = q[2];  J(1,1) = 0.0;             J(1,2) = 0.5*T_ramp+q[0];
			J(2,0) = 0.0; J(2,1) = 2*q[1]; J(2,2) = 2*q[2];
		}
		else
		{
			J(0,0) = vxi + q[0]*TR_*(q[1]-vxi);  J(0,1) = TR2_*q[0]*q[0];   J(0,2) = 0.0;
			J(1,0) = vyi + q[0]*TR_*(q[2]-vyi);  J(1,1) = 0.0;              J(1,2) = TR2_*q[0]*q[0];
			J(2,0) = 0.0;                        J(2,1) = 2*q[1];           J(2,2) = 2*q[2];
		}

		Eigen::Vector3d q_incr = J.householderQr().solve(r);
		q-=q_incr;

		err_mod = r.norm();
		sol_found = (err_mod<err_threshold);
	}

	if (sol_found && q[0]>=.0)
	{
		const double alpha = atan2(q[2],q[1]);
		out_k =  CParameterizedTrajectoryGenerator::alpha2index( alpha );
		
		const double solved_t = q[0];
		const unsigned int solved_step = solved_t/PATH_TIME_STEP;
		const double found_dist = this->getPathDist(out_k, solved_step);

		out_d = found_dist / this->refDistance;
		return true;
	}
	else {
		return false;
	}
}

bool CPTG_Holo_Blend::PTG_IsIntoDomain(double x, double y ) const
{
	int k; 
	double d;
	return inverseMap_WS2TP(x,y,k,d);
}

void CPTG_Holo_Blend::internal_initialize(const std::string & cacheFilename, const bool verbose )
{
	// No need to initialize anything, just do some params sanity checks:
	ASSERT_(T_ramp_max>0);
	ASSERT_(V_MAX>0);
	ASSERT_(W_MAX>0);
	ASSERT_(m_alphaValuesCount>0);
	ASSERT_(m_robotRadius>0);
	ASSERT_(maxAllowedDirAngle>.0);

#ifdef DO_PERFORMANCE_BENCHMARK
	tl.dumpAllStats();
#endif
}

void CPTG_Holo_Blend::internal_deinitialize()
{
	// Nothing to do in a closed-form PTG.
}

void CPTG_Holo_Blend::directionToMotionCommand( uint16_t k, std::vector<double> &cmd_vel ) const
{
	const double dir_local = CParameterizedTrajectoryGenerator::index2alpha(k);

	// cmd_vel=[vel dir_local ramp_time rot_speed]:
	cmd_vel.resize(4);
	if (std::abs(dir_local)<=maxAllowedDirAngle) {
		cmd_vel[0] = V_MAX;
	} else {
		cmd_vel[0] = .0;
	}
	cmd_vel[1] = dir_local;
	cmd_vel[2] = std::max( 0.1, calc_T_ramp_dir(T_ramp_max, curVelLocal.vx, curVelLocal.vy, dir_local, V_MAX) );
	cmd_vel[3] = mrpt::utils::saturate_val(W_MAX * dir_local / (0.5*M_PI), -W_MAX, W_MAX);
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

	const double vxf = V_MAX * cos(dir), vyf = V_MAX * sin(dir);
	const double vxi = curVelLocal.vx, vyi = curVelLocal.vy;
	const double T_ramp = calc_T_ramp(T_ramp_max,vxi,vyi,vxf,vyf,V_MAX);
	const double TR2_ = 1.0/(2*T_ramp);

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

	const double vxf = V_MAX * cos(dir), vyf = V_MAX * sin(dir);
	const double vxi = curVelLocal.vx, vyi = curVelLocal.vy;
	const double T_ramp = calc_T_ramp(T_ramp_max,vxi,vyi,vxf,vyf,V_MAX);
	const double TR2_ = 1.0/(2*T_ramp);

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
	PERFORMANCE_BENCHMARK;

	const double dir = CParameterizedTrajectoryGenerator::index2alpha(k);

	const double vxf = V_MAX * cos(dir), vyf = V_MAX * sin(dir);
	const double vxi = curVelLocal.vx, vyi = curVelLocal.vy;
	const double T_ramp = calc_T_ramp(T_ramp_max,vxi,vyi,vxf,vyf,V_MAX);
	const double TR2_ = 1.0/(2*T_ramp);

	const double k2 = (vxf-vxi)*TR2_;
	const double k4 = (vyf-vyi)*TR2_;

	// --------------------------------------
	// Solution within  t >= T_ramp ??
	// --------------------------------------
	const double dist_trans_T_ramp = calc_trans_distance_t_below_Tramp(k2,k4,vxi,vyi,T_ramp);
	double t_solved = -1;
		
	if (dist>=dist_trans_T_ramp)
	{
		// Good solution:
		t_solved = T_ramp + (dist-dist_trans_T_ramp)/V_MAX;
	}
	else
	{
		// ------------------------------------
		// Solutions within t < T_ramp
		//
		// Cases:
		// 1) k2=k4=0  --> vi=vf. Path is straight line
		// 2) b=c=0     -> vi=0
		// 3) Otherwise, general case
		// ------------------------------------
		if (std::abs(k2)<eps && std::abs(k4)<eps)
		{
			// Case 1
			t_solved = (dist)/V_MAX;
		}
		else
		{
			const double a = ((k2*k2)*4.0+(k4*k4)*4.0);
			const double b = (k2*vxi*4.0+k4*vyi*4.0);
			const double c = (vxi*vxi+vyi*vyi);

			// Numerically-ill case: b=c=0 (initial vel=0)
			if (std::abs(b)<eps && std::abs(c)<eps)
			{
				// Case 2:
				t_solved = sqrt(2.0)*1.0/pow(a,1.0/4.0)*sqrt(dist);
			}
			else
			{
				// Case 3: general case with non-linear equation:
				// dist = (t/2 + b/(4*a))*(a*t^2 + b*t + c)^(1/2) - (b*c^(1/2))/(4*a) + (log((b/2 + a*t)/a^(1/2) + (a*t^2 + b*t + c)^(1/2))*(- b^2/4 + a*c))/(2*a^(3/2)) - (log((b + 2*a^(1/2)*c^(1/2))/(2*a^(1/2)))*(- b^2/4 + a*c))/(2*a^(3/2))
				// dist = (t*(1.0/2.0)+(b*(1.0/4.0))/a)*sqrt(c+b*t+a*(t*t))-(b*sqrt(c)*(1.0/4.0))/a+1.0/pow(a,3.0/2.0)*log(1.0/sqrt(a)*(b*(1.0/2.0)+a*t)+sqrt(c+b*t+a*(t*t)))*(a*c-(b*b)*(1.0/4.0))*(1.0/2.0)-1.0/pow(a,3.0/2.0)*log(1.0/sqrt(a)*(b+sqrt(a)*sqrt(c)*2.0)*(1.0/2.0))*(a*c-(b*b)*(1.0/4.0))*(1.0/2.0);

				// We must solve this by iterating:
				// Newton method:
				// Minimize f(t)-dist = 0
				//  with: f(t)=calc_trans_distance_t_below_Tramp_abc(t)
				//  and:  f'(t) = sqrt(a*t^2+b*t+c)

				t_solved = T_ramp*0.6; // Initial value for starting interation inside the valid domain of the function t=[0,T_ramp]
				for (int iters=0;iters<10;iters++)
				{
					double err = calc_trans_distance_t_below_Tramp_abc(t_solved,a,b,c) - dist;
					const double diff = std::sqrt(a*t_solved*t_solved+b*t_solved+c);
					ASSERT_(std::abs(diff)>1e-40);
					t_solved -= (err) / diff;
					if (t_solved<0) 
						t_solved=.0;
					if (std::abs(err)<1e-3)
						break; // Good enough!
				}
			}
		}
	}
	if (t_solved>=0)
	{
		out_step = mrpt::utils::round( t_solved / PATH_TIME_STEP );
		return true;
	}
	else return false;
}


void CPTG_Holo_Blend::updateTPObstacle(double ox, double oy, std::vector<double> &tp_obstacles) const
{
	PERFORMANCE_BENCHMARK;

	const double R = m_robotRadius;

	const double vxi = curVelLocal.vx, vyi = curVelLocal.vy;

	for (unsigned int k=0;k<m_alphaValuesCount;k++)
	{
		const double dir = CParameterizedTrajectoryGenerator::index2alpha(k);
		const double vxf = V_MAX * cos(dir), vyf = V_MAX * sin(dir);
		const double T_ramp = calc_T_ramp(T_ramp_max,vxi,vyi,vxf,vyf,V_MAX);
		const double TR2_ = 1.0/(2*T_ramp);
		const double TR_2 = T_ramp*0.5;
		const double T_ramp_thres099 = T_ramp*0.99;
		const double T_ramp_thres101 = T_ramp*1.01;

		double sol_t=-1.0; // candidate solution for shortest time to collision

		// Try to solve first for t<T_ramp:
		const double k2 = (vxf-vxi)*TR2_;
		const double k4 = (vyf-vyi)*TR2_;

		// equation: a*t^4 + b*t^3 + c*t^2 + d*t + e = 0
		const double a = (k2*k2+k4*k4);
		const double b = (k2*vxi*2.0+k4*vyi*2.0);
		const double c = -(k2*ox*2.0+k4*oy*2.0-vxi*vxi-vyi*vyi);
		const double d = -(ox*vxi*2.0+oy*vyi*2.0);
		const double e = -R*R+ox*ox+oy*oy;

		double roots[4];
		int num_real_sols=0;
		if (std::abs(a)>eps) 
		{
			// General case: 4th order equation
			// a * x^4 + b * x^3 + c * x^2 + d * x + e
			num_real_sols = mrpt::math::solve_poly4(roots, b/a,c/a,d/a,e/a);
		}
		else if (std::abs(b)>eps) {
			// Special case: k2=k4=0 (straight line path, no blend)
			// 3rd order equation: 
			// b * x^3 + c * x^2 + d * x + e
			num_real_sols = mrpt::math::solve_poly3(roots, c/b,d/b,e/b);
		} else 
		{
			// Special case: 2nd order equation (a=b=0)
			const double discr = d*d-4*c*e;  // c*t^2 + d*t + e = 0
			if (discr>=0)
			{
				num_real_sols = 2;
				roots[0] = (-d+sqrt(discr))/(2*c);
				roots[1] = (-d-sqrt(discr))/(2*c);
			} else {
				num_real_sols = 0;
			}
		}

		for (int i=0;i<num_real_sols;i++)
		{
			if (roots[i]==roots[i] && // not NaN
				mrpt::math::isFinite(roots[i]) &&
				roots[i]>=.0 && 
				roots[i]<=T_ramp*1.01)
			{
				if (sol_t<0) sol_t=roots[i];
				else mrpt::utils::keep_min(sol_t, roots[i]);
			}
		}

		// Invalid with these equations?
		if (sol_t<0 || sol_t>T_ramp_thres101)
		{
			// Attempt to solve with the equations for t<T_ramp
			sol_t = -1.0;

			// Solve for t>T_ramp:
			const double c1 = TR_2*(vxi-vxf)-ox;
			const double c2 = TR_2*(vyi-vyf)-oy;

			const double a = V_MAX*V_MAX;
			const double b = 2*(c1*vxf+c2*vyf);
			const double c = c1*c1+c2*c2-R*R;

			const double discr = b*b-4*a*c;
			if (discr>=0)
			{
				const double sol_t0 = (-b+sqrt(discr))/(2*a);
				const double sol_t1 = (-b-sqrt(discr))/(2*a);

				// Identify the shortest valid colission time:
				if (sol_t0<T_ramp && sol_t1<T_ramp) sol_t = -1.0;
				else if (sol_t0<T_ramp && sol_t1>=T_ramp_thres099) sol_t = sol_t1;
				else if (sol_t1<T_ramp && sol_t0>=T_ramp_thres099) sol_t = sol_t0;
				else if (sol_t1>=T_ramp_thres099 && sol_t0>=T_ramp_thres099) sol_t = std::min(sol_t0,sol_t1);
			}
		}

		// Valid solution?
		if (sol_t<0) continue;
		// Compute the transversed distance:
		double dist;

		if (sol_t<T_ramp)
		     dist = calc_trans_distance_t_below_Tramp(k2,k4,vxi,vyi,sol_t);
		else dist = (sol_t-T_ramp) * V_MAX + calc_trans_distance_t_below_Tramp(k2,k4,vxi,vyi,T_ramp);

		mrpt::utils::keep_min( tp_obstacles[k], dist);  // TP_Obstacles buffer in un-normalized distances: use "dist" directly

	} // end for each "k" alpha
}

void CPTG_Holo_Blend::internal_processNewRobotShape()
{
	// Nothing to do in a closed-form PTG.
}


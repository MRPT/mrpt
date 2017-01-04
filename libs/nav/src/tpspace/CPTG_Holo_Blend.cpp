/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>

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

double CPTG_Holo_Blend::PATH_TIME_STEP = 10e-3;   // 10 ms
double CPTG_Holo_Blend::eps = 1e-4;               // epsilon for detecting 1/0 situation

// As a macro instead of a function (uglier) to allow for const variables (safer)
#define COMMON_PTG_DESIGN_PARAMS \
	const double vxi = curVelLocal.vx, vyi = curVelLocal.vy; \
	const double vf_mod = internal_get_v(dir); \
	const double vxf = vf_mod*cos(dir), vyf = vf_mod* sin(dir); \
	const double T_ramp = internal_get_T_ramp(dir);

// Axiliary function for calc_trans_distance_t_below_Tramp() and others:
double CPTG_Holo_Blend::calc_trans_distance_t_below_Tramp_abc(double t, double a,double b, double c)
{
	ASSERT_(t>=0);
	if (t==0.0) return .0;

	double dist;
	// Handle special case: degenerate sqrt(a*t^2+b*t+c) =  sqrt((t-r)^2) = |t-r|
	const double discr = b*b-4*a*c;
	if (std::abs(discr)<1e-6)
	{
		const double r = -b/(2*a);
		// dist= definite integral [0,t] of: |t-r| dt
		dist = r*std::abs(r)*0.5 + (t - r)*std::abs(t - r)*0.5;
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
double CPTG_Holo_Blend::calc_trans_distance_t_below_Tramp(double k2, double k4, double vxi,double vyi, double t)
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

void CPTG_Holo_Blend::updateCurrentRobotVel(const mrpt::math::TTwist2D &curVelLocal)
{
	this->curVelLocal = curVelLocal;
}

void CPTG_Holo_Blend::loadDefaultParams()
{
	CParameterizedTrajectoryGenerator::loadDefaultParams();
	CPTG_RobotShape_Circular::loadDefaultParams();

	m_alphaValuesCount = 31;
	T_ramp_max = 0.9;
	V_MAX = 1.0;
	W_MAX = mrpt::utils::DEG2RAD(40);
}

void CPTG_Holo_Blend::loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection)
{
	CParameterizedTrajectoryGenerator::loadFromConfigFile(cfg,sSection);
	CPTG_RobotShape_Circular::loadShapeFromConfigFile(cfg,sSection);

	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(T_ramp_max ,double, T_ramp_max, cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(v_max_mps  ,double, V_MAX, cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(w_max_dps  ,double, W_MAX, cfg,sSection);
	MRPT_LOAD_CONFIG_VAR(turningRadiusReference  ,double, cfg,sSection);

	// For debugging only
	MRPT_LOAD_HERE_CONFIG_VAR(vxi  ,double, curVelLocal.vx, cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(vyi  ,double, curVelLocal.vy, cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES(wi   ,double, curVelLocal.omega, cfg, sSection);

	MRPT_LOAD_HERE_CONFIG_VAR(expr_V, string, expr_V, cfg, sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(expr_W, string, expr_W, cfg, sSection);
	MRPT_LOAD_HERE_CONFIG_VAR(expr_T_ramp, string, expr_T_ramp, cfg, sSection);
}
void CPTG_Holo_Blend::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const
{
	MRPT_START
	const int WN = 25, WV = 30;

	CParameterizedTrajectoryGenerator::saveToConfigFile(cfg,sSection);

	cfg.write(sSection,"T_ramp_max",T_ramp_max,   WN,WV, "Max duration of the velocity interpolation since a vel_cmd is issued [s].");
	cfg.write(sSection,"v_max_mps",V_MAX,   WN,WV, "Maximum linear velocity for trajectories [m/s].");
	cfg.write(sSection,"w_max_dps",mrpt::utils::RAD2DEG(W_MAX),   WN,WV, "Maximum angular velocity for trajectories [deg/s].");
	cfg.write(sSection,"turningRadiusReference",turningRadiusReference,   WN,WV, "An approximate dimension of the robot (not a critical parameter) [m].");

	cfg.write(sSection,"vxi",curVelLocal.vx,   WN,WV, "(Only for debugging) Current robot velocity vx [m/s].");
	cfg.write(sSection,"vyi",curVelLocal.vy,   WN,WV, "(Only for debugging) Current robot velocity vy [m/s].");
	cfg.write(sSection, "wi", mrpt::utils::RAD2DEG(curVelLocal.omega),WN,WV,"(Only for debugging) Current robot velocity omega [deg/s].");

	cfg.write(sSection, "expr_V", expr_V, WN, WV, "Math expr for |V| as a function of `dir`,`V_MAX`,`W_MAX`,`T_ramp_max`.");
	cfg.write(sSection, "expr_W", expr_W, WN, WV, "Math expr for |omega| (disregarding the sign, only the module) as a function of `dir`,`V_MAX`,`W_MAX`,`T_ramp_max`.");
	cfg.write(sSection, "expr_T_ramp", expr_T_ramp, WN, WV, "Math expr for `T_ramp` as a function of `dir`,`V_MAX`,`W_MAX`,`T_ramp_max`.");

	CPTG_RobotShape_Circular::saveToConfigFile(cfg,sSection);

	MRPT_END
}


std::string CPTG_Holo_Blend::getDescription() const
{
	return mrpt::format("PTG_Holo_Blend_Tramp=%.03f_Vmax=%.03f_Wmax=%.03f",T_ramp_max,V_MAX,W_MAX);
}


void CPTG_Holo_Blend::readFromStream(mrpt::utils::CStream &in, int version)
{
	CParameterizedTrajectoryGenerator::internal_readFromStream(in);

	switch (version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		if (version>=1) {
			CPTG_RobotShape_Circular::internal_shape_loadFromStream(in);
		}

		in >> T_ramp_max >> V_MAX >> W_MAX >> turningRadiusReference;
		if (version==2) {
			double dummy_maxAllowedDirAngle; // removed in v3
			in >> dummy_maxAllowedDirAngle;
		}
		if (version >= 4) {
			in >> expr_V >> expr_W >> expr_T_ramp;
		}
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPTG_Holo_Blend::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
	{
		*version = 4;
		return;
	}

	CParameterizedTrajectoryGenerator::internal_writeToStream(out);
	CPTG_RobotShape_Circular::internal_shape_saveToStream(out);

	out << T_ramp_max << V_MAX << W_MAX << turningRadiusReference;
	out << expr_V << expr_W << expr_T_ramp;
}

bool CPTG_Holo_Blend::inverseMap_WS2TP(double x, double y, int &out_k, double &out_d, double tolerance_dist) const
{
	PERFORMANCE_BENCHMARK;

	MRPT_UNUSED_PARAM(tolerance_dist);
	ASSERT_(x!=0 || y!=0);

	const double err_threshold = 1e-3;
	const double T_ramp = T_ramp_max;
	const double vxi = curVelLocal.vx, vyi = curVelLocal.vy;

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
		const double TR_ =  1.0/(T_ramp);
		const double TR2_ = 1.0/(2*T_ramp);

		// Eval residual:
		Eigen::Vector3d  r;
		if (q[0]>=T_ramp)
		{
			r[0] = 0.5*T_ramp *( vxi + q[1] ) + (q[0]-T_ramp)*q[1] - x;
			r[1] = 0.5*T_ramp *( vyi + q[2] ) + (q[0]-T_ramp)*q[2]   - y;
		}
		else
		{
			r[0] = vxi * q[0] + q[0]*q[0] * TR2_ * (q[1]-vxi)   - x;
			r[1] = vyi * q[0] + q[0]*q[0] * TR2_ * (q[2]-vyi)   - y;
		}
		const double alpha = atan2(q[2], q[1]);
		const double V_MAXsq = mrpt::utils::square(this->internal_get_v(alpha));
		r[2] = q[1]*q[1]+q[2]*q[2] - V_MAXsq;

		// Jacobian: q=[t vxf vyf]   q0=t   q1=vxf   q2=vyf
		//  dx/dt  dx/dvxf  dx/dvyf
		//  dy/dt  dy/dvxf  dy/dvyf
		//  dVF/dt  dVF/dvxf  dVF/dvyf
		Eigen::Matrix3d J;
		if (q[0]>=T_ramp)
		{
			J(0,0) = q[1];  J(0,1) = 0.5*T_ramp+q[0]; J(0,2) = 0.0;
			J(1,0) = q[2];  J(1,1) = 0.0;             J(1,2) = 0.5*T_ramp+q[0];
		}
		else
		{
			J(0,0) = vxi + q[0]*TR_*(q[1]-vxi);  J(0,1) = TR2_*q[0]*q[0];   J(0,2) = 0.0;
			J(1,0) = vyi + q[0]*TR_*(q[2]-vyi);  J(1,1) = 0.0;              J(1,2) = TR2_*q[0]*q[0];
		}
		J(2,0) = 0.0; J(2,1) = 2*q[1]; J(2,2) = 2*q[2];

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

void CPTG_Holo_Blend::internal_deinitialize()
{
	// Nothing to do in a closed-form PTG.
}

mrpt::kinematics::CVehicleVelCmdPtr CPTG_Holo_Blend::directionToMotionCommand( uint16_t k) const
{
	const double dir_local = CParameterizedTrajectoryGenerator::index2alpha(k);

	mrpt::kinematics::CVehicleVelCmd_Holo * cmd = new mrpt::kinematics::CVehicleVelCmd_Holo();
	cmd->vel = internal_get_v(dir_local);
	cmd->dir_local = dir_local;
	cmd->ramp_time = internal_get_T_ramp(dir_local);
	cmd->rot_speed = mrpt::utils::signWithZero(dir_local) * internal_get_w(dir_local);

	return mrpt::kinematics::CVehicleVelCmdPtr(cmd);
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
	COMMON_PTG_DESIGN_PARAMS;
	const double wf = mrpt::utils::signWithZero(dir) * this->internal_get_w(dir);
	const double TR2_ = 1.0/(2*T_ramp);

	// Translational part:
	if (t<T_ramp)
	{
		p.x = vxi * t + t*t * TR2_ * (vxf - vxi);
		p.y = vyi * t + t*t * TR2_ * (vyf - vyi);
	}
	else
	{
		p.x = T_ramp *0.5*(vxi + vxf) + (t - T_ramp) * vxf;
		p.y = T_ramp *0.5*(vyi + vyf) + (t - T_ramp) * vyf;
	}

	// Rotational part:
	const double wi = curVelLocal.omega;

	if (t<T_ramp)
	{
		// Time required to align completed?
		const double a = TR2_ * (wf - wi), b = (wi), c = -dir;

		// Solves equation `a*x^2 + b*x + c = 0`.
		double r1, r2;
		int nroots = mrpt::math::solve_poly2(a,b,c, r1,r2);
		if (nroots!=2) {
			p.phi = .0; // typical case: wi=wf=0
		} else  {
			const double t_solve = std::max(r1,r2);
			if (t > t_solve)
				p.phi = dir;
			else
				p.phi = wi*t + t*t* TR2_ * (wf-wi);
		}
	}
	else
	{
		// Time required to align completed?
		const double t_solve = (dir - T_ramp *0.5*(wi + wf))/wf + T_ramp;
		if (t > t_solve)
			p.phi = dir;
		else
			p.phi = T_ramp *0.5*(wi + wf) + (t - T_ramp) * wf;
	}
}

double CPTG_Holo_Blend::getPathDist(uint16_t k, uint16_t step) const
{
	const double t = PATH_TIME_STEP*step;
	const double dir = CParameterizedTrajectoryGenerator::index2alpha(k);

	COMMON_PTG_DESIGN_PARAMS;
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
	COMMON_PTG_DESIGN_PARAMS;

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


void CPTG_Holo_Blend::updateTPObstacleSingle(double ox, double oy, uint16_t k, double &tp_obstacle_k) const
{
	const double R = m_robotRadius;
	const double dir = CParameterizedTrajectoryGenerator::index2alpha(k);
	COMMON_PTG_DESIGN_PARAMS;

	const double TR2_ = 1.0 / (2 * T_ramp);
	const double TR_2 = T_ramp*0.5;
	const double T_ramp_thres099 = T_ramp*0.99;
	const double T_ramp_thres101 = T_ramp*1.01;

	double sol_t = -1.0; // candidate solution for shortest time to collision

	// Note: It's tempting to try to solve first for t>T_ramp because it has simpler (faster) equations, 
	// but there are cases in which we will have valid collisions for t>T_ramp but other valid ones 
	// for t<T_ramp as well, so the only SAFE way to detect shortest distances is to check over increasing values of "t".

	// Try to solve first for t<T_ramp:
	const double k2 = (vxf - vxi)*TR2_;
	const double k4 = (vyf - vyi)*TR2_;

	// equation: a*t^4 + b*t^3 + c*t^2 + d*t + e = 0
	const double a = (k2*k2 + k4*k4);
	const double b = (k2*vxi*2.0 + k4*vyi*2.0);
	const double c = -(k2*ox*2.0 + k4*oy*2.0 - vxi*vxi - vyi*vyi);
	const double d = -(ox*vxi*2.0 + oy*vyi*2.0);
	const double e = -R*R + ox*ox + oy*oy;

	double roots[4];
	int num_real_sols = 0;
	if (std::abs(a)>eps)
	{
		// General case: 4th order equation
		// a * x^4 + b * x^3 + c * x^2 + d * x + e
		num_real_sols = mrpt::math::solve_poly4(roots, b / a, c / a, d / a, e / a);
	}
	else if (std::abs(b)>eps) {
		// Special case: k2=k4=0 (straight line path, no blend)
		// 3rd order equation:
		// b * x^3 + c * x^2 + d * x + e
		num_real_sols = mrpt::math::solve_poly3(roots, c / b, d / b, e / b);
	}
	else
	{
		// Special case: 2nd order equation (a=b=0)
		const double discr = d*d - 4 * c*e;  // c*t^2 + d*t + e = 0
		if (discr >= 0)
		{
			num_real_sols = 2;
			roots[0] = (-d + sqrt(discr)) / (2 * c);
			roots[1] = (-d - sqrt(discr)) / (2 * c);
		}
		else {
			num_real_sols = 0;
		}
	}

	for (int i = 0; i<num_real_sols; i++)
	{
		if (roots[i] == roots[i] && // not NaN
			mrpt::math::isFinite(roots[i]) &&
			roots[i] >= .0 &&
			roots[i] <= T_ramp*1.01)
		{
			if (sol_t<0) sol_t = roots[i];
			else mrpt::utils::keep_min(sol_t, roots[i]);
		}
	}

	// Invalid with these equations?
	if (sol_t<0 || sol_t>T_ramp_thres101)
	{
		// Now, attempt to solve with the equations for t>T_ramp:
		sol_t = -1.0;

		const double c1 = TR_2*(vxi - vxf) - ox;
		const double c2 = TR_2*(vyi - vyf) - oy;

		const double a = vf_mod*vf_mod;
		const double b = 2 * (c1*vxf + c2*vyf);
		const double c = c1*c1 + c2*c2 - R*R;

		const double discr = b*b - 4 * a*c;
		if (discr >= 0)
		{
			const double sol_t0 = (-b + sqrt(discr)) / (2 * a);
			const double sol_t1 = (-b - sqrt(discr)) / (2 * a);

			// Identify the shortest valid collision time:
			if (sol_t0<T_ramp && sol_t1<T_ramp) sol_t = -1.0;
			else if (sol_t0<T_ramp && sol_t1 >= T_ramp_thres099) sol_t = sol_t1;
			else if (sol_t1<T_ramp && sol_t0 >= T_ramp_thres099) sol_t = sol_t0;
			else if (sol_t1 >= T_ramp_thres099 && sol_t0 >= T_ramp_thres099) sol_t = std::min(sol_t0, sol_t1);
		}
	}

	// Valid solution?
	if (sol_t<0) return;
	// Compute the transversed distance:
	double dist;

	if (sol_t<T_ramp)
		dist = calc_trans_distance_t_below_Tramp(k2, k4, vxi, vyi, sol_t);
	else dist = (sol_t - T_ramp) * V_MAX + calc_trans_distance_t_below_Tramp(k2, k4, vxi, vyi, T_ramp);

	// Store in the output variable:
	internal_TPObsDistancePostprocess(ox, oy, dist, tp_obstacle_k);
}

void CPTG_Holo_Blend::updateTPObstacle(double ox, double oy, std::vector<double> &tp_obstacles) const
{
	PERFORMANCE_BENCHMARK;

	for (unsigned int k = 0; k < m_alphaValuesCount; k++)
	{
		updateTPObstacleSingle(ox, oy, k, tp_obstacles[k]);
	} // end for each "k" alpha
}

void CPTG_Holo_Blend::internal_processNewRobotShape()
{
	// Nothing to do in a closed-form PTG.
}

mrpt::kinematics::CVehicleVelCmdPtr CPTG_Holo_Blend::getSupportedKinematicVelocityCommand() const
{
	return mrpt::kinematics::CVehicleVelCmdPtr(new mrpt::kinematics::CVehicleVelCmd_Holo());
}

bool CPTG_Holo_Blend::supportVelCmdNOP() const
{
	return true;
}

double CPTG_Holo_Blend::maxTimeInVelCmdNOP(int path_k) const
{
//	const double dir_local = CParameterizedTrajectoryGenerator::index2alpha(path_k);

	const size_t nSteps = getPathStepCount(path_k);
	const double max_t = nSteps * PATH_TIME_STEP;
	return max_t;
}

double CPTG_Holo_Blend::getPathStepDuration() const
{
	return PATH_TIME_STEP;
}

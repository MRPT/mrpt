/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
// ------------------------------------------------------
//  Refer to the description in the wiki:
//  http://www.mrpt.org/Kalman_Filters
// ------------------------------------------------------

#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>

#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/random.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/distributions.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservationBearingRange.h>

using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace std;

#define BEARING_SENSOR_NOISE_STD 	DEG2RAD(15.0f)
#define RANGE_SENSOR_NOISE_STD 		0.3f
#define DELTA_TIME                  	0.1f

#define VEHICLE_INITIAL_X			4.0f
#define VEHICLE_INITIAL_Y			4.0f
#define VEHICLE_INITIAL_V           1.0f
#define VEHICLE_INITIAL_W           DEG2RAD(20.0f)

#define TRANSITION_MODEL_STD_XY 	0.03f
#define TRANSITION_MODEL_STD_VXY 	0.20f

#define NUM_PARTICLES				2000

// Uncomment to save text files with grount truth vs. estimated states
//#define SAVE_GT_LOGS

// ------------------------------------------------------
//		Implementation of the system models as a EKF
// ------------------------------------------------------
class CRangeBearing :
	public mrpt::bayes::CKalmanFilterCapable<4 /* x y vx vy*/, 2 /* range yaw */, 0               , 1 /* Atime */>
								   // <size_t VEH_SIZE,        size_t OBS_SIZE,   size_t FEAT_SIZE, size_t ACT_SIZE, size typename kftype = double>
{
public:
	CRangeBearing( );
	virtual ~CRangeBearing();

	void  doProcess( double DeltaTime, double observationRange, double observationBearing );

	void getState( KFVector &xkk, KFMatrix &pkk)
	{
		xkk = m_xkk;
		pkk = m_pkk;
	}

 protected:

	float m_obsBearing,m_obsRange;
	float m_deltaTime;

	/** @name Virtual methods for Kalman Filter implementation
		@{
	 */

	/** Must return the action vector u.
	  * \param out_u The action vector which will be passed to OnTransitionModel
	  */
	void OnGetAction( KFArray_ACT &out_u ) const;

	/** Implements the transition model \f$ \hat{x}_{k|k-1} = f( \hat{x}_{k-1|k-1}, u_k ) \f$
	  * \param in_u The vector returned by OnGetAction.
	  * \param inout_x At input has \f$ \hat{x}_{k-1|k-1} \f$, at output must have \f$ \hat{x}_{k|k-1} \f$.
	  * \param out_skip Set this to true if for some reason you want to skip the prediction step (to do not modify either the vector or the covariance). Default:false
	  */
	void OnTransitionModel(
		const KFArray_ACT &in_u,
		KFArray_VEH       &inout_x,
		bool &out_skipPrediction
		 ) const;

	/** Implements the transition Jacobian \f$ \frac{\partial f}{\partial x} \f$
	  * \param out_F Must return the Jacobian.
	  *  The returned matrix must be \f$N \times N\f$ with N being either the size of the whole state vector or get_vehicle_size().
	  */
	void OnTransitionJacobian(KFMatrix_VxV  &out_F ) const;

	/** Implements the transition noise covariance \f$ Q_k \f$
	  * \param out_Q Must return the covariance matrix.
	  *  The returned matrix must be of the same size than the jacobian from OnTransitionJacobian
	  */
	void OnTransitionNoise(KFMatrix_VxV &out_Q ) const;

	/** Return the observation NOISE covariance matrix, that is, the model of the Gaussian additive noise of the sensor.
	  * \param out_R The noise covariance matrix. It might be non diagonal, but it'll usually be.
	  * \note Upon call, it can be assumed that the previous contents of out_R are all zeros.
	  */
	void OnGetObservationNoise(KFMatrix_OxO &out_R) const;

	/** This is called between the KF prediction step and the update step, and the application must return the observations and, when applicable, the data association between these observations and the current map.
	  *
	  * \param out_z N vectors, each for one "observation" of length OBS_SIZE, N being the number of "observations": how many observed landmarks for a map, or just one if not applicable.
	  * \param out_data_association An empty vector or, where applicable, a vector where the i'th element corresponds to the position of the observation in the i'th row of out_z within the system state vector (in the range [0,getNumberOfLandmarksInTheMap()-1]), or -1 if it is a new map element and we want to insert it at the end of this KF iteration.
	  * \param in_all_predictions A vector with the prediction of ALL the landmarks in the map. Note that, in contrast, in_S only comprises a subset of all the landmarks.
	  * \param in_S The full covariance matrix of the observation predictions (i.e. the "innovation covariance matrix"). This is a M·O x M·O matrix with M=length of "in_lm_indices_in_S".
	  * \param in_lm_indices_in_S The indices of the map landmarks (range [0,getNumberOfLandmarksInTheMap()-1]) that can be found in the matrix in_S.
	  *
	  *  This method will be called just once for each complete KF iteration.
	  * \note It is assumed that the observations are independent, i.e. there are NO cross-covariances between them.
	  */
	void OnGetObservationsAndDataAssociation(
		vector_KFArray_OBS			&out_z,
		mrpt::vector_int            &out_data_association,
		const vector_KFArray_OBS	&in_all_predictions,
		const KFMatrix              &in_S,
		const vector_size_t         &in_lm_indices_in_S,
		const KFMatrix_OxO          &in_R
		);

		/** Implements the observation prediction \f$ h_i(x) \f$.
		  * \param idx_landmark_to_predict The indices of the landmarks in the map whose predictions are expected as output. For non SLAM-like problems, this input value is undefined and the application should just generate one observation for the given problem.
		  * \param out_predictions The predicted observations.
		  */
		void OnObservationModel(
			const vector_size_t &idx_landmarks_to_predict,
			vector_KFArray_OBS  &out_predictions
			) const;

		/** Implements the observation Jacobians \f$ \frac{\partial h_i}{\partial x} \f$ and (when applicable) \f$ \frac{\partial h_i}{\partial y_i} \f$.
		  * \param idx_landmark_to_predict The index of the landmark in the map whose prediction is expected as output. For non SLAM-like problems, this will be zero and the expected output is for the whole state vector.
		  * \param Hx  The output Jacobian \f$ \frac{\partial h_i}{\partial x} \f$.
		  * \param Hy  The output Jacobian \f$ \frac{\partial h_i}{\partial y_i} \f$.
		  */
		void OnObservationJacobians(
			const size_t &idx_landmark_to_predict,
			KFMatrix_OxV &Hx,
			KFMatrix_OxF &Hy
			) const;

		/** Computes A=A-B, which may need to be re-implemented depending on the topology of the individual scalar components (eg, angles).
		  */
		void OnSubstractObservationVectors(KFArray_OBS &A, const KFArray_OBS &B) const;

	/** @}
	 */
};


// ---------------------------------------------------------------
//		Implementation of the system models as a Particle Filter
// ---------------------------------------------------------------
struct CParticleVehicleData
{
	float x,y, vx,vy; // Vehicle state (position & velocities)
};

class CRangeBearingParticleFilter :
	public mrpt::bayes::CParticleFilterData<CParticleVehicleData>,
	public mrpt::bayes::CParticleFilterDataImpl<CRangeBearingParticleFilter,mrpt::bayes::CParticleFilterData<CParticleVehicleData>::CParticleList>
{
public:

	 /** Update the m_particles, predicting the posterior of robot pose and map after a movement command.
	  *  This method has additional configuration parameters in "options".
	  *  Performs the update stage of the RBPF, using the sensed Sensorial Frame:
	  *
	  *   \param action This is a pointer to CActionCollection, containing the pose change the robot has been commanded.
	  *   \param observation This must be a pointer to a CSensoryFrame object, with robot sensed observations.
	  *
	  * \sa options
	  */
	void  prediction_and_update_pfStandardProposal(
		const mrpt::obs::CActionCollection	* action,
		const mrpt::obs::CSensoryFrame		* observation,
		const bayes::CParticleFilter::TParticleFilterOptions &PF_options );


	void initializeParticles(size_t  numParticles);

	/** Computes the average velocity & position
   	  */
	void getMean( float &x, float &y, float &vx, float &vy );

};



// ------------------------------------------------------
//				TestBayesianTracking
// ------------------------------------------------------
void TestBayesianTracking()
{
	randomGenerator.randomize();

	CDisplayWindowPlots		winEKF("Tracking - Extended Kalman Filter",450,400);
	CDisplayWindowPlots		winPF("Tracking - Particle Filter",450,400);

	winEKF.setPos(10,10);
	winPF.setPos(480,10);

	winEKF.axis(-2,20,-10,10); winEKF.axis_equal();
	winPF.axis(-2,20,-10,10);  winPF.axis_equal();

	// Create EKF
	// ----------------------
	CRangeBearing 	EKF;
	EKF.KF_options.method = kfEKFNaive;

	EKF.KF_options.verbosity_level = mrpt::utils::LVL_DEBUG;
	EKF.KF_options.enable_profiler = true;

	// Create PF
	// ----------------------
	CParticleFilter::TParticleFilterOptions	PF_options;
	PF_options.adaptiveSampleSize = false;
	PF_options.PF_algorithm = CParticleFilter::pfStandardProposal;
	PF_options.resamplingMethod = CParticleFilter::prSystematic;

	CRangeBearingParticleFilter  particles;
	particles.initializeParticles(NUM_PARTICLES);
	CParticleFilter	PF;
	PF.m_options = PF_options;

#ifdef SAVE_GT_LOGS
	CFileOutputStream  fo_log_ekf("log_GT_vs_EKF.txt");
	fo_log_ekf.printf("%%%% GT_X  GT_Y  EKF_MEAN_X  EKF_MEAN_Y   EKF_STD_X   EKF_STD_Y\n");
#endif

	// Init. simulation:
	// -------------------------
	float x=VEHICLE_INITIAL_X,y=VEHICLE_INITIAL_Y,phi=DEG2RAD(-180),v=VEHICLE_INITIAL_V,w=VEHICLE_INITIAL_W;
	float  t=0;

	while (winEKF.isOpen() && winPF.isOpen() && !mrpt::system::os::kbhit() )
	{
		// Update vehicle:
		x+=v*DELTA_TIME*(cos(phi)-sin(phi));
		y+=v*DELTA_TIME*(sin(phi)+cos(phi));
		phi+=w*DELTA_TIME;

		v+=1.0f*DELTA_TIME*cos(t);
		w-=0.1f*DELTA_TIME*sin(t);


		// Simulate noisy observation:
		float realBearing = atan2( y,x );
		float obsBearing = realBearing  + BEARING_SENSOR_NOISE_STD * randomGenerator.drawGaussian1D_normalized();
		printf("Real/Simulated bearing: %.03f / %.03f deg\n", RAD2DEG(realBearing), RAD2DEG(obsBearing) );

		float realRange = sqrt(square(x)+square(y));
		float obsRange = max(0.0, realRange  + RANGE_SENSOR_NOISE_STD * randomGenerator.drawGaussian1D_normalized() );
		printf("Real/Simulated range: %.03f / %.03f \n", realRange, obsRange );

		// Process with EKF:
		EKF.doProcess(DELTA_TIME,obsRange, obsBearing);

		// Process with PF:
		CSensoryFrame SF;
		CObservationBearingRangePtr obsRangeBear = CObservationBearingRange::Create();
		obsRangeBear->sensedData.resize(1);
		obsRangeBear->sensedData[0].range = obsRange;
		obsRangeBear->sensedData[0].yaw   = obsBearing;
		SF.insert( obsRangeBear );  // memory freed by SF.

		EKF.getProfiler().enter("PF:complete_step");
		PF.executeOn(particles, NULL,&SF);  // Process in the PF
		EKF.getProfiler().leave("PF:complete_step");

		// Show EKF state:
		CRangeBearing::KFVector EKF_xkk;
		CRangeBearing::KFMatrix EKF_pkk;

		EKF.getState( EKF_xkk, EKF_pkk );

		printf("Real: x:%.03f  y=%.03f heading=%.03f v=%.03f w=%.03f\n",x,y,phi,v,w);
		cout << "EKF: " << EKF_xkk << endl;

		// Show PF state:
		cout << "Particle filter ESS: " << particles.ESS() << endl;

		// Draw EKF state:
		CRangeBearing::KFMatrix   COVXY(2,2);
		COVXY(0,0) = EKF_pkk(0,0);
		COVXY(1,1) = EKF_pkk(1,1);
		COVXY(0,1) = COVXY(1,0) = EKF_pkk(0,1);

		winEKF.plotEllipse( EKF_xkk[0], EKF_xkk[1], COVXY, 3, "b-2", "ellipse_EKF" );

		// Save GT vs EKF state:
#ifdef SAVE_GT_LOGS
		// %% GT_X  GT_Y  EKF_MEAN_X  EKF_MEAN_Y   EKF_STD_X   EKF_STD_Y:
		fo_log_ekf.printf("%f %f %f %f %f %f\n",
			x,y, // Real (GT)
			EKF_xkk[0], EKF_xkk[1],
			std::sqrt(EKF_pkk(0,0)), std::sqrt(EKF_pkk(1,1))
			);
#endif

		// Draw the velocity vector:
		vector<float> vx(2),vy(2);
		vx[0] = EKF_xkk[0];  vx[1] = vx[0] + EKF_xkk[2] * 1;
		vy[0] = EKF_xkk[1];  vy[1] = vy[0] + EKF_xkk[3] * 1;
		winEKF.plot( vx,vy, "g-4", "velocityEKF" );


		// Draw PF state:
		{
			size_t i,N = particles.m_particles.size();
			vector<float>   parts_x(N),parts_y(N);
			for (i=0;i<N;i++)
			{
				parts_x[i] = particles.m_particles[i].d->x;
				parts_y[i] = particles.m_particles[i].d->y;
			}

			winPF.plot( parts_x, parts_y, "b.2", "particles" );

			// Draw PF velocities:
			float avrg_x, avrg_y, avrg_vx,avrg_vy;

			particles.getMean(avrg_x, avrg_y, avrg_vx,avrg_vy);

			vector<float> vx(2),vy(2);
			vx[0] = avrg_x;  vx[1] = vx[0] + avrg_vx * 1;
			vy[0] = avrg_y;  vy[1] = vy[0] + avrg_vy * 1;
			winPF.plot( vx,vy, "g-4", "velocityPF" );
		}

		// Draw GT:
		winEKF.plot( vector<float>(1,x), vector<float>(1,y),"k.8","plot_GT");
		winPF.plot( vector<float>(1,x), vector<float>(1,y),"k.8","plot_GT");


		// Draw noisy observations:
		vector<float>  obs_x(2),obs_y(2);
		obs_x[0] = obs_y[0] = 0;
		obs_x[1] = obsRange * cos( obsBearing );
		obs_y[1] = obsRange * sin( obsBearing );

		winEKF.plot(obs_x,obs_y,"r", "plot_obs_ray");
		winPF.plot(obs_x,obs_y,"r", "plot_obs_ray");


		// Delay:
		mrpt::system::sleep((int)(DELTA_TIME*1000));
		t+=DELTA_TIME;
	}
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestBayesianTracking();
		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}




CRangeBearing::CRangeBearing()
{
	//KF_options.method = kfEKFNaive;
	KF_options.method = kfEKFAlaDavison;

	// INIT KF STATE
	m_xkk.resize(4);	// State: (x,y,heading,v,w)
	m_xkk[0]= VEHICLE_INITIAL_X;
	m_xkk[1]= VEHICLE_INITIAL_Y;
	m_xkk[2]=-VEHICLE_INITIAL_V;
	m_xkk[3]=0;

	// Initial cov:  Large uncertainty
	m_pkk.setSize(4,4);
	m_pkk.unit();
	m_pkk(0,0)=
	m_pkk(1,1)= square( 5.0f );
	m_pkk(2,2)=
	m_pkk(3,3)= square( 1.0f );
}

CRangeBearing::~CRangeBearing()
{

}


void  CRangeBearing::doProcess( double DeltaTime, double observationRange, double observationBearing )
{
	m_deltaTime = (float)DeltaTime;
	m_obsBearing = (float)observationBearing;
	m_obsRange = (float) observationRange;

	runOneKalmanIteration();
}


/** Must return the action vector u.
  * \param out_u The action vector which will be passed to OnTransitionModel
  */
void CRangeBearing::OnGetAction( KFArray_ACT &u ) const
{
	u[0] = m_deltaTime;
}

/** Implements the transition model \f$ \hat{x}_{k|k-1} = f( \hat{x}_{k-1|k-1}, u_k ) \f$
  * \param in_u The vector returned by OnGetAction.
  * \param inout_x At input has \f$ \hat{x}_{k-1|k-1} \f$, at output must have \f$ \hat{x}_{k|k-1} \f$.
  * \param out_skip Set this to true if for some reason you want to skip the prediction step (to do not modify either the vector or the covariance). Default:false
  */
void CRangeBearing::OnTransitionModel(
	const KFArray_ACT &in_u,
	KFArray_VEH       &inout_x,
	bool &out_skipPrediction
	) const
{
	// in_u[0] : Delta time
	// in_out_x: [0]:x  [1]:y  [2]:vx  [3]: vy
	inout_x[0] += in_u[0] * inout_x[2];
	inout_x[1] += in_u[0] * inout_x[3];

}

/** Implements the transition Jacobian \f$ \frac{\partial f}{\partial x} \f$
  * \param out_F Must return the Jacobian.
  *  The returned matrix must be \f$N \times N\f$ with N being either the size of the whole state vector or get_vehicle_size().
  */
void CRangeBearing::OnTransitionJacobian(KFMatrix_VxV  &F) const
{
	F.unit();

	F(0,2) = m_deltaTime;
	F(1,3) = m_deltaTime;
}

/** Implements the transition noise covariance \f$ Q_k \f$
  * \param out_Q Must return the covariance matrix.
  *  The returned matrix must be of the same size than the jacobian from OnTransitionJacobian
  */
void CRangeBearing::OnTransitionNoise(KFMatrix_VxV &Q) const
{
	Q(0,0) =
	Q(1,1) = square( TRANSITION_MODEL_STD_XY );
	Q(2,2) =
	Q(3,3) = square( TRANSITION_MODEL_STD_VXY );
}

/** Return the observation NOISE covariance matrix, that is, the model of the Gaussian additive noise of the sensor.
* \param out_R The noise covariance matrix. It might be non diagonal, but it'll usually be.
* \note Upon call, it can be assumed that the previous contents of out_R are all zeros.
*/
void CRangeBearing::OnGetObservationNoise(KFMatrix_OxO &R) const
{
	R(0,0) = square( BEARING_SENSOR_NOISE_STD );
	R(1,1) = square( RANGE_SENSOR_NOISE_STD );
}

void CRangeBearing::OnGetObservationsAndDataAssociation(
	vector_KFArray_OBS			&out_z,
	mrpt::vector_int            &out_data_association,
	const vector_KFArray_OBS	&in_all_predictions,
	const KFMatrix              &in_S,
	const vector_size_t         &in_lm_indices_in_S,
	const KFMatrix_OxO          &in_R
	)
{
	out_z.resize(1);
	out_z[0][0] = m_obsBearing;
	out_z[0][1] = m_obsRange;

	out_data_association.clear(); // Not used
}


/** Implements the observation prediction \f$ h_i(x) \f$.
  * \param idx_landmark_to_predict The indices of the landmarks in the map whose predictions are expected as output. For non SLAM-like problems, this input value is undefined and the application should just generate one observation for the given problem.
  * \param out_predictions The predicted observations.
  */
void CRangeBearing::OnObservationModel(
	const vector_size_t       &idx_landmarks_to_predict,
	vector_KFArray_OBS	&out_predictions
	) const
{
	// predicted bearing:
	kftype x = m_xkk[0];
	kftype y = m_xkk[1];

	kftype h_bear = atan2(y,x);
	kftype h_range = sqrt(square(x)+square(y));

	// idx_landmarks_to_predict is ignored in NON-SLAM problems
	out_predictions.resize(1);
	out_predictions[0][0] = h_bear;
	out_predictions[0][1] = h_range;
}

/** Implements the observation Jacobians \f$ \frac{\partial h_i}{\partial x} \f$ and (when applicable) \f$ \frac{\partial h_i}{\partial y_i} \f$.
  * \param idx_landmark_to_predict The index of the landmark in the map whose prediction is expected as output. For non SLAM-like problems, this will be zero and the expected output is for the whole state vector.
  * \param Hx  The output Jacobian \f$ \frac{\partial h_i}{\partial x} \f$.
  * \param Hy  The output Jacobian \f$ \frac{\partial h_i}{\partial y_i} \f$.
  */
void CRangeBearing::OnObservationJacobians(
	const size_t &idx_landmark_to_predict,
	KFMatrix_OxV &Hx,
	KFMatrix_OxF &Hy
	) const
{
	// predicted bearing:
	kftype x = m_xkk[0];
	kftype y = m_xkk[1];

	Hx.zeros();
	Hx(0,0) = -y/(square(x)+square(y));
	Hx(0,1) = 1/(x*(1+square(y/x)));

	Hx(1,0) = x/sqrt(square(x)+square(y));
	Hx(1,1) = y/sqrt(square(x)+square(y));

	// Hy: Not used
}

/** Computes A=A-B, which may need to be re-implemented depending on the topology of the individual scalar components (eg, angles).
  */
void CRangeBearing::OnSubstractObservationVectors(KFArray_OBS &A, const KFArray_OBS &B) const
{
	A -= B;
	math::wrapToPiInPlace(A[0]); // The angular component
}



/** Update the m_particles, predicting the posterior of robot pose and map after a movement command.
*  This method has additional configuration parameters in "options".
*  Performs the update stage of the RBPF, using the sensed Sensorial Frame:
*
*   \param action This is a pointer to CActionCollection, containing the pose change the robot has been commanded.
*   \param observation This must be a pointer to a CSensoryFrame object, with robot sensed observations.
*
* \sa options
*/
void  CRangeBearingParticleFilter::prediction_and_update_pfStandardProposal(
		const mrpt::obs::CActionCollection	* action,
		const mrpt::obs::CSensoryFrame		* observation,
		const bayes::CParticleFilter::TParticleFilterOptions &PF_options )
{
	size_t i,N = m_particles.size();

	// Transition model:
	for (i=0;i<N;i++)
	{
		m_particles[i].d->x += DELTA_TIME*m_particles[i].d->vx + TRANSITION_MODEL_STD_XY * randomGenerator.drawGaussian1D_normalized();
		m_particles[i].d->y += DELTA_TIME*m_particles[i].d->vy + TRANSITION_MODEL_STD_XY * randomGenerator.drawGaussian1D_normalized();

		m_particles[i].d->vx += TRANSITION_MODEL_STD_VXY * randomGenerator.drawGaussian1D_normalized();
		m_particles[i].d->vy += TRANSITION_MODEL_STD_VXY * randomGenerator.drawGaussian1D_normalized();
	}

	CObservationBearingRangePtr obs = observation->getObservationByClass<CObservationBearingRange>();
	ASSERT_(obs);
	ASSERT_(obs->sensedData.size()==1);
	float obsRange = obs->sensedData[0].range;
	float obsBearing = obs->sensedData[0].yaw;

	// Update weights
	for (i=0;i<N;i++)
	{
		float predicted_range   = sqrt( square(m_particles[i].d->x)+square(m_particles[i].d->y));
		float predicted_bearing = atan2( m_particles[i].d->y, m_particles[i].d->x );

		m_particles[i].log_w +=
			log( math::normalPDF( predicted_range-obsRange, 0, RANGE_SENSOR_NOISE_STD ) ) +
			log( math::normalPDF( math::wrapToPi( predicted_bearing-obsBearing), 0, BEARING_SENSOR_NOISE_STD ) );
	}

	// Resample is automatically performed by CParticleFilter when required.
}


void  CRangeBearingParticleFilter::initializeParticles(size_t  M)
{
	clearParticles();
	m_particles.resize(M);
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
		it->d.reset( new CParticleVehicleData());

	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
	{
		(*it).d->x  = randomGenerator.drawUniform( VEHICLE_INITIAL_X - 2.0f, VEHICLE_INITIAL_X + 2.0f );
		(*it).d->y  = randomGenerator.drawUniform( VEHICLE_INITIAL_Y - 2.0f, VEHICLE_INITIAL_Y + 2.0f );

		(*it).d->vx = randomGenerator.drawGaussian1D( -VEHICLE_INITIAL_V, 0.2f );
		(*it).d->vy = randomGenerator.drawGaussian1D( 0, 0.2f );

		it->log_w	= 0;
	}

}

/** Computes the average velocity
  */
void CRangeBearingParticleFilter::getMean( float &x, float &y, float &vx, float &vy )
{
	double sumW=0;
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
		sumW+=exp( it->log_w );

	ASSERT_(sumW>0)

	x = y = vx = vy = 0;

	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
	{
		const double w = exp(it->log_w) / sumW;

		x += (float)w * (*it).d->x;
		y += (float)w * (*it).d->y;
		vx+= (float)w * (*it).d->vx;
		vy+= (float)w * (*it).d->vy;
	}
}

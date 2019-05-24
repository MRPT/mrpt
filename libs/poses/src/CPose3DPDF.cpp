/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/serialization/CArchive.h>
#include <Eigen/Dense>

using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CPose3DPDF, CSerializable, mrpt::poses)

// copy covariances: x, y
// Move covariances: phi -> yaw
template <class MAT33, class MAT66>
void cov2to3(const MAT33& c2d, MAT66& c3d)
{
	c3d.setZero();
	c3d.asEigen().template block<2, 2>(0, 0) =
		c2d.asEigen().template block<2, 2>(0, 0);
	c3d(3, 3) = c2d(2, 2);
	c3d(0, 3) = c3d(3, 0) = c2d(0, 2);
	c3d(1, 3) = c3d(3, 1) = c2d(1, 2);
}

/*---------------------------------------------------------------
					copyFrom2D
  ---------------------------------------------------------------*/
CPose3DPDF* CPose3DPDF::createFrom2D(const CPosePDF& o)
{
	MRPT_START

	if (o.GetRuntimeClass() == CLASS_ID(CPosePDFGaussian))
	{
		auto* newObj = new CPose3DPDFGaussian();
		const auto* obj = dynamic_cast<const CPosePDFGaussian*>(&o);
		ASSERT_(obj != nullptr);
		newObj->mean = CPose3D(obj->mean);
		cov2to3(obj->cov, newObj->cov);
		return newObj;
	}
	else if (o.GetRuntimeClass() == CLASS_ID(CPosePDFGaussianInf))
	{
		auto* newObj = new CPose3DPDFGaussianInf();
		const auto* obj = dynamic_cast<const CPosePDFGaussianInf*>(&o);
		ASSERT_(obj != nullptr);

		newObj->mean = CPose3D(obj->mean);
		cov2to3(obj->cov_inv, newObj->cov_inv);
		return newObj;
	}
	else if (o.GetRuntimeClass() == CLASS_ID(CPosePDFParticles))
	{
		const auto* obj = dynamic_cast<const CPosePDFParticles*>(&o);
		ASSERT_(obj != nullptr);
		auto* newObj = new CPose3DPDFParticles(obj->size());

		CPosePDFParticles::CParticleList::const_iterator it1;
		CPose3DPDFParticles::CParticleList::iterator it2;
		for (it1 = obj->m_particles.begin(), it2 = newObj->m_particles.begin();
			 it1 != obj->m_particles.end(); ++it1, ++it2)
		{
			it2->log_w = it1->log_w;
			it2->d = TPose3D(it1->d);
		}

		return newObj;
	}
	else if (o.GetRuntimeClass() == CLASS_ID(CPosePDFSOG))
	{
		const auto* obj = dynamic_cast<const CPosePDFSOG*>(&o);
		ASSERT_(obj != nullptr);
		auto* newObj = new CPose3DPDFSOG(obj->size());

		CPosePDFSOG::const_iterator it1;
		CPose3DPDFSOG::iterator it2;

		for (it1 = obj->begin(), it2 = newObj->begin(); it1 != obj->end();
			 ++it1, ++it2)
		{
			it2->log_w = it1->log_w;
			it2->val.mean.setFromValues(
				it1->mean.x(), it1->mean.y(), 0, it1->mean.phi(), 0, 0);

			it2->val.cov.setZero();

			it2->val.cov(0, 0) = it1->cov(0, 0);
			it2->val.cov(1, 1) = it1->cov(1, 1);
			it2->val.cov(3, 3) = it1->cov(2, 2);  // yaw <- phi

			it2->val.cov(0, 1) = it2->val.cov(1, 0) = it1->cov(0, 1);

			it2->val.cov(0, 3) = it2->val.cov(3, 0) = it1->cov(0, 2);

			it2->val.cov(1, 3) = it2->val.cov(3, 1) = it1->cov(1, 2);
		}

		return newObj;
	}
	else
		THROW_EXCEPTION("Class of object not supported by this method!");

	MRPT_END
}

/*---------------------------------------------------------------
					jacobiansPoseComposition
 ---------------------------------------------------------------*/
void CPose3DPDF::jacobiansPoseComposition(
	const CPose3D& x, const CPose3D& u, CMatrixDouble66& df_dx,
	CMatrixDouble66& df_du)
{
	// See this techical report:
	// https://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty

	// Direct equations (for the covariances) in yaw-pitch-roll are too complex.
	//  Make a way around them and consider instead this path:
	//
	//      X(6D)       U(6D)
	//        |           |
	//        v           v
	//      X(7D)       U(7D)
	//        |           |
	//        +--- (+) ---+
	//              |
	//              v
	//            RES(7D)
	//              |
	//              v
	//            RES(6D)
	//

	// FUNCTION = f_quat2eul(  f_quat_comp(  f_eul2quat(x) , f_eul2quat(u)  ) )
	//  Jacobians chain rule:
	//
	//  JACOB_dx = J_Q2E (6x7) * quat_df_dx (7x7) * J_E2Q_dx (7x6)
	//  JACOB_du = J_Q2E (6x7) * quat_df_du (7x7) * J_E2Q_du (7x6)

	// J_E2Q_dx:
	CMatrixFixed<double, 7, 6> J_E2Q_dx;  // Init to zeros
	{
		CMatrixFixed<double, 4, 3> dq_dr_sub(UNINITIALIZED_MATRIX);
		CQuaternionDouble q_dumm(UNINITIALIZED_QUATERNION);
		x.getAsQuaternion(q_dumm, &dq_dr_sub);
		J_E2Q_dx(0, 0) = J_E2Q_dx(1, 1) = J_E2Q_dx(2, 2) = 1;
		J_E2Q_dx.insertMatrix(3, 3, dq_dr_sub);
	}

	// J_E2Q_du:
	CMatrixFixed<double, 7, 6> J_E2Q_du;  // Init to zeros
	{
		CMatrixFixed<double, 4, 3> dq_dr_sub(UNINITIALIZED_MATRIX);
		CQuaternionDouble q_dumm(UNINITIALIZED_QUATERNION);
		u.getAsQuaternion(q_dumm, &dq_dr_sub);
		J_E2Q_du(0, 0) = J_E2Q_du(1, 1) = J_E2Q_du(2, 2) = 1;
		J_E2Q_du.insertMatrix(3, 3, dq_dr_sub);
	}

	// quat_df_dx & quat_df_du
	CMatrixDouble77 quat_df_dx(UNINITIALIZED_MATRIX),
		quat_df_du(UNINITIALIZED_MATRIX);
	const CPose3DQuat quat_x(x);
	const CPose3DQuat quat_u(u);

	CPose3DQuatPDF::jacobiansPoseComposition(
		quat_x,  // x
		quat_u,  // u
		quat_df_dx, quat_df_du);

	// And now J_Q2E:
	//         [  I_3   |    0         ]
	// J_Q2E = [ -------+------------- ]
	//         [  0     | dr_dq_angles ]
	//
	CMatrixFixed<double, 6, 7> J_Q2E;  // Init to zeros
	J_Q2E(0, 0) = J_Q2E(1, 1) = J_Q2E(2, 2) = 1;
	{
		// The end result of the pose composition, as a quaternion:
		CQuaternionDouble q_xu(UNINITIALIZED_QUATERNION);
		q_xu.crossProduct(quat_x.quat(), quat_u.quat());

		// Compute the jacobian:
		CMatrixFixed<double, 3, 4> dr_dq_sub_aux(UNINITIALIZED_MATRIX);
		double yaw, pitch, roll;
		q_xu.rpy_and_jacobian(roll, pitch, yaw, &dr_dq_sub_aux, false);

		CMatrixDouble44 dnorm_dq(UNINITIALIZED_MATRIX);
		q_xu.normalizationJacobian(dnorm_dq);

		CMatrixFixed<double, 3, 4> dr_dq_sub(UNINITIALIZED_MATRIX);
		dr_dq_sub.asEigen() = dr_dq_sub_aux * dnorm_dq;

		J_Q2E.insertMatrix(3, 3, dr_dq_sub);
	}

	// And finally:
	//  JACOB_dx = J_Q2E (6x7) * quat_df_dx (7x7) * J_E2Q_dx (7x6)
	//  JACOB_du = J_Q2E (6x7) * quat_df_du (7x7) * J_E2Q_du (7x6)
	df_dx.asEigen() =
		J_Q2E.asEigen() * quat_df_dx.asEigen() * J_E2Q_dx.asEigen();
	df_du.asEigen() =
		J_Q2E.asEigen() * quat_df_du.asEigen() * J_E2Q_du.asEigen();
}

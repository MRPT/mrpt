/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/poses/CPointPDFSOG.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>
#include <Eigen/Dense>

using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::bayes;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPointPDFSOG, CPosePDF, mrpt::poses)

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPointPDFSOG::CPointPDFSOG(size_t nModes) : m_modes(nModes) {}
/*---------------------------------------------------------------
			clear
  ---------------------------------------------------------------*/
void CPointPDFSOG::clear() { m_modes.clear(); }
/*---------------------------------------------------------------
	Resize
  ---------------------------------------------------------------*/
void CPointPDFSOG::resize(const size_t N) { m_modes.resize(N); }
/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the
 PDF)
 ---------------------------------------------------------------*/
void CPointPDFSOG::getMean(CPoint3D& p) const
{
	size_t N = m_modes.size();
	double X = 0, Y = 0, Z = 0;

	if (N)
	{
		CListGaussianModes::const_iterator it;
		double sumW = 0;

		for (it = m_modes.begin(); it != m_modes.end(); ++it)
		{
			double w;
			sumW += w = exp(it->log_w);
			X += it->val.mean.x() * w;
			Y += it->val.mean.y() * w;
			Z += it->val.mean.z() * w;
		}
		if (sumW > 0)
		{
			X /= sumW;
			Y /= sumW;
			Z /= sumW;
		}
	}

	p.x(X);
	p.y(Y);
	p.z(Z);
}

std::tuple<CMatrixDouble33, CPoint3D> CPointPDFSOG::getCovarianceAndMean() const
{
	size_t N = m_modes.size();

	CMatrixDouble33 estCov;
	CPoint3D p;
	getMean(p);
	estCov.setZero();

	if (N)
	{
		// 1) Get the mean:
		double sumW = 0;
		auto estMean = CMatrixDouble31(p);

		CListGaussianModes::const_iterator it;

		for (it = m_modes.begin(); it != m_modes.end(); ++it)
		{
			double w;
			sumW += w = exp(it->log_w);

			auto estMean_i = CMatrixDouble31(it->val.mean);
			estMean_i -= estMean;

			auto partCov =
				CMatrixDouble33(estMean_i.asEigen() * estMean_i.transpose());
			partCov += it->val.cov;
			partCov *= w;
			estCov += partCov;
		}

		if (sumW != 0) estCov *= (1.0 / sumW);
	}

	return {estCov, p};
}

uint8_t CPointPDFSOG::serializeGetVersion() const { return 1; }
void CPointPDFSOG::serializeTo(mrpt::serialization::CArchive& out) const
{
	uint32_t N = m_modes.size();
	out << N;
	for (const auto& m : m_modes)
	{
		out << m.log_w;
		out << m.val.mean;
		mrpt::math::serializeSymmetricMatrixTo(m.val.cov, out);
	}
}
void CPointPDFSOG::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			uint32_t N;
			in >> N;
			this->resize(N);
			for (auto& m : m_modes)
			{
				in >> m.log_w;

				// In version 0, weights were linear!!
				if (version == 0) m.log_w = log(max(1e-300, m.log_w));

				in >> m.val.mean;
				mrpt::math::deserializeSymmetricMatrixFrom(m.val.cov, in);
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CPointPDFSOG::copyFrom(const CPointPDF& o)
{
	MRPT_START

	if (this == &o) return;  // It may be used sometimes

	if (o.GetRuntimeClass() == CLASS_ID(CPointPDFSOG))
	{
		m_modes = dynamic_cast<const CPointPDFSOG*>(&o)->m_modes;
	}
	else
	{
		// Approximate as a mono-modal gaussian pdf:
		this->resize(1);
		m_modes[0].log_w = 0;
		o.getCovarianceAndMean(m_modes[0].val.cov, m_modes[0].val.mean);
	}

	MRPT_END
}

/*---------------------------------------------------------------
						saveToTextFile
  ---------------------------------------------------------------*/
bool CPointPDFSOG::saveToTextFile(const std::string& file) const
{
	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;

	for (const auto& m_mode : m_modes)
		os::fprintf(
			f, "%e %e %e %e %e %e %e %e %e %e\n", exp(m_mode.log_w),
			m_mode.val.mean.x(), m_mode.val.mean.y(), m_mode.val.mean.z(),
			m_mode.val.cov(0, 0), m_mode.val.cov(1, 1), m_mode.val.cov(2, 2),
			m_mode.val.cov(0, 1), m_mode.val.cov(0, 2), m_mode.val.cov(1, 2));
	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPointPDFSOG::changeCoordinatesReference(const CPose3D& newReferenceBase)
{
	for (auto& m : m_modes) m.val.changeCoordinatesReference(newReferenceBase);
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPointPDFSOG::drawSingleSample(CPoint3D& outSample) const
{
	MRPT_START

	ASSERT_(m_modes.size() > 0);

	// 1st: Select a mode with a probability proportional to its weight:
	vector<double> logWeights(m_modes.size());
	vector<size_t> outIdxs;
	vector<double>::iterator itW;
	CListGaussianModes::const_iterator it;
	for (it = m_modes.begin(), itW = logWeights.begin(); it != m_modes.end();
		 ++it, ++itW)
		*itW = it->log_w;

	CParticleFilterCapable::computeResampling(
		CParticleFilter::prMultinomial,  // Resampling algorithm
		logWeights,  // input: log weights
		outIdxs  // output: indexes
	);

	// we need just one: take the first (arbitrary)
	size_t selectedIdx = outIdxs[0];
	ASSERT_(selectedIdx < m_modes.size());
	const CPointPDFGaussian* selMode = &m_modes[selectedIdx].val;

	// 2nd: Draw a position from the selected Gaussian:
	CVectorDouble vec;
	getRandomGenerator().drawGaussianMultivariate(vec, selMode->cov);

	ASSERT_(vec.size() == 3);
	outSample.x(selMode->mean.x() + vec[0]);
	outSample.y(selMode->mean.y() + vec[1]);
	outSample.z(selMode->mean.z() + vec[2]);

	MRPT_END
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void CPointPDFSOG::bayesianFusion(
	const CPointPDF& p1_, const CPointPDF& p2_,
	const double minMahalanobisDistToDrop)
{
	MRPT_START

	// p1: CPointPDFSOG, p2: CPosePDFGaussian:

	ASSERT_(p1_.GetRuntimeClass() == CLASS_ID(CPointPDFSOG));
	ASSERT_(p2_.GetRuntimeClass() == CLASS_ID(CPointPDFSOG));

	const auto* p1 = dynamic_cast<const CPointPDFSOG*>(&p1_);
	const auto* p2 = dynamic_cast<const CPointPDFSOG*>(&p2_);

	// Compute the new kernel means, covariances, and weights after multiplying
	// to the Gaussian "p2":
	CPointPDFGaussian auxGaussianProduct, auxSOG_Kernel_i;

	float minMahalanobisDistToDrop2 = square(minMahalanobisDistToDrop);

	this->m_modes.clear();
	bool is2D =
		false;  // to detect & avoid errors in 3x3 matrix inversions of range=2.

	for (const auto& m : p1->m_modes)
	{
		CMatrixDouble33 c = m.val.cov;

		// Is a 2D covariance??
		if (c(2, 2) == 0)
		{
			is2D = true;
			c(2, 2) = 1;
		}

		ASSERT_(c(0, 0) != 0 && c(0, 0) != 0);

		const CMatrixDouble33 covInv = c.inverse_LLt();

		Eigen::Vector3d eta = covInv * CMatrixDouble31(m.val.mean);

		// Normal distribution canonical form constant:
		// See: http://www-static.cc.gatech.edu/~wujx/paper/Gaussian.pdf
		double a = -0.5 * (3 * log(M_2PI) - log(covInv.det()) +
						   (eta.transpose() * c.asEigen() * eta)(0, 0));

		for (const auto& m2 : p2->m_modes)
		{
			auxSOG_Kernel_i = m2.val;
			if (auxSOG_Kernel_i.cov(2, 2) == 0)
			{
				auxSOG_Kernel_i.cov(2, 2) = 1;
				is2D = true;
			}
			ASSERT_(
				auxSOG_Kernel_i.cov(0, 0) > 0 && auxSOG_Kernel_i.cov(1, 1) > 0);

			// Should we drop this product term??
			bool reallyComputeThisOne = true;
			if (minMahalanobisDistToDrop > 0)
			{
				// Approximate (fast) mahalanobis distance (square):
				float mahaDist2;

				float stdX2 = max(auxSOG_Kernel_i.cov(0, 0), m.val.cov(0, 0));
				mahaDist2 =
					square(auxSOG_Kernel_i.mean.x() - m.val.mean.x()) / stdX2;

				float stdY2 = max(auxSOG_Kernel_i.cov(1, 1), m.val.cov(1, 1));
				mahaDist2 +=
					square(auxSOG_Kernel_i.mean.y() - m.val.mean.y()) / stdY2;

				if (!is2D)
				{
					float stdZ2 =
						max(auxSOG_Kernel_i.cov(2, 2), m.val.cov(2, 2));
					mahaDist2 +=
						square(auxSOG_Kernel_i.mean.z() - m.val.mean.z()) /
						stdZ2;
				}

				reallyComputeThisOne = mahaDist2 < minMahalanobisDistToDrop2;
			}

			if (reallyComputeThisOne)
			{
				auxGaussianProduct.bayesianFusion(auxSOG_Kernel_i, m.val);

				// ----------------------------------------------------------------------
				// The new weight is given by:
				//
				//   w'_i = w_i * exp( a + a_i - a' )
				//
				//      a = -1/2 ( dimensionality * log(2pi) - log(det(Cov^-1))
				//      + (Cov^-1 * mu)^t * Cov^-1 * (Cov^-1 * mu) )
				//
				// ----------------------------------------------------------------------
				TGaussianMode newKernel;

				newKernel.val = auxGaussianProduct;  // Copy mean & cov

				CMatrixDouble33 covInv_i = auxSOG_Kernel_i.cov.inverse_LLt();
				Eigen::Vector3d eta_i =
					CMatrixDouble31(auxSOG_Kernel_i.mean).asEigen();
				eta_i = covInv_i.asEigen() * eta_i;

				CMatrixDouble33 new_covInv_i = newKernel.val.cov.inverse_LLt();
				Eigen::Vector3d new_eta_i =
					CMatrixDouble31(newKernel.val.mean).asEigen();
				new_eta_i = new_covInv_i.asEigen() * new_eta_i;

				double a_i =
					-0.5 * (3 * log(M_2PI) - log(new_covInv_i.det()) +
							(eta_i.transpose() * auxSOG_Kernel_i.cov.asEigen() *
							 eta_i)(0, 0));
				double new_a_i =
					-0.5 * (3 * log(M_2PI) - log(new_covInv_i.det()) +
							(new_eta_i.transpose() *
							 newKernel.val.cov.asEigen() * new_eta_i)(0, 0));

				newKernel.log_w = m.log_w + m2.log_w + a + a_i - new_a_i;

				// Fix 2D case:
				if (is2D) newKernel.val.cov(2, 2) = 0;

				// Add to the results (in "this") the new kernel:
				this->m_modes.push_back(newKernel);
			}  // end if reallyComputeThisOne
		}  // end for it2

	}  // end for it1

	normalizeWeights();

	MRPT_END
}

/*---------------------------------------------------------------
						enforceCovSymmetry
 ---------------------------------------------------------------*/
void CPointPDFSOG::enforceCovSymmetry()
{
	MRPT_START
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	for (auto& m_mode : m_modes)
	{
		m_mode.val.cov(0, 1) = m_mode.val.cov(1, 0);
		m_mode.val.cov(0, 2) = m_mode.val.cov(2, 0);
		m_mode.val.cov(1, 2) = m_mode.val.cov(2, 1);
	}

	MRPT_END
}

/*---------------------------------------------------------------
						normalizeWeights
 ---------------------------------------------------------------*/
void CPointPDFSOG::normalizeWeights()
{
	MRPT_START

	if (!m_modes.size()) return;

	CListGaussianModes::iterator it;
	double maxW = m_modes[0].log_w;
	for (it = m_modes.begin(); it != m_modes.end(); ++it)
		maxW = max(maxW, it->log_w);

	for (it = m_modes.begin(); it != m_modes.end(); ++it) it->log_w -= maxW;

	MRPT_END
}

/*---------------------------------------------------------------
						ESS
 ---------------------------------------------------------------*/
double CPointPDFSOG::ESS() const
{
	MRPT_START
	CListGaussianModes::const_iterator it;
	double cum = 0;

	/* Sum of weights: */
	double sumLinearWeights = 0;
	for (it = m_modes.begin(); it != m_modes.end(); ++it)
		sumLinearWeights += exp(it->log_w);

	/* Compute ESS: */
	for (it = m_modes.begin(); it != m_modes.end(); ++it)
		cum += square(exp(it->log_w) / sumLinearWeights);

	if (cum == 0)
		return 0;
	else
		return 1.0 / (m_modes.size() * cum);
	MRPT_END
}

/*---------------------------------------------------------------
						evaluatePDFInArea
 ---------------------------------------------------------------*/
void CPointPDFSOG::evaluatePDFInArea(
	float x_min, float x_max, float y_min, float y_max, float resolutionXY,
	float z, CMatrixD& outMatrix, bool sumOverAllZs)
{
	MRPT_START

	ASSERT_(x_max > x_min);
	ASSERT_(y_max > y_min);
	ASSERT_(resolutionXY > 0);

	const auto Nx = (size_t)ceil((x_max - x_min) / resolutionXY);
	const auto Ny = (size_t)ceil((y_max - y_min) / resolutionXY);
	outMatrix.setSize(Ny, Nx);

	for (size_t i = 0; i < Ny; i++)
	{
		const float y = y_min + i * resolutionXY;
		for (size_t j = 0; j < Nx; j++)
		{
			float x = x_min + j * resolutionXY;
			outMatrix(i, j) = evaluatePDF(CPoint3D(x, y, z), sumOverAllZs);
		}
	}

	MRPT_END
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double CPointPDFSOG::evaluatePDF(const CPoint3D& x, bool sumOverAllZs) const
{
	if (!sumOverAllZs)
	{
		// Normal evaluation:
		auto X = CMatrixDouble31(x);
		double ret = 0;

		CMatrixDouble31 MU;

		for (const auto& m_mode : m_modes)
		{
			MU = CMatrixDouble31(m_mode.val.mean);
			ret += exp(m_mode.log_w) * math::normalPDF(X, MU, m_mode.val.cov);
		}

		return ret;
	}
	else
	{
		// Only X,Y:
		CMatrixD X(2, 1), MU(2, 1), COV(2, 2);
		double ret = 0;

		X(0, 0) = x.x();
		X(1, 0) = x.y();

		for (const auto& m_mode : m_modes)
		{
			MU(0, 0) = m_mode.val.mean.x();
			MU(1, 0) = m_mode.val.mean.y();

			COV(0, 0) = m_mode.val.cov(0, 0);
			COV(1, 1) = m_mode.val.cov(1, 1);
			COV(0, 1) = COV(1, 0) = m_mode.val.cov(0, 1);

			ret += exp(m_mode.log_w) * math::normalPDF(X, MU, COV);
		}

		return ret;
	}
}

/*---------------------------------------------------------------
						getMostLikelyMode
 ---------------------------------------------------------------*/
void CPointPDFSOG::getMostLikelyMode(CPointPDFGaussian& outVal) const
{
	if (this->empty())
	{
		outVal = CPointPDFGaussian();
	}
	else
	{
		auto it_best = m_modes.end();
		for (auto it = m_modes.begin(); it != m_modes.end(); ++it)
			if (it_best == m_modes.end() || it->log_w > it_best->log_w)
				it_best = it;

		outVal = it_best->val;
	}
}

/*---------------------------------------------------------------
						getAs3DObject
 ---------------------------------------------------------------*/
// void  CPointPDFSOG::getAs3DObject( mrpt::opengl::CSetOfObjects::Ptr	&outObj
// )
// const
//{
//	// For each gaussian node
//	for (CListGaussianModes::const_iterator it = m_modes.begin(); it!=
// m_modes.end();++it)
//	{
//		opengl::CEllipsoid::Ptr obj =
// std::make_shared<opengl::CEllipsoid>();
//
//		obj->setPose( it->val.mean);
//		obj->setCovMatrix(it->val.cov,  it->val.cov(2,2)==0  ?  2:3);
//
//		obj->setQuantiles(3);
//		obj->enableDrawSolid3D(false);
//		obj->setColor(1,0,0, 0.5);
//
//		outObj->insert( obj );
//	} // end for each gaussian node
//}

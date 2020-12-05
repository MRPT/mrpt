/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers
//
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

#include <Eigen/Dense>
#include <iostream>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPosePDFSOG, CPosePDF, mrpt::poses)

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFSOG::CPosePDFSOG(size_t nModes) : m_modes(nModes) {}
/*---------------------------------------------------------------
			clear
  ---------------------------------------------------------------*/
void CPosePDFSOG::clear() { m_modes.clear(); }
/*---------------------------------------------------------------
	Resize
  ---------------------------------------------------------------*/
void CPosePDFSOG::resize(const size_t N) { m_modes.resize(N); }
/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the
 PDF)
 ---------------------------------------------------------------*/
void CPosePDFSOG::getMean(CPose2D& p) const
{
	if (!m_modes.empty())
	{
		mrpt::poses::SE_average<2> se_averager;
		for (const auto& m : m_modes)
		{
			const double w = exp(m.log_w);
			se_averager.append(m.mean, w);
		}
		se_averager.get_average(p);
	}
	else
	{
		p = CPose2D();
	}
}

std::tuple<CMatrixDouble33, CPose2D> CPosePDFSOG::getCovarianceAndMean() const
{
	const size_t N = m_modes.size();

	mrpt::math::CMatrixDouble33 cov;
	CPose2D estMean2D;

	this->getMean(estMean2D);
	cov.setZero();

	if (N)
	{
		// 1) Get the mean:
		double sumW = 0;
		auto estMeanMat = CMatrixDouble31(estMean2D);
		CMatrixDouble33 temp;
		CMatrixDouble31 estMean_i;

		for (const auto& m : m_modes)
		{
			double w;
			sumW += w = exp(m.log_w);

			estMean_i = CMatrixDouble31(m.mean);
			estMean_i -= estMeanMat;

			temp.matProductOf_AAt(estMean_i);
			temp += m.cov;
			temp *= w;

			cov += temp;
		}

		if (sumW != 0) cov *= (1.0 / sumW);
	}
	return {cov, estMean2D};
}

uint8_t CPosePDFSOG::serializeGetVersion() const { return 2; }
void CPosePDFSOG::serializeTo(mrpt::serialization::CArchive& out) const
{
	uint32_t N = m_modes.size();
	out << N;

	for (const auto& m : m_modes)
	{
		out << m.log_w << m.mean;
		mrpt::math::serializeSymmetricMatrixTo(m.cov, out);
	}
}
void CPosePDFSOG::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			uint32_t N;
			in >> N;
			resize(N);
			for (auto& m : m_modes)
			{
				in >> m.log_w;

				// In version 0, weights were linear!!
				if (version == 0) m.log_w = log(max(1e-300, m.log_w));

				in >> m.mean;

				if (version == 1)  // float's
				{
					CMatrixFloat33 mf;
					mrpt::math::deserializeSymmetricMatrixFrom(mf, in);
					m.cov = mf.cast_double();
				}
				else
				{
					mrpt::math::deserializeSymmetricMatrixFrom(m.cov, in);
				}
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CPosePDFSOG::copyFrom(const CPosePDF& o)
{
	MRPT_START

	if (this == &o) return;  // It may be used sometimes

	if (o.GetRuntimeClass() == CLASS_ID(CPosePDFSOG))
	{
		m_modes = dynamic_cast<const CPosePDFSOG*>(&o)->m_modes;
	}
	else
	{
		// Approximate as a mono-modal gaussian pdf:
		m_modes.resize(1);
		m_modes[0].log_w = 0;
		o.getMean(m_modes[0].mean);
		o.getCovariance(m_modes[0].cov);
	}

	MRPT_END
}

/*---------------------------------------------------------------
						saveToTextFile
  ---------------------------------------------------------------*/
bool CPosePDFSOG::saveToTextFile(const std::string& file) const
{
	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;

	for (const auto& m : m_modes)
		os::fprintf(
			f, "%e %e %e %e %e %e %e %e %e %e\n", exp(m.log_w), m.mean.x(),
			m.mean.y(), m.mean.phi(), m.cov(0, 0), m.cov(1, 1), m.cov(2, 2),
			m.cov(0, 1), m.cov(0, 2), m.cov(1, 2));
	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPosePDFSOG::changeCoordinatesReference(const CPose3D& newReferenceBase_)
{
	const CPose2D newReferenceBase = CPose2D(newReferenceBase_);

	CMatrixDouble44 HM;
	newReferenceBase.getHomogeneousMatrix(HM);

	// Clip the 4x4 matrix
	auto M = CMatrixDouble33(HM.block<3, 3>(0, 0));

	// The variance in phi is unmodified:
	M(0, 2) = 0;
	M(1, 2) = 0;
	M(2, 0) = 0;
	M(2, 1) = 0;
	M(2, 2) = 1;

	for (auto& m : m_modes)
	{
		// The mean:
		m.mean.composeFrom(newReferenceBase, m.mean);

		// The covariance:
		// NOTE: the CMatrixDouble33 is NEEDED to create a temporary copy to
		// allow aliasing
		m.cov = mrpt::math::multiply_HCHt(M, CMatrixDouble33(m.cov));
	}

	enforceCovSymmetry();
}

/*---------------------------------------------------------------
						rotateAllCovariances
 ---------------------------------------------------------------*/
void CPosePDFSOG::rotateAllCovariances(double ang)
{
	CMatrixDouble33 rot;
	rot(0, 0) = rot(1, 1) = cos(ang);
	rot(0, 1) = -sin(ang);
	rot(1, 0) = sin(ang);
	rot(2, 2) = 1;

	for (auto& m : m_modes)
		m.cov = mrpt::math::multiply_HCHt(rot, CMatrixDouble33(m.cov));
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPosePDFSOG::drawSingleSample([[maybe_unused]] CPose2D& outPart) const
{
	MRPT_START
	THROW_EXCEPTION("Not implemented yet!!");
	MRPT_END
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void CPosePDFSOG::drawManySamples(
	[[maybe_unused]] size_t N,
	[[maybe_unused]] std::vector<CVectorDouble>& outSamples) const
{
	MRPT_START

	THROW_EXCEPTION("Not implemented yet!!");

	MRPT_END
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void CPosePDFSOG::bayesianFusion(
	const CPosePDF& p1_, const CPosePDF& p2_,
	[[maybe_unused]] const double minMahalanobisDistToDrop)
{
	MRPT_START

	// p1: CPosePDFSOG, p2: CPosePDFGaussian:

	ASSERT_(p1_.GetRuntimeClass() == CLASS_ID(CPosePDFSOG));
	ASSERT_(p2_.GetRuntimeClass() == CLASS_ID(CPosePDFGaussian));

	const auto* p1 = dynamic_cast<const CPosePDFSOG*>(&p1_);
	const auto* p2 = dynamic_cast<const CPosePDFGaussian*>(&p2_);

	// Compute the new kernel means, covariances, and weights after multiplying
	// to the Gaussian "p2":
	CPosePDFGaussian auxGaussianProduct, auxSOG_Kernel_i;

	const CMatrixDouble33 covInv = p2->cov.inverse_LLt();

	auto eta = CMatrixDouble31(p2->mean);
	eta = covInv * eta;

	// Normal distribution canonical form constant:
	// See: http://www-static.cc.gatech.edu/~wujx/paper/Gaussian.pdf
	double a =
		-0.5 * (3 * log(M_2PI) - log(covInv.det()) +
				(eta.transpose() * p2->cov.asEigen() * eta.asEigen())(0, 0));

	this->m_modes.clear();
	for (const auto& m : p1->m_modes)
	{
		auxSOG_Kernel_i.mean = m.mean;
		auxSOG_Kernel_i.cov = CMatrixDouble(m.cov);
		auxGaussianProduct.bayesianFusion(auxSOG_Kernel_i, *p2);

		// ----------------------------------------------------------------------
		// The new weight is given by:
		//
		//   w'_i = w_i * exp( a + a_i - a' )
		//
		//      a = -1/2 ( dimensionality * log(2pi) - log(det(Cov^-1)) +
		//      (Cov^-1 * mu)^t * Cov^-1 * (Cov^-1 * mu) )
		//
		// ----------------------------------------------------------------------
		TGaussianMode newKernel;
		newKernel.mean = auxGaussianProduct.mean;
		newKernel.cov = auxGaussianProduct.cov;

		const CMatrixDouble33 covInv_i = auxSOG_Kernel_i.cov.inverse_LLt();

		auto eta_i = CMatrixDouble31(auxSOG_Kernel_i.mean);
		eta_i = covInv_i * eta_i;

		const CMatrixDouble33 new_covInv_i = newKernel.cov.inverse_LLt();

		auto new_eta_i = CMatrixDouble31(newKernel.mean);
		new_eta_i = new_covInv_i * new_eta_i;

		double a_i =
			-0.5 * (3 * log(M_2PI) - log(new_covInv_i.det()) +
					(eta_i.transpose() * auxSOG_Kernel_i.cov.asEigen() *
					 eta_i.asEigen())(0, 0));
		double new_a_i =
			-0.5 * (3 * log(M_2PI) - log(new_covInv_i.det()) +
					(new_eta_i.transpose() * newKernel.cov.asEigen() *
					 new_eta_i.asEigen())(0, 0));

		// newKernel.w	   = (it)->w * exp( a + a_i - new_a_i );
		newKernel.log_w = m.log_w + a + a_i - new_a_i;

		// Add to the results (in "this") the new kernel:
		this->m_modes.push_back(newKernel);
	}

	normalizeWeights();

	MRPT_END
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void CPosePDFSOG::inverse(CPosePDF& o) const
{
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPosePDFSOG));
	auto* out = dynamic_cast<CPosePDFSOG*>(&o);

	const_iterator itSrc;
	iterator itDest;

	out->m_modes.resize(m_modes.size());

	for (itSrc = m_modes.begin(), itDest = out->m_modes.begin();
		 itSrc != m_modes.end(); ++itSrc, ++itDest)
	{
		// The mean:
		(itDest)->mean = -(itSrc)->mean;

		// The covariance: Is the same:
		(itDest)->cov = (itSrc)->cov;
	}
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void CPosePDFSOG::operator+=(const CPose2D& Ap)
{
	for (auto& m : m_modes) m.mean = m.mean + Ap;

	this->rotateAllCovariances(Ap.phi());
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double CPosePDFSOG::evaluatePDF(const CPose2D& x, bool sumOverAllPhis) const
{
	if (!sumOverAllPhis)
	{
		// Normal evaluation:
		auto X = CMatrixDouble31(x);
		CMatrixDouble31 MU;
		double ret = 0;

		for (const auto& m : m_modes)
		{
			MU = CMatrixDouble31(m.mean);
			ret += exp(m.log_w) * math::normalPDF(X, MU, m.cov);
		}

		return ret;
	}
	else
	{
		// Only X,Y:
		CMatrixDouble X(2, 1), MU(2, 1), COV(2, 2);
		double ret = 0;

		X(0, 0) = x.x();
		X(1, 0) = x.y();

		for (const auto& m : m_modes)
		{
			MU(0, 0) = m.mean.x();
			MU(1, 0) = m.mean.y();

			COV(0, 0) = m.cov(0, 0);
			COV(1, 1) = m.cov(1, 1);
			COV(0, 1) = COV(1, 0) = m.cov(0, 1);

			ret += exp(m.log_w) * math::normalPDF(X, MU, COV);
		}

		return ret;
	}
}

/*---------------------------------------------------------------
						evaluateNormalizedPDF
 ---------------------------------------------------------------*/
double CPosePDFSOG::evaluateNormalizedPDF(const CPose2D& x) const
{
	auto X = CMatrixDouble31(x);
	CMatrixDouble31 MU;
	double ret = 0;

	for (const auto& m : m_modes)
	{
		MU = CMatrixDouble31(m.mean);
		ret += exp(m.log_w) * math::normalPDF(X, MU, m.cov) /
			   math::normalPDF(MU, MU, m.cov);
	}

	return ret;
}

/*---------------------------------------------------------------
						enforceCovSymmetry
 ---------------------------------------------------------------*/
void CPosePDFSOG::enforceCovSymmetry()
{
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	for (auto& m : m_modes)
	{
		m.cov(0, 1) = m.cov(1, 0);
		m.cov(0, 2) = m.cov(2, 0);
		m.cov(1, 2) = m.cov(2, 1);
	}
}

/*---------------------------------------------------------------
						normalizeWeights
 ---------------------------------------------------------------*/
void CPosePDFSOG::normalizeWeights()
{
	MRPT_START

	if (!m_modes.size()) return;

	double maxW = m_modes[0].log_w;
	for (auto& m : m_modes) maxW = max(maxW, m.log_w);

	for (auto& m : m_modes) m.log_w -= maxW;

	MRPT_END
}

/*---------------------------------------------------------------
						normalizeWeights
 ---------------------------------------------------------------*/
void CPosePDFSOG::evaluatePDFInArea(
	double x_min, double x_max, double y_min, double y_max, double resolutionXY,
	double phi, CMatrixDouble& outMatrix, bool sumOverAllPhis)
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
		double y = y_min + i * resolutionXY;
		for (size_t j = 0; j < Nx; j++)
		{
			double x = x_min + j * resolutionXY;
			outMatrix(i, j) = evaluatePDF(CPose2D(x, y, phi), sumOverAllPhis);
		}
	}

	MRPT_END
}

/*---------------------------------------------------------------
						mergeModes
 ---------------------------------------------------------------*/
void CPosePDFSOG::mergeModes(double max_KLd, bool verbose)
{
	MRPT_START

	normalizeWeights();

	size_t N = m_modes.size();
	if (N < 2) return;  // Nothing to do

	// Method described in:
	// "Kullback-Leibler Approach to Gaussian Mixture Reduction", A.R. Runnalls.
	// IEEE Transactions on Aerospace and Electronic Systems, 2007.
	//  See Eq.(21) for Bij !!

	for (size_t i = 0; i < (N - 1);)
	{
		N = m_modes.size();  // It might have changed.
		double sumW = 0;

		// For getting linear weights:
		sumW = 0;
		for (size_t j = 0; j < N; j++) sumW += exp(m_modes[j].log_w);
		ASSERT_(sumW);

		const double Wi = exp(m_modes[i].log_w) / sumW;

		double min_Bij = std::numeric_limits<double>::max();

		CMatrixDouble33 min_Bij_COV;
		size_t best_j = 0;

		auto MUi = CMatrixDouble31(m_modes[i].mean);

		// Compute B(i,j), j=[i+1,N-1]  (the discriminant)
		for (size_t j = 0; j < N; j++)
			if (i != j)
			{
				const double Wj = exp(m_modes[j].log_w) / sumW;
				const double Wij_ = 1.0 / (Wi + Wj);

				auto Pij =
					CMatrixDouble33(m_modes[i].cov.asEigen() * Wi * Wij_);
				Pij.asEigen() += m_modes[j].cov.asEigen() * Wj * Wij_;

				auto MUij = CMatrixDouble31(m_modes[j].mean);
				MUij -= MUi;
				// Account for circular dimensions:
				mrpt::math::wrapToPiInPlace(MUij(2, 0));

				CMatrixDouble33 AUX;
				AUX.matProductOf_AAt(MUij);  // AUX = MUij * MUij^T

				AUX *= Wi * Wj * Wij_ * Wij_;
				Pij += AUX;

				double Bij = (Wi + Wj) * log(Pij.det()) -
							 Wi * log(m_modes[i].cov.det()) -
							 Wj * log(m_modes[j].cov.det());
				if (verbose)
				{
					cout << "try merge[" << i << ", " << j
						 << "] -> Bij: " << Bij << endl;
					// cout << "AUX: " << endl << AUX;
					// cout << "Wi: " << Wi << " Wj:" << Wj << " Wij_: " << Wij_
					// << endl;
					cout << "Pij: " << Pij << endl
						 << " Pi: " << m_modes[i].cov << endl
						 << " Pj: " << m_modes[j].cov << endl;
				}

				if (Bij < min_Bij)
				{
					min_Bij = Bij;
					best_j = j;
					min_Bij_COV = Pij;
				}
			}

		// Is a good move to merge (i,j)??
		if (verbose)
			cout << "merge[" << i << ", " << best_j
				 << "] Tempting merge: KLd = " << min_Bij;

		if (min_Bij < max_KLd)
		{
			if (verbose) cout << " Accepted." << endl;

			// Do the merge (i,j):
			TGaussianMode Mij;
			TGaussianMode& Mi = m_modes[i];
			TGaussianMode& Mj = m_modes[best_j];

			// Weight:
			Mij.log_w = log(exp(Mi.log_w) + exp(Mj.log_w));

			// Mean:
			const double Wj = exp(Mj.log_w) / sumW;
			const double Wij_ = 1.0 / (Wi + Wj);
			const double Wi_ = Wi * Wij_;
			const double Wj_ = Wj * Wij_;

			Mij.mean = CPose2D(
				Wi_ * Mi.mean.x() + Wj_ * Mj.mean.x(),
				Wi_ * Mi.mean.y() + Wj_ * Mj.mean.y(),
				Wi_ * Mi.mean.phi() + Wj_ * Mj.mean.phi());

			// Cov:
			Mij.cov = min_Bij_COV;

			// Replace Mi <- Mij:
			m_modes[i] = Mij;
			m_modes.erase(m_modes.begin() + best_j);  // erase Mj
		}  // end merge
		else
		{
			if (verbose) cout << " Nope." << endl;

			i++;
		}
	}  // for i

	normalizeWeights();

	MRPT_END
}

/*---------------------------------------------------------------
						getMostLikelyCovarianceAndMean
 ---------------------------------------------------------------*/
void CPosePDFSOG::getMostLikelyCovarianceAndMean(
	CMatrixDouble33& cov, CPose2D& mean_point) const
{
	auto it_best = end();
	double best_log_w = -std::numeric_limits<double>::max();

	for (auto i = begin(); i != end(); ++i)
	{
		if (i->log_w > best_log_w)
		{
			best_log_w = i->log_w;
			it_best = i;
		}
	}

	if (it_best != end())
	{
		mean_point = it_best->mean;
		cov = it_best->cov;
	}
	else
	{
		cov.setIdentity();
		cov.asEigen() *= 1e20;
		mean_point = CPose2D(0, 0, 0);
	}
}

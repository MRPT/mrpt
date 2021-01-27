/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"	// Precompiled headers
//
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/system/os.h>

#include <Eigen/Dense>

using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(CPoint2DPDFGaussian, CPoint2DPDF, mrpt::poses)

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPoint2DPDFGaussian::CPoint2DPDFGaussian() : mean(0, 0), cov() {}
/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPoint2DPDFGaussian::CPoint2DPDFGaussian(
	const CPoint2D& init_Mean, const CMatrixDouble22& init_Cov)
	: mean(init_Mean), cov(init_Cov)
{
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPoint2DPDFGaussian::CPoint2DPDFGaussian(const CPoint2D& init_Mean)
	: mean(init_Mean), cov()
{
}

uint8_t CPoint2DPDFGaussian::serializeGetVersion() const { return 0; }
void CPoint2DPDFGaussian::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << CPoint2D(mean) << cov;
}
void CPoint2DPDFGaussian::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> mean >> cov;
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}
void CPoint2DPDFGaussian::serializeTo(
	mrpt::serialization::CSchemeArchiveBase& out) const
{
	SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
	out["mean"] = mean;
	out["cov"] = CMatrixD(cov);
}
void CPoint2DPDFGaussian::serializeFrom(
	mrpt::serialization::CSchemeArchiveBase& in)
{
	uint8_t version;
	SCHEMA_DESERIALIZE_DATATYPE_VERSION();
	switch (version)
	{
		case 1:
		{
			in["mean"].readTo(mean);
			CMatrixD m;
			in["cov"].readTo(m);
			cov = m;
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

void CPoint2DPDFGaussian::copyFrom(const CPoint2DPDF& o)
{
	if (this == &o) return;	 // It may be used sometimes

	// Convert to gaussian pdf:
	o.getCovarianceAndMean(cov, mean);
}

/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
bool CPoint2DPDFGaussian::saveToTextFile(const std::string& file) const
{
	MRPT_START

	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;

	os::fprintf(f, "%f %f\n", mean.x(), mean.y());

	os::fprintf(f, "%f %f\n", cov(0, 0), cov(0, 1));
	os::fprintf(f, "%f %f\n", cov(1, 0), cov(1, 1));

	os::fclose(f);
	return true;
	MRPT_END
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPoint2DPDFGaussian::changeCoordinatesReference(
	const CPose3D& newReferenceBase)
{
	// Clip the 3x3 rotation matrix
	const auto M = newReferenceBase.getRotationMatrix().blockCopy<2, 2>();

	// The mean:
	mean = CPoint2D(newReferenceBase + mean);

	// The covariance:
	cov = M.asEigen() * cov.asEigen() * M.transpose();
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void CPoint2DPDFGaussian::bayesianFusion(
	const CPoint2DPDFGaussian& p1, const CPoint2DPDFGaussian& p2)
{
	MRPT_START

	const auto C1_inv = p1.cov.asEigen().inverse();
	const auto C2_inv = p2.cov.asEigen().inverse();

	const Eigen::Matrix2d L = C1_inv + C2_inv;

	cov.asEigen() = L.inverse();  // The new cov.

	const Eigen::Vector2d x1{p1.mean.x(), p1.mean.y()};
	const Eigen::Vector2d x2{p2.mean.x(), p2.mean.y()};

	const Eigen::Vector2d x = cov.asEigen() * (C1_inv * x1 + C2_inv * x2);
	mean.x(x[0]);
	mean.y(x[1]);

	MRPT_END
}

/*---------------------------------------------------------------
					productIntegralWith
 ---------------------------------------------------------------*/
double CPoint2DPDFGaussian::productIntegralWith(
	const CPoint2DPDFGaussian& p) const
{
	MRPT_START
	// --------------------------------------------------------------
	// 12/APR/2009 - Jose Luis Blanco:
	//  The integral over all the variable space of the product of two
	//   Gaussians variables amounts to simply the evaluation of
	//   a normal PDF at (0,0), with mean=M1-M2 and COV=COV1+COV2
	// ---------------------------------------------------------------
	// Sum of covs:
	const auto C = cov.asEigen() + p.cov.asEigen();
	const auto C_inv = C.inverse();

	const Eigen::Vector2d MU{mean.x() - p.mean.x(), mean.y() - p.mean.y()};

	return std::pow(M_2PI, -0.5 * state_length) *
		(1.0 / std::sqrt(C.determinant())) *
		exp(-0.5 * MU.transpose() * C_inv * MU);

	MRPT_END
}

/*---------------------------------------------------------------
					productIntegralNormalizedWith
 ---------------------------------------------------------------*/
double CPoint2DPDFGaussian::productIntegralNormalizedWith(
	const CPoint2DPDFGaussian& p) const
{
	return std::exp(-0.5 * square(mahalanobisDistanceTo(p)));
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPoint2DPDFGaussian::drawSingleSample(CPoint2D& outSample) const
{
	MRPT_START

	// Eigen3 emits an out-of-array warning here, but it seems to be a false
	// warning? (WTF)
	CVectorDouble vec;
	getRandomGenerator().drawGaussianMultivariate(vec, cov);

	ASSERT_(vec.size() == 2);
	outSample.x(mean.x() + vec[0]);
	outSample.y(mean.y() + vec[1]);

	MRPT_END
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void CPoint2DPDFGaussian::bayesianFusion(
	const CPoint2DPDF& p1_, const CPoint2DPDF& p2_,
	[[maybe_unused]] const double minMahalanobisDistToDrop)
{
	MRPT_START

	// p1: CPoint2DPDFGaussian, p2: CPosePDFGaussian:
	ASSERT_(p1_.GetRuntimeClass() == CLASS_ID(CPoint2DPDFGaussian));
	ASSERT_(p2_.GetRuntimeClass() == CLASS_ID(CPoint2DPDFGaussian));

	THROW_EXCEPTION("TODO!!!");

	MRPT_END
}

/*---------------------------------------------------------------
					mahalanobisDistanceTo
 ---------------------------------------------------------------*/
double CPoint2DPDFGaussian::mahalanobisDistanceTo(
	const CPoint2DPDFGaussian& other) const
{
	// The difference in means:
	const Eigen::Vector2d deltaX{
		other.mean.x() - mean.x(), other.mean.y() - mean.y()};

	// The inverse of the combined covs:
	return std::sqrt(
		deltaX.transpose() *
		(other.cov.asEigen() + this->cov.asEigen()).inverse() * deltaX);
}

/** Returns the Mahalanobis distance from this PDF to some point */
double CPoint2DPDFGaussian::mahalanobisDistanceToPoint(
	const double x, const double y) const
{
	// The difference in means:
	const Eigen::Vector2d deltaX{x - mean.x(), y - mean.y()};

	// The inverse of the combined covs:
	return std::sqrt(deltaX.transpose() * cov.asEigen().inverse() * deltaX);
}

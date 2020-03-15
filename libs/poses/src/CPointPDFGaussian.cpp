/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/math/matrix_serialization.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/poses/CPointPDFGaussian.h>
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

IMPLEMENTS_SERIALIZABLE(CPointPDFGaussian, CPointPDF, mrpt::poses)

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPointPDFGaussian::CPointPDFGaussian() : mean(0, 0, 0), cov() {}
/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPointPDFGaussian::CPointPDFGaussian(
	const CPoint3D& init_Mean, const CMatrixDouble33& init_Cov)
	: mean(init_Mean), cov(init_Cov)
{
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPointPDFGaussian::CPointPDFGaussian(const CPoint3D& init_Mean)
	: mean(init_Mean), cov()
{
	cov.setZero();
}

/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the
 PDF)
 ---------------------------------------------------------------*/
void CPointPDFGaussian::getMean(CPoint3D& p) const { p = mean; }

uint8_t CPointPDFGaussian::serializeGetVersion() const { return 1; }
void CPointPDFGaussian::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << CPoint3D(mean) << cov;
}
void CPointPDFGaussian::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> mean;

			CMatrixF c;
			in >> c;
			cov = c.cast_double();
		}
		break;
		case 1:
		{
			in >> mean >> cov;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}
void CPointPDFGaussian::serializeTo(
	mrpt::serialization::CSchemeArchiveBase& out) const
{
	SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
	out["mean"] = mean;
	out["cov"] = CMatrixD(cov);
}
void CPointPDFGaussian::serializeFrom(
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
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

void CPointPDFGaussian::copyFrom(const CPointPDF& o)
{
	if (this == &o) return;  // It may be used sometimes

	// Convert to gaussian pdf:
	o.getCovarianceAndMean(cov, mean);
}

/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
bool CPointPDFGaussian::saveToTextFile(const std::string& file) const
{
	MRPT_START
	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;
	os::fprintf(f, "%f %f %f\n", mean.x(), mean.y(), mean.z());
	os::fprintf(f, "%f %f %f\n", cov(0, 0), cov(0, 1), cov(0, 2));
	os::fprintf(f, "%f %f %f\n", cov(1, 0), cov(1, 1), cov(1, 2));
	os::fprintf(f, "%f %f %f\n", cov(2, 0), cov(2, 1), cov(2, 2));
	os::fclose(f);
	return true;
	MRPT_END
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPointPDFGaussian::changeCoordinatesReference(
	const CPose3D& newReferenceBase)
{
	const CMatrixDouble33& M = newReferenceBase.getRotationMatrix();

	// The mean:
	mean = newReferenceBase + mean;

	// The covariance:
	cov = M.asEigen() * cov.asEigen() * M.transpose();
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void CPointPDFGaussian::bayesianFusion(
	const CPointPDFGaussian& p1, const CPointPDFGaussian& p2)
{
	MRPT_START

	CMatrixDouble31 x1, x2;
	const auto C1 = p1.cov;
	const auto C2 = p2.cov;
	const CMatrixDouble33 C1_inv = C1.inverse_LLt();
	const CMatrixDouble33 C2_inv = C2.inverse_LLt();

	x1(0, 0) = p1.mean.x();
	x1(1, 0) = p1.mean.y();
	x1(2, 0) = p1.mean.z();
	x2(0, 0) = p2.mean.x();
	x2(1, 0) = p2.mean.y();
	x2(2, 0) = p2.mean.z();

	cov = CMatrixDouble33(C1_inv + C2_inv).inverse_LLt();

	auto x = cov.asEigen() * (C1_inv.asEigen() * x1.asEigen() +
							  C2_inv.asEigen() * x2.asEigen());

	mean.x(x(0, 0));
	mean.y(x(1, 0));
	mean.z(x(2, 0));

	MRPT_END
}

/*---------------------------------------------------------------
					productIntegralWith
 ---------------------------------------------------------------*/
double CPointPDFGaussian::productIntegralWith(const CPointPDFGaussian& p) const
{
	MRPT_START

	// --------------------------------------------------------------
	// 12/APR/2009 - Jose Luis Blanco:
	//  The integral over all the variable space of the product of two
	//   Gaussians variables amounts to simply the evaluation of
	//   a normal PDF at (0,0), with mean=M1-M2 and COV=COV1+COV2
	// ---------------------------------------------------------------
	CMatrixDouble33 C = cov;
	C += p.cov;  // Sum of covs
	CMatrixDouble33 C_inv = C.inverse_LLt();

	const Eigen::Vector3d MU(
		mean.x() - p.mean.x(), mean.y() - p.mean.y(), mean.z() - p.mean.z());

	return std::pow(M_2PI, -0.5 * state_length) * (1.0 / std::sqrt(C.det())) *
		   exp(-0.5 * (MU.transpose() * C_inv.asEigen() * MU)(0, 0));

	MRPT_END
}

/*---------------------------------------------------------------
					productIntegralWith2D
 ---------------------------------------------------------------*/
double CPointPDFGaussian::productIntegralWith2D(
	const CPointPDFGaussian& p) const
{
	MRPT_START

	// --------------------------------------------------------------
	// 12/APR/2009 - Jose Luis Blanco:
	//  The integral over all the variable space of the product of two
	//   Gaussians variables amounts to simply the evaluation of
	//   a normal PDF at (0,0), with mean=M1-M2 and COV=COV1+COV2
	// ---------------------------------------------------------------
	// Sum of covs:
	const auto C = cov.blockCopy<2, 2>(0, 0) + p.cov.blockCopy<2, 2>(0, 0);
	CMatrixDouble22 C_inv = C.inverse_LLt();

	const Eigen::Vector2d MU(mean.x() - p.mean.x(), mean.y() - p.mean.y());

	return std::pow(M_2PI, -0.5 * (state_length - 1)) *
		   (1.0 / std::sqrt(C.det())) *
		   exp(-0.5 * (MU.transpose() * C_inv.asEigen() * MU)(0, 0));

	MRPT_END
}

/*---------------------------------------------------------------
					productIntegralNormalizedWith
 ---------------------------------------------------------------*/
double CPointPDFGaussian::productIntegralNormalizedWith(
	const CPointPDFGaussian& p) const
{
	return std::exp(-0.5 * square(mahalanobisDistanceTo(p)));
}

/*---------------------------------------------------------------
					productIntegralNormalizedWith
 ---------------------------------------------------------------*/
double CPointPDFGaussian::productIntegralNormalizedWith2D(
	const CPointPDFGaussian& p) const
{
	return std::exp(-0.5 * square(mahalanobisDistanceTo(p, true)));
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPointPDFGaussian::drawSingleSample(CPoint3D& outSample) const
{
	MRPT_START

	CVectorDouble vec;
	getRandomGenerator().drawGaussianMultivariate(vec, cov);

	ASSERT_(vec.size() == 3);
	outSample.x(mean.x() + vec[0]);
	outSample.y(mean.y() + vec[1]);
	outSample.z(mean.z() + vec[2]);

	MRPT_END
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void CPointPDFGaussian::bayesianFusion(
	const CPointPDF& p1_, const CPointPDF& p2_,
	const double minMahalanobisDistToDrop)
{
	MRPT_UNUSED_PARAM(minMahalanobisDistToDrop);
	MRPT_START

	// p1: CPointPDFGaussian, p2: CPosePDFGaussian:
	ASSERT_(p1_.GetRuntimeClass() == CLASS_ID(CPointPDFGaussian));
	ASSERT_(p2_.GetRuntimeClass() == CLASS_ID(CPointPDFGaussian));

	THROW_EXCEPTION("TODO!!!");

	MRPT_END
}

/*---------------------------------------------------------------
					mahalanobisDistanceTo
 ---------------------------------------------------------------*/
double CPointPDFGaussian::mahalanobisDistanceTo(
	const CPointPDFGaussian& other, bool only_2D) const
{
	// The difference in means:
	CMatrixDouble13 deltaX;
	deltaX(0, 0) = other.mean.x() - mean.x();
	deltaX(0, 1) = other.mean.y() - mean.y();
	deltaX(0, 2) = other.mean.z() - mean.z();

	// The inverse of the combined covs:
	CMatrixDouble33 COV = other.cov;
	COV += this->cov;

	if (!only_2D)
	{
		const CMatrixDouble33 COV_inv = COV.inverse_LLt();
		return sqrt(mrpt::math::multiply_HCHt_scalar(deltaX, COV_inv));
	}
	else
	{
		auto C = CMatrixDouble22(COV.block<2, 2>(0, 0));
		const CMatrixDouble22 COV_inv = C.inverse_LLt();
		auto deltaX2 = CMatrixDouble12(deltaX.block<1, 2>(0, 0));
		return std::sqrt(mrpt::math::multiply_HCHt_scalar(deltaX2, COV_inv));
	}
}

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/math/TPose2D.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/system/os.h>
#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPosePDFGaussianInf, CPosePDF, mrpt::poses)

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFGaussianInf::CPosePDFGaussianInf() : mean(0, 0, 0), cov_inv() {}
/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFGaussianInf::CPosePDFGaussianInf(
	const CPose2D& init_Mean, const CMatrixDouble33& init_CovInv)
	: mean(init_Mean), cov_inv(init_CovInv)
{
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFGaussianInf::CPosePDFGaussianInf(const CPose2D& init_Mean)
	: mean(init_Mean), cov_inv()
{
}

uint8_t CPosePDFGaussianInf::serializeGetVersion() const { return 0; }
void CPosePDFGaussianInf::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << mean.x() << mean.y() << mean.phi();
	out << cov_inv(0, 0) << cov_inv(1, 1) << cov_inv(2, 2);
	out << cov_inv(0, 1) << cov_inv(0, 2) << cov_inv(1, 2);
}
void CPosePDFGaussianInf::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			TPose2D p;
			in >> p.x >> p.y >> p.phi;
			mean = CPose2D(p);

			in >> cov_inv(0, 0) >> cov_inv(1, 1) >> cov_inv(2, 2);
			in >> cov_inv(0, 1) >> cov_inv(0, 2) >> cov_inv(1, 2);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CPosePDFGaussianInf::serializeTo(
	mrpt::serialization::CSchemeArchiveBase& out) const
{
	SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
	out["mean"] = mean;
	out["cov_inv"] = CMatrixD(cov_inv);
}
void CPosePDFGaussianInf::serializeFrom(
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
			in["cov_inv"].readTo(m);
			cov_inv = m;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

/*---------------------------------------------------------------
						copyFrom
  ---------------------------------------------------------------*/
void CPosePDFGaussianInf::copyFrom(const CPosePDF& o)
{
	if (this == &o) return;  // It may be used sometimes

	if (IS_CLASS(o, CPosePDFGaussianInf))
	{  // It's my same class:
		const auto* ptr = dynamic_cast<const CPosePDFGaussianInf*>(&o);
		mean = ptr->mean;
		cov_inv = ptr->cov_inv;
	}
	else
	{  // Convert to gaussian pdf:
		o.getMean(mean);

		CMatrixDouble33 o_cov(UNINITIALIZED_MATRIX);
		o.getCovariance(o_cov);
		this->cov_inv = o_cov.inverse_LLt();
	}
}

/*---------------------------------------------------------------
						copyFrom 3D
  ---------------------------------------------------------------*/
void CPosePDFGaussianInf::copyFrom(const CPose3DPDF& o)
{
	// Convert to gaussian pdf:
	mean = CPose2D(o.getMeanVal());

	if (IS_CLASS(o, CPose3DPDFGaussianInf))
	{  // Cov is already in information form:
		const auto* ptr = dynamic_cast<const CPose3DPDFGaussianInf*>(&o);
		cov_inv(0, 0) = ptr->cov_inv(0, 0);
		cov_inv(1, 1) = ptr->cov_inv(1, 1);
		cov_inv(2, 2) = ptr->cov_inv(3, 3);
		cov_inv(0, 1) = cov_inv(1, 0) = ptr->cov_inv(0, 1);
		cov_inv(0, 2) = cov_inv(2, 0) = ptr->cov_inv(0, 3);
		cov_inv(1, 2) = cov_inv(2, 1) = ptr->cov_inv(1, 3);
	}
	else
	{
		CMatrixDouble66 C(UNINITIALIZED_MATRIX);
		o.getCovariance(C);

		// Clip to 3x3:
		CMatrixDouble33 o_cov(UNINITIALIZED_MATRIX);
		o_cov(0, 0) = C(0, 0);
		o_cov(1, 1) = C(1, 1);
		o_cov(2, 2) = C(3, 3);
		o_cov(0, 1) = o_cov(1, 0) = C(0, 1);
		o_cov(0, 2) = o_cov(2, 0) = C(0, 3);
		o_cov(1, 2) = o_cov(2, 1) = C(1, 3);

		this->cov_inv = o_cov.inverse_LLt();
	}
}

/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
bool CPosePDFGaussianInf::saveToTextFile(const std::string& file) const
{
	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;

	os::fprintf(f, "%f %f %f\n", mean.x(), mean.y(), mean.phi());

	for (unsigned int i = 0; i < 3; i++)
		os::fprintf(
			f, "%f %f %f\n", cov_inv(i, 0), cov_inv(i, 1), cov_inv(i, 2));

	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::changeCoordinatesReference(
	const CPose3D& newReferenceBase_)
{
	const CPose2D newReferenceBase = CPose2D(newReferenceBase_);

	// The mean:
	mean.composeFrom(newReferenceBase, mean);

	// The covariance:
	rotateCov(newReferenceBase.phi());
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::changeCoordinatesReference(
	const CPose2D& newReferenceBase)
{
	// The mean:
	mean.composeFrom(newReferenceBase, mean);
	// The covariance:
	rotateCov(newReferenceBase.phi());
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::rotateCov(const double ang)
{
	const double ccos = cos(ang);
	const double ssin = sin(ang);

	alignas(MRPT_MAX_STATIC_ALIGN_BYTES)
		const double rot_vals[] = {ccos, -ssin, 0., ssin, ccos, 0., 0., 0., 1.};

	const CMatrixFixed<double, 3, 3> rot(rot_vals);

	// NEW_COV = H C H^T
	// NEW_COV^(-1) = (H C H^T)^(-1) = (H^T)^(-1) C^(-1) H^(-1)
	// rot: Inverse of a rotation matrix is its trasposed.
	//      But we need H^t^-1 -> H !! so rot stays unchanged:
	cov_inv = rot.asEigen() * cov_inv.asEigen() * rot.asEigen().transpose();
}

void CPosePDFGaussianInf::drawSingleSample(CPose2D& outPart) const
{
	MRPT_START

	const CMatrixDouble33 cov = this->cov_inv.inverse_LLt();

	CVectorDouble v;
	getRandomGenerator().drawGaussianMultivariate(v, cov);

	outPart.x(mean.x() + v[0]);
	outPart.y(mean.y() + v[1]);
	outPart.phi(mean.phi() + v[2]);

	// Range -pi,pi
	outPart.normalizePhi();

	MRPT_END_WITH_CLEAN_UP(cov_inv.saveToTextFile(
		"__DEBUG_EXC_DUMP_drawSingleSample_COV_INV.txt"););
}

void CPosePDFGaussianInf::drawManySamples(
	size_t N, std::vector<CVectorDouble>& outSamples) const
{
	MRPT_START

	const CMatrixDouble33 cov = this->cov_inv.inverse_LLt();

	std::vector<CVectorDouble> rndSamples;

	getRandomGenerator().drawGaussianMultivariateMany(rndSamples, N, cov);
	outSamples.resize(N);
	for (size_t i = 0; i < N; i++)
	{
		outSamples[i].resize(3);
		outSamples[i][0] = mean.x() + rndSamples[i][0];
		outSamples[i][1] = mean.y() + rndSamples[i][1];
		outSamples[i][2] = mean.phi() + rndSamples[i][2];

		wrapToPiInPlace(outSamples[i][2]);
	}

	MRPT_END
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::bayesianFusion(
	const CPosePDF& p1_, const CPosePDF& p2_,
	const double minMahalanobisDistToDrop)
{
	MRPT_START

	MRPT_UNUSED_PARAM(minMahalanobisDistToDrop);  // Not used in this class!

	ASSERT_(p1_.GetRuntimeClass() == CLASS_ID(CPosePDFGaussianInf));
	ASSERT_(p2_.GetRuntimeClass() == CLASS_ID(CPosePDFGaussianInf));

	const auto* p1 = dynamic_cast<const CPosePDFGaussianInf*>(&p1_);
	const auto* p2 = dynamic_cast<const CPosePDFGaussianInf*>(&p2_);

	const CMatrixDouble33& C1_inv = p1->cov_inv;
	const CMatrixDouble33& C2_inv = p2->cov_inv;

	auto x1 = CMatrixDouble31(p1->mean);
	auto x2 = CMatrixDouble31(p2->mean);

	this->cov_inv = C1_inv + C2_inv;

	const CMatrixDouble33 cov = this->cov_inv.inverse_LLt();

	auto x = CMatrixDouble31(cov.asEigen() * (C1_inv * x1 + C2_inv * x2));

	this->mean.x(x(0, 0));
	this->mean.y(x(1, 0));
	this->mean.phi(x(2, 0));
	this->mean.normalizePhi();

	MRPT_END
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::inverse(CPosePDF& o) const
{
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPosePDFGaussianInf));
	auto* out = dynamic_cast<CPosePDFGaussianInf*>(&o);

	// The mean:
	out->mean = CPose2D(0, 0, 0) - mean;

	// The covariance:
	const double ccos = ::cos(mean.phi());
	const double ssin = ::sin(mean.phi());

	// jacobian:
	alignas(MRPT_MAX_STATIC_ALIGN_BYTES) const double H_values[] = {
		-ccos, -ssin, mean.x() * ssin - mean.y() * ccos,
		ssin,  -ccos, mean.x() * ccos + mean.y() * ssin,
		0,	 0,	 -1};
	const CMatrixFixed<double, 3, 3> H(H_values);

	// o.cov = H * cov * Ht. It's the same with inverse covariances.
	out->cov_inv.asEigen().noalias() =
		(H.asEigen() * cov_inv.asEigen() * H.transpose()).eval();
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::operator+=(const CPose2D& Ap)
{
	mean = mean + Ap;
	rotateCov(Ap.phi());
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double CPosePDFGaussianInf::evaluatePDF(const CPose2D& x) const
{
	auto X = CMatrixDouble31(x);
	auto MU = CMatrixDouble31(mean);

	return math::normalPDF(X, MU, cov_inv.inverse());
}

/*---------------------------------------------------------------
						evaluateNormalizedPDF
 ---------------------------------------------------------------*/
double CPosePDFGaussianInf::evaluateNormalizedPDF(const CPose2D& x) const
{
	auto X = CMatrixDouble31(x);
	auto MU = CMatrixDouble31(mean);

	const CMatrixDouble33 cov = this->cov_inv.inverse_LLt();

	return math::normalPDF(X, MU, cov) / math::normalPDF(MU, MU, cov);
}

/*---------------------------------------------------------------
						enforceCovSymmetry
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::enforceCovSymmetry()
{
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	cov_inv(0, 1) = cov_inv(1, 0);
	cov_inv(0, 2) = cov_inv(2, 0);
	cov_inv(1, 2) = cov_inv(2, 1);
}

/*---------------------------------------------------------------
						mahalanobisDistanceTo
 ---------------------------------------------------------------*/
double CPosePDFGaussianInf::mahalanobisDistanceTo(
	const CPosePDFGaussianInf& theOther)
{
	MRPT_START

	auto MU = CVectorFixedDouble<3>(mean);
	MU -= CVectorFixedDouble<3>(theOther.mean);

	wrapToPiInPlace(MU[2]);

	if (MU[0] == 0 && MU[1] == 0 && MU[2] == 0)
		return 0;  // This is the ONLY case where we know the result, whatever
	// COVs are.

	CMatrixDouble33 COV_ = this->cov_inv.inverse_LLt();
	const CMatrixDouble33 cov2 = theOther.cov_inv.inverse_LLt();
	COV_ += cov2;  // COV_ = cov1+cov2

	const CMatrixDouble33 COV_inv = COV_.inverse_LLt();

	// (~MU) * (!COV_) * MU
	return std::sqrt(mrpt::math::multiply_HtCH_scalar(MU.asEigen(), COV_inv));

	MRPT_END
}

/*---------------------------------------------------------------
						operator <<
 ---------------------------------------------------------------*/
std::ostream& mrpt::poses::operator<<(
	std::ostream& out, const CPosePDFGaussianInf& obj)
{
	out << "Mean: " << obj.mean << "\n";
	out << "Inverse cov:\n" << obj.cov_inv << "\n";

	return out;
}

/*---------------------------------------------------------------
						operator +
 ---------------------------------------------------------------*/
poses::CPosePDFGaussianInf operator+(
	const mrpt::poses::CPose2D& A, const mrpt::poses::CPosePDFGaussianInf& B)
{
	poses::CPosePDFGaussianInf ret(B);
	ret.changeCoordinatesReference(A);
	return ret;
}

/*---------------------------------------------------------------
						inverseComposition
  Set 'this' = 'x' - 'ref', computing the mean using the "-"
	operator and the covariances through the corresponding Jacobians.
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::inverseComposition(
	const CPosePDFGaussianInf& xv, const CPosePDFGaussianInf& xi)
{
	// Use implementation in CPosePDFGaussian:
	const CMatrixDouble33 xv_cov = xv.cov_inv.inverse_LLt();
	const CMatrixDouble33 xi_cov = xi.cov_inv.inverse_LLt();

	const CPosePDFGaussian xv_(xv.mean, xv_cov);
	const CPosePDFGaussian xi_(xi.mean, xi_cov);

	CPosePDFGaussian RET;
	RET.inverseComposition(xv_, xi_);

	// Copy result to "this":
	this->mean = RET.mean;
	this->cov_inv = RET.cov.inverse_LLt();
}

/*---------------------------------------------------------------
						inverseComposition
  Set \f$ this = x1 \ominus x0 \f$ , computing the mean using
   the "-" operator and the covariances through the corresponding
	Jacobians (Given the 3x3 cross-covariance matrix of variables x0 and x0).
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::inverseComposition(
	const CPosePDFGaussianInf& x1, const CPosePDFGaussianInf& x0,
	const CMatrixDouble33& COV_01)
{
	// Use implementation in CPosePDFGaussian:
	const CMatrixDouble33 x1_cov = x1.cov_inv.inverse_LLt();
	const CMatrixDouble33 x0_cov = x0.cov_inv.inverse_LLt();

	const CPosePDFGaussian x1_(x1.mean, x1_cov);
	const CPosePDFGaussian x0_(x0.mean, x0_cov);

	CPosePDFGaussian RET;
	RET.inverseComposition(x1_, x0_, COV_01);

	// Copy result to "this":
	this->mean = RET.mean;
	this->cov_inv = RET.cov.inverse_LLt();
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::operator+=(const CPosePDFGaussianInf& Ap)
{
	// COV:
	const CMatrixDouble33 OLD_COV = this->cov_inv.inverse_LLt();

	CMatrixDouble33 df_dx, df_du;

	CPosePDF::jacobiansPoseComposition(
		this->mean,  // x
		Ap.mean,  // u
		df_dx, df_du);

	const CMatrixDouble33 Ap_cov = Ap.cov_inv.inverse_LLt();

	this->cov_inv =
		(multiply_HCHt(df_dx, OLD_COV) + multiply_HCHt(df_du, Ap_cov))
			.inverse_LLt();

	// MEAN:
	this->mean = this->mean + Ap.mean;
}

bool mrpt::poses::operator==(
	const CPosePDFGaussianInf& p1, const CPosePDFGaussianInf& p2)
{
	return p1.mean == p2.mean && p1.cov_inv == p2.cov_inv;
}

/** Pose compose operator: RES = A (+) B , computing both the mean and the
 * covariance */
CPosePDFGaussianInf mrpt::poses::operator+(
	const CPosePDFGaussianInf& a, const CPosePDFGaussianInf& b)
{
	CPosePDFGaussianInf res(a);
	res += b;
	return res;
}

/** Pose inverse compose operator: RES = A (-) B , computing both the mean and
 * the covariance */
CPosePDFGaussianInf mrpt::poses::operator-(
	const CPosePDFGaussianInf& a, const CPosePDFGaussianInf& b)
{
	CPosePDFGaussianInf res;
	res.inverseComposition(a, b);
	return res;
}

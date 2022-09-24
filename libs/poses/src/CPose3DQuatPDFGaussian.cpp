/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"	// Precompiled headers
//
#include <mrpt/math/distributions.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/math/transform_gaussian.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;

bool USE_SUT_EULER2QUAT_CONVERSION_value = false;

bool mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION()
{
	return USE_SUT_EULER2QUAT_CONVERSION_value;
}
void mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION(bool value)
{
	USE_SUT_EULER2QUAT_CONVERSION_value = value;
}

IMPLEMENTS_SERIALIZABLE(CPose3DQuatPDFGaussian, CPose3DQuatPDF, mrpt::poses)

/** Default constructor - set all values to zero. */
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian() : mean(), cov() {}
// Un-initialized constructor:
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian(	 //
	[[maybe_unused]] TConstructorFlags_Quaternions constructor_dummy_param)
	: mean(UNINITIALIZED_QUATERNION), cov(UNINITIALIZED_MATRIX)
{
}

/** Constructor from a default mean value, covariance equals to zero. */
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian(const CPose3DQuat& init_Mean)
	: mean(init_Mean), cov()
{
}

/** Constructor with mean and covariance. */
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian(
	const CPose3DQuat& init_Mean, const CMatrixDouble77& init_Cov)
	: mean(init_Mean), cov(init_Cov)
{
}

/** Constructor from a Gaussian 2D pose PDF (sets to 0 the missing variables).
 */
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian(const CPosePDFGaussian& o)
	: mean(UNINITIALIZED_QUATERNION), cov(UNINITIALIZED_MATRIX)
{
	this->copyFrom(CPose3DPDFGaussian(o));
}

/** Constructor from an equivalent Gaussian in Euler angles representation. */
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian(const CPose3DPDFGaussian& o)
	: mean(UNINITIALIZED_QUATERNION), cov(UNINITIALIZED_MATRIX)
{
	this->copyFrom(o);
}

void CPose3DQuatPDFGaussian::getMean(CPose3DQuat& p) const { p = mean; }

uint8_t CPose3DQuatPDFGaussian::serializeGetVersion() const { return 0; }
void CPose3DQuatPDFGaussian::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << mean;
	mrpt::math::serializeSymmetricMatrixTo(cov, out);
}
void CPose3DQuatPDFGaussian::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> mean;
			mrpt::math::deserializeSymmetricMatrixFrom(cov, in);
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CPose3DQuatPDFGaussian::copyFrom(const CPose3DQuatPDF& o)
{
	if (this == &o) return;	 // It may be used sometimes

	// Convert to gaussian pdf:
	o.getCovarianceAndMean(cov, mean);
}

void CPose3DQuatPDFGaussian::copyFrom(const CPosePDF& o)
{
	CPose3DPDFGaussian aux;
	aux.copyFrom(o);
	this->copyFrom(aux);
}

void aux_poseypr2posequat(
	const CVectorFixedDouble<6>& x, [[maybe_unused]] const double& dummy,
	CVectorFixedDouble<7>& y)
{
	y[0] = x[0];
	y[1] = x[1];
	y[2] = x[2];

	CPose3D p(0, 0, 0, x[3], x[4], x[5]);
	CQuaternionDouble q(UNINITIALIZED_QUATERNION);
	p.getAsQuaternion(q);
	y[3] = q[0];
	y[4] = q[1];
	y[5] = q[2];
	y[6] = q[3];
}

void CPose3DQuatPDFGaussian::copyFrom(const CPose3DPDFGaussian& o)
{
	if (!USE_SUT_EULER2QUAT_CONVERSION_value)
	{  // Use Jacobians
		CMatrixFixed<double, 4, 3> dq_dr_sub(UNINITIALIZED_MATRIX);

		// Mean:
		mean.x(o.mean.x());
		mean.y(o.mean.y());
		mean.z(o.mean.z());

		o.mean.getAsQuaternion(mean.quat(), dq_dr_sub);

		// Cov:
		CMatrixFixed<double, 7, 6> dq_dr;
		dq_dr(0, 0) = dq_dr(1, 1) = dq_dr(2, 2) = 1;
		dq_dr.insertMatrix(3, 3, dq_dr_sub);
		// Now for the covariance:
		this->cov = mrpt::math::multiply_HCHt(dq_dr, o.cov);
	}
	else
	{
		// Use UT transformation:
		//   f: R^6 => R^7
		const CVectorFixedDouble<6> x_mean(o.mean);

		const double dummy = 0;
		mrpt::math::transform_gaussian_unscented(
			x_mean, o.cov, &aux_poseypr2posequat, dummy, this->mean, this->cov);
	}
}

/*---------------------------------------------------------------
					saveToTextFile
  ---------------------------------------------------------------*/
bool CPose3DQuatPDFGaussian::saveToTextFile(const string& file) const
{
	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;

	os::fprintf(
		f, "%e %e %e %e %e %e %e\n", mean.x(), mean.y(), mean.z(),
		mean.quat()[0], mean.quat()[1], mean.quat()[2], mean.quat()[3]);

	for (unsigned int i = 0; i < 7; i++)
		os::fprintf(
			f, "%e %e %e %e %e %e %e\n", cov(i, 0), cov(i, 1), cov(i, 2),
			cov(i, 3), cov(i, 4), cov(i, 5), cov(i, 6));

	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussian::changeCoordinatesReference(
	const CPose3D& newReferenceBase)
{
	MRPT_START
	changeCoordinatesReference(CPose3DQuat(newReferenceBase));
	MRPT_END
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussian::changeCoordinatesReference(
	const CPose3DQuat& newReferenceBaseQuat)
{
	MRPT_START

	// COV:
	const CMatrixDouble77 OLD_COV = this->cov;
	CMatrixDouble77 df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		newReferenceBaseQuat,  // x
		this->mean,	 // u
		df_dx, df_du,
		&this->mean	 // Output:  newReferenceBaseQuat + this->mean;
	);

	// this->cov = H1*this->cov*H1' + H2*Ap.cov*H2';
	// df_dx: not used, since its COV are all zeros...
	cov = mrpt::math::multiply_HCHt(df_du, OLD_COV);

	MRPT_END
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussian::drawSingleSample(CPose3DQuat& outPart) const
{
	MRPT_START
	getRandomGenerator().drawGaussianMultivariate(outPart, cov, &mean);
	MRPT_END
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussian::drawManySamples(
	size_t N, vector<CVectorDouble>& outSamples) const
{
	MRPT_START

	getRandomGenerator().drawGaussianMultivariateMany(outSamples, N, cov);

	for (auto& outSample : outSamples)
		for (unsigned int k = 0; k < 7; k++)
			outSample[k] += mean[k];

	MRPT_END
}

/*---------------------------------------------------------------
					inverse Jacobian
 ---------------------------------------------------------------*/
mrpt::math::CMatrixDouble77 CPose3DQuatPDFGaussian::inverseJacobian() const
{
	CMatrixFixed<double, 3, 7> df_dpose(UNINITIALIZED_MATRIX);
	double lx, ly, lz;
	this->mean.inverseComposePoint(0, 0, 0, lx, ly, lz, nullptr, &df_dpose);

	CMatrixFixed<double, 7, 7> jacob;
	jacob.insertMatrix(0, 0, df_dpose);
	jacob(3, 3) = 1;
	jacob(4, 4) = -1;
	jacob(5, 5) = -1;
	jacob(6, 6) = -1;

	CMatrixDouble44 norm_jacob(UNINITIALIZED_MATRIX);
	this->mean.quat().normalizationJacobian(norm_jacob);
	jacob.asEigen().block<4, 4>(3, 3) *= norm_jacob.asEigen();

	return jacob;
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussian::inverse(CPose3DQuatPDF& o) const
{
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPose3DQuatPDFGaussian));
	auto& out = dynamic_cast<CPose3DQuatPDFGaussian&>(o);
	double lx, ly, lz;
	this->mean.inverseComposePoint(0, 0, 0, lx, ly, lz, nullptr, nullptr);

	CMatrixFixed<double, 7, 7> jacob = this->inverseJacobian();

	// C(0:2,0:2): H C H^t
	out.cov = mrpt::math::multiply_HCHt(jacob, this->cov);

	// Mean:
	out.mean.x(lx);
	out.mean.y(ly);
	out.mean.z(lz);
	this->mean.quat().conj(out.mean.quat());
}

/*---------------------------------------------------------------
			inverse composition with cross correlation
 ---------------------------------------------------------------*/
mrpt::poses::CPose3DQuatPDFGaussian
	CPose3DQuatPDFGaussian::inverseCompositionCrossCorrelation(
		const mrpt::poses::CPose3DQuatPDFGaussian& pose_to) const
{
	mrpt::poses::CPose3DQuatPDFGaussian pose_from_inv;
	this->inverse(pose_from_inv);

	CMatrixFixed<double, 7, 7> jacob_inv = this->inverseJacobian();

	mrpt::math::CMatrixDouble44 norm_jacob_pose_from(
		mrpt::math::UNINITIALIZED_MATRIX);
	this->mean.quat().normalizationJacobian(norm_jacob_pose_from);

	mrpt::math::CMatrixDouble77 Hc1, Hc2, Hc1i, Hc2i;
	mrpt::poses::CPose3DQuatPDFGaussian::jacobiansPoseComposition(
		pose_from_inv.mean, pose_to.mean, Hc1, Hc2);

	mrpt::math::CMatrixDouble44 norm_jacob_pose_from_inv(
		mrpt::math::UNINITIALIZED_MATRIX);
	pose_from_inv.mean.quat().normalizationJacobian(norm_jacob_pose_from_inv);
	Hc1.asEigen().block<4, 4>(3, 3) *= norm_jacob_pose_from_inv.asEigen();

	mrpt::math::CMatrixDouble44 norm_jacob_pose_to(
		mrpt::math::UNINITIALIZED_MATRIX);
	pose_to.mean.quat().normalizationJacobian(norm_jacob_pose_to);
	Hc2.asEigen().block<4, 4>(3, 3) *= norm_jacob_pose_to.asEigen();

	mrpt::poses::CPose3DQuatPDFGaussian displacement(pose_from_inv + pose_to);
	mrpt::poses::CPose3DQuatPDFGaussian::jacobiansPoseComposition(
		this->mean, displacement.mean, Hc1i, Hc2i);
	Hc1i.asEigen().block<4, 4>(3, 3) *= norm_jacob_pose_from.asEigen();

	mrpt::math::CMatrixDouble77 cov_correlation(
		Hc1i.asEigen() * this->cov.asEigen());
	mrpt::math::CMatrixDouble77 cov_correlation_jac(
		Hc2.asEigen() * cov_correlation.asEigen() *
		jacob_inv.asEigen().transpose() * Hc1.asEigen().transpose());

	displacement.cov.asEigen() += cov_correlation_jac.asEigen();
	displacement.cov.asEigen() += cov_correlation_jac.asEigen().transpose();

	for (int i = 0; i < displacement.cov.rows(); i++)
		ASSERT_(displacement.cov(i, i) >= 0.0);

	return displacement;
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussian::operator+=(const CPose3DQuat& Ap)
{
	// COV:
	const CMatrixDouble77 OLD_COV = this->cov;
	CMatrixDouble77 df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		this->mean,	 // x
		Ap,	 // u
		df_dx, df_du,
		&this->mean	 // Output: this->mean + Ap;
	);

	// this->cov = H1*this->cov*H1' + H2*Ap.cov*H2';
	cov = mrpt::math::multiply_HCHt(df_dx, OLD_COV);
	// df_du: Nothing to do, since COV(Ap) = zeros
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussian::operator+=(const CPose3DQuatPDFGaussian& Ap)
{
	// COV:
	const CMatrixDouble77 OLD_COV = this->cov;
	CMatrixDouble77 df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		this->mean,	 // x
		Ap.mean,  // u
		df_dx, df_du,
		&this->mean	 // Output:  this->mean + Ap.mean;
	);

	// this->cov = H1*this->cov*H1' + H2*Ap.cov*H2';
	cov = mrpt::math::multiply_HCHt(df_dx, OLD_COV);
	cov += mrpt::math::multiply_HCHt(df_du, Ap.cov);
}

/*---------------------------------------------------------------
							-=
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussian::operator-=(const CPose3DQuatPDFGaussian& Ap)
{
	// THIS = THIS (-) Ap             -->
	// THIS = inverse(Ap) (+) THIS
	CPose3DQuatPDFGaussian inv_Ap = -Ap;
	*this = inv_Ap + *this;
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double CPose3DQuatPDFGaussian::evaluatePDF(const CPose3DQuat& x) const
{
	return mrpt::math::normalPDF(
		CMatrixDouble71(x), CMatrixDouble71(this->mean), this->cov);
}

/*---------------------------------------------------------------
						evaluateNormalizedPDF
 ---------------------------------------------------------------*/
double CPose3DQuatPDFGaussian::evaluateNormalizedPDF(const CPose3DQuat& x) const
{
	return mrpt::math::normalPDF(
		CMatrixDouble71(x), CMatrixDouble71(this->mean), this->cov, true);
}

/*---------------------------------------------------------------
						enforceCovSymmetry
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussian::enforceCovSymmetry()
{
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	for (int i = 0; i < cov.rows() - 1; i++)
		for (int j = i + 1; j < cov.rows(); j++)
			cov(i, j) = cov(j, i);
}

/*---------------------------------------------------------------
						mahalanobisDistanceTo
 ---------------------------------------------------------------*/
double CPose3DQuatPDFGaussian::mahalanobisDistanceTo(
	const CPose3DQuatPDFGaussian& theOther)
{
	MRPT_START
	const CMatrixDouble77 COV2 = cov + theOther.cov;
	return mrpt::math::mahalanobisDistance(
		CMatrixDouble71(this->mean) - CMatrixDouble71(theOther.mean), COV2);
	MRPT_END
}

/*---------------------------------------------------------------
						operator <<
 ---------------------------------------------------------------*/
ostream& mrpt::poses::operator<<(
	ostream& out, const CPose3DQuatPDFGaussian& obj)
{
	out << "Mean: " << obj.mean << "\n";
	out << "Covariance:\n" << obj.cov << "\n";
	return out;
}

bool mrpt::poses::operator==(
	const CPose3DQuatPDFGaussian& p1, const CPose3DQuatPDFGaussian& p2)
{
	return p1.mean == p2.mean && p1.cov == p2.cov;
}

CPose3DQuatPDFGaussian mrpt::poses::operator+(
	const CPose3DQuatPDFGaussian& x, const CPose3DQuatPDFGaussian& u)
{
	CPose3DQuatPDFGaussian res(x);
	res += u;
	return res;
}

CPose3DQuatPDFGaussian mrpt::poses::operator-(
	const CPose3DQuatPDFGaussian& x, const CPose3DQuatPDFGaussian& u)
{
	CPose3DQuatPDFGaussian res(x);
	res -= u;
	return res;
}

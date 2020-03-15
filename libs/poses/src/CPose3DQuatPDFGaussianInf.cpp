/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/math/CMatrixFixed.h>  // for CMatrixF...
#include <mrpt/math/CQuaternion.h>  // for CQuatern...
#include <mrpt/math/distributions.h>
#include <mrpt/poses/CPose3D.h>  // for CPose3D
#include <mrpt/poses/CPose3DQuat.h>  // for CPose3DQuat
#include <mrpt/poses/CPose3DQuatPDF.h>  // for CPose3DQ...
#include <mrpt/poses/CPose3DQuatPDFGaussianInf.h>
#include <mrpt/random/RandomGenerators.h>  // for CRandomG...
#include <mrpt/serialization/CSerializable.h>  // for CSeriali...
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/os.h>  // for fopen
#include <Eigen/Dense>
#include <algorithm>  // for move, max
#include <cstdio>  // for size_t
#include <exception>  // for exception
#include <new>  // for operator...
#include <ostream>  // for operator<<
#include <string>  // for allocator
#include <vector>  // for vector

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPose3DQuatPDFGaussianInf, CPose3DQuatPDF, mrpt::poses)

/** Default constructor - set all values to zero. */
CPose3DQuatPDFGaussianInf::CPose3DQuatPDFGaussianInf() : mean(), cov_inv() {}
// Un-initialized constructor:
CPose3DQuatPDFGaussianInf::CPose3DQuatPDFGaussianInf(
	TConstructorFlags_Quaternions constructor_dummy_param)
	: mean(UNINITIALIZED_QUATERNION), cov_inv(UNINITIALIZED_MATRIX)
{
	MRPT_UNUSED_PARAM(constructor_dummy_param);
}

/** Constructor from a default mean value, covariance equals to zero. */
CPose3DQuatPDFGaussianInf::CPose3DQuatPDFGaussianInf(
	const CPose3DQuat& init_Mean)
	: mean(init_Mean), cov_inv()
{
}

/** Constructor with mean and covariance. */
CPose3DQuatPDFGaussianInf::CPose3DQuatPDFGaussianInf(
	const CPose3DQuat& init_Mean, const CMatrixDouble77& init_CovInv)
	: mean(init_Mean), cov_inv(init_CovInv)
{
}

uint8_t CPose3DQuatPDFGaussianInf::serializeGetVersion() const { return 0; }
void CPose3DQuatPDFGaussianInf::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << mean;

	for (int r = 0; r < cov_inv.rows(); r++) out << cov_inv(r, r);
	for (int r = 0; r < cov_inv.rows(); r++)
		for (int c = r + 1; c < cov_inv.cols(); c++) out << cov_inv(r, c);
}
void CPose3DQuatPDFGaussianInf::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> mean;

			for (int r = 0; r < cov_inv.rows(); r++) in >> cov_inv(r, r);
			for (int r = 0; r < cov_inv.rows(); r++)
				for (int c = r + 1; c < cov_inv.cols(); c++)
				{
					double x;
					in >> x;
					cov_inv(r, c) = cov_inv(c, r) = x;
				}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CPose3DQuatPDFGaussianInf::copyFrom(const CPose3DQuatPDF& o)
{
	if (this == &o) return;  // It may be used sometimes

	// Convert to gaussian pdf:
	CMatrixDouble77 C(UNINITIALIZED_MATRIX);
	o.getCovarianceAndMean(C, this->mean);
	this->cov_inv = C.inverse_LLt();
}

/*---------------------------------------------------------------
					saveToTextFile
  ---------------------------------------------------------------*/
bool CPose3DQuatPDFGaussianInf::saveToTextFile(const string& file) const
{
	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;

	os::fprintf(
		f, "%e %e %e %e %e %e %e\n", mean.x(), mean.y(), mean.z(),
		mean.quat()[0], mean.quat()[1], mean.quat()[2], mean.quat()[3]);

	for (unsigned int i = 0; i < 7; i++)
		os::fprintf(
			f, "%e %e %e %e %e %e %e\n", cov_inv(i, 0), cov_inv(i, 1),
			cov_inv(i, 2), cov_inv(i, 3), cov_inv(i, 4), cov_inv(i, 5),
			cov_inv(i, 6));

	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussianInf::changeCoordinatesReference(
	const CPose3D& newReferenceBase)
{
	MRPT_START
	changeCoordinatesReference(CPose3DQuat(newReferenceBase));
	MRPT_END
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussianInf::changeCoordinatesReference(
	const CPose3DQuat& newReferenceBaseQuat)
{
	MRPT_START

	// COV:
	const CMatrixDouble77 OLD_COV = this->cov_inv.inverse_LLt();

	CMatrixDouble77 df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		newReferenceBaseQuat,  // x
		this->mean,  // u
		df_dx, df_du,
		&this->mean  // Output:  newReferenceBaseQuat + this->mean;
	);

	// this->cov = H1*this->cov*H1' + H2*Ap.cov*H2';
	// df_dx: not used, since its COV are all zeros...

	this->cov_inv = mrpt::math::multiply_HCHt(df_du, OLD_COV).inverse_LLt();

	MRPT_END
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussianInf::drawSingleSample(CPose3DQuat& outPart) const
{
	MRPT_START
	const CMatrixDouble77 COV = this->cov_inv.inverse_LLt();

	getRandomGenerator().drawGaussianMultivariate(outPart, COV, &mean);
	MRPT_END
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussianInf::drawManySamples(
	size_t N, vector<CVectorDouble>& outSamples) const
{
	MRPT_START
	const CMatrixDouble77 COV = this->cov_inv.inverse_LLt();

	getRandomGenerator().drawGaussianMultivariateMany(outSamples, N, COV);

	for (auto& outSample : outSamples)
		for (unsigned int k = 0; k < 7; k++) outSample[k] += mean[k];

	MRPT_END
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussianInf::inverse(CPose3DQuatPDF& o) const
{
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPose3DQuatPDFGaussianInf));
	auto& out = dynamic_cast<CPose3DQuatPDFGaussianInf&>(o);

	// COV:
	CMatrixFixed<double, 3, 7> df_dpose(UNINITIALIZED_MATRIX);
	double lx, ly, lz;
	mean.inverseComposePoint(0, 0, 0, lx, ly, lz, nullptr, &df_dpose);

	CMatrixFixed<double, 7, 7> jacob;
	jacob.insertMatrix(0, 0, df_dpose);
	jacob(3, 3) = 1;
	jacob(4, 4) = -1;
	jacob(5, 5) = -1;
	jacob(6, 6) = -1;

	// C(0:2,0:2): H C H^t
	const CMatrixDouble77 COV = this->cov_inv.inverse_LLt();
	out.cov_inv = mrpt::math::multiply_HCHt(jacob, COV).inverse_LLt();

	// Mean:
	out.mean.x(lx);
	out.mean.y(ly);
	out.mean.z(lz);
	this->mean.quat().conj(out.mean.quat());
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussianInf::operator+=(const CPose3DQuat& Ap)
{
	// COV:
	const CMatrixDouble77 OLD_COV = this->cov_inv.inverse_LLt();

	CMatrixDouble77 df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		this->mean,  // x
		Ap,  // u
		df_dx, df_du,
		&this->mean  // Output: this->mean + Ap;
	);

	// this->cov = H1*this->cov*H1' + H2*Ap.cov*H2';
	this->cov_inv = mrpt::math::multiply_HCHt(df_dx, OLD_COV).inverse_LLt();
	// df_du: Nothing to do, since COV(Ap) = zeros
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussianInf::operator+=(const CPose3DQuatPDFGaussianInf& Ap)
{
	// COV:
	const CMatrixDouble77 OLD_COV = this->cov_inv.inverse_LLt();

	CMatrixDouble77 df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		this->mean,  // x
		Ap.mean,  // u
		df_dx, df_du,
		&this->mean  // Output:  this->mean + Ap.mean;
	);

	// this->cov = H1*this->cov*H1' + H2*Ap.cov*H2';
	const CMatrixDouble77 Ap_cov = Ap.cov_inv.inverse_LLt();
	CMatrixDouble77 NEW_COV = mrpt::math::multiply_HCHt(df_dx, OLD_COV) +
							  mrpt::math::multiply_HCHt(df_du, Ap_cov);

	this->cov_inv = NEW_COV.inverse_LLt();
}

/*---------------------------------------------------------------
							-=
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussianInf::operator-=(const CPose3DQuatPDFGaussianInf& Ap)
{
	// THIS = THIS (-) Ap             -->
	// THIS = inverse(Ap) (+) THIS
	CPose3DQuatPDFGaussianInf inv_Ap = -Ap;
	*this = inv_Ap + *this;
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double CPose3DQuatPDFGaussianInf::evaluatePDF(const CPose3DQuat& x) const
{
	return mrpt::math::normalPDFInf(
		CMatrixDouble71(x), CMatrixDouble71(this->mean), this->cov_inv);
}

/*---------------------------------------------------------------
						evaluateNormalizedPDF
 ---------------------------------------------------------------*/
double CPose3DQuatPDFGaussianInf::evaluateNormalizedPDF(
	const CPose3DQuat& x) const
{
	return mrpt::math::normalPDFInf(
		CMatrixDouble71(x), CMatrixDouble71(this->mean), this->cov_inv, true);
}

/*---------------------------------------------------------------
						operator <<
 ---------------------------------------------------------------*/
ostream& mrpt::poses::operator<<(
	ostream& out, const CPose3DQuatPDFGaussianInf& obj)
{
	out << "Mean: " << obj.mean << "\n";
	out << "Information:\n" << obj.cov_inv << "\n";
	return out;
}

bool mrpt::poses::operator==(
	const CPose3DQuatPDFGaussianInf& p1, const CPose3DQuatPDFGaussianInf& p2)
{
	return p1.mean == p2.mean && p1.cov_inv == p2.cov_inv;
}

/** Pose composition for two 3D pose Gaussians  \sa
 * CPose3DQuatPDFGaussianInf::operator += */
CPose3DQuatPDFGaussianInf mrpt::poses::operator+(
	const CPose3DQuatPDFGaussianInf& x, const CPose3DQuatPDFGaussianInf& u)
{
	CPose3DQuatPDFGaussianInf res(x);
	res += u;
	return res;
}

/** Inverse pose composition for two 3D pose Gaussians  \sa
 * CPose3DQuatPDFGaussianInf::operator -= */
CPose3DQuatPDFGaussianInf mrpt::poses::operator-(
	const CPose3DQuatPDFGaussianInf& x, const CPose3DQuatPDFGaussianInf& u)
{
	CPose3DQuatPDFGaussianInf res(x);
	res -= u;
	return res;
}

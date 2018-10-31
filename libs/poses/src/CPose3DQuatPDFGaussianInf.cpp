/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPose3DQuatPDFGaussianInf.h>
#include <mrpt/math/types_math.h>  // for CMatrixF...
#include <cstdio>  // for size_t
#include <algorithm>  // for move, max
#include <exception>  // for exception
#include <new>  // for operator...
#include <ostream>  // for operator<<
#include <string>  // for allocator
#include <vector>  // for vector
#include <mrpt/math/CMatrixFixedNumeric.h>  // for CMatrixF...
#include <mrpt/math/CQuaternion.h>  // for CQuatern...
#include <mrpt/poses/CPose3D.h>  // for CPose3D
#include <mrpt/poses/CPose3DQuat.h>  // for CPose3DQuat
#include <mrpt/poses/CPose3DQuatPDF.h>  // for CPose3DQ...
#include <mrpt/random/RandomGenerators.h>  // for CRandomG...
#include <mrpt/serialization/CSerializable.h>  // for CSeriali...
#include <mrpt/system/os.h>  // for fopen
#include <mrpt/math/distributions.h>
#include <mrpt/serialization/stl_serialization.h>

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

	for (int r = 0; r < cov_inv.rows(); r++) out << cov_inv.get_unsafe(r, r);
	for (int r = 0; r < cov_inv.rows(); r++)
		for (int c = r + 1; c < cov_inv.cols(); c++)
			out << cov_inv.get_unsafe(r, c);
}
void CPose3DQuatPDFGaussianInf::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> mean;

			for (int r = 0; r < cov_inv.rows(); r++)
				in >> cov_inv.get_unsafe(r, r);
			for (int r = 0; r < cov_inv.rows(); r++)
				for (int c = r + 1; c < cov_inv.cols(); c++)
				{
					double x;
					in >> x;
					cov_inv.get_unsafe(r, c) = cov_inv.get_unsafe(c, r) = x;
				}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPose3DQuatPDFGaussianInf::copyFrom(const CPose3DQuatPDF& o)
{
	if (this == &o) return;  // It may be used sometimes

	// Convert to gaussian pdf:
	CMatrixDouble77 C(UNINITIALIZED_MATRIX);
	o.getCovarianceAndMean(C, this->mean);
	C.inv_fast(this->cov_inv);
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
	CMatrixDouble77 OLD_COV(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(OLD_COV);

	CMatrixDouble77 df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		newReferenceBaseQuat,  // x
		this->mean,  // u
		df_dx, df_du,
		&this->mean  // Output:  newReferenceBaseQuat + this->mean;
	);

	// this->cov = H1*this->cov*~H1 + H2*Ap.cov*~H2;
	// df_dx: not used, since its COV are all zeros... // df_dx.multiply_HCHt(
	// OLD_COV, cov );
	CMatrixDouble77 NEW_COV(UNINITIALIZED_MATRIX);
	df_du.multiply_HCHt(OLD_COV, NEW_COV);
	NEW_COV.inv_fast(this->cov_inv);

	MRPT_END
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussianInf::drawSingleSample(CPose3DQuat& outPart) const
{
	MRPT_START
	CMatrixDouble77 COV(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(COV);

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
	CMatrixDouble77 COV(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(COV);

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
	CMatrixFixedNumeric<double, 3, 7> df_dpose(UNINITIALIZED_MATRIX);
	double lx, ly, lz;
	mean.inverseComposePoint(0, 0, 0, lx, ly, lz, nullptr, &df_dpose);

	CMatrixFixedNumeric<double, 7, 7> jacob;
	jacob.insertMatrix(0, 0, df_dpose);
	jacob.set_unsafe(3, 3, 1);
	jacob.set_unsafe(4, 4, -1);
	jacob.set_unsafe(5, 5, -1);
	jacob.set_unsafe(6, 6, -1);

	// C(0:2,0:2): H C H^t
	CMatrixDouble77 COV(UNINITIALIZED_MATRIX), NEW_COV(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(COV);

	jacob.multiply_HCHt(COV, NEW_COV);
	NEW_COV.inv_fast(out.cov_inv);

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
	CMatrixDouble77 OLD_COV(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(OLD_COV);

	CMatrixDouble77 df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		this->mean,  // x
		Ap,  // u
		df_dx, df_du,
		&this->mean  // Output: this->mean + Ap;
	);

	// this->cov = H1*this->cov*~H1 + H2*Ap.cov*~H2;
	CMatrixDouble77 NEW_COV(UNINITIALIZED_MATRIX);
	df_dx.multiply_HCHt(OLD_COV, NEW_COV);
	NEW_COV.inv_fast(this->cov_inv);
	// df_du: Nothing to do, since COV(Ap) = zeros
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussianInf::operator+=(const CPose3DQuatPDFGaussianInf& Ap)
{
	// COV:
	CMatrixDouble77 OLD_COV(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(OLD_COV);

	CMatrixDouble77 df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		this->mean,  // x
		Ap.mean,  // u
		df_dx, df_du,
		&this->mean  // Output:  this->mean + Ap.mean;
	);

	// this->cov = H1*this->cov*~H1 + H2*Ap.cov*~H2;
	CMatrixDouble77 NEW_COV(UNINITIALIZED_MATRIX);
	CMatrixDouble77 Ap_cov(UNINITIALIZED_MATRIX);
	Ap.cov_inv.inv(Ap_cov);

	df_dx.multiply_HCHt(OLD_COV, NEW_COV);
	df_du.multiply_HCHt(Ap_cov, NEW_COV, true);  // Accumulate result

	NEW_COV.inv_fast(this->cov_inv);
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

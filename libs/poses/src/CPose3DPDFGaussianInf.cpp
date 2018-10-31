/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/random.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;

using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPose3DPDFGaussianInf, CPose3DPDF, mrpt::poses)

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussianInf::CPose3DPDFGaussianInf() : mean(0, 0, 0), cov_inv() {}
/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussianInf::CPose3DPDFGaussianInf(
	TConstructorFlags_Poses constructor_dummy_param)
	: mean(UNINITIALIZED_POSE), cov_inv(UNINITIALIZED_MATRIX)
{
	MRPT_UNUSED_PARAM(constructor_dummy_param);
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussianInf::CPose3DPDFGaussianInf(
	const CPose3D& init_Mean, const CMatrixDouble66& init_Cov)
	: mean(init_Mean), cov_inv(init_Cov)
{
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussianInf::CPose3DPDFGaussianInf(const CPose3D& init_Mean)
	: mean(init_Mean), cov_inv()
{
}

/*---------------------------------------------------------------
					CPose3DPDFGaussianInf
 ---------------------------------------------------------------*/
CPose3DPDFGaussianInf::CPose3DPDFGaussianInf(const CPose3DQuatPDFGaussian& o)
	: mean(UNINITIALIZED_POSE), cov_inv(UNINITIALIZED_MATRIX)
{
	this->copyFrom(o);
}

/*---------------------------------------------------------------
						copyFrom
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::copyFrom(const CPose3DQuatPDFGaussian& o)
{
	const CPose3DPDFGaussian p(o);
	this->copyFrom(p);
}

uint8_t CPose3DPDFGaussianInf::serializeGetVersion() const { return 0; }
void CPose3DPDFGaussianInf::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << mean;
	mrpt::math::serializeSymmetricMatrixTo(cov_inv, out);
}
void CPose3DPDFGaussianInf::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> mean;
			mrpt::math::deserializeSymmetricMatrixFrom(cov_inv, in);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPose3DPDFGaussianInf::copyFrom(const CPose3DPDF& o)
{
	if (this == &o) return;  // It may be used sometimes

	if (IS_CLASS(&o, CPose3DPDFGaussianInf))
	{  // It's my same class:
		const auto* ptr = dynamic_cast<const CPose3DPDFGaussianInf*>(&o);
		ASSERT_(ptr != nullptr);
		mean = ptr->mean;
		cov_inv = ptr->cov_inv;
	}
	else
	{
		// Convert to gaussian pdf:
		CMatrixDouble66 cov(UNINITIALIZED_MATRIX);
		o.getCovarianceAndMean(cov, mean);
		cov.inv_fast(this->cov_inv);
	}
}

void CPose3DPDFGaussianInf::copyFrom(const CPosePDF& o)
{
	if (IS_CLASS(&o, CPosePDFGaussianInf))
	{  // cov is already inverted, but it's a 2D pose:
		const auto* ptr = dynamic_cast<const CPosePDFGaussianInf*>(&o);
		ASSERT_(ptr != nullptr);

		mean = CPose3D(ptr->mean);

		// 3x3 inv_cov -> 6x6 inv_cov
		cov_inv.zeros();
		cov_inv(0, 0) = ptr->cov_inv(0, 0);
		cov_inv(1, 1) = ptr->cov_inv(1, 1);
		cov_inv(3, 3) = ptr->cov_inv(2, 2);

		cov_inv(0, 1) = cov_inv(1, 0) = ptr->cov_inv(0, 1);
		cov_inv(0, 3) = cov_inv(3, 0) = ptr->cov_inv(0, 2);
		cov_inv(1, 3) = cov_inv(3, 1) = ptr->cov_inv(1, 2);
	}
	else
	{
		CPose3DPDFGaussian p(UNINITIALIZED_POSE);
		p.copyFrom(o);
		this->copyFrom(p);
	}
}

/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
bool CPose3DPDFGaussianInf::saveToTextFile(const string& file) const
{
	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;

	os::fprintf(
		f, "%e %e %e %e %e %e\n", mean.x(), mean.y(), mean.z(), mean.yaw(),
		mean.pitch(), mean.roll());

	for (unsigned int i = 0; i < 6; i++)
		os::fprintf(
			f, "%e %e %e %e %e %e\n", cov_inv(i, 0), cov_inv(i, 1),
			cov_inv(i, 2), cov_inv(i, 3), cov_inv(i, 4), cov_inv(i, 5));

	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::changeCoordinatesReference(
	const CPose3D& newReferenceBase)
{
	MRPT_START

	CPose3DPDFGaussian a;
	a.copyFrom(*this);
	a.changeCoordinatesReference(newReferenceBase);
	this->copyFrom(a);

	MRPT_END
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::drawSingleSample(CPose3D& outPart) const
{
	MRPT_UNUSED_PARAM(outPart);
	MRPT_START

	CMatrixDouble66 cov(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(cov);

	CVectorDouble v;
	getRandomGenerator().drawGaussianMultivariate(v, cov);

	outPart.setFromValues(
		mean.x() + v[0], mean.y() + v[1], mean.z() + v[2], mean.yaw() + v[3],
		mean.pitch() + v[4], mean.roll() + v[5]);

	MRPT_END_WITH_CLEAN_UP(cov_inv.saveToTextFile(
		"__DEBUG_EXC_DUMP_drawSingleSample_COV_INV.txt"););
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::drawManySamples(
	size_t N, vector<CVectorDouble>& outSamples) const
{
	MRPT_START

	CMatrixDouble66 cov(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(cov);

	getRandomGenerator().drawGaussianMultivariateMany(outSamples, N, cov);

	for (auto& outSample : outSamples)
	{
		outSample[0] += mean.x();
		outSample[1] += mean.y();
		outSample[2] += mean.z();
		outSample[3] = math::wrapToPi(outSample[3] + mean.yaw());
		outSample[4] = math::wrapToPi(outSample[4] + mean.pitch());
		outSample[5] = math::wrapToPi(outSample[5] + mean.roll());
	}

	MRPT_END
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::bayesianFusion(
	const CPose3DPDF& p1_, const CPose3DPDF& p2_)
{
	MRPT_UNUSED_PARAM(p1_);
	MRPT_UNUSED_PARAM(p2_);

	THROW_EXCEPTION("TO DO!!!");
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::inverse(CPose3DPDF& o) const
{
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPose3DPDFGaussianInf));
	auto& out = dynamic_cast<CPose3DPDFGaussianInf&>(o);

	// This is like: b=(0,0,0)
	//  OUT = b - THIS
	CPose3DPDFGaussianInf b;  // Init: all zeros.
	out = b - *this;
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::operator+=(const CPose3D& Ap)
{
	// COV:
	const CMatrixDouble66 OLD_COV_INV = this->cov_inv;
	CMatrixDouble66 df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DPDF::jacobiansPoseComposition(
		this->mean,  // x
		Ap,  // u
		df_dx, df_du);

	// this->cov = H1*this->cov*~H1 + H2*Ap.cov*~H2;
	// cov_inv   = ... => The same than above!
	df_dx.multiply_HCHt(OLD_COV_INV, cov_inv);
	// df_du: Nothing to do, since COV(Ap) = zeros

	// MEAN:
	this->mean = this->mean + Ap;
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::operator+=(const CPose3DPDFGaussianInf& Ap)
{
	CPose3DPDFGaussian a(UNINITIALIZED_POSE);
	CPose3DPDFGaussian b(UNINITIALIZED_POSE);
	a.copyFrom(*this);
	b.copyFrom(Ap);

	a += b;

	this->mean = a.mean;
	a.cov.inv(this->cov_inv);
}

/*---------------------------------------------------------------
							-=
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::operator-=(const CPose3DPDFGaussianInf& Ap)
{
	CPose3DPDFGaussian a(UNINITIALIZED_POSE);
	CPose3DPDFGaussian b(UNINITIALIZED_POSE);
	a.copyFrom(*this);
	b.copyFrom(Ap);

	a -= b;

	this->mean = a.mean;
	a.cov.inv(this->cov_inv);
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double CPose3DPDFGaussianInf::evaluatePDF(const CPose3D& x) const
{
	MRPT_UNUSED_PARAM(x);
	THROW_EXCEPTION("TO DO!!!");
}

/*---------------------------------------------------------------
						evaluateNormalizedPDF
 ---------------------------------------------------------------*/
double CPose3DPDFGaussianInf::evaluateNormalizedPDF(const CPose3D& x) const
{
	MRPT_UNUSED_PARAM(x);
	THROW_EXCEPTION("TO DO!!!");
}

/*---------------------------------------------------------------
						assureSymmetry
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::assureSymmetry()
{
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	for (int i = 0; i < cov_inv.rows() - 1; i++)
		for (int j = i + 1; j < cov_inv.rows(); j++)
			cov_inv.get_unsafe(i, j) = cov_inv.get_unsafe(j, i);
}

/*---------------------------------------------------------------
						mahalanobisDistanceTo
 ---------------------------------------------------------------*/
double CPose3DPDFGaussianInf::mahalanobisDistanceTo(
	const CPose3DPDFGaussianInf& theOther)
{
	MRPT_START

	const CMatrixDouble66 cov = this->cov_inv.inv();
	const CMatrixDouble66 cov2 = theOther.cov_inv.inv();

	CMatrixDouble66 COV_ = cov + cov2;
	CMatrixDouble61 MU = CMatrixDouble61(theOther.mean) - CMatrixDouble61(mean);

	for (int i = 0; i < 6; i++)
	{
		if (COV_.get_unsafe(i, i) == 0)
		{
			if (MU.get_unsafe(i, 0) != 0)
				return std::numeric_limits<double>::infinity();
			else
				COV_.get_unsafe(i, i) = 1;  // Any arbitrary value since
			// MU(i)=0, and this value doesn't
			// affect the result.
		}
	}

	return std::sqrt(MU.multiply_HtCH_scalar(COV_.inv()));

	MRPT_END
}

/*---------------------------------------------------------------
						operator <<
 ---------------------------------------------------------------*/
ostream& mrpt::poses::operator<<(ostream& out, const CPose3DPDFGaussianInf& obj)
{
	out << "Mean: " << obj.mean << "\n";
	out << "Inverse cov:\n" << obj.cov_inv << "\n";

	return out;
}

/*---------------------------------------------------------------
						getInvCovSubmatrix2D
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::getInvCovSubmatrix2D(CMatrixDouble& out_cov) const
{
	out_cov.setSize(3, 3);

	for (int i = 0; i < 3; i++)
	{
		int a = i == 2 ? 3 : i;
		for (int j = i; j < 3; j++)
		{
			int b = j == 2 ? 3 : j;
			double f = cov_inv.get_unsafe(a, b);
			out_cov.set_unsafe(i, j, f);
			out_cov.set_unsafe(j, i, f);
		}
	}
}

bool mrpt::poses::operator==(
	const CPose3DPDFGaussianInf& p1, const CPose3DPDFGaussianInf& p2)
{
	return p1.mean == p2.mean && p1.cov_inv == p2.cov_inv;
}

CPose3DPDFGaussianInf mrpt::poses::operator+(
	const CPose3DPDFGaussianInf& x, const CPose3DPDFGaussianInf& u)
{
	CPose3DPDFGaussianInf res(x);
	res += u;
	return res;
}

CPose3DPDFGaussianInf mrpt::poses::operator-(
	const CPose3DPDFGaussianInf& x, const CPose3DPDFGaussianInf& u)
{
	CPose3DPDFGaussianInf res(x);
	res -= u;
	return res;
}

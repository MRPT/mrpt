/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/poses/CPoseRandomSampler.h>
#include <mrpt/random.h>
#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::random;

/*---------------------------------------------------------------
		Constructor
  ---------------------------------------------------------------*/
CPoseRandomSampler::CPoseRandomSampler() = default;
CPoseRandomSampler::CPoseRandomSampler(const CPoseRandomSampler& o)
{
	*this = o;
}

CPoseRandomSampler& CPoseRandomSampler::operator=(const CPoseRandomSampler& o)
{
	if (&o == this) return *this;
	m_pdf2D.reset();
	m_pdf3D.reset();
	if (o.m_pdf2D) m_pdf2D.reset(dynamic_cast<CPosePDF*>(o.m_pdf2D->clone()));
	if (o.m_pdf3D) m_pdf3D.reset(dynamic_cast<CPose3DPDF*>(o.m_pdf3D->clone()));
	m_fastdraw_gauss_Z3 = o.m_fastdraw_gauss_Z3;
	m_fastdraw_gauss_Z6 = o.m_fastdraw_gauss_Z6;
	m_fastdraw_gauss_M_2D = o.m_fastdraw_gauss_M_2D;
	m_fastdraw_gauss_M_3D = o.m_fastdraw_gauss_M_3D;
	return *this;
}

CPoseRandomSampler::CPoseRandomSampler(CPoseRandomSampler&& o)
	: m_pdf2D(nullptr), m_pdf3D(nullptr)
{
	if (o.m_pdf2D)
	{
		m_pdf2D = std::move(o.m_pdf2D);
		o.m_pdf2D = nullptr;
	}
	if (o.m_pdf3D)
	{
		m_pdf3D = std::move(o.m_pdf3D);
		o.m_pdf3D = nullptr;
	}
	m_fastdraw_gauss_Z3 = std::move(o.m_fastdraw_gauss_Z3);
	m_fastdraw_gauss_Z6 = std::move(o.m_fastdraw_gauss_Z6);
	m_fastdraw_gauss_M_2D = std::move(o.m_fastdraw_gauss_M_2D);
	m_fastdraw_gauss_M_3D = std::move(o.m_fastdraw_gauss_M_3D);
}
CPoseRandomSampler& CPoseRandomSampler::operator=(CPoseRandomSampler&& o)
{
	if (this == &o) return *this;
	this->clear();
	if (o.m_pdf2D)
	{
		m_pdf2D = std::move(o.m_pdf2D);
		o.m_pdf2D = nullptr;
	}
	if (o.m_pdf3D)
	{
		m_pdf3D = std::move(o.m_pdf3D);
		o.m_pdf3D = nullptr;
	}
	m_fastdraw_gauss_Z3 = std::move(o.m_fastdraw_gauss_Z3);
	m_fastdraw_gauss_Z6 = std::move(o.m_fastdraw_gauss_Z6);
	m_fastdraw_gauss_M_2D = std::move(o.m_fastdraw_gauss_M_2D);
	m_fastdraw_gauss_M_3D = std::move(o.m_fastdraw_gauss_M_3D);
	return *this;
}

/*---------------------------------------------------------------
			clear
  ---------------------------------------------------------------*/
void CPoseRandomSampler::clear()
{
	m_pdf2D.reset();
	m_pdf3D.reset();
}

/*---------------------------------------------------------------
						setPosePDF
  ---------------------------------------------------------------*/
void CPoseRandomSampler::setPosePDF(const CPosePDF& pdf)
{
	MRPT_START

	clear();
	m_pdf2D.reset(dynamic_cast<CPosePDF*>(pdf.clone()));

	// According to the PDF type:
	if (IS_CLASS(pdf, CPosePDFGaussian))
	{
		const auto& gPdf = dynamic_cast<const CPosePDFGaussian&>(pdf);
		const CMatrixDouble33& cov = gPdf.cov;

		m_fastdraw_gauss_M_2D = gPdf.mean;

		std::vector<double> eigVals;
		cov.eig_symmetric(m_fastdraw_gauss_Z3, eigVals);

		// Scale eigenvectors with eigenvalues:
		mrpt::math::CMatrixDouble33 D;
		D.setDiagonal(eigVals);
		D = D.asEigen().array().sqrt().matrix();
		m_fastdraw_gauss_Z3.matProductOf_AB(m_fastdraw_gauss_Z3, D);
	}
	else if (IS_CLASS(pdf, CPosePDFParticles))
	{
		return;  // Nothing to prepare.
	}
	else
	{
		THROW_EXCEPTION_FMT(
			"Unsupported class: %s", m_pdf2D->GetRuntimeClass()->className);
	}

	MRPT_END
}

/*---------------------------------------------------------------
						setPosePDF
  ---------------------------------------------------------------*/
void CPoseRandomSampler::setPosePDF(const CPose3DPDF& pdf)
{
	MRPT_START

	clear();
	m_pdf3D.reset(dynamic_cast<CPose3DPDF*>(pdf.clone()));

	// According to the PDF type:
	if (IS_CLASS(pdf, CPose3DPDFGaussian))
	{
		const auto& gPdf = dynamic_cast<const CPose3DPDFGaussian&>(pdf);
		const CMatrixDouble66& cov = gPdf.cov;

		m_fastdraw_gauss_M_3D = gPdf.mean;

		std::vector<double> eigVals;
		cov.eig_symmetric(m_fastdraw_gauss_Z6, eigVals);

		// Scale eigenvectors with eigenvalues:
		mrpt::math::CMatrixDouble66 D;
		D.setDiagonal(eigVals);

		// Scale eigenvectors with eigenvalues:
		D = D.asEigen().array().sqrt().matrix();
		m_fastdraw_gauss_Z6.matProductOf_AB(m_fastdraw_gauss_Z6, D);
	}
	else if (IS_CLASS(pdf, CPose3DPDFParticles))
	{
		return;  // Nothing to prepare.
	}
	else
	{
		THROW_EXCEPTION_FMT(
			"Unsoported class: %s", m_pdf3D->GetRuntimeClass()->className);
	}

	MRPT_END
}

/*---------------------------------------------------------------
					drawSample
  ---------------------------------------------------------------*/
CPose2D& CPoseRandomSampler::drawSample(CPose2D& p) const
{
	MRPT_START

	if (m_pdf2D)
	{
		do_sample_2D(p);
	}
	else if (m_pdf3D)
	{
		CPose3D q;
		do_sample_3D(q);
		p.x(q.x());
		p.y(q.y());
		p.phi(q.yaw());
	}
	else
		THROW_EXCEPTION("No associated pdf: setPosePDF must be called first.");

	return p;
	MRPT_END
}

/*---------------------------------------------------------------
					drawSample
  ---------------------------------------------------------------*/
CPose3D& CPoseRandomSampler::drawSample(CPose3D& p) const
{
	MRPT_START

	if (m_pdf2D)
	{
		CPose2D q;
		do_sample_2D(q);
		p.setFromValues(q.x(), q.y(), 0, q.phi(), 0, 0);
	}
	else if (m_pdf3D)
	{
		do_sample_3D(p);
	}
	else
		THROW_EXCEPTION("No associated pdf: setPosePDF must be called first.");

	return p;
	MRPT_END
}

/*---------------------------------------------------------------
				  do_sample_2D: Sample from a 2D PDF
  ---------------------------------------------------------------*/
void CPoseRandomSampler::do_sample_2D(CPose2D& p) const
{
	MRPT_START
	ASSERT_(m_pdf2D);

	// According to the PDF type:
	if (IS_CLASS(*m_pdf2D, CPosePDFGaussian))
	{
		// ------------------------------
		//      A single gaussian:
		// ------------------------------
		CVectorDouble rndVector(3);
		rndVector.setZero();
		for (size_t i = 0; i < 3; i++)
		{
			double rnd = getRandomGenerator().drawGaussian1D_normalized();
			for (size_t d = 0; d < 3; d++)
				rndVector[d] += (m_fastdraw_gauss_Z3(d, i) * rnd);
		}

		p.x(m_fastdraw_gauss_M_2D.x() + rndVector[0]);
		p.y(m_fastdraw_gauss_M_2D.y() + rndVector[1]);
		p.phi(m_fastdraw_gauss_M_2D.phi() + rndVector[2]);
		p.normalizePhi();
	}
	else if (IS_CLASS(*m_pdf2D, CPosePDFSOG))
	{
		// -------------------------------------
		//      			SOG
		// -------------------------------------
		THROW_EXCEPTION("TODO");
	}
	else if (IS_CLASS(*m_pdf2D, CPosePDFParticles))
	{
		// -------------------------------------
		//      Particles: just sample as usual
		// -------------------------------------
		const auto& pdf = dynamic_cast<const CPosePDFParticles&>(*m_pdf2D);
		pdf.drawSingleSample(p);
	}
	else
		THROW_EXCEPTION_FMT(
			"Unsoported class: %s", m_pdf2D->GetRuntimeClass()->className);

	MRPT_END
}

/*---------------------------------------------------------------
				  do_sample_3D: Sample from a 3D PDF
  ---------------------------------------------------------------*/
void CPoseRandomSampler::do_sample_3D(CPose3D& p) const
{
	MRPT_START
	ASSERT_(m_pdf3D);

	// According to the PDF type:
	if (IS_CLASS(*m_pdf3D, CPose3DPDFGaussian))
	{
		// ------------------------------
		//      A single gaussian:
		// ------------------------------
		CVectorDouble rndVector(6);
		rndVector.setZero();
		for (size_t i = 0; i < 6; i++)
		{
			double rnd = getRandomGenerator().drawGaussian1D_normalized();
			for (size_t d = 0; d < 6; d++)
				rndVector[d] += (m_fastdraw_gauss_Z6(d, i) * rnd);
		}

		p.setFromValues(
			m_fastdraw_gauss_M_3D.x() + rndVector[0],
			m_fastdraw_gauss_M_3D.y() + rndVector[1],
			m_fastdraw_gauss_M_3D.z() + rndVector[2],
			m_fastdraw_gauss_M_3D.yaw() + rndVector[3],
			m_fastdraw_gauss_M_3D.pitch() + rndVector[4],
			m_fastdraw_gauss_M_3D.roll() + rndVector[5]);
	}
	else if (IS_CLASS(*m_pdf3D, CPose3DPDFSOG))
	{
		// -------------------------------------
		//      			SOG
		// -------------------------------------
		THROW_EXCEPTION("TODO");
	}
	else if (IS_CLASS(*m_pdf3D, CPose3DPDFParticles))
	{
		// -------------------------------------
		//      Particles: just sample as usual
		// -------------------------------------
		const auto& pdf = dynamic_cast<const CPose3DPDFParticles&>(*m_pdf3D);
		pdf.drawSingleSample(p);
	}
	else
		THROW_EXCEPTION_FMT(
			"Unsoported class: %s", m_pdf3D->GetRuntimeClass()->className);

	MRPT_END
}

/*---------------------------------------------------------------
				  isPrepared
  ---------------------------------------------------------------*/
bool CPoseRandomSampler::isPrepared() const { return m_pdf2D || m_pdf3D; }
/*---------------------------------------------------------------
				  getOriginalPDFCov2D
  ---------------------------------------------------------------*/
void CPoseRandomSampler::getOriginalPDFCov2D(CMatrixDouble33& cov3x3) const
{
	MRPT_START
	ASSERT_(this->isPrepared());

	if (m_pdf2D)
	{
		m_pdf2D->getCovariance(cov3x3);
	}
	else
	{
		ASSERT_(m_pdf3D);

		CPosePDFGaussian P;
		P.copyFrom(*m_pdf3D);
		cov3x3 = P.cov;
	}

	MRPT_END
}

/*---------------------------------------------------------------
				  getOriginalPDFCov3D
  ---------------------------------------------------------------*/
void CPoseRandomSampler::getOriginalPDFCov3D(CMatrixDouble66& cov6x6) const
{
	MRPT_START
	ASSERT_(this->isPrepared());

	if (m_pdf2D)
	{
		CPose3DPDFGaussian P;
		P.copyFrom(*m_pdf2D);
		cov6x6 = P.cov;
	}
	else
	{
		ASSERT_(m_pdf3D);
		m_pdf3D->getCovariance(cov6x6);
	}

	MRPT_END
}

/*---------------------------------------------------------------
				  getSamplingMean2D
  ---------------------------------------------------------------*/
CPose2D& CPoseRandomSampler::getSamplingMean2D(CPose2D& out_mean) const
{
	MRPT_START
	ASSERT_(this->isPrepared());

	if (m_pdf2D)
		out_mean = m_fastdraw_gauss_M_2D;
	else
		out_mean = CPose2D(m_fastdraw_gauss_M_3D);

	return out_mean;
	MRPT_END
}

/*---------------------------------------------------------------
				  getSamplingMean3D
  ---------------------------------------------------------------*/
CPose3D& CPoseRandomSampler::getSamplingMean3D(CPose3D& out_mean) const
{
	MRPT_START
	ASSERT_(this->isPrepared());

	if (m_pdf3D)
		out_mean = m_fastdraw_gauss_M_3D;
	else
		out_mean = CPose3D(m_fastdraw_gauss_M_2D);

	return out_mean;
	MRPT_END
}

void CPoseRandomSampler::getOriginalPDFCov2D(
	mrpt::math::CMatrixDouble& cov3x3) const
{
	mrpt::math::CMatrixDouble33 M;
	this->getOriginalPDFCov2D(M);
	cov3x3 = mrpt::math::CMatrixDouble(M);
}

void CPoseRandomSampler::getOriginalPDFCov3D(
	mrpt::math::CMatrixDouble& cov6x6) const
{
	mrpt::math::CMatrixDouble66 M;
	this->getOriginalPDFCov3D(M);
	cov6x6 = mrpt::math::CMatrixDouble(M);
}

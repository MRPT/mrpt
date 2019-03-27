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
#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPose3DPDFSOG, CPose3DPDF, mrpt::poses)

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFSOG::CPose3DPDFSOG(size_t nModes) : m_modes(nModes) {}
/*---------------------------------------------------------------
			clear
  ---------------------------------------------------------------*/
void CPose3DPDFSOG::clear() { m_modes.clear(); }
/*---------------------------------------------------------------
	Resize
  ---------------------------------------------------------------*/
void CPose3DPDFSOG::resize(const size_t N) { m_modes.resize(N); }
/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the
 PDF)
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::getMean(CPose3D& p) const
{
	if (!m_modes.empty())
	{
		// Calc average on SE(3)
		mrpt::poses::SE_average<3> se_averager;
		for (const auto& m : m_modes)
		{
			const double w = exp(m.log_w);
			se_averager.append(m.val.mean, w);
		}
		se_averager.get_average(p);
	}
	else
	{
		p.setFromValues(0, 0, 0, 0, 0, 0);
	}
}

std::tuple<mrpt::math::CMatrixDouble66, CPose3D>
	CPose3DPDFSOG::getCovarianceAndMean() const
{
	const size_t N = m_modes.size();

	// Get mean:
	CPose3D mean;
	getMean(mean);

	// Get cov:
	mrpt::math::CMatrixDouble66 estCov;
	if (N)
	{
		double sumW = 0;

		mrpt::math::CMatrixDouble66 MMt;
		mrpt::math::CMatrixDouble61 estMean_i;
		for (const auto& m : m_modes)
		{
			double w;
			sumW += w = exp(m.log_w);
			estMean_i = mrpt::math::CMatrixDouble61(m.val.mean);
			MMt.matProductOf_AAt(estMean_i);
			MMt += m.val.cov;
			MMt *= w;
			estCov += MMt;  // w * ( (it)->val.cov +
			// ((estMean_i-estMean)*(~(estMean_i-estMean))) );
		}

		if (sumW != 0) estCov *= (1.0 / sumW);
	}

	return {estCov, mean};
}

uint8_t CPose3DPDFSOG::serializeGetVersion() const { return 2; }
void CPose3DPDFSOG::serializeTo(mrpt::serialization::CArchive& out) const
{
	uint32_t N = m_modes.size();
	out << N;
	for (const auto& m : m_modes) out << m.log_w << m.val.mean << m.val.cov;
}
void CPose3DPDFSOG::serializeFrom(
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
			this->resize(N);

			for (auto& m : m_modes)
			{
				in >> m.log_w;

				// In version 0, weights were linear!!
				if (version == 0) m.log_w = log(std::max(1e-300, m.log_w));

				in >> m.val.mean;

				if (version == 1)  // were floats
				{
					THROW_EXCEPTION("Unsupported serialized version: too old");
				}
				else
					in >> m.val.cov;
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CPose3DPDFSOG::copyFrom(const CPose3DPDF& o)
{
	MRPT_START

	if (this == &o) return;  // It may be used sometimes

	if (o.GetRuntimeClass() == CLASS_ID(CPose3DPDFSOG))
	{
		*this = dynamic_cast<const CPose3DPDFSOG&>(o);
	}
	else
	{
		this->resize(1);
		m_modes[0].log_w = 0;
		mrpt::math::CMatrixDouble66 C;
		o.getCovarianceAndMean(C, m_modes[0].val.mean);
		m_modes[0].val.cov = C;
	}

	MRPT_END
}

/*---------------------------------------------------------------
						saveToTextFile
  ---------------------------------------------------------------*/
bool CPose3DPDFSOG::saveToTextFile(const std::string& file) const
{
	FILE* f = os::fopen(file.c_str(), "wt");
	if (!f) return false;

	for (const auto& m : m_modes)
		os::fprintf(
			f, "%e %e %e %e %e %e %e %e %e %e\n", exp(m.log_w), m.val.mean.x(),
			m.val.mean.y(), m.val.mean.z(), m.val.cov(0, 0), m.val.cov(1, 1),
			m.val.cov(2, 2), m.val.cov(0, 1), m.val.cov(0, 2), m.val.cov(1, 2));
	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::changeCoordinatesReference(const CPose3D& newReferenceBase)
{
	for (auto& m : m_modes) m.val.changeCoordinatesReference(newReferenceBase);
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::bayesianFusion(const CPose3DPDF& p1_, const CPose3DPDF& p2_)
{
	MRPT_START

	// p1: CPose3DPDFSOG, p2: CPosePDFGaussian:

	ASSERT_(p1_.GetRuntimeClass() == CLASS_ID(CPose3DPDFSOG));
	ASSERT_(p2_.GetRuntimeClass() == CLASS_ID(CPose3DPDFSOG));

	THROW_EXCEPTION("TODO!!!");

	MRPT_END
}

/*---------------------------------------------------------------
						enforceCovSymmetry
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::enforceCovSymmetry()
{
	MRPT_START
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	for (auto& m : m_modes)
	{
		for (size_t i = 0; i < 6; i++)
			for (size_t j = i + 1; j < 6; j++)
				m.val.cov(i, j) = m.val.cov(j, i);
	}

	MRPT_END
}

/*---------------------------------------------------------------
						normalizeWeights
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::normalizeWeights()
{
	MRPT_START
	if (m_modes.empty()) return;
	double maxW = m_modes[0].log_w;
	for (auto& m : m_modes) maxW = max(maxW, m.log_w);
	for (auto& m : m_modes) m.log_w -= maxW;
	MRPT_END
}

/*---------------------------------------------------------------
						drawSingleSample
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::drawSingleSample(CPose3D& outPart) const
{
	MRPT_UNUSED_PARAM(outPart);
	THROW_EXCEPTION("TO DO!");
}

/*---------------------------------------------------------------
						drawManySamples
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::drawManySamples(
	size_t N, std::vector<CVectorDouble>& outSamples) const
{
	MRPT_UNUSED_PARAM(N);
	MRPT_UNUSED_PARAM(outSamples);
	THROW_EXCEPTION("TO DO!");
}

/*---------------------------------------------------------------
						inverse
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::inverse(CPose3DPDF& o) const
{
	MRPT_START
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPose3DPDFSOG));
	auto* out = dynamic_cast<CPose3DPDFSOG*>(&o);
	ASSERT_(out != nullptr);

	// Prepare the output SOG:
	out->resize(m_modes.size());

	const_iterator it;
	iterator outIt;

	for (it = m_modes.begin(), outIt = out->m_modes.begin();
		 it != m_modes.end(); it++, outIt++)
	{
		(it)->val.inverse((outIt)->val);
		(outIt)->log_w = (it)->log_w;
	}

	MRPT_END
}

/*---------------------------------------------------------------
						appendFrom
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::appendFrom(const CPose3DPDFSOG& o)
{
	MRPT_START

	ASSERT_(&o != this);  // Don't be bad...
	if (o.m_modes.empty()) return;

	// Make copies:
	for (const auto& m_mode : o.m_modes) m_modes.push_back(m_mode);

	normalizeWeights();
	MRPT_END
}

/*---------------------------------------------------------------
						getMostLikelyMode
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::getMostLikelyMode(CPose3DPDFGaussian& outVal) const
{
	if (this->empty())
	{
		outVal = CPose3DPDFGaussian();
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

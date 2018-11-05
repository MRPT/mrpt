/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/system/os.h>
#include <mrpt/random.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/distributions.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/poses/SO_SE_average.h>

using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPosePDFParticles, CPosePDF, mrpt::poses)

CPosePDFParticles::CPosePDFParticles(size_t M)
{
	m_particles.resize(M);
	for (auto& p : m_particles)
	{
		p.log_w = .0;
		p.d = TPose2D();
	}
	TPose2D nullPose(0, 0, 0);
	resetDeterministic(nullPose);
}

void CPosePDFParticles::copyFrom(const CPosePDF& o)
{
	MRPT_START

	CParticleList::iterator itDest;
	CParticleList::const_iterator itSrc;

	if (this == &o) return;  // It may be used sometimes

	if (o.GetRuntimeClass() == CLASS_ID(CPosePDFParticles))
	{
		const auto* pdf = dynamic_cast<const CPosePDFParticles*>(&o);
		ASSERT_(pdf);

		// Both are m_particles:
		m_particles = pdf->m_particles;
	}
	else if (o.GetRuntimeClass() == CLASS_ID(CPosePDFGaussian))
	{
		const auto* pdf = dynamic_cast<const CPosePDFGaussian*>(&o);
		size_t M = m_particles.size();
		std::vector<CVectorDouble> parts;
		std::vector<CVectorDouble>::iterator partsIt;

		getRandomGenerator().drawGaussianMultivariateMany(parts, M, pdf->cov);

		clearParticles();
		m_particles.resize(M);

		for (itDest = m_particles.begin(), partsIt = parts.begin();
			 itDest != m_particles.end(); ++itDest, ++partsIt)
		{
			itDest->log_w = 0;
			itDest->d = TPose2D(
				pdf->mean.x() + (*partsIt)[0], (pdf->mean.y() + (*partsIt)[1]),
				(pdf->mean.phi() + (*partsIt)[2]));
			itDest->d.normalizePhi();
		}
	}

	MRPT_END
}

void CPosePDFParticles::clear() { clearParticles(); }
void CPosePDFParticles::getMean(CPose2D& est_) const
{
	// Calc average on SE(2)
	const size_t n = m_particles.size();
	if (n)
	{
		mrpt::poses::SE_average<2> se_averager;
		for (size_t i = 0; i < n; i++)
		{
			double w = exp(m_particles[i].log_w);
			se_averager.append(m_particles[i].d, w);
		}
		se_averager.get_average(est_);
	}
	else
	{
		est_ = CPose2D();
	}
}

void CPosePDFParticles::getCovarianceAndMean(
	CMatrixDouble33& cov, CPose2D& mean) const
{
	cov.zeros();
	getMean(mean);

	size_t i, n = m_particles.size();
	double var_x = 0, var_y = 0, var_p = 0, var_xy = 0, var_xp = 0, var_yp = 0;
	double mean_phi = mean.phi();

	if (mean_phi < 0) mean_phi = M_2PI + mean_phi;

	double lin_w_sum = 0;

	for (i = 0; i < n; i++) lin_w_sum += exp(m_particles[i].log_w);
	if (lin_w_sum == 0) lin_w_sum = 1;

	for (i = 0; i < n; i++)
	{
		double w = exp(m_particles[i].log_w) / lin_w_sum;

		// Manage 1 PI range:
		double err_x = m_particles[i].d.x - mean.x();
		double err_y = m_particles[i].d.y - mean.y();
		double err_phi = math::wrapToPi(fabs(m_particles[i].d.phi - mean_phi));

		var_x += square(err_x) * w;
		var_y += square(err_y) * w;
		var_p += square(err_phi) * w;
		var_xy += err_x * err_y * w;
		var_xp += err_x * err_phi * w;
		var_yp += err_y * err_phi * w;
	}

	if (n < 2)
	{
		// Not enought information to estimate the variance:
	}
	else
	{
		// Unbiased estimation of variance:
		cov(0, 0) = var_x;
		cov(1, 1) = var_y;
		cov(2, 2) = var_p;

		cov(1, 0) = cov(0, 1) = var_xy;
		cov(2, 0) = cov(0, 2) = var_xp;
		cov(1, 2) = cov(2, 1) = var_yp;
	}
}

uint8_t CPosePDFParticles::serializeGetVersion() const { return 1; }
void CPosePDFParticles::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeParticlesToStream(out);  // v1: changed CPose2D -> TPose2D
}
void CPosePDFParticles::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			mrpt::bayes::CParticleFilterData<
				mrpt::poses::CPose2D, PARTICLE_STORAGE>
				old;
			old.readParticlesFromStream(in);
			m_particles.clear();
			std::transform(
				old.m_particles.begin(), old.m_particles.end(),
				std::back_inserter(m_particles),
				[](const auto& p) -> CParticleData {
					return CParticleData(p.d.asTPose(), p.log_w);
				});
		}
		break;
		case 1:
		{
			readParticlesFromStream(in);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPosePDFParticles::resetDeterministic(
	const TPose2D& location, size_t particlesCount)
{
	if (particlesCount > 0) m_particles.resize(particlesCount);

	for (auto& p : m_particles)
	{
		p.d = location;
		p.log_w = .0;
	}
}

void CPosePDFParticles::resetUniform(
	const double x_min, const double x_max, const double y_min,
	const double y_max, const double phi_min, const double phi_max,
	const int particlesCount)
{
	MRPT_START
	if (particlesCount > 0) m_particles.resize(particlesCount);

	for (auto& p : m_particles)
	{
		p.d.x = getRandomGenerator().drawUniform(x_min, x_max);
		p.d.y = getRandomGenerator().drawUniform(y_min, y_max);
		p.d.phi = getRandomGenerator().drawUniform(phi_min, phi_max);
		p.log_w = 0;
	}
	MRPT_END
}

void CPosePDFParticles::resetAroundSetOfPoses(
	const std::vector<mrpt::math::TPose2D>& list_poses,
	const size_t num_particles_per_pose, const double spread_x,
	const double spread_y, const double spread_phi_rad)
{
	MRPT_START
	ASSERT_(!list_poses.empty());
	ASSERT_(num_particles_per_pose >= 1);

	const size_t N = list_poses.size() * num_particles_per_pose;

	clear();
	m_particles.resize(N);
	size_t i, nSpot;
	for (i = 0, nSpot = 0; nSpot < list_poses.size(); nSpot++)
	{
		const mrpt::math::TPose2D& p = list_poses[nSpot];
		for (size_t k = 0; k < num_particles_per_pose; k++, i++)
		{
			m_particles[i].d.x = getRandomGenerator().drawUniform(
				p.x - spread_x * 0.5, p.x + spread_x * 0.5);
			m_particles[i].d.y = getRandomGenerator().drawUniform(
				p.y - spread_y * 0.5, p.y + spread_y * 0.5);
			m_particles[i].d.phi = getRandomGenerator().drawUniform(
				p.phi - spread_phi_rad * 0.5, p.phi + spread_phi_rad * 0.5);
			m_particles[i].log_w = 0;
		}
	}
	ASSERT_EQUAL_(i, N);
	MRPT_END
}

bool CPosePDFParticles::saveToTextFile(const std::string& file) const
{
	std::string buf;
	buf += mrpt::format("%% x  y  yaw[rad] log_weight\n");

	for (const auto& p : m_particles)
		buf += mrpt::format("%f %f %f %e\n", p.d.x, p.d.y, p.d.phi, p.log_w);

	std::ofstream f(file);
	if (!f.is_open()) return false;
	f << buf;
	return true;
}

TPose2D CPosePDFParticles::getParticlePose(size_t i) const
{
	return m_particles[i].d;
}

void CPosePDFParticles::changeCoordinatesReference(
	const CPose3D& newReferenceBase_)
{
	const TPose2D newReferenceBase = CPose2D(newReferenceBase_).asTPose();
	for (auto& p : m_particles) p.d = newReferenceBase + p.d;
}

void CPosePDFParticles::drawSingleSample(CPose2D& outPart) const
{
	const double uni = getRandomGenerator().drawUniform(0.0, 0.9999);
	double cum = 0;

	for (auto& p : m_particles)
	{
		cum += exp(p.log_w);
		if (uni <= cum)
		{
			outPart = CPose2D(p.d);
			return;
		}
	}

	// Might not come here normally:
	outPart = CPose2D(m_particles.rbegin()->d);
}

void CPosePDFParticles::operator+=(const TPose2D& Ap)
{
	for (auto& p : m_particles) p.d = p.d + Ap;
}

void CPosePDFParticles::append(CPosePDFParticles& o)
{
	for (auto& p : o.m_particles) m_particles.emplace_back(p);
	normalizeWeights();
}

void CPosePDFParticles::inverse(CPosePDF& o) const
{
	MRPT_START
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPosePDFParticles));
	auto* out = dynamic_cast<CPosePDFParticles*>(&o);

	out->copyFrom(*this);
	TPose2D nullPose(0, 0, 0);

	for (auto& p : out->m_particles) p.d = nullPose - p.d;

	MRPT_END
}

mrpt::math::TPose2D CPosePDFParticles::getMostLikelyParticle() const
{
	mrpt::math::TPose2D ret{0, 0, 0};
	double max_w = -std::numeric_limits<double>::max();
	for (const auto& p : m_particles)
	{
		if (p.log_w > max_w)
		{
			ret = p.d;
			max_w = p.log_w;
		}
	}
	return ret;
}

void CPosePDFParticles::bayesianFusion(
	const CPosePDF& p1, const CPosePDF& p2,
	const double minMahalanobisDistToDrop)
{
	MRPT_UNUSED_PARAM(p1);
	MRPT_UNUSED_PARAM(p2);
	MRPT_UNUSED_PARAM(minMahalanobisDistToDrop);

	THROW_EXCEPTION("Not implemented yet!");
}

double CPosePDFParticles::evaluatePDF_parzen(
	const double x, const double y, const double phi, const double stdXY,
	const double stdPhi) const
{
	double ret = 0;
	for (const auto& p : m_particles)
	{
		double difPhi = math::wrapToPi(phi - p.d.phi);
		ret += exp(p.log_w) *
			   math::normalPDF(
				   std::sqrt(square(p.d.x - x) + square(p.d.y - y)), 0, stdXY) *
			   math::normalPDF(std::abs(difPhi), 0, stdPhi);
	}
	return ret;
}

void CPosePDFParticles::saveParzenPDFToTextFile(
	const char* fileName, const double x_min, const double x_max,
	const double y_min, const double y_max, const double phi,
	const double stepSizeXY, const double stdXY, const double stdPhi) const
{
	std::string buf;

	for (double y = y_min; y < y_max; y += stepSizeXY)
		for (double x = x_min; x < x_max; x += stepSizeXY)
			buf += mrpt::format(
				"%f ", evaluatePDF_parzen(x, y, phi, stdXY, stdPhi));
	buf += "\n";

	std::ofstream f(fileName);
	if (!f.is_open()) return;
	f << buf;
}

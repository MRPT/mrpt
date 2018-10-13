/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>

namespace mrpt::poses
{
/** Declares a class that represents a Probability Density function (PDF) of a
 * 3D pose
 *
 *  This class is also the base for the implementation of Monte-Carlo
 * Localization (MCL), in mrpt::slam::CMonteCarloLocalization2D.
 *
 *  See the application "app/pf-localization" for an example of usage.
 *
 * \sa CPose3D, CPose3DPDF, CPoseGaussianPDF
 * \ingroup poses_pdf_grp
 */
class CPose3DPDFParticles
	: public CPose3DPDF,
	  public mrpt::bayes::CParticleFilterData<
		  mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>,
	  public mrpt::bayes::CParticleFilterDataImpl<
		  CPose3DPDFParticles,
		  mrpt::bayes::CParticleFilterData<
			  mrpt::math::TPose3D,
			  mrpt::bayes::particle_storage_mode::VALUE>::CParticleList>
{
	DEFINE_SERIALIZABLE(CPose3DPDFParticles)

   public:
	/** Constructor
	 * \param M The number of m_particles.
	 */
	CPose3DPDFParticles(size_t M = 1);

	/** Copy operator, translating if necesary (for example, between m_particles
	 * and gaussian representations) */
	void copyFrom(const CPose3DPDF& o) override;

	/** Reset the PDF to a single point: All m_particles will be set exactly to
	 * the supplied pose.
	 * \param location The location to set all the m_particles.
	 * \param particlesCount If this is set to 0 the number of m_particles
	 * remains unchanged.
	 *  \sa resetUniform, resetUniformFreeSpace */
	void resetDeterministic(
		const mrpt::math::TPose3D& location, size_t particlesCount = 0);

	/** Returns an estimate of the pose, (the mean, or mathematical expectation
	 * of the PDF), computed as a weighted average over all m_particles. \sa
	 * getCovariance */
	void getMean(CPose3D& mean_pose) const override;
	/** Returns an estimate of the pose covariance matrix (6x6 cov matrix) and
	 * the mean, both at once. \sa getMean */
	void getCovarianceAndMean(
		mrpt::math::CMatrixDouble66& cov, CPose3D& mean_point) const override;

	/** Returns the pose of the i'th particle */
	mrpt::math::TPose3D getParticlePose(int i) const;

	/** Save PDF's m_particles to a text file. In each line it will go: "x y z"
	 */
	bool saveToTextFile(const std::string& file) const override;

	/** Get the m_particles count (equivalent to "particlesCount") */
	size_t size() const { return m_particles.size(); }
	/** this = p (+) this. This can be used to convert a PDF from local
	 * coordinates to global, providing the point (newReferenceBase) from which
	 *   "to project" the current pdf. Result PDF substituted the currently
	 * stored one in the object. */
	void changeCoordinatesReference(const CPose3D& newReferenceBase) override;
	/** Draws a single sample from the distribution (WARNING: weights are
	 * assumed to be normalized!) */
	void drawSingleSample(CPose3D& outPart) const override;
	/** Draws a number of samples from the distribution, and saves as a list of
	 * 1x6 vectors, where each row contains a (x,y,phi) datum. */
	void drawManySamples(
		size_t N,
		std::vector<mrpt::math::CVectorDouble>& outSamples) const override;
	/** Appends (pose-composition) a given pose "p" to each particle */
	void operator+=(const CPose3D& Ap);
	/** Appends (add to the list) a set of m_particles to the existing ones, and
	 * then normalize weights. */
	void append(CPose3DPDFParticles& o);
	/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF */
	void inverse(CPose3DPDF& o) const override;
	/** Returns the particle with the highest weight. */
	mrpt::math::TPose3D getMostLikelyParticle() const;
	/** Bayesian fusion */
	void bayesianFusion(const CPose3DPDF& p1, const CPose3DPDF& p2) override;

};  // End of class def.
}  // namespace mrpt::poses

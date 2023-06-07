/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/core/Stringifyable.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPoseRandomSampler.h>

namespace mrpt::poses
{
using PFDataTPose2D = mrpt::bayes::CParticleFilterData<
	mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>;

/** Declares a class that represents a Probability Density Function (PDF) over a
 * 2D pose (x,y,phi), using a set of weighted samples.
 *
 *  This class is also the base for the implementation of Monte-Carlo
 * Localization (MCL), in mrpt::slam::CMonteCarloLocalization2D.
 *
 *  See the application "app/pf-localization" for an example of usage.
 *
 * \sa CPose2D, CPosePDF, CPoseGaussianPDF, CParticleFilterCapable
 * \ingroup poses_pdf_grp
 */
class CPosePDFParticles : public CPosePDF,
						  public PFDataTPose2D,
						  public mrpt::bayes::CParticleFilterDataImpl<
							  CPosePDFParticles, PFDataTPose2D::CParticleList>,
						  public mrpt::Stringifyable
{
	DEFINE_SERIALIZABLE(CPosePDFParticles, mrpt::poses)

   public:
	/** Free all the memory associated to m_particles, and set the number of
	 * parts = 0 */
	void clear();

	/** Constructor
	 * \param M The number of m_particles.
	 */
	CPosePDFParticles(size_t M = 1);

	/** Copy operator, translating if necesary (for example, between m_particles
	 * and gaussian representations)
	 */
	void copyFrom(const CPosePDF& o) override;

	/** Reset the PDF to a single point: All m_particles will be set exactly to
	 * the supplied pose.
	 * \param location The location to set all the m_particles.
	 * \param particlesCount If this is set to 0 the number of m_particles
	 * remains unchanged.
	 *  \sa resetUniform, CMonteCarloLocalization2D::resetUniformFreeSpace,
	 * resetAroundSetOfPoses
	 */
	void resetDeterministic(
		const mrpt::math::TPose2D& location, size_t particlesCount = 0);

	/** Reset the PDF to an uniformly distributed one, inside of the defined
	 * 2D area `[x_min,x_max]x[y_min,y_max]` (in meters) and for
	 * orientations `[phi_min, phi_max]` (in radians).
	 *
	 * \param particlesCount New particle count, or leave count unchanged if set
	 * to -1 (default).
	 *
	 * \note Orientations can be outside of the [-pi,pi] range if so desired,
	 *       but it must hold `phi_max>=phi_min`.
	 * \sa resetDeterministic, CMonteCarloLocalization2D::resetUniformFreeSpace,
	 * resetAroundSetOfPoses
	 */
	void resetUniform(
		const double x_min, const double x_max, const double y_min,
		const double y_max, const double phi_min = -M_PI,
		const double phi_max = M_PI, const int particlesCount = -1);

	/** Reset the PDF to a multimodal distribution over a set of "spots"
	 * (x,y,phi)
	 * The total number of particles will be `list_poses.size() *
	 * num_particles_per_pose`.
	 * \param[in] list_poses The poses (x,y,phi) around which particles will be
	 * spread. Must contains at least one pose.
	 * \param[in] num_particles_per_pose Number of particles to be spread
	 * around each of the "spots" in list_poses. Must be >=1.
	 *
	 * Particles will be spread uniformly in a box of width
	 * `spread_{x,y,phi_rad}` in each of
	 * the three coordinates (meters, radians), so it can be understood as the
	 * "initial uncertainty".
	 *
	 *  \sa resetDeterministic, CMonteCarloLocalization2D::resetUniformFreeSpace
	 */
	void resetAroundSetOfPoses(
		const std::vector<mrpt::math::TPose2D>& list_poses,
		size_t num_particles_per_pose, const double spread_x,
		const double spread_y, const double spread_phi_rad);

	void getMean(CPose2D& mean_pose) const override;

	std::tuple<cov_mat_t, type_value> getCovarianceAndMean() const override;

	/** Returns the pose of the i'th particle.
	 */
	mrpt::math::TPose2D getParticlePose(size_t i) const;

	/** Save PDF's m_particles to a text file. In each line it will go: "x y phi
	 * weight"
	 */
	bool saveToTextFile(const std::string& file) const override;

	/** Get the m_particles count (equivalent to "particlesCount")
	 */
	inline size_t size() const { return m_particles.size(); }
	/** this = p (+) this. This can be used to convert a PDF from local
	 * coordinates to global, providing the point (newReferenceBase) from which
	 *   "to project" the current pdf. Result PDF substituted the currently
	 * stored one in the object.
	 */
	void changeCoordinatesReference(const CPose3D& newReferenceBase) override;

	/** Draws a single sample from the distribution (WARNING: weights are
	 * assumed to be normalized!)
	 */
	void drawSingleSample(CPose2D& outPart) const override;

	/** Appends (pose-composition) a given pose "p" to each particle
	 */
	void operator+=(const mrpt::math::TPose2D& Ap);

	/** Appends (add to the list) a set of m_particles to the existing ones, and
	 * then normalize weights.
	 */
	void append(CPosePDFParticles& o);

	/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
	 */
	void inverse(CPosePDF& o) const override;

	/** Returns the particle with the highest weight.
	 */
	mrpt::math::TPose2D getMostLikelyParticle() const;

	/** Bayesian fusion.
	 */
	void bayesianFusion(
		const CPosePDF& p1, const CPosePDF& p2,
		const double minMahalanobisDistToDrop = 0) override;

	/** Evaluates the PDF at a given arbitrary point as reconstructed by a
	 * Parzen window.
	 * \sa saveParzenPDFToTextFile
	 */
	double evaluatePDF_parzen(
		const double x, const double y, const double phi, const double stdXY,
		const double stdPhi) const;

	/** Save a text file (compatible with matlab) representing the 2D evaluation
	 * of the PDF as reconstructed by a Parzen window.
	 * \sa evaluatePDF_parzen
	 */
	void saveParzenPDFToTextFile(
		const char* fileName, const double x_min, const double x_max,
		const double y_min, const double y_max, const double phi,
		const double stepSizeXY, const double stdXY, const double stdPhi) const;

	std::string asString() const override;

};	// End of class def.

}  // namespace mrpt::poses

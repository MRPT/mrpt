/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPointPDF.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>

namespace mrpt::poses
{
/** A probability distribution of a 2D/3D point, represented as a set of random
 * samples (particles).
 * \sa CPointPDF
 * \ingroup poses_pdf_grp
 */
class CPointPDFParticles
	: public CPointPDF,
	  public mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3Df>,
	  public mrpt::bayes::CParticleFilterDataImpl<
		  CPointPDFParticles, mrpt::bayes::CParticleFilterData<
								  mrpt::math::TPoint3Df>::CParticleList>
{
	DEFINE_SERIALIZABLE(CPointPDFParticles)

   public:
	/** Default constructor */
	CPointPDFParticles(size_t numParticles = 1);

	/** Clear all the particles (free memory) */
	void clear();

	/** Erase all the previous particles and change the number of particles,
	 * with a given initial value  */
	void setSize(
		size_t numberParticles,
		const mrpt::math::TPoint3Df& defaultValue = mrpt::math::TPoint3Df{0, 0,
																		  0});

	/** Returns the number of particles */
	size_t size() const { return m_particles.size(); }
	/** Returns an estimate of the point, (the mean, or mathematical expectation
	 * of the PDF) \sa getCovariance */
	void getMean(CPoint3D& mean_point) const override;

	/** Returns an estimate of the point covariance matrix (3x3 cov matrix) and
	 * the mean, both at once. \sa getMean  */
	void getCovarianceAndMean(
		mrpt::math::CMatrixDouble33& cov, CPoint3D& mean_point) const override;

	/** Copy operator, translating if necesary (for example, between particles
	 * and gaussian representations) */
	void copyFrom(const CPointPDF& o) override;

	/** Save PDF's particles to a text file, where each line is: X Y Z LOG_W */
	bool saveToTextFile(const std::string& file) const override;

	/** this = p (+) this. This can be used to convert a PDF from local
	 * coordinates to global, providing the point (newReferenceBase) from which
	 *   "to project" the current pdf. Result PDF substituted the currently
	 * stored one in the object. Both the mean value and the covariance matrix
	 * are updated correctly.  */
	void changeCoordinatesReference(const CPose3D& newReferenceBase) override;

	/** Compute the kurtosis of the distribution */
	double computeKurtosis();

	/** Draw a sample from the pdf */
	void drawSingleSample(CPoint3D& outSample) const override;

	/** Bayesian fusion of two point distributions (product of two
	 * distributions->new distribution), then save the result in this object
	 * (WARNING: See implementing classes to see classes that can and cannot be
	 * mixtured!)
	 * \param p1 The first distribution to fuse
	 * \param p2 The second distribution to fuse
	 * \param minMahalanobisDistToDrop If set to different of 0, the result of
	 * very separate Gaussian modes (that will result in negligible components)
	 * in SOGs will be dropped to reduce the number of modes in the output.
	 */
	void bayesianFusion(
		const CPointPDF& p1, const CPointPDF& p2,
		const double minMahalanobisDistToDrop = 0) override;

};  // End of class def.
}  // namespace mrpt::poses

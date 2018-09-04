/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPoint2DPDF.h>

namespace mrpt::poses
{
/** A gaussian distribution for 2D points. Also a method for bayesian fusion is
 * provided.
 * \ingroup poses_pdf_grp
 * \sa CPoint2DPDF
 */
class CPoint2DPDFGaussian : public CPoint2DPDF
{
	DEFINE_SERIALIZABLE(CPoint2DPDFGaussian)
	DEFINE_SCHEMA_SERIALIZABLE()

   public:
	/** Default constructor */
	CPoint2DPDFGaussian();
	/** Constructor */
	CPoint2DPDFGaussian(const CPoint2D& init_Mean);
	/** Constructor */
	CPoint2DPDFGaussian(
		const CPoint2D& init_Mean, const mrpt::math::CMatrixDouble22& init_Cov);

	/** The mean value */
	CPoint2D mean;
	/** The 2x2 covariance matrix */
	mrpt::math::CMatrixDouble22 cov;

	/** Returns an estimate of the point, (the mean, or mathematical expectation
	 * of the PDF) */
	void getMean(CPoint2D& p) const override { p = this->mean; }
	/** Returns an estimate of the point covariance matrix (2x2 cov matrix) and
	 * the mean, both at once. \sa getMean */
	void getCovarianceAndMean(
		mrpt::math::CMatrixDouble22& out_cov,
		CPoint2D& mean_point) const override
	{
		out_cov = this->cov;
		mean_point = this->mean;
	}

	/** Copy operator, translating if necesary (for example, between particles
	 * and gaussian representations) */
	void copyFrom(const CPoint2DPDF& o) override;

	/** Save PDF's particles to a text file, containing the 2D pose in the first
	 * line, then the covariance matrix in next 3 lines */
	bool saveToTextFile(const std::string& file) const override;

	/** this = p (+) this. This can be used to convert a PDF from local
	 * coordinates to global, providing the point (newReferenceBase) from which
	 *   "to project" the current pdf. Result PDF substituted the currently
	 * stored one in the object. Both the mean value and the covariance matrix
	 * are updated correctly. */
	void changeCoordinatesReference(const CPose3D& newReferenceBase) override;

	/** Bayesian fusion of two points gauss. distributions, then save the result
	 *in this object.
	 *  The process is as follows:<br>
	 *		- (x1,S1): Mean and variance of the p1 distribution.
	 *		- (x2,S2): Mean and variance of the p2 distribution.
	 *		- (x,S): Mean and variance of the resulting distribution.
	 *
	 *    S = (S1<sup>-1</sup> + S2<sup>-1</sup>)<sup>-1</sup>;
	 *    x = S * ( S1<sup>-1</sup>*x1 + S2<sup>-1</sup>*x2 );
	 */
	void bayesianFusion(
		const CPoint2DPDFGaussian& p1, const CPoint2DPDFGaussian& p2);

	/** Computes the "correspondence likelihood" of this PDF with another one:
	 * This is implemented as the integral from -inf to +inf of the product of
	 * both PDF.
	 * The resulting number is >=0.
	 * \sa productIntegralNormalizedWith
	 * \exception std::exception On errors like covariance matrix with null
	 * determinant, etc...
	 */
	double productIntegralWith(const CPoint2DPDFGaussian& p) const;

	/** Computes the "correspondence likelihood" of this PDF with another one:
	 * This is implemented as the integral from -inf to +inf of the product of
	 * both PDF.
	 * The resulting number is in the range [0,1].
	 *  Note that the resulting value is in fact
	 *  \f[ exp( -\frac{1}{2} D^2 ) \f]
	 *  , with \f$ D^2 \f$ being the square Mahalanobis distance between the
	 * two pdfs.
	 * \sa productIntegralWith
	 * \exception std::exception On errors like covariance matrix with null
	 * determinant, etc...
	 */
	double productIntegralNormalizedWith(const CPoint2DPDFGaussian& p) const;

	/** Draw a sample from the pdf */
	void drawSingleSample(CPoint2D& outSample) const override;

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
		const CPoint2DPDF& p1, const CPoint2DPDF& p2,
		const double minMahalanobisDistToDrop = 0) override;

	/** Returns the Mahalanobis distance from this PDF to another PDF, that is,
	 * it's evaluation at (0,0,0) */
	double mahalanobisDistanceTo(const CPoint2DPDFGaussian& other) const;
	/** Returns the Mahalanobis distance from this PDF to some point */
	double mahalanobisDistanceToPoint(const double x, const double y) const;

};  // End of class def.
}  // namespace mrpt::poses

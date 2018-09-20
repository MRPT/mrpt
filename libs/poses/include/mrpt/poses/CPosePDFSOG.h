/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/core/aligned_std_vector.h>
#include <ostream>

namespace mrpt::poses
{
/** Declares a class that represents a Probability Density  function (PDF) of a
 * 2D pose \f$ p(\mathbf{x}) = [x ~ y ~ \phi ]^t \f$.
 *   This class implements that PDF as the following multi-modal Gaussian
 * distribution:
 *
 * \f$ p(\mathbf{x}) = \sum\limits_{i=1}^N \omega^i \mathcal{N}( \mathbf{x} ;
 * \bar{\mathbf{x}}^i, \mathbf{\Sigma}^i )  \f$
 *
 *  Where the number of modes N is the size of CPosePDFSOG::m_modes
 *
 *  See mrpt::poses::CPosePDF for more details.
 *
 * \sa CPose2D, CPosePDF, CPosePDFParticles
 * \ingroup poses_pdf_grp
 */
class CPosePDFSOG : public CPosePDF
{
	DEFINE_SERIALIZABLE(CPosePDFSOG)

   public:
	/** The struct for each mode:
	 */
	struct TGaussianMode
	{
		TGaussianMode() : mean(), cov() {}
		CPose2D mean;
		mrpt::math::CMatrixDouble33 cov;

		/** The log-weight
		 */
		double log_w{0};

	   public:
		MRPT_MAKE_ALIGNED_OPERATOR_NEW;

		friend std::ostream& operator<<(
			std::ostream& o, const TGaussianMode& mode)
		{
			o << "Mean: " << mode.mean << std::endl
			  << "Covariance: " << std::endl
			  << mode.cov << std::endl
			  << "Log-weight: " << mode.log_w << std::endl;
			return o;
		}
	};

	using CListGaussianModes = mrpt::aligned_std_vector<TGaussianMode>;
	using const_iterator = CListGaussianModes::const_iterator;
	using iterator = CListGaussianModes::iterator;

	const CListGaussianModes& getSOGModes() const { return m_modes; }

   protected:
	/** Ensures the symmetry of the covariance matrix (eventually certain
	 * operations in the math-coprocessor lead to non-symmetric matrixes!) */
	void assureSymmetry();

	/** The list of SOG modes */
	CListGaussianModes m_modes;

   public:
	/** Default constructor
	 * \param nModes The initial size of CPosePDFSOG::m_modes */
	CPosePDFSOG(size_t nModes = 1);

	/** Return the number of Gaussian modes. */
	size_t size() const { return m_modes.size(); }
	/** Return whether there is any Gaussian mode. */
	bool empty() const { return m_modes.empty(); }
	/** Clear the list of modes */
	void clear();

	/** Access to individual beacons */
	const TGaussianMode& operator[](size_t i) const
	{
		ASSERT_(i < m_modes.size());
		return m_modes[i];
	}
	/** Access to individual beacons */
	TGaussianMode& operator[](size_t i)
	{
		ASSERT_(i < m_modes.size());
		return m_modes[i];
	}

	/** Access to individual beacons */
	const TGaussianMode& get(size_t i) const
	{
		ASSERT_(i < m_modes.size());
		return m_modes[i];
	}
	/** Access to individual beacons */
	TGaussianMode& get(size_t i)
	{
		ASSERT_(i < m_modes.size());
		return m_modes[i];
	}

	/** Inserts a copy of the given mode into the SOG */
	void push_back(const TGaussianMode& m) { m_modes.push_back(m); }
	iterator begin() { return m_modes.begin(); }
	iterator end() { return m_modes.end(); }
	const_iterator begin() const { return m_modes.begin(); }
	const_iterator end() const { return m_modes.end(); }
	iterator erase(iterator i) { return m_modes.erase(i); }
	/** Resize the number of SOG modes */
	void resize(const size_t N);

	/** Merge very close modes so the overall number of modes is reduced while
	 * preserving the total distribution.
	 *  This method uses the approach described in the paper:
	 *  - "Kullback-Leibler Approach to Gaussian Mixture Reduction" AR
	 * Runnalls. IEEE Transactions on Aerospace and Electronic Systems, 2007.
	 *
	 *  \param max_KLd The maximum KL-divergence to consider the merge of two
	 * nodes (and then stops the process).
	 */
	void mergeModes(double max_KLd = 0.5, bool verbose = false);

	/** Returns an estimate of the pose, (the mean, or mathematical expectation
	 * of the PDF) \sa getCovariance */
	void getMean(CPose2D& mean_pose) const override;
	/** Returns an estimate of the pose covariance matrix (3x3 cov matrix) and
	 * the mean, both at once. \sa getMean */
	void getCovarianceAndMean(
		mrpt::math::CMatrixDouble33& cov, CPose2D& mean_point) const override;
	/** For the most likely Gaussian mode in the SOG, returns the pose
	 * covariance matrix (3x3 cov matrix) and the mean. \sa getMean */
	void getMostLikelyCovarianceAndMean(
		mrpt::math::CMatrixDouble33& cov, CPose2D& mean_point) const;
	/** Normalize the weights in m_modes such as the maximum log-weight is 0 */
	void normalizeWeights();

	/** Copy operator, translating if necesary (for example, between particles
	 * and gaussian representations) */
	void copyFrom(const CPosePDF& o) override;

	/** Save the density to a text file, with the following format:
	 *  There is one row per Gaussian "mode", and each row contains 10
	 * elements:
	 *   - w (The weight)
	 *   - x_mean (gaussian mean value)
	 *   - y_mean (gaussian mean value)
	 *   - phi_mean (gaussian mean value)
	 *   - C11 (Covariance elements)
	 *   - C22 (Covariance elements)
	 *   - C33 (Covariance elements)
	 *   - C12 (Covariance elements)
	 *   - C13 (Covariance elements)
	 *   - C23 (Covariance elements)
	 */
	bool saveToTextFile(const std::string& file) const override;

	/** this = p (+) this. This can be used to convert a PDF from local
	 * coordinates to global, providing the point (newReferenceBase) from which
	 *   "to project" the current pdf. Result PDF substituted the currently
	 * stored one in the object. */
	void changeCoordinatesReference(const CPose3D& newReferenceBase) override;

	/** Rotate all the covariance matrixes by replacing them by \f$
	 * \mathbf{R}~\mathbf{COV}~\mathbf{R}^t \f$, where \f$ \mathbf{R} = \left[
	 * \begin{array}{ccc} \cos\alpha & -\sin\alpha & 0 \\ \sin\alpha &
	 * \cos\alpha & 0 \\ 0 & 0 & 1 \end{array}\right] \f$ */
	void rotateAllCovariances(const double& ang);
	/** Draws a single sample from the distribution */
	void drawSingleSample(CPose2D& outPart) const override;
	/** Draws a number of samples from the distribution, and saves as a list of
	 * 1x3 vectors, where each row contains a (x,y,phi) datum. */
	void drawManySamples(
		size_t N,
		std::vector<mrpt::math::CVectorDouble>& outSamples) const override;
	/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF */
	void inverse(CPosePDF& o) const override;

	/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the
	 * mean, and the covariance matrix are updated). */
	void operator+=(const mrpt::poses::CPose2D& Ap);

	/** Evaluates the PDF at a given point. */
	double evaluatePDF(
		const mrpt::poses::CPose2D& x, bool sumOverAllPhis = false) const;
	/** Evaluates the ratio PDF(x) / max_PDF(x*), that is, the normalized PDF in
	 * the range [0,1]. */
	double evaluateNormalizedPDF(const mrpt::poses::CPose2D& x) const;

	/** Evaluates the PDF within a rectangular grid (and a fixed orientation)
	 * and saves the result in a matrix (each row contains values for a fixed
	 * y-coordinate value). */
	void evaluatePDFInArea(
		const double& x_min, const double& x_max, const double& y_min,
		const double& y_max, const double& resolutionXY, const double& phi,
		mrpt::math::CMatrixD& outMatrix, bool sumOverAllPhis = false);

	/** Bayesian fusion of two pose distributions, then save the result in this
	 * object (WARNING: Currently p1 must be a mrpt::poses::CPosePDFSOG object
	 * and p2 a mrpt::poses::CPosePDFGaussian object) */
	void bayesianFusion(
		const CPosePDF& p1, const CPosePDF& p2,
		const double minMahalanobisDistToDrop = 0) override;

};  // End of class def.
}  // namespace mrpt::poses

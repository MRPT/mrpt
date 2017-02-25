/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPointPDFSOG_H
#define CPointPDFSOG_H

#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/CMatrixD.h>

namespace mrpt
{
	namespace poses
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPointPDFSOG, CPointPDF )

		/** Declares a class that represents a Probability Density function (PDF) of a 3D point \f$ p(\mathbf{x}) = [x ~ y ~ z ]^t \f$.
		 *   This class implements that PDF as the following multi-modal Gaussian distribution:
		 *
		 * \f$ p(\mathbf{x}) = \sum\limits_{i=1}^N \omega^i \mathcal{N}( \mathbf{x} ; \bar{\mathbf{x}}^i, \mathbf{\Sigma}^i )  \f$
		 *
		 *  Where the number of modes N is the size of CPointPDFSOG::m_modes
		 *
		 *  See mrpt::poses::CPointPDF for more details.
		 *
		 * \sa CPointPDF, CPosePDF,
		 * \ingroup poses_pdf_grp
		 */
		class BASE_IMPEXP  CPointPDFSOG : public CPointPDF
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CPointPDFSOG )

		public:
			/** The struct for each mode:
			 */
			struct BASE_IMPEXP TGaussianMode
			{
				TGaussianMode() : val(), log_w(0)
				{
				}

				CPointPDFGaussian		val;

				/** The log-weight
				  */
				double		log_w;
			};

			typedef std::deque<TGaussianMode> CListGaussianModes;
			typedef std::deque<TGaussianMode>::const_iterator const_iterator;
			typedef std::deque<TGaussianMode>::iterator iterator;

		protected:
			/** Assures the symmetry of the covariance matrix (eventually certain operations in the math-coprocessor lead to non-symmetric matrixes!)
			  */
			void  assureSymmetry();

			/** The list of SOG modes */
			CListGaussianModes	m_modes;

		 public:
			/** Default constructor
			  * \param nModes The initial size of CPointPDFSOG::m_modes
			  */
			CPointPDFSOG( size_t nModes = 1 );

			void clear(); //!< Clear all the gaussian modes

			/** Access to individual beacons */
			const TGaussianMode& operator [](size_t i) const {
				ASSERT_(i<m_modes.size())
				return  m_modes[i];
			}
			/** Access to individual beacons */
			TGaussianMode& operator [](size_t i) {
				ASSERT_(i<m_modes.size())
				return  m_modes[i];
			}

			/** Access to individual beacons */
			const TGaussianMode& get(size_t i) const {
				ASSERT_(i<m_modes.size())
				return  m_modes[i];
			}
			/** Access to individual beacons */
			TGaussianMode& get(size_t i) {
				ASSERT_(i<m_modes.size())
				return  m_modes[i];
			}

			/** Inserts a copy of the given mode into the SOG */
			void push_back(const TGaussianMode& m) {
				m_modes.push_back(m);
			}

			iterator begin() { return m_modes.begin(); }
			iterator end() { return m_modes.end(); }
			const_iterator begin() const { return m_modes.begin(); }
			const_iterator end()const { return m_modes.end(); }

			iterator erase(iterator i) { return m_modes.erase(i); }

			void resize(const size_t N); //!< Resize the number of SOG modes
			size_t size() const { return m_modes.size(); } //!< Return the number of Gaussian modes.
			bool empty() const { return m_modes.empty(); } //!< Return whether there is any Gaussian mode.

			 /** Returns an estimate of the point, (the mean, or mathematical expectation of the PDF) \sa getCovariance   */
			void getMean(CPoint3D &mean_point) const MRPT_OVERRIDE;

			/** Returns an estimate of the point covariance matrix (3x3 cov matrix) and the mean, both at once. \sa getMean */
			void getCovarianceAndMean(mrpt::math::CMatrixDouble33 &cov,CPoint3D &mean_point) const MRPT_OVERRIDE;

			/** Normalize the weights in m_modes such as the maximum log-weight is 0 */
			void  normalizeWeights();

			/** Return the Gaussian mode with the highest likelihood (or an empty Gaussian if there are no modes in this SOG) */
			void getMostLikelyMode( CPointPDFGaussian& outVal ) const;

			/** Computes the "Effective sample size" (typical measure for Particle Filters), applied to the weights of the individual Gaussian modes, as a measure of the equality of the modes (in the range [0,total # of modes]). */
			double ESS() const;

			void  copyFrom(const CPointPDF &o) MRPT_OVERRIDE; //!< Copy operator, translating if necesary (for example, between particles and gaussian representations)

			/** Save the density to a text file, with the following format:
			  *  There is one row per Gaussian "mode", and each row contains 10 elements:
			  *   - w (The weight)
			  *   - x_mean (gaussian mean value)
			  *   - y_mean (gaussian mean value)
			  *   - x_mean (gaussian mean value)
			  *   - C11 (Covariance elements)
			  *   - C22 (Covariance elements)
			  *   - C33 (Covariance elements)
			  *   - C12 (Covariance elements)
			  *   - C13 (Covariance elements)
			  *   - C23 (Covariance elements)
			  *
			 */
			void  saveToTextFile(const std::string &file) const MRPT_OVERRIDE;

			/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
			  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object. */
			void  changeCoordinatesReference(const CPose3D &newReferenceBase ) MRPT_OVERRIDE;

			/** Draw a sample from the pdf. */
			void drawSingleSample(CPoint3D  &outSample) const MRPT_OVERRIDE;

			/** Bayesian fusion of two point distributions (product of two distributions->new distribution), then save the result in this object (WARNING: See implementing classes to see classes that can and cannot be mixtured!)
			  * \param p1 The first distribution to fuse
			  * \param p2 The second distribution to fuse
			  * \param minMahalanobisDistToDrop If set to different of 0, the result of very separate Gaussian modes (that will result in negligible components) in SOGs will be dropped to reduce the number of modes in the output.
			  */
			void  bayesianFusion( const CPointPDF &p1, const CPointPDF &p2, const double &minMahalanobisDistToDrop = 0) MRPT_OVERRIDE;


			/** Evaluates the PDF within a rectangular grid and saves the result in a matrix (each row contains values for a fixed y-coordinate value).
			  */
			void  evaluatePDFInArea(
				float		x_min,
				float		x_max,
				float		y_min,
				float		y_max,
				float		resolutionXY,
				float		z,
				mrpt::math::CMatrixD	&outMatrix,
				bool		sumOverAllZs = false );

			/** Evaluates the PDF at a given point */
			double  evaluatePDF( const	CPoint3D &x, bool	sumOverAllZs ) const;

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPointPDFSOG, CPointPDF )
	} // End of namespace
} // End of namespace
#endif

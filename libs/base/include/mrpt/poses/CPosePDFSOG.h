/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CPosePDFSOG_H
#define CPosePDFSOG_H

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/CMatrixD.h>


namespace mrpt
{
	namespace poses
	{
		using namespace mrpt::math;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPosePDFSOG , CPosePDF )

		/** Declares a class that represents a Probability Density  function (PDF) of a 2D pose \f$ p(\mathbf{x}) = [x ~ y ~ \phi ]^t \f$.
		 *   This class implements that PDF as the following multi-modal Gaussian distribution:
		 *
		 * \f$ p(\mathbf{x}) = \sum\limits_{i=1}^N \omega^i \mathcal{N}( \mathbf{x} ; \bar{\mathbf{x}}^i, \mathbf{\Sigma}^i )  \f$
		 *
		 *  Where the number of modes N is the size of CPosePDFSOG::m_modes
		 *
		 *  See mrpt::poses::CPosePDF for more details.
		 *
		 * \sa CPose2D, CPosePDF, CPosePDFParticles
		 * \ingroup poses_pdf_grp
		 */
		class BASE_IMPEXP CPosePDFSOG : public CPosePDF
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CPosePDFSOG )

		public:
			/** The struct for each mode:
			 */
			struct BASE_IMPEXP TGaussianMode
			{
				TGaussianMode() :
					mean(),
					cov(),
					log_w(0)
				{ }

				CPose2D			mean;
				CMatrixDouble33	cov;

				/** The log-weight
				  */
				double		log_w;

			public:
			  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			};

			typedef mrpt::aligned_containers<TGaussianMode>::vector_t	CListGaussianModes;
			typedef CListGaussianModes::const_iterator const_iterator;
			typedef CListGaussianModes::iterator iterator;

		protected:
			/** Assures the symmetry of the covariance matrix (eventually certain operations in the math-coprocessor lead to non-symmetric matrixes!)
			  */
			void  assureSymmetry();

			/** The list of SOG modes */
			CListGaussianModes	m_modes;

		 public:
			/** Default constructor
			  * \param nModes The initial size of CPosePDFSOG::m_modes
			  */
			CPosePDFSOG( size_t nModes = 1 );

			size_t size() const { return m_modes.size(); } //!< Return the number of Gaussian modes.
			bool empty() const { return m_modes.empty(); } //!< Return whether there is any Gaussian mode.

			/** Clear the list of modes */
			void clear();

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

			/** Merge very close modes so the overall number of modes is reduced while preserving the total distribution.
			  *  This method uses the approach described in the paper:
			  *  - "Kullback-Leibler Approach to Gaussian Mixture Reduction" AR Runnalls. IEEE Transactions on Aerospace and Electronic Systems, 2007.
			  *
			  *  \param max_KLd The maximum KL-divergence to consider the merge of two nodes (and then stops the process).
			  */
			void mergeModes( double max_KLd = 0.5, bool verbose = false );

			 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF).
			   * \sa getCovariance
			   */
			void getMean(CPose2D &mean_pose) const;

			/** Returns an estimate of the pose covariance matrix (3x3 cov matrix) and the mean, both at once.
			  * \sa getMean
			  */
			void getCovarianceAndMean(CMatrixDouble33 &cov,CPose2D &mean_point) const;

			/** For the most likely Gaussian mode in the SOG, returns the pose covariance matrix (3x3 cov matrix) and the mean.
			  * \sa getMean
			  */
			void getMostLikelyCovarianceAndMean(CMatrixDouble33 &cov,CPose2D &mean_point) const;

			/** Normalize the weights in m_modes such as the maximum log-weight is 0.
			  */
			void  normalizeWeights();

			/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
			  */
			void  copyFrom(const CPosePDF &o);

			/** Save the density to a text file, with the following format:
			  *  There is one row per Gaussian "mode", and each row contains 10 elements:
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
			  *
			 */
			void  saveToTextFile(const std::string &file) const;

			/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
			  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
			  */
			void  changeCoordinatesReference(const CPose3D &newReferenceBase );

			/** Rotate all the covariance matrixes by replacing them by \f$ \mathbf{R}~\mathbf{COV}~\mathbf{R}^t \f$, where \f$ \mathbf{R} = \left[ \begin{array}{ccc} \cos\alpha & -\sin\alpha & 0 \\ \sin\alpha & \cos\alpha & 0 \\ 0 & 0 & 1 \end{array}\right] \f$.
			  */
			void  rotateAllCovariances(const double &ang);

			/** Draws a single sample from the distribution
			  */
			void  drawSingleSample( CPose2D &outPart ) const;

			/** Draws a number of samples from the distribution, and saves as a list of 1x3 vectors, where each row contains a (x,y,phi) datum.
			  */
			void  drawManySamples( size_t N, std::vector<vector_double> & outSamples ) const;

			/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
			  */
			void	 inverse(CPosePDF &o) const;

			/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated).
			  */
			void  operator += ( const CPose2D &Ap);

			/** Evaluates the PDF at a given point.
			  */
			double  evaluatePDF( const CPose2D &x, bool sumOverAllPhis = false ) const;

			/** Evaluates the ratio PDF(x) / max_PDF(x*), that is, the normalized PDF in the range [0,1].
			  */
			double  evaluateNormalizedPDF( const CPose2D &x ) const;

			/** Evaluates the PDF within a rectangular grid (and a fixed orientation) and saves the result in a matrix (each row contains values for a fixed y-coordinate value).
			  */
			void  evaluatePDFInArea(
				const double &		x_min,
				const double &		x_max,
				const double &		y_min,
				const double &		y_max,
				const double &		resolutionXY,
				const double &		phi,
				CMatrixD	&outMatrix,
				bool		sumOverAllPhis = false );

			/** Bayesian fusion of two pose distributions, then save the result in this object (WARNING: Currently p1 must be a mrpt::poses::CPosePDFSOG object and p2 a mrpt::poses::CPosePDFGaussian object)
			  */
			void  bayesianFusion(const  CPosePDF &p1,const  CPosePDF &p2, const double &minMahalanobisDistToDrop=0 );


		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif

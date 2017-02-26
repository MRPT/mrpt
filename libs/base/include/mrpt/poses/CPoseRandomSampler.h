/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPoseRandomSampler_H
#define CPoseRandomSampler_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/math_frwds.h>

namespace mrpt
{
    namespace poses
    {
        /** An efficient generator of random samples drawn from a given 2D (CPosePDF) or 3D (CPose3DPDF) pose probability density function (pdf).
         * This class keeps an internal state which speeds up the sequential generation of samples. It can manage
         *  any kind of pose PDF.
		 *
         * Use with CPoseRandomSampler::setPosePDF, then CPoseRandomSampler::drawSample to draw values.
		 *
         * Notice that you can pass a 2D or 3D pose PDF, then ask for a 2D or 3D sample. This class always returns
         *  the kind of sample you ask it for, but will skip missing terms or fill out with zeroes as required.
		 * Specifically, when sampling 3D poses from a 2D pose pdf, this class will be smart enought to draw only
		 *  the 3 required dimensions, avoiding a waste of time with the other 3 missing components.
         *
		 * \ingroup poses_pdf_grp
         * \sa CPosePDF, CPose3DPDF
         */
        class BASE_IMPEXP CPoseRandomSampler
        {
        protected:
			// Only ONE of these can be not-NULL at a time.
            CPosePDF*   m_pdf2D;        //!< A local copy of the PDF
            CPose3DPDF* m_pdf3D;        //!< A local copy of the PDF

            mrpt::math::CMatrixDouble33 m_fastdraw_gauss_Z3;
            mrpt::math::CMatrixDouble66 m_fastdraw_gauss_Z6;
            CPose2D         m_fastdraw_gauss_M_2D;
            CPose3D         m_fastdraw_gauss_M_3D;

            void clear(); //!< Clear internal pdf

			void do_sample_2D( CPose2D &p ) const;	//!< Used internally: sample from m_pdf2D
			void do_sample_3D( CPose3D &p ) const;	//!< Used internally: sample from m_pdf3D

        public:
            /** Default constructor */
            CPoseRandomSampler();

            /** Destructor */
            ~CPoseRandomSampler();

            /** This method must be called to select the PDF from which to draw samples.
              * \sa drawSample
              */
            void setPosePDF( const CPosePDF *pdf );

            /** This method must be called to select the PDF from which to draw samples.
              * \sa drawSample
              */
            void setPosePDF( const CPosePDFPtr &pdf );

            /** This method must be called to select the PDF from which to draw samples.
              * \sa drawSample
              */
            void setPosePDF( const CPosePDF &pdf ) { setPosePDF(&pdf); }

            /** This method must be called to select the PDF from which to draw samples.
              * \sa drawSample
              */
            void setPosePDF( const CPose3DPDF *pdf );

            /** This method must be called to select the PDF from which to draw samples.
              * \sa drawSample
              */
            void setPosePDF( const CPose3DPDFPtr &pdf );

            /** This method must be called to select the PDF from which to draw samples.
              * \sa drawSample
              */
            void setPosePDF( const CPose3DPDF &pdf ) { setPosePDF(&pdf); }

            /** Generate a new sample from the selected PDF.
              * \return A reference to the same object passed as argument.
              * \sa setPosePDF
              */
            CPose2D & drawSample( CPose2D &p ) const;

            /** Generate a new sample from the selected PDF.
              * \return A reference to the same object passed as argument.
              * \sa setPosePDF
              */
            CPose3D & drawSample( CPose3D &p ) const;

			/** Return true if samples can be generated, which only requires a previous call to setPosePDF */
			bool isPrepared() const;

			/** If the object has been loaded with setPosePDF this method returns the 2D pose mean samples will be drawn around. \return A reference to the argument */
			CPose2D &getSamplingMean2D(CPose2D &out_mean) const;

			/** If the object has been loaded with setPosePDF this method returns the 3D pose mean samples will be drawn around. \return A reference to the argument */
			CPose3D &getSamplingMean3D(CPose3D &out_mean) const;

			/** Retrieves the 3x3 covariance of the original PDF in \f$ [ x ~ y ~ \phi ] \f$. */
			void getOriginalPDFCov2D( mrpt::math::CMatrixDouble33 &cov3x3 ) const;

			/** Retrieves the 3x3 covariance of the original PDF in \f$ [ x ~ y ~ \phi ] \f$. */
			inline void getOriginalPDFCov2D( mrpt::math::CMatrixDouble &cov3x3 ) const {
				mrpt::math::CMatrixDouble33 M;
				this->getOriginalPDFCov2D(M);
				cov3x3 = mrpt::math::CMatrixDouble(M);
			}

			/** Retrieves the 6x6 covariance of the original PDF in \f$ [ x ~ y ~ z ~ yaw ~ pitch ~ roll ] \f$. */
			void getOriginalPDFCov3D( mrpt::math::CMatrixDouble66 &cov6x6 ) const;

			/** Retrieves the 6x6 covariance of the original PDF in \f$ [ x ~ y ~ z ~ yaw ~ pitch ~ roll ] \f$. */
			inline void getOriginalPDFCov3D( mrpt::math::CMatrixDouble &cov6x6 ) const {
				mrpt::math::CMatrixDouble66 M;
				this->getOriginalPDFCov3D(M);
				cov6x6 = mrpt::math::CMatrixDouble(M);
			}

        }; // End of class def.
	} // End of namespace
} // End of namespace

#endif

/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
#ifndef CPoseRandomSampler_H
#define CPoseRandomSampler_H

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>

namespace mrpt
{
    namespace poses
    {
        using namespace mrpt::math;

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

            CMatrixDouble33 m_fastdraw_gauss_Z3;
            CMatrixDouble66 m_fastdraw_gauss_Z6;
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
            void setPosePDF( const CPosePDFPtr &pdf ) { setPosePDF(pdf.pointer()); }

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
            void setPosePDF( const CPose3DPDFPtr &pdf ) { setPosePDF(pdf.pointer()); }

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
			void getOriginalPDFCov2D( CMatrixDouble33 &cov3x3 ) const;

			/** Retrieves the 3x3 covariance of the original PDF in \f$ [ x ~ y ~ \phi ] \f$. */
			inline void getOriginalPDFCov2D( CMatrixDouble &cov3x3 ) const {
				CMatrixDouble33 M;
				this->getOriginalPDFCov2D(M);
				cov3x3 = CMatrixDouble(M);
			}

			/** Retrieves the 6x6 covariance of the original PDF in \f$ [ x ~ y ~ z ~ yaw ~ pitch ~ roll ] \f$. */
			void getOriginalPDFCov3D( CMatrixDouble66 &cov6x6 ) const;

			/** Retrieves the 6x6 covariance of the original PDF in \f$ [ x ~ y ~ z ~ yaw ~ pitch ~ roll ] \f$. */
			inline void getOriginalPDFCov3D( CMatrixDouble &cov6x6 ) const {
				CMatrixDouble66 M;
				this->getOriginalPDFCov3D(M);
				cov6x6 = CMatrixDouble(M);
			}

        }; // End of class def.
	} // End of namespace
} // End of namespace

#endif

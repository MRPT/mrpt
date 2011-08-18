/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CPointPDFParticles_H
#define CPointPDFParticles_H

#include <mrpt/poses/CPointPDF.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/bayes/CParticleFilterData.h>

namespace mrpt
{
namespace poses
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( TSimple3DPoint, mrpt::utils::CSerializable )

	/** Data within each particle
	 * \ingroup poses_pdf_grp
	  */
	class BASE_IMPEXP TSimple3DPoint : public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( TSimple3DPoint )
	public:
		TSimple3DPoint(const TSimple3DPoint&o) : x(o.x),y(o.y),z(o.z)
		{
		}

		TSimple3DPoint() : x(0),y(0),z(0)
		{
		}

		TSimple3DPoint(const CPoint3D &v) : x(v.x()),y(v.y()),z(v.z())
		{
		}

		float	x,y,z;
	};

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPointPDFParticles, CPointPDF )

	/** A probability distribution of a 2D/3D point, represented as a set of random samples (particles).
	 * \sa CPointPDF
	 * \ingroup poses_pdf_grp
	 */
	class BASE_IMPEXP CPointPDFParticles : public CPointPDF, public mrpt::bayes::CParticleFilterData<TSimple3DPoint>
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPointPDFParticles )

		// This uses CParticleFilterData to implement some methods required for CParticleFilterCapable:
		IMPLEMENT_PARTICLE_FILTER_CAPABLE(TSimple3DPoint);

	 public:
		/** Default constructor
		  */
		CPointPDFParticles(size_t numParticles = 1);

		/** Destructor
		  */
		virtual ~CPointPDFParticles();

		/** Clear all the particles (free memory)
		  */
		void clear()  { setSize(0);	}

		/** Erase all the previous particles and change the number of particles, with a given initial value
		  */
		void setSize(size_t numberParticles, const CPoint3D &defaultValue = CPoint3D(0,0,0) );

		/** Returns the number of particles
		  */
		size_t size() const
		{
			return m_particles.size();
		}

		 /** Returns an estimate of the point, (the mean, or mathematical expectation of the PDF).
		   * \sa getCovariance
		   */
		void getMean(CPoint3D &mean_point) const;

		/** Returns an estimate of the point covariance matrix (3x3 cov matrix) and the mean, both at once.
		  * \sa getMean
		  */
		void getCovarianceAndMean(CMatrixDouble33 &cov,CPoint3D &mean_point) const;

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		void  copyFrom(const CPointPDF &o);

		/** Save PDF's particles to a text file, where each line is: X Y Z LOG_W
		 */
		void  saveToTextFile(const std::string &file) const;

		/** This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object. Both the mean value and the covariance matrix are updated correctly.
		  */
		void  changeCoordinatesReference( const CPose3D &newReferenceBase );

		/** Compute the kurtosis of the distribution.
		  */
		double computeKurtosis();

		/** Draw a sample from the pdf.
		  */
		void drawSingleSample(CPoint3D  &outSample) const;

		/** Bayesian fusion of two point distributions (product of two distributions->new distribution), then save the result in this object (WARNING: See implementing classes to see classes that can and cannot be mixtured!)
		  * \param p1 The first distribution to fuse
		  * \param p2 The second distribution to fuse
		  * \param minMahalanobisDistToDrop If set to different of 0, the result of very separate Gaussian modes (that will result in negligible components) in SOGs will be dropped to reduce the number of modes in the output.
		  */
		void  bayesianFusion( const CPointPDF &p1, const CPointPDF &p2, const double &minMahalanobisDistToDrop = 0);

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif

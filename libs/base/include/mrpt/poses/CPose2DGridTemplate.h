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
#ifndef CPose2DGridTemplate_H
#define CPose2DGridTemplate_H

#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
namespace poses
{
	using namespace mrpt::math;
	using namespace mrpt::utils;

	/** This is a template class for storing a 3D (2D+heading) grid containing any kind of data.
	 * \ingroup poses_pdf_grp
	 */
	template<class T>
	class CPose2DGridTemplate
	{
	protected:
		/** The limits and resolution of the grid:
		  */
		double				m_xMin, m_xMax,
							m_yMin, m_yMax,
							m_phiMin, m_phiMax,
							m_resolutionXY,m_resolutionPhi;

		/** The size of "m_data" is m_sizeX ·m_sizeY ·m_sizePhi
		  */
		size_t		m_sizeX,m_sizeY,m_sizePhi, m_sizeXY;

		/** The indexes of the "left" borders:
		  */
		int					m_idxLeftX, m_idxLeftY, m_idxLeftPhi;

		/** The data:
		  */
		std::vector<T>		m_data;

	public:
		/** Returns "indexes" from coordinates:
		  */
		size_t  x2idx(double x) const
		{
			int idx = round( (x-m_xMin) / m_resolutionXY );
			ASSERT_(idx>=0 && idx< static_cast<int>(m_sizeX));
			return idx;
		}

		/** Returns "indexes" from coordinates:
		  */
		size_t  y2idx(double y) const
		{
			int idx = round( (y-m_yMin) / m_resolutionXY );
			ASSERT_(idx>=0 && idx< static_cast<int>(m_sizeY));
			return idx;
		}

		/** Returns "indexes" from coordinates:
		  */
		size_t  phi2idx(double phi) const
		{
			int idx = round( (phi-m_phiMin) / m_resolutionPhi );
			ASSERT_(idx>=0 && idx< static_cast<int>(m_sizePhi) );
			return idx;
		}

		/** Returns coordinates from "indexes":
		  */
		double  idx2x(size_t x) const
		{
			ASSERT_(x<m_sizeX);
			return m_xMin + x * m_resolutionXY;
		}

		/** Returns coordinates from "indexes":
		  */
		double  idx2y(size_t y) const
		{
			ASSERT_(y<m_sizeY);
			return m_yMin + y * m_resolutionXY;
		}

		/** Returns coordinates from "indexes":
		  */
		double  idx2phi(size_t phi) const
		{
			ASSERT_(phi<m_sizePhi);
			return m_phiMin + phi * m_resolutionPhi;
		}

		/** Default constructor:
		  */
		CPose2DGridTemplate(
			double		xMin = -1.0f,
			double		xMax = 1.0f,
			double		yMin = -1.0f,
			double		yMax = 1.0f,
			double		resolutionXY = 0.5f,
			double		resolutionPhi = DEG2RAD(180),
			double		phiMin = -M_PIf,
			double		phiMax = M_PIf
			) :
				m_xMin(), m_xMax(),
				m_yMin(), m_yMax(),
				m_phiMin(), m_phiMax(),
				m_resolutionXY(),m_resolutionPhi(),
				m_sizeX(),m_sizeY(),m_sizePhi(), m_sizeXY(),
				m_idxLeftX(), m_idxLeftY(), m_idxLeftPhi(),
				m_data()
		{
			setSize(xMin,xMax,yMin,yMax,resolutionXY,resolutionPhi,phiMin,phiMax);
		}

		virtual ~CPose2DGridTemplate() { }


		/** Changes the limits and size of the grid, erasing previous contents:
		  */
		void  setSize(
			double		xMin,
			double		xMax,
			double		yMin,
			double		yMax,
			double		resolutionXY,
			double		resolutionPhi,
			double		phiMin = -M_PIf,
			double		phiMax = M_PIf
			)
		{
			// Checks
			ASSERT_( xMax > xMin );
			ASSERT_( yMax > yMin );
			ASSERT_( phiMax >= phiMin );
			ASSERT_( resolutionXY>0 );
			ASSERT_( resolutionPhi>0 );

			// Copy data:
			m_xMin = xMin;			m_xMax = xMax;
			m_yMin = yMin;			m_yMax = yMax;
			m_phiMin = phiMin;		m_phiMax = phiMax;
			m_resolutionXY = resolutionXY;
			m_resolutionPhi = resolutionPhi;

			// Compute the indexes of the starting borders:
			m_idxLeftX = round( xMin/resolutionXY ) ;
			m_idxLeftY = round( yMin/resolutionXY ) ;
			m_idxLeftPhi = round( phiMin/resolutionPhi ) ;

			// Compute new required space:
			m_sizeX = round( xMax/resolutionXY ) - m_idxLeftX + 1;
			m_sizeY = round( yMax/resolutionXY ) - m_idxLeftY + 1;
			m_sizePhi = round( phiMax/resolutionPhi ) - m_idxLeftPhi + 1;
			m_sizeXY = m_sizeX * m_sizeY;

			// Resize "m_data":
			m_data.clear();
			m_data.resize( m_sizeX * m_sizeY * m_sizePhi );
		}

		/** Reads the contents of a cell
		  */
		const T*  getByPos( double x,double y, double phi ) const
		{
            return getByIndex( x2idx(x),y2idx(y),phi2idx(phi) );
		}

		/** Reads the contents of a cell
		  */
		T*  getByPos( double x,double y, double phi )
		{
            return getByIndex( x2idx(x),y2idx(y),phi2idx(phi) );
		}

		/** Reads the contents of a cell
		  */
		const T*  getByIndex( size_t x,size_t y, size_t phi )  const
		{
			ASSERT_(x>=0 && x<m_sizeX);
			ASSERT_(y>=0 && y<m_sizeY);
			ASSERT_(phi>=0 && phi<m_sizePhi);
			return &m_data[ phi*m_sizeXY + y*m_sizeX + x ];
		}

		/** Reads the contents of a cell
		  */
		T*  getByIndex( size_t x,size_t y, size_t phi )
		{
			ASSERT_(x>=0 && x<m_sizeX);
			ASSERT_(y>=0 && y<m_sizeY);
			ASSERT_(phi>=0 && phi<m_sizePhi);
			return &m_data[ phi*m_sizeXY + y*m_sizeX + x ];
		}

		/** Returns the whole grid as a matrix, for a given constant "phi" and where each row contains values for a fixed "y".
		  */
		void getAsMatrix( const double &phi, CMatrixTemplate<T> &outMat )
		{
			MRPT_START
			outMat.setSize( m_sizeY, m_sizeX );
			size_t phiIdx = phi2idx(phi);
			ASSERT_(phi<m_sizePhi);
			for (size_t y=0;y<m_sizeY;y++)
				for (size_t x=0;x<m_sizeX;x++)
					outMat(y,x)=m_data[ phiIdx*m_sizeXY + y*m_sizeX + x ];
			MRPT_END
		}

		/** Get info about the grid:
		  */
		double  getXMin() const { return m_xMin; }
		double  getXMax() const { return m_xMax; }
		double  getYMin() const { return m_yMin; }
		double  getYMax() const { return m_yMax; }
		double  getPhiMin() const { return m_phiMin; }
		double  getPhiMax() const { return m_phiMax; }
		double  getResolutionXY() const { return m_resolutionXY; }
		double  getResolutionPhi() const { return m_resolutionPhi; }
		size_t  getSizeX() const { return m_sizeX; }
		size_t  getSizeY() const { return m_sizeY; }
		size_t  getSizePhi() const { return m_sizePhi; }


	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif

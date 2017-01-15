/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPose2DGridTemplate_H
#define CPose2DGridTemplate_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/round.h> // for round()
#include <mrpt/utils/bits.h>  // for DEG2RAD()

namespace mrpt
{
namespace poses
{
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

		/** The size of "m_data" is m_sizeX * m_sizeY * m_sizePhi
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
			int idx = mrpt::utils::round( (x-m_xMin) / m_resolutionXY );
			ASSERT_(idx>=0 && idx< static_cast<int>(m_sizeX));
			return idx;
		}

		/** Returns "indexes" from coordinates:
		  */
		size_t  y2idx(double y) const
		{
			int idx = mrpt::utils::round( (y-m_yMin) / m_resolutionXY );
			ASSERT_(idx>=0 && idx< static_cast<int>(m_sizeY));
			return idx;
		}

		/** Returns "indexes" from coordinates:
		  */
		size_t  phi2idx(double phi) const
		{
			int idx = mrpt::utils::round( (phi-m_phiMin) / m_resolutionPhi );
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
			double		resolutionPhi = mrpt::utils::DEG2RAD(180),
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
			m_idxLeftX = mrpt::utils::round( xMin/resolutionXY ) ;
			m_idxLeftY = mrpt::utils::round( yMin/resolutionXY ) ;
			m_idxLeftPhi = mrpt::utils::round( phiMin/resolutionPhi ) ;

			// Compute new required space:
			m_sizeX = mrpt::utils::round( xMax/resolutionXY ) - m_idxLeftX + 1;
			m_sizeY = mrpt::utils::round( yMax/resolutionXY ) - m_idxLeftY + 1;
			m_sizePhi = mrpt::utils::round( phiMax/resolutionPhi ) - m_idxLeftPhi + 1;
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
			ASSERT_(x<m_sizeX && y<m_sizeY && phi<m_sizePhi)
			return &m_data[ phi*m_sizeXY + y*m_sizeX + x ];
		}

		/** Reads the contents of a cell
		  */
		T*  getByIndex( size_t x,size_t y, size_t phi )
		{
			ASSERT_(x<m_sizeX && y<m_sizeY && phi<m_sizePhi)
			return &m_data[ phi*m_sizeXY + y*m_sizeX + x ];
		}

		/** Returns the whole grid as a matrix, for a given constant "phi" and where each row contains values for a fixed "y".
		  */
		template <class MATRIXLIKE>
		void getAsMatrix( const double &phi, MATRIXLIKE &outMat )
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

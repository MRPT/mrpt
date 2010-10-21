/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#ifndef opengl_CPointCloud_H
#define opengl_CPointCloud_H

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/utils/CImage.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CPointCloud;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CPointCloud, CRenderizable, OPENGL_IMPEXP )


		/** A cloud of points, all with the same color or each depending on its value along a particular coordinate axis.
		  *  This class is just an OpenGL representation of a point cloud. For operating with maps of points, see mrpt::slam::CPointsMap and derived classes.
		  *
		  *  \sa opengl::CPlanarLaserScan, opengl::COpenGLScene, opengl::CPointCloudColoured, mrpt::slam::CPointsMap
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CPointCloud </td> <td> \image html preview_CPointCloud.png </td> </tr>
		  *  </table>
		  *  </div>
		  *  
		  */
		class OPENGL_IMPEXP CPointCloud : public CRenderizable
		{
			DEFINE_SERIALIZABLE( CPointCloud )
		protected:
			enum Axis { None=0, Z, Y, X} m_colorFromDepth;
			vector_float	m_xs,m_ys,m_zs;
			float           m_pointSize; //!< By default is 1.0
			bool			m_pointSmooth; //!< Default: false

		public:
			inline void enableColorFromX(bool v=true) { m_colorFromDepth = v ? CPointCloud::X : CPointCloud::None;  }
			inline void enableColorFromY(bool v=true) { m_colorFromDepth = v ? CPointCloud::Y : CPointCloud::None; }
			inline void enableColorFromZ(bool v=true) { m_colorFromDepth = v ? CPointCloud::Z : CPointCloud::None; }

			inline void resize(size_t N) { m_xs.resize(N); m_ys.resize(N); m_zs.resize(N);  }
			inline void reserve(size_t N) { m_xs.reserve(N); m_ys.reserve(N); m_zs.reserve(N);  }

			inline vector_float & getArrayX() {return m_xs;} //!< Get a reference to the internal array of X coordinates
			inline vector_float & getArrayY() {return m_ys;} //!< Get a reference to the internal array of Y coordinates
			inline vector_float & getArrayZ() {return m_zs;} //!< Get a reference to the internal array of Z coordinates

			inline void setPointSize(float p) { m_pointSize=p; }  //!< By default is 1.0
			inline float getPointSize() const { return m_pointSize; }

			inline void enablePointSmooth(bool enable=true) { m_pointSmooth=enable; }
			inline void disablePointSmooth() { m_pointSmooth=false; }
			inline bool isPointSmoothEnabled() const { return m_pointSmooth; }

			/** Empty the list of points. */
			void clear();

			/** Adds a new point to the cloud */
			void insertPoint( float x,float y, float z );

			/** Load the points from a pointsMap (mrpt::slam::CPointsMap), passed as a pointer.
			  * Note that the method is a template since CPointsMap belongs to a different mrpt library.
			  */
			template <class POINTSMAP>
			inline void  loadFromPointsMap( const POINTSMAP *themap) {
				themap->getAllPoints(m_xs,m_ys,m_zs);
			}

			/** Load the points from a list of TPoint3D
			  */
			template<class LISTOFPOINTS> void  loadFromPointsList( LISTOFPOINTS &pointsList)
			{
				MRPT_START

				size_t N = pointsList.size();

				m_xs.resize(N);
				m_ys.resize(N);
				m_zs.resize(N);

				vector_float::iterator X, Y, Z;
				typename LISTOFPOINTS::const_iterator it;

				for ( it=pointsList.begin(), X=m_xs.begin(), Y=m_ys.begin(),Z=m_zs.begin(); it!=pointsList.end(); it++, X++, Y++, Z++)
				{
					*X = (*it).x;
					*Y = (*it).y;
					*Z = (*it).z;
				}

				MRPT_END

			}

			/** Render
			  */
			void  render() const;

			/** Sets the colors used as extremes when colorFromDepth is enabled. */
			void  setGradientColors( const mrpt::utils::TColorf &colorMin, const mrpt::utils::TColorf &colorMax );

		private:
			/** Constructor */
			CPointCloud();

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CPointCloud() { }

			mutable float m_min, m_max; 	//!< Buffer for min/max coords when m_colorFromDepth is true.
			mutable bool   m_minmax_valid;

			mrpt::utils::TColorf	m_colorFromDepth_min, m_colorFromDepth_max;	//!< The colors used to interpolate when m_colorFromDepth is true.
		};

	} // end namespace

} // End of namespace


#endif

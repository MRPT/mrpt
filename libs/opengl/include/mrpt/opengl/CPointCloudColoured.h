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

#ifndef opengl_CPointCloudColoured_H
#define opengl_CPointCloudColoured_H

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/stl_extensions.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CPointCloudColoured;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CPointCloudColoured, CRenderizable, OPENGL_IMPEXP )


		/** A cloud of points, each one with an individual colour (R,G,B). The alpha component is shared by all the points and is stored in the base member m_color_A.
		  *
		  *  To load from a points-map, see mrpt::slam::CPointsMap::loadIntoPointCloud().
		  *
		  *  \sa opengl::COpenGLScene, opengl::CPointCloud
		  */
		class OPENGL_IMPEXP CPointCloudColoured : public CRenderizable
		{
			DEFINE_SERIALIZABLE( CPointCloudColoured )

		public:
			struct TPointColour
			{
				TPointColour( float _x=0,float _y=0,float _z=0,float _R=0,float _G=0,float _B=0 ) :
					x(_x),y(_y),z(_z),R(_R),G(_G),B(_B)
				{ }
				float x,y,z,R,G,B;	// Float is precission enough for rendering
			};

		private:
			typedef std::vector<TPointColour> TListPointColour;

			TListPointColour	m_points;
			float				m_pointSize; //!< By default is 1.0
			bool				m_pointSmooth; //!< Default: false

			/** Constructor
			  */
			CPointCloudColoured( ) :
				m_points(),
				m_pointSize(1),
				m_pointSmooth(false)
			{
			}
			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CPointCloudColoured() { }

		public:
			typedef TListPointColour::iterator iterator;
			typedef TListPointColour::const_iterator const_iterator;

			inline iterator begin() { return m_points.begin(); }
			inline const_iterator begin() const { return m_points.begin(); }
			inline iterator end() { return m_points.end(); }
			inline const_iterator end() const { return m_points.end(); }

			/** Inserts a new point into the point cloud. */
			inline void push_back(float x,float y,float z, float R, float G, float B) {
				m_points.push_back(TPointColour(x,y,z,R,G,B));
			}

			inline void reserve(size_t N) { m_points.reserve(N); }
			inline void resize(size_t N) { m_points.resize(N); }

			/** Read or write access to each individual point. */
			inline TPointColour &operator [](size_t i) { return m_points[i]; }

			inline size_t size() const { return m_points.size(); }
			inline void clear() { m_points.clear(); }

			inline void setPointSize(float pointSize) { m_pointSize = pointSize; }
			inline float getPointSize() const { return m_pointSize; }

			inline void enablePointSmooth(bool enable=true) { m_pointSmooth=enable; }
			inline void disablePointSmooth() { m_pointSmooth=false; }
			inline bool isPointSmoothEnabled() const { return m_pointSmooth; }

			/** Load the points from a points map (passed as a pointer), depending on the type of point map passed: for the case of a mrpt::slam::CColouredPointMap the colours of individual points will be also copied.
			  *  The possible classes accepted as arguments are: mrpt::slam::CColouredPointsMap, or in general any mrpt::slam::CPointsMap.
			  * \note The method is a template since CPointsMap belongs to a different mrpt library.
			  */
			template <class POINTSMAP>
			void  loadFromPointsMap( const POINTSMAP *m)
			{
				if (m->hasColorPoints())
				{
					size_t N = m->size();
					m_points.resize(N);
					for (size_t i=0;i<N;i++)
					{
						m->getPoint(i,
							m_points[i].x,
							m_points[i].y,
							m_points[i].z,
							m_points[i].R,
							m_points[i].G,
							m_points[i].B );
					}
				}
				else
				{
					// Default colors:
					vector_float	xs,ys,zs;
					m->getAllPoints(xs,ys,zs);

					size_t N = xs.size();
					m_points.resize(N);
					for (size_t i=0;i<N;i++)
					{
						m_points[i].x = xs[i];
						m_points[i].y = ys[i];
						m_points[i].z = zs[i];
						m_points[i].R = m_color_R;
						m_points[i].G = m_color_G;
						m_points[i].B = m_color_B;
					}
				}
			}

			/** Render
			  */
			void  render() const;

		};

		OPENGL_IMPEXP mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,  CPointCloudColoured::TPointColour &o);
		OPENGL_IMPEXP mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out, const CPointCloudColoured::TPointColour &o);

	} // end namespace

	namespace utils
	{
		using namespace mrpt::opengl;

		// Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME(CPointCloudColoured::TPointColour)
	}

} // End of namespace


#endif

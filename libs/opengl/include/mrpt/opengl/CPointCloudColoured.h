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

#ifndef opengl_CPointCloudColoured_H
#define opengl_CPointCloudColoured_H

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/COctreePointRenderer.h>
#include <mrpt/utils/PLY_import_export.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CPointCloudColoured;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CPointCloudColoured, CRenderizable, OPENGL_IMPEXP )


		/** A cloud of points, each one with an individual colour (R,G,B). The alpha component is shared by all the points and is stored in the base member m_color_A.
		  *
		  *  To load from a points-map, CPointCloudColoured::loadFromPointsMap().
		  *
		  *   This class uses smart optimizations while rendering to efficiently draw clouds of millions of points,
		  *   as described in this page: http://www.mrpt.org/Efficiently_rendering_point_clouds_of_millions_of_points
		  *
		  *  \sa opengl::COpenGLScene, opengl::CPointCloud
		  *
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CPointCloudColoured </td> <td> \image html preview_CPointCloudColoured.png </td> </tr>
		  *  </table>
		  *  </div>
		  *
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CPointCloudColoured :
			public CRenderizable,
			public COctreePointRenderer<CPointCloudColoured>,
			public mrpt::utils::PLY_Importer,
			public mrpt::utils::PLY_Exporter
		{
			DEFINE_SERIALIZABLE( CPointCloudColoured )

		public:
			struct TPointColour
			{
				inline TPointColour() { }
				inline TPointColour(float _x,float _y,float _z,float _R,float _G,float _B ) : x(_x),y(_y),z(_z),R(_R),G(_G),B(_B) { }
				float x,y,z,R,G,B;	// Float is precission enough for rendering
			};

		private:
			typedef std::vector<TPointColour> TListPointColour;
			TListPointColour	m_points;

			typedef TListPointColour::iterator iterator;
			typedef TListPointColour::const_iterator const_iterator;
			inline iterator begin() { return m_points.begin(); }
			inline const_iterator begin() const { return m_points.begin(); }
			inline iterator end() { return m_points.end(); }
			inline const_iterator end() const { return m_points.end(); }


			float				m_pointSize; //!< By default is 1.0
			bool				m_pointSmooth; //!< Default: false
			mutable volatile size_t	m_last_rendered_count, m_last_rendered_count_ongoing;

			/** Constructor
			  */
			CPointCloudColoured( ) :
				m_points(),
				m_pointSize(1),
				m_pointSmooth(false),
				m_last_rendered_count(0),
				m_last_rendered_count_ongoing(0)
			{
			}
			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CPointCloudColoured() { }

			void markAllPointsAsNew(); //!< Do needed internal work if all points are new (octree rebuilt,...)

		public:

			/** @name Read/Write of the list of points to render
			    @{ */

			/** Inserts a new point into the point cloud. */
			void push_back(float x,float y,float z, float R, float G, float B);

			/** Set the number of points, with undefined contents */
			inline void resize(size_t N) { m_points.resize(N); }

			/** Like STL std::vector's reserve */
			inline void reserve(size_t N) { m_points.reserve(N); }

			/** Read access to each individual point (checks for "i" in the valid range only in Debug). */
			inline const TPointColour &operator [](size_t i) const {
#ifdef _DEBUG
				ASSERT_BELOW_(i,size())
#endif
				return m_points[i];
			}

			/** Read access to each individual point (checks for "i" in the valid range only in Debug). */
			inline const TPointColour &getPoint(size_t i) const {
#ifdef _DEBUG
				ASSERT_BELOW_(i,size())
#endif
				return m_points[i];
			}

			/** Read access to each individual point (checks for "i" in the valid range only in Debug). */
			inline mrpt::math::TPoint3Df getPointf(size_t i) const {
#ifdef _DEBUG
				ASSERT_BELOW_(i,size())
#endif
				return mrpt::math::TPoint3Df(m_points[i].x,m_points[i].y,m_points[i].z);
			}

			/** Write an individual point (checks for "i" in the valid range only in Debug). */
			void setPoint(size_t i, const TPointColour &p );

			inline size_t size() const { return m_points.size(); } //!< Return the number of points

			inline void clear() { m_points.clear(); markAllPointsAsNew(); }  //!< Erase all the points


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
					const mrpt::utils::TColorf col(m_color);
					for (size_t i=0;i<N;i++)
					{
						m_points[i].x = xs[i];
						m_points[i].y = ys[i];
						m_points[i].z = zs[i];
						m_points[i].R = col.R;
						m_points[i].G = col.G;
						m_points[i].B = col.B;
					}
				}
				markAllPointsAsNew();
			}

			/** Get the number of elements actually rendered in the last render event. */
			size_t getActuallyRendered() const { return m_last_rendered_count; }

			/** @} */


			/** @name Modify the appearance of the rendered points
			    @{ */

			inline void setPointSize(float pointSize) { m_pointSize = pointSize; }
			inline float getPointSize() const { return m_pointSize; }

			inline void enablePointSmooth(bool enable=true) { m_pointSmooth=enable; }
			inline void disablePointSmooth() { m_pointSmooth=false; }
			inline bool isPointSmoothEnabled() const { return m_pointSmooth; }

			/** @} */


			/** Render */
			void  render() const;

			/** Render a subset of points (required by octree renderer) */
			void  render_subset(const bool all, const std::vector<size_t>& idxs, const float render_area_sqpixels ) const;

		protected:
			/** @name PLY Import virtual methods to implement in base classes
			    @{ */
			/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_vertex */
			virtual void PLY_import_set_vertex_count(const size_t N);
			/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_face */
			virtual void PLY_import_set_face_count(const size_t N) {  } 
			/** In a base class, will be called after PLY_import_set_vertex_count() once for each loaded point. 
			  *  \param pt_color Will be NULL if the loaded file does not provide color info.
			  */
			virtual void PLY_import_set_vertex(const size_t idx, const mrpt::math::TPoint3Df &pt, const mrpt::utils::TColorf *pt_color = NULL);
			/** @} */

			/** @name PLY Export virtual methods to implement in base classes
			    @{ */

			/** In a base class, return the number of vertices */
			virtual size_t PLY_export_get_vertex_count() const;

			/** In a base class, return the number of faces */
			virtual size_t PLY_export_get_face_count() const { return 0; }

			/** In a base class, will be called after PLY_export_get_vertex_count() once for each exported point. 
			  *  \param pt_color Will be NULL if the loaded file does not provide color info.
			  */
			virtual void PLY_export_get_vertex(
				const size_t idx, 
				mrpt::math::TPoint3Df &pt, 
				bool &pt_has_color,
				mrpt::utils::TColorf &pt_color) const;

			/** @} */


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

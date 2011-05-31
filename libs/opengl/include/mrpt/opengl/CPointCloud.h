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

#ifndef opengl_CPointCloud_H
#define opengl_CPointCloud_H

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/COctreePointRenderer.h>
#include <mrpt/utils/PLY_import_export.h>

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
		  *  To load from a points-map, CPointCloud::loadFromPointsMap().
		  *
		  *   This class uses smart optimizations while rendering to efficiently draw clouds of millions of points, 
		  *   as described in this page: http://www.mrpt.org/Efficiently_rendering_point_clouds_of_millions_of_points
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
		class OPENGL_IMPEXP CPointCloud :
			public CRenderizable,
			public COctreePointRenderer<CPointCloud>,
			public mrpt::utils::PLY_Importer,
			public mrpt::utils::PLY_Exporter
		{
			DEFINE_SERIALIZABLE( CPointCloud )
		protected:
			enum Axis { None=0, Z, Y, X} m_colorFromDepth;
			std::vector<float>	m_xs,m_ys,m_zs;
			float           m_pointSize; //!< By default is 1.0
			bool			m_pointSmooth; //!< Default: false

			mutable volatile size_t m_last_rendered_count, m_last_rendered_count_ongoing;

			void markAllPointsAsNew(); //!< Do needed internal work if all points are new (octree rebuilt,...)

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


		public:

			/** @name Read/Write of the list of points to render
			    @{ */

			inline size_t size() const { return m_xs.size(); }

			/** Set the number of points (with contents undefined) */
			inline void resize(size_t N) { m_xs.resize(N); m_ys.resize(N); m_zs.resize(N);  }

			/** Like STL std::vector's reserve */
			inline void reserve(size_t N) { m_xs.reserve(N); m_ys.reserve(N); m_zs.reserve(N);  }

			/** Set the list of (X,Y,Z) point coordinates, all at once, from three vectors with their coordinates */
			void setAllPoints(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z)
			{
				m_xs = x;
				m_ys = y;
				m_zs = z;
				markAllPointsAsNew();
			}

			/** Set the list of (X,Y,Z) point coordinates, DESTROYING the contents of the input vectors (via swap) */
			void setAllPointsFast(std::vector<float> &x, std::vector<float> &y, std::vector<float> &z)
			{
				this->clear();
				m_xs.swap(x);
				m_ys.swap(y);
				m_zs.swap(z);
				markAllPointsAsNew();
			}

			inline const std::vector<float> & getArrayX() const {return m_xs;} //!< Get a const reference to the internal array of X coordinates
			inline const std::vector<float> & getArrayY() const {return m_ys;} //!< Get a const reference to the internal array of Y coordinates
			inline const std::vector<float> & getArrayZ() const {return m_zs;} //!< Get a const reference to the internal array of Z coordinates

			void clear();	//!< Empty the list of points.

			/** Adds a new point to the cloud */
			void insertPoint( float x,float y, float z );

			/** Read access to each individual point (checks for "i" in the valid range only in Debug). */
			inline mrpt::math::TPoint3D  operator [](size_t i) const {
#ifdef _DEBUG
				ASSERT_BELOW_(i,size())
#endif
				return mrpt::math::TPoint3D(m_xs[i],m_ys[i],m_zs[i]);
			}

			/** Read access to each individual point (checks for "i" in the valid range only in Debug). */
			inline mrpt::math::TPoint3D getPoint(size_t i) const {
#ifdef _DEBUG
				ASSERT_BELOW_(i,size())
#endif
				return mrpt::math::TPoint3D(m_xs[i],m_ys[i],m_zs[i]);
			}

			/** Read access to each individual point (checks for "i" in the valid range only in Debug). */
			inline mrpt::math::TPoint3Df getPointf(size_t i) const {
#ifdef _DEBUG
				ASSERT_BELOW_(i,size())
#endif
				return mrpt::math::TPoint3Df(m_xs[i],m_ys[i],m_zs[i]);
			}

			/** Write an individual point (checks for "i" in the valid range only in Debug). */
			void setPoint(size_t i, const float x,const float y, const float z);


			/** Load the points from a pointsMap (mrpt::slam::CPointsMap), passed as a pointer.
			  * Note that the method is a template since CPointsMap belongs to a different mrpt library.
			  */
			template <class POINTSMAP>
			inline void  loadFromPointsMap( const POINTSMAP *themap) {
				themap->getAllPoints(m_xs,m_ys,m_zs);
				markAllPointsAsNew();
			}

			/** Load the points from a list of TPoint3D
			  */
			template<class LISTOFPOINTS> void  loadFromPointsList( LISTOFPOINTS &pointsList)
			{
				MRPT_START
				const size_t N = pointsList.size();

				m_xs.resize(N);
				m_ys.resize(N);
				m_zs.resize(N);

				size_t idx;
				typename LISTOFPOINTS::const_iterator it;
				for ( idx=0,it=pointsList.begin() ; idx<N ; ++idx,++it)
				{
					m_xs[idx]=it->x;
					m_ys[idx]=it->y;
					m_zs[idx]=it->z;
				}
				markAllPointsAsNew();
				MRPT_END
			}

			/** Get the number of elements actually rendered in the last render event. */
			size_t getActuallyRendered() const { return m_last_rendered_count; }

			/** @} */


			/** @name Modify the appearance of the rendered points
			    @{ */
			inline void enableColorFromX(bool v=true) { m_colorFromDepth = v ? CPointCloud::X : CPointCloud::None;  }
			inline void enableColorFromY(bool v=true) { m_colorFromDepth = v ? CPointCloud::Y : CPointCloud::None; }
			inline void enableColorFromZ(bool v=true) { m_colorFromDepth = v ? CPointCloud::Z : CPointCloud::None; }

			inline void setPointSize(float p) { m_pointSize=p; }  //!< By default is 1.0
			inline float getPointSize() const { return m_pointSize; }

			inline void enablePointSmooth(bool enable=true) { m_pointSmooth=enable; }
			inline void disablePointSmooth() { m_pointSmooth=false; }
			inline bool isPointSmoothEnabled() const { return m_pointSmooth; }

			/** Sets the colors used as extremes when colorFromDepth is enabled. */
			void  setGradientColors( const mrpt::utils::TColorf &colorMin, const mrpt::utils::TColorf &colorMax );

			/** @} */

			/** Render */
			void  render() const;


			/** Render a subset of points (required by octree renderer) */
			void  render_subset(const bool all, const std::vector<size_t>& idxs, const float render_area_sqpixels ) const;

		private:
			/** Constructor */
			CPointCloud();

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CPointCloud() { }

			mutable float  m_min, m_max,m_max_m_min,m_max_m_min_inv; 	//!< Buffer for min/max coords when m_colorFromDepth is true.
			mutable mrpt::utils::TColorf m_col_slop,m_col_slop_inv; //!< Color linear function slope
			mutable bool   m_minmax_valid;

			mrpt::utils::TColorf	m_colorFromDepth_min, m_colorFromDepth_max;	//!< The colors used to interpolate when m_colorFromDepth is true.

			inline void internal_render_one_point(size_t i) const;
		};

	} // end namespace



} // End of namespace


#endif

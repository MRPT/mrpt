/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef opengl_CPointCloud_H
#define opengl_CPointCloud_H

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/COctreePointRenderer.h>
#include <mrpt/utils/PLY_import_export.h>
#include <mrpt/utils/adapters.h>

namespace mrpt
{
	namespace opengl
	{


		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CPointCloud, CRenderizable, OPENGL_IMPEXP )


		/** A cloud of points, all with the same color or each depending on its value along a particular coordinate axis.
		  *  This class is just an OpenGL representation of a point cloud. For operating with maps of points, see mrpt::maps::CPointsMap and derived classes.
		  *
		  *  To load from a points-map, CPointCloud::loadFromPointsMap().
		  *
		  *   This class uses smart optimizations while rendering to efficiently draw clouds of millions of points,
		  *   as described in this page: http://www.mrpt.org/Efficiently_rendering_point_clouds_of_millions_of_points
		  *
		  *  \sa opengl::CPlanarLaserScan, opengl::COpenGLScene, opengl::CPointCloudColoured, mrpt::maps::CPointsMap
		  *
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CPointCloud </td> <td> \image html preview_CPointCloud.png </td> </tr>
		  *  </table>
		  *  </div>
		  *
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CPointCloud :
			public CRenderizable,
			public COctreePointRenderer<CPointCloud>,
			public mrpt::utils::PLY_Importer,
			public mrpt::utils::PLY_Exporter
		{
			DEFINE_SERIALIZABLE( CPointCloud )
		protected:
			enum Axis { colNone=0, colZ, colY, colX} m_colorFromDepth;
			std::vector<float>	m_xs,m_ys,m_zs;
			float           m_pointSize; //!< By default is 1.0
			bool			m_pointSmooth; //!< Default: false

			mutable volatile size_t m_last_rendered_count, m_last_rendered_count_ongoing;

			void markAllPointsAsNew(); //!< Do needed internal work if all points are new (octree rebuilt,...)

		protected:
			/** @name PLY Import virtual methods to implement in base classes
			    @{ */
			/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_vertex */
			virtual void PLY_import_set_vertex_count(const size_t N) MRPT_OVERRIDE;

			/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_face */
			virtual void PLY_import_set_face_count(const size_t N) MRPT_OVERRIDE {
				MRPT_UNUSED_PARAM(N);
			}

			/** In a base class, will be called after PLY_import_set_vertex_count() once for each loaded point.
			  *  \param pt_color Will be NULL if the loaded file does not provide color info.
			  */
			virtual void PLY_import_set_vertex(const size_t idx, const mrpt::math::TPoint3Df &pt, const mrpt::utils::TColorf *pt_color = NULL) MRPT_OVERRIDE;
			/** @} */

			/** @name PLY Export virtual methods to implement in base classes
			    @{ */
			size_t PLY_export_get_vertex_count() const MRPT_OVERRIDE;
			size_t PLY_export_get_face_count() const MRPT_OVERRIDE { return 0; }
			void   PLY_export_get_vertex(const size_t idx,mrpt::math::TPoint3Df &pt,bool &pt_has_color,mrpt::utils::TColorf &pt_color) const MRPT_OVERRIDE;
			/** @} */

		public:
			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE
			{
				this->octree_getBoundingBox(bb_min, bb_max);
			}

			/** @name Read/Write of the list of points to render
			    @{ */

			inline size_t size() const { return m_xs.size(); }

			/** Set the number of points (with contents undefined) */
			inline void resize(size_t N) { m_xs.resize(N); m_ys.resize(N); m_zs.resize(N); m_minmax_valid = false; markAllPointsAsNew(); }

			/** Like STL std::vector's reserve */
			inline void reserve(size_t N) { m_xs.reserve(N); m_ys.reserve(N); m_zs.reserve(N);  }

			/** Set the list of (X,Y,Z) point coordinates, all at once, from three vectors with their coordinates */
			void setAllPoints(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z)
			{
				m_xs = x;
				m_ys = y;
				m_zs = z;
				m_minmax_valid = false;
				markAllPointsAsNew();
			}

			/** Set the list of (X,Y,Z) point coordinates, DESTROYING the contents of the input vectors (via swap) */
			void setAllPointsFast(std::vector<float> &x, std::vector<float> &y, std::vector<float> &z)
			{
				this->clear();
				m_xs.swap(x);
				m_ys.swap(y);
				m_zs.swap(z);
				m_minmax_valid = false;
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

			/** Write an individual point (without checking validity of the index). */
			inline void setPoint_fast(size_t i, const float x,const float y, const float z)
			{
				m_xs[i] = x;
				m_ys[i] = y;
				m_zs[i] = z;
				m_minmax_valid = false;
				markAllPointsAsNew();
			}


			/** Load the points from any other point map class supported by the adapter mrpt::utils::PointCloudAdapter. */
			template <class POINTSMAP>
			void loadFromPointsMap( const POINTSMAP *themap);
			// Must be implemented at the end of the header.

			/** Load the points from a list of mrpt::math::TPoint3D
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
			inline void enableColorFromX(bool v=true) { m_colorFromDepth = v ? CPointCloud::colX : CPointCloud::colNone;  }
			inline void enableColorFromY(bool v=true) { m_colorFromDepth = v ? CPointCloud::colY : CPointCloud::colNone; }
			inline void enableColorFromZ(bool v=true) { m_colorFromDepth = v ? CPointCloud::colZ : CPointCloud::colNone; }

			inline void setPointSize(float p) { m_pointSize=p; }  //!< By default is 1.0
			inline float getPointSize() const { return m_pointSize; }

			inline void enablePointSmooth(bool enable=true) { m_pointSmooth=enable; }
			inline void disablePointSmooth() { m_pointSmooth=false; }
			inline bool isPointSmoothEnabled() const { return m_pointSmooth; }

			/** Sets the colors used as extremes when colorFromDepth is enabled. */
			void  setGradientColors( const mrpt::utils::TColorf &colorMin, const mrpt::utils::TColorf &colorMax );

			/** @} */

			/** Render */
			void  render() const MRPT_OVERRIDE;

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
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CPointCloud, CRenderizable, OPENGL_IMPEXP )

	} // end namespace


	namespace utils
	{
		/** Specialization mrpt::utils::PointCloudAdapter<mrpt::opengl::CPointCloud> \ingroup mrpt_adapters_grp */
		template <>
		class PointCloudAdapter<mrpt::opengl::CPointCloud> : public detail::PointCloudAdapterHelperNoRGB<mrpt::opengl::CPointCloud,float>
		{
		private:
			mrpt::opengl::CPointCloud &m_obj;
		public:
			typedef float  coords_t;         //!< The type of each point XYZ coordinates
			static const int HAS_RGB   = 0;  //!< Has any color RGB info?
			static const int HAS_RGBf  = 0;  //!< Has native RGB info (as floats)?
			static const int HAS_RGBu8 = 0;  //!< Has native RGB info (as uint8_t)?

			/** Constructor (accept a const ref for convenience) */
			inline PointCloudAdapter(const mrpt::opengl::CPointCloud &obj) : m_obj(*const_cast<mrpt::opengl::CPointCloud*>(&obj)) { }
			/** Get number of points */
			inline size_t size() const { return m_obj.size(); }
			/** Set number of points (to uninitialized values) */
			inline void resize(const size_t N) { m_obj.resize(N); }

			/** Get XYZ coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ(const size_t idx, T &x,T &y, T &z) const {
				x=m_obj.getArrayX()[idx];
				y=m_obj.getArrayY()[idx];
				z=m_obj.getArrayZ()[idx];
			}
			/** Set XYZ coordinates of i'th point */
			inline void setPointXYZ(const size_t idx, const coords_t x,const coords_t y, const coords_t z) {
				m_obj.setPoint_fast(idx,x,y,z);
			}

		}; // end of PointCloudAdapter<mrpt::opengl::CPointCloud>
	}

	namespace opengl
	{
		// After declaring the adapter we can here implement this method:
		template <class POINTSMAP>
		void CPointCloud::loadFromPointsMap( const POINTSMAP *themap)
		{
			ASSERT_(themap!=NULL)
			mrpt::utils::PointCloudAdapter<CPointCloud>     pc_dst(*this);
			const mrpt::utils::PointCloudAdapter<POINTSMAP> pc_src(*themap);
			const size_t N=pc_src.size();
			pc_dst.resize(N);
			for (size_t i=0;i<N;i++)
			{
				float x,y,z;
				pc_src.getPointXYZ(i,x,y,z);
				pc_dst.setPointXYZ(i,x,y,z);
			}
		}
	}

} // End of namespace


#endif

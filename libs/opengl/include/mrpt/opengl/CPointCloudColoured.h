/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef opengl_CPointCloudColoured_H
#define opengl_CPointCloudColoured_H

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/COctreePointRenderer.h>
#include <mrpt/utils/PLY_import_export.h>
#include <mrpt/utils/adapters.h>
#include <mrpt/utils/color_maps.h>

namespace mrpt
{
	namespace opengl
	{
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

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE
			{
				this->octree_getBoundingBox(bb_min, bb_max);
			}

			/** @name Read/Write of the list of points to render
			    @{ */

			/** Inserts a new point into the point cloud. */
			void push_back(float x,float y,float z, float R, float G, float B);

			/** Set the number of points, with undefined contents */
			inline void resize(size_t N) { m_points.resize(N); markAllPointsAsNew(); }

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

			/** Like \a setPoint() but does not check for index out of bounds */
			inline void setPoint_fast(const size_t i, const TPointColour &p ) {
				m_points[i] = p;
				markAllPointsAsNew();
			}

			/** Like \a setPoint() but does not check for index out of bounds */
			inline void setPoint_fast(const size_t i, const float x,const float y, const float z ) {
			 TPointColour &p = m_points[i];
				p.x=x; p.y=y; p.z=z;
				markAllPointsAsNew();
			}

			/** Like \c setPointColor but without checking for out-of-index erors */
			inline void setPointColor_fast(size_t index,float R, float G, float B)
			{
				m_points[index].R=R;
				m_points[index].G=G;
				m_points[index].B=B;
			}
			/** Like \c getPointColor but without checking for out-of-index erors */
			inline void getPointColor_fast( size_t index, float &R, float &G, float &B ) const
			{
				R = m_points[index].R;
				G = m_points[index].G;
				B = m_points[index].B;
			}

			inline size_t size() const { return m_points.size(); } //!< Return the number of points

			inline void clear() { m_points.clear(); markAllPointsAsNew(); }  //!< Erase all the points

			/** Load the points from any other point map class supported by the adapter mrpt::utils::PointCloudAdapter. */
			template <class POINTSMAP>
			void  loadFromPointsMap( const POINTSMAP *themap);
			// Must be implemented at the end of the header.

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

			/** Regenerates the color of each point according the one coordinate (coord_index:0,1,2 for X,Y,Z) and the given color map. */
			void recolorizeByCoordinate(const float coord_min, const float coord_max, const int coord_index = 2, const mrpt::utils::TColormap color_map = mrpt::utils::cmJET );
			/** @} */

			/** Render */
			void  render() const MRPT_OVERRIDE;

			/** Render a subset of points (required by octree renderer) */
			void  render_subset(const bool all, const std::vector<size_t>& idxs, const float render_area_sqpixels ) const;

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
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CPointCloudColoured, CRenderizable, OPENGL_IMPEXP )

		OPENGL_IMPEXP mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,  CPointCloudColoured::TPointColour &o);
		OPENGL_IMPEXP mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out, const CPointCloudColoured::TPointColour &o);

	} // end namespace

	namespace utils
	{
		// Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME_NAMESPACE(CPointCloudColoured::TPointColour, mrpt::opengl)
	}

	namespace utils
	{
		/** Specialization mrpt::utils::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>  \ingroup mrpt_adapters_grp*/
		template <>
		class PointCloudAdapter<mrpt::opengl::CPointCloudColoured>
		{
		private:
			mrpt::opengl::CPointCloudColoured &m_obj;
		public:
			typedef float  coords_t;  //!< The type of each point XYZ coordinates
			static const int HAS_RGB   = 1;  //!< Has any color RGB info?
			static const int HAS_RGBf  = 1;  //!< Has native RGB info (as floats)?
			static const int HAS_RGBu8 = 0;  //!< Has native RGB info (as uint8_t)?

			/** Constructor (accept a const ref for convenience) */
			inline PointCloudAdapter(const mrpt::opengl::CPointCloudColoured &obj) : m_obj(*const_cast<mrpt::opengl::CPointCloudColoured*>(&obj)) { }
			/** Get number of points */
			inline size_t size() const { return m_obj.size(); }
			/** Set number of points (to uninitialized values) */
			inline void resize(const size_t N) { m_obj.resize(N); }

			/** Get XYZ coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ(const size_t idx, T &x,T &y, T &z) const {
				const mrpt::opengl::CPointCloudColoured::TPointColour &pc = m_obj[idx];
				x=pc.x;
				y=pc.y;
				z=pc.z;
			}
			/** Set XYZ coordinates of i'th point */
			inline void setPointXYZ(const size_t idx, const coords_t x,const coords_t y, const coords_t z) {
				m_obj.setPoint_fast(idx, x,y,z);
			}

			/** Get XYZ_RGBf coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ_RGBf(const size_t idx, T &x,T &y, T &z, float &r,float &g,float &b) const {
				const mrpt::opengl::CPointCloudColoured::TPointColour &pc = m_obj[idx];
				x=pc.x; y=pc.y; z=pc.z;
				r=pc.R; g=pc.G; b=pc.B;
			}
			/** Set XYZ_RGBf coordinates of i'th point */
			inline void setPointXYZ_RGBf(const size_t idx, const coords_t x,const coords_t y, const coords_t z, const float r,const float g,const float b) {
				m_obj.setPoint_fast(idx, mrpt::opengl::CPointCloudColoured::TPointColour(x,y,z,r,g,b) );
			}

			/** Get XYZ_RGBu8 coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ_RGBu8(const size_t idx, T &x,T &y, T &z, uint8_t &r,uint8_t &g,uint8_t &b) const {
				const mrpt::opengl::CPointCloudColoured::TPointColour &pc = m_obj[idx];
				x=pc.x; y=pc.y; z=pc.z;
				r=pc.R*255; g=pc.G*255; b=pc.B*255;
			}
			/** Set XYZ_RGBu8 coordinates of i'th point */
			inline void setPointXYZ_RGBu8(const size_t idx, const coords_t x,const coords_t y, const coords_t z, const uint8_t r,const uint8_t g,const uint8_t b) {
				m_obj.setPoint_fast(idx, mrpt::opengl::CPointCloudColoured::TPointColour(x,y,z,r/255.f,g/255.f,b/255.f) );
			}

			/** Get RGBf color of i'th point */
			inline void getPointRGBf(const size_t idx, float &r,float &g,float &b) const { m_obj.getPointColor_fast(idx,r,g,b); }
			/** Set XYZ_RGBf coordinates of i'th point */
			inline void setPointRGBf(const size_t idx, const float r,const float g,const float b) { m_obj.setPointColor_fast(idx,r,g,b); }

			/** Get RGBu8 color of i'th point */
			inline void getPointRGBu8(const size_t idx, uint8_t &r,uint8_t &g,uint8_t &b) const {
				float R,G,B;
				m_obj.getPointColor_fast(idx,R,G,B);
				r=R*255; g=G*255; b=B*255;
			}
			/** Set RGBu8 coordinates of i'th point */
			inline void setPointRGBu8(const size_t idx,const uint8_t r,const uint8_t g,const uint8_t b) {
				m_obj.setPointColor_fast(idx,r/255.f,g/255.f,b/255.f);
			}

		}; // end of PointCloudAdapter<mrpt::opengl::CPointCloudColoured>
	}
	namespace opengl
	{
		// After declaring the adapter we can here implement this method:
		template <class POINTSMAP>
		void  CPointCloudColoured::loadFromPointsMap( const POINTSMAP *themap)
		{
			mrpt::utils::PointCloudAdapter<CPointCloudColoured> pc_dst(*this);
			const mrpt::utils::PointCloudAdapter<POINTSMAP>     pc_src(*themap);
			const size_t N=pc_src.size();
			pc_dst.resize(N);
			for (size_t i=0;i<N;i++)
			{
				float x,y,z,r,g,b;
				pc_src.getPointXYZ_RGBf(i,x,y,z,r,g,b);
				pc_dst.setPointXYZ_RGBf(i,x,y,z,r,g,b);
			}
		}
	}
} // End of namespace

#endif

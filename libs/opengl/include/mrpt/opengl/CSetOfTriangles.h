/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
#ifndef opengl_CSetOfTriangles_H
#define opengl_CSetOfTriangles_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/geometry.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CSetOfTriangles;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSetOfTriangles, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A set of colored triangles.
		  *  This class can be used to draw any solid, arbitrarily complex object (without textures).
		  *  \sa opengl::COpenGLScene, CSetOfTexturedTriangles
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CSetOfTriangles : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CSetOfTriangles )
		public:
			/**
			  * Triangle definition. Each vertex has three spatial coordinates and four color values.
			  */
			struct OPENGL_IMPEXP TTriangle
			{
				inline TTriangle() {  }
				inline TTriangle(const mrpt::math::TPolygon3D &p)  {
					ASSERT_(p.size()==3)
					for (size_t i=0;i<3;i++) {
						x[i]=p[i].x; y[i]=p[i].y; z[i]=p[i].z; r[i]=g[i]=b[i]=a[i]=1; }
				}
				float	x[3],y[3],z[3];
				float	r[3],g[3],b[3],a[3];
			};
			/**
			  * Const iterator type.
			  */
			typedef std::vector<TTriangle>::const_iterator const_iterator;
			/**
			  * Const reverse iterator type.
			  */
			typedef std::vector<TTriangle>::const_reverse_iterator const_reverse_iterator;
		protected:
			/**
			  * List of triangles.
			  * \sa TTriangle
			  */
			std::vector<TTriangle>		m_triangles;
			/**
			  * Transparency enabling.
			  */
			bool						m_enableTransparency;
			/**
			  * Mutable variable used to check whether polygons need to be recalculated.
			  */
			mutable bool polygonsUpToDate;
			/**
			  * Polygon cache.
			  */
			mutable std::vector<mrpt::math::TPolygonWithPlane> tmpPolygons;
		public:
			/**
			  * Polygon cache updating.
			  */
			void updatePolygons() const;
			/**
			  * Clear this object.
			  */
			inline void clearTriangles() { m_triangles.clear();polygonsUpToDate=false; CRenderizableDisplayList::notifyChange(); }
			/**
			  * Get triangle count.
			  */
			inline size_t getTrianglesCount() const { return m_triangles.size(); }
			/**
			  * Gets the triangle in a given position.
			  */
			inline void getTriangle(size_t idx, TTriangle &t) const { ASSERT_(idx<m_triangles.size()); t=m_triangles[idx]; }
			/**
			  * Inserts a triangle into the set.
			  */
			inline void insertTriangle( const TTriangle &t ) { m_triangles.push_back(t);polygonsUpToDate=false; CRenderizableDisplayList::notifyChange(); }
			/**
			  * Inserts a set of triangles, bounded by iterators, into this set.
			  * \sa insertTriangle
			  */
			template<class InputIterator> inline void insertTriangles(const InputIterator &begin,const InputIterator &end)	{
				m_triangles.insert(m_triangles.end(),begin,end);
				polygonsUpToDate=false;
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Inserts an existing CSetOfTriangles into this one.
			  */
			inline void insertTriangles(const CSetOfTrianglesPtr &p)	{
				reserve(m_triangles.size()+p->m_triangles.size());
				m_triangles.insert(m_triangles.end(),p->m_triangles.begin(),p->m_triangles.end());
				polygonsUpToDate=false;
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Reserves memory for certain number of triangles, avoiding multiple memory allocation calls.
			  */
			inline void reserve(size_t t)	{
				m_triangles.reserve(t);
				CRenderizableDisplayList::notifyChange();
			}

			/** Enables or disables transparency. */
			inline void enableTransparency( bool v )	{ m_enableTransparency = v; CRenderizableDisplayList::notifyChange(); }

			virtual CRenderizable& setColor_u8(const mrpt::utils::TColor &c);
			virtual CRenderizable& setColorR_u8(const uint8_t r);
			virtual CRenderizable& setColorG_u8(const uint8_t g);
			virtual CRenderizable& setColorB_u8(const uint8_t b);
			virtual CRenderizable& setColorA_u8(const uint8_t a);

			/** Render
			  */
			void  render_dl() const;

			/** Ray tracing
			  */
			virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const;

			/**
			  * Gets the polygon cache.
			  * \sa insertTriangles
			  */
			void getPolygons(std::vector<mrpt::math::TPolygon3D> &polys) const;

			/**
			  * Inserts a set of triangles, given in a container of either TTriangle's or TPolygon3D
			  * \sa insertTriangle
			  */
			template<class CONTAINER>
			inline void insertTriangles(const CONTAINER &c)	 {
				this->insertTriangles(c.begin(),c.end());
				CRenderizableDisplayList::notifyChange();
			}

			/**
			  * Gets the beginning iterator to this object.
			  */
			inline const_iterator begin() const	{
				return m_triangles.begin();
			}
			/**
			  * Gets the ending iterator to this object.
			  */
			inline const_iterator end() const	{
				return m_triangles.end();
			}
			/**
			  * Gets the reverse beginning iterator to this object, which points to the last triangle.
			  */
			inline const_reverse_iterator rbegin() const	{
				return m_triangles.rbegin();
			}
			/**
			  * Gets the reverse ending iterator to this object, which points to the beginning of the actual set.
			  */
			inline const_reverse_iterator rend() const	{
				return m_triangles.rend();
			}

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

		private:
			/** Constructor
			  */
			CSetOfTriangles( bool enableTransparency = false ) :
				m_triangles(),
				m_enableTransparency(enableTransparency),
				polygonsUpToDate(false)
			{
			}

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CSetOfTriangles() { }
		};
		/** Inserts a set of triangles into the list; note that this method allows to pass another CSetOfTriangles as argument. Allows call chaining.
		  * \sa mrpt::opengl::CSetOfTriangles::insertTriangle
		  */
		template<class T> inline CSetOfTrianglesPtr &operator<<(CSetOfTrianglesPtr &s,const T &t)	{
			s->insertTriangles(t.begin(),t.end());
			return s;
		}
		/** Inserts a triangle into the list. Allows call chaining.
		  * \sa mrpt::opengl::CSetOfTriangles::insertTriangle
		  */
		template<> inline CSetOfTrianglesPtr &operator<<(CSetOfTrianglesPtr &s,const CSetOfTriangles::TTriangle &t)	{
			s->insertTriangle(t);
			return s;
		}

	} // end namespace

} // End of namespace


#endif

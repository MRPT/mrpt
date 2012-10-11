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
#ifndef opengl_CSetOfTexturedTriangles_H
#define opengl_CSetOfTexturedTriangles_H

#include <mrpt/opengl/CTexturedObject.h>

namespace mrpt
{
	namespace utils { class CStream; }

	namespace opengl
	{
		using mrpt::utils::CStream;

		class OPENGL_IMPEXP CSetOfTexturedTriangles;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSetOfTexturedTriangles, CTexturedObject, OPENGL_IMPEXP )

		/** A set of textured triangles.
		  *  This class can be used to draw any solid, arbitrarily complex object with textures.
		  *  \sa opengl::COpenGLScene
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CSetOfTexturedTriangles : public CTexturedObject
		{
			DEFINE_SERIALIZABLE( CSetOfTexturedTriangles )

		public:
			/** Triangle vertex. This structure encapsulates the vertex coordinates and the image pixels.
			  */
			struct OPENGL_IMPEXP TVertex
			{
				/** Default constructor. */
				TVertex( ) :
					m_x(0.0), m_y(0.0), m_z(0.0), m_u(0), m_v(0) { }
				/** Constructor. */
				TVertex(float x, float y, float z, uint32_t u, uint32_t v) :
					m_x(x), m_y(y), m_z(z), m_u(u), m_v(v) { }
				/** 3D vertex coordinates.  */
				float			m_x, m_y, m_z;
				/** 2D texture coordinates. Notice that the texture coordinates are 2D pixels!!! */
				uint32_t	m_u, m_v;

				void writeToStream(CStream &out) const { out << m_x << m_y << m_z  << m_u << m_v; }
				void readFromStream(CStream &in) { in >> m_x >> m_y >> m_z >> m_u >> m_v; }
			};

			/** Triangle. This structure encapsulates the triangle vertices.
			  */
			struct OPENGL_IMPEXP TTriangle
			{
			/** Default constructor.  */
				TTriangle( ) :
					m_v1(), m_v2(), m_v3() { }
			/** Constructor. */
				TTriangle(TVertex v1, TVertex v2, TVertex v3) :
					m_v1(v1), m_v2(v2), m_v3(v3) { }
			/** Vertices.  */
				TVertex	m_v1, m_v2, m_v3;

				void writeToStream(CStream &out) const {  m_v1.writeToStream(out); m_v2.writeToStream(out); m_v3.writeToStream(out); }
				void readFromStream(CStream &in) { m_v1.readFromStream(in); m_v2.readFromStream(in);  m_v3.readFromStream(in); }
			};

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

		protected:
			/** Triangle array. */
			std::vector<TTriangle>	m_triangles;

			/** Render */
			void  render_texturedobj( ) const;

		public:
			void clearTriangles( ) { m_triangles.clear(); CRenderizableDisplayList::notifyChange(); }
			size_t getTrianglesCount( ) const { return m_triangles.size(); }
			const TTriangle & getTriangle( size_t idx) const { ASSERT_(idx<m_triangles.size()); return m_triangles[idx];  }
			void getTriangle( size_t idx, TTriangle &t ) const { ASSERT_(idx<m_triangles.size()); t = m_triangles[idx]; CRenderizableDisplayList::notifyChange(); }
			void insertTriangle( const TTriangle &t ) { m_triangles.push_back(t); CRenderizableDisplayList::notifyChange(); }


			/** Ray Trace
			  */
			virtual bool traceRay( const mrpt::poses::CPose3D &o,double &dist ) const;

		private:
			/** Constructor
			  */
			CSetOfTexturedTriangles( ) : m_triangles()
			{ }

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CSetOfTexturedTriangles();
		};

	} // end namespace

} // End of namespace

#endif

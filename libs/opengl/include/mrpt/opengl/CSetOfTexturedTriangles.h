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

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CSetOfTexturedTriangles_H
#define opengl_CSetOfTexturedTriangles_H

#include <mrpt/opengl/CTexturedObject.h>

namespace mrpt
{
	namespace utils { class CStream; }

	namespace opengl
	{
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
				TVertex( ); //!< Default constructor			
				TVertex(float x, float y, float z, uint32_t u, uint32_t v);
				float m_x, m_y, m_z; //!< 3D vertex coordinates.
				uint32_t m_u, m_v; //!< 2D texture coordinates. Notice that the texture coordinates are 2D pixels!!!
				void writeToStream(mrpt::utils::CStream &out) const;
				void readFromStream(mrpt::utils::CStream &in);
			};

			/** Triangle. This structure encapsulates the triangle vertices.
			  */
			struct OPENGL_IMPEXP TTriangle
			{
				TTriangle( ); //!< Default constructor
				TTriangle(TVertex v1, TVertex v2, TVertex v3);
				TVertex	m_v1, m_v2, m_v3; //!< vertices
				void writeToStream(mrpt::utils::CStream &out) const;
				void readFromStream(mrpt::utils::CStream &in);
			};

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

		protected:
			/** Triangle array. */
			std::vector<TTriangle>	m_triangles;

			void  render_texturedobj( ) const MRPT_OVERRIDE;

		public:
			void clearTriangles( ) { m_triangles.clear(); CRenderizableDisplayList::notifyChange(); }
			size_t getTrianglesCount( ) const { return m_triangles.size(); }
			const TTriangle & getTriangle( size_t idx) const { ASSERT_(idx<m_triangles.size()); return m_triangles[idx];  }
			void getTriangle( size_t idx, TTriangle &t ) const { ASSERT_(idx<m_triangles.size()); t = m_triangles[idx]; CRenderizableDisplayList::notifyChange(); }
			void insertTriangle( const TTriangle &t ) { m_triangles.push_back(t); CRenderizableDisplayList::notifyChange(); }


			virtual bool traceRay( const mrpt::poses::CPose3D &o,double &dist ) const MRPT_OVERRIDE;

		private:
			/** Constructor
			  */
			CSetOfTexturedTriangles( ) : m_triangles()
			{ }

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CSetOfTexturedTriangles();
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CSetOfTexturedTriangles, CTexturedObject, OPENGL_IMPEXP )

	} // end namespace

} // End of namespace

#endif

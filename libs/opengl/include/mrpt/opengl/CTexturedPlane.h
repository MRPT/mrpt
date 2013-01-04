/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#ifndef opengl_CTexturedPlane_H
#define opengl_CTexturedPlane_H

#include <mrpt/opengl/CTexturedObject.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CTexturedPlane;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CTexturedPlane, CTexturedObject, OPENGL_IMPEXP )

		/** A 2D plane in the XY plane with a texture image.
		  *  \sa opengl::COpenGLScene
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CTexturedPlane : public CTexturedObject
		{
			DEFINE_SERIALIZABLE( CTexturedPlane )
		protected:
			mutable float				m_tex_x_min,m_tex_x_max;
			mutable float				m_tex_y_min,m_tex_y_max;

			float			m_xMin, m_xMax;
			float			m_yMin, m_yMax;

			mutable bool polygonUpToDate;
			mutable std::vector<mrpt::math::TPolygonWithPlane> tmpPoly;   //!< Used for ray-tracing
			void updatePoly() const;
			void unloadTexture();

			void  render_texturedobj() const;

		public:
			/** Set the texture coordinates of the four corners (in the range 0-1). */
			void setTextureCornerCoords( float tex_x_min, float tex_x_max, float tex_y_min, float tex_y_max)
			{
				m_tex_x_min=tex_x_min;
				m_tex_x_max=tex_x_max;
				m_tex_y_min=tex_y_min;
				m_tex_y_max=tex_y_max;
				CRenderizableDisplayList::notifyChange();
			}

			/** Set the coordinates of the four corners that define the plane on the XY plane. */
			void setPlaneCorners(float xMin, float xMax, float yMin, float yMax)
			{
				m_xMin = xMin; m_xMax = xMax;
				m_yMin = yMin; m_yMax = yMax;
				polygonUpToDate=false;
				CRenderizableDisplayList::notifyChange();
			}

			/** Get the coordinates of the four corners that define the plane on the XY plane. */
			inline void getPlaneCorners(float &xMin, float &xMax, float &yMin, float &yMax) const
			{
				xMin = m_xMin; xMax = m_xMax;
				yMin = m_yMin; yMax = m_yMax;
			}

			/** Class factory  */
			static CTexturedPlanePtr Create(
				float				x_min,
				float				x_max,
				float				y_min,
				float				y_max)
			{
				return CTexturedPlanePtr( new CTexturedPlane(x_min, x_max, y_min, y_max) );
			}

			/** Ray trace
			  */
			virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

		private:
			/** Constructor
			  */
			CTexturedPlane(
				float				x_min = -1,
				float				x_max = 1,
				float				y_min = -1,
				float				y_max = 1
				);

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CTexturedPlane();
		};

	} // end namespace

} // End of namespace


#endif

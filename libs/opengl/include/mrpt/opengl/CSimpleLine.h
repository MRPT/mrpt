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
#ifndef opengl_CSimpleLine_H
#define opengl_CSimpleLine_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CSimpleLine;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSimpleLine, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A line segment
		  *  \sa opengl::COpenGLScene
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CSimpleLine : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CSimpleLine )

		protected:
			float	m_x0,m_y0,m_z0;
			float	m_x1,m_y1,m_z1;
            float	m_lineWidth;
		public:
			void setLineWidth(float w) { m_lineWidth=w; CRenderizableDisplayList::notifyChange(); }
			float getLineWidth() const { return  m_lineWidth;}

			void setLineCoords(float x0,float y0,float z0, float x1, float y1, float z1)
			{
				m_x0=x0; m_y0=y0; m_z0=z0;
				m_x1=x1; m_y1=y1; m_z1=z1;
				CRenderizableDisplayList::notifyChange();
			}

			void getLineCoords(float &x0,float &y0,float &z0, float &x1, float &y1, float &z1) const
			{
				x0=m_x0; y0=m_y0; z0=m_z0;
				x1=m_x1; y1=m_y1; z1=m_z1;
			}

			/** Render
			  */
			void  render_dl() const;

			/** Class factory */
			static CSimpleLinePtr Create(
				float x0,float y0, float z0,
				float x1,float y1, float z1, float lineWidth = 1 )
			{
				return CSimpleLinePtr(new CSimpleLine(x0,y0,z0,x1,y1,z1,lineWidth));
			}

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

		private:
			/** Constructor
			  */
			CSimpleLine(
				float x0=0,float y0=0, float z0=0,
				float x1=0,float y1=0, float z1=0, float lineWidth = 1 ) :
					m_x0(x0),m_y0(y0),m_z0(z0),
					m_x1(x1),m_y1(y1),m_z1(z1),
					m_lineWidth(lineWidth)
			{
			}

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CSimpleLine() { }
		};

	} // end namespace

} // End of namespace


#endif

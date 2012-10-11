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
#ifndef opengl_CDisk_H
#define opengl_CDisk_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/geometry.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CDisk;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CDisk, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A planar disk in the XY plane.
		  *  \sa opengl::COpenGLScene
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CDisk </td> <td> \image html preview_CDisk.png </td> </tr>
		  *  </table>
		  *  </div>
		  *  
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CDisk : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CDisk )

		protected:
			float		m_radiusIn,m_radiusOut;
			uint32_t	m_nSlices, m_nLoops;

		public:
			void setDiskRadius(float outRadius, float inRadius=0) { m_radiusIn=inRadius; m_radiusOut=outRadius; CRenderizableDisplayList::notifyChange(); }

			float getInRadius() const { return m_radiusIn; }
			float getOutRadius() const { return m_radiusOut; }

			void setSlicesCount(uint32_t N) { m_nSlices=N; CRenderizableDisplayList::notifyChange(); }  //!< Default=50
			void setLoopsCount(uint32_t N) { m_nLoops=N; CRenderizableDisplayList::notifyChange(); }  //!< Default=4


			/** Render
			  */
			void  render_dl() const;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

			/** Ray tracing
			  */
			virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const;

			static CDiskPtr Create(float radiusOut,float radiusIn,uint32_t slices=50,uint32_t loops=4)	{
				return CDiskPtr(new CDisk(radiusOut,radiusIn,slices,loops));
			}

		private:
			/** Constructor
			  */
			CDisk():m_radiusIn(0),m_radiusOut(1),m_nSlices(50),m_nLoops(4) {}

			CDisk(float rOut,float rIn,uint32_t slices,uint32_t loops):m_radiusIn(rIn),m_radiusOut(rOut),m_nSlices(slices),m_nLoops(loops)	{}

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CDisk() { }
		};

	} // end namespace

} // End of namespace


#endif

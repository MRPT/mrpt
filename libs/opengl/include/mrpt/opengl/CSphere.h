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
#ifndef opengl_CSphere_H
#define opengl_CSphere_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CSphere;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSphere, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A solid or wire-frame sphere.
		  *  \sa opengl::COpenGLScene
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CSphere </td> <td> \image html preview_CSphere.png </td> </tr>
		  *  </table>
		  *  </div>
		  *  
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CSphere : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CSphere )

		protected:
			float			m_radius;
			int				m_nDivsLongitude,m_nDivsLatitude;
			bool			m_keepRadiusIndependentEyeDistance;

		public:
			void setRadius(float r) { m_radius=r; CRenderizableDisplayList::notifyChange(); }
			float getRadius() const {return m_radius; }

			void setNumberDivsLongitude(int N) { m_nDivsLongitude=N; CRenderizableDisplayList::notifyChange(); }
			void setNumberDivsLatitude(int N) { m_nDivsLatitude=N;  CRenderizableDisplayList::notifyChange();}
			void enableRadiusIndependentOfEyeDistance(bool v=true)  { m_keepRadiusIndependentEyeDistance=v; CRenderizableDisplayList::notifyChange(); }

			/** \sa CRenderizableDisplayList */
			virtual bool should_skip_display_list_cache() const { return m_keepRadiusIndependentEyeDistance; }

			/** Class factory  */
			static CSpherePtr Create(
				float				radius,
				int					nDivsLongitude = 20,
				int					nDivsLatitude = 20 )
			{
				return CSpherePtr( new CSphere(radius,nDivsLongitude,nDivsLatitude) );
			}

			/** Render */
			void  render_dl() const;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

			/** Ray tracing
			  */
			virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const;

		private:
			/** Constructor
			  */
			CSphere(
				float				radius = 1.0f,
				int					nDivsLongitude = 20,
				int					nDivsLatitude = 20
				) :
				m_radius(radius),
				m_nDivsLongitude(nDivsLongitude),
				m_nDivsLatitude(nDivsLatitude),
				m_keepRadiusIndependentEyeDistance(false)
			{
			}

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CSphere() { }
		};

	} // end namespace

} // End of namespace

#endif

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CSphere_H
#define opengl_CSphere_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{


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
			bool should_skip_display_list_cache() const  MRPT_OVERRIDE { return m_keepRadiusIndependentEyeDistance; }

			/** Class factory  */
			static CSpherePtr Create(
				float				radius,
				int					nDivsLongitude = 20,
				int					nDivsLatitude = 20 );

			/** Render */
			void  render_dl() const MRPT_OVERRIDE;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

			/** Ray tracing
			  */
			bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const MRPT_OVERRIDE;

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
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CSphere, CRenderizableDisplayList, OPENGL_IMPEXP )

	} // end namespace

} // End of namespace

#endif

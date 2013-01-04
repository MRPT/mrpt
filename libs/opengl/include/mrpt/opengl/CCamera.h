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

#ifndef opengl_CCamera_H
#define opengl_CCamera_H

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/poses/CPoseOrPoint.h>

namespace mrpt
{
	namespace opengl
	{
		class COpenGLViewport;
		class CCamera;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CCamera, CRenderizable, OPENGL_IMPEXP )

		/** A camera: if added to a scene, the viewpoint defined by this camera will be used instead of the camera parameters set in COpenGLViewport::m_camera.
		  *  A camera can be defined to provide a projective or orthogonal view of the world by setting the member CCamera::m_projectiveModel.
		  *  \sa opengl::COpenGLScene
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CCamera : public CRenderizable
		{
			friend class COpenGLViewport;

			DEFINE_SERIALIZABLE( CCamera )
		protected:

			float	m_pointingX,m_pointingY,m_pointingZ;
			float 	m_distanceZoom;
			float	m_azimuthDeg,m_elevationDeg;

			bool	m_projectiveModel;	//!< If set to true (default), camera model is projective, otherwise, it's orthogonal.
			float	m_projectiveFOVdeg;	//!< Field-of-View in degs, only when projectiveModel=true (default=30 deg).
			bool	m_6DOFMode; //!< If set to true, camera pose is used when rendering the viewport


		public:
			void setPointingAt(float x,float y, float z) { m_pointingX=x; m_pointingY=y; m_pointingZ=z; }

			template <class POSEORPOINT>
			void setPointingAt(const POSEORPOINT &p)
			{
				m_pointingX=p.x();
				m_pointingY=p.y();
				m_pointingZ=p.is3DPoseOrPoint() ? p.m_coords[2] : 0;
			}
			inline void setPointingAt(const mrpt::math::TPoint3D &p) { setPointingAt(p.x,p.y,p.z); }


			float getPointingAtX() const { return m_pointingX; }
			float getPointingAtY() const { return m_pointingY; }
			float getPointingAtZ() const { return m_pointingZ; }

			void setZoomDistance(float z) { m_distanceZoom=z; }
			float getZoomDistance() const { return m_distanceZoom; }

			float getAzimuthDegrees() const { return m_azimuthDeg; }
			float getElevationDegrees() const { return m_elevationDeg; }

			void setAzimuthDegrees(float ang) { m_azimuthDeg=ang; }
			void setElevationDegrees(float ang) { m_elevationDeg=ang; }

			void setProjectiveModel(bool v=true) { m_projectiveModel=v; } //!< Enable/Disable projective mode (vs. orthogonal)
			void setOrthogonal(bool v=true) { m_projectiveModel=!v; }     //!< Enable/Disable orthogonal mode (vs. projective)
			
			/** Set 6DOFMode, if enabled camera is set according to its pose (default=false).
			 *  Conventionally, eye is set looking towards the positive direction of Z axis.
			 *  Up is set as the Y axis.
			 *  In this mode azimuth/elevation are ignored.
			 **/			
			void set6DOFMode(bool v) { m_6DOFMode=v; }    
			
			bool isProjective() const { return m_projectiveModel; }
			bool isOrthogonal() const { return !m_projectiveModel; }
			bool is6DOFMode() const { return m_6DOFMode; }

			void setProjectiveFOVdeg(float ang) { m_projectiveFOVdeg=ang; }  //!< Field-of-View in degs, only when projectiveModel=true (default=30 deg).
			float getProjectiveFOVdeg() const { return m_projectiveFOVdeg; }  //!< Field-of-View in degs, only when projectiveModel=true (default=30 deg).


			/** Render does nothing here. */
			void  render()  const {  }

			/** In this class, returns a fixed box (max,max,max), (-max,-max,-max). */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

		private:
			/** Constructor
			  */
			CCamera();

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CCamera() { }

		};

	} // end namespace opengl

} // End of namespace


#endif

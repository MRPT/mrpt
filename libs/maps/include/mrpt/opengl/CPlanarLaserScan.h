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

#ifndef opengl_CPlanarLaserScan_H
#define opengl_CPlanarLaserScan_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

#include <mrpt/slam/CMetricMap.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CSimplePointsMap.h>


namespace mrpt
{
	/** \ingroup mrpt_maps_grp */
	namespace opengl
	{
		class CPlanarLaserScan;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CPlanarLaserScan, CRenderizableDisplayList, MAPS_IMPEXP )

		/** This object renders a 2D laser scan by means of three elements: the points, the line along end-points and the 2D scanned surface.
		  *
		  *  By default, all those three elements are drawn, but you can individually switch them on/off with:
		  *    - CPlanarLaserScan::enablePoints()
		  *    - CPlanarLaserScan::enableLine()
		  *    - CPlanarLaserScan::enableSurface()
		  *
		  *  To change the final result, more methods allow further customization of the 3D object (color of each element, etc.).
		  *
		  *  The scan is passed or updated through CPlanarLaserScan::setScan()
		  *
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CPlanarLaserScan </td> <td> \image html preview_CPlanarLaserScan.png </td> </tr>
		  *  </table>
		  *  </div>
		  *
		  *  \note The laser points are projected at the sensor pose as given in the "scan" object, so this CPlanarLaserScan object should be placed at the exact pose of the robot coordinates origin.
		  *
		  *  \sa mrpt::opengl::CPointCloud, opengl::COpenGLScene
	  	  * \ingroup mrpt_maps_grp
		  */
		class MAPS_IMPEXP CPlanarLaserScan : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CPlanarLaserScan )
		protected:
			mrpt::slam::CObservation2DRangeScan	m_scan;
			mutable mrpt::slam::CSimplePointsMap		m_cache_points;
			mutable bool	m_cache_valid;


            float	m_line_width;
            float	m_line_R,m_line_G,m_line_B,m_line_A;

            float	m_points_width;
            float	m_points_R,m_points_G,m_points_B,m_points_A;

            float	m_plane_R,m_plane_G,m_plane_B,m_plane_A;

			bool	m_enable_points;
			bool	m_enable_line;
			bool	m_enable_surface;

		public:
			void clear();	//!<< Clear the scan

			/** Show or hides the scanned points \sa sePointsWidth, setPointsColor*/
			inline void enablePoints(bool enable=true) { m_enable_points=enable; CRenderizableDisplayList::notifyChange(); }

			/** Show or hides lines along all scanned points \sa setLineWidth, setLineColor*/
			inline void enableLine(bool enable=true) { m_enable_line=enable; CRenderizableDisplayList::notifyChange(); }

			/** Show or hides the scanned area as a 2D surface \sa setSurfaceColor */
			inline void enableSurface(bool enable=true) { m_enable_surface=enable; CRenderizableDisplayList::notifyChange(); }

			void setLineWidth(float w) { m_line_width=w; }
			float getLineWidth() const { return  m_line_width;}

			void setPointsWidth(float w) { m_points_width=w; }

			void setLineColor(float R,float G, float B, float A=1.0f)
			{
				m_line_R=R;
				m_line_G=G;
				m_line_B=B;
				m_line_A=A;
			}
			void setPointsColor(float R,float G, float B, float A=1.0f)
			{
				m_points_R=R;
				m_points_G=G;
				m_points_B=B;
				m_points_A=A;
			}
			void setSurfaceColor(float R,float G, float B, float A=1.0f)
			{
				m_plane_R=R;
				m_plane_G=G;
				m_plane_B=B;
				m_plane_A=A;
			}

			void setScan( const mrpt::slam::CObservation2DRangeScan	&scan)
			{
				CRenderizableDisplayList::notifyChange();
				m_cache_valid = false;
				m_scan = scan;
			}

			/** Render
			  */
			void  render_dl() const;

		private:
			/** Constructor
			  */
			CPlanarLaserScan( );

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CPlanarLaserScan() { }
		};

	} // end namespace

} // End of namespace


#endif

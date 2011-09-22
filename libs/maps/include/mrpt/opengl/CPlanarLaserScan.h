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

			void sePointsWidth(float w) { m_points_width=w; }

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

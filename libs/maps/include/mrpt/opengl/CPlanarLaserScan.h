/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef opengl_CPlanarLaserScan_H
#define opengl_CPlanarLaserScan_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/maps/CSimplePointsMap.h>


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
			mrpt::obs::CObservation2DRangeScan	m_scan;
			mutable mrpt::maps::CSimplePointsMap		m_cache_points;
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

			void setScan( const mrpt::obs::CObservation2DRangeScan	&scan)
			{
				CRenderizableDisplayList::notifyChange();
				m_cache_valid = false;
				m_scan = scan;
			}

			/** Render
			  */
			void  render_dl() const MRPT_OVERRIDE;

			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

		private:
			/** Constructor
			  */
			CPlanarLaserScan( );

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CPlanarLaserScan() { }
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CPlanarLaserScan, CRenderizableDisplayList, MAPS_IMPEXP )

	} // end namespace

} // End of namespace


#endif

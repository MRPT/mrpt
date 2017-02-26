/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CHeightGridMap2D_Base_H
#define CHeightGridMap2D_Base_H

#include <mrpt/obs/CObservation.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace maps
	{
		/** Virtual base class for Digital Elevation Model (DEM) maps. See derived classes for details.
		  * This class implements those operations which are especific to DEMs.
		  * \ingroup mrpt_maps_grp */
		class MAPS_IMPEXP CHeightGridMap2D_Base
		{
		public:
			CHeightGridMap2D_Base();
			virtual ~CHeightGridMap2D_Base();

			/** @name Specific API for Digital Elevation Model (DEM) maps 
			    @{ */
			/** Gets the intersection between a 3D line and a Height Grid map (taking into account the different heights of each individual cell)  */
			bool intersectLine3D(const mrpt::math::TLine3D &r1, mrpt::math::TObject3D &obj) const;

			/** Computes the minimum and maximum height in the grid.
			  * \return False if there is no observed cell yet. */
			bool getMinMaxHeight(float &z_min, float &z_max) const;

			/** Extra params for insertIndividualPoint() */
			struct MAPS_IMPEXP TPointInsertParams
			{
				double pt_z_std; //!< (Default:0.0) If !=0, use this value as the uncertainty (standard deviation) for the point "z" coordinate, instead of the map-wise default value.
				bool   update_map_after_insertion; //!< (default: true) run any required operation to ensure the map reflects the changes caused by this point. Otherwise, calling dem_update_map() is required.

				TPointInsertParams();
			};
			/** Update the DEM with one new point.
			  * \sa mrpt::maps::CMetricMap::insertObservation() for inserting higher-level objects like 2D/3D LIDAR scans
			  * \return true if updated OK, false if (x,y) is out of bounds */
			virtual bool insertIndividualPoint(const double x,const double y,const double z, const TPointInsertParams & params = TPointInsertParams() ) = 0;

			virtual double dem_get_resolution() const = 0;
			virtual size_t dem_get_size_x() const = 0;
			virtual size_t dem_get_size_y() const = 0;
			virtual bool   dem_get_z_by_cell(const size_t cx, const size_t cy, double &z_out) const = 0; //!< Get cell 'z' by (cx,cy) cell indices. \return false if out of bounds or un-observed cell.
			virtual bool   dem_get_z(const double x, const double y, double &z_out) const = 0; //!< Get cell 'z' (x,y) by metric coordinates. \return false if out of bounds or un-observed cell.
			virtual void   dem_update_map() = 0; //!< Ensure that all observations are reflected in the map estimate
			/** @} */

			/** Internal method called by internal_insertObservation() */
			bool dem_internal_insertObservation(const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D *robotPose = NULL );

		};
	} // End of namespace
} // End of namespace
#endif

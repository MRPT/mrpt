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

#ifndef CHeightGridMap2D_H
#define CHeightGridMap2D_H

#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/geometry.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/stl_extensions.h>

#include <mrpt/slam/CMetricMap.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace poses
	{
		class CPose2D;
	}
	namespace slam
	{
		using namespace mrpt::utils;

		class CObservation;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CHeightGridMap2D, CMetricMap, MAPS_IMPEXP  )

		/** The contents of each cell in a CHeightGridMap2D map.
		 **/
		struct MAPS_IMPEXP THeightGridmapCell
		{
			/** Constructor
			  */
			THeightGridmapCell() : h(0), w(0)
			{}

			/** The current average height (in meters).
			  */
			float	h;

			/** The current standard deviation of the height (in meters).
			  */
			float	var;

			/** Auxiliary variable for storing the incremental mean value (in meters).
			  */
			float	u;

			/** Auxiliary (in meters).
			  */
			float	v;


			/** [mrSimpleAverage model] The accumulated weight: initially zero if un-observed, increased by one for each observation.
			  */
			uint32_t	w;
		};

		/** A mesh representation of a surface which keeps the estimated height for each (x,y) location.
		  *  Important implemented features are the insertion of 2D laser scans (from arbitrary 6D poses) and the exportation as 3D scenes.
		  *
		  *   Each cell contains the up-to-date average height from measured falling in that cell. Algorithms that can be used:
		  *		- mrSimpleAverage: Each cell only stores the current average value.
		  */
		class MAPS_IMPEXP CHeightGridMap2D : public CMetricMap, public utils::CDynamicGrid<THeightGridmapCell>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CHeightGridMap2D )
		public:

			/** Calls the base CMetricMap::clear
			  * Declared here to avoid ambiguity between the two clear() in both base classes.
			  */
			inline void clear() { CMetricMap::clear(); }

			float cell2float(const THeightGridmapCell& c) const
			{
				return float(c.h);
			}

			/** The type of map representation to be used.
			  *  See mrpt::slam::CHeightGridMap2D for discussion.
			  */
			enum TMapRepresentation
			{
				mrSimpleAverage = 0
//				mrSlidingWindow
			};

			/** Constructor
			  */
			CHeightGridMap2D(
				TMapRepresentation	mapType = mrSimpleAverage,
				float				x_min = -2,
				float				x_max = 2,
				float				y_min = -2,
				float				y_max = 2,
				float				resolution = 0.1
				);

			 /** Returns true if the map is empty/no observation has been inserted.
			   */
			 bool  isEmpty() const;

			/** Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
			 *
			 * \param takenFrom The robot's pose the observation is supposed to be taken from.
			 * \param obs The observation.
			 * \return This method returns a likelihood in the range [0,1].
			 *
			 * \sa Used in particle filter algorithms, see: CMultiMetricMapPDF::update
			 */
			 double	 computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom );

			/** Parameters related with inserting observations into the map.
			  */
			struct MAPS_IMPEXP TInsertionOptions : public utils::CLoadableOptions
			{
				/** Default values loader:
				  */
				TInsertionOptions();

				/** See utils::CLoadableOptions
				  */
				void  loadFromConfigFile(
					const mrpt::utils::CConfigFileBase  &source,
					const std::string &section);

				/** See utils::CLoadableOptions
				  */
				void  dumpToTextStream(CStream	&out) const;

				/** Wether to perform filtering by z-coordinate (default=false): coordinates are always RELATIVE to the robot for this filter.
				  */
				bool	filterByHeight;

				/** Only when filterByHeight is true: coordinates are always RELATIVE to the robot for this filter.
				  */
				float	z_min,z_max;

				float	minDistBetweenPointsWhenInserting;	//!< When inserting a scan, a point cloud is first created with this minimum distance between the 3D points (default=0).

			} insertionOptions;

			/** Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
			 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
			 * \param  otherMap					  [IN] The other map to compute the matching with.
			 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
			 * \param  minDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
			 * \param  minMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
			 *
			 * \return The matching ratio [0,1]
			 * \sa computeMatchingWith2D
			 */
			float  compute3DMatchingRatio(
					const CMetricMap						*otherMap,
					const CPose3D							&otherMapPose,
					float									minDistForCorr = 0.10f,
					float									minMahaDistForCorr = 2.0f
					) const;

			/** The implementation in this class just calls all the corresponding method of the contained metric maps.
			  */
			void  saveMetricMapRepresentationToFile(
				const std::string	&filNamePrefix
				) const;

			/** Returns a 3D object representing the map: by default, it will be a mrpt::opengl::CMesh object, unless
			  *   it is specified otherwise in mrpt::
			  */
			void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

			/** Return the type of the gas distribution map, according to parameters passed on construction.
			  */
			TMapRepresentation	 getMapType();


			/** Gets the intersection between a 3D line and a Height Grid map (taking into account the different heights of each individual cell).
			  */
			bool intersectLine3D(const TLine3D &r1, TObject3D &obj) const;

			/** Computes the minimum and maximum height in the grid.
			  * \return False if there is no observed cell yet.
			  */
			bool getMinMaxHeight(float &z_min, float &z_max) const;

			/** Return the number of cells with at least one height data inserted. */
			size_t countObservedCells() const; 

		protected:

			/** The map representation type of this map.
			  */
			TMapRepresentation		m_mapType;

			 /** Erase all the contents of the map
			  */
			 virtual void  internal_clear();

			 /** Insert the observation information into this map. This method must be implemented
			  *    in derived classes.
			  * \param obs The observation
			  * \param robotPose The 3D pose of the robot mobile base in the map reference system, or NULL (default) if you want to use CPose2D(0,0,deg)
			  *
			  * \sa CObservation::insertObservationInto
			  */
			 virtual bool  internal_insertObservation( const CObservation *obs, const CPose3D *robotPose = NULL );

		};


	} // End of namespace

	namespace global_settings
	{
		/** If set to true (default), mrpt::slam::CHeightGridMap2D will be exported as a opengl::CMesh, otherwise, as a opengl::CPointCloudColoured
		  * Affects to:
		  *		- CHeightGridMap2D::getAs3DObject
		  */
		extern MAPS_IMPEXP bool HEIGHTGRIDMAP_EXPORT3D_AS_MESH;
	}

	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<slam::CHeightGridMap2D::TMapRepresentation>
		{
			typedef slam::CHeightGridMap2D::TMapRepresentation enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(slam::CHeightGridMap2D::mrSimpleAverage,     "mrSimpleAverage");
			}
		};
	} // End of namespace

} // End of namespace

#endif

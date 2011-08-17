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

#ifndef CReflectivityGridMap2D_H
#define CReflectivityGridMap2D_H

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/stl_extensions.h>

#include <mrpt/slam/CMetricMap.h>
#include <mrpt/slam/CLogOddsGridMap2D.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		using namespace mrpt;
		using namespace mrpt::utils;

		class CObservation;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CReflectivityGridMap2D, CMetricMap, MAPS_IMPEXP  )

		/** A 2D grid map representing the reflectivity of the environment (for example, measured with an IR proximity sensor).
		  *
		  *  Important implemented features are:
		  *		- Insertion of mrpt::slam::CObservationReflectivity observations.
		  *		- Probability estimation of observations. See base class.
		  *		- Rendering as 3D object: a 2D textured plane.
		  *		- Automatic resizing of the map limits when inserting observations close to the border.
		  *
		  *   Each cell contains the up-to-date average height from measured falling in that cell. Algorithms that can be used:
		  *		- mrSimpleAverage: Each cell only stores the current average value.
	  	  * \ingroup mrpt_maps_grp
		  */
		class MAPS_IMPEXP CReflectivityGridMap2D :
			public CMetricMap,
			public utils::CDynamicGrid<int8_t>,
			public CLogOddsGridMap2D<int8_t>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CReflectivityGridMap2D )

		protected:
			static CLogOddsGridMapLUT<cell_t>  m_logodd_lut; //!< Lookup tables for log-odds

		public:

			/** Calls the base CMetricMap::clear
			  * Declared here to avoid ambiguity between the two clear() in both base classes.
			  */
			inline void clear() { CMetricMap::clear(); }

			float cell2float(const int8_t& c) const
			{
				return m_logodd_lut.l2p(c);
			}

			/** Constructor
			  */
			CReflectivityGridMap2D(
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

			/** Returns the grid as a 8-bit graylevel image, where each pixel is a cell (output image is RGB only if forceRGB is true)
			  */
			void  getAsImage( utils::CImage	&img, bool verticalFlip = false, bool forceRGB=false) const;

		protected:

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


} // End of namespace

#endif

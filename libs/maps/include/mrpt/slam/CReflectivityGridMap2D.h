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

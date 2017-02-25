/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CReflectivityGridMap2D_H
#define CReflectivityGridMap2D_H

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CLogOddsGridMap2D.h>
#include <mrpt/obs/obs_frwds.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace maps
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CReflectivityGridMap2D, CMetricMap, MAPS_IMPEXP  )

		/** A 2D grid map representing the reflectivity of the environment (for example, measured with an IR proximity sensor).
		  *
		  *  Important implemented features are:
		  *		- Insertion of mrpt::obs::CObservationReflectivity observations.
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

			float cell2float(const int8_t& c) const MRPT_OVERRIDE
			{
				return m_logodd_lut.l2p(c);
			}

			/** Constructor  */
			CReflectivityGridMap2D(double x_min = -2, double x_max = 2, double y_min = -2, double y_max = 2,double resolution = 0.1);

			 /** Returns true if the map is empty/no observation has been inserted. */
			 bool isEmpty() const MRPT_OVERRIDE;


			/** Parameters related with inserting observations into the map.
			  */
			struct MAPS_IMPEXP TInsertionOptions : public utils::CLoadableOptions
			{
				/** Default values loader */
				TInsertionOptions();

				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
				void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs
			} insertionOptions;

			/** See docs in base class: in this class this always returns 0 */
			float compute3DMatchingRatio(const mrpt::maps::CMetricMap *otherMap, const mrpt::poses::CPose3D &otherMapPose, const TMatchingRatioParams &params) const MRPT_OVERRIDE;

			void saveMetricMapRepresentationToFile(const std::string &filNamePrefix ) const MRPT_OVERRIDE;

			void getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &outObj) const MRPT_OVERRIDE;

			/** Returns the grid as a 8-bit graylevel image, where each pixel is a cell (output image is RGB only if forceRGB is true) */
			void  getAsImage( utils::CImage	&img, bool verticalFlip = false, bool forceRGB=false) const;

		protected:

			// See docs in base class
			void  internal_clear() MRPT_OVERRIDE;
			bool  internal_insertObservation( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D *robotPose = NULL ) MRPT_OVERRIDE;
			double internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom )  MRPT_OVERRIDE;

			MAP_DEFINITION_START(CReflectivityGridMap2D,MAPS_IMPEXP)
				double min_x,max_x,min_y,max_y,resolution;	//!< See CReflectivityGridMap2DOptions::CReflectivityGridMap2DOptions
				mrpt::maps::CReflectivityGridMap2D::TInsertionOptions	insertionOpts;
			MAP_DEFINITION_END(CReflectivityGridMap2D,MAPS_IMPEXP)
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CReflectivityGridMap2D, CMetricMap, MAPS_IMPEXP  )


	} // End of namespace


} // End of namespace

#endif

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CWirelessPowerGridMap2D_H
#define CWirelessPowerGridMap2D_H

#include <mrpt/maps/CRandomFieldGridMap2D.h>
#include <mrpt/obs/CObservationWirelessPower.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
namespace maps
{
	using namespace mrpt::utils;
	using namespace mrpt::poses;
	using namespace mrpt::math;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CWirelessPowerGridMap2D , CRandomFieldGridMap2D, MAPS_IMPEXP )

	/** CWirelessPowerGridMap2D represents a PDF of wifi concentrations over a 2D area.
	  *
	  *  There are a number of methods available to build the wifi grid-map, depending on the value of
	  *    "TMapRepresentation maptype" passed in the constructor (see CRandomFieldGridMap2D for a discussion).
	  *
	  * Update the map with insertIndividualReading() or insertObservation()
	  *
	  * \sa mrpt::maps::CRandomFieldGridMap2D, mrpt::maps::CMetricMap, mrpt::utils::CDynamicGrid, The application icp-slam, mrpt::maps::CMultiMetricMap
	  * \ingroup mrpt_maps_grp
	  */
	class MAPS_IMPEXP CWirelessPowerGridMap2D : public CRandomFieldGridMap2D
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CWirelessPowerGridMap2D )
	public:
		/** Constructor
		  */
		CWirelessPowerGridMap2D(
			TMapRepresentation	mapType = mrKernelDM,
            float				x_min = -2,
			float				x_max = 2,
			float				y_min = -2,
			float				y_max = 2,
			float				resolution = 0.1
			);

		/** Destructor */
		virtual ~CWirelessPowerGridMap2D();


		// See docs in base class
		double	 computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom );


		/** Parameters related with inserting observations into the map:
		  */
		struct MAPS_IMPEXP TInsertionOptions :
			public utils::CLoadableOptions,
			public TInsertionOptionsCommon
		{
			TInsertionOptions();	//!< Default values loader

			/** See utils::CLoadableOptions */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			void  dumpToTextStream(mrpt::utils::CStream	&out) const; //!< See utils::CLoadableOptions

		} insertionOptions;


		/** Returns an image just as described in \a saveAsBitmapFile */
		virtual void  getAsBitmapFile(mrpt::utils::CImage &out_img) const;

		/** The implementation in this class just calls all the corresponding method of the contained metric maps.
		  */
		void  saveMetricMapRepresentationToFile(
			const std::string	&filNamePrefix
			) const;

		/** Save a matlab ".m" file which represents as 3D surfaces the mean and a given confidence level for the concentration of each cell.
		  *  This method can only be called in a KF map model.
		  */
		void  saveAsMatlab3DGraph(const std::string  &filName) const;

		/** Returns a 3D object representing the map.
		  */
		void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

	protected:
		/** Get the part of the options common to all CRandomFieldGridMap2D classes */
		virtual CRandomFieldGridMap2D::TInsertionOptionsCommon * getCommonInsertOptions() {
			return &insertionOptions;
		}

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
		 virtual bool  internal_insertObservation( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D *robotPose = NULL );


	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CWirelessPowerGridMap2D , CRandomFieldGridMap2D, MAPS_IMPEXP )

	} // End of namespace
} // End of namespace

#endif

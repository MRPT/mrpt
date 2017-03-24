/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
		/** Constructor */
		CWirelessPowerGridMap2D(TMapRepresentation mapType = mrKernelDM, double x_min = -2, double x_max = 2, double y_min = -2, double y_max = 2, double resolution = 0.1);

		/** Destructor */
		virtual ~CWirelessPowerGridMap2D();

		/** Parameters related with inserting observations into the map:
		  */
		struct MAPS_IMPEXP TInsertionOptions :
			public utils::CLoadableOptions,
			public TInsertionOptionsCommon
		{
			TInsertionOptions();	//!< Default values loader

			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

		} insertionOptions;

		void getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &outObj) const MRPT_OVERRIDE; //!< Returns a 3D object representing the map 

	protected:
		/** Get the part of the options common to all CRandomFieldGridMap2D classes */
		virtual CRandomFieldGridMap2D::TInsertionOptionsCommon * getCommonInsertOptions() MRPT_OVERRIDE {
			return &insertionOptions;
		}

		// See docs in derived class
		void  internal_clear() MRPT_OVERRIDE;
		bool  internal_insertObservation( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D *robotPose = NULL ) MRPT_OVERRIDE;
		double internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom ) MRPT_OVERRIDE;

		MAP_DEFINITION_START(CWirelessPowerGridMap2D,MAPS_IMPEXP)
			double min_x,max_x,min_y,max_y,resolution;	//!< See CWirelessPowerGridMap2D::CWirelessPowerGridMap2D
			mrpt::maps::CWirelessPowerGridMap2D::TMapRepresentation	mapType;	//!< The kind of map representation (see CWirelessPowerGridMap2D::CWirelessPowerGridMap2D)
			mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions	insertionOpts;
		MAP_DEFINITION_END(CWirelessPowerGridMap2D,MAPS_IMPEXP)

	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CWirelessPowerGridMap2D , CRandomFieldGridMap2D, MAPS_IMPEXP )

	} // End of namespace
} // End of namespace

#endif

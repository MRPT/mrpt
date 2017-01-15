/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CGasConcentrationGridMap2D_H
#define CGasConcentrationGridMap2D_H

#include <mrpt/maps/CRandomFieldGridMap2D.h>
#include <mrpt/obs/CObservationGasSensors.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
namespace maps
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CGasConcentrationGridMap2D , CRandomFieldGridMap2D, MAPS_IMPEXP )

	/** CGasConcentrationGridMap2D represents a PDF of gas concentrations over a 2D area.
	  *
	  *  There are a number of methods available to build the gas grid-map, depending on the value of
	  *    "TMapRepresentation maptype" passed in the constructor (see base class mrpt::maps::CRandomFieldGridMap2D).
	  *
	  * Update the map with insertIndividualReading() or insertObservation()
	  *
	  * \sa mrpt::maps::CRandomFieldGridMap2D, mrpt::maps::CMetricMap, mrpt::utils::CDynamicGrid, The application icp-slam, mrpt::maps::CMultiMetricMap
	  * \ingroup mrpt_maps_grp
	  */
	class MAPS_IMPEXP CGasConcentrationGridMap2D : public CRandomFieldGridMap2D
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CGasConcentrationGridMap2D )
	public:

		/** Constructor
		  */
		CGasConcentrationGridMap2D(
			TMapRepresentation	mapType = mrAchim,
            float				x_min = -2,
			float				x_max = 2,
			float				y_min = -2,
			float				y_max = 2,
			float				resolution = 0.1
			);

		/** Destructor */
		virtual ~CGasConcentrationGridMap2D();

		/** Parameters related with inserting observations into the map:
		  */
		struct MAPS_IMPEXP TInsertionOptions :
			public utils::CLoadableOptions,
			public TInsertionOptionsCommon
		{
			TInsertionOptions();	//!< Default values loader

			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			/** @name For all mapping methods
			    @{ */
			std::string gasSensorLabel;	//!< The label of the CObservationGasSensor used to generate the map
			uint16_t enose_id;			//!< id for the enose used to generate this map (must be < gasGrid_count)
			uint16_t gasSensorType;		//!< The sensor type for the gas concentration map (0x0000 ->mean of all installed sensors, 0x2600, 0x6810, ...)
			std::string windSensorLabel; //!< The label of the WindSenor used to simulate advection

			//[Advection Options]
			bool useWindInformation;	//! Indicates if wind information must be used to simulate Advection
			float advectionFreq;		//! Frequency for simulating advection (only used to transform wind speed to distance)
			float std_windNoise_phi, std_windNoise_mod;  //! The std to consider on wind information measurements
			float default_wind_direction, default_wind_speed;	//! The default value for the wind information

			/** @} */

		} insertionOptions;

		/** Returns a 3D object representing the map */
		virtual void getAs3DObject( mrpt::opengl::CSetOfObjectsPtr &outObj ) const MRPT_OVERRIDE;

		/** Returns two 3D objects representing the mean and variance maps */
		virtual void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&meanObj, mrpt::opengl::CSetOfObjectsPtr	&varObj ) const MRPT_OVERRIDE;

		/** Returns the 3D object representing the wind grid information */
		void  getWindAs3DObject( mrpt::opengl::CSetOfObjectsPtr &windObj) const;

		 /** Increase the kf_std of all cells from the m_map
		 *	This mehod is usually called by the main_map to simulate loss of confidence in measurements when time passes */
		virtual void increaseUncertainty(const double STD_increase_value);

		 /** Implements the transition model of the gasConcentration map using the information of the wind maps  */
		bool simulateAdvection(const double &STD_increase_value);

		// Params for the estimation of the gaussian volume in a cell.
		struct MAPS_IMPEXP TGaussianCell
		{
			int cx;			//x-index of the cell
			int cy;			//y-index of the cell
			float value;	//volume approximation
		};

		//Params for the estimation of the wind effect on each cell of the grid
		struct MAPS_IMPEXP TGaussianWindTable
		{
			//Fixed params
			float resolution;	//Cell_resolution. To be read from config-file
			float std_phi;		//to be read from config-file
			float std_r;		//to be read from config-file

			//unsigned int subcell_count; //subcell_count x subcell_count	subcells
			//float subcell_res;
			float phi_inc;	//rad
			unsigned int phi_count;
			float r_inc;	//m
			float max_r;	//maximum distance (m)
			unsigned int r_count;

			std::vector< std::vector< std::vector<TGaussianCell> > > *table;
		}LUT;

	protected:

		/** Get the part of the options common to all CRandomFieldGridMap2D classes */
		virtual CRandomFieldGridMap2D::TInsertionOptionsCommon * getCommonInsertOptions()  MRPT_OVERRIDE {
			return &insertionOptions;
		}

		// See docs in base class
		void  internal_clear() MRPT_OVERRIDE;
		bool  internal_insertObservation( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D *robotPose = NULL ) MRPT_OVERRIDE;
		double internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom ) MRPT_OVERRIDE;

		 /** Builds a LookUp table with the values of the Gaussian Weights result of the wind advection
		 *   for a specific std_windNoise_phi value.
		 */
		 bool build_Gaussian_Wind_Grid();

		 bool save_Gaussian_Wind_Grid_To_File();
		 bool load_Gaussian_Wind_Grid_From_File();

		 /** Gridmaps of the wind Direction/Module */
		 mrpt::utils::CDynamicGrid<double> windGrid_module, windGrid_direction;

		 /** The timestamp of the last time the advection simulation was executed */
		 mrpt::system::TTimeStamp timeLastSimulated;

		MAP_DEFINITION_START(CGasConcentrationGridMap2D,MAPS_IMPEXP)
			float	min_x,max_x,min_y,max_y,resolution;	//!< See CGasConcentrationGridMap2D::CGasConcentrationGridMap2D
			mrpt::maps::CGasConcentrationGridMap2D::TMapRepresentation	mapType;	//!< The kind of map representation (see CGasConcentrationGridMap2D::CGasConcentrationGridMap2D)
			mrpt::maps::CGasConcentrationGridMap2D::TInsertionOptions   insertionOpts;	//!< Observations insertion options
		MAP_DEFINITION_END(CGasConcentrationGridMap2D,MAPS_IMPEXP)

	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CGasConcentrationGridMap2D , CRandomFieldGridMap2D, MAPS_IMPEXP )

	} // End of namespace

} // End of namespace

#endif

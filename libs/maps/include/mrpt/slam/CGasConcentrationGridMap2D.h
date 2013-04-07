/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#ifndef CGasConcentrationGridMap2D_H
#define CGasConcentrationGridMap2D_H

#include <mrpt/slam/CRandomFieldGridMap2D.h>
#include <mrpt/slam/CObservationGasSensors.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	using namespace mrpt::utils;
	using namespace mrpt::poses;
	using namespace mrpt::math;
	using namespace std;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CGasConcentrationGridMap2D , CRandomFieldGridMap2D, MAPS_IMPEXP )

	typedef TRandomFieldCell TGasConcentrationCell;  //!< Defined for backward compatibility only (mrpt <0.9.5)

	/** CGasConcentrationGridMap2D represents a PDF of gas concentrations over a 2D area.
	  *
	  *  There are a number of methods available to build the gas grid-map, depending on the value of
	  *    "TMapRepresentation maptype" passed in the constructor (see base class mrpt::slam::CRandomFieldGridMap2D).
	  *
	  * Update the map with insertIndividualReading() or insertObservation()
	  *
	  * \sa mrpt::slam::CRandomFieldGridMap2D, mrpt::slam::CMetricMap, mrpt::utils::CDynamicGrid, The application icp-slam, mrpt::slam::CMultiMetricMap
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

		/** Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
		 *
		 * \param takenFrom The robot's pose the observation is supposed to be taken from.
		 * \param obs The observation.
		 * \return This method returns a likelihood in the range [0,1].
		 *
		 * \sa Used in particle filter algorithms, see: CMultiMetricMapPDF::update
		 */
		 virtual double	 computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom );


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

			void  dumpToTextStream(CStream	&out) const; //!< See utils::CLoadableOptions

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


		/** Returns an image just as described in \a saveAsBitmapFile */
		virtual void  getAsBitmapFile(mrpt::utils::CImage &out_img) const;

		/** The implementation in this class just calls all the corresponding method of the contained metric maps.
		  */
		virtual void  saveMetricMapRepresentationToFile(
			const std::string	&filNamePrefix
			) const;

		/** Save a matlab ".m" file which represents as 3D surfaces the mean and a given confidence level for the concentration of each cell.
		  *  This method can only be called in a KF map model.
		  */
		virtual void  saveAsMatlab3DGraph(const std::string  &filName) const;

		/** Returns a 3D object representing the map.
		  */
		virtual void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

		/** Returns two 3D objects representing the mean and variance maps.
		  */
		virtual void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&meanObj, mrpt::opengl::CSetOfObjectsPtr	&varObj ) const;

		/** Returns the 3D object representing the wind grid information
		 */
		void  getWindAs3DObject( mrpt::opengl::CSetOfObjectsPtr &windObj) const;

		 /** Increase the kf_std of all cells from the m_map
		 *	This mehod is usually called by the main_map to simulate loss of confidence in measurements when time passes
		 */
		virtual void increaseUncertainty(const double STD_increase_value);

		 /** Implements the transition model of the gasConcentration map using the information of the wind maps.
		 */
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
		virtual CRandomFieldGridMap2D::TInsertionOptionsCommon * getCommonInsertOptions() {
			return &insertionOptions;
		}

		 /** Erase all the contents of the map */
		 virtual void  internal_clear();

		 /** Insert the observation information into this map. This method must be implemented
		  *    in derived classes.
		  * \param obs The observation
		  * \param robotPose The 3D pose of the robot mobile base in the map reference system, or NULL (default) if you want to use CPose2D(0,0,deg)
		  *
		  * \sa CObservation::insertObservationInto
		  */
		 virtual bool  internal_insertObservation( const CObservation *obs, const CPose3D *robotPose = NULL );

		 /** Builds a LookUp table with the values of the Gaussian Weights result of the wind advection
		 *   for a specific std_windNoise_phi value.
		 */
		 bool build_Gaussian_Wind_Grid();

		 bool save_Gaussian_Wind_Grid_To_File();
		 bool load_Gaussian_Wind_Grid_From_File();

		 /** Gridmaps of the wind Direction/Module.
		  */
		 mrpt::slam::CDynamicGrid<double> windGrid_module, windGrid_direction;

		 /** The timestamp of the last time the advection simulation was executed */
		 mrpt::system::TTimeStamp timeLastSimulated;


	};

	} // End of namespace

} // End of namespace

#endif

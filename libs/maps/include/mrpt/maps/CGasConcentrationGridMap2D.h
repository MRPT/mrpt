/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CRandomFieldGridMap2D.h>
#include <mrpt/obs/CObservationGasSensors.h>

namespace mrpt::maps
{
/** CGasConcentrationGridMap2D represents a PDF of gas concentrations over a 2D
 * area.
 *
 *  There are a number of methods available to build the gas grid-map,
 * depending on the value of
 *    "TMapRepresentation maptype" passed in the constructor (see base class
 * mrpt::maps::CRandomFieldGridMap2D).
 *
 * Update the map with insertIndividualReading() or insertObservation()
 *
 * \sa mrpt::maps::CRandomFieldGridMap2D, mrpt::maps::CMetricMap,
 * mrpt::containers::CDynamicGrid, The application icp-slam,
 * mrpt::maps::CMultiMetricMap
 * \ingroup mrpt_maps_grp
 */
class CGasConcentrationGridMap2D : public CRandomFieldGridMap2D
{
	DEFINE_SERIALIZABLE(CGasConcentrationGridMap2D, mrpt::maps)
   public:
	/** Constructor
	 */
	CGasConcentrationGridMap2D(
		TMapRepresentation mapType = mrAchim, float x_min = -2, float x_max = 2,
		float y_min = -2, float y_max = 2, float resolution = 0.1f);

	/** Destructor */
	~CGasConcentrationGridMap2D() override;

	/** Parameters related with inserting observations into the map:
	 */
	struct TInsertionOptions : public mrpt::config::CLoadableOptions,
							   public TInsertionOptionsCommon
	{
		/** Default values loader */
		TInsertionOptions();

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		/** @name For all mapping methods
			@{ */
		/** The label of the CObservationGasSensor used to generate the map */
		std::string gasSensorLabel;
		/** id for the enose used to generate this map (must be < gasGrid_count)
		 */
		uint16_t enose_id{0};
		/** The sensor type for the gas concentration map (0x0000 ->mean of all
		 * installed sensors, 0x2600, 0x6810, ...) */
		uint16_t gasSensorType{0x0000};
		/** The label of the WindSenor used to simulate advection */
		std::string windSensorLabel;

		//[Advection Options]
		bool useWindInformation{
			false};  //! Indicates if wind information must be used
		//! to simulate Advection
		float advectionFreq;  //! Frequency for simulating advection (only used
		//! to transform wind speed to distance)
		float std_windNoise_phi{0.2f},
			std_windNoise_mod{0.2f};  //! The std to consider on
		//! wind information
		//! measurements
		float default_wind_direction{0.0f},
			default_wind_speed{
				1.0f};  //! The default value for the wind information

		/** @} */

	} insertionOptions;

	/** Returns a 3D object representing the map */
	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

	/** Returns two 3D objects representing the mean and variance maps */
	void getAs3DObject(
		mrpt::opengl::CSetOfObjects::Ptr& meanObj,
		mrpt::opengl::CSetOfObjects::Ptr& varObj) const override;

	/** Returns the 3D object representing the wind grid information */
	void getWindAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& windObj) const;

	/** Increase the kf_std of all cells from the m_map
	 *	This mehod is usually called by the main_map to simulate loss of
	 *confidence in measurements when time passes */
	virtual void increaseUncertainty(const double STD_increase_value);

	/** Implements the transition model of the gasConcentration map using the
	 * information of the wind maps  */
	bool simulateAdvection(double STD_increase_value);

	// Params for the estimation of the gaussian volume in a cell.
	struct TGaussianCell
	{
		int cx = 0;  // x-index of the cell
		int cy = 0;  // y-index of the cell
		float value = 0;  // volume approximation
	};

	// Params for the estimation of the wind effect on each cell of the grid
	struct TGaussianWindTable
	{
		// Fixed params
		float resolution = 0;  // Cell_resolution. To be read from config-file
		float std_phi = 0;  // to be read from config-file
		float std_r = 0;  // to be read from config-file

		// unsigned int subcell_count; //subcell_count x subcell_count	subcells
		// float subcell_res;
		float phi_inc = 0;  // rad
		unsigned int phi_count = 0;
		float r_inc = 0;  // m
		float max_r = 0;  // maximum distance (m)
		unsigned int r_count = 0;

		std::vector<std::vector<std::vector<TGaussianCell>>>* table;
	};

	TGaussianWindTable LUT;

   protected:
	/** Get the part of the options common to all CRandomFieldGridMap2D classes
	 */
	CRandomFieldGridMap2D::TInsertionOptionsCommon* getCommonInsertOptions()
		override
	{
		return &insertionOptions;
	}

	// See docs in base class
	void internal_clear() override;
	bool internal_insertObservation(
		const mrpt::obs::CObservation& obs,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;
	double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation& obs,
		const mrpt::poses::CPose3D& takenFrom) override;

	/** Builds a LookUp table with the values of the Gaussian Weights result of
	 * the wind advection
	 *   for a specific std_windNoise_phi value.
	 */
	bool build_Gaussian_Wind_Grid();

	bool save_Gaussian_Wind_Grid_To_File();
	bool load_Gaussian_Wind_Grid_From_File();

	/** Gridmaps of the wind Direction/Module */
	mrpt::containers::CDynamicGrid<double> windGrid_module, windGrid_direction;

	/** The timestamp of the last time the advection simulation was executed */
	mrpt::system::TTimeStamp timeLastSimulated;

	MAP_DEFINITION_START(CGasConcentrationGridMap2D)
	/** See CGasConcentrationGridMap2D::CGasConcentrationGridMap2D */
	float min_x{-2}, max_x{2}, min_y{-2}, max_y{2}, resolution{0.10f};
	/** The kind of map representation (see
	 * CGasConcentrationGridMap2D::CGasConcentrationGridMap2D) */
	mrpt::maps::CGasConcentrationGridMap2D::TMapRepresentation mapType{
		CGasConcentrationGridMap2D::mrKernelDM};
	/** Observations insertion options */
	mrpt::maps::CGasConcentrationGridMap2D::TInsertionOptions insertionOpts;
	MAP_DEFINITION_END(CGasConcentrationGridMap2D)
};

}  // namespace mrpt::maps

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CRandomFieldGridMap2D.h>
#include <mrpt/maps/CHeightGridMap2D_Base.h>

namespace mrpt::maps
{
/** CHeightGridMap2D_MRF represents digital-elevation-model over a 2D area, with
 * uncertainty, based on a Markov-Random-Field (MRF) estimator.
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
 * \note New in MRPT 1.4.0
 * \ingroup mrpt_maps_grp
 */
class CHeightGridMap2D_MRF : public CRandomFieldGridMap2D,
							 public CHeightGridMap2D_Base
{
	DEFINE_SERIALIZABLE(CHeightGridMap2D_MRF)
   public:
	/** Constructor */
	CHeightGridMap2D_MRF(
		TMapRepresentation mapType = mrGMRF_SD, double x_min = -2,
		double x_max = 2, double y_min = -2, double y_max = 2,
		double resolution = 0.5,
		/** [in] Whether to call updateMapEstimation(). If false, make sure of
		   calling that function before accessing map contents. */
		bool run_first_map_estimation_now = true);

	/** Parameters related with inserting observations into the map */
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
	} insertionOptions;

	/** Returns a 3D object representing the map */
	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

	/** Returns two 3D objects representing the mean and variance maps */
	void getAs3DObject(
		mrpt::opengl::CSetOfObjects::Ptr& meanObj,
		mrpt::opengl::CSetOfObjects::Ptr& varObj) const override;

	bool insertIndividualPoint(
		const double x, const double y, const double z,
		const CHeightGridMap2D_Base::TPointInsertParams& params =
			CHeightGridMap2D_Base::TPointInsertParams()) override;
	double dem_get_resolution() const override;
	size_t dem_get_size_x() const override;
	size_t dem_get_size_y() const override;
	bool dem_get_z_by_cell(
		const size_t cx, const size_t cy, double& z_out) const override;
	bool dem_get_z(
		const double x, const double y, double& z_out) const override;
	void dem_update_map() override;

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
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;
	double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D& takenFrom) override;

	MAP_DEFINITION_START(CHeightGridMap2D_MRF)
	/** Runs map estimation at start up (Default:true) */
	bool run_map_estimation_at_ctor{true};
	/** See CHeightGridMap2D_MRF::CHeightGridMap2D_MRF */
	double min_x{-2}, max_x{2}, min_y{-2}, max_y{2}, resolution{0.10f};
	/** The kind of map representation (see
	 * CHeightGridMap2D_MRF::CHeightGridMap2D_MRF) */
	mrpt::maps::CHeightGridMap2D_MRF::TMapRepresentation mapType{mrGMRF_SD};
	/** Observations insertion options */
	mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions insertionOpts;
	MAP_DEFINITION_END(CHeightGridMap2D_MRF)
};

}  // namespace mrpt::maps

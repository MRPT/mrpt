/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef CWirelessPowerGridMap2D_H
#define CWirelessPowerGridMap2D_H

#include <mrpt/maps/CRandomFieldGridMap2D.h>
#include <mrpt/obs/CObservationWirelessPower.h>

namespace mrpt
{
namespace maps
{
/** CWirelessPowerGridMap2D represents a PDF of wifi concentrations over a 2D
 * area.
  *
  *  There are a number of methods available to build the wifi grid-map,
 * depending on the value of
  *    "TMapRepresentation maptype" passed in the constructor (see
 * CRandomFieldGridMap2D for a discussion).
  *
  * Update the map with insertIndividualReading() or insertObservation()
  *
  * \sa mrpt::maps::CRandomFieldGridMap2D, mrpt::maps::CMetricMap,
 * mrpt::containers::CDynamicGrid, The application icp-slam,
 * mrpt::maps::CMultiMetricMap
  * \ingroup mrpt_maps_grp
  */
class CWirelessPowerGridMap2D : public CRandomFieldGridMap2D
{
	DEFINE_SERIALIZABLE(CWirelessPowerGridMap2D)
   public:
	/** Constructor */
	CWirelessPowerGridMap2D(
		TMapRepresentation mapType = mrKernelDM, double x_min = -2,
		double x_max = 2, double y_min = -2, double y_max = 2,
		double resolution = 0.1);

	/** Destructor */
	virtual ~CWirelessPowerGridMap2D();

	/** Parameters related with inserting observations into the map:
	  */
	struct TInsertionOptions : public utils::CLoadableOptions,
							   public TInsertionOptionsCommon
	{
		/** Default values loader */
		TInsertionOptions();

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(std::ostream& out) const override;  // See base docs

	} insertionOptions;

	/** Returns a 3D object representing the map  */
	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

   protected:
	/** Get the part of the options common to all CRandomFieldGridMap2D classes
	 */
	virtual CRandomFieldGridMap2D::TInsertionOptionsCommon*
		getCommonInsertOptions() override
	{
		return &insertionOptions;
	}

	// See docs in derived class
	void internal_clear() override;
	bool internal_insertObservation(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;
	double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D& takenFrom) override;

	MAP_DEFINITION_START(CWirelessPowerGridMap2D)
	/** See CWirelessPowerGridMap2D::CWirelessPowerGridMap2D */
	double min_x, max_x, min_y, max_y, resolution;
	/** The kind of map representation (see
	 * CWirelessPowerGridMap2D::CWirelessPowerGridMap2D) */
	mrpt::maps::CWirelessPowerGridMap2D::TMapRepresentation mapType;
	mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions insertionOpts;
	MAP_DEFINITION_END(CWirelessPowerGridMap2D, )
};

}  // End of namespace
}  // End of namespace

#endif

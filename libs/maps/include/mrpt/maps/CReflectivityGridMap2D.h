/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CLogOddsGridMap2D.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt::maps
{
/** A 2D grid map representing the reflectivity of the environment (for example,
 *measured with an IR proximity sensor).
 *
 *  Important implemented features are:
 *		- Insertion of mrpt::obs::CObservationReflectivity observations.
 *		- Probability estimation of observations. See base class.
 *		- Rendering as 3D object: a 2D textured plane.
 *		- Automatic resizing of the map limits when inserting observations close
 *to
 *the border.
 *
 *   Each cell contains the up-to-date average height from measured falling in
 *that cell. Algorithms that can be used:
 *		- mrSimpleAverage: Each cell only stores the current average value.
 * \ingroup mrpt_maps_grp
 */
class CReflectivityGridMap2D : public CMetricMap,
							   public mrpt::containers::CDynamicGrid<int8_t>,
							   public CLogOddsGridMap2D<int8_t>
{
	DEFINE_SERIALIZABLE(CReflectivityGridMap2D)

   protected:
	/** Lookup tables for log-odds */
	static CLogOddsGridMapLUT<cell_t> m_logodd_lut;

   public:
	/** Calls the base CMetricMap::clear
	 * Declared here to avoid ambiguity between the two clear() in both base
	 * classes.
	 */
	inline void clear() { CMetricMap::clear(); }
	float cell2float(const int8_t& c) const override
	{
		return m_logodd_lut.l2p(c);
	}

	/** Constructor  */
	CReflectivityGridMap2D(
		double x_min = -2, double x_max = 2, double y_min = -2,
		double y_max = 2, double resolution = 0.1);

	/** Returns true if the map is empty/no observation has been inserted. */
	bool isEmpty() const override;

	/** Parameters related with inserting observations into the map.
	 */
	struct TInsertionOptions : public mrpt::config::CLoadableOptions
	{
		/** Default values loader */
		TInsertionOptions();

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		int16_t channel{-1};  //!< The reflectivity channel for this map. If
		//! channel=-1, then any channel will be accepted.
		//! Otherwise, the map will ignore
		//! CObservationReflectivity instances with a differing
		//! channel. (Default=-1)
	} insertionOptions;

	/** See docs in base class: in this class this always returns 0 */
	float compute3DMatchingRatio(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose3D& otherMapPose,
		const TMatchingRatioParams& params) const override;

	void saveMetricMapRepresentationToFile(
		const std::string& filNamePrefix) const override;

	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

	/** Returns the grid as a 8-bit graylevel image, where each pixel is a cell
	 * (output image is RGB only if forceRGB is true) */
	void getAsImage(
		mrpt::img::CImage& img, bool verticalFlip = false,
		bool forceRGB = false) const;

   protected:
	// See docs in base class
	void internal_clear() override;
	bool internal_insertObservation(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;
	double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D& takenFrom) override;

	MAP_DEFINITION_START(CReflectivityGridMap2D)
	/** See CReflectivityGridMap2DOptions::CReflectivityGridMap2DOptions */
	double min_x{-10.0f}, max_x{10.0f}, min_y{-10.0f}, max_y{10.0f},
		resolution{0.10f};
	mrpt::maps::CReflectivityGridMap2D::TInsertionOptions insertionOpts;
	MAP_DEFINITION_END(CReflectivityGridMap2D)
};

}  // namespace mrpt::maps

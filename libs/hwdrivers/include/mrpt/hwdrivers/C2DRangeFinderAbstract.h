/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/gui/CDisplayWindow3D.h>

namespace mrpt::hwdrivers
{
/** This is the base, abstract class for "software drivers" interfaces to 2D
 * scanners (laser range finders).
 *  Physical devices may be interfaced through a serial port, a USB
 * connection,etc. but this class
 *   abstract those details throught the "binding" of the specific scanner
 * driver to a given I/O channel,
 *   which must be set by calling "hwdrivers::C2DRangeFinderAbstract::bindIO".
 * See also the derived classes.
 *
 *  There is support for "exclusion polygons", areas where points, if detected,
 * should be marked as invalid.
 *  Those areas are useful in cases where the scanner always detects part of
 * the vehicle itself, and those
 *   points want to be ignored (see
 * C2DRangeFinderAbstract::loadExclusionAreas).
 *
 * \sa comms::CSerialPort
 * \ingroup mrpt_hwdrivers_grp
 */
class C2DRangeFinderAbstract : public mrpt::system::COutputLogger,
							   public mrpt::hwdrivers::CGenericSensor
{
   private:
	mrpt::obs::CObservation2DRangeScan m_lastObservation;
	bool m_lastObservationIsNew{false};
	bool m_hardwareError{false};

	/** For being thread-safe.
	 */
	std::mutex m_csChangeStream, m_csLastObservation;

	/** A dynamic object used as buffer in doProcess */
	mrpt::obs::CObservation2DRangeScan::Ptr m_nextObservation;

	/** A list of optional exclusion polygons, in coordinates relative to the
	 * vehicle, that is, taking into account the "sensorPose". */
	mrpt::obs::CObservation2DRangeScan::TListExclusionAreasWithRanges
		m_lstExclusionPolys;
	/** A list of pairs of angles <init,end> such as all sensor ranges falling
	 * in those forbiden angles will be marked as invalid. */
	std::vector<std::pair<double, double>> m_lstExclusionAngles;

	/** If true, shows a 3D window with a preview of the grabber data */
	bool m_showPreview{false};
	mrpt::gui::CDisplayWindow3D::Ptr m_win;

   protected:
	/** The I/O channel (will be nullptr if not bound). */
	mrpt::io::CStream* m_stream{nullptr};

	/** Should be call by derived classes at "loadConfig" (loads exclusion areas
	 *AND exclusion angles).
	 *  This loads a sequence of vertices of a polygon given by its (x,y)
	 *coordinates relative to the vehicle, that is, taking into account the
	 *"sensorPose".
	 *   - exclusionZone%u_x
	 *   - exclusionZone%u_y
	 *   for %u=1,2,3,...
	 *   All points within the 2D polygon will be ignored, for any Z, unless an
	 *optional entry is found:
	 *   - exclusionZone%u_z=[z_min z_max]
	 *	  In that case, only the points within the 2D polygon AND the given
	 *range in Z will be ignored.
	 *
	 *  The number of zones is variable, but they must start at 1 and be
	 *consecutive.
	 *
	 * This also loads any other common params (e.g. 'preview')
	 * \sa filterByExclusionAreas
	 */
	void loadCommonParams(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& iniSection);

	/** Mark as invalid those points which (x,y) coordinates fall within the
	 * exclusion polygons.
	 * \sa loadExclusionAreas
	 */
	void filterByExclusionAreas(mrpt::obs::CObservation2DRangeScan& obs) const;

	/** Mark as invalid those ranges in a set of forbiden angle ranges.
	 * \sa loadExclusionAreas
	 */
	void filterByExclusionAngles(mrpt::obs::CObservation2DRangeScan& obs) const;

	/** Must be called inside the capture method to allow optional GUI preview
	 * of scans */
	void processPreview(const mrpt::obs::CObservation2DRangeScan& obs);

   public:
	/** Default constructor */
	C2DRangeFinderAbstract();
	/** Destructor */
	~C2DRangeFinderAbstract() override;

	/** Enables GUI visualization in real-time */
	void showPreview(bool enable = true) { m_showPreview = enable; }
	/** Binds the object to a given I/O channel.
	 *  The stream object must not be deleted before the destruction of this
	 * class.
	 * \sa comms::CSerialPort
	 */
	void bindIO(mrpt::io::CStream* streamIO);

	/** Get the last observation from the sensor, if available, and unmarks it
	 * as being "the last one" (thus a new scan must arrive or subsequent calls
	 * will find no new observations).
	 */
	void getObservation(
		bool& outThereIsObservation,
		mrpt::obs::CObservation2DRangeScan& outObservation,
		bool& hardwareError);

	/** Main method for a CGenericSensor */
	void doProcess() override;

	/** Specific laser scanner "software drivers" must process here new data
	 * from the I/O stream, and, if a whole scan has arrived, return it.
	 *  This method MUST BE CALLED in a timely fashion by the user to allow the
	 * proccessing of incoming data. It can be run in a different thread safely.
	 */
	virtual void doProcessSimple(
		bool& outThereIsObservation,
		mrpt::obs::CObservation2DRangeScan& outObservation,
		bool& hardwareError) = 0;

	/** Enables the scanning mode (which may depend on the specific laser
	 * device); this must be called before asking for observations to assure
	 * that the protocol has been initializated.
	 * \return If everything works "true", or "false" if there is any error.
	 */
	virtual bool turnOn() = 0;

	/** Disables the scanning mode (this can be used to turn the device in low
	 * energy mode, if available)
	 * \return If everything works "true", or "false" if there is any error.
	 */
	virtual bool turnOff() = 0;

};  // End of class
}  // namespace mrpt::hwdrivers

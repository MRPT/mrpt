/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef C2DRangeFinderAbstract_H
#define C2DRangeFinderAbstract_H

#include <mrpt/utils/CStream.h>
#include <mrpt/synch.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/utils/CConfigFileBase.h>

#include <mrpt/hwdrivers/link_pragmas.h>
#include <mrpt/hwdrivers/CGenericSensor.h>

#include <mrpt/math/CPolygon.h>

namespace mrpt
{
	namespace hwdrivers
	{
		using namespace mrpt::utils;
		using namespace mrpt::slam;

		/** This is the base, abstract class for "software drivers" interfaces to 2D scanners (laser range finders).
		  *  Physical devices may be interfaced through a serial port, a USB connection,etc. but this class
		  *   abstract those details throught the "binding" of the specific scanner driver to a given I/O channel,
		  *   which must be set by calling "hwdrivers::C2DRangeFinderAbstract::bindIO". See also the derived classes.
		  *
		  *  There is support for "exclusion polygons", areas where points, if detected, should be marked as invalid.
		  *  Those areas are useful in cases where the scanner always detects part of the vehicle itself, and those
		  *   points want to be ignored (see C2DRangeFinderAbstract::loadExclusionAreas).
		  *
		  * \sa hwdrivers::CSerialPort
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP C2DRangeFinderAbstract : public mrpt::utils::CDebugOutputCapable, public mrpt::hwdrivers::CGenericSensor
		{
		private:
			CObservation2DRangeScan	m_lastObservation;
			bool							m_lastObservationIsNew;
			bool							m_hardwareError;

			/** For being thread-safe.
			  */
			synch::CCriticalSection	m_csChangeStream,m_csLastObservation;

			CObservation2DRangeScanPtr		m_nextObservation;		//!< A dynamic object used as buffer in doProcess

			CObservation2DRangeScan::TListExclusionAreasWithRanges 	m_lstExclusionPolys;	//!< A list of optional exclusion polygons, in coordinates relative to the vehicle, that is, taking into account the "sensorPose".
			std::vector<std::pair<double,double> >  m_lstExclusionAngles; //!< A list of pairs of angles <init,end> such as all sensor ranges falling in those forbiden angles will be marked as invalid.


		protected:
			/** The I/O channel (will be NULL if not bound).
			  */
			utils::CStream					*m_stream;

			/** Should be call by derived classes at "loadConfig" (loads exclusion areas AND exclusion angles).
			  *  This loads a sequence of vertices of a polygon given by its (x,y) coordinates relative to the vehicle, that is, taking into account the "sensorPose".
			  *   - exclusionZone%u_x
			  *   - exclusionZone%u_y
			  *   for %u=1,2,3,...
			  *   All points within the 2D polygon will be ignored, for any Z, unless an optional entry is found:
			  *   - exclusionZone%u_z=[z_min z_max]
			  *	  In that case, only the points within the 2D polygon AND the given range in Z will be ignored.
			  *
			  *  The number of zones is variable, but they must start at 1 and be consecutive.
			  * \sa filterByExclusionAreas
			  */
			void loadExclusionAreas(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

			/** Mark as invalid those points which (x,y) coordinates fall within the exclusion polygons.
			  * \sa loadExclusionAreas
			  */
			void filterByExclusionAreas( CObservation2DRangeScan &obs) const;

			/** Mark as invalid those ranges in a set of forbiden angle ranges.
			  * \sa loadExclusionAreas
			  */
			void filterByExclusionAngles( CObservation2DRangeScan &obs) const;

		public:

			/** Default constructor
			  */
			C2DRangeFinderAbstract();

			/** Destructor
			  */
			virtual ~C2DRangeFinderAbstract();

			/** Binds the object to a given I/O channel.
			  *  The stream object must not be deleted before the destruction of this class.
			  * \sa hwdrivers::CSerialPort
			  */
			void  bindIO( CStream	*streamIO );

			/** Get the last observation from the sensor, if available, and unmarks it as being "the last one" (thus a new scan must arrive or subsequent calls will find no new observations).
			  */
			void  getObservation(
				bool							&outThereIsObservation,
				CObservation2DRangeScan	&outObservation,
				bool							&hardwareError );

			/** Main method for a CGenericSensor
			  */
			void doProcess();

			/** Specific laser scanner "software drivers" must process here new data from the I/O stream, and, if a whole scan has arrived, return it.
			  *  This method MUST BE CALLED in a timely fashion by the user to allow the proccessing of incoming data. It can be run in a different thread safely.
			  */
			virtual void  doProcessSimple(
				bool							&outThereIsObservation,
				CObservation2DRangeScan	&outObservation,
				bool							&hardwareError ) = 0;

			/** Enables the scanning mode (which may depend on the specific laser device); this must be called before asking for observations to assure that the protocol has been initializated.
			  * \return If everything works "true", or "false" if there is any error.
			  */
			virtual bool  turnOn() = 0;

			/** Disables the scanning mode (this can be used to turn the device in low energy mode, if available)
			  * \return If everything works "true", or "false" if there is any error.
			  */
			virtual bool  turnOff() = 0;


		};	// End of class
	} // End of namespace
} // End of namespace


#endif

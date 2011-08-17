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

#ifndef CServoeNeck_H
#define CServoeNeck_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/hwdrivers/link_pragmas.h>
#include <mrpt/hwdrivers/CInterfaceFTDIMessages.h>

namespace mrpt
{
	namespace hwdrivers
	{

		/** A USB-interface for a custom "robotic neck" designed at MAPIR lab.
		  * \ingroup mrpt_hwdrivers_grp */
		class HWDRIVERS_IMPEXP CServoeNeck : public hwdrivers::CInterfaceFTDIMessages
		{
		public:
			CServoeNeck();
			~CServoeNeck();

			/** Gets the firmware version of the eNeck board.
			  * \param out_firmwareVersion: [OUTPUT] A string containing the firmware version.
			  * \return Whether or not the procedure succeded.
			  */
			bool queryFirmwareVersion( std::string &out_firmwareVersion );			

			/** Gets the current angle of the servo (in radians within (-pi,pi))
			  * \param Angle: [OUT] The current angle.
			  * \param Servo: [IN] The id of the servo (in our ATMEGA16, from 0 to 2).
			  * \return Whether or not the procedure succeded.
			  */
			bool getCurrentAngle( double &angle, const uint8_t servo = 0 );

			/** Turns the servo up to the specified angle (in radians in the range -pi,pi, other values will be saturated to the maximum or the mininum)
			  * \param Angle: the desired angle to turn.
			  * \param Servo: the id of the servo to move (in our ATMEGA16, from 0 to 2).
			  * \param Fast: indicates if the servo must reach the angle at maximum speed
			  * \return Whether or not the procedure succeded.
			  */
			bool setAngle( double angle, const uint8_t servo = 0, bool fast = false );

			/** Turns the servo up to the specified angle (in radians in the range -pi,pi) filtered by average with the last N specified angles.
			  * \param Angle: the new desired angle to turn.
			  * \param Servo: the id of the servo to move (in our ATMEGA16, from 0 to 2).
			  * \param Fast: indicates if the servo must reach the angle at maximum speed
			  * \return Whether or not the procedure succeded.
			  */
			bool setAngleWithFilter( double angle, const uint8_t servo = 0, bool fast = false );

			/** Disables the servo so the neck will be loose.
			  * \param Servo: the id of the servo to move (in our ATMEGA16, from 0 to 2).
			  * \return Whether or not the procedure succeded.
			  */
			bool disableServo( const uint8_t servo = 0 );

			/** Enables the servo so the neck will be tight.
			  * \param Servo: the id of the servo to move (in our ATMEGA16, from 0 to 2).
			  * \return Whether or not the procedure succeded.
			  */
			bool enableServo( const uint8_t servo = 0 );

			/** Centers the servo at zero position
			  */
			bool center( const uint8_t servo = 0 );

			/** Gets the truncate factor of the turn
			  */
			double getTruncateFactor(){ return m_TruncateFactor; }

			/** Gets the truncate factor of the turn
			  */
			void setTruncateFactor( const double factor ){ ASSERT_( factor > 0 && factor < 1 ); m_TruncateFactor = factor; }

			/** Gets the truncate factor of the turn
			  */
			void setNumberOfPreviousAngles( const unsigned int number ){ m_NumPrevAngles = number; }

			/** Gets the truncate factor of the turn
			  */
			unsigned int getNumberOfPreviousAngles(){ return m_NumPrevAngles; }

		protected:
			std::string	m_usbSerialNumber;				//!< A copy of the device serial number (to open the USB FTDI chip).
			double m_MaxValue;							//!< The value set in the ICR register within the ATMEGA16 controller.
			double m_TruncateFactor;					//!< The range of turn of the servo will be truncated to "+-m_truncate_factor*(pi/2)".
			std::deque<double> m_PrevAngles;			//!< A vector containing the last N angles which where passed to the servo (for averaging)
			unsigned int m_NumPrevAngles;				//!< Number of previous angles to store for averaging

			bool setRegisterValue( const uint16_t value, const uint8_t servo = 0, bool fast = false );
			bool getRegisterValue( uint16_t &value, const uint8_t servo = 0 );

		private:
			/** Converts from a decimal angle (in radians) to the corresponding register value for the ATMEGA16 controller (for inner use only).
			  * \param The angle to convert.
			  * \return The value of the register to send.
			  */
			unsigned int angle2RegValue( const double angle );						// Angle in rad

			/** Converts from a certain value of the ATMEGA16 PWM register to the corresponding decimal angle (for inner use only).
			  * \param The value to convert.
			  * \return The corresponding angle.
			  */
			double regValue2angle( const uint16_t value );

			/** Tries to connect to the USB device (if disconnected).
			  * \return True on connection OK, false on error.
			  */
			bool checkConnectionAndConnect();

		};	// End of class

	} // End of namespace

} // End of namespace

#endif

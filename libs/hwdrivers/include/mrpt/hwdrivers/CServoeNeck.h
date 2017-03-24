/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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

			/** Turns the servo up to the specified angle (in radians in the range -pi,pi, other values will be saturated to the maximum or the mininum)
			  * \param Angle: the desired angle to turn.
			  * \param Servo: the id of the servo to move (in our ATMEGA16, from 0 to 2).
			  * \param Speed: indicates the speed of the servo
			  * \return Whether or not the procedure succeded.
			  */
			bool setAngleAndSpeed( double angle, const uint8_t servo, const uint8_t speed );

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

			/**Load the Offset values for each servo
			 */
			void setOffsets(float offset0, float offset1, float offset2);

		protected:
			std::string	m_usbSerialNumber;				//!< A copy of the device serial number (to open the USB FTDI chip).
			double m_MaxValue;							//!< The value set in the ICR register within the ATMEGA16 controller.
			double m_TruncateFactor;					//!< The range of turn of the servo will be truncated to "+-m_truncate_factor*(pi/2)".
			std::deque<double> m_PrevAngles;			//!< A vector containing the last N angles which where passed to the servo (for averaging)
			unsigned int m_NumPrevAngles;				//!< Number of previous angles to store for averaging
			std::vector<float> m_offsets;					//!< The offset used for each servo

			bool setRegisterValue( const uint16_t value, const uint8_t servo = 0, bool fast = false );
			bool setRegisterValueAndSpeed( const uint16_t value, const uint8_t servo, const uint16_t speed );
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

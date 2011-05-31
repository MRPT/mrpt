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

#include <mrpt/hwdrivers.h> // Precompiled headers

#include <mrpt/utils/net_utils.h>

const double MAX_VALUE = 10000;					// ICR value in the ATMEGA16

using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::hwdrivers;
using namespace mrpt::math;

/*-------------------------------------------------------------
					default constructor
-------------------------------------------------------------*/
CServoeNeck::CServoeNeck() :
	m_usbSerialNumber("eNeck001"),
	m_MaxValue(10000),
	m_TruncateFactor(0.5),
	m_PrevAngles(0),
	m_NumPrevAngles(5)
{
} // end-constructor
/*-------------------------------------------------------------
					default destructor
-------------------------------------------------------------*/
CServoeNeck::~CServoeNeck()
{}

/*-------------------------------------------------------------
					queryFirmwareVersion
-------------------------------------------------------------*/
bool CServoeNeck::queryFirmwareVersion( string &out_firmwareVersion )
{
	try
	{
		utils::CMessage		msg,msgRx;

		// Try to connect to the device:
		if (!checkConnectionAndConnect())	return false;

		msg.type = 0x10;
		sendMessage( msg );

		if (receiveMessage( msgRx ) )
		{
			msgRx.getContentAsString( out_firmwareVersion );
			mrpt::system::sleep(200);
			return true;
		}
		else
			return false;
	}
	catch(...)
	{
		Close();
		return false;
	}
} // end-queryFirmwareVersion

/*-------------------------------------------------------------
					angle2RegValue
-------------------------------------------------------------*/
unsigned int CServoeNeck::angle2RegValue( const double angle /* rad */ )
{
	return ( (uint16_t)( (m_MaxValue/20)*(-2*angle/M_PI+1.5) ) );			// equation: v = s*(a-a0) + v0 where s = 450/pi; a0 = 0 and v0 = 750
	//return ( (uint16_t)( (m_MaxValue/20)*(2*angle/M_PI+1.5) ) );			// equation: v = s*(a-a0) + v0 where s = 450/pi; a0 = 0 and v0 = 750
	//return ( (uint16_t)( (900/M_PI)*nangle + 750 ) );			// equation: v = s*(a-a0) + v0 where s = 450/pi; a0 = 0 and v0 = 750
} // end-angle2RegValue

/*-------------------------------------------------------------
					regValue2angle
-------------------------------------------------------------*/
double CServoeNeck::regValue2angle( const uint16_t value )
{
	return ( -M_PI*0.5*(20*value/m_MaxValue-1.5) );					// equation: angle = (pi/2)*(20*OCR/ICR-1.5)
	// return ( M_PI*0.5*(20*value/m_MaxValue-1.5) );					// equation: angle = (pi/2)*(20*OCR/ICR-1.5)
	// return ( (value-750)/(900/M_PI) );
} // end-regValue2angle

/*-------------------------------------------------------------
					setRegisterValue
-------------------------------------------------------------*/
bool CServoeNeck::setRegisterValue( const uint16_t value, const uint8_t servo, bool fast )
{
	try
	{
		if (!isOpen())
			return false;

		utils::CMessage		msg,msgRx;

		// Send cmd for setting the value of the register:
		// ------------------------------------------------
		if( fast )
			msg.type = 0x15;
		else
			msg.type = 0x11;
		msg.content.resize( 3 );
		msg.content[2] = (uint8_t)value;		// Low byte
		msg.content[1] = (uint8_t)(value>>8);	// High byte
		msg.content[0] = servo;					// Servo number

		sendMessage(msg);
		if (!receiveMessage(msgRx) )
			return false;	// Error

		mrpt::system::sleep(200);
		return true;
	}
	catch(...)
	{
		// Error opening device:
		Close();
		return false;
	}

} // end-setRegisterValue

/*-------------------------------------------------------------
					getRegisterValue
-------------------------------------------------------------*/
bool CServoeNeck::getRegisterValue( uint16_t &value, const uint8_t servo )
{
	try
	{
		if (!isOpen())
			return false;

		utils::CMessage		msg,msgRx;

		// Send cmd for obtaining the value of the OCR1A register:
		// --------------------------------------------------------
		msg.type = 0x12;
		msg.content.resize( 1 );
		msg.content[0] = servo;

		sendMessage( msg );
		if (receiveMessage(msgRx) )
		{
			if ( msgRx.content.size() != 2 )
				return false;

			value = (msgRx.content[0] << 8) + msgRx.content[1];
			return true;
		}
		else
			return false;

		return true;
	}
	catch(...)
	{
		// Error opening device:
		Close();
		return false;
	}

} // end-getRegisterValue

/*-------------------------------------------------------------
					getCurrentAngle
-------------------------------------------------------------*/
bool CServoeNeck::getCurrentAngle( double &angle, const uint8_t servo )
{
	uint16_t value;
	if( getRegisterValue( value, servo ) )
	{
		angle = regValue2angle( value );
		return true;
	}
	else
		return false;
} // end-getCurrentAngle

/*-------------------------------------------------------------
					setAngle
-------------------------------------------------------------*/
bool CServoeNeck::setAngle( double angle, const uint8_t servo, bool fast )
{
	//double nangle = -angle;

	//if( nangle < -m_TruncateFactor*M_PI/2 )	nangle = -m_TruncateFactor*M_PI/2;
	//if( nangle > m_TruncateFactor*M_PI/2 )		nangle = m_TruncateFactor*M_PI/2;

	//unsigned int reg = angle2RegValue( nangle );

	//std::cout << "Angle: " << RAD2DEG( nangle ) << " - Reg: " << reg << std::endl;
	//return setRegisterValue( reg, servo );

	if( angle < -m_TruncateFactor*M_PI/2 )	angle = -m_TruncateFactor*M_PI/2;
	if( angle > m_TruncateFactor*M_PI/2 )	angle = m_TruncateFactor*M_PI/2;

	unsigned int reg = angle2RegValue( angle );

	std::cout << "Angle: " << RAD2DEG( angle ) << " - Reg: " << reg << std::endl;
	return setRegisterValue( reg, servo, fast );

} // end-getCurrentAngle

/*-------------------------------------------------------------
					setAngleWithFilter
-------------------------------------------------------------*/
bool CServoeNeck::setAngleWithFilter( double angle, const uint8_t servo, bool fast )
{
	double nangle = 0;
	if( m_PrevAngles.size() == m_NumPrevAngles && m_NumPrevAngles != 0)	// If the deque is full populated
		m_PrevAngles.erase( m_PrevAngles.begin() );						// Erase the first angle of the deque

	m_PrevAngles.push_back( angle );									// Push back the new angle

	std::deque<double>::iterator it;
	for( it = m_PrevAngles.begin(); it != m_PrevAngles.end(); it++ )	// Sum up all the elements in the deque
		nangle += *it;
	nangle /= m_PrevAngles.size();										// Mean angle

	return ( setAngle( nangle, servo, fast ) );
}

/*-------------------------------------------------------------
					disableServo
-------------------------------------------------------------*/
bool CServoeNeck::disableServo( const uint8_t servo )
{
	try
	{
		if (!isOpen())
			return false;

		utils::CMessage		msg,msgRx;

		// Send cmd for disabling servo:
		// ----------------------------
		msg.type = 0x13;
		msg.content.resize( 1 );
		msg.content[0] = servo;					// Servo number

		sendMessage(msg);
		if (!receiveMessage(msgRx) )
			return false;	// Error

		return true;
	}
	catch(...)
	{
		// Error opening device:
		Close();
		return false;
	}

} // end-getCurrentAngle

/*-------------------------------------------------------------
					enableServo
-------------------------------------------------------------*/
bool CServoeNeck::enableServo( const uint8_t servo )
{
	try
	{
		if (!isOpen())
			return false;

		utils::CMessage		msg,msgRx;

		// Send cmd for enabling the servo:
		// --------------------------------
		msg.type = 0x14;
		msg.content.resize( 1 );
		msg.content[0] = servo;					// Servo number

		sendMessage(msg);
		if (!receiveMessage(msgRx) )
			return false;	// Error

		return true;
	}
	catch(...)
	{
		// Error opening device:
		Close();
		return false;
	}

} // end-getCurrentAngle

/*-------------------------------------------------------------
						Center
-------------------------------------------------------------*/
bool CServoeNeck::center( const uint8_t servo )
{
	unsigned int value = angle2RegValue( 0 );
	return setRegisterValue( value, servo  );
} // end-Center

/*-------------------------------------------------------------
					checkConnectionAndConnect
-------------------------------------------------------------*/
bool CServoeNeck::checkConnectionAndConnect()
{
	if( isOpen() )
		return true;

	try
	{
		OpenBySerialNumber( m_usbSerialNumber );
		mrpt::system::sleep(10);
		Purge();
		mrpt::system::sleep(10);
		SetLatencyTimer(1);
		SetTimeouts(300,100);

		return true;
	}
	catch(...)
	{
		// Error opening device:
		Close();
		return false;
	}
} // end-checkConnectionAndConnect

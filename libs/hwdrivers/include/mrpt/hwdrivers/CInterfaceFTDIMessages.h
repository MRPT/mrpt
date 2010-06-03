/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#ifndef CInterfaceFTDIMessages_H
#define CInterfaceFTDIMessages_H

#include <mrpt/hwdrivers/CInterfaceFTDI.h>
#include <mrpt/utils/CMessage.h>

namespace mrpt
{
	namespace hwdrivers
	{

		/** An implementation of message passing over a FTDI USB link.
		 *  	The limit for the body size is 255 bytes. The frame format is an array of bytes, in this order:
		  \code
			<START_FLAG> <HEADER> <LENGTH> <BODY> <END_FLAG>

				<START_FLAG> 	= 0x69
				<HEADER> 		= A header byte
				<LENGHT>		= Number of bytes of BODY
				<BODY>			= N x bytes
				<END_FLAG>		= 0X96
					 Total length 	= 	<LENGTH> + 4
		  \endcode
		 * \sa CInterfaceFTDI
		 */
		class HWDRIVERS_IMPEXP CInterfaceFTDIMessages : public hwdrivers::CInterfaceFTDI
		{
		public:
			/** Constructor
			  */
			CInterfaceFTDIMessages();

			/** Destructor
			  */
			virtual ~CInterfaceFTDIMessages();

			/** Send a message to the device.
			  *  Note that only the low byte from the "type" field will be used.
			  * \exception std::exception On communication errors
			  */
			void  sendMessage( const utils::CMessage &msg);

			/** Tries to receive a message from the device.
			  * \exception std::exception On communication errors
			  * \returns True if successful, false if there is no new data from the device (but communications seem to work fine)
			  */
			bool  receiveMessage( utils::CMessage &msg );
		}; // end of class

	} // end of namespace
} // end of namespace

#endif



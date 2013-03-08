/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#ifndef CInterfaceNI845x_H
#define CInterfaceNI845x_H

#include <mrpt/config.h>
#include <mrpt/hwdrivers/link_pragmas.h>
#include <mrpt/utils/CUncopiable.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** A generic C++ wrapper for the specific API of the SPI/I2C board "National Instruments USB 8451/8452"
		 *
		 * \ingroup mrpt_hwdrivers_grp
		 */
		class HWDRIVERS_IMPEXP CInterfaceNI845x : public mrpt::utils::CUncopiable
		{
		public:
			/** Constructor (doesn't connect to any device at this point). */
			CInterfaceNI845x();

			/** Destructor */
			virtual ~CInterfaceNI845x();

			/** Opens the i'th device connected to the system (0=first one) \exception std::exception If any error was found */
			void open(const size_t deviceIdx = 0);

			/** Opens the device with the given "resource name" \exception std::exception If any error was found */
			void open(const std::string &resourceName);

			bool isOpen() const; //!< Check whether the device is correctly open.
			std::string getDeviceDescriptor() const; //!< Get the descriptor of the device to which this object has been connected.

			void close(); //!< Close the connection (there's no need to explicitly call this, it is called anyway at destructor).

			/** Changes the IO voltage. Accepted values are: 12 (1.2), 15 (1.5), 18 (1.8), 25 (2.5), 33 (3.3)
			  * \exception std::exception If any error was found
			  */
			void setIOVoltageLevel(const int volt); 

			/** Change port direction (each bit=1:output, =0:input) */
			void setIOPortDirection(const uint8_t port, const uint8_t dir);

			/** Write port */
			void writeIOPort(const uint8_t port, const uint8_t value);
			/** Read port */
			uint8_t readIOPort(const uint8_t port);

			void create_SPI_configurations(size_t num_configurations);
			/** Must call reserveSPI_configurations() first to reserve configuration blocks, whose indices are selected with config_idx  */
			void set_SPI_configuration(size_t config_idx, uint8_t  chip_select_line, uint16_t clock_speed_Khz, bool clock_polarity_idle_low, bool clock_phase_first_edge );

			/** Performs one SPI transaction  */
			void read_write_SPI(size_t config_idx, size_t num_write_bytes, const uint8_t *write_data, size_t &out_read_bytes, uint8_t * read_data );


			void deviceLock();   //!< Lock (for multi-threading apps)
			void deviceUnlock();   //!< Unlock (for multi-threading apps)


		private:
			void checkErr(int errCode); 
			void* m_niDevHandle; //!< opaque handler
			void* m_niSPIConfHandles; //!< opaque handler
			size_t m_niSPIConfHandlesCount;
			std::string m_deviceDescr; //!< The descriptor of the open device

			void close_SPI_configurations(); //!< Closes and free these structs


		}; // end of class

	} // end of namespace
} // end of namespace
#endif

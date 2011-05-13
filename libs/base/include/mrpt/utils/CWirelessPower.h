/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Emil Khatib  <emilkhatib@gmail.com>	                       |
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
#ifndef  CWIRELESSPOWER_H
#define  CWIRELESSPOWER_H

#include <mrpt/utils/utils_defs.h>
#include <string.h>
#include <vector>

namespace mrpt
{
namespace utils
{
	/** This class implements a wireless power probe.
	 */
	class BASE_IMPEXP CWirelessPower
	{
	private:
		/** SSID of the WiFi network
		 */
		std::string ssid;

		/** GUID of the WiFi interface
		 */
		std::string guid;
		
		/** Handle to the WLAN server (Windows)
		 */
		void* hClient;
		
	public:
		/** Default constructor.
		 */
		CWirelessPower(){};

		/** Set the SSID and GUID of the target network.
		 * \exception std::exception In case there is a failure
		 * \param ssid_ SSID of the target network
		 * \param guid_ GUID of the network interface
		 */
		void setNet(std::string ssid_, std::string guid_);


		/** Gets a list of the interfaces
		 * \exception std::exception In case there is a failure
		 * \return std::vector returns the identifiers (GUID) of the available interfaces 
		 */
		std::vector<std::string>	ListInterfaces();


		/** Gets the power of a given network
		 * \exception std::exception In case there is a failure
		 * \return Returns the power (0-100).
		 */
		int		GetPower();

		/** Gets a list of the networks available for an interface
		 * \exception std::exception In case there is a failure
		 * \return std::vector returns handles to the available networks of a given interface
		 */
		std::vector<std::string>	ListNetworks();

	}; // End of class def.

	} // End of namespace
} // End of namespace
#endif
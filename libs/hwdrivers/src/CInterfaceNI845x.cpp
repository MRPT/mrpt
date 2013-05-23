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

#include <mrpt/hwdrivers.h> // Precompiled headers

#include <mrpt/hwdrivers/CInterfaceNI845x.h>

#if MRPT_HAS_NI845x
#	if (MRPT_WORD_SIZE==64) && !defined(WIN64)
#		define WIN64
#	endif
#	include "ni845x.h"  // Include file for NI-485x functions and constants
#endif

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::synch;

#if MRPT_HAS_NI845x
#	define DEV_HANDLER  reinterpret_cast<NiHandle*>(m_niDevHandle)
#	define CONF_HANDLERS  reinterpret_cast<NiHandle*>(m_niSPIConfHandles)
#	define I2C_CONF_HANDLERS  reinterpret_cast<NiHandle*>(m_niI2CConfHandles)
#endif

// Ctor
CInterfaceNI845x::CInterfaceNI845x() : 
#if MRPT_HAS_NI845x
	m_niDevHandle     ( malloc(sizeof(NiHandle)) ),
#else
	m_niDevHandle( NULL ),
#endif
	m_niSPIConfHandles ( NULL ),
	m_niSPIConfHandlesCount(0),
	m_niI2CConfHandles (NULL),
	m_niI2CConfHandlesCount(0)
{
#if MRPT_HAS_NI845x
	*DEV_HANDLER = 0;
#endif
}

// Dtor
CInterfaceNI845x::~CInterfaceNI845x()
{
	this->close();

	if (m_niDevHandle) 
	{
		free(m_niDevHandle);
		m_niDevHandle=NULL;
	}
}

void CInterfaceNI845x::checkErr(int errCode)
{
#if MRPT_HAS_NI845x
	if (errCode<0)
	{
		char auxStr[1024];
		ni845xStatusToString(errCode, sizeof (auxStr), auxStr);
		THROW_EXCEPTION_CUSTOM_MSG1("Device error: %s",auxStr)
	}
#endif
}

/** Opens the i'th device connected to the system (0=first one) \return true on success */
void CInterfaceNI845x::open(const size_t deviceIdx)
{
#if MRPT_HAS_NI845x
	if (isOpen()) this->close();

	NiHandle hSearch=0;
	uInt32   nDevFound=0;
	char     devDescr[260];

	/* find first device */
	size_t dev_idx = 0;
	checkErr( ni845xFindDevice(devDescr, &hSearch, &nDevFound) );

	if (deviceIdx>nDevFound)
	{
		THROW_EXCEPTION( mrpt::format("Error: Cannot open the device #%u because only %u were found.",static_cast<unsigned int>(deviceIdx),static_cast<unsigned int>(nDevFound)  ) )
	}

	while (dev_idx<deviceIdx)
	{
		dev_idx++;
		checkErr( ni845xFindDeviceNext(nDevFound,devDescr) );
		if (!strlen(devDescr))
			THROW_EXCEPTION( mrpt::format("Error: Cannot open the device #%u (enumeration stops before reaching that index).",static_cast<unsigned int>(deviceIdx) ) )
	}

	ni845xCloseFindDeviceHandle(hSearch);

	this->open( std::string(devDescr) );

#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}

/** Opens the device with the given "resource name" \return true on success */
void CInterfaceNI845x::open(const std::string &resourceName)
{
#if MRPT_HAS_NI845x
	if (isOpen()) this->close();

	checkErr( ni845xOpen(const_cast<char*>(resourceName.c_str()), DEV_HANDLER ) );
	m_deviceDescr = resourceName;

#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}

// Check whether the device is correctly open.
bool CInterfaceNI845x::isOpen() const 
{
#if MRPT_HAS_NI845x
	return DEV_HANDLER &&  *DEV_HANDLER!=0;
#else
	return false;
#endif
}

//!< Close the connection (there's no need to explicitly call this, it is called anyway at destructor).
void CInterfaceNI845x::close()
{
#if MRPT_HAS_NI845x
	if (!isOpen()) return;

	// Close configurations:
	this->close_SPI_configurations();
	this->close_I2C_configurations();

	// Close device:
	ni845xClose(*DEV_HANDLER);
	*DEV_HANDLER = 0;
	m_deviceDescr.clear();
#endif
}

void CInterfaceNI845x::close_SPI_configurations()
{
#if MRPT_HAS_NI845x
	if (m_niSPIConfHandles && m_niSPIConfHandlesCount)
	{
		for (size_t i=0;i<m_niSPIConfHandlesCount;i++)
		{
			ni845xSpiConfigurationClose(CONF_HANDLERS[i]);
			CONF_HANDLERS[i]=0;
		}
		delete[] m_niSPIConfHandles;
		m_niSPIConfHandles=NULL;
	}
	m_niSPIConfHandlesCount=0;
#endif
}


std::string CInterfaceNI845x::getDeviceDescriptor() const
{
	return m_deviceDescr;	
}

/** Changes the IO voltage. Accepted values are: 1.2, 1.5, 1.8, 2.5, 3.3
	* \exception std::exception If any error was found
	*/
void CInterfaceNI845x::setIOVoltageLevel(const int volt)
{
	if (!isOpen()) THROW_EXCEPTION("Device is not open!")
#if MRPT_HAS_NI845x
	checkErr( ni845xSetIoVoltageLevel(*DEV_HANDLER,volt) );
#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}


void CInterfaceNI845x::deviceLock()
{
	if (!isOpen()) THROW_EXCEPTION("Device is not open!")
#if MRPT_HAS_NI845x
	checkErr( ni845xDeviceLock(*DEV_HANDLER) );
#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}

void CInterfaceNI845x::deviceUnlock()
{
	if (!isOpen()) THROW_EXCEPTION("Device is not open!")
#if MRPT_HAS_NI845x
	checkErr( ni845xDeviceUnlock(*DEV_HANDLER) );
#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}

/** Change port direction (each bit=1:output, =0:input) */
void CInterfaceNI845x::setIOPortDirection(const uint8_t port, const uint8_t dir)
{
	if (!isOpen()) THROW_EXCEPTION("Device is not open!")
#if MRPT_HAS_NI845x
	checkErr( ni845xDioSetPortLineDirectionMap(*DEV_HANDLER,port,dir) );
#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}

/** Write port */
void CInterfaceNI845x::writeIOPort(const uint8_t port, const uint8_t value)
{
	if (!isOpen()) THROW_EXCEPTION("Device is not open!")
#if MRPT_HAS_NI845x
	checkErr( ni845xDioWritePort(*DEV_HANDLER,port,value) );
#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}

/** Read port */
uint8_t CInterfaceNI845x::readIOPort(const uint8_t port)
{
	if (!isOpen()) THROW_EXCEPTION("Device is not open!")
#if MRPT_HAS_NI845x
	uInt8 data;
	checkErr( ni845xDioReadPort(*DEV_HANDLER,port,&data) );
	return data;
#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}

void CInterfaceNI845x::create_SPI_configurations(size_t num_configurations)
{
#if MRPT_HAS_NI845x
	this->close_SPI_configurations();
	
	m_niSPIConfHandles = new NiHandle[num_configurations];
	m_niSPIConfHandlesCount = num_configurations;

	// Create one SPI configuration:
	for (size_t i=0;i<num_configurations;i++)
		checkErr( ni845xSpiConfigurationOpen(&CONF_HANDLERS[i]) );
	
#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}
/** Must call reserveSPI_configurations() first to reserve configuration blocks, whose indices are selected with config_idx  */
void CInterfaceNI845x::set_SPI_configuration(size_t config_idx, uint8_t  chip_select_line, uint16_t clock_speed_Khz, bool clock_polarity_idle_low, bool clock_phase_first_edge )
{
#if MRPT_HAS_NI845x
	NiHandle hConf = CONF_HANDLERS[config_idx];

	checkErr (ni845xSpiConfigurationSetChipSelect (hConf, chip_select_line ));
	checkErr (ni845xSpiConfigurationSetClockRate (hConf,clock_speed_Khz));
	checkErr (ni845xSpiConfigurationSetClockPolarity (hConf, clock_polarity_idle_low ? kNi845xSpiClockPolarityIdleLow : kNi845xSpiClockPolarityIdleHigh ));
	checkErr (ni845xSpiConfigurationSetClockPhase (hConf, clock_phase_first_edge ? kNi845xSpiClockPhaseFirstEdge : kNi845xSpiClockPhaseSecondEdge));
	checkErr (ni845xSpiConfigurationSetNumBitsPerSample(hConf, 8 ) );
#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}

/** Performs one SPI transaction  */
void CInterfaceNI845x::read_write_SPI(size_t config_idx, size_t num_write_bytes, const uint8_t *write_data, size_t &out_read_bytes, uint8_t * read_data  )
{
#if MRPT_HAS_NI845x
	uInt32 nRead;
	checkErr (ni845xSpiWriteRead (*DEV_HANDLER, CONF_HANDLERS[config_idx], num_write_bytes,const_cast<uInt8*>(write_data), &nRead, read_data));
	out_read_bytes = nRead;
#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}

void CInterfaceNI845x::create_I2C_configurations(size_t num_configurations)
{
#if MRPT_HAS_NI845x
	this->close_I2C_configurations();
	
	m_niI2CConfHandles = new NiHandle[num_configurations];
	m_niI2CConfHandlesCount = num_configurations;

	// Create one I2C configuration:
	for (size_t i=0;i<num_configurations;i++)
		checkErr( ni845xI2cConfigurationOpen(&I2C_CONF_HANDLERS[i]) );
#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}

/** Must call create_I2C_configurations() first to reserve configuration blocks, whose indices are selected with config_idx  */
void CInterfaceNI845x::set_I2C_configuration(
	size_t config_idx, 
	uint16_t address, 
	uint16_t  timeout, 
	int32_t addr_size, 
	uint16_t clock_rate_khz, 
	uint8_t I2C_port )
{
#if MRPT_HAS_NI845x
	NiHandle hConf = I2C_CONF_HANDLERS[config_idx];

	checkErr (ni845xI2cConfigurationSetAckPollTimeout (hConf, timeout ));
	checkErr (ni845xI2cConfigurationSetAddress (hConf, address ));
	checkErr (ni845xI2cConfigurationSetAddressSize (hConf, addr_size==7 ? kNi845xI2cAddress7Bit : kNi845xI2cAddress10Bit  ));
	checkErr (ni845xI2cConfigurationSetClockRate (hConf, clock_rate_khz ));
	checkErr (ni845xI2cConfigurationSetPort (hConf, I2C_port ));

#else
	THROW_EXCEPTION("MRPT was compiled without support for this device")
#endif
}

void CInterfaceNI845x::close_I2C_configurations() //!< Closes and free these struct
{
#if MRPT_HAS_NI845x
	if (m_niI2CConfHandles && m_niI2CConfHandlesCount)
	{
		for (size_t i=0;i<m_niI2CConfHandlesCount;i++)
		{
			ni845xI2cConfigurationClose(I2C_CONF_HANDLERS[i]);
			I2C_CONF_HANDLERS[i]=0;
		}
		delete[] m_niI2CConfHandles;
		m_niI2CConfHandles=NULL;
	}
	m_niI2CConfHandlesCount=0;
#endif
}

/** Read I2C \return false on any error */
void CInterfaceNI845x::read_I2C(size_t config_idx, size_t num_bytes_to_read, size_t & num_bytes_read, uint8_t * read_data )
{
#if MRPT_HAS_NI845x
	uInt32 nRead;
	checkErr (ni845xI2cRead(*DEV_HANDLER, I2C_CONF_HANDLERS[config_idx], num_bytes_to_read,&nRead, read_data));
	num_bytes_read = nRead;
#endif
}
			
/** Write I2C \return false on any error */
void CInterfaceNI845x::write_I2C(size_t config_idx, size_t num_bytes_to_write, const uint8_t * write_data )
{
#if MRPT_HAS_NI845x
	checkErr (ni845xI2cWrite(*DEV_HANDLER, I2C_CONF_HANDLERS[config_idx], num_bytes_to_write, const_cast<uint8_t*>(write_data) ));
#endif
}

/** Write+read I2C */
void CInterfaceNI845x::write_read_I2C(size_t config_idx, size_t num_bytes_to_write, const uint8_t * write_data, size_t num_bytes_to_read, size_t & num_bytes_read, uint8_t * read_data )
{
#if MRPT_HAS_NI845x
	uInt32 nRead;
	checkErr (ni845xI2cWriteRead(*DEV_HANDLER, I2C_CONF_HANDLERS[config_idx], num_bytes_to_write, const_cast<uint8_t*>(write_data), num_bytes_to_read,&nRead, read_data));
	num_bytes_read = nRead;
#endif
}

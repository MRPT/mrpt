/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CNTRIPEmitter.h>

#include <mrpt/system/string_utils.h>

IMPLEMENTS_GENERIC_SENSOR(CNTRIPEmitter,mrpt::hwdrivers)

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;

/*-------------------------------------------------------------
						CNTRIPEmitter
-------------------------------------------------------------*/
CNTRIPEmitter::CNTRIPEmitter() :
	m_client(),
#ifdef MRPT_OS_WINDOWS
	m_com_port("COM1"),
#else
	m_com_port("ttyUSB0"),
#endif
	m_com_bauds(38400),
	m_transmit_to_server(true)
{

}

/*-------------------------------------------------------------
						~CNTRIPEmitter
-------------------------------------------------------------*/
CNTRIPEmitter::~CNTRIPEmitter()
{
	m_client.close();
	if (m_out_COM.isOpen()) m_out_COM.close();
}

/*-------------------------------------------------------------
						doProcess
-------------------------------------------------------------*/
void CNTRIPEmitter::doProcess()
{
	vector_byte  buf;
	m_client.stream_data.readAndClear(buf);

	if (!buf.empty())
	{
		// Send through the serial port:
		cout << format("[NTRIP %s] RX: %u bytes\n", mrpt::system::timeLocalToString(mrpt::system::now()).c_str(),(unsigned) buf.size() );
		m_out_COM.WriteBuffer(&buf[0],buf.size());
	}

	// Try to read a msg from the receiver -> NTRIP caster
	if (m_transmit_to_server)
	{
		char rxbuf[50];
		const size_t nReadActual = m_out_COM.Read(rxbuf,sizeof(rxbuf)-1);
		if (nReadActual)
		{
			rxbuf[nReadActual] = 0;
			if (m_verbose) cout << format("[NTRIP %s] TX (%u bytes)\n", mrpt::system::timeLocalToString(mrpt::system::now()).c_str(), (unsigned int)nReadActual);
		}
	}

	mrpt::system::sleep(1);
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
void CNTRIPEmitter::initialize()
{
	if (m_out_COM.isOpen()) m_out_COM.close();

	cout << format("[NTRIP] Opening %s...\n",m_com_port.c_str() );
	m_out_COM.open(m_com_port);
	m_out_COM.setConfig(m_com_bauds);
	m_out_COM.setTimeouts(0,0,10,0,1);
	m_out_COM.purgeBuffers();
	cout << format("[NTRIP] Open %s Ok.\n",m_com_port.c_str() );

	string errstr;
	if (!m_client.open(m_ntrip_args,errstr))
		THROW_EXCEPTION_CUSTOM_MSG1("ERROR trying to connect to NTRIP caster: %s",errstr.c_str());
}


/* -----------------------------------------------------
                loadConfig_sensorSpecific
   ----------------------------------------------------- */
void  CNTRIPEmitter::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
#ifdef MRPT_OS_WINDOWS
	m_com_port = configSource.read_string(iniSection, "COM_port_WIN", m_com_port, true );
#else
	m_com_port = configSource.read_string(iniSection, "COM_port_LIN", m_com_port, true );
#endif

	m_com_bauds = configSource.read_int( iniSection, "baudRate",m_com_bauds, true );

	m_transmit_to_server = configSource.read_bool( iniSection, "transmit_to_server",m_transmit_to_server);

	m_ntrip_args.mountpoint = mrpt::system::trim( configSource.read_string(iniSection, "mountpoint","",true) );
	m_ntrip_args.server     = mrpt::system::trim( configSource.read_string(iniSection, "server","",true) );
	m_ntrip_args.port       = configSource.read_int(iniSection, "port",2101,true);

	m_ntrip_args.user = mrpt::system::trim( configSource.read_string(iniSection, "user","") );
	m_ntrip_args.password = mrpt::system::trim( configSource.read_string(iniSection, "password","") );
}


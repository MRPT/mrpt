/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

#include <mrpt/hwdrivers/CNTRIPEmitter.h>

#include <mrpt/system/string_utils.h>

IMPLEMENTS_GENERIC_SENSOR(CNTRIPEmitter,mrpt::hwdrivers)

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
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
	m_com_bauds(38400)
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

	mrpt::system::sleep(50);
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

	m_ntrip_args.mountpoint = mrpt::system::trim( configSource.read_string(iniSection, "mountpoint","",true) );
	m_ntrip_args.server     = mrpt::system::trim( configSource.read_string(iniSection, "server","",true) );
	m_ntrip_args.port       = configSource.read_int(iniSection, "port",2101,true);

	m_ntrip_args.user = mrpt::system::trim( configSource.read_string(iniSection, "user","") );
	m_ntrip_args.password = mrpt::system::trim( configSource.read_string(iniSection, "password","") );
}


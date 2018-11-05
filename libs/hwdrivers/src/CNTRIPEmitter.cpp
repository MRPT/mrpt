/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers

#include <mrpt/hwdrivers/CNTRIPEmitter.h>

#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <iostream>

IMPLEMENTS_GENERIC_SENSOR(CNTRIPEmitter, mrpt::hwdrivers)

using namespace std;
using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;

/*-------------------------------------------------------------
						CNTRIPEmitter
-------------------------------------------------------------*/
CNTRIPEmitter::CNTRIPEmitter() : m_client(), m_com_port("") {}
/*-------------------------------------------------------------
						~CNTRIPEmitter
-------------------------------------------------------------*/
CNTRIPEmitter::~CNTRIPEmitter()
{
	m_client.close();
	if (m_out_COM.isOpen()) m_out_COM.close();
	if (m_raw_output_file_stream.is_open()) m_raw_output_file_stream.close();
}

/*-------------------------------------------------------------
						doProcess
-------------------------------------------------------------*/
void CNTRIPEmitter::doProcess()
{
	std::vector<uint8_t> buf;
	m_client.stream_data.readAndClear(buf);

	if (!buf.empty())
	{
		if (m_verbose)
		{
			const double At = m_rate_timer.Tac();
			m_rate_count += buf.size();
			if (At > 5.0)
			{
				const double estim_rate_Bps = m_rate_count / At;
				cout << format(
					"[NTRIP %s] Rate: %.02f B/s\n",
					mrpt::system::timeLocalToString(mrpt::system::now())
						.c_str(),
					estim_rate_Bps);
				m_rate_timer.Tic();
				m_rate_count = 0;
			}

			cout << format(
				"[NTRIP %s] RX (%u bytes)\n",
				mrpt::system::timeLocalToString(mrpt::system::now()).c_str(),
				(unsigned int)buf.size());
		}
		if (m_out_COM.isOpen())
		{
			// Send through the serial port:
			cout << format(
				"[NTRIP %s] RX: %u bytes\n",
				mrpt::system::timeLocalToString(mrpt::system::now()).c_str(),
				(unsigned)buf.size());
			m_out_COM.Write(&buf[0], buf.size());
		}

		if (m_raw_output_file_stream.is_open())
		{
			m_raw_output_file_stream.write(
				reinterpret_cast<const char*>(&buf[0]), buf.size());
		}
	}

	// Try to read a msg from the receiver -> NTRIP caster
	if (m_transmit_to_server && m_out_COM.isOpen())
	{
		char rxbuf[50];
		const size_t nReadActual = m_out_COM.Read(rxbuf, sizeof(rxbuf) - 1);
		if (nReadActual)
		{
			rxbuf[nReadActual] = 0;
			if (m_verbose)
				cout << format(
					"[NTRIP %s] TX (%u bytes)\n",
					mrpt::system::timeLocalToString(mrpt::system::now())
						.c_str(),
					(unsigned int)nReadActual);
		}
	}

	std::this_thread::sleep_for(1ms);
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
void CNTRIPEmitter::initialize()
{
	if (m_out_COM.isOpen()) m_out_COM.close();

	if (!m_com_port.empty())
	{
		cout << format("[NTRIP] Opening %s...\n", m_com_port.c_str());
		m_out_COM.open(m_com_port);
		m_out_COM.setConfig(m_com_bauds);
		m_out_COM.setTimeouts(0, 0, 10, 0, 1);
		m_out_COM.purgeBuffers();
		cout << format("[NTRIP] Open %s Ok.\n", m_com_port.c_str());
	}

	if (m_raw_output_file_stream.is_open())
	{
		m_raw_output_file_stream.close();
	}

	if (!m_raw_output_file_prefix.empty())
	{
		const string fil = mrpt::system::fileNameStripInvalidChars(
			m_raw_output_file_prefix +
			mrpt::system::dateTimeLocalToString(mrpt::system::now()) +
			string(".bin"));
		m_raw_output_file_stream.open(
			fil.c_str(), std::ofstream::out | std::ofstream::binary);
		if (!m_raw_output_file_stream.is_open())
			THROW_EXCEPTION_FMT(
				"Error opening output raw file: `%s`", fil.c_str());
	}

	string errstr;
	if (!m_client.open(m_ntrip_args, errstr))
		THROW_EXCEPTION_FMT(
			"ERROR trying to connect to NTRIP caster: %s", errstr.c_str());
}

/* -----------------------------------------------------
				loadConfig_sensorSpecific
   ----------------------------------------------------- */
void CNTRIPEmitter::loadConfig_sensorSpecific(
	const mrpt::config::CConfigFileBase& c, const std::string& s)
{
#ifdef _WIN32
	m_com_port = c.read_string(s, "COM_port_WIN", "");
#else
	m_com_port = c.read_string(s, "COM_port_LIN", "");
#endif

	m_raw_output_file_prefix = c.read_string(s, "raw_output_file_prefix", "");

	ASSERTMSG_(
		!m_raw_output_file_prefix.empty() || !m_com_port.empty(),
		"At least one of either raw file output or serial COM file must be "
		"specified in configuration file!");

	if (!m_com_port.empty())
	{
		m_com_bauds = c.read_int(s, "baudRate", m_com_bauds, true);
	}

	m_transmit_to_server =
		c.read_bool(s, "transmit_to_server", m_transmit_to_server);

	m_ntrip_args.mountpoint =
		mrpt::system::trim(c.read_string(s, "mountpoint", "", true));
	m_ntrip_args.server =
		mrpt::system::trim(c.read_string(s, "server", "", true));
	m_ntrip_args.port = c.read_int(s, "port", 2101, true);

	m_ntrip_args.user = mrpt::system::trim(c.read_string(s, "user", ""));
	m_ntrip_args.password =
		mrpt::system::trim(c.read_string(s, "password", ""));
}

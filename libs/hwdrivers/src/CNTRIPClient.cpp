/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers

#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/comms/net_utils.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/hwdrivers/CNTRIPClient.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/system/string_utils.h>
#include <cstring>
#include <iostream>

using namespace mrpt;
using namespace mrpt::comms;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::math;
using namespace std;

/* --------------------------------------------------------
					CNTRIPClient
   -------------------------------------------------------- */
CNTRIPClient::CNTRIPClient() : m_thread(), m_args()
{
	m_thread = std::thread(&CNTRIPClient::private_ntrip_thread, this);
}

/* --------------------------------------------------------
					~CNTRIPClient
   -------------------------------------------------------- */
CNTRIPClient::~CNTRIPClient()
{
	this->close();
	if (m_thread.joinable())
	{
		m_thread_exit = true;
		m_thread.join();
	}
}

/* --------------------------------------------------------
					close
   -------------------------------------------------------- */
void CNTRIPClient::close()
{
	m_upload_data.clear();
	if (!m_thread_do_process) return;
	m_thread_do_process = false;
	m_sem_sock_closed.get_future().wait_for(500ms);
}

/* --------------------------------------------------------
					open
   -------------------------------------------------------- */
bool CNTRIPClient::open(const NTRIPArgs& params, string& out_errmsg)
{
	this->close();

	if (params.mountpoint.empty())
	{
		out_errmsg = "MOUNTPOINT cannot be empty.";
		return false;
	}
	if (params.server.empty())
	{
		out_errmsg = "Server address cannot be empty.";
		return false;
	}

	// Try to open it:
	m_waiting_answer_connection = true;
	m_answer_connection = connError;
	out_errmsg.clear();

	m_args = params;
	m_thread_do_process = true;

	// Wait until the thread tell us the initial result...
	if (m_sem_first_connect_done.get_future().wait_for(6s) ==
		std::future_status::timeout)
	{
		out_errmsg = "Timeout waiting thread response";
		return false;
	}

	switch (m_answer_connection)
	{
		case connOk:
			return true;
		case connError:
			out_errmsg = format(
				"Error trying to connect to server '%s'",
				params.server.c_str());
			return false;
		case connUnauthorized:
			out_errmsg = format(
				"Authentication failed for server '%s'", params.server.c_str());
			return false;

		default:
			out_errmsg = "UNKNOWN m_answer_connection!!";
			return false;
	}
}

/* --------------------------------------------------------
					THE WORKING THREAD
   -------------------------------------------------------- */
void CNTRIPClient::private_ntrip_thread()
{
	try
	{
		CClientTCPSocket my_sock;

		bool last_thread_do_process = m_thread_do_process;

		while (!m_thread_exit)
		{
			if (!m_thread_do_process)
			{
				if (my_sock.isConnected())
				{
					// Close connection:
					try
					{
						my_sock.close();
					}
					catch (...)
					{
					}
				}
				else
				{
					// Nothing to be done... just wait
				}

				if (last_thread_do_process)  // Let the waiting caller continue
					// now.
					m_sem_sock_closed.set_value();

				last_thread_do_process = m_thread_do_process;
				std::this_thread::sleep_for(100ms);
				continue;
			}

			last_thread_do_process = m_thread_do_process;

			// We have a mission to do here... is the channel already open??

			if (!my_sock.isConnected())
			{
				TConnResult connect_res = connError;

				std::vector<uint8_t> buf;
				try
				{
					// Nope, it's the first time: get params and try open the
					// connection:
					stream_data.clear();

					cout << format(
						"[CNTRIPClient] Trying to connect to %s:%i\n",
						m_args.server.c_str(), m_args.port);

					my_sock.connect(m_args.server, m_args.port);
					if (m_thread_exit) break;

					// Prepare HTTP request:
					// -------------------------------------------
					string req = format(
						"GET /%s HTTP/1.0\r\n", m_args.mountpoint.c_str());

					if (isalpha(m_args.server[0]))
						req += format("Host: %s\r\n", m_args.server.c_str());

					req += "User-Agent: NTRIP MRPT Library\r\n";
					req += "Accept: */*\r\n";
					req += "Connection: close\r\n";

					// Implement HTTP Basic authentication:
					// See:
					// http://en.wikipedia.org/wiki/Basic_access_authentication
					if (!m_args.user.empty())
					{
						string auth_str =
							m_args.user + string(":") + m_args.password;
						std::vector<uint8_t> v(auth_str.size());
						std::memcpy(&v[0], &auth_str[0], auth_str.size());

						string encoded_str;
						mrpt::system::encodeBase64(v, encoded_str);

						req += "Authorization: Basic ";
						req += encoded_str;
						req += "\r\n";
					}

					// End:
					req += "\r\n";
					// cout << req;

					// Send:
					my_sock.sendString(req);

					// Try to read the header of the response:
					size_t to_read_now = 30;
					buf.resize(to_read_now);
					size_t len =
						my_sock.readAsync(&buf[0], to_read_now, 4000, 200);

					buf.resize(len);

					if ((len != 0) && my_sock.isConnected())
						connect_res = connOk;
				}
				catch (std::exception&)
				{
					// cout << e.what() << endl;
					connect_res = connError;
				}

				// We are not disconnected yet, it's a good thing... anyway,
				// check the answer code:
				if (!buf.empty())
				{
					string resp;
					resp.resize(buf.size());
					std::memcpy(&resp[0], &buf[0], buf.size());

					if (resp.find(" 200 ") == string::npos)
					{
						// It's NOT a good response...
						connect_res = connError;

						// 401?
						if (resp.find(" 401 ") != string::npos)
							connect_res = connUnauthorized;
					}
				}

				// Signal my caller that the connection is established:
				// ---------------------------------------------------------------
				if (m_waiting_answer_connection)
				{
					m_waiting_answer_connection = false;

					m_answer_connection = connect_res;
					m_sem_first_connect_done.set_value();
				}

				if (connect_res != connOk) my_sock.close();
			}

			// Retry if it was a failed connection.
			if (!my_sock.isConnected())
			{
				std::this_thread::sleep_for(500ms);
				continue;
			}

			// Read data from the stream and accumulate it in a buffer:
			// ----------------------------------------------------------------------
			std::vector<uint8_t> buf;
			size_t to_read_now = 1000;
			buf.resize(to_read_now);
			size_t len = my_sock.readAsync(&buf[0], to_read_now, 10, 5);

			buf.resize(len);

			if (my_sock.isConnected())
			{
				// Send data to main buffer:
				if (stream_data.size() > 1024 * 8)
					stream_data.clear();  // It seems nobody's reading it...

				stream_data.appendData(buf);
				buf.clear();
			}

			// Send back data to the server, if so requested:
			// ------------------------------------------
			std::vector<uint8_t> upload_data;
			m_upload_data.readAndClear(upload_data);
			if (!upload_data.empty())
			{
				const size_t N = upload_data.size();
				const size_t nWritten =
					my_sock.writeAsync(&upload_data[0], N, 1000);
				if (nWritten != N)
					cerr << "*ERROR*: Couldn't write back " << N
						 << " bytes to NTRIP server!.\n";
			}

			std::this_thread::sleep_for(10ms);
		}  // end while

	}  // end try
	catch (exception& e)
	{
		cerr << "[CNTRIPClient] Exception in working thread: " << endl
			 << e.what() << endl;
	}
	catch (...)
	{
		cerr << "[CNTRIPClient] Runtime exception in working thread." << endl;
	}

}  // end working thread

/* --------------------------------------------------------
					retrieveListOfMountpoints
   -------------------------------------------------------- */
bool CNTRIPClient::retrieveListOfMountpoints(
	TListMountPoints& out_list, string& out_errmsg, const string& server,
	int port, const string& auth_user, const string& auth_pass)
{
	string content;
	net::HttpRequestOptions httpOptions;
	net::HttpRequestOutput httpOut;

	out_list.clear();

	httpOptions.port = port;
	httpOptions.auth_user = auth_user;
	httpOptions.auth_pass = auth_pass;
	httpOptions.timeout_ms = 6000;

	net::http_errorcode ret = net::http_get(
		string("http://") + server, content, httpOptions, httpOut);

	out_errmsg = httpOut.errormsg;

	// Parse contents:
	if (ret != net::http_errorcode::Ok) return false;

	std::stringstream ss(content);
	string lin;
	while (std::getline(ss, lin, '\n'))
	{
		if (lin.size() < 5) continue;
		if (0 != ::strncmp("STR;", lin.c_str(), 4)) continue;

		// ok, it's a stream:
		deque<string> fields;
		mrpt::system::tokenize(lin, ";", fields);

		if (fields.size() < 13) continue;

		TMountPoint mnt;

		mnt.mountpoint_name = fields[1];
		mnt.id = fields[2];
		mnt.format = fields[3];
		mnt.format_details = fields[4];
		mnt.carrier = atoi(fields[5].c_str());
		mnt.nav_system = fields[6];
		mnt.network = fields[7];
		mnt.country_code = fields[8];
		mnt.latitude = atof(fields[9].c_str());
		mnt.longitude = atof(fields[10].c_str());

		// Longitude in range: -180,180
		mnt.longitude = RAD2DEG(mrpt::math::wrapToPi(DEG2RAD(mnt.longitude)));

		mnt.needs_nmea = atoi(fields[11].c_str()) != 0;
		mnt.net_ref_stations = atoi(fields[12].c_str()) != 0;

		if (fields.size() >= 19) mnt.extra_info = fields[18];

		out_list.push_back(mnt);
	}

	return true;
}

/** Enqueues a string to be sent back to the NTRIP server (e.g. GGA frames) */
void CNTRIPClient::sendBackToServer(const std::string& data)
{
	if (data.empty()) return;

	std::vector<uint8_t> d(data.size());
	std::memcpy(&d[0], &data[0], data.size());
	m_upload_data.appendData(d);
}

/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/comms/net_utils.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/hwdrivers/CNTRIPClient.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/system/string_utils.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

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
  if (!m_thread_do_process)
  {
    return;
  }
  // A promise can be satisfied only once: use a fresh one for each close.
  m_sem_sock_closed = std::promise<void>();
  auto closed = m_sem_sock_closed.get_future();
  m_thread_do_process = false;
  closed.wait_for(500ms);
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
  m_answer_connection = connError;
  out_errmsg.clear();

  // Each request has its own result: a worker still busy with an earlier one
  // (e.g. after a timeout) cannot answer this one.
  auto firstConnectPromise = std::make_shared<std::promise<void>>();
  auto firstConnectDone = firstConnectPromise->get_future();
  {
    std::lock_guard<std::mutex> lck(m_args_mtx);
    m_args = params;
    m_first_connect_promise = firstConnectPromise;
    m_attempt_id++;
  }
  m_thread_do_process = true;

  // Wait until the thread tell us the initial result...
  if (firstConnectDone.wait_for(6s) == std::future_status::timeout)
  {
    out_errmsg = "Timeout waiting thread response";
    this->close();
    return false;
  }

  switch (m_answer_connection)
  {
    case connOk:
      return true;
    case connError:
      out_errmsg = mrpt::format("Error trying to connect to server '%s'", params.server.c_str());
      break;
    case connUnauthorized:
      out_errmsg = mrpt::format("Authentication failed for server '%s'", params.server.c_str());
      break;

    default:
      out_errmsg = "UNKNOWN m_answer_connection!!";
      break;
  }
  // Do not keep retrying to connect in the background after reporting failure:
  this->close();
  return false;
}

namespace
{
/** Returns the position right after the header of an NTRIP server reply, or
 * npos if it has not been received completely. NTRIP v1 replies "ICY 200 OK"
 * and starts the stream right after that line; HTTP style replies end their
 * headers with a blank line. */
size_t responseHeaderEnd(const std::string& resp)
{
  if (resp.rfind("ICY ", 0) == 0)
  {
    const size_t eol = resp.find("\r\n");
    return eol == std::string::npos ? eol : eol + 2;
  }
  const size_t blank = resp.find("\r\n\r\n");
  return blank == std::string::npos ? blank : blank + 4;
}
}  // namespace

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
        bool headerComplete = false;
        size_t bodyStart = 0;
        uint64_t attemptId = 0;
        try
        {
          // Nope, it's the first time: get params and try open the
          // connection. The parameters are copied, so that a later call to
          // open() cannot change them in the middle of this attempt (and send
          // its credentials to the server of this one).
          NTRIPArgs args;
          {
            std::lock_guard<std::mutex> lck(m_args_mtx);
            args = m_args;
            attemptId = m_attempt_id;
          }
          stream_data.clear();

          std::cout << mrpt::format(
              "[CNTRIPClient] Trying to connect to %s:%i\n", args.server.c_str(), args.port);

          // (bounded, so a dead server does not keep this thread busy after
          // open() gave up waiting)
          my_sock.connect(args.server, static_cast<unsigned short>(args.port), 4000);
          if (m_thread_exit) break;

          // Prepare HTTP request:
          // -------------------------------------------
          string req = mrpt::format("GET /%s HTTP/1.0\r\n", args.mountpoint.c_str());

          if (isalpha(args.server[0])) req += mrpt::format("Host: %s\r\n", args.server.c_str());

          req += "User-Agent: NTRIP MRPT Library\r\n";
          req += "Accept: */*\r\n";
          req += "Connection: close\r\n";

          // Implement HTTP Basic authentication:
          // See:
          // http://en.wikipedia.org/wiki/Basic_access_authentication
          if (!args.user.empty())
          {
            string auth_str = args.user + string(":") + args.password;
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
          // std::cout << req;

          // Send:
          my_sock.sendString(req);

          // Read the response header: the status line and, unless it is an
          // NTRIP v1 "ICY" reply, the headers up to the blank line. Data of
          // the stream can already follow it in the same packets.
          constexpr size_t MAX_HEADER = 8192;
          string resp;
          while (resp.size() < MAX_HEADER)
          {
            std::vector<uint8_t> chunk(1024);
            const size_t len =
                my_sock.readAsync(&chunk[0], chunk.size(), resp.empty() ? 4000 : 1000, 200);
            if (len == 0)
            {
              break;
            }
            resp.append(chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(len));
            const auto end = responseHeaderEnd(resp);
            if (end != string::npos)
            {
              headerComplete = true;
              bodyStart = end;
              break;
            }
          }
          buf.assign(resp.begin(), resp.end());

          if (headerComplete && my_sock.isConnected()) connect_res = connOk;
        }
        catch (std::exception&)
        {
          // std::cout << e.what() << "\n";
          connect_res = connError;
        }

        // We are not disconnected yet, it's a good thing... anyway,
        // check the answer code:
        if (!buf.empty())
        {
          const string resp(buf.begin(), buf.end());
          const size_t eol = resp.find("\r\n");
          const string statusLine = resp.substr(0, eol);

          const bool statusOk =
              statusLine.find(" 200 ") != string::npos ||
              (statusLine.size() >= 4 && statusLine.compare(statusLine.size() - 4, 4, " 200") == 0);
          if (!statusOk || !headerComplete)
          {
            // It's NOT a good response...
            connect_res = connError;

            // 401?
            if (statusLine.find(" 401 ") != string::npos) connect_res = connUnauthorized;
          }
          else if (connect_res == connOk && bodyStart < buf.size())
          {
            // Stream data that came along with the header:
            stream_data.appendData(std::vector<uint8_t>(
                buf.begin() + static_cast<std::ptrdiff_t>(bodyStart), buf.end()));
          }
        }

        // Signal my caller that the connection is established:
        // ---------------------------------------------------------------
        {
          std::lock_guard<std::mutex> lck(m_args_mtx);
          if (attemptId != m_attempt_id)
          {
            // An open() newer than this attempt has replaced the parameters:
            // this connection is for another server, drop it.
            connect_res = connError;
          }
          else if (m_first_connect_promise)
          {
            m_answer_connection = connect_res;
            m_first_connect_promise->set_value();
            m_first_connect_promise.reset();
          }
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
        if (stream_data.size() > 1024 * 8) stream_data.clear();  // It seems nobody's reading it...

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
        const size_t nWritten = my_sock.writeAsync(&upload_data[0], N, 1000);
        if (nWritten != N)
          cerr << "*ERROR*: Couldn't write back " << N << " bytes to NTRIP server!.\n";
      }

      std::this_thread::sleep_for(10ms);
    }  // end while

  }  // end try
  catch (exception& e)
  {
    cerr << "[CNTRIPClient] Exception in working thread: " << endl << e.what() << "\n";
  }
  catch (...)
  {
    cerr << "[CNTRIPClient] Runtime exception in working thread."
         << "\n";
  }

}  // end working thread

/* --------------------------------------------------------
          retrieveListOfMountpoints
   -------------------------------------------------------- */
bool CNTRIPClient::retrieveListOfMountpoints(
    TListMountPoints& out_list,
    string& out_errmsg,
    const string& server,
    int port,
    const string& auth_user,
    const string& auth_pass)
{
  string content;
  net::HttpRequestOptions httpOptions;
  net::HttpRequestOutput httpOut;

  out_list.clear();

  httpOptions.port = port;
  httpOptions.auth_user = auth_user;
  httpOptions.auth_pass = auth_pass;
  httpOptions.timeout_ms = 6000;

  net::http_errorcode ret =
      net::http_get(string("http://") + server, content, httpOptions, httpOut);

  out_errmsg = httpOut.errormsg;

  // Parse contents:
  if (ret != net::http_errorcode::Ok)
  {
    return false;
  }

  std::stringstream ss(content);
  string lin;
  while (std::getline(ss, lin, '\n'))
  {
    if (lin.size() < 5) continue;
    if (0 != ::strncmp("STR;", lin.c_str(), 4)) continue;

    // ok, it's a stream:
    if (!lin.empty() && lin.back() == '\r') lin.pop_back();

    // Keep blank fields: they hold their column position.
    deque<string> fields;
    mrpt::system::tokenize(lin, ";", fields, false /*do not skip blank tokens*/);

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

    if (fields.size() >= 14) mnt.generator_model = fields[13];
    if (fields.size() >= 15) mnt.compr_encryp = fields[14];
    if (fields.size() >= 16 && !fields[15].empty()) mnt.authentication = fields[15][0];
    if (fields.size() >= 17) mnt.pay_service = (fields[16] == "Y");
    if (fields.size() >= 18) mnt.stream_bitspersec = atoi(fields[17].c_str());
    if (fields.size() >= 19) mnt.extra_info = fields[18];

    out_list.push_back(mnt);
  }

  return true;
}

/** Enqueues a string to be sent back to the NTRIP server (e.g. GGA frames) */
void CNTRIPClient::sendBackToServer(const std::string& data)
{
  if (data.empty())
  {
    return;
  }
  std::vector<uint8_t> d(data.size());
  std::memcpy(&d[0], &data[0], data.size());
  m_upload_data.appendData(d);
}

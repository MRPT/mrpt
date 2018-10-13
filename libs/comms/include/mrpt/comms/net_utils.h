/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>
#include <vector>
#include <mrpt/system/TParameters.h>

namespace mrpt
{
namespace comms
{
/** A set of useful routines for networking. (in #include
 * <mrpt/comms/net_utils.h>)
 * \ingroup mrpt_comms_grp
 */
namespace net
{
/** \addtogroup mrpt_comms_grp
 * @{ */

using std::string;

/** Possible returns from a HTTP request. */
enum ERRORCODE_HTTP
{
	erOk = 0,
	erBadURL,
	erCouldntConnect,
	erNotFound,
	erOtherHTTPError
};

/** Perform an HTTP GET operation (version for retrieving the data as a
 * std::vector<uint8_t>)
 * \param url Must be a simple string of the form
 * "http://<servername>/<relative-address>".
 * \param port The server port, if different from 80.
 * \param extra_headers If provided, the given extra HTTP headers will be sent.
 * \param out_errormsg On exit will contain a description of the error or "Ok".
 * \param out_content The buffer with the retrieved data.
 * \param out_http_responsecode If provided, will hold the HTTP code, eg: 200,
 * 404...
 * \param out_headers If provided, a copy of all the headers returned by the
 * server will be saved here.
 * \param auth_user Send a basic HTTP authorization request with the given user
 * & password.
 * \param auth_pass Send a basic HTTP authorization request with the given user
 * & password.
 *
 * \return The error or success code.
 * \sa http_request()
 */
ERRORCODE_HTTP http_get(
	const string& url, std::vector<uint8_t>& out_content, string& out_errormsg,
	int port = 80, const string& auth_user = string(),
	const string& auth_pass = string(), int* out_http_responsecode = nullptr,
	mrpt::system::TParameters<string>* extra_headers = nullptr,
	mrpt::system::TParameters<string>* out_headers = nullptr,
	int timeout_ms = 1000);

/** Perform an HTTP GET operation (version for retrieving the data as text)
 * \param url Must be a simple string of the form
 * "http://<servername>/<relative-address>".
 * \param port The server port, if different from 80.
 * \param extra_headers If provided, the given extra HTTP headers will be sent.
 * \param out_errormsg On exit will contain a description of the error or "Ok".
 * \param out_content The buffer with the retrieved data.
 * \param out_http_responsecode If provided, will hold the HTTP code, eg: 200,
 * 404...
 * \param out_headers If provided, a copy of all the headers returned by the
 * server will be saved here.
 * \param auth_user Send a basic HTTP authorization request with the given user
 * & password.
 * \param auth_pass Send a basic HTTP authorization request with the given user
 * & password.
 *
 * \return The error or success code.
 * \sa http_request()
 */
ERRORCODE_HTTP http_get(
	const string& url, string& out_content, string& out_errormsg, int port = 80,
	const string& auth_user = string(), const string& auth_pass = string(),
	int* out_http_responsecode = nullptr,
	mrpt::system::TParameters<string>* extra_headers = nullptr,
	mrpt::system::TParameters<string>* out_headers = nullptr,
	int timeout_ms = 1000);

/** Generic function for HTTP GET & POST methods. \sa http_get */
ERRORCODE_HTTP http_request(
	const string& http_method, const string& http_send_content,
	const string& url, std::vector<uint8_t>& out_content, string& out_errormsg,
	int port = 80, const string& auth_user = string(),
	const string& auth_pass = string(), int* out_http_responsecode = nullptr,
	mrpt::system::TParameters<string>* extra_headers = nullptr,
	mrpt::system::TParameters<string>* out_headers = nullptr,
	int timeout_ms = 1000);

/** Resolve a server address by its name, returning its IP address as a
 * string - This method has a timeout for the maximum time to wait for
 * the DNS server.  For example: server_name="www.google.com" ->
 * out_ip="209.85.227.99"
 *
 * \return true on success, false on timeout or other error.
 */
bool DNS_resolve_async(
	const std::string& server_name, std::string& out_ip,
	const unsigned int timeout_ms = 3000);

/** Returns a description of the last Sockets error */
std::string getLastSocketErrorStr();

/** @brief Ping an IP address
 *
 * @param[in] address Address to ping.
 * @param[in] max_attempts Number of attempts to try and ping.
 * @param[out] output String containing output information
 *
 * @return True if responsive, false otherwise.
 *
 * @note { I am redirecting stderr to stdout, so that the overall process
 * is simplified.  Otherwise see:
 * https://jineshkj.wordpress.com/2006/12/22/how-to-capture-stdin-stdout-and-stderr-of-child-program/
 * }
 *
 */
bool Ping(
	const std::string& address, const int max_attempts,
	std::string* output_str = nullptr);

/** @} */  // end grouping

}  // namespace net
}  // namespace comms
}  // namespace mrpt

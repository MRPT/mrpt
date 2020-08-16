/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/core/optional_ref.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace mrpt::comms
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
enum class http_errorcode : uint8_t
{
	Ok = 0,
	BadURL,
	CouldntConnect,
	NotFound,
	OtherHTTPError
};

struct HttpRequestOptions
{
	int port = 80;

	/** Send a basic HTTP authorization request with the given user & password
	 */
	const string auth_user, auth_pass;

	/** Additional HTTP headers to send */
	std::map<std::string, std::string> extra_headers;

	int timeout_ms = 1000;
};

struct HttpRequestOutput
{
	int http_responsecode = 0;

	string errormsg;

	/** Received HTTP headers */
	std::map<std::string, std::string> out_headers;
};

/** Perform an HTTP GET operation (version for retrieving the data as a
 * std::vector<uint8_t>)
 * \param url Must be a simple string of the form
 * "http://<servername>/<relative-address>".
 * \param port The server port, if different from 80.
 * \param extra_headers If provided, the given extra HTTP headers will be sent.
 * \param errormsg On exit will contain a description of the error or "Ok".
 * \param out_content The buffer with the retrieved data.
 * \param out_http_responsecode If provided, will hold the HTTP code, eg: 200,
 * 404...
 * \param out_headers If provided, a copy of all the headers returned by the
 * server will be saved here.
 * \param auth_user
 * \param auth_pass Send a basic HTTP authorization request with the given user
 * & password.
 *
 * \return The error or success code.
 * \sa http_request()
 */
http_errorcode http_get(
	const string& url, std::vector<uint8_t>& out_content,
	const HttpRequestOptions& options = HttpRequestOptions(),
	mrpt::optional_ref<HttpRequestOutput> output = std::nullopt);

/** \overload (version for retrieving the data as text)
 */
http_errorcode http_get(
	const string& url, string& out_content,
	const HttpRequestOptions& options = HttpRequestOptions(),
	mrpt::optional_ref<HttpRequestOutput> output = std::nullopt);

/** Generic function for HTTP GET & POST methods. \sa http_get */
http_errorcode http_request(
	const string& http_method, const string& http_send_content,
	const string& url, std::vector<uint8_t>& out_content,
	const HttpRequestOptions& options = HttpRequestOptions(),
	mrpt::optional_ref<HttpRequestOutput> output = std::nullopt);

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
}  // namespace mrpt::comms

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_NET_UTILS_H
#define  MRPT_NET_UTILS_H

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/utils/TParameters.h>

namespace mrpt
{
	namespace utils
	{
		/** A set of useful routines for networking. (in #include <mrpt/utils/net_utils.h>)
		  * \ingroup network_grp
		  */
		namespace net
		{
			/** \addtogroup network_grp
			  * @{ */

			using std::string;

			/** Possible returns from a HTTP request.
			  */
			enum ERRORCODE_HTTP {
				erOk = 0,
				erBadURL,
				erCouldntConnect,
				erNotFound,
				erOtherHTTPError
			};

			/** Perform an HTTP GET operation (version for retrieving the data as a vector_byte)
			  * \param url Must be a simple string of the form "http://<servername>/<relative-address>".
			  * \param port The server port, if different from 80.
			  * \param extra_headers If provided, the given extra HTTP headers will be sent.
			  * \param out_errormsg On exit will contain a description of the error or "Ok".
			  * \param out_content The buffer with the retrieved data.
			  * \param out_http_responsecode If provided, will hold the HTTP code, eg: 200, 404...
			  * \param out_headers If provided, a copy of all the headers returned by the server will be saved here.
			  * \param auth_user Send a basic HTTP authorization request with the given user & password.
			  * \param auth_pass Send a basic HTTP authorization request with the given user & password.
			  *
			  * \return The error or success code.
			  * \sa mrpt::utils::vectorToBinaryFile
			  */
			ERRORCODE_HTTP BASE_IMPEXP
			http_get(
				const string	&url,
				vector_byte		&out_content,
				string			&out_errormsg,
				int				port = 80,
				const string	&auth_user = string(),
				const string	&auth_pass = string(),
				int				*out_http_responsecode = NULL,
				mrpt::utils::TParameters<string>  *extra_headers = NULL,
				mrpt::utils::TParameters<string>  *out_headers = NULL,
				int  timeout_ms = 1000
				);

			/** Perform an HTTP GET operation (version for retrieving the data as text)
			  * \param url Must be a simple string of the form "http://<servername>/<relative-address>".
			  * \param port The server port, if different from 80.
			  * \param extra_headers If provided, the given extra HTTP headers will be sent.
			  * \param out_errormsg On exit will contain a description of the error or "Ok".
			  * \param out_content The buffer with the retrieved data.
			  * \param out_http_responsecode If provided, will hold the HTTP code, eg: 200, 404...
			  * \param out_headers If provided, a copy of all the headers returned by the server will be saved here.
			  * \param auth_user Send a basic HTTP authorization request with the given user & password.
			  * \param auth_pass Send a basic HTTP authorization request with the given user & password.
			  *
			  * \return The error or success code.
			  * \sa mrpt::utils::vectorToBinaryFile
			  */
			ERRORCODE_HTTP  BASE_IMPEXP
			http_get(
				const string	&url,
				string			&out_content,
				string			&out_errormsg,
				int				port = 80,
				const string	&auth_user = string(),
				const string	&auth_pass = string(),
				int				*out_http_responsecode = NULL,
				mrpt::utils::TParameters<string>  *extra_headers = NULL,
				mrpt::utils::TParameters<string>  *out_headers = NULL,
				int  timeout_ms = 1000
				);


			/** Resolve a server address by its name, returning its IP address as a string - This method has a timeout for the maximum time to wait for the DNS server.
			  *   For example: server_name="www.google.com" -> out_ip="209.85.227.99"
			  *
			  * \return true on success, false on timeout or other error.
			  */
			bool BASE_IMPEXP DNS_resolve_async(
				const std::string &server_name,
				std::string	 &out_ip,
				const unsigned int timeout_ms = 3000
				);

			/** Returns a description of the last Sockets error */
			std::string BASE_IMPEXP getLastSocketErrorStr();

			/** @} */  // end grouping

		} // End of namespace
	} // End of namespace
} // end of namespace

#endif

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/net_utils.h>
#include <mrpt/utils/CClientTCPSocket.h>
#include <mrpt/utils/CServerTCPSocket.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/threads.h>
#include <mrpt/synch/CSemaphore.h>
#include <mrpt/synch/CThreadSafeVariable.h>
#include <cstring>
#include <stdio.h>

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
	#define  INVALID_SOCKET		(-1)
	#include <sys/socket.h>
	#include <unistd.h>
	#include <fcntl.h>
	#include <errno.h>
	#include <sys/types.h>
	#include <sys/ioctl.h>
	#include <netdb.h>
	#include <arpa/inet.h>
	#include <netinet/in.h>
#endif

#ifdef MRPT_OS_WINDOWS
	#include <winsock.h>
#endif


using namespace mrpt;
using namespace mrpt::synch;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::utils::net;
using namespace std;

/*---------------------------------------------------------------
							http_get
  ---------------------------------------------------------------*/
ERRORCODE_HTTP  BASE_IMPEXP
mrpt::utils::net::http_get(
	const string	&url,
	string			&out_content,
	string			&out_errormsg,
	int				port,
	const string	&auth_user,
	const string	&auth_pass,
	int				*out_http_responsecode,
	mrpt::utils::TParameters<string>  *extra_headers,
	mrpt::utils::TParameters<string>  *out_headers,
	int  timeout_ms
	)
{
	vector_byte data;
	ERRORCODE_HTTP ret = http_get(url,data,out_errormsg,port,auth_user,auth_pass,out_http_responsecode,extra_headers,out_headers,timeout_ms);


	out_content.resize(data.size());
	if (!data.empty())
		::memcpy(&out_content[0],&data[0],data.size());

	return ret;
}


/*---------------------------------------------------------------
							http_get
  ---------------------------------------------------------------*/
ERRORCODE_HTTP  BASE_IMPEXP
mrpt::utils::net::http_get(
	const string	&url,
	vector_byte		&out_content,
	string			&out_errormsg,
	int				port,
	const string	&auth_user,
	const string	&auth_pass,
	int				*out_http_responsecode,
	mrpt::utils::TParameters<string>  *extra_headers,
	mrpt::utils::TParameters<string>  *out_headers,
	int  timeout_ms
	)
{
	// Reset output data:
	out_content.clear();
	if (out_http_responsecode) *out_http_responsecode=0;
	if (out_headers) out_headers->clear();

	// URL must be:
	// http://<SERVER>/<LOCAL_ADDR>

	if (0!=::strncmp(url.c_str(),"http://",7))
	{
		out_errormsg="URL must start with 'http://'";
		return net::erBadURL;
	}

	string server_addr = url.substr(7);
	string get_object = "/";

	// Remove from the first "/" on:
	size_t pos = server_addr.find("/");
	if (pos==0)
	{
		out_errormsg="Server name not found in URL";
		return net::erBadURL;
	}
	if (pos!=string::npos)
	{
		get_object = server_addr.substr(pos);
		server_addr.resize(pos);
	}


	CClientTCPSocket	sock;

	try {
		// Connect:
		sock.connect(server_addr,port, timeout_ms);
	}
	catch (std::exception &e) {
		out_errormsg = e.what();
		return net::erCouldntConnect;
	}

	try {
		// Set the user-defined headers (we may overwrite them if needed)
		TParameters<string>  headers_to_send;
		if (extra_headers)
			headers_to_send=*extra_headers;

		headers_to_send["Connection"]="close"; // Don't keep alive

		if (!headers_to_send.has("User-Agent"))
			headers_to_send["User-Agent"]="MRPT Library";

		// Implement HTTP Basic authentication:
		// See: http://en.wikipedia.org/wiki/Basic_access_authentication
		if (!auth_user.empty())
		{
			string auth_str = auth_user+string(":")+auth_pass;
			vector_byte  v(auth_str.size());
			::memcpy(&v[0],&auth_str [0],auth_str .size());

			string encoded_str;
			mrpt::system::encodeBase64(v,encoded_str);

			headers_to_send["Authorization"]=string("Basic ")+encoded_str;
		}


		// Prepare the request string
		// ---------------------------------
		string req = format(
		  "GET %s HTTP/1.1\r\n"
          "Host: %s\r\n",
		  get_object.c_str(),server_addr.c_str());

		// Other headers:
		for (TParameters<string>::const_iterator i=headers_to_send.begin();i!=headers_to_send.end();++i)
		{
			req+=i->first;
			req+=": ";
			req+=i->second;
			req+="\r\n";
		}

		// End:
		req+="\r\n";

		// Send:
		sock.sendString(req);

		// Read answer:
		vector_byte		buf;
		buf.reserve(1<<14);

		size_t	total_read = 0;
		bool	content_length_read = false;
		bool    all_headers_read = false;
		size_t  content_length = 0;
		size_t  content_offset = 0;
		int		http_code = 0;
		mrpt::utils::TParameters<string>  rx_headers;

		CTicTac  watchdog;
		watchdog.Tic();

		while (!content_length_read || total_read<(content_offset+content_length))
		{
			// Read until "Content-Length: XXX \r\n" is read or the whole message is read,
			//  or an error code is read.
			size_t	to_read_now;

			if (!content_length_read)
			{
				to_read_now = 1500;
			}
			else
			{
				to_read_now	= (content_length+content_offset) -total_read;
			}

			// make room for the data to come:
			buf.resize(total_read+to_read_now+1);

			// Read:
			size_t len = sock.readAsync(&buf[total_read],to_read_now, timeout_ms,100);
			if (!len)
			{
				//
				if (!sock.isConnected())
				{
					if (all_headers_read) // It seems we're done...
						break;
					else
					{
						out_errormsg = "Connection to server was lost";
						return net::erCouldntConnect;
					}
				}

				if ( watchdog.Tac()>1e-3*timeout_ms)
				{
					out_errormsg = "Timeout waiting answer from server";
					return net::erCouldntConnect;
				}
				mrpt::system::sleep(10);
				continue;
			}
			total_read+=len;
			watchdog.Tic();

			buf[total_read]='\0';

			// do we have a \r\n\r\n  ??
			if (!all_headers_read)
			{
				const char *ptr = ::strstr(reinterpret_cast<const char*>(&buf[0]),"\r\n\r\n");
				if (ptr)
				{
					all_headers_read = true;
					const size_t pos_dblret = ((char*)ptr)-(char*)(&buf[0]);

					// Process the headers:
					// ------------------------------
					if (!::strncmp("HTTP/",(const char*)&buf[0], 5))
					{
						http_code = ::atoi( (const char*)&buf[9] );
					}
					else
					{
						// May it be a "SOURCETABLE " answer for NTRIP protocol??
						if (!::strncmp("SOURCETABLE ",(const char*)&buf[0], 12))
						{
							http_code = ::atoi( (const char*)&buf[12] );
						}
						else
						{
							out_errormsg = "Server didn't send an HTTP/1.1 answer.";
							return net::erOtherHTTPError;
						}
					}

					// Check the HTTP code and the content-length:
					content_offset = pos_dblret + 4;

					// Do we have a "Content-Length:"??
					const char *ptr_len = ::strstr(reinterpret_cast<const char*>(&buf[0]),"Content-Length:");
					if (ptr_len)
					{
						content_length = ::atol( ptr_len+15 );
						content_length_read = true;
					}

					// Parse the rest of HTTP headers:
					{
						string aux_all_headers;
						deque<string> lstLines;
						aux_all_headers.resize(content_offset);
						::memcpy( &aux_all_headers[0],&buf[0],content_offset);

						mrpt::system::tokenize(aux_all_headers,"\r\n",lstLines);

						for (deque<string>::const_iterator i=lstLines.begin();i!=lstLines.end();++i)
						{
							const size_t p = i->find(":");
							if (p==string::npos) continue;

							const string key = i->substr(0,p);
							const string val = i->substr(p+2);
							rx_headers[key] = val;
						}
					}

				}
			}
		} // end while


		if (out_http_responsecode) *out_http_responsecode=http_code;
		if (out_headers) *out_headers = rx_headers;

		// Remove the headers from the content:
		buf.erase(buf.begin(),buf.begin()+content_offset);

		// Process: "Transfer-Encoding: chunked"
		if (rx_headers.has("Transfer-Encoding") && rx_headers["Transfer-Encoding"]=="chunked" )
		{
			// See: http://en.wikipedia.org/wiki/Chunked_transfer_encoding

			size_t index = 0;
			while (index<buf.size())
			{
				if (buf[index]=='\r' && buf[index+1]=='\n')
				{
					buf.erase(buf.begin()+index,buf.begin()+index+2);
					continue;
				}

				const char *pCRLF = ::strstr((const char*)&buf[index],"\r\n");
				if (!pCRLF) break;

				const size_t len_substr = ((char*)pCRLF)-(char*)(&buf[index]);

				string  sLen((const char*)&buf[index], len_substr);
				sLen=string("0x")+sLen;

				unsigned int lenChunk;
				int fields = ::sscanf(sLen.c_str(),"%x",&lenChunk);
				if (!fields) break;

				// Remove the len of this chunk header from the data:
				buf.erase(buf.begin()+index,buf.begin()+index+len_substr+2);

				index+=lenChunk;

				if (!lenChunk)
				{
					buf.resize(index);
					break;
				}
			}
		}


		// Set the content output:
		out_content.swap(buf);

		if (http_code==200)
		{
			return net::erOk;
		}
		else
		{
			out_errormsg = format("HTTP error %i",http_code);
			return net::erOtherHTTPError;
		}
	}
	catch (std::exception &e) {
		out_errormsg = e.what();
		return net::erCouldntConnect;
	}

	return net::erOk;
}


// ===========   net::DNS_resolve_async   ==============
// Yeah, it's gonna got pretty complicated...
struct TDNSThreadData
{
	TDNSThreadData() :
		sem_solved(0,1),
		sem_caller_quitted(0,1),
		in_servername(),
		out_solved_ip()
	{}
	synch::CSemaphore   sem_solved;
	synch::CSemaphore	sem_caller_quitted;   // This is needed in order to the thread to wait for deleting this struct!
	std::string			in_servername;
	CThreadSafeVariable<std::string>  out_solved_ip;  // Will be set to NULL by the caller if the function returns.
};

void thread_DNS_solver_async(TDNSThreadData &dat); // Frd decl.



/** Resolve a server address by its name, returning its IP address as a string - This method has a timeout for the maximum time to wait for the DNS server.
  *   For example: server_name="www.google.com" -> out_ip="209.85.227.99"
  *
  * \return true on success, false on timeout or other error.
  */

bool net::DNS_resolve_async(
	const std::string &server_name,
	std::string	 &out_ip,
	const unsigned int timeout_ms
	)
{
	// Firstly: If it's a numeric address already, do nothing:
	if (server_name.find_first_not_of("0123456789. ")==std::string::npos)
	{
		// It's a pure IP address:
		out_ip = server_name;
		return true;
	}

	// Solve DNS --------------
	// It seems that the only reliable way of *with a timeout* is to launch a separate thread.
	TDNSThreadData param;
	param.in_servername = server_name;

	TThreadHandle th =
	mrpt::system::createThreadRef( thread_DNS_solver_async,param );

	bool res =(param.sem_solved.waitForSignal(timeout_ms));
	// Let the thread now about me quitting:
	param.sem_caller_quitted.release();
	mrpt::system::joinThread(th);

	if (res)
	{
		// Done: Anyway, it can still be an error result:
		out_ip = param.out_solved_ip;
		return !out_ip.empty();
	}
	else
	{
		// Timeout:
		out_ip.clear();

		return false;
	}
}

void thread_DNS_solver_async(TDNSThreadData &param)
{
	// Windows-specific stuff:
#ifdef MRPT_OS_WINDOWS
	{
		// Init the WinSock Library:
		WORD		wVersionRequested = MAKEWORD( 2, 0 );
		WSADATA		wsaData;
		if (WSAStartup( wVersionRequested, &wsaData ) )
		{
			std::cerr << "thread_DNS_solver_async: Error calling WSAStartup";
			return;
		}
	}
#endif

	// Do the DNS lookup:
	std::string  dns_result;

	hostent *he = gethostbyname( param.in_servername.c_str() );
	if (!he)
	{
		dns_result.clear();  // empty string -> error.
	}
	else
	{
		struct in_addr ADDR;
		::memcpy(&ADDR, he->h_addr, sizeof(ADDR)); // Was: *((struct in_addr *)he->h_addr);
		// Convert address to text:
		dns_result = string( inet_ntoa(ADDR) );
	}

	// Save in the caller string:
	param.out_solved_ip.set( dns_result );

	// and signal we're done:
	param.sem_solved.release();

	// Finally, wait for the main thread to end so we can safely free the shared struct:
	param.sem_caller_quitted.waitForSignal();

#ifdef MRPT_OS_WINDOWS
	WSACleanup();
#endif
}

/** Returns a description of the last Sockets error */
std::string mrpt::utils::net::getLastSocketErrorStr()
{
#ifdef MRPT_OS_WINDOWS
	const int errnum = WSAGetLastError();
	char * s = NULL;
	FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, 
		NULL, errnum,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPSTR)&s, 0, NULL);
	const std::string str = mrpt::format("%s [errno=%d]",s,errnum);
	LocalFree(s);
	return str;
#else
	return std::string(strerror(errno));
#endif
}


/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/net_utils.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::utils::net;
using namespace std;


string url = "http://www.google.es/";
//string url = "http://static.meneame.net/cache/avatars/101/50898-20.jpg";

/* ------------------------------------------------------------------------
					Test: HTTP get
   ------------------------------------------------------------------------ */
void Test_HTTP_get()
{
	string  content;
	string  errmsg;
	TParameters<string>  out_headers;

	cout << "Retrieving " << url << "..." << endl;

	//CClientTCPSocket::DNS_LOOKUP_TIMEOUT_MS = 5000;

	ERRORCODE_HTTP ret = http_get(url,content,errmsg,80,"","",NULL,NULL,&out_headers);

	if ( ret != net::erOk )
	{
		cout << " Error: " << errmsg << endl;
		return;
	}

	string typ = out_headers.has("Content-Type") ? out_headers["Content-Type"] : string("???");

	cout << "Ok: " << content.size() << " bytes of type: " << typ << endl;
	//cout << content << endl;

}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		if (argc>1)
			url = string(argv[1]);

		Test_HTTP_get();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!");
		return -1;
	}
}

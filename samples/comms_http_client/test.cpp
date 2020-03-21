/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/** \example comms_http_client/test.cpp */

//! [example-http-get]

#include <mrpt/comms/net_utils.h>
#include <mrpt/core/exceptions.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::comms;
using namespace mrpt::comms::net;
using namespace std;

string url = "http://www.google.es/";

void Test_HTTP_get()
{
	string content;
	string errmsg;
	mrpt::system::TParameters<string> out_headers;

	cout << "Retrieving " << url << "..." << endl;

	ERRORCODE_HTTP ret = http_get(
		url, content, errmsg, 80, "", "", nullptr, nullptr, &out_headers);

	if (ret != net::erOk)
	{
		cout << " Error: " << errmsg << endl;
		return;
	}

	string typ = out_headers.has("Content-Type") ? out_headers["Content-Type"]
												 : string("???");

	cout << "Ok: " << content.size() << " bytes of type: " << typ << endl;
	// cout << content << endl;
}
//! [example-http-get]

int main(int argc, char** argv)
{
	try
	{
		if (argc > 1) url = string(argv[1]);

		Test_HTTP_get();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}

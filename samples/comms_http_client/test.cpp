/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
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

std::string url = "http://www.google.es/";

void Test_HTTP_get()
{
	std::string content;

	mrpt::comms::net::HttpRequestOptions httpOptions;
	mrpt::comms::net::HttpRequestOutput httpOut;

	std::cout << "Retrieving " << url << "..." << std::endl;

	http_errorcode ret = http_get(url, content, httpOptions, httpOut);

	if (ret != net::http_errorcode::Ok)
	{
		std::cout << " Error: " << httpOut.errormsg << std::endl;
		return;
	}

	string typ = httpOut.out_headers.count("Content-Type")
		? httpOut.out_headers.at("Content-Type")
		: string("???");

	std::cout << "Ok: " << content.size() << " bytes of type: " << typ
			  << std::endl;
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

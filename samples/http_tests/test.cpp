/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/net_utils.h>

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

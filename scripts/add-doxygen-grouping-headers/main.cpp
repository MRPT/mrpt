/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
// Parse all ".h" files and replace the first line with:
//   +---------------------------------------------------------------------------+ */
//
//  by :
//
//   +---------------------------------------------------------------------------+ */
//   /** \addtogroup mrpt_vision @{ */
//
//
//  and the closing   /** @} */ at the end
//



#include <cstdlib>
#include <cstdio>

#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char **argv)
{
	if (argc!=3)
	{
		printf("usage: %s GROUP_NAME <C++ header file>\n",argv[0]);
		return 1;
	}

	const char *GROUP_NAME = argv[1];

	const char *in_file_name  = argv[2];


	const char *out_file_name = "replace-header.tmp";

	printf("add-doxygen-grouping-headers: working on %s...", in_file_name);

	ifstream	f( in_file_name );
	if (f.fail())
	{
		cerr << "ERROR: cannot open " << in_file_name << endl;
		return 1;
	}

	ofstream	of( out_file_name );
	if (of.fail())
	{
		cerr << "ERROR: cannot create " << out_file_name << endl;
		return 1;
	}


	int  		nLines = 0;
	string	 	inLine;
	//bool		copyThisLine, insertMyHeading, lookingForHeadEnd = false;
	while ( !f.eof() && !f.fail() )
	{
		std::getline(f,inLine);
		if (!f.fail())
		{
			if (inLine == "   +---------------------------------------------------------------------------+ */")
			{
				of << inLine << "\n";
				of << "\n/** \\addtogroup " << GROUP_NAME << "\n";
				of << "      @{ */\n";
			}
			else
			{
				of << inLine << "\n";
			}
		}
		nLines ++;
	}

	of << "\n/** @} */\n\n";

	f.close();
	of.close();


	system( (string("mv replace-header.tmp ")+ string(in_file_name)).c_str() );
	printf("done (%i lines)\n", nLines);

	return 0;
}



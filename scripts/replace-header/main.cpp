/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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

#include <cstdlib>
#include <cstdio>

#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char **argv)
{
	if (argc!=2)
	{
		printf("usage: %s <C++ source file>\n",argv[0]);
		return 1;
	}

	const char *in_file_name  = argv[1];

	
	const char *out_file_name = "replace-header.tmp";

	printf("replace-header: working on %s...", in_file_name);
	
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
	bool		copyThisLine, insertMyHeading, lookingForHeadEnd = false;
	while ( !f.eof() && !f.fail() )
	{
		copyThisLine = true;
		insertMyHeading = false;

		std::getline(f,inLine);
		if (!f.fail())
		{
			// Still looking for header start?
			if (nLines==0)
			{
				// Started??
				if ( inLine.find("/*")!= string::npos )
				{
					lookingForHeadEnd = true;
					copyThisLine = false;
				}
				else 
				{
					// Seems there is no heading: add ours:
					insertMyHeading = true;
				}
			}

			// Looking for end?
			if (lookingForHeadEnd)
			{
				copyThisLine = false;

				if ( inLine.find("*/")!= string::npos )
				{
					lookingForHeadEnd=false;
					insertMyHeading = true;
				}
			}

			// Insert comment block:
			if (insertMyHeading)
			{
				of << "/* +---------------------------------------------------------------------------+\r\n";
				of << "   |                 The Mobile Robot Programming Toolkit (MRPT)               |\r\n";
				of << "   |                                                                           |\r\n";
				of << "   |                          http://www.mrpt.org/                             |\r\n";
				of << "   |                                                                           |\r\n";
				of << "   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |\r\n";
				of << "   | All rights reserved.                                                      |\r\n";
				of << "   |                                                                           |\r\n";
				of << "   | Redistribution and use in source and binary forms, with or without        |\r\n";
				of << "   | modification, are permitted provided that the following conditions are    |\r\n";
				of << "   | met:                                                                      |\r\n";
				of << "   |    * Redistributions of source code must retain the above copyright       |\r\n";
				of << "   |      notice, this list of conditions and the following disclaimer.        |\r\n";
				of << "   |    * Redistributions in binary form must reproduce the above copyright    |\r\n";
				of << "   |      notice, this list of conditions and the following disclaimer in the  |\r\n";
				of << "   |      documentation and/or other materials provided with the distribution. |\r\n";
				of << "   |    * Neither the name of the copyright holders nor the                    |\r\n";
				of << "   |      names of its contributors may be used to endorse or promote products |\r\n";
				of << "   |      derived from this software without specific prior written permission.|\r\n";
				of << "   |                                                                           |\r\n"; 
				of << "   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |\r\n";  
				of << "   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |\r\n"; 
				of << "   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|\r\n"; 
				of << "   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |\r\n"; 
				of << "   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|\r\n"; 
				of << "   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|\r\n"; 
				of << "   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |\r\n";
				of << "   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |\r\n";
				of << "   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |\r\n";
				of << "   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |\r\n";
				of << "   | POSSIBILITY OF SUCH DAMAGE.                                               |\r\n";
				of << "   +---------------------------------------------------------------------------+ */\r\n";

			}

			// Normal copy:
			if (copyThisLine)
			{
				of << inLine << "\r\n";				
			}		

		}
		nLines ++;
	}

	
	f.close();
	of.close();

	
	system( (string("mv replace-header.tmp ")+ string(in_file_name)).c_str() );
	printf("done (%i lines)\n", nLines);

	return 0;
}



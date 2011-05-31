// Parses a .cpp/.h file and replaces the first /***/ multi-line comment block by another fixed one:

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
				of << "   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |\r\n";
				of << "   |                                                                           |\r\n";
				of << "   |                       http://www.mrpt.org/                                |\r\n";
				of << "   |                                                                           |\r\n";
				of << "   |   Copyright (C) 2005-2011  University of Malaga                           |\r\n";
				of << "   |                                                                           |\r\n";
				of << "   |    This software was written by the Machine Perception and Intelligent    |\r\n";
				of << "   |      Robotics Lab, University of Malaga (Spain).                          |\r\n";
				of << "   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |\r\n";
				of << "   |                                                                           |\r\n";
				of << "   |  This file is part of the MRPT project.                                   |\r\n";
				of << "   |                                                                           |\r\n";
				of << "   |     MRPT is free software: you can redistribute it and/or modify          |\r\n";
				of << "   |     it under the terms of the GNU General Public License as published by  |\r\n";
				of << "   |     the Free Software Foundation, either version 3 of the License, or     |\r\n";
				of << "   |     (at your option) any later version.                                   |\r\n";
				of << "   |                                                                           |\r\n";
    				of << "   |   MRPT is distributed in the hope that it will be useful,                 |\r\n";
				of << "   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |\r\n";
				of << "   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |\r\n";
				of << "   |     GNU General Public License for more details.                          |\r\n";
				of << "   |                                                                           |\r\n";
				of << "   |     You should have received a copy of the GNU General Public License     |\r\n";
				of << "   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |\r\n";
				of << "   |                                                                           |\r\n";
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



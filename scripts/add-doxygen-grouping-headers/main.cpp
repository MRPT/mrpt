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



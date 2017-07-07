/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/****************************************************************************** 
 * 
 *  file:  DocBookOutput.h
 * 
 *  Copyright (c) 2004, Michael E. Smoot
 *  All rights reverved.
 * 
 *  See the file COPYING in the top directory of this distribution for
 *  more information.
 *  
 *  THE SOFTWARE IS PROVIDED _AS IS_, WITHOUT WARRANTY OF ANY KIND, EXPRESS 
 *  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 *  DEALINGS IN THE SOFTWARE.  
 *  
 *****************************************************************************/ 

#ifndef TCLAP_DOCBOOKOUTPUT_H
#define TCLAP_DOCBOOKOUTPUT_H

#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <algorithm>

#include <mrpt/otherlibs/tclap/CmdLineInterface.h>
#include <mrpt/otherlibs/tclap/CmdLineOutput.h>
#include <mrpt/otherlibs/tclap/XorHandler.h>
#include <mrpt/otherlibs/tclap/Arg.h>

namespace TCLAP {

/**
 * A class that generates DocBook output for usage() method for the 
 * given CmdLine and its Args.
 */
class DocBookOutput : public CmdLineOutput
{

	public:

		/**
		 * Prints the usage to stdout.  Can be overridden to 
		 * produce alternative behavior.
		 * \param c - The CmdLine object the output is generated for. 
		 */
		virtual void usage(CmdLineInterface& c);

		/**
		 * Prints the version to stdout. Can be overridden 
		 * to produce alternative behavior.
		 * \param c - The CmdLine object the output is generated for. 
		 */
		virtual void version(CmdLineInterface& c);

		/**
		 * Prints (to stderr) an error message, short usage 
		 * Can be overridden to produce alternative behavior.
		 * \param c - The CmdLine object the output is generated for. 
		 * \param e - The ArgException that caused the failure. 
		 */
		virtual void failure(CmdLineInterface& c, 
						     ArgException& e );

	protected:

		/**
		 * Substitutes the char r for string x in string s.
		 * \param s - The string to operate on. 
		 * \param r - The char to replace. 
		 * \param x - What to replace r with. 
		 */
		void substituteSpecialChars( std::string& s, char r, std::string& x );
		void removeChar( std::string& s, char r);

		void printShortArg(Arg* it);
		void printLongArg(Arg* it);
};


inline void DocBookOutput::version(CmdLineInterface& _cmd) 
{ 
	std::cout << _cmd.getVersion() << std::endl;
}

inline void DocBookOutput::usage(CmdLineInterface& _cmd ) 
{
	std::list<Arg*> argList = _cmd.getArgList();
	std::string progName = _cmd.getProgramName();
	std::string version = _cmd.getVersion();
	XorHandler xorHandler = _cmd.getXorHandler();
	std::vector< std::vector<Arg*> > xorList = xorHandler.getXorList();


	std::cout << "<?xml version='1.0'?>" << std::endl;
	std::cout << "<!DOCTYPE book PUBLIC \"-//Norman Walsh//DTD DocBk XML V4.2//EN\"" << std::endl;
	std::cout << "\t\"http://www.oasis-open.org/docbook/xml/4.2/docbookx.dtd\">" << std::endl << std::endl;

	std::cout << "<book>" << std::endl;
	std::cout << "<refentry>" << std::endl;

	std::cout << "<refmeta>" << std::endl;
	std::cout << "<refentrytitle>" << std::endl;
	std::cout << progName << std::endl; 
	std::cout << "</refentrytitle>" << std::endl;
	std::cout << "<manvolnum>1</manvolnum>" << std::endl;
	std::cout << "</refmeta>" << std::endl;

	std::cout << "<refnamediv>" << std::endl;
	std::cout << "<refname>" << std::endl;
	std::cout << progName << std::endl; 
	std::cout << "</refname>" << std::endl;
	std::cout << "</refnamediv>" << std::endl;

	std::cout << "<cmdsynopsis>" << std::endl;

	std::cout << "<command>" << progName << "</command>" << std::endl;

	// xor
	for ( int i = 0; (unsigned int)i < xorList.size(); i++ )
	{
		std::cout << "<group choice='req'>" << std::endl;
		for ( ArgVectorIterator it = xorList[i].begin(); 
						it != xorList[i].end(); it++ )
			printShortArg((*it));

		std::cout << "</group>" << std::endl;
	}
	
	// rest of args
	for (ArgListIterator it = argList.begin(); it != argList.end(); it++)
		if ( !xorHandler.contains( (*it) ) )
			printShortArg((*it));

 	std::cout << "</cmdsynopsis>" << std::endl;

	std::cout << "<refsect1>" << std::endl;
	std::cout << "<title>Description</title>" << std::endl;
	std::cout << "<para>" << std::endl;
	std::cout << _cmd.getMessage() << std::endl; 
	std::cout << "</para>" << std::endl;
	std::cout << "</refsect1>" << std::endl;

	std::cout << "<refsect1>" << std::endl;
	std::cout << "<title>Options</title>" << std::endl;
	std::cout << "<para>" << std::endl;
	std::cout << "<itemizedlist>" << std::endl;
	// xor
	for ( int i = 0; (unsigned int)i < xorList.size(); i++ )
	{
		std::cout << "<itemizedlist>" << std::endl;
		size_t xlen = xorList.size() - 1;
		size_t xcount = 0; 
		for ( ArgVectorIterator it = xorList[i].begin(); 
						it != xorList[i].end(); it++, xcount++ )
		{
			printLongArg((*it));
			if ( xcount < xlen )
				std::cout << "<listitem>OR</listitem>" << std::endl;
		}

		std::cout << "</itemizedlist>" << std::endl;
	}
	
	// rest of args
	for (ArgListIterator it = argList.begin(); it != argList.end(); it++)
		if ( !xorHandler.contains( (*it) ) )
			printLongArg((*it));

	std::cout << "</itemizedlist>" << std::endl;
	std::cout << "</para>" << std::endl;
	std::cout << "</refsect1>" << std::endl;

	std::cout << "<refsect1>" << std::endl;
	std::cout << "<title>Version</title>" << std::endl;
	std::cout << "<para>" << std::endl;
	std::cout << version << std::endl; 
	std::cout << "</para>" << std::endl;
	std::cout << "</refsect1>" << std::endl;
	
	std::cout << "</refentry>" << std::endl;
 	std::cout << "</book>" << std::endl;

}

inline void DocBookOutput::failure( CmdLineInterface& _cmd,
				                ArgException& e ) 
{ 
		std::cout << e.what() << std::endl;
}

inline void DocBookOutput::substituteSpecialChars( std::string& s,
				                                   char r,
												   std::string& x )
{
	size_t p;
	while ( (p = s.find_first_of(r)) != std::string::npos )
	{
		s.erase(p,1);
		s.insert(p,x);
	}
}

inline void DocBookOutput::removeChar( std::string& s, char r)
{
	size_t p;
	while ( (p = s.find_first_of(r)) != std::string::npos )
	{
		s.erase(p,1);
	}
}

inline void DocBookOutput::printShortArg(Arg* a)
{
	std::string lt = "&lt;"; 
	std::string gt = "&gt;"; 

	std::string id = a->shortID();
	substituteSpecialChars(id,'<',lt);
	substituteSpecialChars(id,'>',gt);
	removeChar(id,'[');
	removeChar(id,']');
	
	std::string choice = "opt";
	if ( a->isRequired() )
		choice = "req";

	std::string repeat = "norepeat";
	if ( a->acceptsMultipleValues() )
		repeat = "repeat";

		
				
	std::cout << "<arg choice='" << choice 
			  << "' repeat='" << repeat << "'>" 
			  << id << "</arg>" << std::endl; 

}

inline void DocBookOutput::printLongArg(Arg* a)
{
	std::string lt = "&lt;"; 
	std::string gt = "&gt;"; 

	std::string id = a->longID();
	substituteSpecialChars(id,'<',lt);
	substituteSpecialChars(id,'>',gt);
	removeChar(id,'[');
	removeChar(id,']');

	std::string desc = a->getDescription();
	substituteSpecialChars(desc,'<',lt);
	substituteSpecialChars(desc,'>',gt);

	std::cout << "<simplelist>" << std::endl;

	std::cout << "<member>" << std::endl;
	std::cout << id << std::endl;
	std::cout << "</member>" << std::endl;

	std::cout << "<member>" << std::endl;
	std::cout << desc << std::endl;
	std::cout << "</member>" << std::endl;

	std::cout << "</simplelist>" << std::endl;
}

} //namespace TCLAP
#endif 

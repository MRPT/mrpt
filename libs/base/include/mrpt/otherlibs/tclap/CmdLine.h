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
 *  file:  CmdLine.h
 *
 *  Copyright (c) 2003, Michael E. Smoot .
 *  Copyright (c) 2004, Michael E. Smoot, Daniel Aarno.
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

#ifndef TCLAP_CMDLINE_H
#define TCLAP_CMDLINE_H

#include <mrpt/otherlibs/tclap/SwitchArg.h>
#include <mrpt/otherlibs/tclap/MultiSwitchArg.h>
#include <mrpt/otherlibs/tclap/UnlabeledValueArg.h>
#include <mrpt/otherlibs/tclap/UnlabeledMultiArg.h>

#include <mrpt/otherlibs/tclap/XorHandler.h>
#include <mrpt/otherlibs/tclap/HelpVisitor.h>
#include <mrpt/otherlibs/tclap/VersionVisitor.h>
#include <mrpt/otherlibs/tclap/IgnoreRestVisitor.h>

#include <mrpt/otherlibs/tclap/CmdLineOutput.h>
#include <mrpt/otherlibs/tclap/StdOutput.h>

#include <mrpt/otherlibs/tclap/Constraint.h>
#include <mrpt/otherlibs/tclap/ValuesConstraint.h>

#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace TCLAP {

/**
 * The base class that manages the command line definition and passes
 * along the parsing to the appropriate Arg classes.
 */
class CmdLine : public CmdLineInterface
{
	protected:

		/**
		 * The list of arguments that will be tested against the
		 * command line.
		 */
		std::list<Arg*> _argList;

		/**
		 * The name of the program.  Set to argv[0].
		 */
		std::string _progName;

		/**
		 * A message used to describe the program.  Used in the usage output.
		 */
		std::string _message;

		/**
		 * The version to be displayed with the --version switch.
		 */
		std::string _version;

		/**
		 * The number of arguments that are required to be present on
		 * the command line. This is set dynamically, based on the
		 * Args added to the CmdLine object.
		 */
		int _numRequired;

		/**
		 * The character that is used to separate the argument flag/name
		 * from the value.  Defaults to ' ' (space).
		 */
		char _delimiter;

		/**
		 * The handler that manages xoring lists of args.
		 */
		XorHandler _xorHandler;

		/**
		 * A list of Args to be explicitly deleted when the destructor
		 * is called.  At the moment, this only includes the three default
		 * Args.
		 */
		std::list<Arg*> _argDeleteOnExitList;

		/**
		 * A list of Visitors to be explicitly deleted when the destructor
		 * is called.  At the moment, these are the Vistors created for the
		 * default Args.
		 */
		std::list<Visitor*> _visitorDeleteOnExitList;

		/**
		 * Object that handles all output for the CmdLine.
		 */
		CmdLineOutput* _output;

		/**
		 * Checks whether a name/flag string matches entirely matches
		 * the Arg::blankChar.  Used when multiple switches are combined
		 * into a single argument.
		 * \param s - The message to be used in the usage.
		 */
		bool _emptyCombined(const std::string& s);

		/**
		 * Perform a delete ptr; operation on ptr when this object is deleted.
		 */
		void deleteOnExit(Arg* ptr);

		/**
		 * Perform a delete ptr; operation on ptr when this object is deleted.
		 */
		void deleteOnExit(Visitor* ptr);

	private:

		/**
		 * Encapsulates the code common to the constructors (which is all
		 * of it).
		 */
		void _constructor();

		/**
		 * Is set to true when a user sets the output object. We use this so
		 * that we don't delete objects that are created outside of this lib.
		 */
		bool _userSetOutput;

		/**
		 * Whether or not to automatically create help and version switches.
		 */
		bool _helpAndVersion;

	public:

		/**
		 * Command line constructor. Defines how the arguments will be
		 * parsed.
		 * \param message - The message to be used in the usage
		 * output.
		 * \param delimiter - The character that is used to separate
		 * the argument flag/name from the value.  Defaults to ' ' (space).
		 * \param version - The version number to be used in the
		 * --version switch.
		 * \param helpAndVersion - Whether or not to create the Help and
		 * Version switches. Defaults to true.
		 */
		CmdLine(const std::string& message,
				const char delimiter = ' ',
				const std::string& version = "none",
				bool helpAndVersion = true);

		/**
		 * Deletes any resources allocated by a CmdLine object.
		 */
		virtual ~CmdLine();

		/**
		 * Adds an argument to the list of arguments to be parsed.
		 * \param a - Argument to be added.
		 */
		void add( Arg& a );

		/**
		 * An alternative add.  Functionally identical.
		 * \param a - Argument to be added.
		 */
		void add( Arg* a );

		/**
		 * Add two Args that will be xor'd.  If this method is used, add does
		 * not need to be called.
		 * \param a - Argument to be added and xor'd.
		 * \param b - Argument to be added and xor'd.
		 */
		void xorAdd( Arg& a, Arg& b );

		/**
		 * Add a list of Args that will be xor'd.  If this method is used,
		 * add does not need to be called.
		 * \param xors - List of Args to be added and xor'd.
		 */
		void xorAdd( std::vector<Arg*>& xors );

		/**
		 * Parses the command line.
		 * \param argc - Number of arguments.
		 * \param argv - Array of arguments.
		 * \return (Added by JLBC for MRPT): Return false if the program should exit (error in args, it was --help, etc...)
		 */
		bool parse(int argc, char** argv);

		/**
		 *
		 */
		CmdLineOutput* getOutput();

		/**
		 *
		 */
		void setOutput(CmdLineOutput* co);

		/**
		 *
		 */
		std::string& getVersion();

		/**
		 *
		 */
		std::string& getProgramName();

		/**
		 *
		 */
		std::list<Arg*>& getArgList();

		/**
		 *
		 */
		XorHandler& getXorHandler();

		/**
		 *
		 */
		char getDelimiter();

		/**
		 *
		 */
		std::string& getMessage();

		/**
		 *
		 */
		bool hasHelpAndVersion();
};


///////////////////////////////////////////////////////////////////////////////
//Begin CmdLine.cpp
///////////////////////////////////////////////////////////////////////////////

inline CmdLine::CmdLine(const std::string& m,
				        char delim,
						const std::string& v,
						bool help )
: _progName("not_set_yet"),
  _message(m),
  _version(v),
  _numRequired(0),
  _delimiter(delim),
  _userSetOutput(false),
  _helpAndVersion(help)
{
	_constructor();
}

inline CmdLine::~CmdLine()
{
	ArgListIterator argIter;
	VisitorListIterator visIter;

	for( argIter = _argDeleteOnExitList.begin();
		 argIter != _argDeleteOnExitList.end();
		 ++argIter)
		delete *argIter;

	for( visIter = _visitorDeleteOnExitList.begin();
		 visIter != _visitorDeleteOnExitList.end();
		 ++visIter)
		delete *visIter;

	if ( !_userSetOutput )
		delete _output;
}

inline void CmdLine::_constructor()
{
	_output = new StdOutput;

	Arg::setDelimiter( _delimiter );

	Visitor* v;

	if ( _helpAndVersion )
	{
		v = new HelpVisitor( this, &_output );
		SwitchArg* help = new SwitchArg("h","help",
						"Displays usage information and exits.",
						false, v);
		add( help );
		deleteOnExit(help);
		deleteOnExit(v);

		v = new VersionVisitor( this, &_output );
		SwitchArg* vers = new SwitchArg("","version",
					"Displays version information and exits.",
					false, v);
		add( vers );
		deleteOnExit(vers);
		deleteOnExit(v);
	}

	v = new IgnoreRestVisitor();
	SwitchArg* ignore  = new SwitchArg(Arg::flagStartString(),
					   Arg::ignoreNameString(),
			   "Ignores the rest of the labeled arguments following this flag.",
					   false, v);
	add( ignore );
	deleteOnExit(ignore);
	deleteOnExit(v);
}

inline void CmdLine::xorAdd( std::vector<Arg*>& ors )
{
	_xorHandler.add( ors );

	for (ArgVectorIterator it = ors.begin(); it != ors.end(); it++)
	{
		(*it)->forceRequired();
		(*it)->setRequireLabel( "OR required" );

		add( *it );
	}
}

inline void CmdLine::xorAdd( Arg& a, Arg& b )
{
    std::vector<Arg*> ors;
    ors.push_back( &a );
    ors.push_back( &b );
	xorAdd( ors );
}

inline void CmdLine::add( Arg& a )
{
	add( &a );
}

inline void CmdLine::add( Arg* a )
{
	for( ArgListIterator it = _argList.begin(); it != _argList.end(); it++ )
		if ( *a == *(*it) )
			throw( SpecificationException(
			       	"Argument with same flag/name already exists!",
					a->longID() ) );

	a->addToList( _argList );

	if ( a->isRequired() )
		_numRequired++;
}

inline bool CmdLine::parse(int argc, char** argv)
{
	try {

	_progName = argv[0];

	// this step is necessary so that we have easy access to mutable strings.
	std::vector<std::string> args;
  	for (int i = 1; i < argc; i++)
		args.push_back(argv[i]);

	int requiredCount = 0;

  	for (int i = 0; static_cast<unsigned int>(i) < args.size(); i++)
	{
		bool matched = false;
		for (ArgListIterator it = _argList.begin(); it != _argList.end(); it++)
        {
			if ( (*it)->processArg( &i, args ) )
			{
				requiredCount += _xorHandler.check( *it );
				matched = true;
				break;
			}
        }

		// checks to see if the argument is an empty combined switch ...
		// and if so, then we've actually matched it
		if ( !matched && _emptyCombined( args[i] ) )
			matched = true;

		if ( !matched && !Arg::ignoreRest() )
			throw(CmdLineParseException("Couldn't find match for argument",
			                             args[i]));
    }

	if ( requiredCount < _numRequired )
		throw(CmdLineParseException("One or more required arguments missing!"));

	if ( requiredCount > _numRequired )
		throw(CmdLineParseException("Too many arguments!"));

	return true; // Ok

	}
	catch ( ActionDoneException e )
	{
		return false; // Done
	}
	catch ( ArgException e )
	{
		_output->failure(*this,e);
		return false; // Error
	}
}

inline bool CmdLine::_emptyCombined(const std::string& s)
{
	if ( s[0] != Arg::flagStartChar() )
		return false;

	for ( int i = 1; static_cast<unsigned int>(i) < s.length(); i++ )
		if ( s[i] != Arg::blankChar() )
			return false;

	return true;
}

inline void CmdLine::deleteOnExit(Arg* ptr)
{
	_argDeleteOnExitList.push_back(ptr);
}

inline void CmdLine::deleteOnExit(Visitor* ptr)
{
	_visitorDeleteOnExitList.push_back(ptr);
}

inline CmdLineOutput* CmdLine::getOutput()
{
	return _output;
}

inline void CmdLine::setOutput(CmdLineOutput* co)
{
	_userSetOutput = true;
	_output = co;
}

inline std::string& CmdLine::getVersion()
{
	return _version;
}

inline std::string& CmdLine::getProgramName()
{
	return _progName;
}

inline std::list<Arg*>& CmdLine::getArgList()
{
	return _argList;
}

inline XorHandler& CmdLine::getXorHandler()
{
	return _xorHandler;
}

inline char CmdLine::getDelimiter()
{
	return _delimiter;
}

inline std::string& CmdLine::getMessage()
{
	return _message;
}

inline bool CmdLine::hasHelpAndVersion()
{
	return _helpAndVersion;
}

///////////////////////////////////////////////////////////////////////////////
//End CmdLine.cpp
///////////////////////////////////////////////////////////////////////////////



} //namespace TCLAP
#endif

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/******************************************************************************
 *
 *  file:  ArgException.h
 *
 *  Copyright (c) 2003, Michael E. Smoot .
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


#pragma once

#include <string>
#include <exception>
#include <stdexcept>

namespace TCLAP {

/**
 * A simple class that defines and argument exception.  Should be caught
 * whenever a CmdLine is created and parsed.
 */
class ArgException : public std::exception
{
	public:

		/**
		 * Constructor.
		 * \param text - The text of the exception.
		 * \param id - The text identifying the argument source.
		 * \param td - Text describing the type of ArgException it is.
		 * of the exception.
		 */
		ArgException( const std::string& text = "undefined exception",
					  const std::string& id = "undefined",
					  const std::string& td = "Generic ArgException")
			: std::exception(),
			  _errorText(text),
			  _argId( id ),
			  _typeDescription(td)
		{ }

		/**
		 * Destructor.
		 */
		~ArgException() noexcept override = default;

		/**
		 * Returns the error text.
		 */
		std::string error() const { return ( _errorText ); }

		/**
		 * Returns the argument id.
		 */
		std::string argId() const
		{
			if ( _argId == "undefined" )
				return " ";
			else
				return ( "Argument: " + _argId );
		}

		/**
		 * Returns the arg id and error text.
		 */
		const char* what() const noexcept override
		{
			static std::string ex;
			ex = _argId + " -- " + _errorText;
			return ex.c_str();
		}

		/**
		 * Returns the type of the exception.  Used to explain and distinguish
		 * between different child exceptions.
		 */
		std::string typeDescription() const
		{
			return _typeDescription;
		}


	private:

		/**
		 * The text of the exception message.
		 */
		std::string _errorText;

		/**
		 * The argument related to this exception.
		 */
		std::string _argId;

		/**
		 * Describes the type of the exception.  Used to distinguish
		 * between different child exceptions.
		 */
		std::string _typeDescription;

};

/**
 * Thrown from within the child Arg classes when it fails to properly
 * parse the argument it has been passed.
 */
class ArgParseException : public ArgException
{
	public:
		/**
		 * Constructor.
		 * \param text - The text of the exception.
		 * \param id - The text identifying the argument source
		 * of the exception.
		 */
		ArgParseException( const std::string& text = "undefined exception",
					       const std::string& id = "undefined" )
			: ArgException( text,
			                id,
							std::string( "Exception found while parsing " ) +
							std::string( "the value the Arg has been passed." ))
			{ }
};

/**
 * Thrown from CmdLine when the arguments on the command line are not
 * properly specified, e.g. too many arguments, required argument missing, etc.
 */
class CmdLineParseException : public ArgException
{
	public:
		/**
		 * Constructor.
		 * \param text - The text of the exception.
		 * \param id - The text identifying the argument source
		 * of the exception.
		 */
		CmdLineParseException( const std::string& text = "undefined exception",
					           const std::string& id = "undefined" )
			: ArgException( text,
			                id,
							std::string( "Exception found when the values ") +
							std::string( "on the command line do not meet ") +
							std::string( "the requirements of the defined ") +
							std::string( "Args." ))
		{ }
};

/**
 * Thrown from Arg and CmdLine when an Arg is improperly specified, e.g.
 * same flag as another Arg, same name, etc.
 */
class SpecificationException : public ArgException
{
	public:
		/**
		 * Constructor.
		 * \param text - The text of the exception.
		 * \param id - The text identifying the argument source
		 * of the exception.
		 */
		SpecificationException( const std::string& text = "undefined exception",
					            const std::string& id = "undefined" )
			: ArgException( text,
			                id,
							std::string("Exception found when an Arg object ")+
							std::string("is improperly defined by the ") +
							std::string("developer." ))
		{ }

};

/**
 * (Added by JLBC for MRPT): An exception that indicates to CmdLine::parse that
 *   help,version,... have been invoked so it should return false for the main program to exit.
 */
class ActionDoneException : public std::runtime_error
{
public:
	ActionDoneException(const std::string &text = std::string() ) :
	  std::runtime_error(text.c_str())
	{
	}
};


} // namespace TCLAP



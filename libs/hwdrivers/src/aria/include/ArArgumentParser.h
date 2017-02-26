/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARARGUMENTPARSER_H
#define ARARGUMENTPARSER_H

#include "ariaTypedefs.h"
#include "ArArgumentBuilder.h"

/// Class for parsing arguments
/**
   This class is made for parsing arguments from the argv and argc variables
   passed into a program's main() function by the operating system, from
   an ArArgumentBuilder object, or just from a string (e.g. provided by WinMain()
   in a Windows MFC program.)

   It will also load default argument values if you call
   loadDefaultArguments().   Aria::init() adds the file /etc/Aria.args and the
   environment variable ARIAARGS as locations for argument defaults, so
   loadDefaultArguments() will always search those. You can
   use this mechanism to avoid needing to always supply command line parameters
   to all programs. For example, if you use different serial ports than the defaults
   for the robot and laser, you can put a -robotPort or -laserPort argument in
   /etc/Aria.args for all programs that call loadDefaultArguments() to use.
   You can add other files or environment variables
   to the list of default argument locations with
   addDefaultArgumentFile() and addDefaultArgumentEnv().
**/
class ArArgumentParser
{
public:
  /// Constructor, takes the argc argv
  AREXPORT ArArgumentParser(int *argc, char **argv);
  /// Constructor, takes an argument builder
  AREXPORT ArArgumentParser(ArArgumentBuilder *builder);
  /// Destructor
  AREXPORT ~ArArgumentParser();
  /// Returns true if the argument was found
  AREXPORT bool checkArgument(const char *argument);
  /// Returns the word/argument after given argument
  AREXPORT char *checkParameterArgument(const char *argument,
					bool returnFirst = false);
  /// Returns the word/argument after given argument
  AREXPORT bool checkParameterArgumentString(const char *argument,  	// Mod JLBC
					     const char **dest,
					     bool *wasReallySet = NULL,
					     bool returnFirst = false);
  /// Returns the integer after given argument
  AREXPORT bool checkParameterArgumentInteger(const char *argument, int *dest,
					      bool *wasReallySet = NULL,
					      bool returnFirst = false);
  /// Returns the word/argument after given argument
  AREXPORT bool checkParameterArgumentBool(char *argument, bool *dest,
					   bool *wasReallySet = NULL,
					   bool returnFirst = false);
  /// Returns the floating point number after given argument
  AREXPORT bool checkParameterArgumentFloat(char *argument, float *dest,
                bool *wasReallySet = NULL, bool returnFirst = false);
  /// Adds a string as a default argument
  AREXPORT void addDefaultArgument(const char *argument, int position = -1);
  /// Adds args from default files and environmental variables
  AREXPORT void loadDefaultArguments(int positon = 1);
  /// Checks for the help strings and warns about unparsed arguments
  AREXPORT bool checkHelpAndWarnUnparsed(unsigned int numArgsOkay = 0);
  /// Gets how many arguments are left in this parser
  AREXPORT size_t getArgc(void) const;
  /// Gets the argv
  AREXPORT char** getArgv(void) const;
  /// Gets a specific argument
  AREXPORT const char* getArg(size_t whichArg) const;
  /// Prints out the arguments left in this parser
  AREXPORT void log(void) const;
  /// Internal function to remove an argument that was parsed
  AREXPORT void removeArg(size_t which);
  /// Adds another file or environmental variable to the list of defaults
  AREXPORT static void addDefaultArgumentFile(const char *file);
  /// Adds another file or environmental variable to the list of defaults
  AREXPORT static void addDefaultArgumentEnv(const char *env);
  /// Logs the default argument locations
  AREXPORT static void logDefaultArgumentLocations(void);
#ifndef SWIG
  /** @brief Returns true if the argument was found
   *  @swigomit
   */
  AREXPORT bool checkArgumentVar(const char *argument, ...);
  /** @brief Returns the word/argument after given argument
   *  @swigomit
   */
  AREXPORT char *checkParameterArgumentVar(char *argument, ...);
  /** @brief Returns the word/argument after given argument
   *  @swigomit
   */
  AREXPORT bool checkParameterArgumentStringVar(bool *wasReallySet,
						const char **dest,
						const char *argument, ...);
  /** @brief Returns the word/argument after given argument
   *  @swigomit
   */
  AREXPORT bool checkParameterArgumentBoolVar(bool *wasReallySet, bool *dest,
					      const char *argument, ...);
  /** @brief Returns the integer after given argument
   *  @swigomit
   */
  AREXPORT bool checkParameterArgumentIntegerVar(bool *wasReallySet, int *dest,
						 const char *argument, ...);
#endif
protected:
  static std::list<std::string> ourDefaultArgumentLocs;
  static std::list<bool> ourDefaultArgumentLocIsFile;
  bool myOwnBuilder;
  ArArgumentBuilder *myBuilder;
  bool myUsingBuilder;
  char **myArgv;
  int *myArgc;
  char myEmptyArg[1];
};


#endif // ARARGUMENTPARSER_H

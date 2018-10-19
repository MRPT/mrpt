/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>
#include <stdexcept>
#include <iosfwd>

namespace mrpt::config
{
// Frwd. decls:
class CConfigFileBase;

/** This is a virtual base class for sets of options than can be loaded from
 * and/or saved to configuration plain-text files.
 * \todo Automatize this class thru a proxy auxiliary class where variables are
 * registered from pointers, etc...
 * \ingroup mrpt_base_grp
 */
class CLoadableOptions
{
   protected:
	/** Used to print variable info from dumpToTextStream with the macro
	 * LOADABLEOPTS_DUMP_VAR */
	static void dumpVar_int(std::ostream& out, const char* varName, int v);
	static void dumpVar_float(std::ostream& out, const char* varName, float v);
	static void dumpVar_double(
		std::ostream& out, const char* varName, double v);
	static void dumpVar_bool(std::ostream& out, const char* varName, bool v);
	static void dumpVar_string(
		std::ostream& out, const char* varName, const std::string& v);

   public:
	/** This method load the options from a ".ini"-like file or memory-stored
	 * string list.
	 *   Only those parameters found in the given "section" and having
	 *   the same name that the variable are loaded. Those not found in
	 *   the file will stay with their previous values (usually the default
	 *   values loaded at initialization). An example of an ".ini" file:
	 *  \code
	 *  [section]
	 *  resolution    = 0.10   // blah blah...
	 *  modeSelection = 1      // 0=blah, 1=blah,...
	 *  \endcode
	 *
	 * \sa loadFromConfigFileName, saveToConfigFile
	 */
	virtual void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& source,
		const std::string& section) = 0;

	/** Behaves like loadFromConfigFile, but you can pass directly a file name
	 * and a temporary CConfigFile object will be created automatically to load
	 * the file.
	 * \sa loadFromConfigFile
	 */
	void loadFromConfigFileName(
		const std::string& config_file, const std::string& section);

	/** This method saves the options to a ".ini"-like file or memory-stored
	 * string list.
	 * \sa loadFromConfigFile, saveToConfigFileName
	 */
	virtual void saveToConfigFile(
		mrpt::config::CConfigFileBase& target,
		const std::string& section) const;

	/** Behaves like saveToConfigFile, but you can pass directly a file name and
	 * a temporary CConfigFile object will be created automatically to save the
	 * file.
	 * \sa saveToConfigFile, loadFromConfigFileName
	 */
	void saveToConfigFileName(
		const std::string& config_file, const std::string& section) const;

	/** Just like \a dumpToTextStream() but sending the text to the console
	 * (std::cout) */
	void dumpToConsole() const;

	/** This method should clearly display all the contents of the structure in
	 * textual form, sending it to a std::ostream.
	 * The default implementation in this base class relies on \a
	 * saveToConfigFile() to generate a plain text representation of all the
	 * parameters.
	 */
	virtual void dumpToTextStream(std::ostream& out) const;

	/** Virtual destructor */
	virtual ~CLoadableOptions() = default;
};  // End of class def.

/** Macro for dumping a variable to a stream, within the method
 * "dumpToTextStream(out)" (Variable types are: int, double, float, bool, string
 */
#define LOADABLEOPTS_DUMP_VAR(variableName, variableType)                 \
	{                                                                     \
		dumpVar_##variableType(                                           \
			out, #variableName, static_cast<variableType>(variableName)); \
	}

/** Macro for dumping a variable to a stream, transforming the argument from
 * radians to degrees.  */
#define LOADABLEOPTS_DUMP_VAR_DEG(variableName)                              \
	{                                                                        \
		dumpVar_double(                                                      \
			out, #variableName, RAD2DEG(static_cast<double>(variableName))); \
	}
}  // namespace mrpt::config

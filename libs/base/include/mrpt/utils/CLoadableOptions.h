/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
#ifndef  CLoadableOptions_H
#define  CLoadableOptions_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
namespace utils
{
	/** This is a virtual base class for sets of options than can be loaded from and/or saved to configuration plain-text files.
	  * \todo Automatize this class thru a proxy auxiliary class where variables are registered from pointers, etc...
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP  CLoadableOptions
	{
	protected:

		/** Used to print variable info from dumpToTextStream with the macro LOADABLEOPTS_DUMP_VAR */
		static void dumpVar_int( CStream &out, const char *varName, int v );
		static void dumpVar_float( CStream &out, const char *varName, float v );
		static void dumpVar_double( CStream &out, const char *varName, double v );
		static void dumpVar_bool( CStream &out, const char *varName, bool v );
		static void dumpVar_string( CStream &out, const char *varName, const std::string &v );


	public:
		/** This method load the options from a ".ini"-like file or memory-stored string list.
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
		virtual void  loadFromConfigFile(
			const mrpt::utils::CConfigFileBase & source,
			const std::string                  & section) = 0;

		/** Behaves like loadFromConfigFile, but you can pass directly a file name and a temporary CConfigFile object will be created automatically to load the file.
		  * \sa loadFromConfigFile
		  */
		void  loadFromConfigFileName(
			const std::string   & config_file,
			const std::string   & section);

		/** This method saves the options to a ".ini"-like file or memory-stored string list.
		 * \sa loadFromConfigFile, saveToConfigFileName
		 */
		virtual void  saveToConfigFile(
			mrpt::utils::CConfigFileBase &target,
			const std::string            &section) const
			{
				THROW_EXCEPTION("The child class does not implement this method.");
			}

		/** Behaves like saveToConfigFile, but you can pass directly a file name and a temporary CConfigFile object will be created automatically to save the file.
		  * \sa saveToConfigFile, loadFromConfigFileName
		  */
		void  saveToConfigFileName(
			const std::string		&config_file,
			const std::string		&section)  const;

		/** Just like \a dumpToTextStream() but sending the text to the console (std::cout) */
		void  dumpToConsole() const;

		/** This method should clearly display all the contents of the structure in textual form, sending it to a CStream.
		  * The default implementation in this base class relies on \a saveToConfigFile() to generate a plain text representation of all the parameters.
		  */
		virtual void  dumpToTextStream(CStream	&out) const;

        /** Virtual destructor
          */
        virtual ~CLoadableOptions()
		{
		}

	}; // End of class def.

	/** Macro for dumping a variable to a stream, within the method "dumpToTextStream(out)" (Variable types are: int, double, float, bool, string  */
	#define LOADABLEOPTS_DUMP_VAR(variableName,variableType) { dumpVar_##variableType(out, #variableName,static_cast<variableType>(variableName)); }

	/** Macro for dumping a variable to a stream, transforming the argument from radians to degrees.  */
	#define LOADABLEOPTS_DUMP_VAR_DEG(variableName) { dumpVar_double(out, #variableName,RAD2DEG(static_cast<double>(variableName))); }

	} // End of namespace
} // end of namespace
#endif

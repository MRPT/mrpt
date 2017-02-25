/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_utils_parameters_H
#define mrpt_utils_parameters_H

#include <cstdarg>
#include <cstdio>
#include <map>
#include <string>

namespace mrpt
{
	namespace utils
	{
		/** For usage when passing a dynamic number of (numeric) arguments to a function, by name.
		  *  \code
		  *    TParameters<double> p;  // or TParametersDouble
		  *    p["v_max"] = 1.0;  // Write
		  *    ...
		  *    cout << p["w_max"]; // Read, even if "p" was const.
		  *  \endcode
		  *
		  *  A default list of parameters can be passed to the constructor as a sequence
		  *   of pairs "name, value", which MUST end in a NULL name string. Names MUST BE "const char*"
		  *   (that is, "old plain strings" are OK), not std::string objects!.
		  *  See this example:
		  *
		  *  \code
		  *    TParameters<double> p("par1",2.0, "par2",-4.5, "par3",9.0, NULL); // MUST end with a NULL
		  *  \endcode
		  *
		  *  <b>VERY IMPORTANT:</b> If you use the NULL-ended constructor above, make sure all the values are of the proper
		  *    type or it will crash in runtime. For example, in a TParametersDouble all values must be double's, so
		  *    if you type "10" the compiler will make it an "int". Instead, write "10.0".
		  * \ingroup mrpt_base_grp
		  * \sa the example in MRPT/samples/params-by-name
		  */
		template <typename T>
		struct TParameters : public std::map<std::string,T>
		{
			typedef std::map<std::string,T> BASE;
			/** Default constructor (initializes empty) */
			TParameters() : BASE() { }
			/** Constructor with a list of initial values (see the description and use example in mrpt::utils::TParameters) */
			TParameters(const char *nam1,...) : BASE() {
				if (!nam1) return; // No parameters
				T val;
				va_list args;
				va_start(args,nam1);
				// 1st one out of the loop:
				val = va_arg(args,T);
				BASE::operator[](std::string(nam1)) = val;
				// Loop until NULL:
				const char *nam;
				do{
					nam = va_arg(args,const char*);
					if (nam) {
						val = va_arg(args,T);
						BASE::operator[](std::string(nam)) = val;
					}
				} while (nam);
				va_end(args);
			}
			inline bool has(const std::string &s) const { return std::map<std::string,T>::end()!=BASE::find(s); }
			/** A const version of the [] operator, for usage as read-only.
			  * \exception std::logic_error On parameter not present. Please, check existence with "has" before reading.
			  */
			inline T operator[](const std::string &s) const {
				typename BASE::const_iterator it =BASE::find(s);
				if (BASE::end()==it)
					throw std::logic_error(std::string("Parameter '")+s+std::string("' is not present.").c_str());
				return it->second;
			}
			/** A const version of the [] operator and with a default value in case the parameter is not set (for usage as read-only).
			  */
			inline T getWithDefaultVal(const std::string &s, const T& defaultVal) const {
				typename BASE::const_iterator it =BASE::find(s);
				if (BASE::end()==it)
						return defaultVal;
				else 	return it->second;
			}
			/** The write (non-const) version of the [] operator. */
			inline T & operator[](const std::string &s) { return BASE::operator[](s); }

			/** Dumps to console the output from getAsString() */
			inline void dumpToConsole() const { ::fputs(getAsString().c_str(),stdout); }

			/** Returns a multi-line string representation of the parameters like : 'nam   = val\nnam2   = val2...' */
			inline std::string getAsString() const { std::string s; getAsString(s); return s; }

			/** Returns a multi-line string representation of the parameters like : 'nam   = val\nnam2   = val2...' */
			void getAsString(std::string &s) const {
				size_t maxStrLen = 10;
				for (typename BASE::const_iterator it=BASE::begin();it!=BASE::end();++it) maxStrLen = std::max(maxStrLen, it->first.size() );
				maxStrLen++;
				std::stringstream str;
				for (typename BASE::const_iterator it=BASE::begin();it!=BASE::end();++it)
					str << it->first << std::string(maxStrLen-it->first.size(),' ') << " = " << it->second << std::endl;
				s = str.str();
			}
		};

		typedef TParameters<double>       TParametersDouble; //!< See the generic template mrpt::utils::TParameters
		typedef TParameters<std::string>  TParametersString; //!< See the generic template mrpt::utils::TParameters

	} // end namespace
}

#endif


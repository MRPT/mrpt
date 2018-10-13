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
#include <vector>
#include <type_traits>
#include <mrpt/system/string_utils.h>  // tokenize
#include <mrpt/core/exceptions.h>
#include <mrpt/core/bits_math.h>  // DEG2RAD()

namespace mrpt
{
// Frd decl:
namespace typemeta
{
template <typename ENUMTYPE>
struct TEnumType;
}
namespace config
{
// Frwd. decl:
class CConfigFilePrefixer;

/** Default padding sizes for macros MRPT_SAVE_CONFIG_VAR_COMMENT(), etc. */
int MRPT_SAVE_NAME_PADDING();
int MRPT_SAVE_VALUE_PADDING();

/** This class allows loading and storing values and vectors of different types
 * from a configuration text, which can be implemented as a ".ini" file, a
 * memory-stored string, etc...
 *   This is a virtual class, use only as a pointer to an implementation of one
 * of the derived classes.
 *
 * See: \ref config_file_format
 * \ingroup mrpt_base_grp
 */
class CConfigFileBase
{
	friend class CConfigFilePrefixer;

   protected:
	/** A virtual method to write a generic string.
	 */
	virtual void writeString(
		const std::string& section, const std::string& name,
		const std::string& str) = 0;

	/** Write a generic string with optional padding and a comment field ("//
	 * ...") at the end of the line. */
	void writeString(
		const std::string& section, const std::string& name,
		const std::string& str, const int name_padding_width,
		const int value_padding_width, const std::string& comment);

	/** A virtual method to read a generic string.
	 * \exception std::exception If the key name is not found and
	 * "failIfNotFound" is true. Otherwise the "defaultValue" is returned. */
	virtual std::string readString(
		const std::string& section, const std::string& name,
		const std::string& defaultStr, bool failIfNotFound = false) const = 0;

   public:
	/** dtor */
	virtual ~CConfigFileBase();

	/** Returns a list with all the section names. */
	virtual void getAllSections(std::vector<std::string>& sections) const = 0;

	/** Returs a list with all the keys into a section */
	virtual void getAllKeys(
		const std::string& section, std::vector<std::string>& keys) const = 0;

	/** Checks if a given section exists (name is case insensitive) */
	bool sectionExists(const std::string& section_name) const;

	template <
		typename enum_t,
		typename = std::enable_if_t<std::is_enum<enum_t>::value>>
	void write(
		const std::string& section, const std::string& name, enum_t value,
		const int name_padding_width = -1, const int value_padding_width = -1,
		const std::string& comment = std::string())
	{
		this->write(
			section, name, mrpt::typemeta::TEnumType<enum_t>::value2name(value),
			name_padding_width, value_padding_width, comment);
	}
	/** @name Save a configuration parameter. Optionally pads with spaces up to
	 * the desired width in number of characters (-1: no fill), and add a final
	 * comment field at the end of the line (a "// " prefix is automatically
	 * inserted).
	 * @{ */
	template <
		typename data_t,
		typename = std::enable_if_t<!std::is_enum<data_t>::value>>
	void write(
		const std::string& section, const std::string& name,
		const data_t& value, const int name_padding_width = -1,
		const int value_padding_width = -1,
		const std::string& comment = std::string())
	{
		writeString(
			section, name, mrpt::to_string(value), name_padding_width,
			value_padding_width, comment);
	}
	template <typename data_t>
	void write(
		const std::string& section, const std::string& name,
		const std::vector<data_t>& value, const int name_padding_width = -1,
		const int value_padding_width = -1,
		const std::string& comment = std::string())
	{
		std::string s;
		for (typename std::vector<data_t>::const_iterator it = value.begin();
			 it != value.end(); ++it)
		{
			s += mrpt::to_string(*it);
			s += " ";
		}
		writeString(
			section, name, s, name_padding_width, value_padding_width, comment);
	}
	void write(
		const std::string& section, const std::string& name, double value,
		const int name_padding_width = -1, const int value_padding_width = -1,
		const std::string& comment = std::string());
	void write(
		const std::string& section, const std::string& name, float value,
		const int name_padding_width = -1, const int value_padding_width = -1,
		const std::string& comment = std::string());

	/** @} */

	/** @name Read a configuration parameter, launching exception if key name is
	 * not found and `failIfNotFound`=true
	 * @{ */
	double read_double(
		const std::string& section, const std::string& name,
		double defaultValue, bool failIfNotFound = false) const;
	float read_float(
		const std::string& section, const std::string& name, float defaultValue,
		bool failIfNotFound = false) const;
	bool read_bool(
		const std::string& section, const std::string& name, bool defaultValue,
		bool failIfNotFound = false) const;
	int read_int(
		const std::string& section, const std::string& name, int defaultValue,
		bool failIfNotFound = false) const;
	uint64_t read_uint64_t(
		const std::string& section, const std::string& name,
		uint64_t defaultValue, bool failIfNotFound = false) const;
	std::string read_string(
		const std::string& section, const std::string& name,
		const std::string& defaultValue, bool failIfNotFound = false) const;
	/** Reads a configuration parameter of type "string", and keeps only the
	 * first word (this can be used to eliminate possible comments at the end of
	 * the line) */
	std::string read_string_first_word(
		const std::string& section, const std::string& name,
		const std::string& defaultValue, bool failIfNotFound = false) const;
	/** Reads a configuration parameter of type vector, stored in the file as a
	 * string: "[v1 v2 v3 ... ]", where spaces could also be commas. \exception
	 * std::exception If the key name is not found and "failIfNotFound" is true.
	 * Otherwise the "defaultValue" is returned. */
	template <class VECTOR_TYPE>
	void read_vector(
		const std::string& section, const std::string& name,
		const VECTOR_TYPE& defaultValue, VECTOR_TYPE& outValues,
		bool failIfNotFound = false) const
	{
		std::string aux(readString(section, name, "", failIfNotFound));
		// Parse the text into a vector:
		std::vector<std::string> tokens;
		mrpt::system::tokenize(aux, "[], \t", tokens);

		if (tokens.size() == 0)
		{
			outValues = defaultValue;
		}
		else
		{
			// Parse to numeric type:
			const size_t N = tokens.size();
			outValues.resize(N);
			for (size_t i = 0; i < N; i++)
			{
				double val = std::stod(tokens[i]);
				outValues[i] = val;
			}
		}
	}

	/** Reads a configuration parameter as a matrix written in a matlab-like
	 * format - for example: "[2 3 4 ; 7 8 9]".
	 * This template method can be instantiated for matrices of the types: int,
	 * long, unsinged int, unsigned long, float, double, long double
	 * \exception std::exception If the key name is not found and
	 * "failIfNotFound" is true. Otherwise the "defaultValue" is returned.
	 */
	template <class MATRIX_TYPE>
	void read_matrix(
		const std::string& section, const std::string& name,
		MATRIX_TYPE& outMatrix,
		const MATRIX_TYPE& defaultMatrix = MATRIX_TYPE(),
		bool failIfNotFound = false) const
	{
		std::string aux = readString(section, name, "", failIfNotFound);
		if (aux.empty())
			outMatrix = defaultMatrix;
		else
		{
			// Parse the text into a vector:
			if (!outMatrix.fromMatlabStringFormat(aux))
				THROW_EXCEPTION_FMT("Error parsing matrix: '%s'", aux.c_str())
		}
	}

	/** Reads an "enum" value, where the value in the config file can be either
	 * a numerical value or the symbolic name, for example:
	 *   In the code:
	 *  \code
	 *    enum my_type_t { type_foo=0, type_bar };
	 *  \endcode
	 *  In the config file:
	 *  \code
	 *    [section]
	 *    type   = type_bar   // Use the symbolic name, or
	 *    type   = 1          // use the numerical value (both lines will be
	 * equivalent)
	 *  \endcode
	 *  Which can be loaded with:
	 *  \code
	 *    cfgfile.read_enum<my_type_t>("section","type", type_foo );
	 *  \endcode
	 *
	 *  \note For an enum type to work with this template it is required that
	 * it defines a specialization of mrpt::typemeta::TEnumType
	 */
	template <typename ENUMTYPE>
	ENUMTYPE read_enum(
		const std::string& section, const std::string& name,
		const ENUMTYPE& defaultValue, bool failIfNotFound = false) const
	{
		MRPT_START
		const std::string sVal =
			read_string_first_word(section, name, "", failIfNotFound);
		if (sVal.empty()) return defaultValue;
		// Text or numeric value?
		if (::isdigit(sVal[0]))
		{  // Seems a number:
			return static_cast<ENUMTYPE>(::atoi(&sVal[0]));
		}
		else
		{  // Name look-up:
			try
			{
				return mrpt::typemeta::TEnumType<ENUMTYPE>::name2value(sVal);
			}
			catch (std::exception&)
			{
				THROW_EXCEPTION_FMT(
					"Invalid value '%s' for enum type while reading key='%s'.",
					sVal.c_str(), name.c_str())
			}
		}
		MRPT_END
	}
	/** @} */
};  // End of class def.

/** An useful macro for loading variables stored in a INI-like file under a key
 * with the same name that the variable, and assigning the variable the current
 * value if not found in the config file.
 *  The variableType must be the suffix of "read_XXX" functions, i.e. int,
 * bool,...
 */
#define MRPT_LOAD_CONFIG_VAR(                                     \
	variableName, variableType, configFileObject, sectionNameStr) \
	{                                                             \
		variableName = configFileObject.read_##variableType(      \
			sectionNameStr, #variableName, variableName);         \
	}

/** Shortcut for MRPT_LOAD_CONFIG_VAR() for config file object named `c` and
 * section string named `s` */
#define MRPT_LOAD_CONFIG_VAR_CS(variableName, variableType) \
	MRPT_LOAD_CONFIG_VAR(variableName, variableType, c, s)

/** Loads a double variable, stored as radians but entered in the INI-file as
 * degrees */
#define MRPT_LOAD_CONFIG_VAR_DEGREES(                                     \
	variableName, configFileObject, sectionNameStr)                       \
	{                                                                     \
		variableName = mrpt::DEG2RAD(configFileObject.read_double(        \
			sectionNameStr, #variableName, mrpt::RAD2DEG(variableName))); \
	}

/** Loads a double, required, variable, stored as radians but entered in the
 * INI-file as degrees */
#define MRPT_LOAD_CONFIG_VAR_DEGREES_NO_DEFAULT(                        \
	variableName, configFileObject, sectionNameStr)                     \
	{                                                                   \
		variableName = mrpt::DEG2RAD(configFileObject.read_double(      \
			sectionNameStr, #variableName, mrpt::RAD2DEG(variableName), \
			true));                                                     \
	}

#define MRPT_LOAD_CONFIG_VAR_CAST(                                  \
	variableName, variableType, variableTypeCast, configFileObject, \
	sectionNameStr)                                                 \
	{                                                               \
		variableName = static_cast<variableTypeCast>(               \
			configFileObject.read_##variableType(                   \
				sectionNameStr, #variableName, variableName));      \
	}

#define MRPT_LOAD_HERE_CONFIG_VAR(                                \
	variableName, variableType, targetVariable, configFileObject, \
	sectionNameStr)                                               \
	targetVariable = configFileObject.read_##variableType(        \
		sectionNameStr, #variableName, targetVariable, false);

#define MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(                              \
	variableName, variableType, targetVariable, configFileObject,          \
	sectionNameStr)                                                        \
	{                                                                      \
		try                                                                \
		{                                                                  \
			targetVariable = configFileObject.read_##variableType(         \
				sectionNameStr, #variableName, targetVariable, true);      \
		}                                                                  \
		catch (std::exception&)                                            \
		{                                                                  \
			THROW_EXCEPTION(mrpt::format(                                  \
				"Value for '%s' not found in config file in section '%s'", \
				static_cast<const char*>(#variableName),                   \
				std::string(sectionNameStr).c_str()));                     \
		}                                                                  \
	}

#define MRPT_LOAD_HERE_CONFIG_VAR_DEGREES(                               \
	variableName, variableType, targetVariable, configFileObject,        \
	sectionNameStr)                                                      \
	targetVariable = mrpt::DEG2RAD(configFileObject.read_##variableType( \
		sectionNameStr, #variableName, mrpt::RAD2DEG(targetVariable), false));

#define MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(                      \
	variableName, variableType, targetVariable, configFileObject,          \
	sectionNameStr)                                                        \
	{                                                                      \
		try                                                                \
		{                                                                  \
			targetVariable =                                               \
				mrpt::DEG2RAD(configFileObject.read_##variableType(        \
					sectionNameStr, #variableName, targetVariable, true)); \
		}                                                                  \
		catch (std::exception&)                                            \
		{                                                                  \
			THROW_EXCEPTION(mrpt::format(                                  \
				"Value for '%s' not found in config file in section '%s'", \
				static_cast<const char*>(#variableName),                   \
				std::string(sectionNameStr).c_str()));                     \
		}                                                                  \
	}

#define MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(                                   \
	variableName, variableType, configFileObject, sectionNameStr)          \
	{                                                                      \
		try                                                                \
		{                                                                  \
			variableName = configFileObject.read_##variableType(           \
				sectionNameStr, #variableName, variableName, true);        \
		}                                                                  \
		catch (std::exception&)                                            \
		{                                                                  \
			THROW_EXCEPTION(mrpt::format(                                  \
				"Value for '%s' not found in config file in section '%s'", \
				static_cast<const char*>(#variableName),                   \
				std::string(sectionNameStr).c_str()));                     \
		}                                                                  \
	}

/** Shortcut for MRPT_LOAD_CONFIG_VAR_NO_DEFAULT() for REQUIRED variables config
 * file object named `c` and section string named `s` */
#define MRPT_LOAD_CONFIG_VAR_REQUIRED_CS(variableName, variableType) \
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(variableName, variableType, c, s)

#define MRPT_LOAD_CONFIG_VAR_CAST_NO_DEFAULT(                              \
	variableName, variableType, variableTypeCast, configFileObject,        \
	sectionNameStr)                                                        \
	{                                                                      \
		try                                                                \
		{                                                                  \
			variableName = static_cast<variableTypeCast>(                  \
				configFileObject.read_##variableType(                      \
					sectionNameStr, #variableName, variableName, true));   \
		}                                                                  \
		catch (std::exception&)                                            \
		{                                                                  \
			THROW_EXCEPTION(mrpt::format(                                  \
				"Value for '%s' not found in config file in section '%s'", \
				static_cast<const char*>(#variableName),                   \
				std::string(sectionNameStr).c_str()));                     \
		}                                                                  \
	}

#define MRPT_LOAD_HERE_CONFIG_VAR_CAST(                                     \
	variableName, variableType, variableTypeCast, targetVariable,           \
	configFileObject, sectionNameStr)                                       \
	targetVariable =                                                        \
		static_cast<variableTypeCast>(configFileObject.read_##variableType( \
			sectionNameStr, #variableName, targetVariable));

#define MRPT_LOAD_HERE_CONFIG_VAR_CAST_NO_DEFAULT(                         \
	variableName, variableType, variableTypeCast, targetVariable,          \
	configFileObject, sectionNameStr)                                      \
	{                                                                      \
		try                                                                \
		{                                                                  \
			targetVariable = static_cast<variableTypeCast>(                \
				configFileObject.read_##variableType(                      \
					sectionNameStr, #variableName, targetVariable, true)); \
		}                                                                  \
		catch (std::exception&)                                            \
		{                                                                  \
			THROW_EXCEPTION(mrpt::format(                                  \
				"Value for '%s' not found in config file in section '%s'", \
				static_cast<const char*>(#variableName),                   \
				std::string(sectionNameStr).c_str()));                     \
		}                                                                  \
	}

#define MRPT_SAVE_CONFIG_VAR(variableName, configFileObject, sectionNameStr) \
	{                                                                        \
		configFileObject.write(sectionNameStr, #variableName, variableName); \
	}

#define MRPT_SAVE_CONFIG_VAR_DEGREES(                                    \
	variableName, configFileObject, sectionNameStr)                      \
	{                                                                    \
		configFileObject.write(                                          \
			sectionNameStr, #variableName, mrpt::RAD2DEG(variableName)); \
	}

#define MRPT_SAVE_CONFIG_VAR_COMMENT(variableName, __comment)    \
	{                                                            \
		c.write(                                                 \
			s, #variableName, variableName,                      \
			mrpt::config::MRPT_SAVE_NAME_PADDING(),              \
			mrpt::config::MRPT_SAVE_VALUE_PADDING(), __comment); \
	}
#define MRPT_SAVE_CONFIG_VAR_DEGREES_COMMENT(                    \
	__entryName, __variable, __comment)                          \
	{                                                            \
		c.write(                                                 \
			s, __entryName, mrpt::RAD2DEG(__variable),           \
			mrpt::config::MRPT_SAVE_NAME_PADDING(),              \
			mrpt::config::MRPT_SAVE_VALUE_PADDING(), __comment); \
	}

}  // namespace config
}  // namespace mrpt

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

// SimpleIni: Use Debian icu package instead of copyrighted ConvertUTF.h
#define SI_CONVERT_ICU 1

#include <SimpleIni.h>
#include <mrpt/core/format.h>
#include <mrpt/expr/CRuntimeCompiledExpression.h>
#include <mrpt/system/string_utils.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <string>

namespace mrpt::config::simpleini
{
/** MRPT custom INI file parser to allow minimal file preprocessing:
 * - multiline entries via an end-of-line backslash ('\')
 */
struct MRPT_IniFileParser : public SI_ConvertA<char>
{
	MRPT_IniFileParser(bool /*m_bStoreIsUtf8*/) : SI_ConvertA<char>() {}
	/* copy and assignment */
	MRPT_IniFileParser(const MRPT_IniFileParser& rhs) : SI_ConvertA<char>(rhs)
	{
		SI_ConvertA<char>::operator=(rhs);
	}
	MRPT_IniFileParser& operator=(const MRPT_IniFileParser& rhs)
	{
		SI_ConvertA<char>::operator=(rhs);
		return *this;
	}

	size_t SizeFromStore(
		const char* a_pInputData, size_t a_uInputDataLen)  // override
	{
		SI_ASSERT(a_uInputDataLen != (size_t)-1);
		return do_parse(a_pInputData, a_uInputDataLen, nullptr);
	}

	bool ConvertFromStore(
		const char* a_pInputData, size_t a_uInputDataLen, char* a_pOutputData,
		size_t a_uOutputDataSize)  // override
	{
		this->do_parse(a_pInputData, a_uInputDataLen, a_pOutputData);
		return true;
	}

   private:
	struct ParseContext
	{
		std::map<std::string, std::string> defined_vars;
		std::map<std::string, double> defined_vars_values;
		unsigned int line_count = 1;
	};

	// Return a string or a number (as string) if expr = "$eval{...}"
	std::string parse_process_var_eval(const ParseContext& pc, std::string expr)
	{
		expr = mrpt::system::trim(expr);
		while (expr.size() > 5)
		{
			auto p = expr.find("$env{");
			if (p != std::string::npos)
			{
				auto pend = expr.find("}", p);
				if (pend == std::string::npos)
					throw std::runtime_error(mrpt::format(
						"Line %u: Expected closing `}` near: `%s`",
						pc.line_count, expr.c_str()));
				const auto substr = expr.substr(p + 5, pend - p - 5);
				std::string new_expr = expr.substr(0, p);
				auto env_val = ::getenv(substr.c_str());
				if (env_val) new_expr += std::string(env_val);
				new_expr += expr.substr(pend + 1);
				new_expr.swap(expr);
			}
			else if ((p = expr.find("$eval{")) != std::string::npos)
			{
				auto pend = expr.find("}", p);
				if (pend == std::string::npos)
					throw std::runtime_error(mrpt::format(
						"Line %u: Expected closing `}` near: `%s`",
						pc.line_count, expr.c_str()));

				const auto substr = expr.substr(p + 6, pend - p - 6);
				mrpt::expr::CRuntimeCompiledExpression cexpr;
				cexpr.compile(
					substr, pc.defined_vars_values,
					mrpt::format("Line %u: ", pc.line_count));

				std::string new_expr = expr.substr(0, p);
				new_expr += mrpt::format("%e", cexpr.eval());
				new_expr += expr.substr(pend + 1);
				new_expr.swap(expr);
			}
			else
				break;  // nothing else to evaluate
		}
		return expr;
	}

	void parse_process_var_define(
		ParseContext& pc, const std::string& var_name,
		const std::string& var_value)
	{
		if (!var_name.empty())
		{
			pc.defined_vars[var_name] = var_value;
			if (!var_value.empty())
			{
				pc.defined_vars_values[var_name] =
					::atof(parse_process_var_eval(pc, var_value).c_str());
			}
		}
	}

	/** Shared code for the two virtual methods. If out_str==NULL, just count
	 * output bytes */
	size_t do_parse(const char* in_str, const size_t in_len, char* out_str)
	{
		ParseContext pc;
		size_t out_len = 0, i = 0;
		while (i < in_len)
		{
			const char c = in_str[i];
			if (c == '\n')
			{
				pc.line_count++;
			}

			if (c == '\\' && i < in_len - 1 &&
				(in_str[i + 1] == '\r' || in_str[i + 1] == '\n'))
			{
				// Skip the backslash + one newline: CR "\r", LF "\n", CR+LF
				// "\r\n"
				if (i < in_len - 2 && in_str[i + 1] == '\r' &&
					in_str[i + 2] == '\n')
				{
					// out_len += 0;
					i += 3;
				}
				else if (in_str[i + 1] == '\r' || in_str[i + 1] == '\n')
				{
					// out_len += 0;
					i += 2;
				}
				else
				{
					throw std::runtime_error(
						"MRPT_IniFileParser: parse error, shouldn't reach "
						"here!");
				}
			}
			else
			{
				// Handle "@define varname value"
				if (in_len > i + 7 && !::strncmp(in_str + i, "@define", 7))
				{
					// Extract rest of this line:
					i += 7;
					std::string var_name, var_value;
					bool in_var_name = false, done_var_name = false;
					while (i < in_len && in_str[i] != '\r' && in_str[i] != '\n')
					{
						const char ch = in_str[i];
						i++;
						if (ch != ' ' && ch != '\t')
						{
							// not whitespace
							if (!in_var_name && !done_var_name)
							{
								in_var_name = true;
							}
						}
						else
						{
							// whitespace
							if (in_var_name)
							{
								in_var_name = false;
								done_var_name = true;
							}
						}
						if (in_var_name)
						{
							var_name += ch;
						}
						if (done_var_name)
						{
							var_value += ch;
						}
					}

					parse_process_var_define(pc, var_name, var_value);
					continue;
				}

				// Handle "${varname}"
				if (in_len > i + 4 && in_str[i] == '$' && in_str[i + 1] == '{')
				{
					// extract varname:
					i += 2;
					std::string varname;
					bool end_ok = false;
					while (i < in_len && in_str[i] != '\n' && in_str[i] != '\r')
					{
						const char ch = in_str[i];
						i++;
						if (ch == '}')
						{
							end_ok = true;
							break;
						}
						varname += ch;
					}
					if (!end_ok)
					{
						throw std::runtime_error(mrpt::format(
							"Line %u: Expected closing `}` near: `%s`",
							pc.line_count, varname.c_str()));
					}

					const auto it = pc.defined_vars.find(varname);
					if (it == pc.defined_vars.end())
						throw std::runtime_error(mrpt::format(
							"Line %u: Unknown variable `${%s}`", pc.line_count,
							varname.c_str()));

					const auto str_out = parse_process_var_eval(pc, it->second);

					for (const char ch : str_out)
					{
						if (out_str) out_str[out_len] = ch;
						out_len++;
					}
					continue;
				}

				// Handle "$eval{expression}"
				if (in_len > i + 7 && !strncmp(in_str + i, "$eval{", 6))
				{
					// extract expression:
					std::string expr;
					bool end_ok = false;
					while (i < in_len && in_str[i] != '\n' && in_str[i] != '\r')
					{
						const char ch = in_str[i];
						i++;
						expr += ch;
						if (ch == '}')
						{
							end_ok = true;
							break;
						}
					}
					if (!end_ok)
					{
						throw std::runtime_error(mrpt::format(
							"Line %u: Expected closing `}` near: `%s`",
							pc.line_count, expr.c_str()));
					}

					const std::string res = parse_process_var_eval(pc, expr);

					for (const char ch : res)
					{
						if (out_str) out_str[out_len] = ch;
						out_len++;
					}
					continue;
				}

				// Handle "$env{var}"
				if (in_len > i + 6 && !strncmp(in_str + i, "$env{", 5))
				{
					// extract expression:
					std::string expr;
					bool end_ok = false;
					while (i < in_len && in_str[i] != '\n' && in_str[i] != '\r')
					{
						const char ch = in_str[i];
						i++;
						expr += ch;
						if (ch == '}')
						{
							end_ok = true;
							break;
						}
					}
					if (!end_ok)
					{
						throw std::runtime_error(mrpt::format(
							"Line %u: Expected closing `}` near: `%s`",
							pc.line_count, expr.c_str()));
					}

					const std::string res = parse_process_var_eval(pc, expr);

					for (const char ch : res)
					{
						if (out_str) out_str[out_len] = ch;
						out_len++;
					}
					continue;
				}

				// Normal case:
				if (out_str)
				{
					out_str[out_len] = c;
				}
				out_len++;
				i++;
			}
		}
		return out_len;
	}
};

// ---------------------------------------------------------------------------
//                                  TYPE DEFINITIONS
// ---------------------------------------------------------------------------
using MRPT_CSimpleIni =
	CSimpleIniTempl<char, SI_GenericNoCase<char>, MRPT_IniFileParser>;

}  // namespace mrpt::config::simpleini

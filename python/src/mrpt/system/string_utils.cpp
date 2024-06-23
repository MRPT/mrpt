#include <iterator>
#include <memory>
#include <mrpt/system/string_utils.h>
#include <string>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_system_string_utils(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::system::tokenize(const std::string &, const std::string &, class std::vector<std::string > &, bool) file:mrpt/system/string_utils.h line:50
	M("mrpt::system").def("tokenize", [](const std::string & a0, const std::string & a1, class std::vector<std::string > & a2) -> void { return mrpt::system::tokenize(a0, a1, a2); }, "", pybind11::arg("inString"), pybind11::arg("inDelimiters"), pybind11::arg("outTokens"));
	M("mrpt::system").def("tokenize", (void (*)(const std::string &, const std::string &, class std::vector<std::string > &, bool)) &mrpt::system::tokenize<std::vector<std::string >>, "Tokenizes a string according to a set of delimiting characters.\n Example:\n \n\n\n\n\n  Will generate 3 tokens:\n		- \"Pepe\"\n		- \"Er\"\n		- \"Muo\"\n \n\n If `true`, consecutive \"delimiters\" will be\n considered one single delimiters. If `false`, a blank token will be returned\n between each pair of delimiters.\n \n\n Can be a std::vector or std::deque of std::string's.\n\nC++: mrpt::system::tokenize(const std::string &, const std::string &, class std::vector<std::string > &, bool) --> void", pybind11::arg("inString"), pybind11::arg("inDelimiters"), pybind11::arg("outTokens"), pybind11::arg("skipBlankTokens"));

	// mrpt::system::trim(const std::string &) file:mrpt/system/string_utils.h line:69
	M("mrpt::system").def("trim", (std::string (*)(const std::string &)) &mrpt::system::trim, "Removes leading and trailing spaces \n\nC++: mrpt::system::trim(const std::string &) --> std::string", pybind11::arg("str"));

	// mrpt::system::upperCase(const std::string &) file:mrpt/system/string_utils.h line:73
	M("mrpt::system").def("upperCase", (std::string (*)(const std::string &)) &mrpt::system::upperCase, "Returns a upper-case version of a string.\n \n\n lowerCase  \n\nC++: mrpt::system::upperCase(const std::string &) --> std::string", pybind11::arg("str"));

	// mrpt::system::lowerCase(const std::string &) file:mrpt/system/string_utils.h line:77
	M("mrpt::system").def("lowerCase", (std::string (*)(const std::string &)) &mrpt::system::lowerCase, "Returns an lower-case version of a string.\n \n\n upperCase  \n\nC++: mrpt::system::lowerCase(const std::string &) --> std::string", pybind11::arg("str"));

	// mrpt::system::unitsFormat(const double, int, bool) file:mrpt/system/string_utils.h line:112
	M("mrpt::system").def("unitsFormat", [](const double & a0) -> std::string { return mrpt::system::unitsFormat(a0); }, "", pybind11::arg("val"));
	M("mrpt::system").def("unitsFormat", [](const double & a0, int const & a1) -> std::string { return mrpt::system::unitsFormat(a0, a1); }, "", pybind11::arg("val"), pybind11::arg("nDecimalDigits"));
	M("mrpt::system").def("unitsFormat", (std::string (*)(const double, int, bool)) &mrpt::system::unitsFormat, "This function implements formatting with the appropriate SI metric unit\n prefix: 1e-12->'p', 1e-9->'n', 1e-6->'u', 1e-3->'m', 1->'', 1e3->'K',\n 1e6->'M', 1e9->'G', 1e12->'T'\n If the input is exactly 0, it will return the string `\"0\"`, so the overall\n result looks like that (e.g. using meter units on the caller side):\n \n\n\n\n\n \n\n intervalFormat \n\nC++: mrpt::system::unitsFormat(const double, int, bool) --> std::string", pybind11::arg("val"), pybind11::arg("nDecimalDigits"), pybind11::arg("middle_space"));

	// mrpt::system::rightPad(const std::string &, size_t, bool) file:mrpt/system/string_utils.h line:115
	M("mrpt::system").def("rightPad", [](const std::string & a0, size_t const & a1) -> std::string { return mrpt::system::rightPad(a0, a1); }, "", pybind11::arg("str"), pybind11::arg("total_len"));
	M("mrpt::system").def("rightPad", (std::string (*)(const std::string &, size_t, bool)) &mrpt::system::rightPad, "Enlarge the string with spaces up to the given length. \n\nC++: mrpt::system::rightPad(const std::string &, size_t, bool) --> std::string", pybind11::arg("str"), pybind11::arg("total_len"), pybind11::arg("truncate_if_larger"));

	// mrpt::system::strCmp(const std::string &, const std::string &) file:mrpt/system/string_utils.h line:118
	M("mrpt::system").def("strCmp", (bool (*)(const std::string &, const std::string &)) &mrpt::system::strCmp, "Return true if the two strings are equal (case sensitive)  \n strCmpI  \n\nC++: mrpt::system::strCmp(const std::string &, const std::string &) --> bool", pybind11::arg("s1"), pybind11::arg("s2"));

	// mrpt::system::strCmpI(const std::string &, const std::string &) file:mrpt/system/string_utils.h line:121
	M("mrpt::system").def("strCmpI", (bool (*)(const std::string &, const std::string &)) &mrpt::system::strCmpI, "Return true if the two strings are equal (case insensitive)  \n strCmp \n\nC++: mrpt::system::strCmpI(const std::string &, const std::string &) --> bool", pybind11::arg("s1"), pybind11::arg("s2"));

	// mrpt::system::strStarts(const std::string &, const std::string &) file:mrpt/system/string_utils.h line:125
	M("mrpt::system").def("strStarts", (bool (*)(const std::string &, const std::string &)) &mrpt::system::strStarts, "Return true if \"str\" starts with \"subStr\" (case sensitive)  \n strStartsI\n\nC++: mrpt::system::strStarts(const std::string &, const std::string &) --> bool", pybind11::arg("str"), pybind11::arg("subStr"));

	// mrpt::system::strStartsI(const std::string &, const std::string &) file:mrpt/system/string_utils.h line:129
	M("mrpt::system").def("strStartsI", (bool (*)(const std::string &, const std::string &)) &mrpt::system::strStartsI, "Return true if \"str\" starts with \"subStr\" (case insensitive)  \n strStarts\n\nC++: mrpt::system::strStartsI(const std::string &, const std::string &) --> bool", pybind11::arg("str"), pybind11::arg("subStr"));

	// mrpt::system::stringListAsString(const class std::vector<std::string > &, std::string &, const std::string &) file:mrpt/system/string_utils.h line:150
	M("mrpt::system").def("stringListAsString", [](const class std::vector<std::string > & a0, std::string & a1) -> void { return mrpt::system::stringListAsString(a0, a1); }, "", pybind11::arg("lst"), pybind11::arg("out"));
	M("mrpt::system").def("stringListAsString", (void (*)(const class std::vector<std::string > &, std::string &, const std::string &)) &mrpt::system::stringListAsString, "Convert a string list to one single string with new-lines. \n\nC++: mrpt::system::stringListAsString(const class std::vector<std::string > &, std::string &, const std::string &) --> void", pybind11::arg("lst"), pybind11::arg("out"), pybind11::arg("newline"));

	// mrpt::system::nthOccurrence(const std::string &, const std::string &, size_t) file:mrpt/system/string_utils.h line:160
	M("mrpt::system").def("nthOccurrence", (size_t (*)(const std::string &, const std::string &, size_t)) &mrpt::system::nthOccurrence, "Finds the position of the n-th occurence of the given substring, or\n std::string::npos if it does not happen.\n \n\n New in MRPT 2.3.2\n\nC++: mrpt::system::nthOccurrence(const std::string &, const std::string &, size_t) --> size_t", pybind11::arg("str"), pybind11::arg("strToFind"), pybind11::arg("nth"));

	// mrpt::system::firstNLines(const std::string &, size_t) file:mrpt/system/string_utils.h line:165
	M("mrpt::system").def("firstNLines", (std::string (*)(const std::string &, size_t)) &mrpt::system::firstNLines, "Returns the first `n` lines (splitted by '' chars) of the given text.\n \n\n New in MRPT 2.3.2\n\nC++: mrpt::system::firstNLines(const std::string &, size_t) --> std::string", pybind11::arg("str"), pybind11::arg("n"));

}

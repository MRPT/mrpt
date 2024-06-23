#include <ios>
#include <istream>
#include <iterator>
#include <memory>
#include <mrpt/io/CTextFileLinesParser.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/io/vector_loadsave.h>
#include <sstream>
#include <sstream> // __str__
#include <streambuf>
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

void bind_mrpt_io_CTextFileLinesParser(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::io::CTextFileLinesParser file:mrpt/io/CTextFileLinesParser.h line:25
		pybind11::class_<mrpt::io::CTextFileLinesParser, std::shared_ptr<mrpt::io::CTextFileLinesParser>> cl(M("mrpt::io"), "CTextFileLinesParser", "A class for parsing text files, returning each non-empty and non-comment\n line, along its line number. Lines are strip out of leading and trailing\n whitespaces. By default, lines starting with either \"#\", \"//\" or \"%\" are\n skipped as comment lines, unless this behavior is explicitly disabled with\n \n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::io::CTextFileLinesParser(); } ) );
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("filename") );

		cl.def( pybind11::init( [](mrpt::io::CTextFileLinesParser const &o){ return new mrpt::io::CTextFileLinesParser(o); } ) );
		cl.def("open", (void (mrpt::io::CTextFileLinesParser::*)(const std::string &)) &mrpt::io::CTextFileLinesParser::open, "Open a file (an alternative to the constructor with a file name) \n\nC++: mrpt::io::CTextFileLinesParser::open(const std::string &) --> void", pybind11::arg("fil"));
		cl.def("close", (void (mrpt::io::CTextFileLinesParser::*)()) &mrpt::io::CTextFileLinesParser::close, "Close the file (no need to call it normally, the file is closed upon\n destruction) \n\nC++: mrpt::io::CTextFileLinesParser::close() --> void");
		cl.def("rewind", (void (mrpt::io::CTextFileLinesParser::*)()) &mrpt::io::CTextFileLinesParser::rewind, "Reset the read pointer to the beginning of the file \n\nC++: mrpt::io::CTextFileLinesParser::rewind() --> void");
		cl.def("getNextLine", (bool (mrpt::io::CTextFileLinesParser::*)(std::string &)) &mrpt::io::CTextFileLinesParser::getNextLine, "Reads from the file and return the next (non-comment) line, as a\n std::string\n \n\n false on EOF.\n\nC++: mrpt::io::CTextFileLinesParser::getNextLine(std::string &) --> bool", pybind11::arg("out_str"));
		cl.def("getCurrentLineNumber", (size_t (mrpt::io::CTextFileLinesParser::*)() const) &mrpt::io::CTextFileLinesParser::getCurrentLineNumber, "Return the line number of the last line returned with  \n\nC++: mrpt::io::CTextFileLinesParser::getCurrentLineNumber() const --> size_t");
		cl.def("enableCommentFilters", (void (mrpt::io::CTextFileLinesParser::*)(bool, bool, bool)) &mrpt::io::CTextFileLinesParser::enableCommentFilters, "Enable/disable filtering of lines starting with \"%\", \"//\" or \"#\",\n respectively. \n\nC++: mrpt::io::CTextFileLinesParser::enableCommentFilters(bool, bool, bool) --> void", pybind11::arg("filter_MATLAB_comments"), pybind11::arg("filter_C_comments"), pybind11::arg("filter_SH_comments"));
		cl.def("assign", (class mrpt::io::CTextFileLinesParser & (mrpt::io::CTextFileLinesParser::*)(const class mrpt::io::CTextFileLinesParser &)) &mrpt::io::CTextFileLinesParser::operator=, "C++: mrpt::io::CTextFileLinesParser::operator=(const class mrpt::io::CTextFileLinesParser &) --> class mrpt::io::CTextFileLinesParser &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::io::lazy_load_absolute_path(const std::string &) file:mrpt/io/lazy_load_path.h line:20
	M("mrpt::io").def("lazy_load_absolute_path", (std::string (*)(const std::string &)) &mrpt::io::lazy_load_absolute_path, "Makes sure of building an absolute path for the given relative (or possibly\n absolute) lazy-load object.\n\n \n\n \n\nC++: mrpt::io::lazy_load_absolute_path(const std::string &) --> std::string", pybind11::arg("relativeOrAbsolutePath"));

	// mrpt::io::getLazyLoadPathBase() file:mrpt/io/lazy_load_path.h line:27
	M("mrpt::io").def("getLazyLoadPathBase", (const std::string & (*)()) &mrpt::io::getLazyLoadPathBase, "Gets the current path to be used to locate relative lazy-load externally\n stored objects via lazy_load_absolute_path(). Default is `\".\"`.\n\n \n\n \n\nC++: mrpt::io::getLazyLoadPathBase() --> const std::string &", pybind11::return_value_policy::automatic);

	// mrpt::io::setLazyLoadPathBase(const std::string &) file:mrpt/io/lazy_load_path.h line:34
	M("mrpt::io").def("setLazyLoadPathBase", (void (*)(const std::string &)) &mrpt::io::setLazyLoadPathBase, "Changes the base path to be used to locate relative lazy-load externally\n stored objects via lazy_load_absolute_path().\n\n \n\n \n\nC++: mrpt::io::setLazyLoadPathBase(const std::string &) --> void", pybind11::arg("path"));

	// mrpt::io::loadTextFile(class std::vector<std::string > &, const std::string &) file:mrpt/io/vector_loadsave.h line:39
	M("mrpt::io").def("loadTextFile", (bool (*)(class std::vector<std::string > &, const std::string &)) &mrpt::io::loadTextFile, "Loads a text file as a vector of string lines.\n \n\n Returns false on any error, true on everything OK.\n \n\n file_get_contents()\n\nC++: mrpt::io::loadTextFile(class std::vector<std::string > &, const std::string &) --> bool", pybind11::arg("o"), pybind11::arg("fileName"));

	// mrpt::io::file_get_contents(const std::string &) file:mrpt/io/vector_loadsave.h line:47
	M("mrpt::io").def("file_get_contents", (std::string (*)(const std::string &)) &mrpt::io::file_get_contents, "Loads an entire text file and return its contents as a single std::string.\n \n\n std::runtime_error On any read error.\n \n\n loadBinaryFile(), loadTextFile()\n \n\n Relying on C++17 RVO to return a string without worring on\n return-by-value of big objects.\n\nC++: mrpt::io::file_get_contents(const std::string &) --> std::string", pybind11::arg("fileName"));

}

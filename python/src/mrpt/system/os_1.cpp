#include <functional>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/os.h>
#include <optional>
#include <sstream>
#include <sstream> // __str__
#include <string>
#include <string_view>

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

void bind_mrpt_system_os_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::system::pause(const std::string &) file:mrpt/system/os.h line:130
	M("mrpt::system").def("pause", []() -> void { return mrpt::system::pause(); }, "");
	M("mrpt::system").def("pause", (void (*)(const std::string &)) &mrpt::system::pause, "Shows the message \"Press any key to continue\" (or other custom message) to\n the current standard output and returns when a key is pressed \n\nC++: mrpt::system::pause(const std::string &) --> void", pybind11::arg("msg"));

	// mrpt::system::clearConsole() file:mrpt/system/os.h line:133
	M("mrpt::system").def("clearConsole", (void (*)()) &mrpt::system::clearConsole, "Clears the console window \n\nC++: mrpt::system::clearConsole() --> void");

	// mrpt::system::MRPT_getCompilationDate() file:mrpt/system/os.h line:137
	M("mrpt::system").def("MRPT_getCompilationDate", (std::string (*)()) &mrpt::system::MRPT_getCompilationDate, "Returns the MRPT source code timestamp, according to the Reproducible-Builds\n specifications: https://reproducible-builds.org/specs/source-date-epoch/  \n\nC++: mrpt::system::MRPT_getCompilationDate() --> std::string");

	// mrpt::system::MRPT_getVersion() file:mrpt/system/os.h line:140
	M("mrpt::system").def("MRPT_getVersion", (std::string (*)()) &mrpt::system::MRPT_getVersion, "Returns a string describing the MRPT version \n\nC++: mrpt::system::MRPT_getVersion() --> std::string");

	// mrpt::system::getMRPTLicense() file:mrpt/system/os.h line:145
	M("mrpt::system").def("getMRPTLicense", (const std::string & (*)()) &mrpt::system::getMRPTLicense, "Returns a const ref to a text with the same text that appears at the\n beginning of each MRPT file (useful for displaying the License text in GUIs)\n\nC++: mrpt::system::getMRPTLicense() --> const std::string &", pybind11::return_value_policy::automatic);

	// mrpt::system::find_mrpt_shared_dir() file:mrpt/system/os.h line:149
	M("mrpt::system").def("find_mrpt_shared_dir", (std::string (*)()) &mrpt::system::find_mrpt_shared_dir, "Finds the \"[MRPT]/share/mrpt/\" directory, if available in the system. This\n searches in (1) source code tree, (2) install target paths. \n\nC++: mrpt::system::find_mrpt_shared_dir() --> std::string");

	// mrpt::system::ConsoleForegroundColor file:mrpt/system/os.h line:154
	pybind11::enum_<mrpt::system::ConsoleForegroundColor>(M("mrpt::system"), "ConsoleForegroundColor", "For use in consoleColorAndStyle().\n  \n\n Numerical values from vt100-console escape codes.")
		.value("DEFAULT", mrpt::system::ConsoleForegroundColor::DEFAULT)
		.value("BLACK", mrpt::system::ConsoleForegroundColor::BLACK)
		.value("RED", mrpt::system::ConsoleForegroundColor::RED)
		.value("GREEN", mrpt::system::ConsoleForegroundColor::GREEN)
		.value("YELLOW", mrpt::system::ConsoleForegroundColor::YELLOW)
		.value("BLUE", mrpt::system::ConsoleForegroundColor::BLUE)
		.value("MAGENTA", mrpt::system::ConsoleForegroundColor::MAGENTA)
		.value("CYAN", mrpt::system::ConsoleForegroundColor::CYAN)
		.value("WHITE", mrpt::system::ConsoleForegroundColor::WHITE)
		.value("BRIGHT_BLACK", mrpt::system::ConsoleForegroundColor::BRIGHT_BLACK)
		.value("BRIGHT_RED", mrpt::system::ConsoleForegroundColor::BRIGHT_RED)
		.value("BRIGHT_GREEN", mrpt::system::ConsoleForegroundColor::BRIGHT_GREEN)
		.value("BRIGHT_YELLOW", mrpt::system::ConsoleForegroundColor::BRIGHT_YELLOW)
		.value("BRIGHT_BLUE", mrpt::system::ConsoleForegroundColor::BRIGHT_BLUE)
		.value("BRIGHT_MAGENTA", mrpt::system::ConsoleForegroundColor::BRIGHT_MAGENTA)
		.value("BRIGHT_CYAN", mrpt::system::ConsoleForegroundColor::BRIGHT_CYAN)
		.value("BRIGHT_WHITE", mrpt::system::ConsoleForegroundColor::BRIGHT_WHITE);

;

	// mrpt::system::ConsoleBackgroundColor file:mrpt/system/os.h line:178
	pybind11::enum_<mrpt::system::ConsoleBackgroundColor>(M("mrpt::system"), "ConsoleBackgroundColor", "For use in consoleColorAndStyle().\n  \n\n Numerical values from vt100-console escape codes.")
		.value("DEFAULT", mrpt::system::ConsoleBackgroundColor::DEFAULT)
		.value("BLACK", mrpt::system::ConsoleBackgroundColor::BLACK)
		.value("RED", mrpt::system::ConsoleBackgroundColor::RED)
		.value("GREEN", mrpt::system::ConsoleBackgroundColor::GREEN)
		.value("YELLOW", mrpt::system::ConsoleBackgroundColor::YELLOW)
		.value("BLUE", mrpt::system::ConsoleBackgroundColor::BLUE)
		.value("MAGENTA", mrpt::system::ConsoleBackgroundColor::MAGENTA)
		.value("CYAN", mrpt::system::ConsoleBackgroundColor::CYAN)
		.value("WHITE", mrpt::system::ConsoleBackgroundColor::WHITE)
		.value("BRIGHT_BLACK", mrpt::system::ConsoleBackgroundColor::BRIGHT_BLACK)
		.value("BRIGHT_RED", mrpt::system::ConsoleBackgroundColor::BRIGHT_RED)
		.value("BRIGHT_GREEN", mrpt::system::ConsoleBackgroundColor::BRIGHT_GREEN)
		.value("BRIGHT_YELLOW", mrpt::system::ConsoleBackgroundColor::BRIGHT_YELLOW)
		.value("BRIGHT_BLUE", mrpt::system::ConsoleBackgroundColor::BRIGHT_BLUE)
		.value("BRIGHT_MAGENTA", mrpt::system::ConsoleBackgroundColor::BRIGHT_MAGENTA)
		.value("BRIGHT_CYAN", mrpt::system::ConsoleBackgroundColor::BRIGHT_CYAN)
		.value("BRIGHT_WHITE", mrpt::system::ConsoleBackgroundColor::BRIGHT_WHITE);

;

	// mrpt::system::ConsoleTextStyle file:mrpt/system/os.h line:202
	pybind11::enum_<mrpt::system::ConsoleTextStyle>(M("mrpt::system"), "ConsoleTextStyle", "For use in consoleColorAndStyle().\n  \n\n Numerical values from vt100-console escape codes.")
		.value("REGULAR", mrpt::system::ConsoleTextStyle::REGULAR)
		.value("BOLD", mrpt::system::ConsoleTextStyle::BOLD)
		.value("DIM", mrpt::system::ConsoleTextStyle::DIM)
		.value("ITALIC", mrpt::system::ConsoleTextStyle::ITALIC)
		.value("UNDERLINED", mrpt::system::ConsoleTextStyle::UNDERLINED)
		.value("BLINKING", mrpt::system::ConsoleTextStyle::BLINKING)
		.value("REVERSE", mrpt::system::ConsoleTextStyle::REVERSE)
		.value("INVISIBLE", mrpt::system::ConsoleTextStyle::INVISIBLE);

;

	// mrpt::system::consoleColorAndStyle(enum mrpt::system::ConsoleForegroundColor, enum mrpt::system::ConsoleBackgroundColor, enum mrpt::system::ConsoleTextStyle, bool) file:mrpt/system/os.h line:230
	M("mrpt::system").def("consoleColorAndStyle", [](enum mrpt::system::ConsoleForegroundColor const & a0) -> void { return mrpt::system::consoleColorAndStyle(a0); }, "", pybind11::arg("fg"));
	M("mrpt::system").def("consoleColorAndStyle", [](enum mrpt::system::ConsoleForegroundColor const & a0, enum mrpt::system::ConsoleBackgroundColor const & a1) -> void { return mrpt::system::consoleColorAndStyle(a0, a1); }, "", pybind11::arg("fg"), pybind11::arg("bg"));
	M("mrpt::system").def("consoleColorAndStyle", [](enum mrpt::system::ConsoleForegroundColor const & a0, enum mrpt::system::ConsoleBackgroundColor const & a1, enum mrpt::system::ConsoleTextStyle const & a2) -> void { return mrpt::system::consoleColorAndStyle(a0, a1, a2); }, "", pybind11::arg("fg"), pybind11::arg("bg"), pybind11::arg("style"));
	M("mrpt::system").def("consoleColorAndStyle", (void (*)(enum mrpt::system::ConsoleForegroundColor, enum mrpt::system::ConsoleBackgroundColor, enum mrpt::system::ConsoleTextStyle, bool)) &mrpt::system::consoleColorAndStyle, "Changes the text color and style in the console for the text written from\n now on. See available colors in ConsoleForegroundColor and\n ConsoleBackgroundColor.\n\n By default the color of \"cout\" is changed, unless changeStdErr=true, in\n which case \"cerr\" is changed.\n\n \n GNU/Linux: If stdout/stderr is not a real terminal with color support,\n calling this function will have no effect (i.e. no escape characters will be\n emitted).\n\n \n The current implementation only supports a subset of all colors for\n Windows terminals.\n\n \n (New in MRPT 2.3.3)\n\nC++: mrpt::system::consoleColorAndStyle(enum mrpt::system::ConsoleForegroundColor, enum mrpt::system::ConsoleBackgroundColor, enum mrpt::system::ConsoleTextStyle, bool) --> void", pybind11::arg("fg"), pybind11::arg("bg"), pybind11::arg("style"), pybind11::arg("applyToStdErr"));

	// mrpt::system::executeCommand(const std::string &, std::string *, const std::string &) file:mrpt/system/os.h line:246
	M("mrpt::system").def("executeCommand", [](const std::string & a0) -> int { return mrpt::system::executeCommand(a0); }, "", pybind11::arg("command"));
	M("mrpt::system").def("executeCommand", [](const std::string & a0, std::string * a1) -> int { return mrpt::system::executeCommand(a0, a1); }, "", pybind11::arg("command"), pybind11::arg("output"));
	M("mrpt::system").def("executeCommand", (int (*)(const std::string &, std::string *, const std::string &)) &mrpt::system::executeCommand, "Execute Generic Shell Command\n\n \n Command to execute\n \n\n  Pointer to string containing the shell output\n \n\n read/write access\n\n \n 0 for success, -1 otherwise.\n\n \n Original code snippet found in http://stackoverflow.com/a/30357710\n\nC++: mrpt::system::executeCommand(const std::string &, std::string *, const std::string &) --> int", pybind11::arg("command"), pybind11::arg("output"), pybind11::arg("mode"));

	// mrpt::system::launchProcess(const std::string &) file:mrpt/system/os.h line:255
	M("mrpt::system").def("launchProcess", (bool (*)(const std::string &)) &mrpt::system::launchProcess, "Executes the given command (which may contain a program + arguments), and\nwaits until it finishes.\n\n \n false on any error, true otherwise\n\nC++: mrpt::system::launchProcess(const std::string &) --> bool", pybind11::arg("command"));

	// mrpt::system::VerbosityLevel file:mrpt/system/COutputLogger.h line:32
	pybind11::enum_<mrpt::system::VerbosityLevel>(M("mrpt::system"), "VerbosityLevel", pybind11::arithmetic(), "Enumeration of available verbosity levels. \n COutputLogger ")
		.value("LVL_DEBUG", mrpt::system::LVL_DEBUG)
		.value("LVL_INFO", mrpt::system::LVL_INFO)
		.value("LVL_WARN", mrpt::system::LVL_WARN)
		.value("LVL_ERROR", mrpt::system::LVL_ERROR)
		.value("NUMBER_OF_VERBOSITY_LEVELS", mrpt::system::NUMBER_OF_VERBOSITY_LEVELS)
		.export_values();

;

	{ // mrpt::system::COutputLoggerStreamWrapper file:mrpt/system/COutputLogger.h line:345
		pybind11::class_<mrpt::system::COutputLoggerStreamWrapper, std::shared_ptr<mrpt::system::COutputLoggerStreamWrapper>> cl(M("mrpt::system"), "COutputLoggerStreamWrapper", "For use in MRPT_LOG_DEBUG_STREAM(), etc. ");
	}
}

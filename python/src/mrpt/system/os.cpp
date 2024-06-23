#include <mrpt/system/os.h>

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

void bind_mrpt_system_os(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::system::os::strcat(char *, size_t, const char *) file:mrpt/system/os.h line:72
	M("mrpt::system::os").def("strcat", (char * (*)(char *, size_t, const char *)) &mrpt::system::os::strcat, "An OS-independent version of strcat.\n \n\n It will always return the \"dest\" pointer.\n\nC++: mrpt::system::os::strcat(char *, size_t, const char *) --> char *", pybind11::return_value_policy::automatic, pybind11::arg("dest"), pybind11::arg("destSize"), pybind11::arg("source"));

	// mrpt::system::os::strcpy(char *, size_t, const char *) file:mrpt/system/os.h line:77
	M("mrpt::system::os").def("strcpy", (char * (*)(char *, size_t, const char *)) &mrpt::system::os::strcpy, "An OS-independent version of strcpy.\n \n\n It will always return the \"dest\" pointer.\n\nC++: mrpt::system::os::strcpy(char *, size_t, const char *) --> char *", pybind11::return_value_policy::automatic, pybind11::arg("dest"), pybind11::arg("destSize"), pybind11::arg("source"));

	// mrpt::system::os::_strcmp(const char *, const char *) file:mrpt/system/os.h line:82
	M("mrpt::system::os").def("_strcmp", (int (*)(const char *, const char *)) &mrpt::system::os::_strcmp, "An OS-independent version of strcmp.\n \n\n It will return 0 when both strings are equal, casi sensitive.\n\nC++: mrpt::system::os::_strcmp(const char *, const char *) --> int", pybind11::arg("str1"), pybind11::arg("str2"));

	// mrpt::system::os::_strcmpi(const char *, const char *) file:mrpt/system/os.h line:87
	M("mrpt::system::os").def("_strcmpi", (int (*)(const char *, const char *)) &mrpt::system::os::_strcmpi, "An OS-independent version of strcmpi.\n \n\n It will return 0 when both strings are equal, casi insensitive.\n\nC++: mrpt::system::os::_strcmpi(const char *, const char *) --> int", pybind11::arg("str1"), pybind11::arg("str2"));

	// mrpt::system::os::_strncmp(const char *, const char *, size_t) file:mrpt/system/os.h line:92
	M("mrpt::system::os").def("_strncmp", (int (*)(const char *, const char *, size_t)) &mrpt::system::os::_strncmp, "An OS-independent version of strncmp.\n \n\n It will return 0 when both strings are equal, casi sensitive.\n\nC++: mrpt::system::os::_strncmp(const char *, const char *, size_t) --> int", pybind11::arg("str"), pybind11::arg("subStr"), pybind11::arg("count"));

	// mrpt::system::os::_strnicmp(const char *, const char *, size_t) file:mrpt/system/os.h line:97
	M("mrpt::system::os").def("_strnicmp", (int (*)(const char *, const char *, size_t)) &mrpt::system::os::_strnicmp, "An OS-independent version of strnicmp.\n \n\n It will return 0 when both strings are equal, casi insensitive.\n\nC++: mrpt::system::os::_strnicmp(const char *, const char *, size_t) --> int", pybind11::arg("str"), pybind11::arg("subStr"), pybind11::arg("count"));

	// mrpt::system::os::memcpy(void *, size_t, const void *, size_t) file:mrpt/system/os.h line:109
	M("mrpt::system::os").def("memcpy", (void (*)(void *, size_t, const void *, size_t)) &mrpt::system::os::memcpy, "An OS and compiler independent version of \"memcpy\"\n\nC++: mrpt::system::os::memcpy(void *, size_t, const void *, size_t) --> void", pybind11::arg("dest"), pybind11::arg("destSize"), pybind11::arg("src"), pybind11::arg("copyCount"));

	// mrpt::system::os::getch() file:mrpt/system/os.h line:114
	M("mrpt::system::os").def("getch", (int (*)()) &mrpt::system::os::getch, "An OS-independent version of getch, which waits until a key is pushed.\n \n\n The pushed key code\n\nC++: mrpt::system::os::getch() --> int");

	// mrpt::system::os::kbhit() file:mrpt/system/os.h line:119
	M("mrpt::system::os").def("kbhit", (bool (*)()) &mrpt::system::os::kbhit, "An OS-independent version of kbhit, which returns true if a key has been\n pushed.\n\nC++: mrpt::system::os::kbhit() --> bool");

}

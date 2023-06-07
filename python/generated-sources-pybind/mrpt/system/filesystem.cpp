#include <iterator>
#include <memory>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/md5.h>
#include <mrpt/system/memory.h>
#include <mrpt/system/progress.h>
#include <mrpt/system/scheduler.h>
#include <mrpt/system/thread_name.h>
#include <string>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_system_filesystem(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::system::getTempFileName() file:mrpt/system/filesystem.h line:35
	M("mrpt::system").def("getTempFileName", (std::string (*)()) &mrpt::system::getTempFileName, "Returns the name of a proposed temporary file name \n\nC++: mrpt::system::getTempFileName() --> std::string");

	// mrpt::system::getcwd() file:mrpt/system/filesystem.h line:38
	M("mrpt::system").def("getcwd", (std::string (*)()) &mrpt::system::getcwd, "Returns the current working directory  \n\nC++: mrpt::system::getcwd() --> std::string");

	// mrpt::system::getShareMRPTDir() file:mrpt/system/filesystem.h line:44
	M("mrpt::system").def("getShareMRPTDir", (std::string (*)()) &mrpt::system::getShareMRPTDir, "Attempts to find the directory `[PREFIX/]share/mrpt/` and returns its\n absolute path, or empty string if not found.\n Example return paths: Linux after installing = `/usr/share/mrpt/`;\n manually-built system = `[MRPT_SOURCE_DIR]/share/mrpt/`, etc.  \n\nC++: mrpt::system::getShareMRPTDir() --> std::string");

	// mrpt::system::createDirectory(const std::string &) file:mrpt/system/filesystem.h line:50
	M("mrpt::system").def("createDirectory", (bool (*)(const std::string &)) &mrpt::system::createDirectory, "Creates a directory\n \n\n Returns false on any error, true on directory created or already\n existed.\n\nC++: mrpt::system::createDirectory(const std::string &) --> bool", pybind11::arg("dirName"));

	// mrpt::system::deleteFile(const std::string &) file:mrpt/system/filesystem.h line:56
	M("mrpt::system").def("deleteFile", (bool (*)(const std::string &)) &mrpt::system::deleteFile, "Deletes a single file. For multiple files see deleteFiles\n \n\n Returns false on any error, true on everything OK.\n \n\n deleteFiles\n\nC++: mrpt::system::deleteFile(const std::string &) --> bool", pybind11::arg("fileName"));

	// mrpt::system::deleteFiles(const std::string &) file:mrpt/system/filesystem.h line:64
	M("mrpt::system").def("deleteFiles", (void (*)(const std::string &)) &mrpt::system::deleteFiles, "Delete one or more files, especified by the (optional) path and the file\n name (including '?' or '*') - Use forward slash ('/') for directories for\n compatibility between Windows and Linux, since they will be internally\n traslated into backward slashes ('\\') if MRPT is compiled under Windows.\n \n\n deleteFile\n\nC++: mrpt::system::deleteFiles(const std::string &) --> void", pybind11::arg("s"));

	// mrpt::system::renameFile(const std::string &, const std::string &, std::string *) file:mrpt/system/filesystem.h line:71
	M("mrpt::system").def("renameFile", [](const std::string & a0, const std::string & a1) -> bool { return mrpt::system::renameFile(a0, a1); }, "", pybind11::arg("oldFileName"), pybind11::arg("newFileName"));
	M("mrpt::system").def("renameFile", (bool (*)(const std::string &, const std::string &, std::string *)) &mrpt::system::renameFile, "Renames a file - If the target path is different and the filesystem allows\n it, it will be moved to the new location.\n \n\n false on any error. In that case, if a pointer to a receiver string\n is passed in error_msg, a description of the error is saved there.\n\nC++: mrpt::system::renameFile(const std::string &, const std::string &, std::string *) --> bool", pybind11::arg("oldFileName"), pybind11::arg("newFileName"), pybind11::arg("error_msg"));

	// mrpt::system::deleteFilesInDirectory(const std::string &, bool) file:mrpt/system/filesystem.h line:80
	M("mrpt::system").def("deleteFilesInDirectory", [](const std::string & a0) -> bool { return mrpt::system::deleteFilesInDirectory(a0); }, "", pybind11::arg("s"));
	M("mrpt::system").def("deleteFilesInDirectory", (bool (*)(const std::string &, bool)) &mrpt::system::deleteFilesInDirectory, "Delete all the files in a given directory (nothing done if directory does\n not exists, or path is a file).\n \n\n deleteFile\n \n\n true on success\n\nC++: mrpt::system::deleteFilesInDirectory(const std::string &, bool) --> bool", pybind11::arg("s"), pybind11::arg("deleteDirectoryAsWell"));

	// mrpt::system::extractFileName(const std::string &) file:mrpt/system/filesystem.h line:88
	M("mrpt::system").def("extractFileName", (std::string (*)(const std::string &)) &mrpt::system::extractFileName, "Extract just the name (without extension) of a filename from a complete path\n plus name plus extension.\n  This function works for either \"/\" or \"\" directory separators.\n \n\n extractFileExtension,extractFileDirectory\n\nC++: mrpt::system::extractFileName(const std::string &) --> std::string", pybind11::arg("filePath"));

	// mrpt::system::extractFileExtension(const std::string &, bool) file:mrpt/system/filesystem.h line:97
	M("mrpt::system").def("extractFileExtension", [](const std::string & a0) -> std::string { return mrpt::system::extractFileExtension(a0); }, "", pybind11::arg("filePath"));
	M("mrpt::system").def("extractFileExtension", (std::string (*)(const std::string &, bool)) &mrpt::system::extractFileExtension, "Extract the extension of a filename.\n  For example, for \"dummy.cpp\", it will return \"cpp\".\n  If \"ignore_gz\" is true, the second extension will be returned if the file\n name\n   ends in \".gz\", for example, for \"foo.map.gz\", this will return \"map\".\n \n\n extractFileName,extractFileDirectory\n\nC++: mrpt::system::extractFileExtension(const std::string &, bool) --> std::string", pybind11::arg("filePath"), pybind11::arg("ignore_gz"));

	// mrpt::system::extractFileDirectory(const std::string &) file:mrpt/system/filesystem.h line:105
	M("mrpt::system").def("extractFileDirectory", (std::string (*)(const std::string &)) &mrpt::system::extractFileDirectory, "Extract the whole path (the directory) of a filename from a complete path\n plus name plus extension.\n  This function works for either \"/\" or \"\" directory separators.\n \n\n extractFileName,extractFileExtension\n\nC++: mrpt::system::extractFileDirectory(const std::string &) --> std::string", pybind11::arg("filePath"));

	// mrpt::system::fileExists(const std::string &) file:mrpt/system/filesystem.h line:110
	M("mrpt::system").def("fileExists", (bool (*)(const std::string &)) &mrpt::system::fileExists, "Test if a given file (or directory) exists.\n \n\n directoryExists\n\nC++: mrpt::system::fileExists(const std::string &) --> bool", pybind11::arg("fileName"));

	// mrpt::system::directoryExists(const std::string &) file:mrpt/system/filesystem.h line:116
	M("mrpt::system").def("directoryExists", (bool (*)(const std::string &)) &mrpt::system::directoryExists, "Test if a given directory exists (it fails if the given path refers to an\n existing file).\n \n\n fileExists\n\nC++: mrpt::system::directoryExists(const std::string &) --> bool", pybind11::arg("fileName"));

	// mrpt::system::fileNameStripInvalidChars(const std::string &, const char) file:mrpt/system/filesystem.h line:122
	M("mrpt::system").def("fileNameStripInvalidChars", [](const std::string & a0) -> std::string { return mrpt::system::fileNameStripInvalidChars(a0); }, "", pybind11::arg("filename"));
	M("mrpt::system").def("fileNameStripInvalidChars", (std::string (*)(const std::string &, const char)) &mrpt::system::fileNameStripInvalidChars, "Replace invalid filename chars by underscores ('_') or any other user-given\n char.\n  Invalid chars are: '<','>',':','\"','/','\\','|','?','*'\n\nC++: mrpt::system::fileNameStripInvalidChars(const std::string &, const char) --> std::string", pybind11::arg("filename"), pybind11::arg("replacement_to_invalid_chars"));

	// mrpt::system::fileNameChangeExtension(const std::string &, const std::string &) file:mrpt/system/filesystem.h line:131
	M("mrpt::system").def("fileNameChangeExtension", (std::string (*)(const std::string &, const std::string &)) &mrpt::system::fileNameChangeExtension, "Replace the filename extension by another one.\n  Example:\n   \n\n\n\n \n\nC++: mrpt::system::fileNameChangeExtension(const std::string &, const std::string &) --> std::string", pybind11::arg("filename"), pybind11::arg("newExtension"));

	// mrpt::system::getFileSize(const std::string &) file:mrpt/system/filesystem.h line:136
	M("mrpt::system").def("getFileSize", (uint64_t (*)(const std::string &)) &mrpt::system::getFileSize, "Return the size of the given file, or size_t(-1) if some error is found\n accessing that file.  \n\nC++: mrpt::system::getFileSize(const std::string &) --> uint64_t", pybind11::arg("fileName"));

	// mrpt::system::getFileModificationTime(const std::string &) file:mrpt/system/filesystem.h line:140
	M("mrpt::system").def("getFileModificationTime", (long (*)(const std::string &)) &mrpt::system::getFileModificationTime, "Return the time of the file last modification, or \"0\" if the file doesn't\n exist.  \n\nC++: mrpt::system::getFileModificationTime(const std::string &) --> long", pybind11::arg("filename"));

	// mrpt::system::filePathSeparatorsToNative(const std::string &) file:mrpt/system/filesystem.h line:143
	M("mrpt::system").def("filePathSeparatorsToNative", (std::string (*)(const std::string &)) &mrpt::system::filePathSeparatorsToNative, "Windows: replace all '/'->'\\' , in Linux/MacOS: replace all '\\'->'/' \n\nC++: mrpt::system::filePathSeparatorsToNative(const std::string &) --> std::string", pybind11::arg("filePath"));

	// mrpt::system::copyFile(const std::string &, const std::string &, std::string *) file:mrpt/system/filesystem.h line:153
	M("mrpt::system").def("copyFile", [](const std::string & a0, const std::string & a1) -> bool { return mrpt::system::copyFile(a0, a1); }, "", pybind11::arg("sourceFile"), pybind11::arg("targetFile"));
	M("mrpt::system").def("copyFile", (bool (*)(const std::string &, const std::string &, std::string *)) &mrpt::system::copyFile, "Copies file  to  If the target file exists, it\n will be overwritten.\n\n \n true on success, false on any error, whose description can be\n optionally get in outErrStr\n\n \n (In MRPT 2.5.0, the copyAttributes param was removed)\n\nC++: mrpt::system::copyFile(const std::string &, const std::string &, std::string *) --> bool", pybind11::arg("sourceFile"), pybind11::arg("targetFile"), pybind11::arg("outErrStr"));

	// mrpt::system::toAbsolutePath(const std::string &, bool) file:mrpt/system/filesystem.h line:171
	M("mrpt::system").def("toAbsolutePath", [](const std::string & a0) -> std::string { return mrpt::system::toAbsolutePath(a0); }, "", pybind11::arg("path"));
	M("mrpt::system").def("toAbsolutePath", (std::string (*)(const std::string &, bool)) &mrpt::system::toAbsolutePath, "Portable version of std::filesystem::absolute() and canonical()\n\n If `canonical==true` relative paths, symlinks, etc. will be resolved too,\n but an exception will be thrown if the referenced file/path does not exist.\n If `canonical==true`, an absolute path will be always returned, even if does\n not actually exist.\n\n  \n\n\n\n  \n (New in MRPT 2.5.0)\n\nC++: mrpt::system::toAbsolutePath(const std::string &, bool) --> std::string", pybind11::arg("path"), pybind11::arg("resolveToCanonical"));

	// mrpt::system::pathJoin(const class std::vector<std::string, class std::allocator<std::string > > &) file:mrpt/system/filesystem.h line:184
	M("mrpt::system").def("pathJoin", (std::string (*)(const class std::vector<std::string, class std::allocator<std::string > > &)) &mrpt::system::pathJoin, "Portable version of std::filesystem::path::append(), with Python-like name.\n\n  \n\n\n\n\n  \n (New in MRPT 2.5.0)\n\nC++: mrpt::system::pathJoin(const class std::vector<std::string, class std::allocator<std::string > > &) --> std::string", pybind11::arg("paths"));

	// mrpt::system::md5(const std::string &) file:mrpt/system/md5.h line:23
	M("mrpt::system").def("md5", (std::string (*)(const std::string &)) &mrpt::system::md5, "Header: `#include <mrpt/system/md5.h>`.\n Library: \n  \n\n\n @{ \n\n Computes the md5 of a block of data. \n\nC++: mrpt::system::md5(const std::string &) --> std::string", pybind11::arg("str"));

	// mrpt::system::md5(const unsigned char *, size_t) file:mrpt/system/md5.h line:27
	M("mrpt::system").def("md5", (std::string (*)(const unsigned char *, size_t)) &mrpt::system::md5, "Computes the md5 of a block of data. \n\nC++: mrpt::system::md5(const unsigned char *, size_t) --> std::string", pybind11::arg("data"), pybind11::arg("len"));

	// mrpt::system::getMemoryUsage() file:mrpt/system/memory.h line:25
	M("mrpt::system").def("getMemoryUsage", (unsigned long (*)()) &mrpt::system::getMemoryUsage, "Returns the memory occupied by this process, in bytes \n\nC++: mrpt::system::getMemoryUsage() --> unsigned long");

	// mrpt::system::progress(const double, const unsigned long, bool) file:mrpt/system/progress.h line:29
	M("mrpt::system").def("progress", [](const double & a0, const unsigned long & a1) -> std::string { return mrpt::system::progress(a0, a1); }, "", pybind11::arg("progressRatio0to1"), pybind11::arg("barLength"));
	M("mrpt::system").def("progress", (std::string (*)(const double, const unsigned long, bool)) &mrpt::system::progress, "Build an text incremental progress bar with\n [UNICODE block elements](https://en.wikipedia.org/wiki/Block_Elements),\n for example:\n \n\n\n\n See example: \n  system_progress_bar/test.cpp example-system-progress-bar\n\n \n\n \n (New in MRPT 2.3.0)\n\nC++: mrpt::system::progress(const double, const unsigned long, bool) --> std::string", pybind11::arg("progressRatio0to1"), pybind11::arg("barLength"), pybind11::arg("encloseInSquareBrackets"));

	// mrpt::system::TProcessPriority file:mrpt/system/scheduler.h line:22
	pybind11::enum_<mrpt::system::TProcessPriority>(M("mrpt::system"), "TProcessPriority", pybind11::arithmetic(), "The type for cross-platform process (application) priorities.\n \n\n changeCurrentProcessPriority")
		.value("ppIdle", mrpt::system::ppIdle)
		.value("ppNormal", mrpt::system::ppNormal)
		.value("ppHigh", mrpt::system::ppHigh)
		.value("ppVeryHigh", mrpt::system::ppVeryHigh)
		.export_values();

;

	// mrpt::system::TThreadPriority file:mrpt/system/scheduler.h line:33
	pybind11::enum_<mrpt::system::TThreadPriority>(M("mrpt::system"), "TThreadPriority", pybind11::arithmetic(), "The type for cross-platform thread priorities.\n \n\n changeThreadPriority")
		.value("tpLowests", mrpt::system::tpLowests)
		.value("tpLower", mrpt::system::tpLower)
		.value("tpLow", mrpt::system::tpLow)
		.value("tpNormal", mrpt::system::tpNormal)
		.value("tpHigh", mrpt::system::tpHigh)
		.value("tpHigher", mrpt::system::tpHigher)
		.value("tpHighest", mrpt::system::tpHighest)
		.export_values();

;

	// mrpt::system::changeCurrentThreadPriority(enum mrpt::system::TThreadPriority) file:mrpt/system/scheduler.h line:54
	M("mrpt::system").def("changeCurrentThreadPriority", (void (*)(enum mrpt::system::TThreadPriority)) &mrpt::system::changeCurrentThreadPriority, "Change the priority of the current thread - for Windows, see also\n changeCurrentProcessPriority()\n - Windows: This is equivalent to\n [SetThreadPriority()](https://msdn.microsoft.com/en-us/library/windows/desktop/ms686277(v=vs.85).aspx)\n (read the docs there)\n - Linux (pthreads): May require `root` permissions! This sets the Round Robin\n scheduler with the given priority level. Read\n [sched_setscheduler](http://linux.die.net/man/2/sched_setscheduler). \n\n\n createThread, changeCurrentProcessPriority, changeCurrentThreadPriority\n\nC++: mrpt::system::changeCurrentThreadPriority(enum mrpt::system::TThreadPriority) --> void", pybind11::arg("priority"));

	// mrpt::system::changeCurrentProcessPriority(enum mrpt::system::TProcessPriority) file:mrpt/system/scheduler.h line:66
	M("mrpt::system").def("changeCurrentProcessPriority", (void (*)(enum mrpt::system::TProcessPriority)) &mrpt::system::changeCurrentProcessPriority, "Change the priority of the given process (it applies to all the threads,\n  plus independent modifiers for each thread).\n  - Windows: See\n  [SetPriorityClass](https://msdn.microsoft.com/es-es/library/windows/desktop/ms686219(v=vs.85).aspx)\n  - Linux (pthreads): Requires `root` permissions to increase process\n  priority! Internally it calls [nice()](http://linux.die.net/man/3/nice), so it\n  has no effect if\n  () was called and a SCHED_RR is already active.\n \n\n createThread, changeThreadPriority\n\nC++: mrpt::system::changeCurrentProcessPriority(enum mrpt::system::TProcessPriority) --> void", pybind11::arg("priority"));

	// mrpt::system::thread_name(const std::string &, class std::thread &) file:mrpt/system/thread_name.h line:20
	M("mrpt::system").def("thread_name", (void (*)(const std::string &, class std::thread &)) &mrpt::system::thread_name, "Sets the name of the given thread; useful for debuggers.\n \n\n\n \n New in MRPT 2.0.4\n\nC++: mrpt::system::thread_name(const std::string &, class std::thread &) --> void", pybind11::arg("name"), pybind11::arg("theThread"));

}

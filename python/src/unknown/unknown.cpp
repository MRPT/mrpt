#include <iterator>
#include <memory>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CStream.h>
#include <mrpt/io/open_flags.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
#include <optional>
#include <string>

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

void bind_unknown_unknown(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::serialization::archiveFrom(class mrpt::io::CFileGZInputStream &) file: line:15
	M("mrpt::serialization").def("archiveFrom", (class mrpt::serialization::CArchiveStreamBase<class mrpt::io::CFileGZInputStream> (*)(class mrpt::io::CFileGZInputStream &)) &mrpt::serialization::archiveFrom, "C++: mrpt::serialization::archiveFrom(class mrpt::io::CFileGZInputStream &) --> class mrpt::serialization::CArchiveStreamBase<class mrpt::io::CFileGZInputStream>", pybind11::arg("s"));

	// mrpt::serialization::archiveFrom(class mrpt::io::CFileGZOutputStream &) file: line:20
	M("mrpt::serialization").def("archiveFrom", (class mrpt::serialization::CArchiveStreamBase<class mrpt::io::CFileGZOutputStream> (*)(class mrpt::io::CFileGZOutputStream &)) &mrpt::serialization::archiveFrom, "C++: mrpt::serialization::archiveFrom(class mrpt::io::CFileGZOutputStream &) --> class mrpt::serialization::CArchiveStreamBase<class mrpt::io::CFileGZOutputStream>", pybind11::arg("s"));

	// mrpt::serialization::archiveFrom(class mrpt::io::CFileInputStream &) file: line:25
	M("mrpt::serialization").def("archiveFrom", (class mrpt::serialization::CArchiveStreamBase<class mrpt::io::CFileInputStream> (*)(class mrpt::io::CFileInputStream &)) &mrpt::serialization::archiveFrom, "C++: mrpt::serialization::archiveFrom(class mrpt::io::CFileInputStream &) --> class mrpt::serialization::CArchiveStreamBase<class mrpt::io::CFileInputStream>", pybind11::arg("s"));

	// mrpt::serialization::archiveFrom(class mrpt::io::CFileOutputStream &) file: line:30
	M("mrpt::serialization").def("archiveFrom", (class mrpt::serialization::CArchiveStreamBase<class mrpt::io::CFileOutputStream> (*)(class mrpt::io::CFileOutputStream &)) &mrpt::serialization::archiveFrom, "C++: mrpt::serialization::archiveFrom(class mrpt::io::CFileOutputStream &) --> class mrpt::serialization::CArchiveStreamBase<class mrpt::io::CFileOutputStream>", pybind11::arg("s"));

}

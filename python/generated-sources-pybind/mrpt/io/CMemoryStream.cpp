#include <iterator>
#include <memory>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/io/CStream.h>
#include <sstream> // __str__
#include <string>

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

// mrpt::io::CMemoryStream file:mrpt/io/CMemoryStream.h line:26
struct PyCallBack_mrpt_io_CMemoryStream : public mrpt::io::CMemoryStream {
	using mrpt::io::CMemoryStream::CMemoryStream;

	unsigned long Read(void * a0, unsigned long a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CMemoryStream *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned long>::value) {
				static pybind11::detail::override_caster_t<unsigned long> caster;
				return pybind11::detail::cast_ref<unsigned long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned long>(std::move(o));
		}
		return CMemoryStream::Read(a0, a1);
	}
	unsigned long Write(const void * a0, unsigned long a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CMemoryStream *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned long>::value) {
				static pybind11::detail::override_caster_t<unsigned long> caster;
				return pybind11::detail::cast_ref<unsigned long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned long>(std::move(o));
		}
		return CMemoryStream::Write(a0, a1);
	}
	std::string getStreamDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CMemoryStream *>(this), "getStreamDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CMemoryStream::getStreamDescription();
	}
	unsigned long Seek(long a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CMemoryStream *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned long>::value) {
				static pybind11::detail::override_caster_t<unsigned long> caster;
				return pybind11::detail::cast_ref<unsigned long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned long>(std::move(o));
		}
		return CMemoryStream::Seek(a0, a1);
	}
	unsigned long getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CMemoryStream *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned long>::value) {
				static pybind11::detail::override_caster_t<unsigned long> caster;
				return pybind11::detail::cast_ref<unsigned long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned long>(std::move(o));
		}
		return CMemoryStream::getTotalBytesCount();
	}
	unsigned long getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CMemoryStream *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned long>::value) {
				static pybind11::detail::override_caster_t<unsigned long> caster;
				return pybind11::detail::cast_ref<unsigned long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned long>(std::move(o));
		}
		return CMemoryStream::getPosition();
	}
	unsigned long ReadBufferImmediate(void * a0, unsigned long a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CMemoryStream *>(this), "ReadBufferImmediate");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned long>::value) {
				static pybind11::detail::override_caster_t<unsigned long> caster;
				return pybind11::detail::cast_ref<unsigned long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned long>(std::move(o));
		}
		return CStream::ReadBufferImmediate(a0, a1);
	}
};

void bind_mrpt_io_CMemoryStream(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::io::CMemoryStream file:mrpt/io/CMemoryStream.h line:26
		pybind11::class_<mrpt::io::CMemoryStream, std::shared_ptr<mrpt::io::CMemoryStream>, PyCallBack_mrpt_io_CMemoryStream, mrpt::io::CStream> cl(M("mrpt::io"), "CMemoryStream", "This CStream derived class allow using a memory buffer as a CStream.\n  This class is useful for storing any required set of variables or objects,\n   and then read them to other objects, or storing them to a file, for\n example.\n\n \n CStream\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::io::CMemoryStream(); }, [](){ return new PyCallBack_mrpt_io_CMemoryStream(); } ) );
		cl.def( pybind11::init<const void *, const unsigned long>(), pybind11::arg("data"), pybind11::arg("nBytesInData") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_io_CMemoryStream const &o){ return new PyCallBack_mrpt_io_CMemoryStream(o); } ) );
		cl.def( pybind11::init( [](mrpt::io::CMemoryStream const &o){ return new mrpt::io::CMemoryStream(o); } ) );
		cl.def("Read", (unsigned long (mrpt::io::CMemoryStream::*)(void *, unsigned long)) &mrpt::io::CMemoryStream::Read, "C++: mrpt::io::CMemoryStream::Read(void *, unsigned long) --> unsigned long", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("Write", (unsigned long (mrpt::io::CMemoryStream::*)(const void *, unsigned long)) &mrpt::io::CMemoryStream::Write, "C++: mrpt::io::CMemoryStream::Write(const void *, unsigned long) --> unsigned long", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("assignMemoryNotOwn", (void (mrpt::io::CMemoryStream::*)(const void *, const unsigned long)) &mrpt::io::CMemoryStream::assignMemoryNotOwn, "Initilize the data in the stream from a block of memory which is NEITHER\n OWNED NOR COPIED by the object, so it must exist during the whole live of\n the object.\n  After assigning a block of data with this method, the object becomes\n \"read-only\", so further attempts to change the size of the buffer will\n raise an exception.\n  This method resets the write and read positions to the beginning. \n\nC++: mrpt::io::CMemoryStream::assignMemoryNotOwn(const void *, const unsigned long) --> void", pybind11::arg("data"), pybind11::arg("nBytesInData"));
		cl.def("clear", (void (mrpt::io::CMemoryStream::*)()) &mrpt::io::CMemoryStream::clear, "Clears the memory buffer. \n\nC++: mrpt::io::CMemoryStream::clear() --> void");
		cl.def("getStreamDescription", (std::string (mrpt::io::CMemoryStream::*)() const) &mrpt::io::CMemoryStream::getStreamDescription, "C++: mrpt::io::CMemoryStream::getStreamDescription() const --> std::string");
		cl.def("Seek", [](mrpt::io::CMemoryStream &o, long const & a0) -> unsigned long { return o.Seek(a0); }, "", pybind11::arg("Offset"));
		cl.def("Seek", (unsigned long (mrpt::io::CMemoryStream::*)(long, enum mrpt::io::CStream::TSeekOrigin)) &mrpt::io::CMemoryStream::Seek, "C++: mrpt::io::CMemoryStream::Seek(long, enum mrpt::io::CStream::TSeekOrigin) --> unsigned long", pybind11::arg("Offset"), pybind11::arg("Origin"));
		cl.def("getTotalBytesCount", (unsigned long (mrpt::io::CMemoryStream::*)() const) &mrpt::io::CMemoryStream::getTotalBytesCount, "Returns the total size of the internal buffer  \n\nC++: mrpt::io::CMemoryStream::getTotalBytesCount() const --> unsigned long");
		cl.def("getPosition", (unsigned long (mrpt::io::CMemoryStream::*)() const) &mrpt::io::CMemoryStream::getPosition, "Method for getting the current cursor position, where 0 is the first\n byte and TotalBytesCount-1 the last one \n\nC++: mrpt::io::CMemoryStream::getPosition() const --> unsigned long");
		cl.def("getRawBufferData", (void * (mrpt::io::CMemoryStream::*)()) &mrpt::io::CMemoryStream::getRawBufferData, "Method for getting a pointer to the raw stored data. The lenght in bytes\n is given by getTotalBytesCount \n\nC++: mrpt::io::CMemoryStream::getRawBufferData() --> void *", pybind11::return_value_policy::automatic);
		cl.def("saveBufferToFile", (bool (mrpt::io::CMemoryStream::*)(const std::string &)) &mrpt::io::CMemoryStream::saveBufferToFile, "Saves the entire buffer to a file \n true on success \n\nC++: mrpt::io::CMemoryStream::saveBufferToFile(const std::string &) --> bool", pybind11::arg("file_name"));
		cl.def("loadBufferFromFile", (bool (mrpt::io::CMemoryStream::*)(const std::string &)) &mrpt::io::CMemoryStream::loadBufferFromFile, "Loads the entire buffer from a file \n true on success \n\nC++: mrpt::io::CMemoryStream::loadBufferFromFile(const std::string &) --> bool", pybind11::arg("file_name"));
		cl.def("setAllocBlockSize", (void (mrpt::io::CMemoryStream::*)(unsigned long)) &mrpt::io::CMemoryStream::setAllocBlockSize, "Change the size of the additional memory block that is reserved whenever\n the current block runs too short (default=0x10000 bytes) \n\nC++: mrpt::io::CMemoryStream::setAllocBlockSize(unsigned long) --> void", pybind11::arg("alloc_block_size"));
		cl.def("assign", (class mrpt::io::CMemoryStream & (mrpt::io::CMemoryStream::*)(const class mrpt::io::CMemoryStream &)) &mrpt::io::CMemoryStream::operator=, "C++: mrpt::io::CMemoryStream::operator=(const class mrpt::io::CMemoryStream &) --> class mrpt::io::CMemoryStream &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}

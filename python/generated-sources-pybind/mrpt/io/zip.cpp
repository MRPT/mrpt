#include <iterator>
#include <memory>
#include <mrpt/io/CStream.h>
#include <mrpt/io/zip.h>
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

void bind_mrpt_io_zip(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::io::zip::compress(void *, size_t, class mrpt::io::CStream &) file:mrpt/io/zip.h line:34
	M("mrpt::io::zip").def("compress", (void (*)(void *, size_t, class mrpt::io::CStream &)) &mrpt::io::zip::compress, "Compress an array of bytes and write the result into a stream. \n\nC++: mrpt::io::zip::compress(void *, size_t, class mrpt::io::CStream &) --> void", pybind11::arg("inData"), pybind11::arg("inDataSize"), pybind11::arg("out"));

	// mrpt::io::zip::decompress(void *, size_t, void *, size_t, unsigned long &) file:mrpt/io/zip.h line:50
	M("mrpt::io::zip").def("decompress", (void (*)(void *, size_t, void *, size_t, unsigned long &)) &mrpt::io::zip::decompress, "Decompress an array of bytes into another one\n \n\n std::exception If the apriori estimated decompressed size is not\n enough\n\nC++: mrpt::io::zip::decompress(void *, size_t, void *, size_t, unsigned long &) --> void", pybind11::arg("inData"), pybind11::arg("inDataSize"), pybind11::arg("outData"), pybind11::arg("outDataBufferSize"), pybind11::arg("outDataActualSize"));

	// mrpt::io::zip::decompress(class mrpt::io::CStream &, size_t, void *, size_t, unsigned long &) file:mrpt/io/zip.h line:58
	M("mrpt::io::zip").def("decompress", (void (*)(class mrpt::io::CStream &, size_t, void *, size_t, unsigned long &)) &mrpt::io::zip::decompress, "Decompress an array of bytes into another one\n \n\n std::exception If the apriori estimated decompressed size is not\n enough\n\nC++: mrpt::io::zip::decompress(class mrpt::io::CStream &, size_t, void *, size_t, unsigned long &) --> void", pybind11::arg("inStream"), pybind11::arg("inDataSize"), pybind11::arg("outData"), pybind11::arg("outDataBufferSize"), pybind11::arg("outDataActualSize"));

}

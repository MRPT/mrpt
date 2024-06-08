#include <mrpt/random/RandomGenerators.h>
#include <sstream> // __str__

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

void bind_mrpt_random_RandomGenerators(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::random::Generator_MT19937 file:mrpt/random/RandomGenerators.h line:43
		pybind11::class_<mrpt::random::Generator_MT19937, std::shared_ptr<mrpt::random::Generator_MT19937>> cl(M("mrpt::random"), "Generator_MT19937", "Portable MT19937 random generator, C++11 UniformRandomBitGenerator\n compliant.\n\n It is ensured to generate the same numbers on any compiler and system.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::random::Generator_MT19937(); } ) );
		cl.def( pybind11::init( [](mrpt::random::Generator_MT19937 const &o){ return new mrpt::random::Generator_MT19937(o); } ) );
		cl.def_static("min", (unsigned int (*)()) &mrpt::random::Generator_MT19937::min, "C++: mrpt::random::Generator_MT19937::min() --> unsigned int");
		cl.def_static("max", (unsigned int (*)()) &mrpt::random::Generator_MT19937::max, "C++: mrpt::random::Generator_MT19937::max() --> unsigned int");
		cl.def("__call__", (unsigned int (mrpt::random::Generator_MT19937::*)()) &mrpt::random::Generator_MT19937::operator(), "C++: mrpt::random::Generator_MT19937::operator()() --> unsigned int");
		cl.def("seed", (void (mrpt::random::Generator_MT19937::*)(const unsigned int)) &mrpt::random::Generator_MT19937::seed, "C++: mrpt::random::Generator_MT19937::seed(const unsigned int) --> void", pybind11::arg("seed"));
	}
	{ // mrpt::random::CRandomGenerator file:mrpt/random/RandomGenerators.h line:74
		pybind11::class_<mrpt::random::CRandomGenerator, std::shared_ptr<mrpt::random::CRandomGenerator>> cl(M("mrpt::random"), "CRandomGenerator", "A thred-safe pseudo random number generator, based on an internal MT19937\n randomness generator.\n The base algorithm for randomness is platform-independent. See\n http://en.wikipedia.org/wiki/Mersenne_twister\n\n For real thread-safety, each thread must create and use its own instance of\n this class.\n\n Single-thread programs can use the static object\n mrpt::random::randomGenerator\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::random::CRandomGenerator(); } ) );
		cl.def( pybind11::init<const unsigned int>(), pybind11::arg("seed") );

		cl.def("drawUniform", (double (mrpt::random::CRandomGenerator::*)(const double, const double)) &mrpt::random::CRandomGenerator::drawUniform<double>, "C++: mrpt::random::CRandomGenerator::drawUniform(const double, const double) --> double", pybind11::arg("Min"), pybind11::arg("Max"));
		cl.def("drawGaussian1D", (double (mrpt::random::CRandomGenerator::*)(const double, const double)) &mrpt::random::CRandomGenerator::drawGaussian1D<double>, "C++: mrpt::random::CRandomGenerator::drawGaussian1D(const double, const double) --> double", pybind11::arg("mean"), pybind11::arg("std"));
		cl.def("randomize", (void (mrpt::random::CRandomGenerator::*)(const unsigned int)) &mrpt::random::CRandomGenerator::randomize, "Initialize the PRNG from the given random seed \n\nC++: mrpt::random::CRandomGenerator::randomize(const unsigned int) --> void", pybind11::arg("seed"));
		cl.def("randomize", (void (mrpt::random::CRandomGenerator::*)()) &mrpt::random::CRandomGenerator::randomize, "Randomize the generators, based on std::random_device \n\nC++: mrpt::random::CRandomGenerator::randomize() --> void");
		cl.def("drawUniform32bit", (uint32_t (mrpt::random::CRandomGenerator::*)()) &mrpt::random::CRandomGenerator::drawUniform32bit, "Generate a uniformly distributed pseudo-random number using the MT19937\n algorithm, in the whole range of 32-bit integers.\n  See: http://en.wikipedia.org/wiki/Mersenne_twister \n\nC++: mrpt::random::CRandomGenerator::drawUniform32bit() --> uint32_t");
		cl.def("drawUniform64bit", (uint64_t (mrpt::random::CRandomGenerator::*)()) &mrpt::random::CRandomGenerator::drawUniform64bit, "Returns a uniformly distributed pseudo-random number by joining two\n 32bit numbers from  \n\nC++: mrpt::random::CRandomGenerator::drawUniform64bit() --> uint64_t");
		cl.def("drawGaussian1D_normalized", (double (mrpt::random::CRandomGenerator::*)()) &mrpt::random::CRandomGenerator::drawGaussian1D_normalized, "Generate a normalized (mean=0, std=1) normally distributed sample.\n  \n\n If desired, pass a pointer to a double which will\n receive the likelihood of the given sample to have been obtained, that\n is, the value of the normal pdf at the sample value.\n\nC++: mrpt::random::CRandomGenerator::drawGaussian1D_normalized() --> double");
	}
	// mrpt::random::getRandomGenerator() file:mrpt/random/RandomGenerators.h line:386
	M("mrpt::random").def("getRandomGenerator", (class mrpt::random::CRandomGenerator & (*)()) &mrpt::random::getRandomGenerator, "A static instance of a CRandomGenerator class, for use in single-thread\n applications \n\nC++: mrpt::random::getRandomGenerator() --> class mrpt::random::CRandomGenerator &", pybind11::return_value_policy::automatic);

	// mrpt::random::random_generator_for_STL(ptrdiff_t) file:mrpt/random/RandomGenerators.h line:391
	M("mrpt::random").def("random_generator_for_STL", (ptrdiff_t (*)(ptrdiff_t)) &mrpt::random::random_generator_for_STL, "A random number generator for usage in STL algorithms expecting a function\n like this (eg, random_shuffle):\n\nC++: mrpt::random::random_generator_for_STL(ptrdiff_t) --> ptrdiff_t", pybind11::arg("i"));

	// mrpt::random::Randomize(const unsigned int) file:mrpt/random/RandomGenerators.h line:448
	M("mrpt::random").def("Randomize", (void (*)(const unsigned int)) &mrpt::random::Randomize, "Randomize the generators.\n   A seed can be providen, or a current-time based seed can be used (default)\n\nC++: mrpt::random::Randomize(const unsigned int) --> void", pybind11::arg("seed"));

	// mrpt::random::Randomize() file:mrpt/random/RandomGenerators.h line:449
	M("mrpt::random").def("Randomize", (void (*)()) &mrpt::random::Randomize, "C++: mrpt::random::Randomize() --> void");

}

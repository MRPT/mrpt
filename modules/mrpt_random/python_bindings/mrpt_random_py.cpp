/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/random/RandomGenerators.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::random — random number generators";

  // -------------------------------------------------------------------------
  // CRandomGenerator — Mersenne Twister random number generator
  // -------------------------------------------------------------------------
  py::class_<mrpt::random::CRandomGenerator>(m, "CRandomGenerator")
      .def(py::init<>())
      .def(py::init<uint32_t>(), "seed"_a)
      .def(
          "randomize", py::overload_cast<uint32_t>(&mrpt::random::CRandomGenerator::randomize),
          "seed"_a, "Initialize the PRNG from the given seed")
      .def(
          "randomize", py::overload_cast<>(&mrpt::random::CRandomGenerator::randomize),
          "Initialize the PRNG from the current time (non-deterministic)")
      .def("drawUniform32bit", &mrpt::random::CRandomGenerator::drawUniform32bit)
      .def("drawUniform64bit", &mrpt::random::CRandomGenerator::drawUniform64bit)
      .def(
          "drawUniform",
          [](mrpt::random::CRandomGenerator& g, double lo, double hi)
          { return g.drawUniform<double>(lo, hi); },
          "min"_a, "max"_a, "Draw a uniform double from [min, max)")
      .def(
          "drawGaussian1D_normalized", &mrpt::random::CRandomGenerator::drawGaussian1D_normalized,
          "Draw a sample from N(0,1)")
      .def(
          "drawGaussian1D",
          [](mrpt::random::CRandomGenerator& g, double mean, double std)
          { return g.drawGaussian1D<double>(mean, std); },
          "mean"_a, "std"_a, "Draw a sample from N(mean, std)")
      // Numpy helpers — draw arrays of samples
      .def(
          "drawUniformArray",
          [](mrpt::random::CRandomGenerator& g, size_t n, double lo, double hi)
          {
            py::array_t<double> arr(static_cast<ssize_t>(n));
            auto buf = arr.mutable_unchecked<1>();
            for (size_t i = 0; i < n; i++)
            {
              buf(i) = g.drawUniform<double>(lo, hi);
            }
            return arr;
          },
          "n"_a, "min"_a = 0.0, "max"_a = 1.0, "Draw n uniform samples as a 1D float64 numpy array")
      .def(
          "drawGaussianArray",
          [](mrpt::random::CRandomGenerator& g, size_t n, double mean, double std)
          {
            py::array_t<double> arr(static_cast<ssize_t>(n));
            auto buf = arr.mutable_unchecked<1>();
            for (size_t i = 0; i < n; i++)
            {
              buf(i) = g.drawGaussian1D<double>(mean, std);
            }
            return arr;
          },
          "n"_a, "mean"_a = 0.0, "std"_a = 1.0,
          "Draw n Gaussian samples as a 1D float64 numpy array")
      .def("__repr__", [](const mrpt::random::CRandomGenerator&) { return "CRandomGenerator()"; });

  // -------------------------------------------------------------------------
  // Module-level: access the global random generator singleton
  // -------------------------------------------------------------------------
  m.def(
      "getRandomGenerator",
      []() -> mrpt::random::CRandomGenerator& { return mrpt::random::getRandomGenerator(); },
      py::return_value_policy::reference, "Returns the global MRPT random generator singleton");

  m.def(
      "Randomize", py::overload_cast<const uint32_t>(mrpt::random::Randomize), "seed"_a,
      "Seed the global random generator");
  m.def(
      "Randomize", py::overload_cast<>(mrpt::random::Randomize),
      "Seed the global random generator from the current time");
}

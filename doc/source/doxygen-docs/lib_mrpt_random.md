\defgroup mrpt_random_grp [mrpt-random]

Random number generator C++ library support uniform, Gaussian, multivariable
distributions.

[TOC]

# Library mrpt-random


This library is part of MRPT but it's pure C++11 and has no other dependencies.
It can be installed in Debian-based systems with:

		sudo apt install libmrpt-random-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

## Overview

`mrpt-random` provides pseudo-random number generators based on a portable
implementation of the MT19937 (Mersenne twister) algorithm, which produces
the same sequence for a given seed on any compiler and platform. Seeding the
generator makes simulations and unit tests reproducible. Note that Gaussian
samples and 64-bit integers are drawn with C++ standard library
distributions, so those sequences may differ between standard libraries.

## Key classes and functions

- mrpt::random::CRandomGenerator: the main generator, with methods to draw
  uniform and Gaussian scalars, vectors and matrices, multivariate Gaussian
  samples from a covariance matrix, and random permutations.
- mrpt::random::getRandomGenerator(): a thread-local, ready-to-use
  CRandomGenerator. In multi-threaded programs that need reproducible
  results, create one generator per thread with its own seed instead.
- mrpt::random::Generator_MT19937: the bare MT19937 engine, usable as a
  standard C++ `UniformRandomBitGenerator`.
- mrpt::random::shuffle() and mrpt::random::portable_uniform_distribution():
  platform-independent replacements of `std::shuffle()` and
  `std::uniform_int_distribution`, to be used with Generator_MT19937.

## Example

```cpp
#include <mrpt/random/RandomGenerators.h>

// Fixed seed: the same sequence on every run.
mrpt::random::CRandomGenerator rng(1234);

const double u = rng.drawUniform(-1.0, 1.0);
const double g = rng.drawGaussian1D(0.0, 0.5);  // mean, std. deviation

std::vector<double> noise(100);
rng.drawGaussian1DVector(noise, 0.0, 0.1);

const std::vector<int> ids = {0, 1, 2, 3, 4};
const auto shuffled = rng.permuteVector(ids);
```

## Examples

- \ref random_examples: histograms of samples from several distributions.
- Python: [mrpt_random_example.py](pymrpt_example_mrpt_random_example.html).

# Library contents

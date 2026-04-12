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

#include <mrpt/bayes/CParticleFilter.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace mrpt::bayes;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::bayes — particle filter and Bayesian estimation";

  // -------------------------------------------------------------------------
  // TParticleFilterAlgorithm enum
  // -------------------------------------------------------------------------
  py::enum_<CParticleFilter::TParticleFilterAlgorithm>(m, "TParticleFilterAlgorithm")
      .value("pfStandardProposal", CParticleFilter::pfStandardProposal)
      .value("pfAuxiliaryPFStandard", CParticleFilter::pfAuxiliaryPFStandard)
      .value("pfOptimalProposal", CParticleFilter::pfOptimalProposal)
      .value("pfAuxiliaryPFOptimal", CParticleFilter::pfAuxiliaryPFOptimal)
      .export_values();

  // -------------------------------------------------------------------------
  // TParticleResamplingAlgorithm enum
  // -------------------------------------------------------------------------
  py::enum_<CParticleFilter::TParticleResamplingAlgorithm>(m, "TParticleResamplingAlgorithm")
      .value("prMultinomial", CParticleFilter::prMultinomial)
      .value("prResidual", CParticleFilter::prResidual)
      .value("prStratified", CParticleFilter::prStratified)
      .value("prSystematic", CParticleFilter::prSystematic)
      .export_values();

  // -------------------------------------------------------------------------
  // CParticleFilter::TParticleFilterOptions — algorithm configuration
  // -------------------------------------------------------------------------
  py::class_<CParticleFilter::TParticleFilterOptions>(m, "TParticleFilterOptions")
      .def(py::init<>())
      .def_readwrite(
          "adaptiveSampleSize", &CParticleFilter::TParticleFilterOptions::adaptiveSampleSize,
          "If true, enable adaptive number of particles")
      .def_readwrite(
          "BETA", &CParticleFilter::TParticleFilterOptions::BETA,
          "Resampling threshold: resample when ESS < BETA (default=0.5)")
      .def_readwrite(
          "sampleSize", &CParticleFilter::TParticleFilterOptions::sampleSize,
          "Initial number of particles (relevant for adaptiveSampleSize=false)")
      .def_readwrite(
          "PF_algorithm", &CParticleFilter::TParticleFilterOptions::PF_algorithm,
          "The particle filter algorithm to use")
      .def_readwrite(
          "resamplingMethod", &CParticleFilter::TParticleFilterOptions::resamplingMethod,
          "The resampling scheme (default=prMultinomial)")
      .def(
          "__repr__",
          [](const CParticleFilter::TParticleFilterOptions& o)
          {
            return "TParticleFilterOptions(BETA=" + std::to_string(o.BETA) +
                   ", sampleSize=" + std::to_string(o.sampleSize) + ")";
          });

  // -------------------------------------------------------------------------
  // CParticleFilter::TParticleFilterStats — statistics from the last run
  // -------------------------------------------------------------------------
  py::class_<CParticleFilter::TParticleFilterStats>(m, "TParticleFilterStats")
      .def(py::init<>())
      .def_readwrite(
          "ESS_beforeResample", &CParticleFilter::TParticleFilterStats::ESS_beforeResample,
          "Effective sample size (ESS) before resampling step")
      .def_readwrite(
          "weightsVariance_beforeResample",
          &CParticleFilter::TParticleFilterStats::weightsVariance_beforeResample,
          "Weight variance before resampling")
      .def(
          "__repr__", [](const CParticleFilter::TParticleFilterStats& s)
          { return "TParticleFilterStats(ESS=" + std::to_string(s.ESS_beforeResample) + ")"; });

  // -------------------------------------------------------------------------
  // CParticleFilter — the particle filter executor
  // -------------------------------------------------------------------------
  py::class_<CParticleFilter>(m, "CParticleFilter")
      .def(py::init<>())
      .def_readwrite(
          "options", &CParticleFilter::m_options, "Algorithm options (TParticleFilterOptions)")
      .def(
          "__repr__",
          [](const CParticleFilter& pf)
          {
            return "CParticleFilter(BETA=" + std::to_string(pf.m_options.BETA) +
                   ", sampleSize=" + std::to_string(pf.m_options.sampleSize) + ")";
          });
}

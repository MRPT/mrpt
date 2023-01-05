/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

namespace mrpt::graphslam::optimizers
{
template <class GRAPH_T>
void CGraphSlamOptimizer<GRAPH_T>::loadParams(const std::string& source_fname)
{
	std::string section("OptimizerParameters");
	this->setVerbosityLevelFromSection(source_fname, section);
}

template <class GRAPH_T>
void CGraphSlamOptimizer<GRAPH_T>::printParams() const
{
	std::cout << "GSO Verbosity: " << this->getMinLoggingLevelStr()
			  << std::endl;
}
}  // namespace mrpt::graphslam::optimizers

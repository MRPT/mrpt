/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT) |
	 |                          http://www.mrpt.org/ |
	 | |
	 | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file |
	 | See: http://www.mrpt.org/Authors - All rights reserved. |
	 | Released under BSD License. See details in http://www.mrpt.org/License |
	 +---------------------------------------------------------------------------+
   */

#ifndef CGRAPHSLAMOPTIMIZER_IMPL_H_N4XWUTK5
#define CGRAPHSLAMOPTIMIZER_IMPL_H_N4XWUTK5

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
	using namespace std;
	std::cout << "GSO Verbosity: " << this->getMinLoggingLevelStr() << endl;
}
}  // end of namespaces

#endif /* end of include guard: CGRAPHSLAMOPTIMIZER_IMPL_H_N4XWUTK5 */



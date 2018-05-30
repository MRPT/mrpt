/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CRANGESCANEDGEREGISTRATIONDECIDER_IMPL_H
#define CRANGESCANEDGEREGISTRATIONDECIDER_IMPL_H

namespace mrpt::graphslam::deciders
{
template <class GRAPH_T>
CRangeScanEdgeRegistrationDecider<GRAPH_T>::CRangeScanEdgeRegistrationDecider()
	: m_last_total_num_nodes(0)
{
}

template <class GRAPH_T>
CRangeScanEdgeRegistrationDecider<GRAPH_T>::~CRangeScanEdgeRegistrationDecider()
{
}

template <class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::loadParams(
	const std::string& source_fname)
{
	MRPT_START;

	parent_t::loadParams(source_fname);
	range_ops_t::params.loadFromConfigFileName(source_fname, "ICP");

	MRPT_END;
}

template <class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::printParams() const
{
	MRPT_START;

	parent_t::printParams();
	range_ops_t::params.dumpToConsole();

	MRPT_END;
}
}  // end of namespaces

#endif /* end of include guard: CRANGESCANEDGEREGISTRATIONDECIDER_IMPL_H */



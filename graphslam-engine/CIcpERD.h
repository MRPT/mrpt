/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CICPERD_H
#define CICPERD_H

#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/obs/CRawlog.h>

#include "CEdgeRegistrationDecider.h"

#include <iostream>

namespace mrpt { namespace graphslam { namespace deciders {

	/**
	 * Register new edges in the graph with the last added node. Criterion for
	 * adding new nodes should  be the goodness of the potential ICP edge. 
	 */
	template<class GRAPH_t>
		class CIcpERD_t {
	  	public:
				typedef typename GRAPH_t::constraint_t constraint_t;
				typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)

				// Public functions
				//////////////////////////////////////////////////////////////
	    	CIcpERD_t();
				CIcpERD_t(GRAPH_t* graph);
	    	virtual ~CIcpERD_t();

	  	protected:

	  	private:


		};

} } } // end of namespaces
#endif /* end of include guard: CICPERD_H */


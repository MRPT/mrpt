#ifndef CNODEREGISTRATIONDECIDER_H
#define CNODEREGISTRATIONDECIDER_H


#include <mrpt/obs/CActionCollection.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

template<class GRAPH_t>
class CNodeRegistrationDecider_t {
  public:
		typedef typename GRAPH_t::constraint_t constraint_t;
		typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)

    CNodeRegistrationDecider_t() {}
    virtual ~CNodeRegistrationDecider_t() {};

		/**
		 * Generic method for fetching the incremental action/observation readings
		 * from the calling function. Implementations of this interface should use
		 * part of the specified parameters and call the checkRegistrationCondition
		 * to check for potential node registration
		 *
		 * Returns true upon successful node registration in the graph
		 */
		virtual bool updateDeciderState( mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation ) = 0;


  protected:
		/**
		 * Method for checking whether a new node should be registered in the
		 * graph. This should be the key-method in any implementation of this
		 * interface. Should call registerNewNode method if the registration
		 * condition is satisfied.
		 *
		 * Returns true upon successful node registration in the graph
		 */
		virtual bool checkRegistrationCondition() = 0;
		/**
		 * Generic method of adding new poses to the graph. Returns true if node was
		 *
		 * Returns true upon successful node registration in the graph, false
		 * otherwise
     */
    virtual bool registerNewNode() = 0;
    
};

#endif /* end of include guard: CNODEREGISTRATIONDECIDER_H */

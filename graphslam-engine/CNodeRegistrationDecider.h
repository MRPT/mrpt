#include <mrpt/obs/CActionCollection.h>
#include <mrpt/utils/CLoadableOptions.h>

template<class GRAPH_t>
class NodeRegistrationDecider_t {
  public:
		typedef typename GRAPH_t::constraint_t constraint_t;
		typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)

    NodeRegistrationDecider_t(GRAPH_t graph):
    	m_graph(graph) {

    	};
    virtual ~NodeRegistrationDecider_t();

		/**
		 * Generic way of adding new poses to the graph. Returns true if node was
		 * successfully added in the graph.
     */
    virtual bool registerNewNode() = 0;

    class TDeciderParameters: public mrpt::utils::CLoadableOptions {
    	TDeciderParameters();
    	~TDeciderParameters();

    	virtual void loadFromConfigFile(
    			const mrpt::utils::CConfigFileBase &source,
    			const std::string &section) = 0;
			void dumpToConsole () const;
			virtual void 	dumpToTextStream (mrpt::utils::CStream &out) const;
    };

		/**
		 * Generic way of reading observations - Rawlog format #1
		 */
    virtual void readObservation(mrpt::obs::CObservationPtr obs) = 0;

    /**
     * Generic way of reading CAction, CSensoryFrame pairs - Rawlog format #2
     */
    virtual void readActionObservationPair(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations );

  protected:
		GRAPH_t m_graph;
		pose_t current_estimated_pose;
		

  private:
    
};

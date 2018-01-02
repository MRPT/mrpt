#ifndef CINCREMENTALNODEREGISTRATIONDECIDER_H_WPM0MYXZ
#define CINCREMENTALNODEREGISTRATIONDECIDER_H_WPM0MYXZ

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>

#include <mrpt/graphslam/interfaces/CNodeRegistrationDecider.h>

#include <math.h>

namespace mrpt
{
namespace graphslam
{
namespace deciders
{
/**\brief Incremental Node registration decider. Decider adds new nodes in the
 * graph in an incremental fashion (adding nodes at the end of the graph)
 *
 * ### .ini Configuration Parameters
 *
 * \htmlinclude config_params_preamble.txt
 *
 * - \b class_verbosity
 *   + \a Section       : NodeRegistrationDeciderParameters
 *   + \a default value : 1 (LVL_INFO)
 *   + \a Required      : FALSE
 *
 * - \b registration_max_distance
 *  + \a Section       : NodeRegistrationDeciderParameters
 *  + \a Default value : 0.5 // meters
 *  + \a Required      : FALSE
 *
 * - \b registration_max_angle
 *  + \a Section       : NodeRegistrationDeciderParameters
 *  + \a Default value : 10 // degrees
 *  + \a Required      : FALSE
 *
 * \ingroup mrpt_graphslam_grp
 *
 */
template <class GRAPH_T>
class CIncrementalNodeRegistrationDecider
	: public virtual mrpt::graphslam::deciders::CNodeRegistrationDecider<
		  GRAPH_T>
{
   public:
	/**\brief Handy typedefs */
	/**\{*/
	/**\brief type of graph constraints */
	typedef typename GRAPH_T::constraint_t constraint_t;
	/**\brief type of underlying poses (2D/3D). */
	typedef typename GRAPH_T::constraint_t::type_value pose_t;
	typedef typename GRAPH_T::global_pose_t global_pose_t;
	typedef CIncrementalNodeRegistrationDecider<GRAPH_T>
		decider_t; /**< self type - Handy typedef */
	/**\brief Node Registration Decider */
	typedef mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>
		parent_t;

	/**\}*/

	CIncrementalNodeRegistrationDecider();
	virtual ~CIncrementalNodeRegistrationDecider();

	/**\name Registration Conditions Specifiers
	*/
	/**\{ */
	/**\brief If estimated position surpasses the registration max values since
	 * the previous registered node, register a new node in the graph.
	 *
	 * \return True on successful registration.
	 */
	virtual bool checkRegistrationCondition();
	virtual bool checkRegistrationConditionPose(
		const mrpt::poses::CPose2D& p1, const mrpt::poses::CPose2D& p2) const;
	virtual bool checkRegistrationConditionPose(
		const mrpt::poses::CPose3D& p1, const mrpt::poses::CPose3D& p2) const;
	/**\} */

	virtual void loadParams(const std::string& source_fname);
	virtual void printParams() const;
	virtual void getDescriptiveReport(std::string* report_str) const;

   protected:
	/**\brief Parameters structure for managing the relevant to the decider
	 * variables in a compact manner
	 */
	struct TParams : public mrpt::utils::CLoadableOptions
	{
	   public:
		TParams();
		~TParams();

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section);
		void dumpToTextStream(mrpt::utils::CStream& out) const;
		/**\brief Return a string with the configuration parameters
		*/
		void getAsString(std::string* params_out) const;
		std::string getAsString() const;

		// max values for new node registration
		double registration_max_distance;
		double registration_max_angle;
	};

	// Public members
	// ////////////////////////////
	TParams params;

   private:
};
}
}
}  // end of namespaces

#include "CIncrementalNodeRegistrationDecider_impl.h"
#endif /* end of include guard: CINCREMENTALNODEREGISTRATIONDECIDER_H_WPM0MYXZ \
		  */

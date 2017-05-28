#ifndef TNODEANNOTATIONS_H
#define TNODEANNOTATIONS_H

#include <mrpt/utils/stl_serialization.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/utils/mrpt_macros.h>
#include <iostream>
#include <string>

namespace mrpt { namespace graphs { namespace detail {

/**\brief  Abstract class from which NodeAnnotations related classes can be
 * implemented
 *
 * \ingroup mrpt_graphs_grp
 */
struct TNodeAnnotations {
	typedef TNodeAnnotations self_t;

	/**\brief Constructor */
	TNodeAnnotations():
		is_node_annots_empty(false)
	{ }
	/**\brief Destructor */
	virtual ~TNodeAnnotations() { }
	/**\brief Generic copy constructor */
	TNodeAnnotations(const TNodeAnnotations& other) { }

	virtual void getAnnotsAsString(std::string* s) const {
		ASSERT_(s);
	}
	std::string retAnnotsAsString() const {
		std::string s;
		this->getAnnotsAsString(&s);
		return s;
	}

	virtual bool operator==(const TNodeAnnotations& other) const {
		return true;
	}
	virtual bool operator!=(const TNodeAnnotations& other) const {
		return (!(*this == other));
	}

	/**\brief Create and return a copy of the TNodeAnnotations object at hand.
	 *
	 * \warning Caller is responsible of afterwards deleting the object which is
	 * allocaed in heap
	 */
	TNodeAnnotations* getCopyOfAnnots() const {return new TNodeAnnotations();}
	/**\brief Set the properties of the current TNodeAnnotations object
	 *
	 * \return True if setting the annotations part is successful.
	 */
	bool setAnnots(const self_t& other) {
		return true; }
	/**\brief Indicates if this is a dummy TNodeAnnotations struct or if it does
	 * contain meaningful data
	 */
	bool is_node_annots_empty;
};

/////////////////////////////////////////////////////////////////////////////

/**\brief Struct to be used as the NODE_ANNOTATIONS template argument in
 * CNetworkOfPoses class instances for use in multiple-robot SLAM applications
 *
 * \ingroup mrpt_graphs_grp
 */
struct TNodeAnnotationsEmpty : public TNodeAnnotations {
	TNodeAnnotationsEmpty() {
		this->is_node_annots_empty = true;
	}

};

} } } // end of namespaces

// declare as ttypename - in mrpt::utils namespace
namespace mrpt { namespace utils {
	MRPT_DECLARE_TTYPENAME(mrpt::graphs::detail::TNodeAnnotationsEmpty)
} } // end of namespaces

#endif /* end of include guard: TNODEANNOTATIONS_H */

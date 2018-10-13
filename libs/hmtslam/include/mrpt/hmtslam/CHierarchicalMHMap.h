/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/hmtslam/CHMHMapNode.h>
#include <mrpt/hmtslam/CHMHMapArc.h>
#include <mrpt/hmtslam/CHierarchicalMapMHPartition.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::hmtslam
{
/** The most high level class for storing hybrid, multi-hypothesis maps in a
 * graph-based model.
 *   This class is used within the HMT-SLAM implementation in CHMTSLAM.
 * \sa CHMTSLAM, CHMHMapArc, CHMHMapNode, CHierarchicalMHMapPartition
 * \ingroup mrpt_hmtslam_grp
 */
class CHierarchicalMHMap : public mrpt::serialization::CSerializable,
						   public CHierarchicalMapMHPartition
{
	friend class CHMHMapArc;
	friend class CHMHMapNode;

	DEFINE_SERIALIZABLE(CHierarchicalMHMap)
   protected:
	/** Event handler to be called just after a node has being created: it will
	 * be added to the internal list.
	 */
	void onNodeAddition(CHMHMapNode::Ptr& node);

	/** Event handler to be called just after an arc has being created: it will
	 * be added to the internal list.
	 */
	void onArcAddition(CHMHMapArc::Ptr& arc);

	/** Event handler to be called just before a node is being destroyed: it
	 * will be removed from the internal list.
	 * \note At *addition we use a smart pointer to assure all the implied guys
	 * use the same smrt. pnt., but at destructors the objects don't know
	 * anything but "this", thus the usage of plain pointers.
	 */
	void onNodeDestruction(CHMHMapNode* node);

	/** Event handler to be called just before an arc is being destroyed: it
	 * will be removed from the internal list.
	 * \note At *addition we use a smart pointer to assure all the implied guys
	 * use the same smrt. pnt., but at destructors the objects don't know
	 * anything but "this", thus the usage of plain pointers.
	 */
	void onArcDestruction(CHMHMapArc* arc);

   public:
	/** Default constructor
	 */
	CHierarchicalMHMap();

	/** Destructor
	 */

	/** Save the whole graph as a XML file */
	void dumpAsXMLfile(std::string fileName) const;

	/** Load a graph from a XML file */
	void loadFromXMLfile(std::string fileName);

	~CHierarchicalMHMap() override;

	/** Erase all the contents of map (It delete all nodes/arcs objects)
	 */
	void clear();

};  // End of class def.

}  // namespace mrpt::hmtslam

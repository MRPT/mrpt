/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CHierarchicalMHMap_H
#define CHierarchicalMHMap_H

#include <mrpt/hmtslam/CHMHMapNode.h>
#include <mrpt/hmtslam/CHMHMapArc.h>
#include <mrpt/hmtslam/CHierarchicalMapMHPartition.h>
#include <mrpt/slam/CSimpleMap.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/utils/CSerializable.h>


namespace mrpt
{
	namespace hmtslam
	{
		using namespace mrpt::slam;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CHierarchicalMHMap, mrpt::utils::CSerializable,  HMTSLAM_IMPEXP )

		/** The most high level class for storing hybrid, multi-hypothesis maps in a graph-based model.
		 *   This class is used within the HMT-SLAM implementation in CHMTSLAM.
		 * \sa CHMTSLAM, CHMHMapArc, CHMHMapNode, CHierarchicalMHMapPartition
		  * \ingroup mrpt_hmtslam_grp
		 */
		class HMTSLAM_IMPEXP CHierarchicalMHMap : public mrpt::utils::CSerializable, public CHierarchicalMapMHPartition
		{
			friend class HMTSLAM_IMPEXP CHMHMapArc;
			friend class HMTSLAM_IMPEXP CHMHMapNode;

			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CHierarchicalMHMap )
		protected:
			/** Event handler to be called just after a node has being created: it will be added to the internal list.
			  */
			 void  onNodeAddition(CHMHMapNodePtr &node);

			/** Event handler to be called just after an arc has being created: it will be added to the internal list.
			  */
			 void  onArcAddition(CHMHMapArcPtr &arc);

			/** Event handler to be called just before a node is being destroyed: it will be removed from the internal list.
			  * \note At *addition we use a smart pointer to assure all the implied guys use the same smrt. pnt., but at destructors the objects don't know anything but "this", thus the usage of plain pointers.
			  */
			 void  onNodeDestruction(CHMHMapNode* node);

			/** Event handler to be called just before an arc is being destroyed: it will be removed from the internal list.
			  * \note At *addition we use a smart pointer to assure all the implied guys use the same smrt. pnt., but at destructors the objects don't know anything but "this", thus the usage of plain pointers.
			  */
			 void  onArcDestruction(CHMHMapArc* arc);

		public:
			 /** Default constructor
			  */
			 CHierarchicalMHMap();

			 /** Destructor
			  */
			 
			 /** Save the whole graph as a XML file */
			 void  dumpAsXMLfile(std::string fileName) const;

			 /** Load a graph from a XML file */
			 void loadFromXMLfile(std::string fileName);


			 virtual ~CHierarchicalMHMap();

			 /** Erase all the contents of map (It delete all nodes/arcs objects)
			   */
			 void  clear();

		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif

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


#include <mrpt/hmtslam.h> // Precomp header

#include <mrpt/random.h>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CHMHMapNode, CSerializable, mrpt::hmtslam )

/*---------------------------------------------------------------
						Default constructor
  ---------------------------------------------------------------*/
CHMHMapNode::CHMHMapNode(
	CHierarchicalMHMap	*parent,
	const THypothesisIDSet	&hyps) :
		m_hypotheses(hyps),
		m_ID(),
		m_arcs(),
		m_parent(parent),
		m_nodeType( NODE_TYPES, DEFAULT_NODE_TYPE),
		m_label("none")
{
	// Assure that ID is unique in the graph:
	// -----------------------------------------
	if (m_parent.get())
	{
		// Parent will be NULL only in the default constructor for a temporary
		//  initialization before loading the object from "readFromStream"
		m_ID=0;
		do
		{
			m_ID++; /* = (((uint64_t)randomGenerator.drawUniform(0.0f,0xFFFF)) << 32) |
				(((uint64_t)randomGenerator.drawUniform(0.0f,0xFFFF)) << 16) |
				(((uint64_t)randomGenerator.drawUniform(0.0f,0xFFFF)));*/
		} while (m_parent->getNodeByID(m_ID));
	}
}

CHMHMapNodePtr CHMHMapNode::Create(
	CHierarchicalMHMap		*parent,
	const THypothesisIDSet	&hyps )
{
	CHMHMapNodePtr obj = CHMHMapNodePtr( new CHMHMapNode(parent,hyps) );
	if (parent) parent->onNodeAddition(obj);
	return obj;
}


/*---------------------------------------------------------------
					Destructor
  ---------------------------------------------------------------*/
CHMHMapNode::~CHMHMapNode()
{
	// To the graph:
	if (m_parent.get())
		m_parent->onNodeDestruction(this);

	// To the arcs:
	for (TArcList::iterator it=m_arcs.begin();it!=m_arcs.end();++it)
		(*it)->onNodeDestruction(this);
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CHMHMapNode::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		// Data:
		out << m_ID << m_label;
		out << m_nodeType.getType();
		out << m_annotations;
		out << m_hypotheses;
	}

}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CHMHMapNode::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			std::string		type;
			// Data:
			in >> m_ID >> m_label >> type >> m_annotations >> m_hypotheses;

			m_nodeType.setType(type);

			// It's not necessary since at ::Create this is already done (but...check!)
			//if (m_parent.get())
			//	m_parent->onNodeAddition(this);

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					onArcDestruction
  ---------------------------------------------------------------*/
void  CHMHMapNode::onArcDestruction(CHMHMapArc	*arc)
{
	MRPT_START

	// Important note: We cannot create a temporary smart pointer here, since 
	//  it will lead to an infinity recursion!  (BUGFIX, JLBC SEP-2009)

	// Check if arc is from/to this node:
	if (arc->m_nodeFrom==m_ID || arc->m_nodeTo==m_ID)
	{
		// Remove from the list:
		TArcList::iterator it = m_arcs.find_ptr_to(arc);
		if (it!=m_arcs.end())
			m_arcs.erase(it);
	}

	MRPT_END
}

/*---------------------------------------------------------------
					onArcAddition
  ---------------------------------------------------------------*/
void  CHMHMapNode::onArcAddition(CHMHMapArcPtr &arc)
{
	MRPT_START

	// Check if arc is from/to this node:
	if (arc->m_nodeFrom==m_ID || arc->m_nodeTo==m_ID)
	{
		// Already in the list?
		TArcList::iterator it = m_arcs.find(arc);
		if (it==m_arcs.end())
			m_arcs.push_back(arc);	// Add to the list:
	}

	MRPT_END
}
/*---------------------------------------------------------------
					onArcAddition
  ---------------------------------------------------------------*/
CHMHMapNode::TNodeID CHMHMapNode::getID() const
{
	return m_ID;
}

/*---------------------------------------------------------------
					getLevelInTheHierarchy
  ---------------------------------------------------------------*/
unsigned int CHMHMapNode::getLevelInTheHierarchy()
{
	TArcList::iterator		itArc;
	unsigned int			level = 0;

	for (itArc=m_arcs.begin();itArc!=m_arcs.end();itArc++)
	{
		// I am a "level+1" from the level below if a "belongs" arc points to me:
		if ((*itArc)->m_arcType.isType("Membership") &&
			(*itArc)->m_nodeTo == this->m_ID )
		{
			unsigned int L = m_parent->getNodeByID(  (*itArc)->m_nodeFrom )->getLevelInTheHierarchy();
			level = max(L+1, level);
		}
	}

	return level;
}

/*---------------------------------------------------------------
					getRelatedArcsCount
  ---------------------------------------------------------------*/
unsigned int CHMHMapNode::getRelatedArcsCount()
{
	return (unsigned int)m_arcs.size();
}

/*---------------------------------------------------------------
					getArcs
  ---------------------------------------------------------------*/
void CHMHMapNode::getArcs( TArcList &out, const THypothesisID &hyp_id ) const
{
	out.clear();
	for (TArcList::const_iterator it=m_arcs.begin();it!=m_arcs.end();++it)
		if ((*it)->m_hypotheses.has(hyp_id))
			out.push_back(*it);
}


/*---------------------------------------------------------------
					getArcs
  ---------------------------------------------------------------*/
void CHMHMapNode::getArcs( TArcList &out, const char *arcType, const THypothesisID &hyp_id ) const
{
	out.clear();
	for (TArcList::const_iterator it=m_arcs.begin();it!=m_arcs.end();++it)
		if ((*it)->m_hypotheses.has(hyp_id) && (*it)->m_arcType.isType(arcType) )
			out.push_back(*it);
}

/*---------------------------------------------------------------
					isNeighbor
  ---------------------------------------------------------------*/
bool CHMHMapNode::isNeighbor(const TNodeID &otherArea, const THypothesisID &hyp_id ) const
{
	for (TArcList::const_iterator it=m_arcs.begin();it!=m_arcs.end();++it)
		if ((*it)->m_hypotheses.has(hyp_id) &&
			( (*it)->m_nodeFrom==otherArea || (*it)->m_nodeTo==otherArea ) )
			return true;
	return false; // Nope
}

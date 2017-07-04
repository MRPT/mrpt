/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h" // Precomp header

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::hmtslam;

IMPLEMENTS_SERIALIZABLE( CHMHMapArc, CSerializable, mrpt::hmtslam )

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CHMHMapArc::CHMHMapArc(
	const CHMHMapNode::TNodeID		&from ,
	const CHMHMapNode::TNodeID		&to,
	const THypothesisIDSet			&hyps,
	CHierarchicalMHMap		*parent) :
		m_hypotheses(hyps),
		m_nodeFrom(from),
		m_nodeTo(to),
		m_parent(parent),
		m_arcType(ARC_TYPES,DEFAULT_ARC_TYPE ),
		m_annotations()
{
	// parent will be nullptr only inside a ">>" operation, as a temporal
	//  initialization of an empty object with the default constructor:
	// To the graph:
}


/*---------------------------------------------------------------
						Other constructor
  ---------------------------------------------------------------*/
CHMHMapArc::CHMHMapArc(
	CHMHMapNode::Ptr		&from,
	CHMHMapNode::Ptr		&to,
	const THypothesisIDSet		&hyps,
	CHierarchicalMHMap			*parent) :
		m_hypotheses(hyps),
		m_nodeFrom(),
		m_nodeTo(),
		m_parent(parent),
		m_arcType(ARC_TYPES,DEFAULT_ARC_TYPE ),
		m_annotations()
{
	if (from) m_nodeFrom = from->getID();
	if (to)   m_nodeTo = to->getID();

	// parent will be nullptr only inside a ">>" operation, as a temporal
	//  initialization of an empty object with the default constructor:
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CHMHMapArc::~CHMHMapArc()
{
	CHMHMapNode::Ptr node;
	// To the nodes:
	if ((node = m_parent->getNodeByID(m_nodeFrom)))
		node->onArcDestruction(this);
	if ((node = m_parent->getNodeByID(m_nodeTo)))
		node->onArcDestruction(this);

	// To the graph:
	if (m_parent.get())
		m_parent->onArcDestruction(this);
}

/*---------------------------------------------------------------
						onNodeDestruction
  ---------------------------------------------------------------*/
void  CHMHMapArc::onNodeDestruction(CHMHMapNode *node)
{
	MRPT_START

	// Check if arc is from/to this node:
	if (node->getID()==m_nodeFrom)	m_nodeFrom = AREAID_INVALID;
	if (node->getID()==m_nodeTo)	m_nodeTo = AREAID_INVALID;

	MRPT_END
}


/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CHMHMapArc::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << m_nodeFrom << m_nodeTo << m_arcType.getType() << m_annotations << m_hypotheses;
	}

}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CHMHMapArc::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			std::string		type;

			in >> m_nodeFrom >> m_nodeTo >> type >> m_annotations >> m_hypotheses;

			m_arcType.setType(type);

			// Find my smart pointer in the HMT map: we MUST have only 1 smrt. pointer pointing to the same object!!			
			CHMHMapArc::Ptr myPtr;
			for (TArcList::const_iterator it = m_parent->m_arcs.begin();it != m_parent->m_arcs.end();++it)
			{
				if (it->get()==this)
				{
					myPtr = *it;
					break;
				}
			}
			ASSERTMSG_(myPtr,"I cannot be found in my parent HMT map!")

			CHMHMapNode::Ptr node;
			// It's not necessary since at ::Create this is already done (but...check!)
			//m_parent->onArcAddition(this);  

			// To the nodes:
			if ((node = m_parent->getNodeByID(m_nodeFrom)))
				node->onArcAddition(myPtr);
			if ((node = m_parent->getNodeByID(m_nodeTo)))
				node->onArcAddition(myPtr);

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
						TArcList::debugDump
  ---------------------------------------------------------------*/
void  TArcList::debugDump()
{
	printf("Dumping arcs list: %u elements\n", (unsigned int)size());
	for (iterator i=begin();i!=end();++i)
	{
		printf("\t'%s'\t-> '%s'\n", (*i)->m_parent->getNodeByID( (*i)->getNodeFrom() )->m_label.c_str(),
									(*i)->m_parent->getNodeByID( (*i)->getNodeTo() )->m_label.c_str() );
	}
}


void TArcList::read( utils::CStream &in )
{
	uint32_t i,n;
	in >> n;
	BASE::clear();
	for (i=0;i<n;i++)
	{
		CHMHMapArc::Ptr theObj = std::make_shared<CHMHMapArc>();
		in >> *theObj;
		this->push_back( theObj );
	}
}
void TArcList::write( utils::CStream &out ) const
{
	out << static_cast<uint32_t>(this->size());
	for (const_iterator i=begin();i!=end();++i)
		out << **i;
}


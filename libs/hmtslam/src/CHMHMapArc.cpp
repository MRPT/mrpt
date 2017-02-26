/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
	// parent will be NULL only inside a ">>" operation, as a temporal
	//  initialization of an empty object with the default constructor:
	// To the graph:
}

CHMHMapArcPtr CHMHMapArc::Create(
	const CHMHMapNode::TNodeID		&from,
	const CHMHMapNode::TNodeID		&to,
	const THypothesisIDSet			&hyps,
	CHierarchicalMHMap		*parent) 
{
	CHMHMapArcPtr obj = CHMHMapArcPtr(new CHMHMapArc(from,to,hyps,parent));
	if (parent) 
	{
		parent->onArcAddition(obj);
		CHMHMapNodePtr node;
		if ( (node = parent->getNodeByID(from)).present() )
			node->onArcAddition(obj);
		if ( (node = parent->getNodeByID(to)).present() )
			node->onArcAddition(obj);
	}
	return obj;
}


/*---------------------------------------------------------------
						Other constructor
  ---------------------------------------------------------------*/
CHMHMapArc::CHMHMapArc(
	CHMHMapNodePtr		&from,
	CHMHMapNodePtr		&to,
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

	// parent will be NULL only inside a ">>" operation, as a temporal
	//  initialization of an empty object with the default constructor:
}

CHMHMapArcPtr CHMHMapArc::Create(
	CHMHMapNodePtr		&from,
	CHMHMapNodePtr		&to,
	const THypothesisIDSet		&hyps,
	CHierarchicalMHMap			*parent)
{
	CHMHMapArcPtr obj = CHMHMapArcPtr(new CHMHMapArc(from,to,hyps,parent));
	if (parent) 
	{
		// To the graph:
		parent->onArcAddition(obj);
		// To the nodes:
		if (from)	from->onArcAddition(obj);
		if (to)		to->onArcAddition(obj);
	}
	return obj;
}


/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CHMHMapArc::~CHMHMapArc()
{
	CHMHMapNodePtr node;
	// To the nodes:
	if ((node = m_parent->getNodeByID(m_nodeFrom)).present())
		node->onArcDestruction(this);
	if ((node = m_parent->getNodeByID(m_nodeTo)).present())
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
			CHMHMapArcPtr myPtr;
			for (TArcList::const_iterator it = m_parent->m_arcs.begin();it != m_parent->m_arcs.end();++it)
			{
				if (it->pointer()==this)
				{
					myPtr = *it;
					break;
				}
			}
			ASSERTMSG_(myPtr.present(),"I cannot be found in my parent HMT map!")

			CHMHMapNodePtr node;
			// It's not necessary since at ::Create this is already done (but...check!)
			//m_parent->onArcAddition(this);  

			// To the nodes:
			if ((node = m_parent->getNodeByID(m_nodeFrom)).present())
				node->onArcAddition(myPtr);
			if ((node = m_parent->getNodeByID(m_nodeTo)).present())
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
		CHMHMapArcPtr theObj = CHMHMapArc::Create();
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


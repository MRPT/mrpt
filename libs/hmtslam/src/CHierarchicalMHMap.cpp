/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h" // Precomp header

#include <mrpt/utils/CSimpleDatabase.h>
#include <mrpt/utils/metaprogramming.h>
#include <mrpt/poses/CPoint2D.h>

using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::hmtslam;

IMPLEMENTS_SERIALIZABLE( CHierarchicalMHMap, CSerializable, mrpt::hmtslam )

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CHierarchicalMHMap::CHierarchicalMHMap()
{

}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CHierarchicalMHMap::~CHierarchicalMHMap()
{
	clear();
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CHierarchicalMHMap::clear()
{
	// Remaining arcs and nodes will be deleted now
	// A delicate issue; we must:
	// 1) .clear() all the smart pointers
	// 2) then, empty the list of nodes/arcs, which will only
	//     contain empty smart pointers, thus will not raise callbacks again.
	// ----------------------------------------------------------------
	TNodeList  nodes = m_nodes;
	TArcList   arcs = m_arcs;

	// 1:
	std::for_each(nodes.begin(),nodes.end(), metaprogramming::ObjectClearSecond() );
	std::for_each(arcs.begin(),arcs.end(), metaprogramming::ObjectClear2() );

	// 2:
	m_nodes.clear();
	m_arcs.clear();
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CHierarchicalMHMap::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		uint32_t	n;
		TNodeList::const_iterator it;
		TArcList::const_iterator  it2;

		// Nodes:
		n = static_cast<uint32_t>(nodeCount());
		out << n;
		for (it=m_nodes.begin();it!=m_nodes.end();it++)
			out << *it->second;

		// Arcs:
		n = static_cast<uint32_t>(arcCount());
		out << n;
		for (it2=m_arcs.begin();it2!=m_arcs.end();it2++)
			out << *(*it2);
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CHierarchicalMHMap::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	i,n;

			// Clear previous contents:
			clear();

			// Nodes:
			in >> n;
			for (i=0;i<n;i++)
            {
				CHMHMapNodePtr node = CHMHMapNode::Create(this); // This insert the node in my internal list via the callback method
            	in >> *node;
            }

			// Arcs:
			in >> n;
			for (i=0;i<n;i++)
            {
				 // This insert the node in my internal list via the callback method
				CHMHMapArcPtr arc = CHMHMapArc::Create( CHMHMapNodePtr(),CHMHMapNodePtr(), THypothesisIDSet() ,this);
            	in >> *arc;
            }

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
						onNodeDestruction
  ---------------------------------------------------------------*/
void  CHierarchicalMHMap::onNodeDestruction(CHMHMapNode *node)
{
	TNodeList::iterator it;

	it = m_nodes.find( node->getID() );

	if (it!=m_nodes.end())
		if ( node == it->second.pointer() )
			m_nodes.erase( it );
}

/*---------------------------------------------------------------
						onArcDestruction
  ---------------------------------------------------------------*/
void  CHierarchicalMHMap::onArcDestruction(CHMHMapArc *arc)
{
	// Important note: We cannot create a temporary smart pointer here, since
	//  it will lead to an infinity recursion!  (BUGFIX, JLBC SEP-2009)
	TArcList::iterator it = m_arcs.find_ptr_to(arc);
	if (it!=m_arcs.end())
		m_arcs.erase( it );
}

/*---------------------------------------------------------------
						onNodeAddition
  ---------------------------------------------------------------*/
void  CHierarchicalMHMap::onNodeAddition(CHMHMapNodePtr &node)
{
	// Check if it is not already in the list:
	TNodeList::iterator it = m_nodes.find( node->m_ID );

	if ( it!=m_nodes.end() )
	{
		// Already in the list:
		ASSERT_( node == it->second );
		return;
	}
	else
	{
		// It is a new node: add to the list:
		m_nodes[ node->m_ID ] = node;
	}
}

/*---------------------------------------------------------------
						onArcAddition
  ---------------------------------------------------------------*/
void  CHierarchicalMHMap::onArcAddition(CHMHMapArcPtr &arc)
{
	// Check if it is not already in the list:
	TArcList::iterator it = m_arcs.find(arc);

	if (it==m_arcs.end())		// Is it new?
		m_arcs.push_back( arc );
}
/*---------------------------------------------------------------
				loadFromXMLfile
  ---------------------------------------------------------------*/


void CHierarchicalMHMap::loadFromXMLfile(std::string fileName)
{
	CSimpleDatabase		db;
	CSimpleDatabaseTablePtr table;
	size_t j,numnodes,numarcs;

	std::map<size_t,CHMHMapNodePtr>  nodemap;
	std::map<size_t,CHMHMapNodePtr>::iterator  nodemapit;
	typedef std::pair<size_t,CHMHMapNodePtr> IDPair;

	std::map<size_t,CHMHMapNode::TNodeID>  nodeanotmap;
	std::map<size_t,CHMHMapNode::TNodeID>::iterator  nodeanotmapit;
	typedef std::pair<size_t,CHMHMapNode::TNodeID> IDnodeanotPair;

	db.loadFromXML(fileName);

		table=db.getTable("nodes");
		numnodes=table->getRecordCount();

		//printf("Loading nodes\n");
		std::vector<std::string> node_anots;

		for (j=0;j<numnodes;j++)
		{
				CHMHMapNodePtr node;
				node=CHMHMapNode::Create(this);
				node->m_label=table->get(j,"nodename");
				nodemap.insert(IDPair( atoi(table->get(j,"id").c_str()),node)   );
				node->m_nodeType.setType(table->get(j,"nodetype"));
				node->m_hypotheses.insert( COMMON_TOPOLOG_HYP);
				printf("Loaded node %s\n",node->m_label.c_str());


				std::deque<std::string> lista;
				mrpt::system::tokenize(table->get(j,"annotation-list")," ",lista);

				for (size_t r=0;r<lista.size();r++)
					nodeanotmap.insert(IDnodeanotPair((size_t)atoi(lista[r].c_str()),node->getID()));

				//A map with key the id of annotations and value the id of nodes;

		}

		table=db.getTable("arcs");
		numarcs=table->getRecordCount();
		printf("Loading arcs\n");
			for (j=0;j<numarcs;j++)
			{
				CHMHMapArcPtr arc,arcrev;
				size_t from,to;
				from=atoi(table->get(j,"from").c_str());
				to=atoi(table->get(j,"to").c_str());


				CHMHMapNodePtr nodefrom,nodeto;
				nodemapit=nodemap.find(from);
				nodefrom=nodemapit->second;
				std::cout<<"finding nodes"<<std::endl;

				nodemapit=nodemap.find(to);
				nodeto=nodemapit->second;
				std::cout<<"added arc from "<< nodefrom->m_label << " to " <<nodeto->m_label<<std::endl;

				arc=CHMHMapArc::Create(nodefrom,nodeto,0,this);
				arc->m_arcType.setType(table->get(j,"arctype"));
				arc->m_hypotheses.insert( COMMON_TOPOLOG_HYP);


				if (atoi(table->get(j,"bidirectional").c_str())==1)
				{
					printf("Creating bidirectional arc\n");
					arcrev=CHMHMapArc::Create(nodeto,nodefrom,0,this);
					arcrev->m_arcType.setType(table->get(j,"arctype"));
					arcrev->m_hypotheses.insert( COMMON_TOPOLOG_HYP);

				}

			}

			std::cout<<"Graph with ["<<numnodes<<"] nodes and ["<<numarcs<<"] arcs loaded successfully."<<std::endl;

			table=db.getTable("annotations");
			size_t numannot=table->getRecordCount();
			printf("Loading annotations\n");
			for (size_t j=0;j<numannot;j++)
			{
				string type=table->get(j,"annotation-type");
				string value=table->get(j,"annotation-value");
				nodeanotmapit =nodeanotmap.find(atoi(table->get(j,"id").c_str()));

				if (nodeanotmapit!=nodeanotmap.end())
				{
					if (type=="placePose")
					{
						CPoint2DPtr o=CPoint2D::Create();
						o->fromString(value);

						CHMHMapNodePtr node=getNodeByID(nodeanotmapit->second);

						node->m_annotations.set(NODE_ANNOTATION_PLACE_POSE,o,COMMON_TOPOLOG_HYP);

					}
				}
			}




}
/*---------------------------------------------------------------
				dumpAsXMLfile
  ---------------------------------------------------------------*/

void  CHierarchicalMHMap::dumpAsXMLfile(std::string fileName) const
{
	CSimpleDatabase		db;
	CSimpleDatabaseTablePtr tablenodes,tablearcs,tableannots;
	size_t i;

	tablenodes = db.createTable("nodes");
	tablearcs = db.createTable("arcs");
	tableannots = db.createTable("annotations");

	tablenodes->addField("id");
	tablenodes->addField("nodename");
	tablenodes->addField("nodetype");
	tablenodes->addField("annotation-list");

	tablearcs->addField("id");
	tablearcs->addField("from");
	tablearcs->addField("to");
	tablearcs->addField("arctype");
	tablearcs->addField("bidirectional");
	tablearcs->addField("annotation-list");


	tableannots->addField("id");
	tableannots->addField("annotation-type");
	tableannots->addField("annotation-value");

	//for nodes
	printf("Generating nodes\n");
	for (TNodeList::const_iterator it=m_nodes.begin();it!=m_nodes.end();++it)
	{
		i=tablenodes->appendRecord();
		tablenodes->set(i,"nodename",it->second->m_label.c_str());
		tablenodes->set(i,"id",format("%i",static_cast<int>(it->second->getID())));
		tablenodes->set(i,"nodetype",it->second->m_nodeType.getType());

		tablenodes->set(i,"annotation-list",".");
		for (CMHPropertiesValuesList::const_iterator ann = it->second->m_annotations.begin(); ann != it->second->m_annotations.end(); ++ann)
		{
			size_t j=tableannots->appendRecord();
			tableannots->set(j,"id",format("%u",static_cast<unsigned int>(j)));
			tableannots->set(j,"annotation-type",ann->name.c_str());
			ASSERT_(ann->value.present())
			string  str;
			if (IS_CLASS(ann->value,CPoint2D))
			{
				CPoint2DPtr o = CPoint2DPtr(ann->value);
				o->asString(str);
			}
			else
			{
				str=ObjectToString(ann->value.pointer());
			}
			tableannots->set(j,"annotation-value",str);
			if (tablenodes->get(j,"annotation-list")==".")
				tablenodes->set(i,"annotation-list",format("%u",static_cast<unsigned int>(j)));
			else
				tablenodes->set(i,"annotation-list",tablenodes->get(j,"annotation-list")+format("%u",static_cast<unsigned int>(j)));
		}
	}

	//for arcs
	printf("Generating arcs (%u)\n",static_cast<unsigned int>(m_arcs.size()));



	for (TArcList::const_iterator it=m_arcs.begin();it!=m_arcs.end();++it)
	{

		size_t fromid,toid;

		fromid=(int)(*it)->getNodeFrom();
		toid=(int)(*it)->getNodeTo();


		i=tablearcs->appendRecord();
		tablearcs->set(i,"id",format("%u",static_cast<unsigned int>(i)));
		tablearcs->set(i,"from",format("%u",static_cast<unsigned int>(fromid)));
		tablearcs->set(i,"to",format("%u",static_cast<unsigned int>(toid)));
		tablearcs->set(i,"arctype",(*it)->m_arcType.getType());


		for (CMHPropertiesValuesList::const_iterator ann = (*it)->m_annotations.begin(); ann != (*it)->m_annotations.end(); ++ann)
		{

			i=tableannots->appendRecord();
			tableannots->set(i,"id",format("%u",static_cast<unsigned int>(i)));
			tableannots->set(i,"annotation-type",ann->name.c_str());

			// CSerializable *o=ann->value->duplicate();  // JL: duplicate???
		//	tableannots->set(i,"annotation-value",ObjectToString(o));
		}
	}
	printf("Generating XML file\n");
	db.saveAsXML(fileName.c_str());


/*
	std::string  s;
		s += format("NODE ID: %i\t LABEL:%s\tARCS: ", (int)it->second->getID(), it->second->m_label.c_str() );
		TArcList arcs;
		it->second->getArcs(arcs);
		for (TArcList::const_iterator a=arcs.begin();a!=arcs.end();++a)
			s += format("%i-%i, ", (int)(*a)->getNodeFrom(), (int)(*a)->getNodeTo() );

		st << s;

		for (CMHPropertiesValuesList::const_iterator ann = it->second->m_annotations.begin(); ann != it->second->m_annotations.end(); ++ann)
		{
			s= format("   [HYPO ID #%02i] Annotation '%s' Class: ", (int)ann->ID, ann->name.c_str() );
			if ( ann->value )
					s+= string(ann->value->GetRuntimeClass()->className);
			else	s+= "(NULL)";

			st << s;

			if ( ann->name == NODE_ANNOTATION_REF_POSEID )
			{
				TPoseID refID;
				it->second->m_annotations.getElemental(NODE_ANNOTATION_REF_POSEID, refID, ann->ID );
				st << format("     VALUE: %i",(int)refID);
			}
			else if ( ann->name == NODE_ANNOTATION_POSES_GRAPH )
			{
				CRobotPosesGraphPtr posesGraph = it->second->m_annotations.getAs<CRobotPosesGraph>(NODE_ANNOTATION_POSES_GRAPH,ann->ID);
				ASSERT_(posesGraph);

				st << format("     CRobotPosesGraph has %i poses:",(int)posesGraph->size());
				CPose3D  pdfMean;
				for (CRobotPosesGraph::const_iterator p=posesGraph->begin();p!=posesGraph->end();++p)
				{
					const CPose3DPDFParticles &pdf = p->second.pdf;
					pdf.getMean(pdfMean);
					st << format("       Pose %i \t (%.03f,%.03f,%.03fdeg)",
						(int)p->first,
						pdfMean.x(),
						pdfMean.y(),
						RAD2DEG(pdfMean.yaw()));
				}
			}
		}

		st << "";

	}


	st << "";
	st << "";
	st << "LIST OF ARCS";
	st << "================";

	for (TArcList::const_iterator it=m_arcs.begin();it!=m_arcs.end();++it)
	{
		std::string  s;
		s += format("ARC: %i -> %i\n", (int)(*it)->getNodeFrom(), (int)(*it)->getNodeTo() );

		s+= string("   Arc type: ")+(*it)->m_arcType.getType();

		st << s;

		for (CMHPropertiesValuesList::const_iterator ann = (*it)->m_annotations.begin(); ann != (*it)->m_annotations.end(); ++ann)
		{
			s= format("   [HYPO ID #%02i] Annotation '%s' Class: ", (int)ann->ID, ann->name.c_str() );
			if ( ann->value )
					s+= string(ann->value->GetRuntimeClass()->className);
			else	s+= "(NULL)";

			st << s;

			if ( ann->name == ARC_ANNOTATION_DELTA_SRC_POSEID )
			{
				TPoseID refID;
				(*it)->m_annotations.getElemental(ARC_ANNOTATION_DELTA_SRC_POSEID, refID, ann->ID );
				st << format("     VALUE: %i",(int)refID);
			}
			else if ( ann->name == ARC_ANNOTATION_DELTA_TRG_POSEID )
			{
				TPoseID refID;
				(*it)->m_annotations.getElemental(ARC_ANNOTATION_DELTA_TRG_POSEID, refID, ann->ID );
				st << format("     VALUE: %i",(int)refID);
			}
			else if ( ann->name == ARC_ANNOTATION_DELTA )
			{
				CSerializablePtr o = (*it)->m_annotations.get(ARC_ANNOTATION_DELTA, ann->ID );
				ASSERT_(o);

				CPose3DPDFGaussian relativePoseAcordToArc;
				relativePoseAcordToArc.copyFrom(*CPose3DPDFPtr(o));

				st << format("     VALUE: (%f,%f,%f , %fdeg,%fdeg,%fdeg)",
					relativePoseAcordToArc.mean.x(),
					relativePoseAcordToArc.mean.y(),
					relativePoseAcordToArc.mean.z(),
					RAD2DEG( relativePoseAcordToArc.mean.yaw() ),
					RAD2DEG( relativePoseAcordToArc.mean.pitch() ),
					RAD2DEG( relativePoseAcordToArc.mean.roll() ) );
			}
		}

		st << "";
	}
*/
}

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precomp header

#include <mrpt/core/aligned_allocator.h>
#include <mrpt/graphslam/levmarq.h>
#include <mrpt/hmtslam/CRobotPosesGraph.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/random.h>
#include <mrpt/system/os.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::random;
using namespace mrpt::hmtslam;
using namespace mrpt::system;
using namespace mrpt::serialization;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::maps;
using namespace mrpt::math;

/*---------------------------------------------------------------
						nodeCount
  ---------------------------------------------------------------*/
size_t CHierarchicalMapMHPartition::nodeCount() const { return m_nodes.size(); }
/*---------------------------------------------------------------
						arcCount
  ---------------------------------------------------------------*/
size_t CHierarchicalMapMHPartition::arcCount() const { return m_arcs.size(); }
/*---------------------------------------------------------------
						getNodeByID
  ---------------------------------------------------------------*/
CHMHMapNode::Ptr CHierarchicalMapMHPartition::getNodeByID(
	CHMHMapNode::TNodeID id)
{
	MRPT_START
	if (id == AREAID_INVALID) return CHMHMapNode::Ptr();

	auto it = m_nodes.find(id);
	return it == m_nodes.end() ? CHMHMapNode::Ptr() : it->second;

	MRPT_END
}
/*---------------------------------------------------------------
						getNodeByID
  ---------------------------------------------------------------*/
const CHMHMapNode::Ptr CHierarchicalMapMHPartition::getNodeByID(
	CHMHMapNode::TNodeID id) const
{
	MRPT_START
	if (id == AREAID_INVALID) return CHMHMapNode::Ptr();

	auto it = m_nodes.find(id);
	return it == m_nodes.end() ? CHMHMapNode::Ptr() : it->second;

	MRPT_END
}

/*---------------------------------------------------------------
						getNodeByLabel
  ---------------------------------------------------------------*/
CHMHMapNode::Ptr CHierarchicalMapMHPartition::getNodeByLabel(
	const std::string& label, const THypothesisID& hypothesisID)
{
	MRPT_START

	// Look for the ID:
	for (auto it = m_nodes.begin(); it != m_nodes.end(); ++it)
		if (it->second->m_hypotheses.has(hypothesisID))
			if (!os::_strcmpi(it->second->m_label.c_str(), label.c_str()))
				return it->second;

	// Not found:
	return CHMHMapNode::Ptr();

	MRPT_END
}
/*---------------------------------------------------------------
						getNodeByLabel
  ---------------------------------------------------------------*/
const CHMHMapNode::Ptr CHierarchicalMapMHPartition::getNodeByLabel(
	const std::string& label, const THypothesisID& hypothesisID) const
{
	MRPT_START

	// Look for the ID:
	for (const auto& m_node : m_nodes)
		if (m_node.second->m_hypotheses.has(hypothesisID))
			if (!os::_strcmpi(m_node.second->m_label.c_str(), label.c_str()))
				return m_node.second;

	// Not found:
	return CHMHMapNode::Ptr();

	MRPT_END
}

/*---------------------------------------------------------------
					getFirstNode
  ---------------------------------------------------------------*/
CHMHMapNode::Ptr CHierarchicalMapMHPartition::getFirstNode()
{
	if (m_nodes.empty())
		return CHMHMapNode::Ptr();
	else
		return (m_nodes.begin())->second;
}

/*---------------------------------------------------------------
					saveAreasDiagramForMATLAB
  ---------------------------------------------------------------*/
void CHierarchicalMapMHPartition::saveAreasDiagramForMATLAB(
	[[maybe_unused]] const std::string& filName,
	[[maybe_unused]] const CHMHMapNode::TNodeID& idReferenceNode,
	[[maybe_unused]] const THypothesisID& hypothesisID) const
{
	/*
		MRPT_START
		unsigned int	nAreaNodes=0;

		const CHMHMapNode	*refNode = getNodeByID( idReferenceNode );
		ASSERT_(refNode!=nullptr);
		ASSERT_(refNode->nodeType.isType("Area"));

		FILE	*f=os::fopen(filName.c_str(),"wt");
		if (!f) THROW_EXCEPTION("Can not open output file!");

		// The list of all the m_nodes to be plotted:
		// --------------------------------------------
		std::map<CHMHMapNode::TNodeID,CPose2D>			nodesPoses;
	   // The ref. pose of each area
		std::map<CHMHMapNode::TNodeID,CPose2D>			nodesMeanPoses;
	   // The mean pose of the observations in the area
		std::map<CHMHMapNode::TNodeID,CPose2D>::iterator	it;

		// First, insert the reference node:
		nodesPoses[refNode->ID] = CPose2D(0,0,0);

		// Find the rest of "Area" m_nodes:
		for (unsigned int i=0;i<nodeCount();i++)
		{
			const CHMHMapNode	*nod = getNodeByIndex(i);

			// Is this a Area node?
			if (nod->nodeType.isType("Area"))
			{
				nAreaNodes++;	// Counter
				if (nod!=refNode)
				{
					CPosePDFParticles	posePDF;

					computeCoordinatesTransformationBetweenNodes(
									refNode->ID,
									nod->ID,
									posePDF,
									hypothesisID,
									100);

					nodesPoses[nod->ID] = posePDF.getEstimatedPose();
				}
			}
		} // end for each node "i"

		// Assure that all area m_nodes have been localized:
		ASSERT_(nAreaNodes == nodesPoses.size() );


		// Find the mean of the observations in each area:
		for (it=nodesPoses.begin();it!=nodesPoses.end();it++)
		{
			CPose3D		meanPose = it->second;
			const CHMHMapNode	*node = getNodeByID( it->first );

			CSimpleMap	*localizedSFs = (CSimpleMap*)
	   node->annotations.get("localizedObservations");

			if (localizedSFs->size())
			{
				// Compute the mean pose:
				CPose3D		meanSFs(0,0,0);

				for (unsigned int k=0;k<localizedSFs->size();k++)
				{
					CPose3DPDF			*pdf;
					CSensoryFrame		*dummy_sf;
					CPose3D				pdfMean;

					localizedSFs->get(k,pdf,dummy_sf);

					pdfMean = pdf->getEstimatedPose();
					meanSFs.addComponents( pdfMean );
				}
				meanSFs *= 1.0f/(localizedSFs->size());
				meanSFs.normalizeAngles();

				meanPose = meanPose + meanSFs;
			}
			else
			{
				// Let the ref. pose to represent the node
			}

			nodesMeanPoses[it->first] = meanPose;
		}

		// --------------------------------------------------------------
		//  Now we have the global poses of all the m_nodes: Draw them!
		// -------------------------------------------------------------
		// Header:
		os::fprintf(f,"%%-------------------------------------------------------\n");
		os::fprintf(f,"%% File automatically generated using the MRPT
	   method:\n");
		os::fprintf(f,"%%'CHierarchicalMapMHPartition::saveAreasDiagramForMATLAB'\n");
		os::fprintf(f,"%%\n");
		os::fprintf(f,"%%                        ~ MRPT ~\n");
		os::fprintf(f,"%%  Jose Luis Blanco Claraco, University of Malaga @
	   2006\n");
		os::fprintf(f,"%%  http://www.isa.uma.es/ \n");
		os::fprintf(f,"%%-------------------------------------------------------\n\n");

		os::fprintf(f,"hold on;\n");

		float	nodesRadius = 1.5f;

		// Draw the m_nodes:
		// ----------------------
		for (it = nodesMeanPoses.begin();it!=nodesMeanPoses.end();it++)
		{
			const CHMHMapNode	*node = getNodeByID( it->first );
			CPose2D		pose( it->second );

			os::fprintf(f,"\n%% Node: %s\n", node->label.c_str() );
			os::fprintf(f,"rectangle('Curvature',[1 1],'Position',[%f %f %f
	   %f],'FaceColor',[1 1 1]);\n",
							pose.x - nodesRadius,
							pose.y - nodesRadius,
							2*nodesRadius,
							2*nodesRadius);

			// Draw the node's label:
			os::fprintf(f,"text(%f,%f,'%s','FontSize',7,'HorizontalAlignment','center','Interpreter','none');\n",
							pose.x,
							pose.y,
							node->label.c_str() );

			// And their m_arcs:
			// ----------------------
			for (unsigned int a=0;a<node->m_arcs.size();a++)
			{
				CHMHMapArc	*arc = node->m_arcs[a];
				if (arc->nodeFrom==node->ID)
				{
					CPose2D		poseTo(
	   nodesMeanPoses.find(arc->nodeTo)->second );
					float		x1,x2,y1,y2, Ax,Ay;

					// Compute a unitary direction vector:
					Ax = poseTo.x - pose.x;
					Ay = poseTo.y - pose.y;

					float	len = sqrt(square(Ax)+square(Ay));

					if (len>nodesRadius)
					{
						Ax /= len;
						Ay /= len;

						x1 = pose.x + Ax * nodesRadius;
						y1 = pose.y + Ay * nodesRadius;

						x2 = pose.x + Ax * (len-nodesRadius);
						y2 = pose.y + Ay * (len-nodesRadius);

						os::fprintf(f,"line([%f %f],[%f,%f]);\n", x1,x2,y1,y2);
					}
				}
			}
		}

		os::fprintf(f,"axis equal; zoom on;");
		os::fclose(f);
		MRPT_END
	*/
}

/*---------------------------------------------------------------
				saveAreasDiagramWithEllipsedForMATLAB
  ---------------------------------------------------------------*/
void CHierarchicalMapMHPartition::saveAreasDiagramWithEllipsedForMATLAB(
	[[maybe_unused]] const std::string& filName,
	[[maybe_unused]] const CHMHMapNode::TNodeID& idReferenceNode,
	[[maybe_unused]] const THypothesisID& hypothesisID,
	[[maybe_unused]] float uncertaintyExagerationFactor,
	[[maybe_unused]] bool drawArcs,
	[[maybe_unused]] unsigned int numberOfIterationsForOptimalGlobalPoses) const
{
	/*	MRPT_START

		const CHMHMapNode	*refNode = getNodeByID( idReferenceNode );
		ASSERT_(refNode!=nullptr);
		ASSERT_(refNode->nodeType.isType("Area"));

		FILE	*f=os::fopen(filName.c_str(),"wt");
		if (!f) THROW_EXCEPTION("Can not open output file!");

		// The list of all the m_nodes to be plotted:
		// --------------------------------------------
		std::map<CHMHMapNode::TNodeID,CPosePDFGaussian>	nodesPoses;
	   // The ref. pose of each area
		std::map<CHMHMapNode::TNodeID,CPosePDFGaussian>::iterator	it;
		std::map<CHMHMapNode::TNodeID,CPosePDFGaussian>	nodesMeanPoses;
	   // The mean pose of the observations in the area
		std::map<CHMHMapNode::TNodeID,CPose2D>::iterator	it2;

		computeGloballyConsistentNodeCoordinates( nodesPoses, idReferenceNode,
	   numberOfIterationsForOptimalGlobalPoses );


		// Find the mean of the observations in each area:
		for (it=nodesPoses.begin();it!=nodesPoses.end();it++)
		{
			//CPosePDFGaussian		posePDF = it->second;
			CPose2D					meanPose = it->second.mean;

			const CHMHMapNode			*node = getNodeByID( it->first );

			CSimpleMap	*localizedSFs = (CSimpleMap*)
	   node->annotations.get("localizedObservations");

			if (localizedSFs->size())
			{
				// Compute the mean pose:
				CPose3D		meanSFs(0,0,0);

				for (unsigned int k=0;k<localizedSFs->size();k++)
				{
					CPose3DPDF			*pdf;
					CSensoryFrame		*dummy_sf;
					CPose3D				pdfMean;

					localizedSFs->get(k,pdf,dummy_sf);

					pdfMean = pdf->getEstimatedPose();
					meanSFs.addComponents( pdfMean );
				}
				meanSFs *= 1.0f/(localizedSFs->size());
				meanSFs.normalizeAngles();

				meanPose = meanPose + meanSFs;
			}
			else
			{
				// Let the ref. pose to represent the node
			}
			nodesMeanPoses[it->first].mean = meanPose;
			nodesMeanPoses[it->first].cov  = it->second.cov;
		}

		// --------------------------------------------------------------
		//  Now we have the global poses of all the m_nodes: Draw them!
		// -------------------------------------------------------------
		// Header:
		os::fprintf(f,"%%-------------------------------------------------------\n");
		os::fprintf(f,"%% File automatically generated using the MRPT
	   method:\n");
		os::fprintf(f,"%%'CHierarchicalMapMHPartition::saveAreasDiagramWithEllipsedForMATLAB'\n");
		os::fprintf(f,"%%\n");
		os::fprintf(f,"%%                        ~ MRPT ~\n");
		os::fprintf(f,"%%  Jose Luis Blanco Claraco, University of Malaga @
	   2006\n");
		os::fprintf(f,"%%  http://www.isa.uma.es/ \n");
		os::fprintf(f,"%%-------------------------------------------------------\n\n");

		os::fprintf(f,"hold on;\n");

		//float	nodesRadius = 0; //2.0f;

		// Draw the m_nodes:
		// ----------------------
		for (it = nodesMeanPoses.begin();it!=nodesMeanPoses.end();it++)
		{
			const CHMHMapNode			*node = getNodeByID( it->first );
			CPosePDFGaussian	posePDF = it->second;
			CMatrixF				C( posePDF.cov );
			CPose2D				pose( posePDF.mean );

			if (C.det()==0)
			{
				C.unit();
				C(0,0)=1e-4f;
				C(1,1)=1e-4f;
			}

			os::fprintf(f,"\n%% Node: %s\n", node->label.c_str() );
			os::fprintf(f,"c=[%e %e;%e %e];m=[%f
	   %f];error_ellipse(%f*c,m,'conf',0.95);\n",
					C(0,0), C(0,1),
					C(1,0), C(1,1),
					pose.x,pose.y,
					square(uncertaintyExagerationFactor) );

			// Draw the node's label:
			os::fprintf(f,"text(%f,%f,'
	   %s','FontSize',8,'HorizontalAlignment','center','Interpreter','none');\n",
							pose.x+1.5,
							pose.y+1.5,
							node->label.c_str() );
		}

		// The m_arcs:
		// ----------------------
		if ( drawArcs )
		for (it = nodesMeanPoses.begin();it!=nodesMeanPoses.end();it++)
		{
			const CHMHMapNode		*node = getNodeByID( it->first );
			CPose2D				pose( it->second.mean );

			float nodesRadius = 0;
			for (unsigned int a=0;a<node->m_arcs.size();a++)
			{
				CHMHMapArc	*arc = node->m_arcs[a];
				if (arc->nodeFrom==node->ID)
				{
					CPose2D		poseTo(
	   nodesMeanPoses.find(arc->nodeTo)->second.mean );
					float		x1,x2,y1,y2, Ax,Ay;

					// Compute a unitary direction vector:
					Ax = poseTo.x - pose.x;
					Ay = poseTo.y - pose.y;

					float	len = sqrt(square(Ax)+square(Ay));

					if (len>nodesRadius)
					{
						Ax /= len;
						Ay /= len;

						x1 = pose.x + Ax * nodesRadius;
						y1 = pose.y + Ay * nodesRadius;

						x2 = pose.x + Ax * (len-nodesRadius);
						y2 = pose.y + Ay * (len-nodesRadius);

						os::fprintf(f,"line([%f %f],[%f,%f]);\n", x1,x2,y1,y2);
					}
				}
			}
		}

		os::fprintf(f,"axis equal; zoom on;");

		os::fclose(f);

		// Free memory:
		nodesPoses.clear();

		MRPT_END
		*/
}

/*---------------------------------------------------------------
					saveGlobalMapForMATLAB
  ---------------------------------------------------------------*/
void CHierarchicalMapMHPartition::saveGlobalMapForMATLAB(
	[[maybe_unused]] const std::string& filName,
	[[maybe_unused]] const THypothesisID& hypothesisID,
	[[maybe_unused]] const CHMHMapNode::TNodeID& idReferenceNode) const
{
	/*
		unsigned int	nAreaNodes=0;

		const CHMHMapNode	*refNode = getNodeByID( idReferenceNode );
		ASSERT_(refNode!=nullptr);
		ASSERT_(refNode->nodeType.isType("Area"));

		FILE	*f=os::fopen(filName.c_str(),"wt");
		if (!f) THROW_EXCEPTION("Can not open output file!");

		// The list of all the m_nodes to be plotted:
		// --------------------------------------------
		std::map<CHMHMapNode::TNodeID,CPose2D>			nodesPoses;
	// The ref. pose of each area
		std::map<CHMHMapNode::TNodeID,CPose2D>			nodesMeanPoses;
	// The mean pose of the observations in the area
		std::map<CHMHMapNode::TNodeID,CPose2D>::iterator	it;

		// First, insert the reference node:
		nodesPoses[refNode->ID] = CPose2D(0,0,0);

		// Find the rest of "Area" m_nodes:
		for (unsigned int i=0;i<nodeCount();i++)
		{
			const CHMHMapNode	*nod = getNodeByIndex(i);

			// Is this a Area node?
			if (nod->nodeType.isType("Area"))
			{
				nAreaNodes++;	// Counter
				if (nod!=refNode)
				{
					CPosePDFParticles	posePDF;

					computeCoordinatesTransformationBetweenNodes(
									refNode->ID,
									nod->ID,
									posePDF,
									hypothesisID,
									100);

					nodesPoses[nod->ID] = posePDF.getEstimatedPose();
				}
			}
		} // end for each node "i"

		// Assure that all area m_nodes have been localized:
		ASSERT_(nAreaNodes == nodesPoses.size() );

		// Find the mean of the observations in each area:
		for (it=nodesPoses.begin();it!=nodesPoses.end();it++)
		{
			CPose2D		meanPose = it->second;
			const CHMHMapNode	*node = getNodeByID( it->first );

			CSimpleMap	*localizedSFs = (CSimpleMap*)
	node->annotations.get("localizedObservations");

			if (localizedSFs->size())
			{
				// Compute the mean pose:
				CPose3D		meanSFs(0,0,0);

				for (unsigned int k=0;k<localizedSFs->size();k++)
				{
					CPose3DPDF			*pdf;
					CSensoryFrame		*dummy_sf;
					CPose3D				pdfMean;

					localizedSFs->get(k,pdf,dummy_sf);

					pdfMean = pdf->getEstimatedPose();
					meanSFs.addComponents( pdfMean );
				}
				meanSFs *= 1.0f/(localizedSFs->size());
				meanSFs.normalizeAngles();

				meanPose = meanPose + meanSFs;
			}
			else
			{
				// Let the ref. pose to represent the node
			}

			nodesMeanPoses[it->first] = meanPose;
		}

		// --------------------------------------------------------------
		//  Now we have the global poses of all the m_nodes: Draw them!
		// -------------------------------------------------------------
		// Header:
		os::fprintf(f,"%%-------------------------------------------------------\n");
		os::fprintf(f,"%% File automatically generated using the MRPT
	method:\n");
		os::fprintf(f,"%%'CHierarchicalMapMHPartition::saveAreasDiagramForMATLAB'\n");
		os::fprintf(f,"%%\n");
		os::fprintf(f,"%%                        ~ MRPT ~\n");
		os::fprintf(f,"%%  Jose Luis Blanco Claraco, University of Malaga @
	2006\n");
		os::fprintf(f,"%%  http://www.isa.uma.es/ \n");
		os::fprintf(f,"%%-------------------------------------------------------\n\n");

		os::fprintf(f,"hold on;\n");

		float	nodesRadius = 1.5f;

		// Draw the metric maps of each area:
		for (it = nodesPoses.begin();it!=nodesPoses.end();it++)
		{
			CPose2D		meanPose = it->second;
			const CHMHMapNode	*node = getNodeByID( it->first );

			CMultiMetricMap		*metricMap = (CMultiMetricMap*)
	node->annotations.get("hybridMetricMap");
	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
			ASSERT_(metricMap!=nullptr);
			ASSERT_(metricMap->m_pointsMap);
	#endif
			int			n;
			float		*xs,*ys,*zs;
			CPoint2D	pL,pG;

			metricMap->m_pointsMap->getPointsBuffer(n,xs,ys,zs);

			os::fprintf(f,"\n%% Metric map for node: %s\n", node->label.c_str()
	);

			if (n)
			{
				os::fprintf(f,"map=[" );
				for (int j=0;j<n;j+=10)
				{
					if (j!=0)	os::fprintf(f,";" );
					// Local coords:
					pL.x = xs[j];
					pL.y = ys[j];

					// Global coords:
					pG = meanPose + pL;

					// write:
					os::fprintf(f,"%.03f %.03f",pG.x,pG.y );
				}
				os::fprintf(f,"];\n" );
				os::fprintf(f,"plot(map(:,1),map(:,2),'.k','MarkerSize',3);\n"
	);
			}

		}


		// Draw the m_nodes:
		// ----------------------
		for (it = nodesMeanPoses.begin();it!=nodesMeanPoses.end();it++)
		{
			const CHMHMapNode	*node = getNodeByID( it->first );
			CPose2D		pose( it->second );

			os::fprintf(f,"\n%% Node: %s\n", node->label.c_str() );
			os::fprintf(f,"rectangle('Curvature',[1 1],'Position',[%f %f %f
	%f],'FaceColor',[1 1 1]);\n",
							pose.x - nodesRadius,
							pose.y - nodesRadius,
							2*nodesRadius,
							2*nodesRadius);

			// Draw the node's label:
			os::fprintf(f,"text(%f,%f,'%s','FontSize',7,'HorizontalAlignment','center','Interpreter','none');\n",
							pose.x,
							pose.y,
							node->label.c_str() );

			// And their m_arcs:
			// ----------------------
			for (unsigned int a=0;a<node->m_arcs.size();a++)
			{
				CHMHMapArc	*arc = node->m_arcs[a];
				if (arc->nodeFrom==node->ID)
				{
					CPose2D		poseTo(
	nodesMeanPoses.find(arc->nodeTo)->second );
					float		x1,x2,y1,y2, Ax,Ay;

					// Compute a unitary direction vector:
					Ax = poseTo.x - pose.x;
					Ay = poseTo.y - pose.y;

					float	len = sqrt(square(Ax)+square(Ay));

					if (len>nodesRadius)
					{
						Ax /= len;
						Ay /= len;

						x1 = pose.x + Ax * nodesRadius;
						y1 = pose.y + Ay * nodesRadius;

						x2 = pose.x + Ax * (len-nodesRadius);
						y2 = pose.y + Ay * (len-nodesRadius);

						os::fprintf(f,"line([%f %f],[%f,%f]);\n", x1,x2,y1,y2);
					}

				}
			}

		}



		os::fprintf(f,"axis equal; zoom on;");

		os::fclose(f);
	MRPT_END
	*/
}

// Variables:
struct TDistance
{
	TDistance() : dist(std::numeric_limits<unsigned>::max()) {}
	TDistance(const unsigned& D) : dist(D) {}
	const TDistance& operator=(const unsigned& D)
	{
		dist = D;
		return *this;
	}

	unsigned dist;
};

struct TPrevious
{
	TPrevious() : id(AREAID_INVALID) {}
	CHMHMapNode::TNodeID id;
};

/*---------------------------------------------------------------
					findPathBetweenNodes
  ---------------------------------------------------------------*/
void CHierarchicalMapMHPartition::findPathBetweenNodes(
	const CHMHMapNode::TNodeID& source, const CHMHMapNode::TNodeID& target,
	const THypothesisID& hypothesisID, TArcList& ret, bool direction) const
{
	MRPT_START

	/*
	1  function Dijkstra(G, w, s)
	2     for each vertex v in V[G]                        // Initializations
	3           d[v] := infinity
	4           previous[v] := undefined
	5     d[s] := 0
	6     S := empty set
	7     Q := V[G]
	8     while Q is not an empty set                      // The algorithm
	itself
	9           u := Extract_Min(Q)
								------>	(u=t)? -> end!
	10           S := S union {u}
	11           for each edge (u,v) outgoing from u
	12                  if d[u] + w(u,v) < d[v]             // Relax (u,v)
	13                        d[v] := d[u] + w(u,v)
	14                        previous[v] := u
	*/

	map<CHMHMapNode::TNodeID, TDistance> d;  // distance
	map<CHMHMapNode::TNodeID, TPrevious> previous;
	map<CHMHMapNode::TNodeID, CHMHMapArc::Ptr> previous_arcs;
	map<CHMHMapNode::TNodeID, bool> visited;

	unsigned visitedCount = 0;

	ASSERTMSG_(
		m_nodes.find(source) != m_nodes.end(),
		format("Source node %u not found in the H-Map", (unsigned int)source));
	ASSERTMSG_(
		m_nodes.find(target) != m_nodes.end(),
		format("Target node %u not found in the H-Map", (unsigned int)target));

	ASSERT_(m_nodes.find(source)->second->m_hypotheses.has(hypothesisID));
	ASSERT_(m_nodes.find(target)->second->m_hypotheses.has(hypothesisID));

	// Init:
	// d: already initialized to infinity by default.
	// previous: idem
	// previous_arcs: idem
	// visited: idem

	d[source] = 0;

	TNodeList::const_iterator u;

	// The algorithm:
	do
	{
		// Finds the index of the minimum:
		// unsigned int	i;
		unsigned min_d = std::numeric_limits<unsigned>::max();

		u = m_nodes.end();

		for (auto i = m_nodes.begin(); i != m_nodes.end(); ++i)
		{
			if (i->second->m_hypotheses.has(hypothesisID))
			{
				if (d[i->first].dist < min_d && !visited[i->first])
				{
					u = i;
					min_d = d[u->first].dist;
				}
			}
		}

		ASSERT_(u != m_nodes.end());

		visited[u->first] = true;
		visitedCount++;

		// For each arc from "u":
		const CHMHMapNode::Ptr nodeU = getNodeByID(u->first);
		TArcList arcs;
		nodeU->getArcs(arcs, hypothesisID);
		for (auto i = arcs.begin(); i != arcs.end(); ++i)
		{
			CHMHMapNode::TNodeID vID;
			if (!direction)
			{
				if ((*i)->getNodeFrom() != nodeU->getID())
					vID = (*i)->getNodeFrom();
				else
					vID = (*i)->getNodeTo();
			}
			else
			{
				if ((*i)->getNodeFrom() != nodeU->getID())
					continue;
				else
					vID = (*i)->getNodeTo();
			}

			if ((min_d + 1) < d[vID].dist)
			{
				d[vID].dist = min_d + 1;
				previous[vID].id = u->first;
				previous_arcs[vID] = *i;
			}
		}
	} while (u->first != target && visitedCount < m_nodes.size());

	// Arrived to target?
	ret.clear();

	if (u->first == target)
	{
		// Path found:
		CHMHMapNode::TNodeID nod = target;
		do
		{
			ret.push_front(previous_arcs[nod]);
			nod = previous[nod].id;
		} while (nod != source);
	}

	MRPT_END
}

/*---------------------------------------------------------------
		computeCoordinatesTransformationBetweenNodes
  ---------------------------------------------------------------*/
void CHierarchicalMapMHPartition::computeCoordinatesTransformationBetweenNodes(
	const CHMHMapNode::TNodeID& nodeFrom, const CHMHMapNode::TNodeID& nodeTo,
	CPose3DPDFParticles& posePDF, const THypothesisID& hypothesisID,
	unsigned int particlesCount, float additionalNoiseXYratio,
	float additionalNoisePhiRad) const
{
	MRPT_START

	TArcList arcsPath;
	TArcList::const_iterator arcsIt;
	const TPose3D nullPose(0, 0, 0, 0, 0, 0);
	CHMHMapNode::TNodeID lastNode, nextNode;
	size_t pathLength;

	using TPose3DList = std::vector<CPose3D>;
	std::vector<TPose3DList> listSamples;
	std::vector<TPose3DList>::iterator lstIt;
	TPose3DList dummyList;
	TPose3DList::iterator poseIt;

	// Find the shortest sequence of arcs connecting the nodes:
	findPathBetweenNodes(nodeFrom, nodeTo, hypothesisID, arcsPath);
	ASSERT_(arcsPath.size());
	pathLength = arcsPath.size();

	// Prepare the PDF:
	posePDF.resetDeterministic(nullPose, particlesCount);

	// Draw the pose sample:
	lastNode = nodeFrom;
	dummyList.resize(particlesCount);
	listSamples.resize(pathLength, dummyList);
	for (arcsIt = arcsPath.begin(), lstIt = listSamples.begin();
		 arcsIt != arcsPath.end(); lstIt++, arcsIt++)
	{
		// Flag for the pose PDF needing to be reversed:
		bool reversedArc = (*arcsIt)->getNodeTo() == lastNode;
		nextNode =
			reversedArc ? (*arcsIt)->getNodeFrom() : (*arcsIt)->getNodeTo();

		// Check reference pose IDs between arcs:
		// ------------------------------------------------
		TPoseID curNodeRefPoseID, nextNodeRefPoseID;
		getNodeByID((*arcsIt)->getNodeFrom())
			->m_annotations.getElemental(
				NODE_ANNOTATION_REF_POSEID, curNodeRefPoseID, hypothesisID,
				true);
		getNodeByID((*arcsIt)->getNodeTo())
			->m_annotations.getElemental(
				NODE_ANNOTATION_REF_POSEID, nextNodeRefPoseID, hypothesisID,
				true);

		TPoseID srcRefPoseID, trgRefPoseID;
		(*arcsIt)->m_annotations.getElemental(
			ARC_ANNOTATION_DELTA_SRC_POSEID, srcRefPoseID, hypothesisID, true);
		(*arcsIt)->m_annotations.getElemental(
			ARC_ANNOTATION_DELTA_TRG_POSEID, trgRefPoseID, hypothesisID, true);

		ASSERT_(curNodeRefPoseID == srcRefPoseID);
		ASSERT_(nextNodeRefPoseID == trgRefPoseID);

		// Get the pose PDF:
		CSerializable::Ptr anotation =
			(*arcsIt)->m_annotations.get(ARC_ANNOTATION_DELTA, hypothesisID);
		ASSERT_(anotation);

		CPose3DPDFGaussian pdf;  // Convert to gaussian
		pdf.copyFrom(*std::dynamic_pointer_cast<CPose3DPDF>(anotation));

		vector<CVectorDouble> samples;

		pdf.drawManySamples(lstIt->size(), samples);
		ASSERT_(samples.size() == lstIt->size());
		ASSERT_(samples[0].size() == 6);

		vector<CVectorDouble>::const_iterator samplIt;
		for (poseIt = lstIt->begin(), samplIt = samples.begin();
			 poseIt != lstIt->end(); poseIt++, samplIt++)
		{
			// Minimum added noise:
			poseIt->setFromValues(
				(*samplIt)[0] +
					additionalNoiseXYratio *
						getRandomGenerator().drawGaussian1D_normalized(),
				(*samplIt)[1] +
					additionalNoiseXYratio *
						getRandomGenerator().drawGaussian1D_normalized(),
				(*samplIt)[2],
				(*samplIt)[3] +
					additionalNoisePhiRad *
						getRandomGenerator().drawGaussian1D_normalized(),
				(*samplIt)[4], (*samplIt)[5]);

			// Pose composition:
			if (reversedArc) *poseIt = (CPose3D() - CPose3D(*poseIt));
		}

		// for the next iter:
		lastNode = nextNode;
	}

	// Compute the pose composition:
	for (unsigned int i = 0; i < particlesCount; i++)
	{
		CPose3D sample = CPose3D(posePDF.m_particles[i].d);

		for (unsigned int j = 0; j < pathLength; j++)
		{
			if (j)
				sample = sample + listSamples[j][i];
			else
				sample = listSamples[j][i];
		}
	}

	posePDF.normalizeWeights();

#if 0
	CPose3DPDFGaussian	auxPDF;
	auxPDF.copyFrom( posePDF );
	cout << "[computeCoordinatesTransformationBetweenNodes] Nodes: " << nodeFrom << " - " << nodeTo << ": " << auxPDF;
#endif

	MRPT_END
}

/*---------------------------------------------------------------
			computeMatchProbabilityBetweenNodes
  ---------------------------------------------------------------*/
float CHierarchicalMapMHPartition::computeMatchProbabilityBetweenNodes(
	[[maybe_unused]] const CHMHMapNode::TNodeID& nodeFrom,
	[[maybe_unused]] const CHMHMapNode::TNodeID& nodeTo,
	[[maybe_unused]] float& maxMatchProb,
	[[maybe_unused]] CPose3DPDFSOG& estimatedRelativePose,
	[[maybe_unused]] const THypothesisID& hypothesisID,
	[[maybe_unused]] unsigned int monteCarloSamplesPose)
{
	MRPT_START
	THROW_EXCEPTION("TO DO");
	MRPT_END
}

/*---------------------------------------------------------------
					findArcsBetweenNodes
  ---------------------------------------------------------------*/
void CHierarchicalMapMHPartition::findArcsBetweenNodes(
	const CHMHMapNode::TNodeID& node1id, const CHMHMapNode::TNodeID& node2id,
	const THypothesisID& hypothesisID, TArcList& ret) const
{
	MRPT_START

	ret.clear();
	const CHMHMapNode::Ptr node1 = getNodeByID(node1id);

	TArcList lstArcs;
	TArcList::const_iterator itArc;

	node1->getArcs(lstArcs);
	for (itArc = lstArcs.begin(); itArc != lstArcs.end(); itArc++)
	{
		if ((*itArc)->m_hypotheses.has(hypothesisID))
			if ((*itArc)->m_nodeFrom == node2id ||
				(*itArc)->m_nodeTo == node2id)
			{
				ret.push_back(*itArc);
			}
	}

	MRPT_END
}

/*---------------------------------------------------------------
					findArcsOfTypeBetweenNodes
  ---------------------------------------------------------------*/
void CHierarchicalMapMHPartition::findArcsOfTypeBetweenNodes(
	const CHMHMapNode::TNodeID& node1id, const CHMHMapNode::TNodeID& node2id,
	const THypothesisID& hypothesisID, const std::string& arcType,
	TArcList& ret) const
{
	MRPT_START

	ret.clear();
	const CHMHMapNode::Ptr node1 = getNodeByID(node1id);

	TArcList lstArcs;
	TArcList::const_iterator itArc;

	node1->getArcs(lstArcs);
	for (itArc = lstArcs.begin(); itArc != lstArcs.end(); itArc++)
	{
		if ((*itArc)->m_hypotheses.has(hypothesisID))
			if ((*itArc)->m_nodeFrom == node2id ||
				(*itArc)->m_nodeTo == node2id)
			{
				if ((*itArc)->m_arcType == arcType) ret.push_back(*itArc);
			}
	}

	MRPT_END
}

/*---------------------------------------------------------------
					areNodesNeightbour
  ---------------------------------------------------------------*/
bool CHierarchicalMapMHPartition::areNodesNeightbour(
	const CHMHMapNode::TNodeID& node1id, const CHMHMapNode::TNodeID& node2id,
	const THypothesisID& hypothesisID, const char* requiredAnnotation) const
{
	MRPT_START

	const CHMHMapNode::Ptr node1 = getNodeByID(node1id);

	TArcList lstArcs;
	TArcList::const_iterator itArc;

	node1->getArcs(lstArcs);
	for (itArc = lstArcs.begin(); itArc != lstArcs.end(); itArc++)
	{
		if ((*itArc)->m_hypotheses.has(hypothesisID))
			if ((*itArc)->m_nodeFrom == node2id ||
				(*itArc)->m_nodeTo == node2id)
			{
				if (!requiredAnnotation)
					return true;
				else if ((*itArc)->m_annotations.get(
							 requiredAnnotation, hypothesisID))
					return true;
			}
	}

	return false;

	MRPT_END
}

/*---------------------------------------------------------------
						getAs3DScene
  ---------------------------------------------------------------*/

void CHierarchicalMapMHPartition::getAs3DScene(
	COpenGLScene& outScene, const CHMHMapNode::TNodeID& idReferenceNode,
	const THypothesisID& hypothesisID,
	const unsigned int& numberOfIterationsForOptimalGlobalPoses,
	bool showRobotPoseIDs) const
{
	MRPT_START

	// First: Clear and add the cool "ground grid" :-P
	outScene.clear();
	{
		mrpt::opengl::CGridPlaneXY::Ptr obj =
			mrpt::opengl::CGridPlaneXY::Create(-500, 500, -500, 500, 0, 5);
		obj->setColor(0.3f, 0.3f, 0.3f);
		outScene.insert(obj);
	}

	using TMapID2PosePDF = std::map<CHMHMapNode::TNodeID, CPose3DPDFGaussian>;
	// The ref. pose of each area
	TMapID2PosePDF nodesPoses;
	TMapID2PosePDF::iterator it;

	using TMapID2Pose2D = std::map<CHMHMapNode::TNodeID, CPose2D>;
	// The mean pose of the observations in the area
	TMapID2Pose2D nodesMeanPoses;
	TMapID2Pose2D::iterator it2;

	// Only those nodes in the "hypothesisID" are computed.
	computeGloballyConsistentNodeCoordinates(
		nodesPoses, idReferenceNode, hypothesisID,
		numberOfIterationsForOptimalGlobalPoses);

	// Find the mean of the observations in each area:
	for (it = nodesPoses.begin(); it != nodesPoses.end(); it++)
	{
		CPose2D meanPose = CPose2D(it->second.mean);
		const CHMHMapNode::Ptr node = getNodeByID(it->first);

		CRobotPosesGraph::Ptr posesGraph =
			node->m_annotations.getAs<CRobotPosesGraph>(
				NODE_ANNOTATION_POSES_GRAPH, hypothesisID);

		if (posesGraph && posesGraph->size())
		{
			// Compute the mean pose:
			CPose3D meanSFs(0, 0, 0);

			for (auto n : *posesGraph)
				meanSFs.addComponents(n.second.pdf.getMeanVal());

			meanSFs *= 1.0f / (posesGraph->size());
			meanSFs.normalizeAngles();

			meanPose = meanPose + CPose2D(meanSFs);
		}
		else
		{
			// Let the ref. pose to represent the node
		}

		nodesMeanPoses[it->first] = meanPose;
	}

	// ----------------------------------------------------------------
	//  Now we have the global poses of all the m_nodes: Draw'em all
	// ---------------------------------------------------------------
	// Draw the (grid maps) metric maps of each area:
	for (it = nodesPoses.begin(); it != nodesPoses.end(); it++)
	{
		const CPose3D& pose = it->second.mean;
		const CHMHMapNode::Ptr node = getNodeByID(it->first);

		CMultiMetricMap::Ptr metricMap =
			node->m_annotations.getAs<CMultiMetricMap>(
				NODE_ANNOTATION_METRIC_MAPS, hypothesisID);
		if (metricMap)  // ASSERT_(metricMap);
		{
			mrpt::opengl::CSetOfObjects::Ptr objTex =
				mrpt::opengl::CSetOfObjects::Create();
			metricMap->getAs3DObject(objTex);
			objTex->setPose(pose);
			outScene.insert(objTex);
		}
	}

	float nodes_height = 10.0f;
	float nodes_radius = 1.0f;

	// Draw the m_nodes:
	// ----------------------
	for (it = nodesPoses.begin(), it2 = nodesMeanPoses.begin();
		 it != nodesPoses.end(); it++, it2++)
	{
		const CHMHMapNode::Ptr node = getNodeByID(it->first);
		const CPose3D& pose = it->second.mean;
		const CPose3D meanPose = CPose3D(it2->second);

		// The sphere of the node:
		mrpt::opengl::CSphere::Ptr objSphere = mrpt::opengl::CSphere::Create();

		objSphere->setName(node->m_label);
		objSphere->setColor(0, 0, 1);
		objSphere->setLocation(
			meanPose.x(), meanPose.y(), meanPose.z() + nodes_height);
		objSphere->setRadius(nodes_radius);
		objSphere->setNumberDivsLongitude(16);
		objSphere->setNumberDivsLatitude(16);

		outScene.insert(objSphere);

		// The label with the name of the node:
		mrpt::opengl::CText::Ptr objText = mrpt::opengl::CText::Create();
		//	objText->m_str = node->m_label;
		objText->setString(format("%li", (long int)node->getID()));
		// objText->m_fontHeight = 20;
		objText->setColor(1, 0, 0);
		objText->setLocation(
			meanPose.x(), meanPose.y(), meanPose.z() + nodes_height);

		outScene.insert(objText);

		// And the observations "on the ground" as disks
		// ---------------------------------------------------
		CRobotPosesGraph::Ptr posesGraph =
			node->m_annotations.getAs<CRobotPosesGraph>(
				NODE_ANNOTATION_POSES_GRAPH, hypothesisID);
		// ASSERT_(posesGraph);

		if (posesGraph)
		{
			for (auto& n : *posesGraph)
			{
				CPose3D SF_pose;
				n.second.pdf.getMean(SF_pose);

				CPose3D auxPose(pose + SF_pose);

				mrpt::opengl::CDisk::Ptr glObj = mrpt::opengl::CDisk::Create();

				glObj->setColor(1, 0, 0);

				glObj->setPose(auxPose);
				// glObj->m_z = 0;

				glObj->setDiskRadius(0.05f);
				glObj->setSlicesCount(20);

				if (showRobotPoseIDs)
				{
					glObj->setName(format("%i", (int)it->first));
					glObj->enableShowName();
				}

				outScene.insert(glObj);

				// And a line up-to the node:
				mrpt::opengl::CSimpleLine::Ptr objLine =
					mrpt::opengl::CSimpleLine::Create();

				objLine->setColor(1, 0, 0, 0.2);
				objLine->setLineWidth(1.5);

				objLine->setLineCoords(
					meanPose.x(), meanPose.y(), nodes_height, auxPose.x(),
					auxPose.y(), 0);

				outScene.insert(objLine);
			}  // end for observation "disks"
		}

		// And their m_arcs:
		// ----------------------
		TArcList arcs;
		node->getArcs(arcs, hypothesisID);
		for (auto a = arcs.begin(); a != arcs.end(); ++a)
		{
			CHMHMapArc::Ptr arc = *a;

			if (arc->getNodeFrom() == node->getID())
			{
				CPose3D poseTo(nodesMeanPoses.find(arc->getNodeTo())->second);

				// Draw the line:
				mrpt::opengl::CSimpleLine::Ptr objLine =
					mrpt::opengl::CSimpleLine::Create();

				objLine->setColor(0, 1, 0, 0.5);
				objLine->setLineWidth(5);

				objLine->setLineCoords(
					meanPose.x(), meanPose.y(), meanPose.z() + nodes_height,
					poseTo.x(), poseTo.y(), poseTo.z() + nodes_height);

				outScene.insert(objLine);
			}
		}
	}

	MRPT_END
}

void CHierarchicalMapMHPartition::computeGloballyConsistentNodeCoordinates(
	std::map<CHMHMapNode::TNodeID, mrpt::poses::CPose3DPDFGaussian>& nodePoses,
	const CHMHMapNode::TNodeID& idReferenceNode,
	const THypothesisID& hypothesisID,
	const unsigned int& numberOfIterations) const
{
	MRPT_START

	nodePoses.clear();

	if (m_arcs.empty()) return;  // Nothing we can do!

	// 1) Convert hmt-slam graph into graphslam graph... (this should be avoided
	// in future version of HTML-SLAM!!)
	graphs::CNetworkOfPoses3DInf pose_graph;

	for (const auto& m_arc : m_arcs)
	{
		if (!m_arc->m_hypotheses.has(hypothesisID)) continue;

		const CHMHMapNode::TNodeID id_from = m_arc->getNodeFrom();
		const CHMHMapNode::TNodeID id_to = m_arc->getNodeTo();

		CSerializable::Ptr anotation =
			m_arc->m_annotations.get(ARC_ANNOTATION_DELTA, hypothesisID);
		if (!anotation) continue;

		CPose3DPDFGaussianInf edge_rel_pose_pdf;  // Convert to gaussian
		edge_rel_pose_pdf.copyFrom(
			*std::dynamic_pointer_cast<CPose3DPDF>(anotation));

		pose_graph.insertEdgeAtEnd(id_from, id_to, edge_rel_pose_pdf);
	}

	// 2) Initialize global poses of nodes with Dijkstra:
	pose_graph.root = idReferenceNode;
	pose_graph.dijkstra_nodes_estimate();

	// 3) Optimize with graph-slam:
	graphslam::TResultInfoSpaLevMarq out_info;
	mrpt::containers::yaml graphslam_params;
	graphslam_params["max_iterations"] = numberOfIterations;

	graphslam::optimize_graph_spa_levmarq(
		pose_graph, out_info,
		nullptr,  // Optimize all nodes
		graphslam_params);

	// 4) Copy back optimized results into the HMT-SLAM graph:
	for (auto it_node = pose_graph.nodes.begin();
		 it_node != pose_graph.nodes.end(); ++it_node)
	{
		const CHMHMapNode::TNodeID node_id = it_node->first;

		// To the output map:
		CPose3DPDFGaussian& new_pose = nodePoses[node_id];
		new_pose.mean = it_node->second;
		new_pose.cov.setIdentity();  // *** At present, graphslam does not
		// output the uncertainty of poses... ***
	}

#if __computeGloballyConsistentNodeCoordinates__VERBOSE
	for (map<CHMHMapNode::TNodeID, CPose3DPDFGaussian>::const_iterator it =
			 nodePoses.begin();
		 it != nodePoses.end(); ++it)
		cout << it->first << ": " << it->second.mean << endl;
	cout << endl;
#endif

	MRPT_END
}

/*---------------------------------------------------------------
				dumpAsText
  ---------------------------------------------------------------*/
void CHierarchicalMapMHPartition::dumpAsText(std::vector<std::string>& st) const
{
	st.clear();
	st.emplace_back("LIST OF NODES");
	st.emplace_back("================");

	for (const auto& m_node : m_nodes)
	{
		std::string s;
		s += format(
			"NODE ID: %i\t LABEL:%s\tARCS: ", (int)m_node.second->getID(),
			m_node.second->m_label.c_str());
		TArcList arcs;
		m_node.second->getArcs(arcs);
		for (auto a = arcs.begin(); a != arcs.end(); ++a)
			s += format(
				"%i-%i, ", (int)(*a)->getNodeFrom(), (int)(*a)->getNodeTo());

		st.push_back(s);

		for (auto ann = m_node.second->m_annotations.begin();
			 ann != m_node.second->m_annotations.end(); ++ann)
		{
			s = format(
				"   [HYPO ID #%02i] Annotation '%s' Class: ", (int)ann->ID,
				ann->name.c_str());
			if (ann->value)
				s += string(ann->value->GetRuntimeClass()->className);
			else
				s += "(nullptr)";

			st.push_back(s);

			if (ann->name == NODE_ANNOTATION_REF_POSEID)
			{
				TPoseID refID;
				m_node.second->m_annotations.getElemental(
					NODE_ANNOTATION_REF_POSEID, refID, ann->ID);
				st.push_back(format("     VALUE: %i", (int)refID));
			}
			else if (ann->name == NODE_ANNOTATION_POSES_GRAPH)
			{
				CRobotPosesGraph::Ptr posesGraph =
					m_node.second->m_annotations.getAs<CRobotPosesGraph>(
						NODE_ANNOTATION_POSES_GRAPH, ann->ID);
				ASSERT_(posesGraph);

				st.push_back(format(
					"     CRobotPosesGraph has %i poses:",
					(int)posesGraph->size()));
				CPose3D pdfMean;
				for (auto p = posesGraph->begin(); p != posesGraph->end(); ++p)
				{
					const CPose3DPDFParticles& pdf = p->second.pdf;
					pdf.getMean(pdfMean);
					st.push_back(format(
						"       Pose %i \t (%.03f,%.03f,%.03fdeg)",
						(int)p->first, pdfMean.x(), pdfMean.y(),
						RAD2DEG(pdfMean.yaw())));
				}
			}
		}

		st.emplace_back("");
	}

	st.emplace_back("");
	st.emplace_back("");
	st.emplace_back("LIST OF ARCS");
	st.emplace_back("================");

	for (const auto& m_arc : m_arcs)
	{
		std::string s;
		s += format(
			"ARC: %i -> %i\n", (int)m_arc->getNodeFrom(),
			(int)m_arc->getNodeTo());

		s += string("   Arc type: ") + m_arc->m_arcType;

		st.push_back(s);

		for (auto ann = m_arc->m_annotations.begin();
			 ann != m_arc->m_annotations.end(); ++ann)
		{
			s = format(
				"   [HYPO ID #%02i] Annotation '%s' Class: ", (int)ann->ID,
				ann->name.c_str());
			if (ann->value)
				s += string(ann->value->GetRuntimeClass()->className);
			else
				s += "(nullptr)";

			st.push_back(s);

			if (ann->name == ARC_ANNOTATION_DELTA_SRC_POSEID)
			{
				TPoseID refID;
				m_arc->m_annotations.getElemental(
					ARC_ANNOTATION_DELTA_SRC_POSEID, refID, ann->ID);
				st.push_back(format("     VALUE: %i", (int)refID));
			}
			else if (ann->name == ARC_ANNOTATION_DELTA_TRG_POSEID)
			{
				TPoseID refID = 0;
				m_arc->m_annotations.getElemental(
					ARC_ANNOTATION_DELTA_TRG_POSEID, refID, ann->ID);
				st.push_back(format("     VALUE: %i", (int)refID));
			}
			else if (ann->name == ARC_ANNOTATION_DELTA)
			{
				CSerializable::Ptr o =
					m_arc->m_annotations.get(ARC_ANNOTATION_DELTA, ann->ID);
				ASSERT_(o);

				CPose3DPDFGaussian relativePoseAcordToArc;
				relativePoseAcordToArc.copyFrom(
					*std::dynamic_pointer_cast<CPose3DPDF>(o));

				st.push_back(format(
					"     VALUE: (%f,%f,%f , %fdeg,%fdeg,%fdeg)",
					relativePoseAcordToArc.mean.x(),
					relativePoseAcordToArc.mean.y(),
					relativePoseAcordToArc.mean.z(),
					RAD2DEG(relativePoseAcordToArc.mean.yaw()),
					RAD2DEG(relativePoseAcordToArc.mean.pitch()),
					RAD2DEG(relativePoseAcordToArc.mean.roll())));
			}
		}

		st.emplace_back("");
	}
}

/*---------------------------------------------------------------
					findArcOfTypeBetweenNodes
  ---------------------------------------------------------------*/
CHMHMapArc::Ptr CHierarchicalMapMHPartition::findArcOfTypeBetweenNodes(
	const CHMHMapNode::TNodeID& node1id, const CHMHMapNode::TNodeID& node2id,
	const THypothesisID& hypothesisID, const std::string& arcType,
	bool& isInverted) const
{
	MRPT_START

	TArcList lstArcs;
	findArcsOfTypeBetweenNodes(
		node1id, node2id, hypothesisID, arcType, lstArcs);

	for (auto a = lstArcs.begin(); a != lstArcs.end(); ++a)
	{
		if ((*a)->getNodeFrom() == node1id)
		{
			// Found, and in the correct direction:
			isInverted = false;
			return *a;
		}
		else
		{
			// Found, in the opossite direction:
			isInverted = true;
			return *a;
		}
	}

	return CHMHMapArc::Ptr();
	MRPT_END
}

/*---------------------------------------------------------------
			computeOverlapProbabilityBetweenNodes
  ---------------------------------------------------------------*/
double CHierarchicalMapMHPartition::computeOverlapProbabilityBetweenNodes(
	const CHMHMapNode::TNodeID& nodeFrom, const CHMHMapNode::TNodeID& nodeTo,
	const THypothesisID& hypothesisID, size_t monteCarloSamples,
	const float margin_to_substract) const
{
	MRPT_START

	size_t i, hits = 0;
	CPose3DPDFParticles posePDF;
	const CHMHMapNode::Ptr from = getNodeByID(nodeFrom);
	const CHMHMapNode::Ptr to = getNodeByID(nodeTo);

	// Draw pose samples:
	computeCoordinatesTransformationBetweenNodes(
		nodeFrom, nodeTo, posePDF, hypothesisID, monteCarloSamples);
	/*0.15f,
	DEG2RAD(3.0f) );*/

	// Get the extends of grid maps:
	float r1_x_min, r1_x_max, r1_y_min, r1_y_max, r2_x_min, r2_x_max, r2_y_min,
		r2_y_max;

	// MAP1:
	CMultiMetricMap::Ptr hMap1 = from->m_annotations.getAs<CMultiMetricMap>(
		NODE_ANNOTATION_METRIC_MAPS, hypothesisID, false);

	auto grid = hMap1->mapByClass<COccupancyGridMap2D>();
	ASSERT_(grid);

	r1_x_min = grid->getXMin();
	r1_x_max = grid->getXMax();
	r1_y_min = grid->getYMin();
	r1_y_max = grid->getYMax();

	if (r1_x_max - r1_x_min > 2 * margin_to_substract)
	{
		r1_x_max -= margin_to_substract;
		r1_x_min += margin_to_substract;
	}
	if (r1_y_max - r1_y_min > 2 * margin_to_substract)
	{
		r1_y_max -= margin_to_substract;
		r1_y_min += margin_to_substract;
	}

	// MAP2:
	CMultiMetricMap::Ptr hMap2 = to->m_annotations.getAs<CMultiMetricMap>(
		NODE_ANNOTATION_METRIC_MAPS, hypothesisID, false);

	auto grid2 = hMap2->mapByClass<COccupancyGridMap2D>();
	ASSERT_(grid2);

	r2_x_min = grid2->getXMin();
	r2_x_max = grid2->getXMax();
	r2_y_min = grid2->getYMin();
	r2_y_max = grid2->getYMax();

	if (r2_x_max - r2_x_min > 2 * margin_to_substract)
	{
		r2_x_max -= margin_to_substract;
		r2_x_min += margin_to_substract;
	}
	if (r2_y_max - r2_y_min > 2 * margin_to_substract)
	{
		r2_y_max -= margin_to_substract;
		r2_y_min += margin_to_substract;
	}

	// Compute the probability:
	for (i = 0; i < monteCarloSamples; i++)
	{
		if (math::RectanglesIntersection(
				r1_x_min, r1_x_max, r1_y_min, r1_y_max, r2_x_min, r2_x_max,
				r2_y_min, r2_y_max, posePDF.m_particles[i].d.x,
				posePDF.m_particles[i].d.y, posePDF.m_particles[i].d.yaw))
		{
			hits++;
		}
	}

	return static_cast<double>(hits) / monteCarloSamples;

	MRPT_END
}

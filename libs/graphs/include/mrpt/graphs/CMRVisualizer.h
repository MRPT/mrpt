/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <iostream>

#include <mrpt/graphs/CVisualizer.h>
#include <mrpt/graphs/TMRSlamNodeAnnotations.h>
#include <utility>

namespace mrpt::graphs::detail
{
// (Dummy) standard version
//////////////////////////////////////////////////////////

/**\brief Wrapper class that provides visualization of a network of poses that
 * have been registered by many graphSLAM agents/robots
 *
 * \note This is a dummy 4 template arguments version of the CMRVisualizer
 * class just to silence GCC warnings on conversions.
 * For implementation details users should see its specialized version that
 * runs specifically for TMRSlamNodeAnnotations as the 3rd template argument.
 *
 */
template <
	class CPOSE,  // Type of edges
	class MAPS_IMPLEMENTATION,  // Use std::map<> vs. std::vector<>
	class NODE_ANNOTATIONS = mrpt::graphs::detail::TMRSlamNodeAnnotations,
	class EDGE_ANNOTATIONS = mrpt::graphs::detail::edge_annotations_empty>
class CMRVisualizer
	: public CVisualizer<
		  CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>
{
   public:
	using parent = CVisualizer<
		CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>;
	using GRAPH_T = mrpt::graphs::CNetworkOfPoses<
		CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>;

	CMRVisualizer(const GRAPH_T& graph_in);

	~CMRVisualizer() override;
	void drawNodePoints(
		mrpt::opengl::CSetOfObjects::Ptr& object,
		const mrpt::system::TParametersDouble* viz_params =
			nullptr) const override;
	void drawEdges(
		mrpt::opengl::CSetOfObjects::Ptr& object,
		const mrpt::system::TParametersDouble* viz_params =
			nullptr) const override;

   private:
};

// Specialized version
//////////////////////////////////////////////////////////

template <
	class CPOSE,  // Type of edges
	class MAPS_IMPLEMENTATION,  // Use std::map<> vs. std::vector<>
	class EDGE_ANNOTATIONS>
class CMRVisualizer<
	CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>
	: public CVisualizer<
		  CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>
{
   public:
	using parent = CVisualizer<
		CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>;
	using GRAPH_T = mrpt::graphs::CNetworkOfPoses<
		CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>;

	CMRVisualizer(const GRAPH_T& graph_in);

	~CMRVisualizer();
	void drawNodePoints(
		mrpt::opengl::CSetOfObjects::Ptr& object,
		const mrpt::system::TParametersDouble* viz_params = nullptr) const;
	void drawEdges(
		mrpt::opengl::CSetOfObjects::Ptr& object,
		const mrpt::system::TParametersDouble* viz_params = nullptr) const;

   private:
};
}  // namespace mrpt::graphs::detail
#include <mrpt/graphs/CMRVisualizer_impl.h>

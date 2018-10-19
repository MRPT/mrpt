/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/containers/stl_containers_utils.h>

// Implementattion file for TUncertaintyPath struct

namespace mrpt::graphslam
{
template <class GRAPH_T>
TUncertaintyPath<GRAPH_T>::TUncertaintyPath()
{
	this->clear();
}
template <class GRAPH_T>
TUncertaintyPath<GRAPH_T>::TUncertaintyPath(
	const mrpt::graphs::TNodeID& starting_node)
{
	this->clear();
	nodes_traversed.push_back(starting_node);
}
template <class GRAPH_T>
TUncertaintyPath<GRAPH_T>::TUncertaintyPath(
	const mrpt::graphs::TNodeID& starting_node,
	const mrpt::graphs::TNodeID& ending_node, const constraint_t& edge)
{
	this->clear();
	nodes_traversed.push_back(starting_node);
	this->addToPath(ending_node, edge);
}

template <class GRAPH_T>
void TUncertaintyPath<GRAPH_T>::clear()
{
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

	// clear the vector of traversed nodes
	nodes_traversed.clear();

	// clear the relative edge
	curr_pose_pdf.mean = pose_t();
	// by default the information matrix is set to the unit matrix
	CMatrixDouble33 init_path_mat;
	init_path_mat.unit();
	// put a really large number - we are certain of this position
	init_path_mat *= 10000;  // TODO - justify this..
	curr_pose_pdf.cov_inv = init_path_mat;

	determinant_is_updated = false;
	determinant_cached = 0;

}  // end of clear

template <class GRAPH_T>
bool TUncertaintyPath<GRAPH_T>::isEmpty() const
{
	return *this == self_t();
}

template <class GRAPH_T>
void TUncertaintyPath<GRAPH_T>::assertIsBetweenNodeIDs(
	const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to) const
{
	ASSERTDEBMSG_(
		this->getSource() == from,
		format(
			"\nnodeID %lu is not the source of the path\n%s\n\n",
			static_cast<unsigned long>(from), this->getAsString().c_str()));
	ASSERTDEBMSG_(
		this->getDestination() == to,
		format(
			"\nnodeID %lu is not the destination of the path\n%s\n\n",
			static_cast<unsigned long>(to), this->getAsString().c_str()));
}

template <class GRAPH_T>
TUncertaintyPath<GRAPH_T>& TUncertaintyPath<GRAPH_T>::operator+=(
	const self_t& other)
{
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

	// other path should start where this ends
	ASSERTDEBMSG_(
		other.nodes_traversed.begin()[0] == this->nodes_traversed.rbegin()[0],
		"\"other\" instance must start from the nodeID that this "
		"TUncertaintyPath has ended.");
	ASSERTDEBMSG_(
		other.nodes_traversed.size(),
		"\"other\" instance doesn't have an initialized list of traversed "
		"nodes");
	ASSERTDEBMSG_(
		this->nodes_traversed.size(),
		"\"this\" instance doesn't have an initialized list of traversed "
		"nodes");

	//////// TODO Remove these - >>>>>>>>>>>>>>>>>>>>>
	// cout << string(20, '-') << "Aggregating 2 paths.."
	//<< string(20, '-') << endl;
	// this->dumpToConsole(); other.dumpToConsole();
	////// TODO Remove these - <<<<<<<<<<<<<<<<<<<<<

	// aggregate the two gaussian - mean & information matrix
	this->curr_pose_pdf += other.curr_pose_pdf;

	// add the traversed nodes
	this->nodes_traversed.insert(
		this->nodes_traversed.end(), other.nodes_traversed.begin() + 1,
		other.nodes_traversed.end());

	////// TODO Remove these - >>>>>>>>>>>>>>>>>>>>>
	// cout << std::string(10, '%') << endl << "AFTER Aggregation..." << endl;
	// this->dumpToConsole();
	// cout << string(50, '-') << endl;
	// mrpt::system::pause();
	////// TODO Remove these - <<<<<<<<<<<<<<<<<<<<<

	determinant_is_updated = false;
	return *this;
}
template <class GRAPH_T>
bool TUncertaintyPath<GRAPH_T>::operator==(const self_t& other) const
{
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

	// check if the traversed nodes are the same as well as the
	// CPoseGaussianInfs are the same..
	return (
		this->nodes_traversed == other.nodes_traversed &&
		this->curr_pose_pdf == other.curr_pose_pdf);
}
template <class GRAPH_T>
bool TUncertaintyPath<GRAPH_T>::operator!=(const self_t& other) const
{
	return !(*this == other);
}

template <class GRAPH_T>
void TUncertaintyPath<GRAPH_T>::addToPath(
	const mrpt::graphs::TNodeID& node, const constraint_t& edge)
{
	// update the path
	curr_pose_pdf += edge;

	// update the traversed nodes
	nodes_traversed.push_back(node);

	determinant_is_updated = false;
}

template <class GRAPH_T>
void TUncertaintyPath<GRAPH_T>::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const std::string& section)
{
}

template <class GRAPH_T>
void TUncertaintyPath<GRAPH_T>::dumpToTextStream(std::ostream& out) const
{
	out << mrpt::format("%s\n", this->getAsString().c_str());
}

template <class GRAPH_T>
void TUncertaintyPath<GRAPH_T>::getAsString(std::string* str) const
{
	using namespace mrpt;
	using namespace mrpt::poses;
	using namespace std;
	using namespace mrpt::math;
	using namespace mrpt::containers;

	stringstream ss;
	string header_sep(30, '=');

	ss << "Path properties: " << endl;
	ss << header_sep << endl << endl;

	ss << "- CPosePDFGaussianInf: "
	   << (curr_pose_pdf.isInfType() ? "TRUE" : "FALSE") << endl;
	ss << "- Nodes list: \n\t< " << getSTLContainerAsString(nodes_traversed)
	   << "\b\b>" << endl;

	ss << endl;
	ss << curr_pose_pdf << endl;
	ss << endl;

	CMatrixDouble33 mat;
	if (curr_pose_pdf.isInfType())
	{
		curr_pose_pdf.getInformationMatrix(mat);
	}
	else
	{
		curr_pose_pdf.getCovariance(mat);
	}
	ss << "Determinant: " << mat.det();

	*str = ss.str();
}
template <class GRAPH_T>
std::string TUncertaintyPath<GRAPH_T>::getAsString() const
{
	std::string s;
	this->getAsString(&s);
	return s;
}

template <class GRAPH_T>
const mrpt::graphs::TNodeID& TUncertaintyPath<GRAPH_T>::getSource() const
{
	return nodes_traversed.at(0);
}
template <class GRAPH_T>
const mrpt::graphs::TNodeID& TUncertaintyPath<GRAPH_T>::getDestination() const
{
	return nodes_traversed.back();
}

template <class GRAPH_T>
double TUncertaintyPath<GRAPH_T>::getDeterminant()
{
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

	// if determinant is up-to-date then return the cached version...
	if (determinant_is_updated) return determinant_cached;

	// update the cached version and return it.
	CMatrixDouble33 mat;
	if (curr_pose_pdf.isInfType())
	{
		curr_pose_pdf.getInformationMatrix(mat);
	}
	else
	{
		curr_pose_pdf.getCovariance(mat);
	}
	double determinant = mat.det();

	determinant_cached = determinant;
	determinant_is_updated = true;

	return determinant;
}

template <class GRAPH_T>
bool TUncertaintyPath<GRAPH_T>::hasLowerUncertaintyThan(
	const self_t& other) const
{
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

	ASSERTDEBMSG_(
		(curr_pose_pdf.isInfType() && other.curr_pose_pdf.isInfType()) ||
			(!curr_pose_pdf.isInfType() && !other.curr_pose_pdf.isInfType()),
		mrpt::format("Constraints of given paths don't have the same "
					 "representation of uncertainty"));

	// If we are talking about information form matrices, the *higher* the
	// determinant the more confident we are.
	// if we are talking about covariances then the *lower*.
	bool has_lower = false;
	if (curr_pose_pdf.isInfType())
	{
		has_lower = this->getDeterminant() > other.getDeterminant();
	}
	else
	{
		has_lower = this->getDeterminant() < other.getDeterminant();
	}

	return has_lower;
}
}  // namespace mrpt::graphslam

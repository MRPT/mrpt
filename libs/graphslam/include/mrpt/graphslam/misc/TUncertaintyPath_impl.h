/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef TUNCERTAINTYPATH_IMPL_H
#define TUNCERTAINTYPATH_IMPL_H

// Implementattion file for TUncertaintyPath struct

namespace mrpt { namespace graphslam {

template<class GRAPH_t>
TUncertaintyPath<GRAPH_t>::TUncertaintyPath() {
	this->clear();
}
template<class GRAPH_t>
TUncertaintyPath<GRAPH_t>::TUncertaintyPath(mrpt::utils::TNodeID starting_node) {
	this->clear();

	nodes_traversed.push_back(starting_node);
}

template<class GRAPH_t>
TUncertaintyPath<GRAPH_t>::~TUncertaintyPath() { }

template<class GRAPH_t>
void TUncertaintyPath<GRAPH_t>::clear() {
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

	// clear the vector of traversed nodes
	nodes_traversed.clear();

	// clear the relative edge
	curr_pose_pdf.mean = pose_t();
	// by default the information matrix is set to the unit matrix
	CMatrixDouble33 init_path_mat; init_path_mat.unit();
	// put a really large number - we are certain of this position
	init_path_mat *= 10000; //TODO - justify this..
	curr_pose_pdf.cov_inv = init_path_mat;

	determinant_is_updated = false;
	determinant_cached = 0;

}

template<class GRAPH_t>
TUncertaintyPath<GRAPH_t>&
TUncertaintyPath<GRAPH_t>::operator+=(
		const TUncertaintyPath<GRAPH_t>& other) {
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

	// other path should start where this ends
	ASSERTMSG_(other.nodes_traversed.begin()[0] ==
			this->nodes_traversed.rbegin()[0],
			"\"other\" instance must start from the nodeID that this "
			"TUncertaintyPath has ended.");
	ASSERTMSG_(other.nodes_traversed.size(),
			"\"other\" instance doesn't have an initialized list of traversed nodes");
	ASSERTMSG_(this->nodes_traversed.size(),
			"\"this\" instance doesn't have an initialized list of traversed nodes");

	//////// TODO Remove these - >>>>>>>>>>>>>>>>>>>>>
	//cout << string(20, '-') << "Aggregating 2 paths.."
		//<< string(20, '-') << endl;
	//this->dumpToConsole(); other.dumpToConsole();
	////// TODO Remove these - <<<<<<<<<<<<<<<<<<<<<

	// aggregate the two gaussian - mean & information matrix
	this->curr_pose_pdf += other.curr_pose_pdf;

	// add the traversed nodes
	this->nodes_traversed.insert(
			this->nodes_traversed.end(),
			other.nodes_traversed.begin()+1,
			other.nodes_traversed.end());

	////// TODO Remove these - >>>>>>>>>>>>>>>>>>>>>
	//cout << std::string(10, '%') << endl << "AFTER Aggregation..." << endl;
	//this->dumpToConsole();
	//cout << string(50, '-') << endl;
	//mrpt::system::pause();
	////// TODO Remove these - <<<<<<<<<<<<<<<<<<<<<

	determinant_is_updated = false;
	return *this;

}
template<class GRAPH_t>
bool TUncertaintyPath<GRAPH_t>::operator==(
		const TUncertaintyPath<GRAPH_t>& other) {
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

		// check if the traversed nodes are the same as well as the
		// CPoseGaussianInfs are the same..
		return ( this->nodes_traversed == other.nodes_traversed &&
				this->curr_pose_pdf == other.curr_pose_pdf );
		
}
template<class GRAPH_t>
bool TUncertaintyPath<GRAPH_t>::operator!=(
		const TUncertaintyPath<GRAPH_t>& other) {

		return !(*this == other);
}

template<class GRAPH_t>
void TUncertaintyPath<GRAPH_t>::addToPath(
		mrpt::utils::TNodeID node,
		typename GRAPH_t::constraint_t edge) {

	// update the path
	curr_pose_pdf += edge;

	// update the traversed nodes
	nodes_traversed.push_back(node);

	determinant_is_updated = false;
}

template<class GRAPH_t>
void TUncertaintyPath<GRAPH_t>::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
		const std::string &section) {}

template<class GRAPH_t>
void TUncertaintyPath<GRAPH_t>::dumpToTextStream(
		mrpt::utils::CStream &out) const {

	out.printf("%s\n", this->getAsString().c_str());

}

template<class GRAPH_t>
void TUncertaintyPath<GRAPH_t>::getAsString(std::string* str) const{
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

	stringstream ss;
	string header_sep(30, '=');

	ss << "Path properties: " << endl;
	ss << header_sep << endl << endl;

	ss << "- CPosePDFGaussianInf: "
		<< (this->isGaussianInfType()?  "TRUE" : "FALSE") << endl;
	ss << "- Nodes list: \n\t< " <<
		getSTLContainerAsString(nodes_traversed)
		<< "\b\b>" << endl;

	ss << endl;
	ss << curr_pose_pdf << endl;
	ss << endl;

	CMatrixDouble33 mat;
	if (this->isGaussianType()) {
		curr_pose_pdf.getCovariance(mat);
	}
	else if (this->isGaussianInfType()) {
		curr_pose_pdf.getInformationMatrix(mat);
	}
	ss << "Determinant: " << mat.det();

	*str = ss.str();
}
template<class GRAPH_t>
std::string TUncertaintyPath<GRAPH_t>::getAsString() const {
	std::string s;
	this->getAsString(&s);
	return s;
}

template<class GRAPH_t>
mrpt::utils::TNodeID TUncertaintyPath<GRAPH_t>::getSource() const {
	return nodes_traversed.at(0);
}
template<class GRAPH_t>
mrpt::utils::TNodeID TUncertaintyPath<GRAPH_t>::getDestination() const {
	return nodes_traversed.back();
}

template<class GRAPH_t>
double TUncertaintyPath<GRAPH_t>::getDeterminant() {
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

	// if determinant is up-to-date then return the cached version...
	if (determinant_is_updated) return determinant_cached;

	CMatrixDouble33 mat;
	if (this->isGaussianInfType()) {
		curr_pose_pdf.getInformationMatrix(mat);
	}
	else if (this->isGaussianType()) {
		curr_pose_pdf.getCovariance(mat);
	}
	double determinant = mat.det();

	// update the cached version
	determinant_cached = determinant;
	determinant_is_updated = true;


	return determinant;

}

template<class GRAPH_t>
bool TUncertaintyPath<GRAPH_t>::hasLowerUncertaintyThan(
		const TUncertaintyPath<GRAPH_t>& other) const {
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

	ASSERTMSG_(
			(this->isGaussianInfType() && other->isGaussianInfType()) ||
			(this->isGaussianType() && other->isGaussianType()),
			mrpt::format(
				"Constraints of given paths don't have the same representation of uncertainty"));

	// If we are talking about information form matrices, the *higher* the 
	// determinant the more confident we are.
	// if we are talking about covariances then the *lower*.
	bool has_lower = false;
	if (this->isGaussianInfType()) {
		has_lower = this->getDeterminant() > other->getDeterminant();
	}
	else if (this->isGaussianType()) {
		has_lower = this->getDeterminant() < other->getDeterminant();
	}

	return has_lower;
}

template<class GRAPH_t>
bool TUncertaintyPath<GRAPH_t>::isGaussianInfType() const {
	using namespace mrpt::poses;
	return curr_pose_pdf.GetRuntimeClass() == CLASS_ID(CPosePDFGaussianInf);
}
template<class GRAPH_t>
bool TUncertaintyPath<GRAPH_t>::isGaussianType() const {
	using namespace mrpt::poses;
	return curr_pose_pdf.GetRuntimeClass() == CLASS_ID(CPosePDFGaussian);
}

} } // end of namespaces

#endif /* end of include guard: TUNCERTAINTYPATH_IMPL_H */

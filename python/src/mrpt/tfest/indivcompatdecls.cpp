#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint2DPDF.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/tfest/indiv-compat-decls.h>
#include <mrpt/tfest/se2.h>
#include <mrpt/tfest/se3.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>
#include <tuple>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_tfest_indivcompatdecls(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::tfest::TPotentialMatch file:mrpt/tfest/indiv-compat-decls.h line:24
		pybind11::class_<mrpt::tfest::TPotentialMatch, std::shared_ptr<mrpt::tfest::TPotentialMatch>> cl(M("mrpt::tfest"), "TPotentialMatch", "For each individual-compatibility (IC) test, the indices of the candidate\n match between elements in both reference frames.\n \n\n TSE3RobustParams::user_individual_compat_callback ,\n TSE2RobustParams::user_individual_compat_callback");
		cl.def( pybind11::init( [](){ return new mrpt::tfest::TPotentialMatch(); } ) );
		cl.def_readwrite("idx_this", &mrpt::tfest::TPotentialMatch::idx_this);
		cl.def_readwrite("idx_other", &mrpt::tfest::TPotentialMatch::idx_other);
	}
	// mrpt::tfest::se2_l2(const class mrpt::tfest::TMatchingPairListTempl<float> &, struct mrpt::math::TPose2D &, class mrpt::math::CMatrixFixed<double, 3, 3> *) file:mrpt/tfest/se2.h line:62
	M("mrpt::tfest").def("se2_l2", [](const class mrpt::tfest::TMatchingPairListTempl<float> & a0, struct mrpt::math::TPose2D & a1) -> bool { return mrpt::tfest::se2_l2(a0, a1); }, "", pybind11::arg("in_correspondences"), pybind11::arg("out_transformation"));
	M("mrpt::tfest").def("se2_l2", (bool (*)(const class mrpt::tfest::TMatchingPairListTempl<float> &, struct mrpt::math::TPose2D &, class mrpt::math::CMatrixFixed<double, 3, 3> *)) &mrpt::tfest::se2_l2, "Least-squares (L2 norm) solution to finding the optimal SE(2) (x,y,yaw)\n between two reference frames.\n  The optimal transformation `q` fulfills \n\n\n, that is, the\n  transformation of frame `other` with respect to `this`.\n\n  \n\n \n The set of correspondences.\n \n\n The pose that minimizes the mean-square-error\n between all the correspondences.\n \n\n If provided (!=nullptr) this will contain\n on return a 3x3 covariance matrix with the NORMALIZED optimal estimate\n uncertainty. This matrix must be multiplied by \n\n, the variance\n of matched points in \n\n and \n (see paper\n https://www.mrpt.org/Paper:Occupancy_Grid_Matching)\n \n\n True if there are at least two correspondences, or false if one or\n none, thus we cannot establish any correspondence.\n \n\n robustRigidTransformation\n\n \n Reference for covariance calculation: J.L. Blanco, J.\n Gonzalez-Jimenez, J.A. Fernandez-Madrigal, \"A Robust, Multi-Hypothesis\n Approach to Matching Occupancy Grid Maps\", Robotica, 2013.\n http://dx.doi.org/10.1017/S0263574712000732\n \n\n [New in MRPT 1.3.0] This function replaces\n mrpt::scanmatching::leastSquareErrorRigidTransformation()\n \n\n This function is hand-optimized for SSE2 architectures (if SSE2 is\n enabled from CMake)\n \n\n se3_l2, se2_l2_robust\n \n\n\n \n\n \n\nC++: mrpt::tfest::se2_l2(const class mrpt::tfest::TMatchingPairListTempl<float> &, struct mrpt::math::TPose2D &, class mrpt::math::CMatrixFixed<double, 3, 3> *) --> bool", pybind11::arg("in_correspondences"), pybind11::arg("out_transformation"), pybind11::arg("out_estimateCovariance"));

	// mrpt::tfest::se2_l2(const class mrpt::tfest::TMatchingPairListTempl<float> &, class mrpt::poses::CPosePDFGaussian &) file:mrpt/tfest/se2.h line:68
	M("mrpt::tfest").def("se2_l2", (bool (*)(const class mrpt::tfest::TMatchingPairListTempl<float> &, class mrpt::poses::CPosePDFGaussian &)) &mrpt::tfest::se2_l2, "C++: mrpt::tfest::se2_l2(const class mrpt::tfest::TMatchingPairListTempl<float> &, class mrpt::poses::CPosePDFGaussian &) --> bool", pybind11::arg("in_correspondences"), pybind11::arg("out_transformation"));

	{ // mrpt::tfest::TSE2RobustParams file:mrpt/tfest/se2.h line:73
		pybind11::class_<mrpt::tfest::TSE2RobustParams, std::shared_ptr<mrpt::tfest::TSE2RobustParams>> cl(M("mrpt::tfest"), "TSE2RobustParams", "Parameters for se2_l2_robust(). See function for more details ");
		cl.def( pybind11::init( [](){ return new mrpt::tfest::TSE2RobustParams(); } ) );
		cl.def( pybind11::init( [](mrpt::tfest::TSE2RobustParams const &o){ return new mrpt::tfest::TSE2RobustParams(o); } ) );
		cl.def_readwrite("ransac_minSetSize", &mrpt::tfest::TSE2RobustParams::ransac_minSetSize);
		cl.def_readwrite("ransac_maxSetSize", &mrpt::tfest::TSE2RobustParams::ransac_maxSetSize);
		cl.def_readwrite("ransac_mahalanobisDistanceThreshold", &mrpt::tfest::TSE2RobustParams::ransac_mahalanobisDistanceThreshold);
		cl.def_readwrite("ransac_nSimulations", &mrpt::tfest::TSE2RobustParams::ransac_nSimulations);
		cl.def_readwrite("ransac_fuseByCorrsMatch", &mrpt::tfest::TSE2RobustParams::ransac_fuseByCorrsMatch);
		cl.def_readwrite("ransac_fuseMaxDiffXY", &mrpt::tfest::TSE2RobustParams::ransac_fuseMaxDiffXY);
		cl.def_readwrite("ransac_fuseMaxDiffPhi", &mrpt::tfest::TSE2RobustParams::ransac_fuseMaxDiffPhi);
		cl.def_readwrite("ransac_algorithmForLandmarks", &mrpt::tfest::TSE2RobustParams::ransac_algorithmForLandmarks);
		cl.def_readwrite("probability_find_good_model", &mrpt::tfest::TSE2RobustParams::probability_find_good_model);
		cl.def_readwrite("ransac_min_nSimulations", &mrpt::tfest::TSE2RobustParams::ransac_min_nSimulations);
		cl.def_readwrite("max_rmse_to_end", &mrpt::tfest::TSE2RobustParams::max_rmse_to_end);
		cl.def_readwrite("verbose", &mrpt::tfest::TSE2RobustParams::verbose);
		cl.def_readwrite("user_individual_compat_callback", &mrpt::tfest::TSE2RobustParams::user_individual_compat_callback);
		cl.def("assign", (struct mrpt::tfest::TSE2RobustParams & (mrpt::tfest::TSE2RobustParams::*)(const struct mrpt::tfest::TSE2RobustParams &)) &mrpt::tfest::TSE2RobustParams::operator=, "C++: mrpt::tfest::TSE2RobustParams::operator=(const struct mrpt::tfest::TSE2RobustParams &) --> struct mrpt::tfest::TSE2RobustParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::tfest::TSE2RobustResult file:mrpt/tfest/se2.h line:130
		pybind11::class_<mrpt::tfest::TSE2RobustResult, std::shared_ptr<mrpt::tfest::TSE2RobustResult>> cl(M("mrpt::tfest"), "TSE2RobustResult", "Output placeholder for se2_l2_robust() ");
		cl.def( pybind11::init( [](){ return new mrpt::tfest::TSE2RobustResult(); } ) );
		cl.def_readwrite("transformation", &mrpt::tfest::TSE2RobustResult::transformation);
		cl.def_readwrite("largestSubSet", &mrpt::tfest::TSE2RobustResult::largestSubSet);
		cl.def_readwrite("ransac_iters", &mrpt::tfest::TSE2RobustResult::ransac_iters);
	}
	// mrpt::tfest::se2_l2_robust(const class mrpt::tfest::TMatchingPairListTempl<float> &, const double, const struct mrpt::tfest::TSE2RobustParams &, struct mrpt::tfest::TSE2RobustResult &) file:mrpt/tfest/se2.h line:185
	M("mrpt::tfest").def("se2_l2_robust", (bool (*)(const class mrpt::tfest::TMatchingPairListTempl<float> &, const double, const struct mrpt::tfest::TSE2RobustParams &, struct mrpt::tfest::TSE2RobustResult &)) &mrpt::tfest::se2_l2_robust, "Robust least-squares (L2 norm) solution to finding the optimal SE(2)\n (x,y,yaw) between two reference frames.\n This method implements a RANSAC-based robust estimation, returning a\n probability distribution over all the possibilities as a Sum of Gaussians.\n\n  The optimal transformation `q` fulfills \n\n, that is, the\n  transformation of frame `other` with respect to `this`.\n\n  \n\n  The technique was described in the paper:\n    - J.L. Blanco, J. Gonzalez-Jimenez, J.A. Fernandez-Madrigal, \"A Robust,\n Multi-Hypothesis Approach to Matching Occupancy Grid Maps\", Robotica, 2013.\n http://dx.doi.org/10.1017/S0263574712000732\n\n This works as follows:\n - Repeat \"ransac_nSimulations\" times:\n 	- Randomly pick TWO correspondences from the set \"in_correspondences\".\n 	- Compute the associated rigid transformation.\n 	- For \"ransac_maxSetSize\" randomly selected correspondences, test for\n \"consensus\" with the current group:\n 		- If if is compatible (ransac_mahalanobisDistanceThreshold), grow\n the \"consensus set\"\n 		- If not, do not add it.\n\n  For more details refer to the tutorial on \n* href=\"http://www.mrpt.org/Scan_Matching_Algorithms\">scan matching\n methods.\n \n\n The standard deviation (not variance)\n of landmarks/points/features being matched in X,Y. Used to normalize\n covariances returned as the SoG. (Refer to paper)\n\n NOTE: Parameter `ransac_maxSetSize` should be set to\n `in_correspondences.size()` to make sure that every correspondence is tested\n for each random permutation.\n\n \n True upon success, false if no subset was found with the minimum\n number of correspondences.\n \n\n [New in MRPT 1.3.0] This function replaces\n mrpt::scanmatching::robustRigidTransformation()\n \n\n se3_l2, se2_l2_robust\n\nC++: mrpt::tfest::se2_l2_robust(const class mrpt::tfest::TMatchingPairListTempl<float> &, const double, const struct mrpt::tfest::TSE2RobustParams &, struct mrpt::tfest::TSE2RobustResult &) --> bool", pybind11::arg("in_correspondences"), pybind11::arg("in_normalizationStd"), pybind11::arg("in_ransac_params"), pybind11::arg("out_results"));

	// mrpt::tfest::se3_l2(const class mrpt::tfest::TMatchingPairListTempl<float> &, class mrpt::poses::CPose3DQuat &, double &, bool) file:mrpt/tfest/se3.h line:45
	M("mrpt::tfest").def("se3_l2", [](const class mrpt::tfest::TMatchingPairListTempl<float> & a0, class mrpt::poses::CPose3DQuat & a1, double & a2) -> bool { return mrpt::tfest::se3_l2(a0, a1, a2); }, "", pybind11::arg("in_correspondences"), pybind11::arg("out_transform"), pybind11::arg("out_scale"));
	M("mrpt::tfest").def("se3_l2", (bool (*)(const class mrpt::tfest::TMatchingPairListTempl<float> &, class mrpt::poses::CPose3DQuat &, double &, bool)) &mrpt::tfest::se3_l2, "Least-squares (L2 norm) solution to finding the optimal SE(3) transform\n between two reference frames using the \"quaternion\" or Horn's method\n \n\n  The optimal transformation `q` fulfills \n\n, that is, the\n  transformation of frame `other` with respect to `this`.\n\n  \n\n \n  The coordinates of the input points for the\n two coordinate systems \"this\" and \"other\"\n \n\n       The output transformation\n \n\n           The computed scale of the optimal\n transformation (will be 1.0 for a perfectly rigid translation + rotation).\n \n\n   Whether or not force the scale employed to\n rotate the coordinate systems to one (rigid transformation)\n \n\n [New in MRPT 1.3.0] This function replaces\n mrpt::scanmatching::leastSquareErrorRigidTransformation6DRANSAC() and\n mrpt::scanmatching::HornMethod()\n \n\n se2_l2, se3_l2_robust\n\nC++: mrpt::tfest::se3_l2(const class mrpt::tfest::TMatchingPairListTempl<float> &, class mrpt::poses::CPose3DQuat &, double &, bool) --> bool", pybind11::arg("in_correspondences"), pybind11::arg("out_transform"), pybind11::arg("out_scale"), pybind11::arg("forceScaleToUnity"));

	{ // mrpt::tfest::TSE3RobustParams file:mrpt/tfest/se3.h line:71
		pybind11::class_<mrpt::tfest::TSE3RobustParams, std::shared_ptr<mrpt::tfest::TSE3RobustParams>> cl(M("mrpt::tfest"), "TSE3RobustParams", "Parameters for se3_l2_robust(). See function for more details ");
		cl.def( pybind11::init( [](){ return new mrpt::tfest::TSE3RobustParams(); } ) );
		cl.def( pybind11::init( [](mrpt::tfest::TSE3RobustParams const &o){ return new mrpt::tfest::TSE3RobustParams(o); } ) );
		cl.def_readwrite("ransac_minSetSize", &mrpt::tfest::TSE3RobustParams::ransac_minSetSize);
		cl.def_readwrite("ransac_nmaxSimulations", &mrpt::tfest::TSE3RobustParams::ransac_nmaxSimulations);
		cl.def_readwrite("ransac_maxSetSizePct", &mrpt::tfest::TSE3RobustParams::ransac_maxSetSizePct);
		cl.def_readwrite("ransac_threshold_lin", &mrpt::tfest::TSE3RobustParams::ransac_threshold_lin);
		cl.def_readwrite("ransac_threshold_ang", &mrpt::tfest::TSE3RobustParams::ransac_threshold_ang);
		cl.def_readwrite("ransac_threshold_scale", &mrpt::tfest::TSE3RobustParams::ransac_threshold_scale);
		cl.def_readwrite("forceScaleToUnity", &mrpt::tfest::TSE3RobustParams::forceScaleToUnity);
		cl.def_readwrite("verbose", &mrpt::tfest::TSE3RobustParams::verbose);
		cl.def_readwrite("user_individual_compat_callback", &mrpt::tfest::TSE3RobustParams::user_individual_compat_callback);
		cl.def("assign", (struct mrpt::tfest::TSE3RobustParams & (mrpt::tfest::TSE3RobustParams::*)(const struct mrpt::tfest::TSE3RobustParams &)) &mrpt::tfest::TSE3RobustParams::operator=, "C++: mrpt::tfest::TSE3RobustParams::operator=(const struct mrpt::tfest::TSE3RobustParams &) --> struct mrpt::tfest::TSE3RobustParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::tfest::TSE3RobustResult file:mrpt/tfest/se3.h line:107
		pybind11::class_<mrpt::tfest::TSE3RobustResult, std::shared_ptr<mrpt::tfest::TSE3RobustResult>> cl(M("mrpt::tfest"), "TSE3RobustResult", "Output placeholder for se3_l2_robust() ");
		cl.def( pybind11::init( [](){ return new mrpt::tfest::TSE3RobustResult(); } ) );
		cl.def( pybind11::init( [](mrpt::tfest::TSE3RobustResult const &o){ return new mrpt::tfest::TSE3RobustResult(o); } ) );
		cl.def_readwrite("transformation", &mrpt::tfest::TSE3RobustResult::transformation);
		cl.def_readwrite("scale", &mrpt::tfest::TSE3RobustResult::scale);
		cl.def_readwrite("inliers_idx", &mrpt::tfest::TSE3RobustResult::inliers_idx);
		cl.def("assign", (struct mrpt::tfest::TSE3RobustResult & (mrpt::tfest::TSE3RobustResult::*)(const struct mrpt::tfest::TSE3RobustResult &)) &mrpt::tfest::TSE3RobustResult::operator=, "C++: mrpt::tfest::TSE3RobustResult::operator=(const struct mrpt::tfest::TSE3RobustResult &) --> struct mrpt::tfest::TSE3RobustResult &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::tfest::se3_l2_robust(const class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::tfest::TSE3RobustParams &, struct mrpt::tfest::TSE3RobustResult &) file:mrpt/tfest/se3.h line:140
	M("mrpt::tfest").def("se3_l2_robust", (bool (*)(const class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::tfest::TSE3RobustParams &, struct mrpt::tfest::TSE3RobustResult &)) &mrpt::tfest::se3_l2_robust, "Least-squares (L2 norm) solution to finding the optimal SE(3) transform\n between two reference frames using RANSAC and the \"quaternion\" or Horn's\n method \n\n  The optimal transformation `q` fulfills \n\n, that is, the\n  transformation of frame `other` with respect to `this`.\n\n  \n\n \n The set of correspondences.\n \n\n Method parameters (see docs for TSE3RobustParams)\n \n\n Results: transformation, scale, etc.\n\n \n True if the minimum number of correspondences was found, false\n otherwise.\n \n\n Implemented by FAMD, 2008. Re-factored by JLBC, 2015.\n \n\n [New in MRPT 1.3.0] This function replaces\n mrpt::scanmatching::leastSquareErrorRigidTransformation6DRANSAC()\n \n\n se2_l2, se3_l2\n\nC++: mrpt::tfest::se3_l2_robust(const class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::tfest::TSE3RobustParams &, struct mrpt::tfest::TSE3RobustResult &) --> bool", pybind11::arg("in_correspondences"), pybind11::arg("in_params"), pybind11::arg("out_results"));

}

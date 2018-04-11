#ifndef CVD_FAST_CORNER_H
#define CVD_FAST_CORNER_H

#include <vector>
#include <utility>

#include <cvd/byte.h>
#include <cvd/image.h>

namespace CVD
{
  
	/** Perform non-maximal suppression on a set of FAST features. This cleans up
	areas where there are multiple adjacent features, using a computed score
	function to leave only the 'best' features. This function is typically called
	immediately after a call to fast_corner_detect() (or one of its variants). This 
	uses the scoring function given in the paper given in @ref fast_corner_detect_9:

	@param im The image used to generate the FAST features
	@param corners The FAST features previously detected (e.g. by calling fast_corner_detect())
	@param  barrier The barrier used to calculate the score, which should be the same as that passed to fast_corner_detect()
	@param max_corners Vector to be filled with the new list of locally maximal corners.
	@ingroup  gVision
	*/
	void fast_nonmax( const BasicImage<byte>& im, const std::vector<ImageRef>& corners, int barrier, std::vector<ImageRef>& max_corners);

	/** Perform non-maximal suppression on a set of FAST features, also returning
	the score for each remaining corner. This function cleans up areas where
	there are multiple adjacent features, using a computed score function to leave
	only the 'best' features. This function is typically called immediately after
	a call to fast_corner_detect() (or one of its variants).
	
	@param im The image used to generate the FAST features
	@param corners The FAST features previously detected (e.g. by calling fast_corner_detect())
	@param barrier The barrier used to calculate the score, which should be the same as that passed to fast_corner_detect()
	@param max_corners Vector to be filled with the new list of
	       locally maximal corners, and their scores.
     	   <code>non_maxcorners[i].first</code> gives the location and 
	       <code>non_maxcorners[i].second</code> gives the score (higher is better).
	
	@ingroup  gVision
	*/
	void fast_nonmax_with_scores( const BasicImage<byte>& im, const std::vector<ImageRef>& corners, int barrier, std::vector<std::pair<ImageRef,int> >& max_corners);

	/// Perform tree based 7 point FAST feature detection. This is more like an edge detector.
	/// If you use this, please cite the paper given in @ref fast_corner_detect_9
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of corner locations
	/// @param barrier	Corner detection threshold
	/// @ingroup	gVision
	void fast_corner_detect_7(const BasicImage<byte>& im, std::vector<ImageRef>& corners, int barrier);

	
	/// Compute the 7 point score (as the maximum threshold at which the point will still be detected) for
	/// a std::vector of features.
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of corner locations
	/// @param barrier	Initial corner detection threshold. Using the same threshold as for corner detection will produce the 
	///                 quickest results, but any lower value (e.g. 0) will produce correct results.
	/// @ingroup	gVision
	void fast_corner_score_7(const BasicImage<byte>& i, const std::vector<ImageRef>& corners, int b, std::vector<int>& scores);


	/// Perform tree based 8 point FAST feature detection. This is more like an edge detector.
	/// If you use this, please cite the paper given in @ref fast_corner_detect_9
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of corner locations
	/// @param barrier	Corner detection threshold
	/// @ingroup	gVision
	void fast_corner_detect_8(const BasicImage<byte>& im, std::vector<ImageRef>& corners, int barrier);
	
	/// Compute the 8 point score (as the maximum threshold at which the point will still be detected) for
	/// a std::vector of features.
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of corner locations
	/// @param barrier	Initial corner detection threshold. Using the same threshold as for corner detection will produce the 
	///                 quickest results, but any lower value (e.g. 0) will produce correct results.
	/// @ingroup	gVision
	void fast_corner_score_8(const BasicImage<byte>& i, const std::vector<ImageRef>& corners, int b, std::vector<int>& scores);

	
	/** Perform tree based 9 point FAST feature detection as described in:
	    Machine Learning for High Speed Corner Detection, E. Rosten and T. Drummond.
		Results show that this is both the fastest and the best of the detectors.
		If you use this in published work, please cite:
	
\verbatim
@inproceedings{rosten2006machine,
	title       =    "Machine Learning for High Speed Corner Detection",
	author      =    "Edward Rosten and Tom Drummond",
	year        =    "2006",     
	month       =    "May",     
	booktitle   =    "9th European Conference on Computer Vision",
}
\endverbatim

	    @param im 		The input image
	    @param corners	The resulting container of corner locations
	    @param barrier	Corner detection threshold
	    @ingroup	gVision
	**/
	void fast_corner_detect_9(const BasicImage<byte>& im, std::vector<ImageRef>& corners, int barrier);

	/// Compute the 9 point score (as the maximum threshold at which the point will still be detected) for
	/// a std::vector of features.
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of corner locations
	/// @param barrier	Initial corner detection threshold. Using the same threshold as for corner detection will produce the 
	///                 quickest results, but any lower value (e.g. 0) will produce correct results.
	/// @ingroup	gVision
	void fast_corner_score_9(const BasicImage<byte>& i, const std::vector<ImageRef>& corners, int b, std::vector<int>& scores);
	

	///Perform FAST-9 corner detection (see @ref fast_corner_detect_9), with nonmaximal
	///suppression (see @ref fast_corner_score_9 and @ref nonmax_suppression)
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of locally maximal corner locations
	/// @param barrier	Corner detection threshold
	/// @ingroup	gVision
	void fast_corner_detect_9_nonmax(const BasicImage<byte>& im, std::vector<ImageRef>& max_corners, int barrier);

	/// Perform tree based 10 point FAST feature detection
	/// If you use this, please cite the paper given in @ref fast_corner_detect
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of corner locations
	/// @param barrier	Corner detection threshold
	/// @ingroup	gVision
	void fast_corner_detect_10(const BasicImage<byte>& im, std::vector<ImageRef>& corners, int barrier);

	/// Compute the 10 point score (as the maximum threshold at which the point will still be detected) for
	/// a std::vector of features.
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of corner locations
	/// @param barrier	Initial corner detection threshold. Using the same threshold as for corner detection will produce the 
	///                 quickest results, but any lower value (e.g. 0) will produce correct results.
	/// @ingroup	gVision
	void fast_corner_score_10(const BasicImage<byte>& i, const std::vector<ImageRef>& corners, int b, std::vector<int>& scores);

	/// Perform tree based 11 point FAST feature detection
	/// If you use this, please cite the paper given in @ref fast_corner_detect_9
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of corner locations
	/// @param barrier	Corner detection threshold
	/// @ingroup	gVision
	void fast_corner_detect_11(const BasicImage<byte>& im, std::vector<ImageRef>& corners, int barrier);

	/// Compute the 11 point score (as the maximum threshold at which the point will still be detected) for
	/// a std::vector of features.
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of corner locations
	/// @param barrier	Initial corner detection threshold. Using the same threshold as for corner detection will produce the 
	///                 quickest results, but any lower value (e.g. 0) will produce correct results.
	/// @ingroup	gVision
	void fast_corner_score_11(const BasicImage<byte>& i, const std::vector<ImageRef>& corners, int b, std::vector<int>& scores);

	/// Perform tree based 12 point FAST feature detection
	/// If you use this, please cite the paper given in @ref fast_corner_detect_9
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of corner locations
	/// @param barrier	Corner detection threshold
	/// @ingroup	gVision
	void fast_corner_detect_12(const BasicImage<byte>& im, std::vector<ImageRef>& corners, int barrier);

	/// Compute the 11 point score (as the maximum threshold at which the point will still be detected) for
	/// a std::vector of features.
	///
	/// @param im 		The input image
	/// @param corners	The resulting container of corner locations
	/// @param barrier	Initial corner detection threshold. Using the same threshold as for corner detection will produce the 
	///                 quickest results, but any lower value (e.g. 0) will produce correct results.
	/// @ingroup	gVision
	void fast_corner_score_12(const BasicImage<byte>& i, const std::vector<ImageRef>& corners, int b, std::vector<int>& scores);



	/// The 16 offsets from the centre pixel used in FAST feature detection.
	///
	/// @ingroup gVision
	extern const ImageRef fast_pixel_ring[16];
}

#endif

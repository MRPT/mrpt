/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/vision/TKeyPoint.h>
#include <mrpt/vision/utils.h>

namespace mrpt::vision
{
/** The central class from which images can be analyzed in search of different
 *kinds of interest points and descriptors computed for them.
 *  To extract features from an image, create an instance of
 *CFeatureExtraction,
 *   fill out its CFeatureExtraction::options field, including the algorithm to
 *use (see
 *   CFeatureExtraction::TOptions::featsType), and call
 *CFeatureExtraction::detectFeatures.
 *  This will return a set of features of the class mrpt::vision::CFeature,
 *which include
 *   details for each interest point as well as the desired descriptors and/or
 *patches.
 *
 *  By default, a 21x21 patch is extracted for each detected feature. If the
 *patch is not needed,
 *   set patchSize to 0 in CFeatureExtraction::options
 *
 *  The implemented <b>detection</b> algorithms are (see
 *CFeatureExtraction::TOptions::featsType):
 *		- KLT (Kanade-Lucas-Tomasi): A detector (no descriptor vector).
 *		- Harris: A detector (no descriptor vector).
 *		- BCD (Binary Corner Detector): A detector (no descriptor vector) (Not
 *implemented yet).
 *		- SIFT: An implementation of the SIFT detector and descriptor. The
 *implemention may be selected with
 *CFeatureExtraction::TOptions::SIFTOptions::implementation.
 *		- SURF: OpenCV's implementation of SURF detector and descriptor.
 *		- The FAST feature detector (OpenCV's implementation)
 *		- The FASTER (9,10,12) detectors (Edward Rosten's libcvd implementation
 *optimized for SSE2).
 *
 *  Additionally, given a list of interest points onto an image, the following
 *   <b>descriptors</b> can be computed for each point by calling
 *CFeatureExtraction::computeDescriptors :
 *		- SIFT descriptor (Lowe's descriptors).
 *		- SURF descriptor (OpenCV's implementation - Requires OpenCV 1.1.0 from
 *SVN
 *or later).
 *		- Intensity-domain spin images (SpinImage): Creates a vector descriptor
 *with the 2D histogram as a single row.
 *		- A circular patch in polar coordinates (Polar images): The matrix
 *descriptor is a 2D polar image centered at the interest point.
 *		- A log-polar image patch (Log-polar images): The matrix descriptor is
 *the
 *2D log-polar image centered at the interest point.
 *
 *
 *  Apart from the normal entry point \a detectFeatures(), these other
 *low-level static methods are provided for convenience:
 *   - CFeatureExtraction::detectFeatures_SSE2_FASTER9()
 *   - CFeatureExtraction::detectFeatures_SSE2_FASTER10()
 *   - CFeatureExtraction::detectFeatures_SSE2_FASTER12()
 *
 * \note The descriptor "Intensity-domain spin images" is described in "A
 *sparse texture representation using affine-invariant regions", S Lazebnik, C
 *Schmid, J Ponce, 2003 IEEE Computer Society Conference on Computer Vision.
 * \sa mrpt::vision::CFeature
 * \ingroup mrptvision_features
 */
class CFeatureExtraction
{
   public:
	/** Timelogger: disabled by default */
	mrpt::system::CTimeLogger profiler{false};

	enum TSIFTImplementation
	{
		LoweBinary = 0 /* obsolete */,
		CSBinary /* obsolete */,
		VedaldiBinary /* obsolete */,
		Hess /* obsolete */,
		OpenCV /* DEFAULT */
	};

	/** The set of parameters for all the detectors & descriptor algorithms */
	struct TOptions : public mrpt::config::CLoadableOptions
	{
		TOptions() = default;
		TOptions(const TKeyPointMethod ft) : TOptions() { featsType = ft; }

		// See base docs
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& c,
			const std::string& s) override;
		void dumpToTextStream(std::ostream& out) const override;

		/** Type of the extracted features */
		TKeyPointMethod featsType{featKLT};

		/** Size of the patch to extract, or 0 if no patch is desired
		 * (default=21).
		 */
		unsigned int patchSize{21};

		/** Whether to use a mask for determining the regions where not to look
		 * for keypoints (default=false).
		 */
		bool useMask{false};

		/** Whether to add the found features to the input feature list or clear
		 * it before adding them (default=false).
		 */
		bool addNewFeatures{false};

		/** Indicates if subpixel accuracy is desired for the extracted points
		 * (only applicable to KLT and Harris features)
		 */
		bool FIND_SUBPIXEL{true};

		/** KLT Options */
		struct TKLTOptions
		{
			/** size of the block of pixels used */
			int radius{5};
			/** (default=0.05f) for rejecting weak local maxima
			 * (with min_eig < threshold*max(eig_image) */
			float threshold{0.05f};
			/** minimum distance between features */
			float min_distance{5.0f};
		} KLTOptions;

		/** Harris Options */
		struct THarrisOptions
		{
			/** (default=0.005) for rejecting weak local maxima
			 * (with min_eig < threshold*max(eig_image)) */
			float threshold{0.005f};
			/** standard deviation for the gaussian smoothing function */
			float sigma{3.0f};
			/** size of the block of pixels used */
			int radius{3};
			/** minimum distance between features */
			float min_distance{5.0f};
			double k{0.04};
		} harrisOptions;

		/** BCD Options */
		struct TBCDOptions
		{
		} BCDOptions;

		/** FAST and FASTER Options */
		struct TFASTOptions
		{
			/** default= 20 */
			int threshold = 20;
			/** (default=5) minimum distance between features (in pixels) */
			unsigned int min_distance = 5;
			/** Default = true */
			bool nonmax_suppression{true};
			/** (default=false) If true, use CImage::KLT_response to compute the
			 * response at each point.
			 */
			bool use_KLT_response{false};
			/** Used if use_KLT_response==true */
			int KLT_response_half_win_size{4};
		} FASTOptions;

		/** ORB Options */
		struct TORBOptions
		{
			TORBOptions() = default;
			size_t n_levels{1};
			size_t min_distance{0};
			float scale_factor{1.2f};
			bool extract_patch{false};
		} ORBOptions;

		/** SIFT Options  */
		struct TSIFTOptions
		{
			TSIFTOptions() = default;
			TSIFTImplementation implementation{OpenCV};  //!< Default: OpenCV
			int octaveLayers{3};
			double threshold{0.04};  //!< default= 0.04
			double edgeThreshold{10};  //!< default= 10
		} SIFTOptions;

		/** SURF Options */
		struct TSURFOptions
		{
			TSURFOptions() = default;

			/** Compute the rotation invariant SURF */
			bool rotation_invariant{true};
			int hessianThreshold{600};
			int nOctaves{2};
			int nLayersPerOctave{4};
		} SURFOptions;

		/** SpinImages Options */
		struct TSpinImagesOptions
		{
			/** Number of bins in the "intensity" axis of the 2D histogram
			 * (default=10). */
			unsigned int hist_size_intensity{10};
			/** Number of bins in the "distance" axis of the 2D histogram
			 * (default=10).*/
			unsigned int hist_size_distance{10};
			/** Standard deviation in "distance", used for the "soft histogram"
			 * (default=0.4 pixels) */
			float std_dist{.4f};
			/** Standard deviation in "intensity", used for the "soft histogram"
			 * (default=20 units [0,255]) */
			float std_intensity{20};
			/** Maximum radius of the area of which the histogram is built, in
			 * pixel units (default=20 pixels) */
			unsigned int radius{20};
		} SpinImagesOptions;

		/** PolarImagesOptions options  */
		struct TPolarImagesOptions
		{
			/** Number of bins in the "angular" axis of the polar image
			 * (default=8). */
			unsigned int bins_angle{8};
			/** Number of bins in the "distance" axis of the polar image
			 * (default=6). */
			unsigned int bins_distance{6};
			/** Maximum radius of the area of which the polar image is built, in
			 * pixel units (default=20 pixels) */
			unsigned int radius{20};
		} PolarImagesOptions;

		/** LogPolarImagesOptions Options
		 */
		struct TLogPolarImagesOptions
		{
			/** Maximum radius of the area of which the log polar image is
			 * built, in pixel units (default=30 pixels) */
			unsigned int radius{30};
			/** (default=16) Log-Polar image patch will have dimensions WxH,
			 * with: W=num_angles,  H= rho_scale*log(radius) */
			unsigned int num_angles{16};
			/** (default=5) Log-Polar image patch will have dimensions WxH,
			 * with:  W=num_angles,  H=rho_scale * log(radius) */
			double rho_scale{5};
		} LogPolarImagesOptions;

		// # added by Raghavender Sahdev
		/** AKAZEOptions Options */
		struct TAKAZEOptions
		{
			/** AKAZE::DESCRIPTOR_MLDB maps to 5 in open cv;
			 * http://docs.opencv.org/trunk/d8/d30/classcv_1_1AKAZE.html */
			int descriptor_type{5};
			int descriptor_size{0};
			int descriptor_channels{3};
			float threshold{0.001f};
			int nOctaves{4};
			int nOctaveLayers{4};
			/** KAZE::DIFF_PM_G2 maps to 1;
			 * http://docs.opencv.org/trunk/d3/d61/classcv_1_1KAZE.html */
			int diffusivity{1};
		} AKAZEOptions;

		/** LSDOptions Options */
		struct TLSDOptions
		{
			int scale{2};
			int nOctaves{1};
		} LSDOptions;

		/** BLDOptions Descriptor Options */
		struct TBLDOptions
		{
			int ksize_{11};
			int reductionRatio{2};
			int widthOfBand{7};
			int numOfOctave{1};
		} BLDOptions;

		/** LATCHOptions Descriptor */
		struct TLATCHOptions
		{
			int bytes{32};
			bool rotationInvariance{true};
			int half_ssd_size{3};
		} LATCHOptions;
	};

	/** Set all the parameters of the desired method here before calling
	 * detectFeatures() */
	TOptions options;

	/** Extract features from the image based on the method defined in
	 * TOptions. \param img (input) The image from where to extract the
	 * images. \param feats (output) A complete list of features (containing
	 * a patch for each one of them if options.patchsize > 0). \param
	 * nDesiredFeatures (op. input) Number of features to be extracted.
	 * Default: all possible.
	 *
	 * \sa computeDescriptors
	 */
	void detectFeatures(
		const mrpt::img::CImage& img, CFeatureList& feats,
		const unsigned int init_ID = 0, const unsigned int nDesiredFeatures = 0,
		const TImageROI& ROI = TImageROI());

	/** Compute one (or more) descriptors for the given set of interest
	 * points onto the image, which may have been filled out manually or
	 * from \a detectFeatures \param in_img (input) The image from where to
	 * compute the descriptors. \param inout_features (input/output) The
	 * list of features whose descriptors are going to be computed. \param
	 * in_descriptor_list (input) The bitwise OR of one or several
	 * descriptors defined in TDescriptorType.
	 *
	 *  Each value in "in_descriptor_list" represents one descriptor to be
	 * computed, for example:
	 *  \code
	 *    // This call will compute both, SIFT and Spin-Image descriptors
	 * for a list of feature points lstFeats. fext.computeDescriptors(img,
	 * lstFeats, descSIFT | descSpinImages ); \endcode
	 *
	 * \note The SIFT descriptors for already located features can only be
	 * computed through the Hess and
	 *        CSBinary implementations which may be specified in
	 * CFeatureExtraction::TOptions::SIFTOptions.
	 *
	 * \note This call will also use additional parameters from \a options
	 */
	void computeDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& inout_features,
		TDescriptorType in_descriptor_list);

	/** @name Static methods with low-level detector functionality
		@{ */

	/** A SSE2-optimized implementation of FASTER-9 (requires img to be
	 * grayscale). If SSE2 is not available, it gratefully falls back to a
	 * non-optimized version.
	 *
	 *  Only the pt.{x,y} fields are filled out for each feature: the rest
	 * of fields are left <b>uninitialized</b> and their content is
	 * <b>undefined</b>.
	 *  Note that (x,y) are already scaled to the 0-level image coordinates
	 * if octave>0, by means of:
	 *
	 *  \code
	 *    pt.x = detected.x << octave;
	 *    pt.y = detected.y << octave;
	 *  \endcode
	 *
	 * If \a append_to_list is true, the \a corners list is not cleared
	 * before adding the newly detected feats.
	 *
	 * If a valid pointer is provided for \a out_feats_index_by_row, upon
	 * return you will find a vector with
	 *  as many entries as rows in the image (the real number of rows,
	 * disregarding the value of \a octave).
	 * The number in each entry is the 0-based index (in \a corners) of
	 *  the first feature that falls in that line of the image. This index
	 * can be used to fasten looking for correspondences.
	 *
	 * \ingroup mrptvision_features
	 */
	static void detectFeatures_SSE2_FASTER9(
		const mrpt::img::CImage& img, TKeyPointList& corners,
		const int threshold = 20, bool append_to_list = false,
		uint8_t octave = 0,
		std::vector<size_t>* out_feats_index_by_row = nullptr);

	/** Just like \a detectFeatures_SSE2_FASTER9() for another version of
	 * the detector. \ingroup mrptvision_features */
	static void detectFeatures_SSE2_FASTER10(
		const mrpt::img::CImage& img, TKeyPointList& corners,
		const int threshold = 20, bool append_to_list = false,
		uint8_t octave = 0,
		std::vector<size_t>* out_feats_index_by_row = nullptr);

	/** Just like \a detectFeatures_SSE2_FASTER9() for another version of
	 * the detector. \ingroup mrptvision_features */
	static void detectFeatures_SSE2_FASTER12(
		const mrpt::img::CImage& img, TKeyPointList& corners,
		const int threshold = 20, bool append_to_list = false,
		uint8_t octave = 0,
		std::vector<size_t>* out_feats_index_by_row = nullptr);

	/** @} */

   private:
	/** Compute the SIFT descriptor of the provided features into the input
	image
	* \param in_img (input) The image from where to compute the descriptors.
	* \param in_features (input/output) The list of features whose
	descriptors are going to be computed.
	*
	* \note The SIFT descriptors for already located features can only be
	computed through the Hess and
			CSBinary implementations which may be specified in
	CFeatureExtraction::TOptions::SIFTOptions.
	*/
	void internal_computeSiftDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features);

	/** Compute the SURF descriptor of the provided features into the input
	 * image
	 * \param in_img (input) The image from where to compute the
	 * descriptors. \param in_features (input/output) The list of features
	 * whose descriptors are going to be computed.
	 */
	void internal_computeSurfDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features);

	/** Compute the ORB descriptor of the provided features into the input
	 * image \param in_img (input) The image from where to compute the
	 * descriptors. \param in_features (input/output) The list of features
	 * whose descriptors are going to be computed.
	 */
	void internal_computeORBDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features);

	/** Compute the intensity-domain spin images descriptor of the provided
	 * features into the input image
	 * \param in_img (input) The image from where to compute the
	 * descriptors. \param in_features (input/output) The list of features
	 * whose descriptors are going to be computed.
	 *
	 * \note Additional parameters from
	 * CFeatureExtraction::TOptions::SpinImagesOptions are used in this
	 * method.
	 */
	void internal_computeSpinImageDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features);

	/** Compute a polar-image descriptor of the provided features into the
	 * input image \param in_img (input) The image from where to compute the
	 * descriptors. \param in_features (input/output) The list of features
	 * whose descriptors are going to be computed.
	 *
	 * \note Additional parameters from
	 * CFeatureExtraction::TOptions::PolarImagesOptions are used in this
	 * method.
	 */
	void internal_computePolarImageDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features);

	/** Compute a log-polar image descriptor of the provided features into
	 * the input image \param in_img (input) The image from where to compute
	 * the descriptors. \param in_features (input/output) The list of
	 * features whose descriptors are going to be computed.
	 *
	 * \note Additional parameters from
	 * CFeatureExtraction::TOptions::LogPolarImagesOptions are used in this
	 * method.
	 */
	void internal_computeLogPolarImageDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features);
	/** Compute a BLD descriptor of the provided features into the input
	 * image \param in_img (input) The image from where to compute the
	 * descriptors. \param in_features (input/output) The list of features
	 * whose descriptors are going to be computed.
	 *
	 * \note Additional parameters from
	 * CFeatureExtraction::TOptions::LogPolarImagesOptions are used in this
	 * method.
	 */
	void internal_computeBLDLineDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features);
	/** Compute a LATCH descriptor of the provided features into the input
	 * image \param in_img (input) The image from where to compute the
	 * descriptors. \param in_features (input/output) The list of features
	 * whose descriptors are going to be computed.
	 *
	 * \note Additional parameters from
	 * CFeatureExtraction::TOptions::LogPolarImagesOptions are used in this
	 * method.
	 */
	void internal_computeLATCHDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features);

	/** Extract features from the image based on the KLT method.
	 * \param img The image from where to extract the images.
	 * \param feats The list of extracted features.
	 * \param nDesiredFeatures Number of features to be extracted. Default:
	 * authomatic.
	 */
	void extractFeaturesKLT(
		const mrpt::img::CImage& img, CFeatureList& feats,
		unsigned int init_ID = 0, unsigned int nDesiredFeatures = 0,
		const TImageROI& ROI = TImageROI());

	// ------------------------------------------------------------------------------------
	//											SIFT
	// ------------------------------------------------------------------------------------
	/** Extract features from the image based on the SIFT method.
	 * \param img The image from where to extract the images.
	 * \param feats The list of extracted features.
	 * \param nDesiredFeatures Number of features to be extracted. Default:
	 * authomatic.
	 * \param ROI (op. input) Region of Interest. Default: All the image.
	 */
	void extractFeaturesSIFT(
		const mrpt::img::CImage& img, CFeatureList& feats,
		unsigned int init_ID = 0, unsigned int nDesiredFeatures = 0,
		const TImageROI& ROI = TImageROI());

	// ------------------------------------------------------------------------------------
	//											ORB
	// ------------------------------------------------------------------------------------
	/** Extract features from the image based on the ORB method.
	 * \param img The image from where to extract the images.
	 * \param feats The list of extracted features.
	 * \param nDesiredFeatures Number of features to be extracted. Default:
	 * authomatic.
	 */
	void extractFeaturesORB(
		const mrpt::img::CImage& img, CFeatureList& feats,
		const unsigned int init_ID = 0, const unsigned int nDesiredFeatures = 0,
		const TImageROI& ROI = TImageROI());

	// ------------------------------------------------------------------------------------
	//											SURF
	// ------------------------------------------------------------------------------------
	/** Extract features from the image based on the SURF method.
	 * \param img The image from where to extract the images.
	 * \param feats The list of extracted features.
	 * \param nDesiredFeatures Number of features to be extracted. Default:
	 * authomatic.
	 */
	void extractFeaturesSURF(
		const mrpt::img::CImage& img, CFeatureList& feats,
		unsigned int init_ID = 0, unsigned int nDesiredFeatures = 0,
		const TImageROI& ROI = TImageROI());

	// ------------------------------------------------------------------------------------
	//											FAST
	// ------------------------------------------------------------------------------------
	/** Extract features from the image based on the FAST method (OpenCV impl.)
	 * \param img The image from where to extract the images.
	 * \param feats The list of extracted features.
	 * \param nDesiredFeatures Number of features to be extracted. Default:
	 * authomatic.
	 */
	void extractFeaturesFAST(
		const mrpt::img::CImage& img, CFeatureList& feats,
		unsigned int init_ID = 0, unsigned int nDesiredFeatures = 0);

	/** Edward's "FASTER & Better" detector, N=9,10,12. (libCVD impl.) */
	void extractFeaturesFASTER_N(
		const int N, const mrpt::img::CImage& img, CFeatureList& feats,
		unsigned int init_ID = 0, unsigned int nDesiredFeatures = 0,
		const TImageROI& ROI = TImageROI());

	// # added by Raghavender Sahdev
	//-------------------------------------------------------------------------------------
	//                               AKAZE
	//-------------------------------------------------------------------------------------
	/** Extract features from the image based on the AKAZE method.
	 * \param img The image from where to extract the images.
	 * \param feats The list of extracted features.
	 * \param nDesiredFeatures Number of features to be extracted. Default:
	 * authomatic.
	 */
	void extractFeaturesAKAZE(
		const mrpt::img::CImage& inImg, CFeatureList& feats,
		unsigned int init_ID, unsigned int nDesiredFeatures,
		const TImageROI& ROI = TImageROI());

	//-------------------------------------------------------------------------------------
	//                               LSD
	//-------------------------------------------------------------------------------------
	/** Extract features from the image based on the LSD method.
	 * \param img The image from where to extract the images.
	 * \param feats The list of extracted features.
	 * \param nDesiredFeatures Number of features to be extracted. Default:
	 * authomatic.
	 */
	void extractFeaturesLSD(
		const mrpt::img::CImage& inImg, CFeatureList& feats,
		unsigned int init_ID, unsigned int nDesiredFeatures,
		const TImageROI& ROI = TImageROI());

};  // end of class
}  // namespace mrpt::vision

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/vision/utils.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/vision/TSimpleFeature.h>

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
		/** Initalizer */
		TOptions(const TFeatureType featsType = featKLT);

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		/** Type of the extracted features
		 */
		TFeatureType featsType;

		/** Size of the patch to extract, or 0 if no patch is desired
		 * (default=21).
		 */
		unsigned int patchSize;

		/** Whether to use a mask for determining the regions where not to look
		 * for keypoints (default=false).
		 */
		bool useMask;

		/** Whether to add the found features to the input feature list or clear
		 * it before adding them (default=false).
		 */
		bool addNewFeatures;

		/** Indicates if subpixel accuracy is desired for the extracted points
		 * (only applicable to KLT and Harris features)
		 */
		bool FIND_SUBPIXEL;

		/** KLT Options */
		struct TKLTOptions
		{
			int radius;  // size of the block of pixels used
			float threshold;  // (default=0.1) for rejecting weak local maxima
			// (with min_eig < threshold*max(eig_image))
			float min_distance;  // minimum distance between features
			bool tile_image;  // splits the image into 8 tiles and search for
			// the best points in all of them (distribute the
			// features over all the image)
		} KLTOptions;

		/** Harris Options */
		struct THarrisOptions
		{
			float threshold;  // (default=0.005) for rejecting weak local maxima
			// (with min_eig < threshold*max(eig_image))
			float k;  // k factor for the Harris algorithm
			float sigma;  // standard deviation for the gaussian smoothing
			// function
			int radius;  // size of the block of pixels used
			float min_distance;  // minimum distance between features
			bool tile_image;  // splits the image into 8 tiles and search for
			// the best points in all of them (distribute the
			// features over all the image)
		} harrisOptions;

		/** BCD Options */
		struct TBCDOptions
		{
		} BCDOptions;

		/** FAST and FASTER Options */
		struct TFASTOptions
		{
			int threshold;  //!< default= 20
			float min_distance;  //!< (default=5) minimum distance between
			//! features (in pixels)
			bool nonmax_suppression;  //!< Default = true
			bool use_KLT_response;  //!< (default=false) If true, use
			//! CImage::KLT_response to compute the
			//! response at each point instead of the
			//! FAST "standard response".
		} FASTOptions;

		/** ORB Options */
		struct TORBOptions
		{
			TORBOptions() = default;
			size_t n_levels{8};
			size_t min_distance{0};
			float scale_factor{1.2f};
			bool extract_patch{false};
		} ORBOptions;

		/** SIFT Options  */
		struct TSIFTOptions
		{
			TSIFTOptions() = default;
			TSIFTImplementation implementation{OpenCV};  //!< Default: OpenCV
			double threshold{0.04};  //!< default= 0.04
			double edgeThreshold{10};  //!< default= 10
		} SIFTOptions;

		struct TSURFOptions
		{
			TSURFOptions() = default;

			/** SURF Options
			 */
			bool rotation_invariant{
				true};  //!< Compute the rotation invariant SURF
			//!(dim=128) if set to true (default), or
			//! the smaller uSURF otherwise (dim=64)
			int hessianThreshold{600};  //!< Default: 600
			int nOctaves{2};  //!< Default: 2
			int nLayersPerOctave{4};  //!< Default: 4
		} SURFOptions;

		struct TSpinImagesOptions
		{
			/** SpinImages Options
			 */
			unsigned int hist_size_intensity;  //!< Number of bins in the
			//!"intensity" axis of the 2D
			//! histogram (default=10).
			unsigned int hist_size_distance;  //!< Number of bins in the
			//!"distance" axis of the 2D
			//! histogram (default=10).
			float std_dist;  //!< Standard deviation in "distance", used for the
			//!"soft histogram" (default=0.4 pixels)
			float std_intensity;  //!< Standard deviation in "intensity", used
			//! for the "soft histogram" (default=20 units
			//![0,255])
			unsigned int radius;  //!< Maximum radius of the area of which the
			//! histogram is built, in pixel units
			//!(default=20 pixels)
		} SpinImagesOptions;

		/** PolarImagesOptions Options
		 */
		struct TPolarImagesOptions
		{
			unsigned int bins_angle;  //!< Number of bins in the "angular" axis
			//! of the polar image (default=8).
			unsigned int bins_distance;  //!< Number of bins in the "distance"
			//! axis of the polar image (default=6).
			unsigned int radius;  //!< Maximum radius of the area of which the
			//! polar image is built, in pixel units
			//!(default=20 pixels)
		} PolarImagesOptions;

		/** LogPolarImagesOptions Options
		 */
		struct TLogPolarImagesOptions
		{
			unsigned int radius;  //!< Maximum radius of the area of which the
			//! log polar image is built, in pixel units
			//!(default=30 pixels)
			unsigned int num_angles;  //!< (default=16) Log-Polar image patch
			//! will have dimensions WxH, with:
			//! W=num_angles,  H= rho_scale *
			//! log(radius)
			double rho_scale;  //!< (default=5) Log-Polar image patch will have
			//! dimensions WxH, with:  W=num_angles,  H=
			//! rho_scale * log(radius)
		} LogPolarImagesOptions;

		// # added by Raghavender Sahdev
		/** AKAZEOptions Options
		 */
		struct TAKAZEOptions
		{
			int descriptor_type;
			int descriptor_size;
			int descriptor_channels;
			float threshold;
			int nOctaves;
			int nOctaveLayers;
			int diffusivity;
		} AKAZEOptions;

		/** LSDOptions Options
		 */
		struct TLSDOptions
		{
			int scale;
			int nOctaves;
		} LSDOptions;

		/** BLDOptions Descriptor Options
		 */
		struct TBLDOptions
		{
			int ksize_;
			int reductionRatio;
			int widthOfBand;
			int numOfOctave;

		} BLDOptions;

		/** LATCHOptions Descriptor
		 */
		struct TLATCHOptions
		{
			int bytes;  // = 32,
			bool rotationInvariance;  // = true,
			int half_ssd_size;  // = 3
		} LATCHOptions;
	};

	TOptions options;  //!< Set all the parameters of the desired method here
	//! before calling "detectFeatures"

	/** Virtual destructor.
	 */
	virtual ~CFeatureExtraction();

	/** Extract features from the image based on the method defined in TOptions.
	 * \param img (input) The image from where to extract the images.
	 * \param feats (output) A complete list of features (containing a patch for
	 * each one of them if options.patchsize > 0).
	 * \param nDesiredFeatures (op. input) Number of features to be extracted.
	 * Default: all possible.
	 *
	 * \sa computeDescriptors
	 */
	void detectFeatures(
		const mrpt::img::CImage& img, CFeatureList& feats,
		const unsigned int init_ID = 0, const unsigned int nDesiredFeatures = 0,
		const TImageROI& ROI = TImageROI()) const;

	/** Compute one (or more) descriptors for the given set of interest points
	 * onto the image, which may have been filled out manually or from \a
	 * detectFeatures
	 * \param in_img (input) The image from where to compute the descriptors.
	 * \param inout_features (input/output) The list of features whose
	 * descriptors are going to be computed.
	 * \param in_descriptor_list (input) The bitwise OR of one or several
	 * descriptors defined in TDescriptorType.
	 *
	 *  Each value in "in_descriptor_list" represents one descriptor to be
	 * computed, for example:
	 *  \code
	 *    // This call will compute both, SIFT and Spin-Image descriptors for a
	 * list of feature points lstFeats.
	 *    fext.computeDescriptors(img, lstFeats, descSIFT | descSpinImages );
	 *  \endcode
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
		TDescriptorType in_descriptor_list) const;

#if 0  // Delete? see comments in .cpp
			/** Extract more features from the image (apart from the provided ones) based on the method defined in TOptions.
			* \param img (input) The image from where to extract the images.
			* \param inList (input) The actual features in the image.
			* \param outList (output) The list of new features (containing a patch for each one of them if options.patchsize > 0).
			* \param nDesiredFeatures (op. input) Number of features to be extracted. Default: all possible.
			*
			*  \sa The more powerful class: mrpt::vision::CGenericFeatureTracker
			*/
			void  findMoreFeatures( const mrpt::img::CImage &img,
									const CFeatureList &inList,
									CFeatureList &outList,
									unsigned int nDesiredFeats = 0) const;
#endif

	/** @name Static methods with low-level detector functionality
		@{ */

	/** A SSE2-optimized implementation of FASTER-9 (requires img to be
	 * grayscale). If SSE2 is not available, it gratefully falls back to a
	 * non-optimized version.
	 *
	 *  Only the pt.{x,y} fields are filled out for each feature: the rest of
	 * fields are left <b>uninitialized</b> and their content is
	 * <b>undefined</b>.
	 *  Note that (x,y) are already scaled to the 0-level image coordinates if
	 * octave>0, by means of:
	 *
	 *  \code
	 *    pt.x = detected.x << octave;
	 *    pt.y = detected.y << octave;
	 *  \endcode
	 *
	 * If \a append_to_list is true, the \a corners list is not cleared before
	 * adding the newly detected feats.
	 *
	 * If a valid pointer is provided for \a out_feats_index_by_row, upon
	 * return you will find a vector with
	 *  as many entries as rows in the image (the real number of rows,
	 * disregarding the value of \a octave).
	 * The number in each entry is the 0-based index (in \a corners) of
	 *  the first feature that falls in that line of the image. This index can
	 * be used to fasten looking for correspondences.
	 *
	 * \ingroup mrptvision_features
	 */
	static void detectFeatures_SSE2_FASTER9(
		const mrpt::img::CImage& img, TSimpleFeatureList& corners,
		const int threshold = 20, bool append_to_list = false,
		uint8_t octave = 0,
		std::vector<size_t>* out_feats_index_by_row = nullptr);

	/** Just like \a detectFeatures_SSE2_FASTER9() for another version of the
	 * detector.
	 * \ingroup mrptvision_features */
	static void detectFeatures_SSE2_FASTER10(
		const mrpt::img::CImage& img, TSimpleFeatureList& corners,
		const int threshold = 20, bool append_to_list = false,
		uint8_t octave = 0,
		std::vector<size_t>* out_feats_index_by_row = nullptr);

	/** Just like \a detectFeatures_SSE2_FASTER9() for another version of the
	 * detector.
	 * \ingroup mrptvision_features */
	static void detectFeatures_SSE2_FASTER12(
		const mrpt::img::CImage& img, TSimpleFeatureList& corners,
		const int threshold = 20, bool append_to_list = false,
		uint8_t octave = 0,
		std::vector<size_t>* out_feats_index_by_row = nullptr);

	/** @} */

   private:
	/** Compute the SIFT descriptor of the provided features into the input
	image
	* \param in_img (input) The image from where to compute the descriptors.
	* \param in_features (input/output) The list of features whose descriptors
	are going to be computed.
	*
	* \note The SIFT descriptors for already located features can only be
	computed through the Hess and
			CSBinary implementations which may be specified in
	CFeatureExtraction::TOptions::SIFTOptions.
	*/
	void internal_computeSiftDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features) const;

	/** Compute the SURF descriptor of the provided features into the input
	 * image
	 * \param in_img (input) The image from where to compute the descriptors.
	 * \param in_features (input/output) The list of features whose descriptors
	 * are going to be computed.
	 */
	void internal_computeSurfDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features) const;

	/** Compute the ORB descriptor of the provided features into the input image
	 * \param in_img (input) The image from where to compute the descriptors.
	 * \param in_features (input/output) The list of features whose descriptors
	 * are going to be computed.
	 */
	void internal_computeORBDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features) const;

	/** Compute the intensity-domain spin images descriptor of the provided
	 * features into the input image
	 * \param in_img (input) The image from where to compute the descriptors.
	 * \param in_features (input/output) The list of features whose descriptors
	 * are going to be computed.
	 *
	 * \note Additional parameters from
	 * CFeatureExtraction::TOptions::SpinImagesOptions are used in this method.
	 */
	void internal_computeSpinImageDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features) const;

	/** Compute a polar-image descriptor of the provided features into the input
	 * image
	 * \param in_img (input) The image from where to compute the descriptors.
	 * \param in_features (input/output) The list of features whose descriptors
	 * are going to be computed.
	 *
	 * \note Additional parameters from
	 * CFeatureExtraction::TOptions::PolarImagesOptions are used in this method.
	 */
	void internal_computePolarImageDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features) const;

	/** Compute a log-polar image descriptor of the provided features into the
	 * input image
	 * \param in_img (input) The image from where to compute the descriptors.
	 * \param in_features (input/output) The list of features whose descriptors
	 * are going to be computed.
	 *
	 * \note Additional parameters from
	 * CFeatureExtraction::TOptions::LogPolarImagesOptions are used in this
	 * method.
	 */
	void internal_computeLogPolarImageDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features) const;
	/** Compute a BLD descriptor of the provided features into the input image
	 * \param in_img (input) The image from where to compute the descriptors.
	 * \param in_features (input/output) The list of features whose descriptors
	 * are going to be computed.
	 *
	 * \note Additional parameters from
	 * CFeatureExtraction::TOptions::LogPolarImagesOptions are used in this
	 * method.
	 */
	void internal_computeBLDLineDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features) const;
	/** Compute a LATCH descriptor of the provided features into the input image
	 * \param in_img (input) The image from where to compute the descriptors.
	 * \param in_features (input/output) The list of features whose descriptors
	 * are going to be computed.
	 *
	 * \note Additional parameters from
	 * CFeatureExtraction::TOptions::LogPolarImagesOptions are used in this
	 * method.
	 */
	void internal_computeLATCHDescriptors(
		const mrpt::img::CImage& in_img, CFeatureList& in_features) const;

	/** Extract features from the image based on the KLT method.
	 * \param img The image from where to extract the images.
	 * \param feats The list of extracted features.
	 * \param nDesiredFeatures Number of features to be extracted. Default:
	 * authomatic.
	 */
	void extractFeaturesKLT(
		const mrpt::img::CImage& img, CFeatureList& feats,
		unsigned int init_ID = 0, unsigned int nDesiredFeatures = 0,
		const TImageROI& ROI = TImageROI()) const;

	// ------------------------------------------------------------------------------------
	//											BCD
	// ------------------------------------------------------------------------------------
	/** Extract features from the image based on the BCD method.
	 * \param img The image from where to extract the images.
	 * \param feats The list of extracted features.
	 * \param nDesiredFeatures Number of features to be extracted. Default:
	 * authomatic.
	 * \param ROI (op. input) Region of Interest. Default: All the image.
	 */
	void extractFeaturesBCD(
		const mrpt::img::CImage& img, CFeatureList& feats,
		unsigned int init_ID = 0, unsigned int nDesiredFeatures = 0,
		const TImageROI& ROI = TImageROI()) const;

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
		const TImageROI& ROI = TImageROI()) const;

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
		const TImageROI& ROI = TImageROI()) const;

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
		const TImageROI& ROI = TImageROI()) const;

	// ------------------------------------------------------------------------------------
	//											FAST
	// ------------------------------------------------------------------------------------
	/** Extract features from the image based on the FAST method.
	 * \param img The image from where to extract the images.
	 * \param feats The list of extracted features.
	 * \param nDesiredFeatures Number of features to be extracted. Default:
	 * authomatic.
	 */
	void extractFeaturesFAST(
		const mrpt::img::CImage& img, CFeatureList& feats,
		unsigned int init_ID = 0, unsigned int nDesiredFeatures = 0,
		const TImageROI& ROI = TImageROI(),
		const mrpt::math::CMatrixBool* mask = nullptr) const;

	/** Edward's "FASTER & Better" detector, N=9,10,12 */
	void extractFeaturesFASTER_N(
		const int N, const mrpt::img::CImage& img, CFeatureList& feats,
		unsigned int init_ID = 0, unsigned int nDesiredFeatures = 0,
		const TImageROI& ROI = TImageROI()) const;

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
		const TImageROI& ROI = TImageROI()) const;

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
		const TImageROI& ROI = TImageROI()) const;

	/** Converts a sequence of openCV features into an MRPT feature list.
	 * \param features The sequence of features.
	 * \param list [in-out] The list of MRPT features.
	 * \param init_ID [in][optional] The initial ID for the features (default =
	 * 0).
	 * \param ROI [in][optional] The initial ID for the features (default =
	 * empty ROI -> not used).
	 */
	void convertCvSeqInCFeatureList(
		void* features, CFeatureList& list, unsigned int init_ID = 0,
		const TImageROI& ROI = TImageROI()) const;

};  // end of class
}  // namespace mrpt::vision

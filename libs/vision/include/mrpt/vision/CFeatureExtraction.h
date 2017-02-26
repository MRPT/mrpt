/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CFeatureExtraction_H
#define CFeatureExtraction_H

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/vision/utils.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/vision/TSimpleFeature.h>

namespace mrpt
{
	namespace vision
	{
		/** The central class from which images can be analyzed in search of different kinds of interest points and descriptors computed for them.
		  *  To extract features from an image, create an instance of CFeatureExtraction,
		  *   fill out its CFeatureExtraction::options field, including the algorithm to use (see
		  *   CFeatureExtraction::TOptions::featsType), and call CFeatureExtraction::detectFeatures.
		  *  This will return a set of features of the class mrpt::vision::CFeature, which include
		  *   details for each interest point as well as the desired descriptors and/or patches.
		  *
		  *  By default, a 21x21 patch is extracted for each detected feature. If the patch is not needed,
		  *   set patchSize to 0 in CFeatureExtraction::options
		  *
		  *  The implemented <b>detection</b> algorithms are (see CFeatureExtraction::TOptions::featsType):
		  *		- KLT (Kanade-Lucas-Tomasi): A detector (no descriptor vector).
		  *		- Harris: A detector (no descriptor vector).
		  *		- BCD (Binary Corner Detector): A detector (no descriptor vector) (Not implemented yet).
		  *		- SIFT: An implementation of the SIFT detector and descriptor. The implemention may be selected with CFeatureExtraction::TOptions::SIFTOptions::implementation.
		  *		- SURF: OpenCV's implementation of SURF detector and descriptor.
		  *		- The FAST feature detector (OpenCV's implementation)
		  *		- The FASTER (9,10,12) detectors (Edward Rosten's libcvd implementation optimized for SSE2).
		  *
		  *  Additionally, given a list of interest points onto an image, the following
		  *   <b>descriptors</b> can be computed for each point by calling CFeatureExtraction::computeDescriptors :
		  *		- SIFT descriptor (Lowe's descriptors).
		  *		- SURF descriptor (OpenCV's implementation - Requires OpenCV 1.1.0 from SVN or later).
		  *		- Intensity-domain spin images (SpinImage): Creates a vector descriptor with the 2D histogram as a single row.
		  *		- A circular patch in polar coordinates (Polar images): The matrix descriptor is a 2D polar image centered at the interest point.
		  *		- A log-polar image patch (Log-polar images): The matrix descriptor is the 2D log-polar image centered at the interest point.
		  *
		  *
		  *  Apart from the normal entry point \a detectFeatures(), these other low-level static methods are provided for convenience:
		  *   - CFeatureExtraction::detectFeatures_SSE2_FASTER9()
		  *   - CFeatureExtraction::detectFeatures_SSE2_FASTER10()
		  *   - CFeatureExtraction::detectFeatures_SSE2_FASTER12()
		  *
		  * \note The descriptor "Intensity-domain spin images" is described in "A sparse texture representation using affine-invariant regions", S Lazebnik, C Schmid, J Ponce, 2003 IEEE Computer Society Conference on Computer Vision.
		  * \sa mrpt::vision::CFeature
		  * \ingroup mrptvision_features
		  */
		class VISION_IMPEXP CFeatureExtraction
		{
		public:
			enum TSIFTImplementation
			{
				LoweBinary = 0,
				CSBinary,
				VedaldiBinary,
				Hess,
				OpenCV
			};

			/** The set of parameters for all the detectors & descriptor algorithms */
			struct VISION_IMPEXP TOptions : public utils::CLoadableOptions
			{
				/** Initalizer */
				TOptions(const TFeatureType featsType = featKLT);

				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
				void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

				/** Type of the extracted features
				*/
				TFeatureType featsType;

				/** Size of the patch to extract, or 0 if no patch is desired (default=21).
				  */
				unsigned int patchSize;

				/** Whether to use a mask for determining the regions where not to look for keypoints (default=false).
				  */
				bool useMask;

				/** Whether to add the found features to the input feature list or clear it before adding them (default=false).
				  */
				bool addNewFeatures;

				/** Indicates if subpixel accuracy is desired for the extracted points (only applicable to KLT and Harris features)
				  */
				bool FIND_SUBPIXEL;

				/** KLT Options */
				struct VISION_IMPEXP TKLTOptions
				{
					int		radius;			// size of the block of pixels used
					float	threshold;		// (default=0.1) for rejecting weak local maxima (with min_eig < threshold*max(eig_image))
					float	min_distance;	// minimum distance between features
					bool	tile_image;		// splits the image into 8 tiles and search for the best points in all of them (distribute the features over all the image)
				} KLTOptions;

				/** Harris Options */
				struct VISION_IMPEXP THarrisOptions
				{
					float	threshold;		// (default=0.005) for rejecting weak local maxima (with min_eig < threshold*max(eig_image))
					float	k;				// k factor for the Harris algorithm
					float	sigma;			// standard deviation for the gaussian smoothing function
					int		radius;			// size of the block of pixels used
					float	min_distance;	// minimum distance between features
					bool	tile_image;		// splits the image into 8 tiles and search for the best points in all of them (distribute the features over all the image)
				} harrisOptions;

				/** BCD Options */
				struct VISION_IMPEXP TBCDOptions
				{
				} BCDOptions;

				/** FAST and FASTER Options */
				struct VISION_IMPEXP TFASTOptions
				{
					int 	threshold;  //!< default= 20
					float	min_distance;	//!< (default=5) minimum distance between features (in pixels)
					bool	nonmax_suppression;		//!< Default = true
					bool    use_KLT_response; //!< (default=false) If true, use CImage::KLT_response to compute the response at each point instead of the FAST "standard response".
				} FASTOptions;

				/** ORB Options */
				struct VISION_IMPEXP TORBOptions
				{
					TORBOptions() : n_levels(8), min_distance(0), scale_factor(1.2f),extract_patch(false) {}

					size_t	n_levels;
					size_t	min_distance;
					float	scale_factor;
					bool	extract_patch;
				} ORBOptions;

				/** SIFT Options  */
				struct VISION_IMPEXP TSIFTOptions
				{
					TSIFTOptions() : threshold(0.04), edgeThreshold(10) { }

					TSIFTImplementation implementation;  //!< Default: Hess (OpenCV should be preferred, but its nonfree module is not always available by default in all systems)
					double threshold;  //!< default= 0.04
					double edgeThreshold; //!< default= 10
				} SIFTOptions;

				struct VISION_IMPEXP TSURFOptions
				{
					TSURFOptions() : rotation_invariant(true),hessianThreshold(600), nOctaves(2), nLayersPerOctave(4) { }

					/** SURF Options
					  */
					bool   rotation_invariant; //!< Compute the rotation invariant SURF (dim=128) if set to true (default), or the smaller uSURF otherwise (dim=64)
					int    hessianThreshold;   //!< Default: 600
					int    nOctaves;           //!< Default: 2
					int    nLayersPerOctave;   //!< Default: 4
				} SURFOptions;

				struct VISION_IMPEXP TSpinImagesOptions
				{
					/** SpinImages Options
					  */
					unsigned int hist_size_intensity; //!< Number of bins in the "intensity" axis of the 2D histogram (default=10).
					unsigned int hist_size_distance;  //!< Number of bins in the "distance" axis of the 2D histogram (default=10).
					float        std_dist;      //!< Standard deviation in "distance", used for the "soft histogram" (default=0.4 pixels)
					float        std_intensity; //!< Standard deviation in "intensity", used for the "soft histogram" (default=20 units [0,255])
					unsigned int radius;		//!< Maximum radius of the area of which the histogram is built, in pixel units (default=20 pixels)
				} SpinImagesOptions;

				/** PolarImagesOptions Options
				  */
				struct VISION_IMPEXP TPolarImagesOptions
				{
					unsigned int bins_angle;     //!< Number of bins in the "angular" axis of the polar image (default=8).
					unsigned int bins_distance;  //!< Number of bins in the "distance" axis of the polar image (default=6).
					unsigned int radius;         //!< Maximum radius of the area of which the polar image is built, in pixel units (default=20 pixels)
				} PolarImagesOptions;

				/** LogPolarImagesOptions Options
				  */
				struct VISION_IMPEXP TLogPolarImagesOptions
				{
					unsigned int radius;		//!< Maximum radius of the area of which the log polar image is built, in pixel units (default=30 pixels)
					unsigned int num_angles;	//!< (default=16) Log-Polar image patch will have dimensions WxH, with:  W=num_angles,  H= rho_scale * log(radius)
					double rho_scale;			//!< (default=5) Log-Polar image patch will have dimensions WxH, with:  W=num_angles,  H= rho_scale * log(radius)
				} LogPolarImagesOptions;

			};

			TOptions options;  //!< Set all the parameters of the desired method here before calling "detectFeatures"

			/** Constructor
			*/
			CFeatureExtraction();

			/** Virtual destructor.
			*/
			virtual ~CFeatureExtraction();

			/** Extract features from the image based on the method defined in TOptions.
			* \param img (input) The image from where to extract the images.
			* \param feats (output) A complete list of features (containing a patch for each one of them if options.patchsize > 0).
			* \param nDesiredFeatures (op. input) Number of features to be extracted. Default: all possible.
			*
			* \sa computeDescriptors
			*/
			void  detectFeatures(	
				const mrpt::utils::CImage		    & img,
				CFeatureList			& feats,
				const unsigned int		init_ID = 0,
				const unsigned int		nDesiredFeatures = 0,
				const TImageROI			&ROI = TImageROI()) const;

			/** Compute one (or more) descriptors for the given set of interest points onto the image, which may have been filled out manually or from \a detectFeatures
			* \param in_img (input) The image from where to compute the descriptors.
			* \param inout_features (input/output) The list of features whose descriptors are going to be computed.
			* \param in_descriptor_list (input) The bitwise OR of one or several descriptors defined in TDescriptorType.
			*
			*  Each value in "in_descriptor_list" represents one descriptor to be computed, for example:
			*  \code
			*    // This call will compute both, SIFT and Spin-Image descriptors for a list of feature points lstFeats.
			*    fext.computeDescriptors(img, lstFeats, descSIFT | descSpinImages );
			*  \endcode
			*
			* \note The SIFT descriptors for already located features can only be computed through the Hess and
			*        CSBinary implementations which may be specified in CFeatureExtraction::TOptions::SIFTOptions.
			*
			* \note This call will also use additional parameters from \a options
			*/
			void  computeDescriptors(
				const mrpt::utils::CImage	&in_img,
				CFeatureList		&inout_features,
				TDescriptorType		in_descriptor_list) const;

#if 0  // Delete? see comments in .cpp
			/** Extract more features from the image (apart from the provided ones) based on the method defined in TOptions.
			* \param img (input) The image from where to extract the images.
			* \param inList (input) The actual features in the image.
			* \param outList (output) The list of new features (containing a patch for each one of them if options.patchsize > 0).
			* \param nDesiredFeatures (op. input) Number of features to be extracted. Default: all possible.
			*
			*  \sa The more powerful class: mrpt::vision::CGenericFeatureTracker
			*/
			void  findMoreFeatures( const mrpt::utils::CImage &img,
									const CFeatureList &inList,
									CFeatureList &outList,
									unsigned int nDesiredFeats = 0) const;
#endif

			/** @name Static methods with low-level detector functionality
			    @{ */

			/** A SSE2-optimized implementation of FASTER-9 (requires img to be grayscale). If SSE2 is not available, it gratefully falls back to a non-optimized version.
			  *
			  *  Only the pt.{x,y} fields are filled out for each feature: the rest of fields are left <b>uninitialized</b> and their content is <b>undefined</b>.
			  *  Note that (x,y) are already scaled to the 0-level image coordinates if octave>0, by means of:
			  *
			  *  \code
			  *    pt.x = detected.x << octave;
			  *    pt.y = detected.y << octave;
			  *  \endcode
			  *
			  * If \a append_to_list is true, the \a corners list is not cleared before adding the newly detected feats.
			  *
			  * If a valid pointer is provided for \a out_feats_index_by_row, upon return you will find a vector with
			  *  as many entries as rows in the image (the real number of rows, disregarding the value of \a octave).
			  * The number in each entry is the 0-based index (in \a corners) of
			  *  the first feature that falls in that line of the image. This index can be used to fasten looking for correspondences.
			  *
			  * \ingroup mrptvision_features
			  */
			static void detectFeatures_SSE2_FASTER9(
				const mrpt::utils::CImage &img,
				TSimpleFeatureList & corners,
				const int threshold = 20,
				bool append_to_list = false,
				uint8_t octave = 0,
				std::vector<size_t> * out_feats_index_by_row = NULL );

			/** Just like \a detectFeatures_SSE2_FASTER9() for another version of the detector.
			  * \ingroup mrptvision_features */
			static void detectFeatures_SSE2_FASTER10(
				const mrpt::utils::CImage &img,
				TSimpleFeatureList & corners,
				const int threshold = 20,
				bool append_to_list = false,
				uint8_t octave = 0,
				std::vector<size_t> * out_feats_index_by_row = NULL );

			/** Just like \a detectFeatures_SSE2_FASTER9() for another version of the detector.
			  * \ingroup mrptvision_features */
			static void detectFeatures_SSE2_FASTER12(
				const mrpt::utils::CImage &img,
				TSimpleFeatureList & corners,
				const int threshold = 20,
				bool append_to_list = false,
				uint8_t octave = 0,
				std::vector<size_t> * out_feats_index_by_row = NULL );

			/** @} */

		private:
			/** Compute the SIFT descriptor of the provided features into the input image
			* \param in_img (input) The image from where to compute the descriptors.
			* \param in_features (input/output) The list of features whose descriptors are going to be computed.
			*
			* \note The SIFT descriptors for already located features can only be computed through the Hess and
			        CSBinary implementations which may be specified in CFeatureExtraction::TOptions::SIFTOptions.
			*/
			void  internal_computeSiftDescriptors( const mrpt::utils::CImage	&in_img,
										  CFeatureList		&in_features) const;


			/** Compute the SURF descriptor of the provided features into the input image
			* \param in_img (input) The image from where to compute the descriptors.
			* \param in_features (input/output) The list of features whose descriptors are going to be computed.
			*/
			void  internal_computeSurfDescriptors( const mrpt::utils::CImage	&in_img,
										  CFeatureList		&in_features) const;

			/** Compute the ORB descriptor of the provided features into the input image
			* \param in_img (input) The image from where to compute the descriptors.
			* \param in_features (input/output) The list of features whose descriptors are going to be computed.
			*/
			void  internal_computeORBDescriptors( const mrpt::utils::CImage	&in_img,
										  CFeatureList		&in_features) const;

			/** Compute the intensity-domain spin images descriptor of the provided features into the input image
			* \param in_img (input) The image from where to compute the descriptors.
			* \param in_features (input/output) The list of features whose descriptors are going to be computed.
			*
			* \note Additional parameters from CFeatureExtraction::TOptions::SpinImagesOptions are used in this method.
			*/
			void  internal_computeSpinImageDescriptors( const mrpt::utils::CImage	&in_img,
										  CFeatureList		&in_features) const;

			/** Compute a polar-image descriptor of the provided features into the input image
			* \param in_img (input) The image from where to compute the descriptors.
			* \param in_features (input/output) The list of features whose descriptors are going to be computed.
			*
			* \note Additional parameters from CFeatureExtraction::TOptions::PolarImagesOptions are used in this method.
			*/
			void  internal_computePolarImageDescriptors( const mrpt::utils::CImage	&in_img,
										  CFeatureList		&in_features) const;

			/** Compute a log-polar image descriptor of the provided features into the input image
			* \param in_img (input) The image from where to compute the descriptors.
			* \param in_features (input/output) The list of features whose descriptors are going to be computed.
			*
			* \note Additional parameters from CFeatureExtraction::TOptions::LogPolarImagesOptions are used in this method.
			*/
			void  internal_computeLogPolarImageDescriptors( const mrpt::utils::CImage	&in_img,
										  CFeatureList		&in_features) const;

#if 0  // Delete? see comments in .cpp
			/** Select good features using the openCV implementation of the KLT method.
			* \param img (input) The image from where to select extract the images.
			* \param feats (output) A complete list of features (containing a patch for each one of them if options.patchsize > 0).
			* \param nDesiredFeatures (op. input) Number of features to be extracted. Default: all possible.
			*/
			void  selectGoodFeaturesKLT(
				const mrpt::utils::CImage	&inImg,
				CFeatureList		&feats,
				unsigned int		init_ID = 0,
				unsigned int		nDesiredFeatures = 0) const;
#endif

			/** Extract features from the image based on the KLT method.
			* \param img The image from where to extract the images.
			* \param feats The list of extracted features.
			* \param nDesiredFeatures Number of features to be extracted. Default: authomatic.
			*/
			void  extractFeaturesKLT(
				const mrpt::utils::CImage		&img,
				CFeatureList			&feats,
				unsigned int			init_ID = 0,
				unsigned int			nDesiredFeatures = 0,
				const TImageROI			&ROI = TImageROI()) const;

			// ------------------------------------------------------------------------------------
			//											BCD
			// ------------------------------------------------------------------------------------
			/** Extract features from the image based on the BCD method.
			* \param img The image from where to extract the images.
			* \param feats The list of extracted features.
			* \param nDesiredFeatures Number of features to be extracted. Default: authomatic.
			* \param ROI (op. input) Region of Interest. Default: All the image.
			*/
			void  extractFeaturesBCD(
				const mrpt::utils::CImage 		&img,
				CFeatureList			&feats,
				unsigned int			init_ID = 0,
				unsigned int			nDesiredFeatures = 0,
				const TImageROI			&ROI = TImageROI()) const;

			// ------------------------------------------------------------------------------------
			//											SIFT
			// ------------------------------------------------------------------------------------
			/** Extract features from the image based on the SIFT method.
			* \param img The image from where to extract the images.
			* \param feats The list of extracted features.
			* \param nDesiredFeatures Number of features to be extracted. Default: authomatic.
			* \param ROI (op. input) Region of Interest. Default: All the image.
			*/
			void  extractFeaturesSIFT(
				const mrpt::utils::CImage		&img,
				CFeatureList			&feats,
				unsigned int			init_ID = 0,
				unsigned int			nDesiredFeatures = 0,
				const TImageROI			&ROI = TImageROI()) const;

			// ------------------------------------------------------------------------------------
			//											ORB
			// ------------------------------------------------------------------------------------
			/** Extract features from the image based on the ORB method.
			* \param img The image from where to extract the images.
			* \param feats The list of extracted features.
			* \param nDesiredFeatures Number of features to be extracted. Default: authomatic.
			*/
			void  extractFeaturesORB(
				const mrpt::utils::CImage			&img,
				CFeatureList			&feats,
				const unsigned int		init_ID = 0,
				const unsigned int		nDesiredFeatures = 0,
				const TImageROI			    & ROI = TImageROI()) const;


			// ------------------------------------------------------------------------------------
			//											SURF
			// ------------------------------------------------------------------------------------
			/** Extract features from the image based on the SURF method.
			* \param img The image from where to extract the images.
			* \param feats The list of extracted features.
			* \param nDesiredFeatures Number of features to be extracted. Default: authomatic.
			*/
			void  extractFeaturesSURF(
				const mrpt::utils::CImage		&img,
				CFeatureList			&feats,
				unsigned int			init_ID = 0,
				unsigned int			nDesiredFeatures = 0,
				const TImageROI			&ROI = TImageROI())  const;

			// ------------------------------------------------------------------------------------
			//											FAST
			// ------------------------------------------------------------------------------------
			/** Extract features from the image based on the FAST method.
			* \param img The image from where to extract the images.
			* \param feats The list of extracted features.
			* \param nDesiredFeatures Number of features to be extracted. Default: authomatic.
			*/
			void  extractFeaturesFAST(
				const mrpt::utils::CImage			&img,
				CFeatureList			&feats,
				unsigned int			init_ID = 0,
				unsigned int			nDesiredFeatures = 0,
				const TImageROI			    & ROI = TImageROI(),
				const mrpt::math::CMatrixBool           * mask= NULL) const;

			/** Edward's "FASTER & Better" detector, N=9,10,12 */
			void  extractFeaturesFASTER_N(
				const int               N,
				const mrpt::utils::CImage			&img,
				CFeatureList			&feats,
				unsigned int			init_ID = 0,
				unsigned int			nDesiredFeatures = 0,
				const TImageROI			    & ROI = TImageROI()) const;


			// ------------------------------------------------------------------------------------
			//								my_scale_space_extrema
			// ------------------------------------------------------------------------------------
			/** Computes extrema in the scale space.
			* \param dog_pyr Pyramid of images.
			* \param octvs Number of considered octaves.
			* \param intvls Number of intervales in octaves.
			*/
			void* my_scale_space_extrema(
				CFeatureList &featList, void* dog_pyr,
				int octvs, int intvls, double contr_thr, int curv_thr,
				void* storage ) const;

			/** Adjust scale if the image was initially doubled.
			* \param features The sequence of features.
			*/
			void	my_adjust_for_img_dbl( void* features ) const;

			/** Gets the number of times that a point in the image is higher or lower than the surroundings in the image-scale space
			* \param dog_pyr Pyramid of images.
			* \param octvs Number of considered octaves.
			* \param intvls Number of intervales in octaves.
			* \param row The row of the feature in the original image.
			* \param col The column of the feature in the original image.
			* \param nMin [out]: Times that the feature is lower than the surroundings.
			* \param nMax [out]: Times that the feature is higher than the surroundings.
			*/
			void	getTimesExtrema( void* dog_pyr, int octvs, int intvls, float row, float col, unsigned int &nMin, unsigned int &nMax ) const;

			/** Computes the Laplacian value of the feature in the corresponing image in the pyramid.
			* \param dog_pyr Pyramid of images.
			* \param octvs Number of considered octaves.
			* \param intvls Number of intervales in octaves.
			* \param row The row of the feature in the original image.
			* \param col The column of the feature in the original image.
			*/
			double	getLaplacianValue( void* dog_pyr, int octvs, int intvls, float row, float col ) const;

			/** Append a sequence of openCV features into an MRPT feature list.
			* \param features The sequence of features.
			* \param list [in-out] The list of MRPT features.
			* \param init_ID [in] The initial ID for the new features.
			*/
			void	insertCvSeqInCFeatureList( void* features, CFeatureList &list, unsigned int init_ID = 0 ) const;

			/** Converts a sequence of openCV features into an MRPT feature list.
			* \param features The sequence of features.
			* \param list [in-out] The list of MRPT features.
			* \param init_ID [in][optional] The initial ID for the features (default = 0).
			* \param ROI [in][optional] The initial ID for the features (default = empty ROI -> not used).
			*/
			void	convertCvSeqInCFeatureList( void* features, CFeatureList &list, unsigned int init_ID = 0, const TImageROI &ROI = TImageROI() ) const;

		}; // end of class
	} // end of namespace
} // end of namespace
#endif

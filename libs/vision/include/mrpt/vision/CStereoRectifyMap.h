/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef mrpt_CStereoRectifyMap_H
#define mrpt_CStereoRectifyMap_H

#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/poses/CPose3DQuat.h>

#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		/** Use this class to rectify stereo images if the same distortion maps are reused over and over again.
		  *  The rectify maps are cached internally and only computed once for the camera parameters.
		  * The stereo camera calibration must be supplied in a mrpt::util::TStereoCamera structure
		  *  (which provides method for loading from a plain text config file) or directly from the
		  *  parameters of a mrpt::obs::CObservationStereoImages object.
		  *
		  * Remember that the rectified images have a different set of intrinsic parameters than the
		  *  original images, which can be retrieved with \a getRectifiedImageParams()
		  *
		  *  Works with grayscale or color images.
		  *
		  *  Refer to the program stereo-calib-gui for a tool that generates the required stereo camera parameters
		  *  from a set of stereo images of a checkerboard.
		  *
		  *  Example of usage with mrpt::obs::CObservationStereoImages:
		  *
		  * \code
		  *   CStereoRectifyMap   rectify_map;
		  *   // Set options as desired:
		  *   // rectify_map.setAlpha(...);
		  *   // rectify_map.enableBothCentersCoincide(...);
		  *
		  *   while (true) {
		  *     mrpt::obs::CObservationStereoImagesPtr obs_stereo = ... // Grab stereo observation from wherever
		  *
		  *     // Only once, construct the rectification maps:
		  *     if (!rectify_map.isSet())
		  *       rectify_map.setFromCamParams(*obs_stereo);
		  *
		  *     // Rectify in place:
		  *     unmap.rectify(*obs_stereo);
		  *     // Rectified images are now in: obs_stereo->imageLeft & obs_stereo->imageRight
		  *   }
		  * \endcode
		  *
		  *  Read also the tutorial page online: http://www.mrpt.org/Rectifying_stereo_images
		  *
		  * \sa CUndistortMap, mrpt::obs::CObservationStereoImages, mrpt::utils::TCamera, the application <a href="http://www.mrpt.org/Application:camera-calib" >camera-calib</a> for calibrating a camera.
		  *
		  * \note This class provides a uniform wrap over different OpenCV versions. The "alpha" parameter is ignored if built against OpenCV 2.0.X
		  *
		  * \ingroup mrpt_vision_grp
		  */
		class VISION_IMPEXP  CStereoRectifyMap
		{
		public:
			CStereoRectifyMap(); //!< Default ctor

		/** @name Rectify map preparation and setting/getting of parameters
		    @{ */
			/** Returns true if \a setFromCamParams() has been already called, false otherwise.
			  *  Can be used within loops to determine the first usage of the object and when it needs to be initialized.
			  */
			inline bool isSet() const { return !m_dat_mapx_left.empty(); }

			/** Prepares the mapping from the intrinsic, distortion and relative pose parameters of a stereo camera.
			  * Must be called before invoking \a rectify().
			  * The \a alpha parameter can be changed with \a setAlpha() before invoking this method; otherwise, the current rectification maps will be marked as invalid and should be prepared again.
			  * \sa setAlpha()
			  */
			void setFromCamParams(const mrpt::utils::TStereoCamera &params);

			/** A wrapper to \a setFromCamParams() which takes the parameters from an stereo observation object */
			void setFromCamParams(const mrpt::obs::CObservationStereoImages &stereo_obs)
			{
				mrpt::utils::TStereoCamera params;
				stereo_obs.getStereoCameraParams(params);
				setFromCamParams(params);
			}

			/** Returns the camera parameters which were used to generate the distortion map, as passed by the user to \a setFromCamParams */
			inline const mrpt::utils::TStereoCamera & getCameraParams() const { return m_camera_params; }

			/** After computing the rectification maps, this method retrieves the calibration parameters of the rectified images
			  *  (which won't have any distortion).
			  * \exception std::exception If the rectification maps have not been computed.
			  */
			const mrpt::utils::TStereoCamera & getRectifiedImageParams() const;

			const mrpt::utils::TCamera & getRectifiedLeftImageParams() const;  //!< Just like \a getRectifiedImageParams() but for the left camera only
			const mrpt::utils::TCamera & getRectifiedRightImageParams() const; //!< Just like \a getRectifiedImageParams() but for the right camera only

			/** Sets the \a alpha parameter which controls the zoom in/out of the rectified images, such that:
			  *  - alpha=0 => rectified images are zoom in so that only valid pixels are visible
			  *  - alpha=1 => rectified images will contain large "black areas" but no pixel from the original image will be lost.
			  * Intermediary values leads to intermediary results.
			  * Its default value (-1) means auto guess by the OpenCV's algorithm.
			  * \note Call this method before building the rectification maps, otherwise they'll be marked as invalid.
			  */
			void setAlpha(double alpha);

			/** Return the \a alpha parameter \sa setAlpha */
			inline double getAlpha() const { return m_alpha; }

			/** If enabled, the computed maps will rectify images to a size different than their original size.
			  * \note Call this method before building the rectification maps, otherwise they'll be marked as invalid.
			  */
			void enableResizeOutput(bool enable, unsigned int target_width=0, unsigned int target_height=0);

			/** Returns whether resizing is enabled (default=false) \sa enableResizeOutput */
			bool isEnabledResizeOutput() const { return m_resize_output; }

			/** Only when \a isEnabledResizeOutput() returns true, this gets the target size  \sa enableResizeOutput */
			mrpt::utils::TImageSize getResizeOutputSize() const { return m_resize_output_value; }

			/** Change remap interpolation method (default=Lineal). This parameter can be safely changed at any instant without consequences. */
			void setInterpolationMethod(const mrpt::utils::TInterpolationMethod interp) {
				m_interpolation_method = interp;
			}

			/** Get the currently selected interpolation method \sa setInterpolationMethod */
			mrpt::utils::TInterpolationMethod getInterpolationMethod() const { return m_interpolation_method; }

			/** If enabled (default=false), the principal points in both output images will coincide.
			  * \note Call this method before building the rectification maps, otherwise they'll be marked as invalid.
			  */
			void enableBothCentersCoincide(bool enable=true);

			/** \sa enableBothCentersCoincide */
			bool isEnabledBothCentersCoincide() const { return m_enable_both_centers_coincide; }

			/** After computing the rectification maps, get the rotation applied to the 
			  * left/right camera so their virtual image plane is the same after rectification */
			const mrpt::poses::CPose3DQuat  & getLeftCameraRot() const { return m_rot_left; } 
			/** See \a getLeftCameraRot()  */
			const mrpt::poses::CPose3DQuat  & getRightCameraRot() const { return m_rot_right; } 
			/** Direct input access to rectify maps */
			void setRectifyMaps( const std::vector<int16_t> &left_x,  const std::vector<uint16_t> &left_y,
								const std::vector<int16_t> &right_x, const std::vector<uint16_t> &right_y );
			
			/** Direct input access to rectify maps. This method swaps the vectors so the inputs are no longer available.*/
			void setRectifyMapsFast( std::vector<int16_t> &left_x,  std::vector<uint16_t> &left_y,
								std::vector<int16_t> &right_x, std::vector<uint16_t> &right_y );

		/** @} */

		/** @name Rectify methods
		    @{ */

			/** Rectify the input image pair and save the result in a different output images - \a setFromCamParams() must have been set prior to calling this.
			  * The previous contents of the output images are completely ignored, but if they are already of the
			  * correct size and type, allocation time will be saved.
			  * Recall that \a getRectifiedImageParams() provides you the new intrinsic parameters of these images.
			  * \exception std::exception If the rectification maps have not been computed.
			  * \note The same image CANNOT be at the same time input and output, in which case an exception will be raised (but see the overloaded version for in-place rectification)
			  */
			void rectify(
				const mrpt::utils::CImage &in_left_image,
				const mrpt::utils::CImage &in_right_image,
				mrpt::utils::CImage &out_left_image,
				mrpt::utils::CImage &out_right_image) const;

			/** Overloaded version for in-place rectification: replace input images with their rectified versions
			  * If \a use_internal_mem_cache is set to \a true (recommended), will reuse over and over again the same
			  * auxiliary images (kept internally to this object) needed for in-place rectification.
			  * The only reason not to enable this cache is when multiple threads can invoke this method simultaneously.
			  */
			void rectify(
				mrpt::utils::CImage &left_image,
				mrpt::utils::CImage &right_image,
				const bool use_internal_mem_cache = true ) const;

			/** Overloaded version for in-place rectification of image pairs stored in a mrpt::obs::CObservationStereoImages.
			  *  Upon return, the new camera intrinsic parameters will be already stored in the observation object.
			  * If \a use_internal_mem_cache is set to \a true (recommended), will reuse over and over again the same
			  * auxiliary images (kept internally to this object) needed for in-place rectification.
			  * The only reason not to enable this cache is when multiple threads can invoke this method simultaneously.
			  * \note This method uses the left & right camera rotations computed by the rectification map to update 
			  *         mrpt::obs::CObservationStereoImages::cameraPose (left camera wrt the robot frame) and 
			  *         mrpt::obs::CObservationStereoImages::rightCameraPose (right wrt left camera).
			  */
			void rectify(
				mrpt::obs::CObservationStereoImages & stereo_image_observation,
				const bool use_internal_mem_cache = true ) const;

			/** Just like rectify() but directly works with OpenCV's "IplImage*", which must be passed as "void*" to avoid header dependencies
			  *  Output images CANNOT coincide with the input images. */
			void rectify_IPL(
				const void* in_left_image,
				const void* in_right_image,
				void* out_left_image,
				void* out_right_image) const;

		/** @} */

		private:
			double   m_alpha;
			bool     m_resize_output;
			bool     m_enable_both_centers_coincide;
			mrpt::utils::TImageSize m_resize_output_value;
			mrpt::utils::TInterpolationMethod m_interpolation_method;

			mutable mrpt::utils::CImage  m_cache1, m_cache2; //!< Memory caches for in-place rectification speed-up.

			std::vector<int16_t>  m_dat_mapx_left,m_dat_mapx_right;
			std::vector<uint16_t> m_dat_mapy_left,m_dat_mapy_right;

			mrpt::utils::TStereoCamera  m_camera_params; //!< A copy of the data provided by the user
			mrpt::utils::TStereoCamera  m_rectified_image_params; //!< Resulting images params

			mrpt::poses::CPose3DQuat  m_rot_left, m_rot_right; //!< The rotation applied to the left/right camera so their virtual image plane is the same after rectification.

			void internal_invalidate();

		}; // end class

	} // end namespace
} // end namespace
#endif

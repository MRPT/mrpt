/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

#include <mrpt/gui/CDisplayWindow.h>

namespace mrpt::hwdrivers
{
/** A class for grabing "range images" from a MESA imaging SwissRanger 3D
 *cameras (SR-2, SR-3000, SR-4k).
 *
 * NOTES:
 *		- This class requires a vendor specific driver installed in the system
 *in
 *order to build MRPT with support for this sensor. Download and install the
 *driver from: http://www.mesa-imaging.ch/drivers.php
 *		- The intensity channel (grayscale image) is converted from 16bit to
 *standard 8bit-per-pixel using a logarithmic, modified A-law compression. This
 *allows exploiting the full dynamic range of the sensor and provides quite
 *good results.
 *
 * As with any other CGenericSensor class, the normal sequence of methods to be
 *called is:
 *   - loadConfig() - Or calls to the individual setXXX() to configure the
 *camera parameters.
 *   - initialize() - to init the comms with the camera
 *   - call getNextObservation() for getting the frames.
 *
 *  This sensor can be also used from within rawlog-grabber.
 *
 *  \code
 *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
 * -------------------------------------------------------
 *   [supplied_section_name]
 *    sensorLabel  = CAM3D         // A text description
 *    preview_window  = true       // Show a window with a preview of the
 *grabbed data in real-time
 *
 *    open_USB     = true          // false means ethernet (default: true)
 *    USB_serial   = 0x4000002f    // only for open_USB=true. If not set, the
 *first camera will be open. Serial is the last part of S/N (e.g.  for the
 *camera SN: 00-00-40-00-00-2F).
 *    IP_address   = 192.168.2.14  // only for open_USB=false. The IP of the
 *camera.
 *
 *    // Options for the data to save in each CObservation3DRangeScan
 *    save_3d            = true			// Save the 3D point cloud (default:
 *true)
 *    save_range_img     = true			// Save the 2D range image (default:
 *true)
 *    save_intensity_img = true			// Save the 2D intensity image
 *(default:
 *true)
 *    save_confidence    = true			// Save the estimated confidence 2D
 *image
 *(default: false)
 *
 *    enable_img_hist_equal = false		// Enable intensity image histogram
 *equalization (default: false)
 *    enable_median_filter  = true			// Enable median filter in range
 *data
 *(default: true)
 *    enable_mediancross_filter = false	// Enable median cross-filter
 *(default:
 *false)
 *    enable_conv_gray      = false		// Enable intensity image scale with
 *range
 *(default: false)
 *    enable_denoise_anf    = true			// Enable this noise filter
 *(default:
 *true)
 *
 *    // Camera calibration parameters: See mrpt::img::TCamera
 *    //  If not provided, a set of default parameters for a SR4000 camera will
 *be loaded.
 *    resolution = [176 144]
 *    cx         = 87.99958
 *    cy         = 68.99957
 *    fx         = 262.9201
 *    fy         = 262.9218
 *    dist       = [-8.258543e-01 6.561022e-01 2.699818e-06 -3.263559e-05 0]
 *
 *    // For externaly stored images, the format of image files (default=jpg)
 *    //external_images_format  = jpg
 *    // (Only when external_images_format=jpg): Optional parameter to set the
 *JPEG compression quality:
 *    //external_images_jpeg_quality = 95    // [1-100]. Default: 95
 *
 *    pose_x=0.21	// Camera position in the robot (meters)
 *    pose_y=0
 *    pose_z=0.34
 *    pose_yaw=0	// Angles in degrees
 *    pose_pitch=0
 *    pose_roll=0
 *
 *  \endcode
 * \ingroup mrpt_hwdrivers_grp
 */
class CSwissRanger3DCamera : public mrpt::hwdrivers::CGenericSensor
{
	DEFINE_GENERIC_SENSOR(CSwissRanger3DCamera)

   public:
	/** Default ctor */
	CSwissRanger3DCamera();
	/** Default ctor */
	~CSwissRanger3DCamera() override;

	/** Initializes the 3D camera - should be invoked after calling loadConfig()
	 *  \exception This method must throw an exception with a descriptive
	 * message if some critical error is found.
	 */
	void initialize() override;

	/** To be called  at a high rate (>XX Hz), this method populates the
	 * internal buffer of received observations.
	 *  This method is mainly intended for usage within rawlog-grabber or
	 * similar programs.
	 *  For an alternative, see getNextObservation()
	 *  \exception This method must throw an exception with a descriptive
	 * message if some critical error is found.
	 * \sa getNextObservation
	 */
	void doProcess() override;

	/** The main data retrieving function, to be called after calling
	 * loadConfig() and initialize().
	 *  \param out_obs The output retrieved observation (only if
	 * there_is_obs=true).
	 *  \param there_is_obs If set to false, there was no new observation.
	 *  \param hardware_error True on hardware/comms error.
	 *
	 * \sa doProcess
	 */
	void getNextObservation(
		mrpt::obs::CObservation3DRangeScan& out_obs, bool& there_is_obs,
		bool& hardware_error);

	/** return false on error - Called automatically from initialize(), no need
	 * normally for the user to call this. */
	bool open();
	void close();

	/** whether the camera is open and comms work ok. To be called after
	 * initialize() */
	bool isOpen() const;

	/** Get the row count in the camera images, loaded automatically upon camera
	 * open(). */
	size_t rows() const { return m_rows; }
	/** Get the col count in the camera images, loaded automatically upon camera
	 * open(). */
	size_t cols() const { return m_cols; }
	/** Get the camera serial number, loaded automatically upon camera open().
	 */
	unsigned int getCameraSerialNumber() const { return m_cam_serial_num; }
	/** Returns the maximum camera range, as deduced from its operating
	 * frequency. */
	double getMaxRange() const { return m_maxRange; }
	/**  Set the path where to save off-rawlog image files (this class DOES take
	 * into account this path).
	 *  An  empty string (the default value at construction) means to save
	 * images embedded in the rawlog, instead of on separate files.
	 * \exception std::exception If the directory doesn't exists and cannot be
	 * created.
	 */
	void setPathForExternalImages(const std::string& directory) override;

	/** @name Capture configuration methods (apart from loadConfig)
		@{ */

	/** true: open from USB, false: open from ethernet. */
	inline void setOpenFromUSB(bool USB) { m_open_from_usb = USB; }
	inline bool getOpenFromUSBMode() const { return m_open_from_usb; }
	inline void setOpenIPAddress(const std::string& IP) { m_ip_address = IP; }
	inline std::string getOpenIPAddress() const { return m_ip_address; }
	inline void setSave3D(bool save) { m_save_3d = save; }
	inline void setSaveRangeImage(bool save) { m_save_range_img = save; }
	inline void setSaveIntensityImage(bool save)
	{
		m_save_intensity_img = save;
	}
	inline void setSaveConfidenceImage(bool save) { m_save_confidence = save; }
	inline void enableImageHistEqualization(bool enable)
	{
		m_enable_img_hist_equal = enable;
	}
	inline bool isEnabledImageHistEqualization() const
	{
		return m_enable_img_hist_equal;
	}

	inline void enableMedianFilter(bool enable)
	{
		m_enable_median_filter = enable;
		internal_resendParamsToCamera();
	}
	inline bool isEnabledMedianFilter() const { return m_enable_median_filter; }
	inline void enableMedianCrossFilter(bool enable)
	{
		m_enable_mediancross_filter = enable;
		internal_resendParamsToCamera();
	}
	inline bool isEnabledMedianCrossFilter() const
	{
		return m_enable_mediancross_filter;
	}

	inline void enableConvGray(bool enable)
	{
		m_enable_conv_gray = enable;
		internal_resendParamsToCamera();
	}
	inline bool isEnabledConvGray() const { return m_enable_conv_gray; }
	inline void enableDenoiseANF(bool enable)
	{
		m_enable_denoise_anf = enable;
		internal_resendParamsToCamera();
	}
	inline bool isEnabledDenoiseANF() const { return m_enable_denoise_anf; }
	inline void enablePreviewWindow(bool enable = true)
	{
		m_preview_window = enable;
	}
	inline bool isEnabledPreviewWindow() const { return m_preview_window; }
	/** @} */

	// List of small functions to be implemented differently in Win/Lin.

	/** Get the version of the MESA library.
	 * \return false on error
	 */
	bool getMesaLibVersion(std::string& out_version) const;

   protected:
	/** See the class documentation at the top for expected parameters */
	void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& section) override;

	void internal_resendParamsToCamera() const;

	mrpt::poses::CPose3D m_sensorPoseOnRobot;

	/** Save the 3D point cloud (default: true) */
	bool m_save_3d{true};
	/** Save the 2D range image (default: true) */
	bool m_save_range_img{true};
	/** Save the 2D intensity image (default: true) */
	bool m_save_intensity_img{true};
	/** Save the estimated confidence 2D image (default: false) */
	bool m_save_confidence{false};

	bool m_enable_img_hist_equal{false};
	bool m_enable_median_filter{true};
	bool m_enable_mediancross_filter{false};
	bool m_enable_conv_gray{false};
	bool m_enable_denoise_anf{true};

	/** true: USB, false: ETH */
	bool m_open_from_usb{true};
	size_t m_usb_serial{0};
	std::string m_ip_address;

	/** Size of camera images, set on open() */
	size_t m_rows{0}, m_cols{0};
	/** Serial number of the camera, set on open() */
	unsigned int m_cam_serial_num{0};
	/** Max range, as deducted from the camera frequency. */
	double m_maxRange{5};

	/** Show preview window while grabbing */
	bool m_preview_window{false};
	mrpt::gui::CDisplayWindow::Ptr m_win_range, m_win_int;

	/** opaque handler to SRCAM. nullptr means it's not open yet. */
	void* m_cam;

	mrpt::img::TCamera m_cameraParams;

   private:
};  // End of class

}  // namespace mrpt::hwdrivers

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/img/CImage.h>
#include <mrpt/gui/gui_frwds.h>

namespace mrpt::gui
{
/** Create a GUI window and display plots with MATLAB-like interfaces and
 * commands.
 *
 *  For a list of supported events with the observer/observable pattern, see the
 * discussion in mrpt::gui::CBaseGUIWindow.
 *
 *   See CDisplayWindowPlots::plot
 * \ingroup mrpt_gui_grp
 */
class CDisplayWindowPlots : public mrpt::gui::CBaseGUIWindow
{
   public:
	using Ptr = std::shared_ptr<CDisplayWindowPlots>;

	/** Type for the callback function used in setMenuCallback */
	using TCallbackMenu =
		void (*)(int menuID, float cursor_x, float cursor_y, void* userParam);

   protected:
	friend class CWindowDialogPlots;

	/** Whether hold_on is enabled */
	bool m_holdon{false};
	bool m_holdon_just_disabled{false};
	/** Counter for hold_on */
	uint32_t m_holdon_cnt{0};
	TCallbackMenu m_callback{nullptr};
	void* m_callback_param{nullptr};

	void internal_plot(
		mrpt::math::CVectorFloat& x, mrpt::math::CVectorFloat& y,
		const std::string& lineFormat, const std::string& plotName);
	template <typename VECTOR1, typename VECTOR2>
	void internal_plot_interface(
		const VECTOR1& x, const VECTOR2& y, const std::string& lineFormat,
		const std::string& plotName)
	{
		mrpt::math::CVectorFloat x1(x.size()), y1(y.size());
		const auto N1 = size_t(x.size());
		for (size_t i = 0; i < N1; i++) x1[i] = x[i];
		const auto N2 = size_t(y.size());
		for (size_t i = 0; i < N2; i++) y1[i] = y[i];
		this->internal_plot(x1, y1, lineFormat, plotName);
	}
	template <typename VECTOR1>
	void internal_plot_interface(
		const VECTOR1& y, const std::string& lineFormat,
		const std::string& plotName)
	{
		const auto N = size_t(y.size());
		mrpt::math::CVectorFloat x1(N), y1(N);
		for (size_t i = 0; i < N; i++)
		{
			x1[i] = i;
			y1[i] = y[i];
		}
		this->internal_plot(x1, y1, lineFormat, plotName);
	}

   public:
	/** Constructor
	 */
	CDisplayWindowPlots(
		const std::string& windowCaption = std::string(),
		unsigned int initialWidth = 350, unsigned int initialHeight = 300);

	/** Class factory returning a smart pointer */
	static CDisplayWindowPlots::Ptr Create(
		const std::string& windowCaption, unsigned int initialWindowWidth = 400,
		unsigned int initialWindowHeight = 300);

	/** Destructor
	 */
	~CDisplayWindowPlots() override;

	/** Gets the last x,y pixel coordinates of the mouse. \return False if the
	 * window is closed. */
	bool getLastMousePosition(int& x, int& y) const override;

	/** Set cursor style to default (cursorIsCross=false) or to a cross
	 * (cursorIsCross=true) */
	void setCursorCross(bool cursorIsCross) override;

	/** Resizes the window, stretching the image to fit into the display area.
	 */
	void resize(unsigned int width, unsigned int height) override;

	/** Changes the position of the window on the screen.
	 */
	void setPos(int x, int y) override;

	/** Changes the window title text.
	 */
	void setWindowTitle(const std::string& str) override;

	/** Enable/disable the feature of pan/zoom with the mouse (default=enabled)
	 */
	void enableMousePanZoom(bool enabled);

	/** Adds a new layer with a 2D plot based on two vectors of X and Y points,
	 *using a MATLAB-like syntax.
	 *  Each call to this function creates a new plot, unless the plot name
	 *coincides with an already existing plot: in this case the X & Y points
	 *are used to update this existing layer (this also applies to using the
	 *default plot name).
	 *  If "hold_on" is enabled, then every call will always create a new plot,
	 *even if no "plotName" is provided.
	 *
	 *  The lineFormat string is a combination of the following characters:
	 * - Line styles:
	 *		- '.': One point for each data point
	 *		- '-': A continuous line
	 *		- ':': A dashed line
	 * - Colors:
	 *		- k: black
	 *		- r: red
	 *		- g: green
	 *		- b: blue
	 *		- m: magenta
	 *		- c: cyan
	 * - Line width:
	 *		- '1' to '9': The line width (default=1)
	 *
	 *  Examples:
	 *   - 'r.' -> red points.
	 *   - 'k3' or 'k-3' -> A black line with a line width of 3 pixels.
	 * \note The vectors x & y can be of types: float or double.
	 * \sa axis, axis_equal, axis_fit, clear, hold_on, hold_off
	 * \tparam VECTOR Can be std::vector<float/double> or
	 *mrpt::dynamicsize_vector<float/double> or a column/row Eigen::Matrix<>
	 */
	template <typename T1, typename T2>
	inline void plot(
		const std::vector<T1>& x, const std::vector<T2>& y,
		const std::string& lineFormat = std::string("b-"),
		const std::string& plotName = std::string("plotXY"))
	{
		this->internal_plot_interface(x, y, lineFormat, plotName);
	}
	//! \overload
	template <typename T1, typename Derived2>
	inline void plot(
		const std::vector<T1>& x, const Eigen::MatrixBase<Derived2>& y,
		const std::string& lineFormat = std::string("b-"),
		const std::string& plotName = std::string("plotXY"))
	{
		this->internal_plot_interface(x, y, lineFormat, plotName);
	}
	//! \overload
	template <typename Derived1, typename T2>
	inline void plot(
		const Eigen::MatrixBase<Derived1>& x, const std::vector<T2>& y,
		const std::string& lineFormat = std::string("b-"),
		const std::string& plotName = std::string("plotXY"))
	{
		this->internal_plot_interface(x, y, lineFormat, plotName);
	}
	//! \overload
	template <typename Derived1, typename Derived2>
	inline void plot(
		const Eigen::MatrixBase<Derived1>& x,
		const Eigen::MatrixBase<Derived2>& y,
		const std::string& lineFormat = std::string("b-"),
		const std::string& plotName = std::string("plotXY"))
	{
		this->internal_plot_interface(x, y, lineFormat, plotName);
	}

	//! \overload
	template <typename T>
	void plot(
		const std::vector<T>& y,
		const std::string& lineFormat = std::string("b-"),
		const std::string& plotName = std::string("plotXY"))
	{
		this->internal_plot_interface(y, lineFormat, plotName);
	}
	//! \overload
	template <typename Derived>
	void plot(
		const Eigen::MatrixBase<Derived>& y,
		const std::string& lineFormat = std::string("b-"),
		const std::string& plotName = std::string("plotXY"))
	{
		this->internal_plot_interface(y, lineFormat, plotName);
	}

	/** Set the view area according to the passed coordinated. */
	void axis(
		float x_min, float x_max, float y_min, float y_max,
		bool aspectRatioFix = false);

	/** Enable/disable the fixed X/Y aspect ratio fix feature
	 * (default=disabled). */
	void axis_equal(bool enable = true);

	/** Fix automatically the view area according to existing graphs. */
	void axis_fit(bool aspectRatioFix = false);

	/** Plots a 2D ellipse given its mean, covariance matrix, and
	 *  Each call to this function creates a new plot, unless the plot name
	 * coincides with an already existing plot: in this case the new values are
	 * used to update this existing layer (this also applies to using the
	 * default plot name).
	 *  If "hold_on" is enabled, then every call will always create a new plot,
	 * even if no "plotName" is provided.
	 *
	 *  For a description of lineFormat see CDisplayWindowPlots::plot.
	 *  The "quantiles" value determines the confidence interval for the
	 * ellipse:
	 *     - 1 : 68.27% confidence interval
	 *     - 2 : 95.45%
	 *     - 3 : 99.73%
	 *     - 4 : 99.994%
	 * \note This method can be called with 2x2 fixed-sized or dynamic-size
	 * matrices of types: float or double.
	 * \sa axis, axis_equal, axis_fit, hold_on, hold_off
	 */
	template <typename T>
	void plotEllipse(
		const T mean_x, const T mean_y,
		const mrpt::math::CMatrixTemplateNumeric<T>& cov22,
		const float quantiles,
		const std::string& lineFormat = std::string("b-"),
		const std::string& plotName = std::string("plotEllipse"),
		bool showName = false);

	//! \overload
	template <typename T>
	void plotEllipse(
		const T mean_x, const T mean_y,
		const mrpt::math::CMatrixFixedNumeric<T, 2, 2>& cov22,
		const float quantiles,
		const std::string& lineFormat = std::string("b-"),
		const std::string& plotName = std::string("plotEllipse"),
		bool showName = false);

	/** Adds a bitmap image layer.
	 *  Each call to this function creates a new layer, unless the plot name
	 * coincides with an already existing plot: in this case the new values are
	 * used to update this existing layer (this also applies to using the
	 * default plot name).
	 *
	 * \sa axis, axis_equal, axis_fit, hold_on, hold_off
	 */
	void image(
		const mrpt::img::CImage& img, const float& x_left,
		const float& y_bottom, const float& x_width, const float& y_height,
		const std::string& plotName = std::string("image"));

	/** Remove all plot objects in the display.
	 * \sa plot
	 */
	void clear();

	/** Remove all plot objects in the display (clear and clf do exactly the
	 * same).
	 * \sa plot, hold_on, hold_off
	 */
	inline void clf() { clear(); }
	/** Enables keeping all the graphs, instead of overwritting them.
	 * \sa hold_off, plot
	 */
	void hold_on();

	/** Disables keeping all the graphs (this is the default behavior).
	 * \sa hold_on, plot
	 */
	void hold_off();

	/** Disables keeping all the graphs (this is the default behavior).
	 * \param label The text that appears in the new popup menu item.
	 * \param menuID Any positive number (0,1,..). Used to tell which menu was
	 * selected in the user callback.
	 * \sa setMenuCallback
	 */
	void addPopupMenuEntry(const std::string& label, int menuID);

	/** Must be called to have a callback when the user selects one of the
	 * user-defined entries in the popup menu.
	 * \sa addPopupMenuEntry
	 */
	void setMenuCallback(TCallbackMenu userFunction, void* userParam = nullptr);

};  // End of class def.
}  // namespace mrpt::gui

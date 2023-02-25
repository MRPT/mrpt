/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/gui/MRPT2NanoguiGLCanvas.h>
#include <mrpt/gui/internal/NanoGUICanvasHeadless.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/system/string_utils.h>  // firstNLines()

#include <mutex>
#include <string>

// Expose nanogui API to mrpt users, for direct use of nanogui classes.
#include <mrpt/config.h>
#if MRPT_HAS_NANOGUI
#include <nanogui/nanogui.h>

namespace mrpt::gui
{
/** Additional parameters to change the window behavior and OpenGL context
 */
struct CDisplayWindowGUI_Params
{
	CDisplayWindowGUI_Params() = default;

	bool resizable = true;
	bool fullscreen = false;
	int colorBits = 8;
	int alphaBits = 8;
	int depthBits = 24;
	int stencilBits = 8;
	int nSamples = 0;
	unsigned int glMajor = 3;
	unsigned int glMinor = 3;
	bool maximized = false;

	/// If true, will try to use GLES instead of OpenGL.
	/// Note that the glMajor and glMinor version should then be changed
	/// accordingly (e.g. to 1.5 instead of 3.3)
	bool gles_context = false;
};

/** A window with powerful GUI capabilities, via the nanogui library.
 *
 * You can add a background mrpt::opengl::Scene object rendered on the
 * background of the entire window by setting an object in field
 * `background_scene`, locking its mutex `background_scene_mtx`.
 *
 * Refer to nanogui API docs or [MRPT examples](examples.html) for further usage
 * examples. A typical lifecycle of a GUI app with this class might look like:
 *
 * \code
 * nanogui::init();
 * {
 *   mrpt::gui::CDisplayWindowGUI win;
 *   // Populate win adding UI controls, etc.
 *   // ...
 *   win.performLayout();
 *
 *   win.drawAll();
 *   win.setVisible(true);
 *   nanogui::mainloop();
 * }
 * nanogui::shutdown();
 * \endcode
 *
 *
 * ![mrpt::gui::CDisplayWindowGUI screenshot](preview_CDisplayWindowGUI.png)
 *
 * Create managed subwindows with createManagedSubWindow(), with built-in
 * support for minimize and restore.
 * See demo video in: https://www.youtube.com/watch?v=QKMzdlZRW50
 *
 *
 * \ingroup mrpt_gui_grp
 */
class CDisplayWindowGUI : public nanogui::Screen
{
   public:
	/** @name Callback functor types
	 * @{ */

	using loop_callback_t = std::function<void(void)>;
	using drop_files_callback_t = std::function<bool /*handled*/ (
		const std::vector<std::string>& /* filenames */)>;
	using keyboard_callback_t = std::function<bool /*handled*/ (
		int /*key*/, int /*scancode*/, int /*action*/, int /*modifiers*/)>;
	/** @} */

	/** @name Ctor and basic window set up
	 * @{ */
	using Ptr = std::shared_ptr<CDisplayWindowGUI>;
	using ConstPtr = std::shared_ptr<const CDisplayWindowGUI>;

	/// Regular ctor from caption and size
	CDisplayWindowGUI(
		const std::string& caption = std::string(), unsigned int width = 400,
		unsigned int height = 300,
		const CDisplayWindowGUI_Params& p = CDisplayWindowGUI_Params());

	virtual ~CDisplayWindowGUI() override;

	/** Class factory returning a smart pointer, equivalent to
	 * `std::make_shared<>(...)` */
	template <typename... Args>
	static Ptr Create(Args&&... args)
	{
		return std::make_shared<CDisplayWindowGUI>(std::forward<Args>(args)...);
	}

	/** Resizes the window */
	void resize(unsigned int width, unsigned int height);

	/** Changes the position of the window on the screen. */
	void setPos(int x, int y);

	/**  Changes the window title. */
	void setWindowTitle(const std::string& str);

	/** Sets the window icon, which must must either RGB or (preferred) RGBA
	 * channels. You can read it from a .png or .ico file using
	 * mrpt::img::CImage::loadFromFile().
	 *
	 * The image will be resized as needed. Good sizes include 16x16, 32x32 and
	 * 48x48.
	 *
	 * \note (New in MRPT 2.4.2)
	 */
	void setIcon(const mrpt::img::CImage& img);

	/** \overload From an icon stored in code as `const char*` exported in the
	 * "GIMP header image formar (RGB)", with a transparent color.
	 * \note (New in MRPT 2.4.2)
	 */
	void setIconFromData(
		const char* imgData, unsigned int width, unsigned int height,
		const uint8_t transparent);

	/** Every time the window is about to be repainted, an optional callback can
	 * be called, if provided via this method. This method can be safely called
	 * multiple times to register multiple callbacks.
	 *
	 * \note (New in MRPT 2.3.2)
	 */
	void addLoopCallback(const loop_callback_t& callback)
	{
		m_loopCallbacks.push_back(callback);
	}
	const auto& loopCallbacks() const { return m_loopCallbacks; }

	/** Handles drag-and drop events of files into the window.
	 * This method can be safely called multiple times to register multiple
	 * callbacks.
	 *
	 * \note (New in MRPT 2.3.2)
	 */
	void addDropFilesCallback(const drop_files_callback_t& callback)
	{
		m_dropFilesCallbacks.push_back(callback);
	}
	const auto& dropFilesCallbacks() const { return m_dropFilesCallbacks; }

	/** Handles keyboard events.
	 * This method can be safely called multiple times to register multiple
	 * callbacks.
	 *
	 * \note (New in MRPT 2.3.2)
	 */
	void addKeyboardCallback(const keyboard_callback_t& callback)
	{
		m_keyboardCallbacks.push_back(callback);
	}
	const auto& keyboardCallbacks() const { return m_keyboardCallbacks; }

	/** \note This call replaces *all* existing loop callbacks. See
	 * addLoopCallback().
	 * \deprecated In MRPT 2.3.2, replaced by addLoopCallback()
	 */
	void setLoopCallback(const loop_callback_t& callback)
	{
		m_loopCallbacks.clear();
		m_loopCallbacks.push_back(callback);
	}
	/** \deprecated In MRPT 2.3.2, replaced by loopCallbacks() */
	loop_callback_t loopCallback() const
	{
		return m_loopCallbacks.empty() ? loop_callback_t()
									   : m_loopCallbacks.at(0);
	}

	/** \note This call replaces *all* existing drop file callbacks. See
	 * addDropFilesCallback().
	 * \deprecated In MRPT 2.3.2, replaced by addDropFilesCallback()
	 */
	void setDropFilesCallback(const drop_files_callback_t& callback)
	{
		m_dropFilesCallbacks.clear();
		m_dropFilesCallbacks.push_back(callback);
	}
	/** \deprecated In MRPT 2.3.2, replaced by dropFilesCallbacks() */
	drop_files_callback_t dropFilesCallback() const
	{
		return m_dropFilesCallbacks.empty() ? drop_files_callback_t()
											: m_dropFilesCallbacks.at(0);
	}

	/** \note This call replaces *all* existing keyboard callbacks. See
	 * addKeyboardCallback().
	 * \deprecated In MRPT 2.3.2, replaced by addKeyboardCallback()
	 */
	void setKeyboardCallback(const keyboard_callback_t& callback)
	{
		m_keyboardCallbacks.clear();
		m_keyboardCallbacks.push_back(callback);
	}
	/** \deprecated In MRPT 2.3.2, replaced by keyboardCallbacks() */
	keyboard_callback_t keyboardCallback() const
	{
		return m_keyboardCallbacks.empty() ? keyboard_callback_t()
										   : m_keyboardCallbacks.at(0);
	}

	/** @} */

	/** @name Managed subwindows subsystem
	 * @{ */

	/** Creates and return a nanogui::Window, adds to it basic minimize/restore
	 * tool buttons and add it to the list of handled subwindows so it gets
	 * listed in the subwindows control UI.
	 * User should set a layout manager, width, height, etc. in the returned
	 * window as desired.
	 *
	 * The returned object is owned by the nanogui system, you should NOT delete
	 * it.
	 *
	 * The first time this is called, an additional subWindow will be created
	 * (default position:bottom left corner) to hold minimized windows.
	 * You can access and modify this windows via getSubWindowsUI().
	 *
	 * \note [New in MRPT 2.1.1]
	 */
	nanogui::Window* createManagedSubWindow(const std::string& title);

	nanogui::Window* getSubWindowsUI() { return m_subWindows.ui; }
	const nanogui::Window* getSubWindowsUI() const { return m_subWindows.ui; }

	/** Direct (read-only) access to managed subwindows by 0-based index
	 * (creation order).
	 *
	 * \sa createManagedSubWindow()
	 * \note [New in MRPT 2.3.1]
	 */
	const nanogui::Window* getSubwindow(size_t index) const
	{
		return m_subWindows.windows.at(index);
	}

	/** Get the number of managed subwindows. \sa createManagedSubWindow()
	 * \note [New in MRPT 2.3.1]
	 */
	size_t getSubwindowCount() const { return m_subWindows.windows.size(); }

	/** Minimize a subwindow. \note [New in MRPT 2.3.1] */
	void subwindowMinimize(size_t index) { m_subWindows.minimize(index); }

	/** Restore a minimized subwindow. \note [New in MRPT 2.3.1] */
	void subwindowRestore(size_t index) { m_subWindows.restore(index); }

	/** Forces focus on a subwindow. \note [New in MRPT 2.3.1] */
	void subwindowSetFocused(size_t index) { m_subWindows.setFocused(index); }

	/** @} */

	/** @name Access to full-window (background) GL scene
	 * @{ */

	mrpt::opengl::Scene::Ptr background_scene;
	std::mutex background_scene_mtx;

	CGlCanvasBase& camera() { return m_background_canvas; }
	const CGlCanvasBase& camera() const { return m_background_canvas; }

	/** @} */

	/** @name Direct access to underlying nanogui API
	 * @{ */

	nanogui::Screen* nanogui_screen()
	{
		return dynamic_cast<nanogui::Screen*>(this);
	}

	/** @} */

	virtual void drawContents() override;
	virtual void onIdleLoopTasks() override;

   protected:
	CDisplayWindowGUI(const CDisplayWindowGUI&) = delete;
	CDisplayWindowGUI& operator=(const CDisplayWindowGUI&) = delete;

	CDisplayWindowGUI(CDisplayWindowGUI&&) = delete;
	CDisplayWindowGUI& operator=(CDisplayWindowGUI&&) = delete;

	/** @name Internal virtual functions to handle GUI events
	 * @{ */
	virtual bool keyboardEvent(
		int key, int scancode, int action, int modifiers) override;
	virtual bool mouseMotionEvent(
		const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button,
		int modifiers) override;
	virtual bool mouseButtonEvent(
		const nanogui::Vector2i& p, int button, bool down,
		int modifiers) override;
	virtual bool scrollEvent(
		const nanogui::Vector2i& p, const nanogui::Vector2f& rel) override;
	virtual bool dropEvent(const std::vector<std::string>& filenames) override;
	/** @} */

	/** Used to keep track of mouse events on the camera */
	internal::NanoGUICanvasHeadless m_background_canvas;

	std::vector<loop_callback_t> m_loopCallbacks;
	std::vector<drop_files_callback_t> m_dropFilesCallbacks;
	std::vector<keyboard_callback_t> m_keyboardCallbacks;

	void createSubWindowsControlUI();

	class SubWindow : public nanogui::Window
	{
	   public:
		SubWindow(
			CDisplayWindowGUI& parentGui, int myIndex, Widget* parent,
			const std::string& title)
			: nanogui::Window(parent, title),
			  m_parentGui(parentGui),
			  m_subWindowIndex(myIndex)
		{
		}
		CDisplayWindowGUI& m_parentGui;
		const int m_subWindowIndex = 0;

		/// Handle a focus change event (default implementation: record the
		/// focus status, but do nothing)
		bool focusEvent(bool focused) override
		{
			if (focused)
				m_parentGui.m_subWindows.onSubWindowFocused(m_subWindowIndex);
			return nanogui::Window::focusEvent(focused);
		}
	};

	struct SubWindows
	{
		SubWindows(CDisplayWindowGUI& p) : parent(p) {}

		std::vector<SubWindow*> windows;
		nanogui::Window* ui = nullptr;
		nanogui::ComboBox* uiCombo = nullptr;

		void minimize(int index);
		void restore(int index);
		void setFocused(int index);

		void onSubWindowFocused(int index);

		CDisplayWindowGUI& parent;
	};

	SubWindows m_subWindows{*this};
};	// namespace mrpt::gui

#define NANOGUI_START_TRY                                                      \
	try                                                                        \
	{
#define NANOGUI_END_TRY(_parentWindowRef_)                                     \
	}                                                                          \
	catch (const std::exception& e)                                            \
	{                                                                          \
		const size_t maxLines = 7;                                             \
		const std::string sErr =                                               \
			mrpt::system::firstNLines(mrpt::exception_to_str(e), maxLines);    \
		auto dlg = new nanogui::MessageDialog(                                 \
			&_parentWindowRef_, nanogui::MessageDialog::Type::Warning,         \
			"Exception", sErr);                                                \
		dlg->requestFocus();                                                   \
		dlg->setCallback([](int /*result*/) {});                               \
	}

}  // namespace mrpt::gui
#endif	// MRPT_HAS_NANOGUI

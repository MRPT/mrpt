/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/gui/MRPT2NanoguiGLCanvas.h>
#include <mrpt/gui/internal/NanoGUICanvasHeadless.h>
#include <mrpt/opengl/COpenGLScene.h>

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
};

/** A window with powerful GUI capabilities, via the nanogui library.
 *
 * You can add a background mrpt::opengl::COpenGLScene object rendered on the
 * background of the entire window by setting an object in field
 * `background_scene`, locking its mutex `background_scene_mtx`.
 *
 * Refer to nanogui API docs or MRPT examples for further usage examples.
 * A typical lifecycle of a GUI app with this class might look like:
 *
 * \rst
 * .. code-block:: cpp
 *
 *    nanogui::init();
 *    {
 *      mrpt::gui::CDisplayWindowGUI win;
 *      win.drawAll();
 *      win.setVisible(true);
 *      nanogui::mainloop();
 *    }
 *    nanogui::shutdown();
 * \endrst
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
	/** @name Ctor and basic window set up
	 * @{ */
	using Ptr = std::shared_ptr<CDisplayWindowGUI>;
	using ConstPtr = std::shared_ptr<const CDisplayWindowGUI>;

	CDisplayWindowGUI(
		const std::string& caption = std::string(), unsigned int width = 400,
		unsigned int height = 300,
		const CDisplayWindowGUI_Params& p = CDisplayWindowGUI_Params());

	virtual ~CDisplayWindowGUI() override;

	/** Class factory returning a smart pointer */
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

	/** Every time the window is about to be repainted, an optional callback can
	 * be called, if provided via this method. */
	void setLoopCallback(const std::function<void(void)>& callback)
	{
		m_loopCallback = callback;
	}
	const auto& loopCallback() const { return m_loopCallback; }

	/** Sets a handle for file drop events */
	void setDropFilesCallback(
		const std::function<
			bool(const std::vector<std::string>& /* filenames */)>& callback)
	{
		m_dropFilesCallback = callback;
	}
	const auto& dropFilesCallback() const { return m_dropFilesCallback; }

	void setKeyboardCallback(const std::function<bool(
								 int /*key*/, int /*scancode*/, int /*action*/,
								 int /*modifiers*/)>& callback)
	{
		m_keyboardCallback = callback;
	}
	const auto& keyboardCallback() const { return m_keyboardCallback; }

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

	/** @} */

	/** @name Access to full-window (background) GL scene
	 * @{ */

	mrpt::opengl::COpenGLScene::Ptr background_scene;
	std::mutex background_scene_mtx;

	CGlCanvasBase& camera() { return m_background_canvas; }
	const CGlCanvasBase& camera() const { return m_background_canvas; }

	/** @} */

	/** @name Direct access to underlying nanogui API
	 * @{ */

	nanogui::Screen* nanogui_screen() { return this; }

	/** @} */

   protected:
	CDisplayWindowGUI(const CDisplayWindowGUI&) = delete;
	CDisplayWindowGUI& operator=(const CDisplayWindowGUI&) = delete;

	CDisplayWindowGUI(CDisplayWindowGUI&&) = delete;
	CDisplayWindowGUI& operator=(CDisplayWindowGUI&&) = delete;

	/** the pointer is owned by the parent class Screen, no need to delete
	 * it */
	virtual bool keyboardEvent(
		int key, int scancode, int action, int modifiers) override;
	virtual void drawContents() override;

	/** @name Internal virtual functions to handle GUI events
	 * @{ */
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

	std::function<void(void)> m_loopCallback;
	// Returns true if handled
	std::function<bool(const std::vector<std::string>& /* filenames */)>
		m_dropFilesCallback;
	// Returns true if handled
	std::function<bool(
		int /*key*/, int /*scancode*/, int /*action*/, int /*modifiers*/)>
		m_keyboardCallback;

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
};

#define NANOGUI_START_TRY \
	try                   \
	{
#define NANOGUI_END_TRY(_parentWindowRef_)                             \
	}                                                                  \
	catch (const std::exception& e)                                    \
	{                                                                  \
		const auto sErr = mrpt::exception_to_str(e);                   \
		auto dlg = new nanogui::MessageDialog(                         \
			&_parentWindowRef_, nanogui::MessageDialog::Type::Warning, \
			"Exception", sErr);                                        \
		dlg->setCallback([](int /*result*/) {});                       \
	}

}  // namespace mrpt::gui
#endif  // MRPT_HAS_NANOGUI

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers
//
#include <mrpt/config.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/gui/default_mrpt_glfw_icon.h>

#if MRPT_HAS_OPENCV
#include <mrpt/3rdparty/do_opencv_includes.h>
#endif

using namespace mrpt::gui;

#if MRPT_HAS_NANOGUI

CDisplayWindowGUI::CDisplayWindowGUI(
	const std::string& caption, unsigned int width, unsigned int height,
	const CDisplayWindowGUI_Params& p)
	: nanogui::Screen(
		  Eigen::Vector2i(width, height), caption, p.resizable, p.fullscreen,
		  p.colorBits, p.alphaBits, p.depthBits, p.stencilBits, p.nSamples,
		  p.glMajor, p.glMinor, p.maximized, p.gles_context)
{
	setIconFromData(
		default_mrpt_glfw_icon, default_mrpt_glfw_icon_width,
		default_mrpt_glfw_icon_height,
		default_mrpt_glfw_icon_transparent_color);
}

CDisplayWindowGUI::~CDisplayWindowGUI()
{
	// Close window:
	nanogui::Screen::setVisible(false);
}

void CDisplayWindowGUI::onIdleLoopTasks()
{
	// If provided, call the user loop code:
	for (const auto& callback : m_loopCallbacks)
	{
		try
		{
			callback();
		}
		catch (const std::exception& e)
		{
			std::cerr << "[CDisplayWindowGUI] Exception in loop callback:\n"
					  << e.what() << std::endl;
		}
	}
}

void CDisplayWindowGUI::drawContents()
{
	// Optional: render background scene.
	std::lock_guard<std::mutex> lck(background_scene_mtx);
	if (!background_scene) return;

	try
	{
		// We need the size of the viewport:
		GLint win_dims[4];
		glGetIntegerv(GL_VIEWPORT, win_dims);

		// Set the camera params in the scene:
		mrpt::opengl::Viewport::Ptr view =
			background_scene->getViewport("main");
		if (!view)
			THROW_EXCEPTION(
				"Fatal error: there is no 'main' viewport in the 3D scene!");
		mrpt::opengl::CCamera& cam = view->getCamera();
		m_background_canvas.updateCameraParams(cam);

		for (const auto& m_viewport : background_scene->viewports())
			m_viewport->render(
				win_dims[2], win_dims[3], win_dims[0], win_dims[1]);
	}
	catch (const std::exception& e)
	{
		std::cerr << "[CDisplayWindowGUI::drawContents] Exception:\n"
				  << mrpt::exception_to_str(e);
	}
}

void CDisplayWindowGUI::resize(unsigned int width, unsigned int height)
{
	Screen::setSize({width, height});
}

void CDisplayWindowGUI::setPos(int x, int y) { Screen::setPosition({x, y}); }

void CDisplayWindowGUI::setWindowTitle(const std::string& str)
{
	Screen::setCaption(str);
}

void CDisplayWindowGUI::setIcon(const mrpt::img::CImage& img)
{
#if MRPT_HAS_OPENCV
	const cv::Mat& cvIn = img.asCvMatRef();

	// We need the image to be in RGBA format:
	cv::Mat icon;
	if (cvIn.channels() == 3)
	{
		// BGR: add alpha
		cv::cvtColor(cvIn, icon, cv::ColorConversionCodes::COLOR_BGR2RGBA);
	}
	else if (cvIn.channels() == 4)
	{
		// BGR: add alpha
		cv::cvtColor(cvIn, icon, cv::ColorConversionCodes::COLOR_BGRA2RGBA);
	}
	else
	{
		THROW_EXCEPTION("Icon image: expected either RGB or RGBA input image.");
	}

	// Set MRPT icon:
	const cv::Size iconSize = icon.size();

	GLFWimage images;
	images.width = iconSize.width;
	images.height = iconSize.height;
	images.pixels = icon.data;

// glfwSetWindowIcon added in glfw 3.2
#if GLFW_VERSION_MAJOR > 3 ||                                                  \
	(GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 2)
	glfwSetWindowIcon(screen()->glfwWindow(), 1, &images);
#endif

#endif
}

void CDisplayWindowGUI::setIconFromData(
	const char* imgData, unsigned int width, unsigned int height,
	const uint8_t transp)
{
	std::vector<uint8_t> dat;

	/* imgData: GIMP header image file format (RGB) */

	/*  Call this macro repeatedly.  After each use, the pixel data can be
	 * extracted  */
#define HEADER_PIXEL(data, pixel)                                              \
	{                                                                          \
		pixel[0] = (((data[0] - 33) << 2) | ((data[1] - 33) >> 4));            \
		pixel[1] = ((((data[1] - 33) & 0xF) << 4) | ((data[2] - 33) >> 2));    \
		pixel[2] = ((((data[2] - 33) & 0x3) << 6) | ((data[3] - 33)));         \
		data += 4;                                                             \
	}

	// GLFW image format: RGBA
	dat.resize(width * height * 4);
	const char* in = imgData;
	uint8_t* out = dat.data();

	for (unsigned int y = 0; y < height; y++)
	{
		for (unsigned int x = 0; x < width; x++)
		{
			// RGB
			HEADER_PIXEL(in, out);
			// alpha
			out[3] = (out[0] == transp && out[1] == transp && out[2] == transp)
				? 0x00
				: 0xff;
			out += 4;
		}
	}

	// Set MRPT icon:
	GLFWimage images;
	images.width = width;
	images.height = height;
	images.pixels = dat.data();

// glfwSetWindowIcon added in glfw 3.2
#if GLFW_VERSION_MAJOR > 3 ||                                                  \
	(GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 2)
	glfwSetWindowIcon(screen()->glfwWindow(), 1, &images);
#endif

#undef HEADER_PIXEL
}

bool CDisplayWindowGUI::mouseButtonEvent(
	const nanogui::Vector2i& p, int button, bool down, int modifiers)
{
	if (!Screen::mouseButtonEvent(p, button, down, modifiers))
		m_background_canvas.mouseButtonEvent(p, button, down, modifiers);

	return true;
}

bool CDisplayWindowGUI::mouseMotionEvent(
	const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button,
	int modifiers)
{
	if (!Screen::mouseMotionEvent(p, rel, button, modifiers))
		m_background_canvas.mouseMotionEvent(p, rel, button, modifiers);

	return true;
}

bool CDisplayWindowGUI::scrollEvent(
	const nanogui::Vector2i& p, const nanogui::Vector2f& rel)
{
	if (!Screen::scrollEvent(p, rel)) m_background_canvas.scrollEvent(p, rel);

	return true;
}

bool CDisplayWindowGUI::dropEvent(const std::vector<std::string>& filenames)
{
	for (const auto& callback : m_dropFilesCallbacks)
	{
		try
		{
			if (callback(filenames)) return true;
		}
		catch (const std::exception& e)
		{
			std::cerr << "[CDisplayWindowGUI] Exception in drop file event "
						 "callback:\n"
					  << e.what() << std::endl;
		}
	}

	return false;
}

bool CDisplayWindowGUI::keyboardEvent(
	int key, int scancode, int action, int modifiers)
{
	for (const auto& callback : m_keyboardCallbacks)
	{
		try
		{
			if (callback(key, scancode, action, modifiers)) return true;
		}
		catch (const std::exception& e)
		{
			std::cerr
				<< "[CDisplayWindowGUI] Exception in keyboard event callback:\n"
				<< e.what() << std::endl;
		}
	}

	if (Screen::keyboardEvent(key, scancode, action, modifiers)) return true;

	// Process special key events?
	return false;
}

nanogui::Window* CDisplayWindowGUI::createManagedSubWindow(
	const std::string& title)
{
	constexpr std::size_t MAX_MINIMIZED_TITLE_LEN = 20;

	// Create UI on first call:
	// Create it before the user-requested window so it doesn't get on top.
	createSubWindowsControlUI();

	// Create subwindow:
	const int thisWinIndex = m_subWindows.windows.size();

	auto w = new SubWindow(*this, thisWinIndex, this, title);
	m_subWindows.windows.push_back(w);

	// Append to combo:
	ASSERT_(m_subWindows.uiCombo);
	auto items = m_subWindows.uiCombo->items();
	auto itemsShort = m_subWindows.uiCombo->itemsShort();

	items.push_back(title);
	itemsShort.push_back(title.substr(0, MAX_MINIMIZED_TITLE_LEN));

	const auto oldSelected = m_subWindows.uiCombo->selectedIndex();

	m_subWindows.uiCombo->setItems(items, itemsShort);
	m_subWindows.uiCombo->setSelectedIndex(oldSelected > 0 ? oldSelected : 0);

	// Add minimize/maximize buttons:
	w->buttonPanel()
		->add<nanogui::Button>("", ENTYPO_ICON_MINUS)
		->setCallback(
			[this, thisWinIndex]() { m_subWindows.minimize(thisWinIndex); });

	return w;
}

void CDisplayWindowGUI::createSubWindowsControlUI()
{
	constexpr int DEFAULT_WIDTH = 120, DEFAULT_HEIGHT = 40;
	constexpr int HIDEN_WIDTH = 35, HIDEN_HEIGHT = 35;

	if (m_subWindows.ui) return;

	auto& w = m_subWindows.ui;
	w = new nanogui::Window(this, "");

	w->setLayout(new nanogui::BoxLayout(
		nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 1, 1));

	w->setSize({DEFAULT_WIDTH, DEFAULT_HEIGHT});
	w->setPosition({0, 0});
	nanogui::Theme* modTheme = new nanogui::Theme(screen()->nvgContext());
	modTheme->mWindowHeaderHeight = 1;
	w->setTheme(modTheme);

	auto stackPanels = w->add<nanogui::StackedWidget>();

	auto page1 = new nanogui::Widget(nullptr);
	auto page2 = new nanogui::Widget(nullptr);

	stackPanels->addChild(0, page1);
	stackPanels->addChild(1, page2);
	stackPanels->setSelectedIndex(0);

	page1->setLayout(new nanogui::BoxLayout(
		nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 1, 1));
	page2->setLayout(new nanogui::BoxLayout(
		nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 1, 1));

	auto btnUnhide =
		page2->add<nanogui::Button>("", ENTYPO_ICON_CHEVRON_WITH_CIRCLE_RIGHT);
	btnUnhide->setCallback([=]() {
		stackPanels->setSelectedIndex(0);
		w->setFixedSize({0, 0});
		this->performLayout();
	});
	btnUnhide->setTooltip("Unhide this panel");

	m_subWindows.uiCombo = page1->add<nanogui::ComboBox>();

	m_subWindows.uiCombo->setCallback([this](int index) {
		m_subWindows.restore(index);
		m_subWindows.setFocused(index);
	});

	auto pn = page1->add<nanogui::Widget>();
	pn->setLayout(new nanogui::BoxLayout(
		nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 4, 4));

	auto btnMin = pn->add<nanogui::Button>("", ENTYPO_ICON_ALIGN_BOTTOM);
	btnMin->setCallback([this]() {
		m_subWindows.minimize(m_subWindows.uiCombo->selectedIndex());
	});
	btnMin->setTooltip("Minimize");

	auto btnMax = pn->add<nanogui::Button>("", ENTYPO_ICON_BROWSER);
	btnMax->setCallback([this]() {
		m_subWindows.restore(m_subWindows.uiCombo->selectedIndex());
	});
	btnMax->setTooltip("Restore selected");

	auto btnMinAll = pn->add<nanogui::Button>("All", ENTYPO_ICON_ALIGN_BOTTOM);
	btnMinAll->setCallback([this]() {
		for (int i = 0; i < static_cast<int>(m_subWindows.windows.size()); i++)
			m_subWindows.minimize(i);
	});
	btnMinAll->setTooltip("Minimize all");

	auto btnMaxAll = pn->add<nanogui::Button>("All", ENTYPO_ICON_BROWSER);
	btnMaxAll->setCallback([this]() {
		for (int i = 0; i < static_cast<int>(m_subWindows.windows.size()); i++)
			m_subWindows.restore(i);
	});
	btnMaxAll->setTooltip("Restore all");

	auto btnHide =
		pn->add<nanogui::Button>("", ENTYPO_ICON_CHEVRON_WITH_CIRCLE_LEFT);
	btnHide->setCallback([=]() {
		stackPanels->setSelectedIndex(1);
		w->setFixedSize({HIDEN_WIDTH, HIDEN_HEIGHT});
		this->performLayout();
	});
	btnHide->setTooltip("Hide this panel");
}

void CDisplayWindowGUI::SubWindows::minimize(int index)
{
	if (index < 0 || index > static_cast<int>(windows.size())) return;
	auto w = windows.at(index);

	w->setVisible(false);
	parent.performLayout();
}

void CDisplayWindowGUI::SubWindows::restore(int index)
{
	if (index < 0 || index > static_cast<int>(windows.size())) return;
	auto w = windows.at(index);

	w->setVisible(true);
	parent.performLayout();
}

void CDisplayWindowGUI::SubWindows::setFocused(int index)
{
	const int n = static_cast<int>(windows.size());
	if (index >= 0 && index < n) windows.at(index)->requestFocus();
}

void CDisplayWindowGUI::SubWindows::onSubWindowFocused(int index)
{
	const int n = static_cast<int>(uiCombo->items().size());
	if (index >= 0 && index < n) uiCombo->setSelectedIndex(index);
}

#endif	// MRPT_HAS_NANOGUI

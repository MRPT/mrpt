/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/containers/yaml.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/random.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/thread_name.h>

#define USE_NANOGUI_WINDOW

#ifdef USE_NANOGUI_WINDOW
#include <mrpt/gui/CDisplayWindowGUI.h>
#else
#include <mrpt/gui/CDisplayWindow.h>
using my_window_t = mrpt::gui::CDisplayWindow;
#endif

#include <iostream>
#include <list>
#include <mutex>
#include <thread>

static const int RENDER_WIDTH = 600, RENDER_HEIGHT = 480;

// sample scene:
static mrpt::opengl::COpenGLScene::Ptr generate_example_scene()
{
	auto s = mrpt::opengl::COpenGLScene::Create();

	if (0)
	{
		auto obj = mrpt::opengl::CAxis::Create(-5, -5, -5, 5, 5, 5);
		s->insert(obj);
	}
	{
		auto obj = mrpt::opengl::CSphere::Create(1.0f);
		obj->setColor_u8(0xff, 0x00, 0x00);
		obj->setLocation({1.0, 1.0, 1.0});
		s->insert(obj);
	}
	{
		auto obj = mrpt::opengl::CSphere::Create(0.25f);
		obj->setColor_u8(0x00, 0x00, 0xff);
		obj->setLocation({-1.0, -1.0, 0.25});
		obj->enableDrawSolid3D(false);
		s->insert(obj);
	}

	return s;
}

mrpt::opengl::COpenGLScene::Ptr commonScene;
mrpt::system::CTimeLogger profiler;

struct RenderResult
{
	RenderResult() = default;

	std::string threadName;
	mrpt::img::CImage img;
	std::string labelText;
};

std::list<RenderResult> renderOutputs;
std::mutex renderOutputs_mtx;

auto& rng = mrpt::random::getRandomGenerator();
std::mutex rngMtx;

static void renderer_thread_impl(
	const std::string name, const int period_ms, const int numImgs)
{
	using namespace std::string_literals;

	mrpt::system::thread_name(name);  // for debuggers

	mrpt::opengl::CFBORender render(RENDER_WIDTH, RENDER_HEIGHT);
	mrpt::img::CImage frame(RENDER_WIDTH, RENDER_HEIGHT, mrpt::img::CH_RGB);

	// here you can put your preferred camera rendering position
	{
		auto& camera = render.getCamera(*commonScene);
		camera.setOrthogonal(false);

		auto lck = mrpt::lockHelper(rngMtx);
		camera.setZoomDistance(rng.drawUniform(15.0, 40.0));
		camera.setElevationDegrees(rng.drawUniform(20.0, 70.0));
		camera.setAzimuthDegrees(rng.drawUniform(-60.0, 60.0));

#if 0
		mrpt::containers::yaml d = mrpt::containers::yaml::Map();
		camera.toYAMLMap(d);
		std::cout << "Thread: " << name << "\nCamera:\n"
				  << d << "\n"
				  << std::endl;
#endif
	}

	const auto nameProfiler = name + "_render"s;

	for (int i = 0; i < numImgs; i++)
	{
		// render the scene
		if (i > 0) profiler.enter(nameProfiler);

		render.render_RGB(*commonScene, frame);

		if (i > 0) profiler.leave(nameProfiler);

		RenderResult res;
		res.img = frame.makeDeepCopy();
		res.labelText = mrpt::format("Img #%i", i);
		res.threadName = name;

		{
			auto lck = mrpt::lockHelper(renderOutputs_mtx);
			renderOutputs.emplace_back(std::move(res));
		}
		std::cerr << "Thread '" << name << "' img #" << i << " done."
				  << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
	}
	std::cout << "\nRendering thread '" << name << "' ends." << std::endl;
}

static void renderer_thread(
	const std::string name, const int period_ms, const int numImgs)
{
	try
	{
		renderer_thread_impl(name, period_ms, numImgs);
	}
	catch (const std::exception& e)
	{
		std::cerr << "Thread '" << name << "' exception: " << e.what()
				  << std::endl;
	}
}

#ifndef USE_NANOGUI_WINDOW
// wxWidgets frontend:
static void viz_thread()
{
	const double MAX_TIME = 10.0;
	const double t0 = mrpt::Clock::nowDouble();

	std::map<std::string, my_window_t::Ptr> wins;

	double t = 0;

	while ((t = mrpt::Clock::nowDouble() - t0) < MAX_TIME)
	{
		std::list<RenderResult> done;
		{
			auto lck = mrpt::lockHelper(renderOutputs_mtx);
			std::swap(done, renderOutputs);
		}

		for (auto& r : done)
		{
			auto& win = wins[r.threadName];
			if (!win)
			{
				// first time:
				win = my_window_t::Create(
					r.threadName, r.img.getWidth(), r.img.getHeight());
			}

			// update image:
			r.img.textOut(5, 5, r.labelText, mrpt::img::TColor::white());
			win->showImage(r.img);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1));

		std::cout << "Showing images from working threads... " << t << "/"
				  << MAX_TIME << "  \r";
	};

	std::cout << "\nVisualization thread ends." << std::endl;
}
#else
// nanogui frontend
static void viz_thread()
{
	try
	{
		nanogui::init();

		mrpt::gui::CDisplayWindowGUI_Params winP;

		auto win = mrpt::gui::CDisplayWindowGUI::Create("main", 800, 600, winP);

		nanogui::Window* winInput = new nanogui::Window(win.get(), "Dummy win");
		winInput->setPosition(nanogui::Vector2i(10, 50));
		winInput->setLayout(new nanogui::GroupLayout());
		winInput->setFixedWidth(350);

		win->performLayout();
		win->drawAll();
		win->setVisible(true);

#if 1
		win->background_scene_mtx.lock();
		win->background_scene = commonScene;
		win->background_scene_mtx.unlock();
#endif

		nanogui::mainloop();
		nanogui::shutdown();

		std::cout << "\nVisualization thread ends." << std::endl;
	}
	catch (const std::exception& e)
	{
		std::cerr << "[viz_thread] Error:\n" << e.what() << std::endl;
	}
}

#endif

// ------------------------------------------------------
//				TestMultithreadRendering
// ------------------------------------------------------
static int TestOffscreenRender()
{
	commonScene = generate_example_scene();

	std::vector<std::thread> allThreads;

	allThreads.emplace_back(&viz_thread);

#if 1
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	allThreads.emplace_back(
		&renderer_thread, "one", 20 /*period*/, 400 /*nImgs*/);

	allThreads.emplace_back(
		&renderer_thread, "two", 10 /*period*/, 700 /*nImgs*/);
#endif

	for (auto& t : allThreads)
		if (t.joinable()) t.join();

	return 0;
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char* argv[])
{
	try
	{
		return TestOffscreenRender();
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}

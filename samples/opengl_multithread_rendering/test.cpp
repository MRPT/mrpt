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
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/random.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
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

auto& rng = mrpt::random::getRandomGenerator();
std::mutex rngMtx;

// sample scene:
static mrpt::opengl::COpenGLScene::Ptr generate_example_scene()
{
	auto s = mrpt::opengl::COpenGLScene::Create();

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
	{
		auto obj = mrpt::opengl::CPointCloud::Create();

		auto lck = mrpt::lockHelper(rngMtx);
		for (int i = 0; i < 200; i++)
		{
			obj->insertPoint(
				rng.drawUniform(-3.0, 0.0), rng.drawUniform(-3.0, 0.0),
				rng.drawUniform(-3.0, 0.0));
		}
		obj->setPointSize(3.0f);
		s->insert(obj);
	}
	{
		using namespace std::string_literals;

		const std::string texture_file = mrpt::system::getShareMRPTDir() +
			"datasets/sample-texture-terrain.jpg"s;

		auto obj = mrpt::opengl::CMesh::Create();

		mrpt::img::CImage im;

		const int W = 128, H = 128;
		mrpt::math::CMatrixDynamic<float> Z(H, W);
		for (int r = 0; r < H; r++)
			for (int c = 0; c < W; c++)
				Z(r, c) = sin(0.05 * (c + r) - 0.5) * cos(0.9 - 0.03 * r);

		if (im.loadFromFile(texture_file))
		{
			obj->setZ(Z);
			obj->assignImageAndZ(im, Z);
			obj->setLocation(-5, 0, 0);
			obj->cullFaces(mrpt::opengl::TCullFace::BACK);
		}
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
#elif MRPT_HAS_NANOGUI
// nanogui frontend
static void viz_thread()
{
	try
	{
		mrpt::system::thread_name("viz");  // for debuggers

		nanogui::init();

		auto win = mrpt::gui::CDisplayWindowGUI::Create("main", 800, 600);

		win->performLayout();
		win->drawAll();
		win->setVisible(true);

#if 1
		win->background_scene_mtx.lock();
		win->background_scene = commonScene;
		win->background_scene_mtx.unlock();
#endif

		struct SubWindowData
		{
			SubWindowData() = default;

			nanogui::Window* win = nullptr;
			mrpt::gui::MRPT2NanoguiGLCanvas* glControl = nullptr;
			nanogui::Label* label = nullptr;
		};

		std::map<std::string, SubWindowData> subWindows;

		win->addLoopCallback([&]() {
			std::list<RenderResult> done;
			{
				auto lck = mrpt::lockHelper(renderOutputs_mtx);
				std::swap(done, renderOutputs);
			}
			if (done.empty()) return;

			for (auto& r : done)
			{
				auto& sw = subWindows[r.threadName];
				if (!sw.win)
				{
					// Add subwindow:
					sw.win = new nanogui::Window(win.get(), r.threadName);
					sw.win->setLayout(new nanogui::GroupLayout());

					sw.label = sw.win->add<nanogui::Label>("label");

					sw.glControl =
						sw.win->add<mrpt::gui::MRPT2NanoguiGLCanvas>();
					sw.win->setPosition(
						{5 + 100 * (subWindows.size() - 1), 10});
					sw.win->setFixedWidth(350);
					{
						auto scene = mrpt::opengl::COpenGLScene::Create();
						auto lck = mrpt::lockHelper(sw.glControl->scene_mtx);
						sw.glControl->scene = std::move(scene);
					}
					win->performLayout();
				}

				{
					auto lck = mrpt::lockHelper(sw.glControl->scene_mtx);
					sw.glControl->scene->getViewport()->setImageView(
						std::move(r.img));
				}
				sw.label->setCaption(r.labelText);
			}
		});

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

	{
		// ==================================================================
		//                         ** CRITICAL **
		// Create dummy FBO Renderer to init GL as required by FBOs *before*
		// nanogui initializes it. Otherwise, GL context errors will be
		// raised by FBOs later on.
		// ==================================================================
		mrpt::opengl::CFBORender render(10, 10);
	}

	allThreads.emplace_back(&viz_thread);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	allThreads.emplace_back(
		&renderer_thread, "one", 5 /*period*/, 600 /*nImgs*/);

	allThreads.emplace_back(
		&renderer_thread, "two", 6 /*period*/, 700 /*nImgs*/);

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
#if MRPT_HAS_NANOGUI
		return TestOffscreenRender();
#else
		std::cerr << "This example requires MRPT built with NANOGUI.\n";
		return 1;
#endif
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}

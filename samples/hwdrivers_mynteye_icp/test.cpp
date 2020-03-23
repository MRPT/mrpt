/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/system/filesystem.h>

#if MRPT_HAS_NANOGUI

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::hwdrivers;
using namespace mrpt::img;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::tfest;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::opengl;
using namespace std;

const double KEYFRAMES_MIN_DISTANCE = 0.50;  // meters
const double KEYFRAMES_MIN_ANG = 20.0_deg;

// Thread for grabbing: Do this is another thread so we divide rendering and
// grabbing
//   and exploit multicore CPUs.
struct TThreadParam
{
	TThreadParam() = default;
	std::atomic_bool quit{false};
	std::atomic<double> Hz{0};

	CObservation3DRangeScan::Ptr new_obs;
};

void thread_grabbing(TThreadParam& p)
{
	try
	{
		mrpt::hwdrivers::CCameraSensor cam;

		const std::string str =
			"[CONFIG]\n"
			"grabber_type=myntd\n";

		mrpt::config::CConfigFileMemory cfg(str);
		cam.loadConfig(cfg, "CONFIG");

		// Open:
		cout << "Calling initialize()...";
		cam.initialize();
		cout << "OK\n";

		CTicTac tictac;
		int nImgs = 0;

		while (!p.quit)
		{
			// Grab new observation from the camera:

			cam.doProcess();

			mrpt::hwdrivers::CGenericSensor::TListObservations obss;
			cam.getObservations(obss);

			if (obss.empty())
			{
				std::this_thread::sleep_for(10ms);
				continue;
			}

			auto obs = mrpt::ptr_cast<mrpt::obs::CObservation3DRangeScan>::from(
				obss.begin()->second);

			if (!obs) continue;

			std::atomic_store(&p.new_obs, obs);

			nImgs++;
			if (nImgs > 10)
			{
				p.Hz = nImgs / tictac.Tac();
				nImgs = 0;
				tictac.Tic();
			}
		}
	}
	catch (const std::exception& e)
	{
		cout << "Exception in Kinect thread: " << mrpt::exception_to_str(e)
			 << endl;
		p.quit = true;
	}
}

// ------------------------------------------------------
//				Test_3DCamICP
// ------------------------------------------------------
void Test_3DCamICP()
{
	// Launch grabbing thread:
	// --------------------------------------------------------
	TThreadParam thrPar;
	std::thread thHandle = std::thread(thread_grabbing, std::ref(thrPar));

	// Wait until data stream starts so we can say for sure the sensor has been
	// initialized OK:
	cout << "Waiting for sensor initialization...\n";
	do
	{
		CObservation3DRangeScan::Ptr possiblyNewObs =
			std::atomic_load(&thrPar.new_obs);
		if (possiblyNewObs && possiblyNewObs->timestamp != INVALID_TIMESTAMP)
			break;
		else
			std::this_thread::sleep_for(10ms);
	} while (!thrPar.quit);

	// Check error condition:
	if (thrPar.quit) return;

	// Create window and prepare OpenGL object in the scene:
	// --------------------------------------------------------
	nanogui::init();

	mrpt::gui::CDisplayWindowGUI win("3D camera ICP demo", 1000, 800);

	win.camera().setAzimuthDegrees(140.0f);
	win.camera().setElevationDegrees(20.0f);
	win.camera().setZoomDistance(8.0f);
	win.camera().setCameraFOV(50.0f);
	win.camera().setCameraPointing(2.5, 0, 0);

	// Aux structure to share UI data between threads.
	struct ui_data_t
	{
		// In the 2D image:
		std::atomic_bool SHOW_FEAT_IDS = true;
		std::atomic_bool SHOW_RESPONSES = true;

		std::atomic_bool hasToReset = false;

		unsigned int icpDecimation = 16;

		std::mutex strStatuses_mtx;
		std::array<std::string, 4> strStatuses;

		opengl::COpenGLViewport::Ptr viewInt;
		std::mutex* viewInt_mtx = nullptr;

		// Set defaults:
		ui_data_t() = default;
	};

	ui_data_t ui_data;

	// The main function to be run in parallel to check for new observations,
	// update the GL objects, etc.
	auto lambdaUpdateThread = [&win, &thrPar, &ui_data]() {
		auto gl_points = mrpt::opengl::CPointCloudColoured::Create();
		auto gl_keyframes = mrpt::opengl::CSetOfObjects::Create();
		auto gl_points_map = mrpt::opengl::CPointCloudColoured::Create();
		auto gl_cur_cam_corner = mrpt::opengl::stock_objects::CornerXYZ(0.4f);
		gl_points->setPointSize(1.25f);
		gl_points_map->setPointSize(1.5f);

		{
			auto scene = mrpt::opengl::COpenGLScene::Create();

			// Create the Opengl object for the point cloud:
			scene->insert(gl_points_map);
			scene->insert(gl_points);
			scene->insert(gl_keyframes);
			scene->insert(mrpt::opengl::CGridPlaneXY::Create());

			scene->insert(gl_cur_cam_corner);

			win.background_scene_mtx.lock();
			win.background_scene = std::move(scene);
			win.background_scene_mtx.unlock();
		}

		// The 6D path of the Kinect camera.
		std::vector<TPose3D> camera_key_frames_path;

		// wrt last pose in "camera_key_frames_path"
		CPose3D currentCamPose_wrt_last;

		unsigned int step_num = 0;

		// Need to update gl_keyframes from camera_key_frames_path??
		bool gl_keyframes_must_refresh = true;

		CObservation3DRangeScan::Ptr cur_obs;
		CColouredPointsMap::Ptr cur_points, prev_points;

		// Global points map:
		CColouredPointsMap globalPtsMap;

		globalPtsMap.colorScheme.scheme =
			CColouredPointsMap::cmFromIntensityImage;  // Take points color from
		// RGB+D observations

		mrpt::slam::CICP icp;
		icp.options.maxIterations = 80;
		icp.options.thresholdDist = 0.10;  // [m]
		icp.options.thresholdAng = 1.0_deg;
		icp.options.ALFA = 0.001;  // wun with only 1 set of thresholds

		mrpt::poses::CPose3D lastIcpRelPose;

		// Should we exit?
		while (!thrPar.quit)
		{
			std::this_thread::sleep_for(5ms);

			CObservation3DRangeScan::Ptr possiblyNewObs =
				std::atomic_load(&thrPar.new_obs);
			if (!possiblyNewObs ||
				possiblyNewObs->timestamp == INVALID_TIMESTAMP ||
				(cur_obs && possiblyNewObs->timestamp == cur_obs->timestamp))
				continue;  // No new data

			// It IS a new observation:
			cur_obs = possiblyNewObs;

			// Unproject 3D points:
			if (!cur_points)
				cur_points = CColouredPointsMap::Create();
			else
				cur_points->clear();

			// Also, unproject all for viz:
			mrpt::obs::T3DPointsProjectionParams pp;
			pp.decimation = ui_data.icpDecimation;

			cur_obs->unprojectInto(*cur_points, pp);

			// ICP -------------------------------------------
			// The grabbed image:
			CImage theImg = cur_obs->intensityImage;

			CPose3DPDF::Ptr icp_out;
			mrpt::slam::CICP::TReturnInfo icp_res;

			if (!prev_points)
			{
				// Make a deep copy:
				prev_points = CColouredPointsMap::Create(*cur_points);
			}

			icp_out = icp.Align3D(
				prev_points.get(), cur_points.get(), lastIcpRelPose, icp_res);

			// Load local points map from 3D points + color:
			cur_obs->unprojectInto(*gl_points);

			// Estimate our current camera pose from feature2feature matching:
			// --------------------------------------------------------------------
			if (icp_out && icp_res.nIterations > 0)
			{
				const CPose3D relativePose = icp_out->getMeanVal();
				lastIcpRelPose = relativePose;

				ui_data.strStatuses_mtx.lock();
				ui_data.strStatuses[0] = mrpt::format(
					"ICP: %d iters, goodness: %.02f%%",
					int(icp_res.nIterations), icp_res.goodness * 100.0f),

				ui_data.strStatuses[1] =
					std::string("rel.pose:") + relativePose.asString();
				ui_data.strStatuses[2] =
					string(icp_res.goodness < 0.3 ? "LOST! Press restart" : "");
				ui_data.strStatuses_mtx.unlock();

				if (icp_res.goodness > 0.65)
				{
					// Seems a good match:
					if ((relativePose.norm() > KEYFRAMES_MIN_DISTANCE ||
						 std::abs(relativePose.yaw()) > KEYFRAMES_MIN_ANG ||
						 std::abs(relativePose.pitch()) > KEYFRAMES_MIN_ANG ||
						 std::abs(relativePose.roll()) > KEYFRAMES_MIN_ANG))
					{
						// Accept this as a new key-frame pose ------------
						// Append new global pose of this key-frame:

						const CPose3D new_keyframe_global =
							CPose3D(*camera_key_frames_path.rbegin()) +
							relativePose;

						camera_key_frames_path.push_back(
							new_keyframe_global.asTPose());

						gl_keyframes_must_refresh = true;
						// It's (0,0,0) since the last
						// key-frame is the current pose!
						currentCamPose_wrt_last = CPose3D();

						cout << "Adding new key-frame: pose="
							 << new_keyframe_global << endl;

						// Update global map: append another map at a given
						// position:
						globalPtsMap.insertAnotherMap(
							cur_points.get(), new_keyframe_global);

						win.background_scene_mtx.lock();
						gl_points_map->loadFromPointsMap(&globalPtsMap);
						win.background_scene_mtx.unlock();

						prev_points = std::move(cur_points);  // new KF
					}
					else
					{
						currentCamPose_wrt_last = relativePose;
						// cout << "cur pose: " << currentCamPose_wrt_last
						// << endl;
					}
				}
			}

			if (camera_key_frames_path.empty())
			{
				// First iteration:
				camera_key_frames_path.clear();
				camera_key_frames_path.emplace_back(0, 0, 0, 0, 0, 0);
				gl_keyframes_must_refresh = true;

				// Update global map:
				globalPtsMap.clear();
				globalPtsMap.insertObservation(*cur_obs);

				win.background_scene_mtx.lock();
				gl_points_map->loadFromPointsMap(&globalPtsMap);
				win.background_scene_mtx.unlock();
			}

			// Update visualization ---------------------------------------

			// Show 3D points & current visible feats, at the current camera 3D
			// pose "currentCamPose_wrt_last"
			// ---------------------------------------------------------------------
			{
				const CPose3D curGlobalPose =
					CPose3D(*camera_key_frames_path.rbegin()) +
					currentCamPose_wrt_last;
				win.background_scene_mtx.lock();
				// All 3D points:
				cur_obs->unprojectInto(*gl_points);
				gl_points->setPose(curGlobalPose);

				gl_cur_cam_corner->setPose(curGlobalPose);

				win.background_scene_mtx.unlock();
			}

			if (gl_keyframes_must_refresh)
			{
				gl_keyframes_must_refresh = false;
				// cout << "Updating gl_keyframes with " <<
				// camera_key_frames_path.size() << " frames.\n";

				win.background_scene_mtx.lock();
				gl_keyframes->clear();
				for (const auto& i : camera_key_frames_path)
				{
					CSetOfObjects::Ptr obj =
						mrpt::opengl::stock_objects::CornerXYZSimple(0.3f, 3);
					obj->setPose(i);
					gl_keyframes->insert(obj);
				}
				win.background_scene_mtx.unlock();
			}

			ui_data.strStatuses_mtx.lock();
			ui_data.strStatuses[3] =
				format("Frames: %.02f Hz", std::atomic_load(&thrPar.Hz));
			ui_data.strStatuses_mtx.unlock();

			step_num++;

			// end update visualization:

			if (ui_data.hasToReset)
			{
				ui_data.hasToReset = false;

				cur_points.reset();
				prev_points.reset();
				lastIcpRelPose = CPose3D();
				camera_key_frames_path.clear();
				gl_keyframes_must_refresh = true;
				globalPtsMap.clear();
				win.background_scene_mtx.lock();
				gl_points_map->loadFromPointsMap(&globalPtsMap);
				win.background_scene_mtx.unlock();
			}

			// Show intensity image
			ui_data.viewInt_mtx->lock();
			ui_data.viewInt->setImageView(std::move(theImg));
			ui_data.viewInt_mtx->unlock();
		}
	};  // end lambdaUpdateThread

	std::thread thWorker = std::thread(lambdaUpdateThread);

	// Add UI controls:
	std::array<nanogui::TextBox*, 4> lbStatuses = {nullptr, nullptr, nullptr,
												   nullptr};
	mrpt::gui::MRPT2NanoguiGLCanvas* glCanvasRGBView = nullptr;
	nanogui::Window* subWin2 = nullptr;

	{
		auto subWin = new nanogui::Window(&win, "Control");
		subWin->setLayout(new nanogui::GroupLayout());
		subWin->setFixedWidth(400);

		subWin->add<nanogui::Label>("Visualization", "sans-bold");
		{
			auto cb = subWin->add<nanogui::CheckBox>("Show feature IDs");
			cb->setCallback(
				[&ui_data](bool checked) { ui_data.SHOW_FEAT_IDS = checked; });
			cb->setChecked(true);
		}

		{
			auto cb = subWin->add<nanogui::CheckBox>("Show keypoint responses");
			cb->setCallback(
				[&ui_data](bool checked) { ui_data.SHOW_RESPONSES = checked; });
			cb->setChecked(true);
		}

		for (unsigned int i = 0; i < lbStatuses.size(); i++)
			lbStatuses[i] = subWin->add<nanogui::TextBox>("");

		subWin->add<nanogui::Label>("RGB window size");
		{
			auto cmb = subWin->add<nanogui::ComboBox>(std::vector<std::string>(
				{"Hidden", "200px", "400px", "800px", "1000px"}));
			cmb->setSelectedIndex(2);
			cmb->setCallback([&](int sel) {
				subWin2->setVisible(sel != 0);

				switch (sel)
				{
					case 0:
						break;
					case 1:
						glCanvasRGBView->setFixedWidth(200);
						break;
					case 2:
						glCanvasRGBView->setFixedWidth(400);
						break;
					case 3:
						glCanvasRGBView->setFixedWidth(800);
						break;
					case 4:
						glCanvasRGBView->setFixedWidth(1000);
						break;
				};
				win.performLayout();
			});
		}

		{
			nanogui::TextBox* slVal =
				subWin->add<nanogui::TextBox>("Point cloud decimation: 8");
			nanogui::Slider* sl = subWin->add<nanogui::Slider>();

			sl->setRange({2, 4});
			sl->setValue(3);
			sl->setCallback([&ui_data, slVal](float v) {
				const unsigned int decim =
					mrpt::round(std::pow(2.0, mrpt::round(v)));
				ui_data.icpDecimation = decim;
				auto s = std::string("Point cloud decimation: ") +
						 std::to_string(ui_data.icpDecimation);
				slVal->setValue(s);
			});
		}

		subWin->add<nanogui::Label>("Actions", "sans-bold");

		{
			auto btn =
				subWin->add<nanogui::Button>("Reset", ENTYPO_ICON_BACK_IN_TIME);
			btn->setCallback([&]() { ui_data.hasToReset = true; });
		}

		{
			auto btn =
				subWin->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_LEFT);
			btn->setCallback([&]() { win.setVisible(false); });
		}
	}

	{
		subWin2 = new nanogui::Window(&win, "Visible channel");
		subWin2->setLayout(new nanogui::BoxLayout(
			nanogui::Orientation::Horizontal, nanogui::Alignment::Fill));

		glCanvasRGBView = subWin2->add<mrpt::gui::MRPT2NanoguiGLCanvas>();
		glCanvasRGBView->setFixedWidth(600);

		// Create the Opengl objects for the planar images each in a
		// separate viewport:

		glCanvasRGBView->scene = mrpt::opengl::COpenGLScene::Create();
		ui_data.viewInt = glCanvasRGBView->scene->getViewport();
		ui_data.viewInt_mtx = &glCanvasRGBView->scene_mtx;

		subWin2->setPosition({10, 500});
	}

	win.performLayout();

	// Set loop hook to update text messages:
	win.setLoopCallback([&lbStatuses, &ui_data]() {
		ui_data.strStatuses_mtx.lock();
		for (unsigned int i = 0; i < lbStatuses.size(); i++)
			lbStatuses[i]->setValue(ui_data.strStatuses[i]);
		ui_data.strStatuses_mtx.unlock();
	});

	// Update view and process events:
	win.drawAll();
	win.setVisible(true);
	nanogui::mainloop();

	nanogui::shutdown();

	cout << "Waiting for grabbing thread to exit...\n";
	thrPar.quit = true;
	thHandle.join();
	thWorker.join();
	cout << "Bye!\n";
}

#endif  // MRPT_HAS_NANOGUI

int main(int argc, char** argv)
{
	try
	{
#if MRPT_HAS_NANOGUI
		Test_3DCamICP();

		std::this_thread::sleep_for(50ms);
		return 0;
#else
		THROW_EXCEPTION("This program requires MRPT compiled with NANOGUI");
#endif
	}
	catch (const std::exception& e)
	{
		std::cout << "EXCEPCION: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}

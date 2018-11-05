/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/detectors.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/gui.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/math/ops_containers.h>
#include <iostream>
#include <mrpt/core/exceptions.h>
#include <mrpt/examples_config.h>
#include <mrpt/img/TColor.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::hwdrivers;
using namespace mrpt::detectors;
using namespace mrpt::config;
using namespace std;
using namespace mrpt::img;
using namespace mrpt::serialization;

string myDataDir(MRPT_EXAMPLES_BASE_DIRECTORY + string("detectors_face/"));
string myInitFile(
	MRPT_EXAMPLES_BASE_DIRECTORY +
	string("detectors_face/FACE_DETECTION_TEST.INI"));

CFaceDetection faceDetector;  // Face detector object

bool showEachDetectedFace;  // If using a 3D face detection (actually with
// swissrange) and we want stop every a face is
// detected for analize it.
bool batchMode;
vector<string> rawlogs;
vector<std::vector<uint32_t>> falsePositives;
vector<std::vector<uint32_t>> ignored;
string rawlogsDir;

#ifdef MRPT_OPENCV_SRC_DIR
static string OPENCV_SRC_DIR = MRPT_OPENCV_SRC_DIR;
#else
static string OPENCV_SRC_DIR = "./";
#endif

// ------------------------------------------------------
//				TestCamera3DFaceDetection
// ------------------------------------------------------
void TestCamera3DFaceDetection(CCameraSensor::Ptr cam)
{
	CDisplayWindow win("Live video");
	CDisplayWindow win2("FaceDetected");

	cout << "Close the window to exit." << endl;

	mrpt::gui::CDisplayWindow3D win3D("3D camera view", 800, 600);
	mrpt::gui::CDisplayWindow3D win3D2;

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(6.0);
	win3D.setCameraPointingToPoint(2.5, 0, 0);

	mrpt::opengl::COpenGLScene::Ptr& scene = win3D.get3DSceneAndLock();
	mrpt::opengl::COpenGLScene::Ptr scene2;

	mrpt::opengl::CPointCloudColoured::Ptr gl_points =
		mrpt::make_aligned_shared<mrpt::opengl::CPointCloudColoured>();
	gl_points->setPointSize(4.5);

	mrpt::opengl::CPointCloudColoured::Ptr gl_points2 =
		mrpt::make_aligned_shared<mrpt::opengl::CPointCloudColoured>();
	gl_points2->setPointSize(4.5);

	// Create the Opengl object for the point cloud:
	scene->insert(gl_points);
	scene->insert(mrpt::make_aligned_shared<mrpt::opengl::CGridPlaneXY>());
	scene->insert(mrpt::opengl::stock_objects::CornerXYZ());

	win3D.unlockAccess3DScene();

	if (showEachDetectedFace)
	{
		win3D2.setWindowTitle("3D Face detected");
		win3D2.resize(400, 300);

		win3D2.setCameraAzimuthDeg(140);
		win3D2.setCameraElevationDeg(20);
		win3D2.setCameraZoom(6.0);
		win3D2.setCameraPointingToPoint(2.5, 0, 0);

		scene2 = win3D2.get3DSceneAndLock();

		scene2->insert(gl_points2);
		scene2->insert(mrpt::make_aligned_shared<mrpt::opengl::CGridPlaneXY>());

		win3D2.unlockAccess3DScene();
	}

	double counter = 0;
	mrpt::system::CTicTac tictac;

	CVectorDouble fps;

	while (win.isOpen())
	{
		if (!counter) tictac.Tic();

		CObservation3DRangeScan::Ptr o;

		try
		{
			o = std::dynamic_pointer_cast<CObservation3DRangeScan>(
				cam->getNextFrame());
		}
		catch (CExceptionEOF&)
		{
			break;
		}
		ASSERT_(o);

		vector_detectable_object detected;

		// CObservation3DRangeScan::Ptr o =
		// std::dynamic_pointer_cast<CObservation3DRangeScan>(obs);

		faceDetector.detectObjects(o, detected);
		// static int x = 0;

		if (detected.size() > 0)
		{
			for (unsigned int i = 0; i < detected.size(); i++)
			{
				ASSERT_(IS_CLASS(detected[i], CDetectable3D));
				CDetectable3D::Ptr obj =
					std::dynamic_pointer_cast<CDetectable3D>(detected[i]);

				if (showEachDetectedFace)
				{
					CObservation3DRangeScan face;
					o->getZoneAsObs(
						face, obj->m_y, obj->m_y + obj->m_height, obj->m_x,
						obj->m_x + obj->m_width);
					win2.showImage(face.intensityImage);

					if (o->hasPoints3D)
					{
						win3D2.get3DSceneAndLock();

						CColouredPointsMap pntsMap;

						if (!o->hasConfidenceImage)
						{
							pntsMap.colorScheme.scheme =
								CColouredPointsMap::cmFromIntensityImage;
							pntsMap.loadFromRangeScan(face);
						}
						else
						{
							vector<float> xs, ys, zs;
							unsigned int i = 0;
							for (unsigned int j = 0;
								 j < face.confidenceImage.getHeight(); j++)
								for (unsigned int k = 0;
									 k < face.confidenceImage.getWidth();
									 k++, i++)
								{
									unsigned char c =
										*(face.confidenceImage.get_unsafe(
											k, j, 0));
									if (c > faceDetector.m_options
												.confidenceThreshold)
									{
										xs.push_back(face.points3D_x[i]);
										ys.push_back(face.points3D_y[i]);
										zs.push_back(face.points3D_z[i]);
									}
								}

							pntsMap.setAllPoints(xs, ys, zs);
						}

						gl_points2->loadFromPointsMap(&pntsMap);

						win3D2.unlockAccess3DScene();
						win3D2.repaint();
					}
				}

				o->intensityImage.rectangle(
					obj->m_x, obj->m_y, obj->m_x + obj->m_width,
					obj->m_y + obj->m_height, TColor(255, 0, 0));

				// x++;
				// if (( showEachDetectedFace ) && ( x > 430 ) )
				// system::pause();
			}
		}

		win.showImage(o->intensityImage);

		/*if (( showEachDetectedFace ) && ( detected.size() ))
				system::pause();*/

		win3D.get3DSceneAndLock();
		CColouredPointsMap pntsMap;
		pntsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
		pntsMap.loadFromRangeScan(*(o.get()));

		gl_points->loadFromPointsMap(&pntsMap);
		win3D.unlockAccess3DScene();
		win3D.repaint();

		if (++counter == 10)
		{
			double t = tictac.Tac();
			cout << "Frame Rate: " << counter / t << " fps" << endl;
			fps.push_back(counter / t);
			counter = 0;
		}
		std::this_thread::sleep_for(2ms);
	}

	cout << "Fps mean: " << fps.sumAll() / fps.size() << endl;

	faceDetector.experimental_showMeasurements();

	cout << "Closing..." << endl;
}

// ------------------------------------------------------
//				TestCameraFaceDetection
// ------------------------------------------------------
void TestCameraFaceDetection()
{
	CCameraSensor::Ptr cam = prepareVideoSourceFromUserSelection();

	if (!cam)
	{
		cerr << "The user didn't pick any camera. Exiting." << endl;
		return;
	}

	mrpt::obs::CObservation::Ptr obs = cam->getNextFrame();
	ASSERT_(obs);

	if (IS_CLASS(obs, CObservation3DRangeScan))
	{
		TestCamera3DFaceDetection(cam);
		return;
	}

	CDisplayWindow win("Live video");

	cout << "Close the window to exit." << endl;

	double counter = 0;
	mrpt::system::CTicTac tictac;

	while (win.isOpen())
	{
		if (!counter) tictac.Tic();

		mrpt::obs::CObservation::Ptr obs;
		try
		{
			obs = cam->getNextFrame();
		}
		catch (CExceptionEOF&)  // Check if eof, f.i. for RawLog files
		{
			break;
		}
		ASSERT_(obs);

		if (IS_CLASS(obs, CObservationImage))
		{
			vector_detectable_object detected;
			faceDetector.detectObjects(obs, detected);

			CObservationImage::Ptr o =
				std::dynamic_pointer_cast<CObservationImage>(obs);
			for (unsigned int i = 0; i < detected.size(); i++)
			{
				ASSERT_(IS_CLASS(detected[i], CDetectable2D));
				CDetectable2D::Ptr obj =
					std::dynamic_pointer_cast<CDetectable2D>(detected[i]);
				o->image.rectangle(
					obj->m_x, obj->m_y, obj->m_x + obj->m_width,
					obj->m_y + obj->m_height, TColor(255, 0, 0));
			}

			win.showImage(o->image);
		}
		else if (IS_CLASS(obs, CObservationStereoImages))
		{
			vector_detectable_object detected;
			faceDetector.detectObjects(obs, detected);

			CObservationStereoImages::Ptr o =
				std::dynamic_pointer_cast<CObservationStereoImages>(obs);

			for (unsigned int i = 0; i < detected.size(); i++)
			{
				ASSERT_(IS_CLASS(detected[i], CDetectable2D));
				CDetectable2D::Ptr obj =
					std::dynamic_pointer_cast<CDetectable2D>(detected[i]);
				o->imageRight.rectangle(
					obj->m_x, obj->m_y, obj->m_x + obj->m_width,
					obj->m_y + obj->m_height, TColor(255, 0, 0));
			}

			win.showImage(o->imageRight);
		}

		if (++counter == 10)
		{
			double t = tictac.Tac();
			cout << "Frame Rate: " << counter / t << " fps" << endl;
			counter = 0;
		}
		std::this_thread::sleep_for(2ms);
	}

	cout << "Closing..." << endl;
}

// ------------------------------------------------------
// 				 TestImagesFaceDetection
// ------------------------------------------------------
void TestImagesFaceDetection(int argc, char* argv[])
{
	CImage img;
	CDisplayWindow win("Result");
	mrpt::system::CTicTac tictac;

	// For each aditional argument, tty to load an image and detect faces
	for (int i = 1; i < argc; i++)
	{
		string fileName(argv[i]);

		if (!img.loadFromFile(myDataDir + fileName))
		{
			cerr << "Cannot load " << myDataDir + fileName << endl;
			continue;
		}

		vector_detectable_object detected;

		tictac.Tic();

		faceDetector.detectObjects(&img, detected);

		cout << "Detection time: " << tictac.Tac() << " s" << endl;

		for (unsigned int i = 0; i < detected.size(); i++)
		{
			ASSERT_(IS_CLASS(detected[i], CDetectable2D));
			CDetectable2D::Ptr obj =
				std::dynamic_pointer_cast<CDetectable2D>(detected[i]);
			img.rectangle(
				obj->m_x, obj->m_y, obj->m_x + obj->m_width,
				obj->m_y + obj->m_height, TColor(255, 0, 0));
		}

		win.showImage(img);

		mrpt::system::pause();
	}
}

void BatchMode()
{
	vector<double> frame_rates;

	for (size_t i = 0; i < rawlogs.size(); i++)
	{
		CRawlog rawlog;

		rawlog.loadFromRawLogFile(string(rawlogsDir + rawlogs[i] + ".rawlog"));

		cout << "Processing Rawlog [" << i + 1 << "/" << rawlogs.size()
			 << "]: " << rawlogs[i] << endl;

		const unsigned int size = rawlog.size();

		mrpt::system::CTicTac tictac;
		size_t counter = 0;

		// PROCESS RAWLOG
		for (unsigned int j = 1; j < size; j++)
		{
			if (!counter) tictac.Tic();

			CObservation3DRangeScan::Ptr o =
				std::dynamic_pointer_cast<CObservation3DRangeScan>(
					rawlog.getAsObservation(j));

			ASSERT_(o);

			vector_detectable_object detected;

			faceDetector.detectObjects(o, detected);

			if (++counter == 10)
			{
				double t = tictac.Tac();
				frame_rates.push_back(counter / t);
				counter = 0;
			}

			std::this_thread::sleep_for(2ms);
		}

		unsigned int falsePositivesDeleted, realFacesDeleted;
		faceDetector.debug_returnResults(
			falsePositives[i], ignored[i], falsePositivesDeleted,
			realFacesDeleted);
		cout << "False positives deleted: " << falsePositivesDeleted << endl;
		cout << "Real faces delted: " << realFacesDeleted << endl << endl;
	}

	cout << "Mean frame rate: " << sum(frame_rates) / frame_rates.size();

	// faceDetector.experimental_showMeasurements();

	system::pause();
}

void mySplit(const string& str)
{
	char *cstr, *p;

	cstr = new char[str.size() + 1];
	strcpy(cstr, str.c_str());

	// cstr now contains a c-string copy of str

	p = strtok(cstr, " ");
	while (p != nullptr)
	{
		string part(p);
		rawlogs.push_back(part);
		p = strtok(nullptr, " ");
	}

	delete[] cstr;
}

// ------------------------------------------------------
//					TestPrepareDetector
// ------------------------------------------------------
void TestPrepareDetector()
{
	CConfigFile cfg(myInitFile);

	int classifierType = cfg.read_int("Example", "classifierType", 0);

	if (classifierType == 0)  // Haar
		cfg.write(
			"CascadeClassifier", "cascadeFileName",
			OPENCV_SRC_DIR +
				"/data/haarcascades/haarcascade_frontalface_alt2.xml");
	else if (classifierType == 1)  // LBP
		cfg.write(
			"CascadeClassifier", "cascadeFileName",
			OPENCV_SRC_DIR + "/data/lbpcascades/lbpcascade_frontalface.xml");
	else
		throw std::runtime_error("Incorrect cascade classifier type.");

	showEachDetectedFace =
		cfg.read_bool("Example", "showEachDetectedFace", false);
	batchMode = cfg.read_bool("Example", "batchMode", false);

	if (batchMode)
	{
		string str = cfg.read_string("Example", "rawlogs", "noRawlogs");
		mySplit(str);

		size_t numRawlogs = rawlogs.size();
		falsePositives.resize(numRawlogs);
		ignored.resize(numRawlogs);

		for (size_t i = 0; i < numRawlogs; i++)
		{
			cfg.read_vector(
				rawlogs[i], "falsePositives", std::vector<uint32_t>(),
				falsePositives[i]);
			cfg.read_vector(
				rawlogs[i], "ignored", std::vector<uint32_t>(), ignored[i]);
		}

		rawlogsDir = cfg.read_string("Example", "rawlogsDir", "");
	}

	faceDetector.init(cfg);
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char* argv[])
{
	try
	{
		registerClass(CLASS_ID(CDetectableObject));
		registerClass(CLASS_ID(CDetectable2D));
		registerClass(CLASS_ID(CDetectable3D));

		TestPrepareDetector();

		if (batchMode)
			BatchMode();
		else
		{
			if (argc > 1)
				TestImagesFaceDetection(argc, argv);
			else
				TestCameraFaceDetection();
		}

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}

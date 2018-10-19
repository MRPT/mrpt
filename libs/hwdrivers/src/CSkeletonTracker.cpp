/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers

#include <mrpt/hwdrivers/CSkeletonTracker.h>

// opengl includes
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/CGridPlaneXZ.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPoint3D.h>

IMPLEMENTS_GENERIC_SENSOR(CSkeletonTracker, mrpt::hwdrivers)

using namespace mrpt::hwdrivers;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::img;
using namespace std;

#define skl_states (static_cast<nite::SkeletonState*>(m_skeletons_ptr))
#define user_tracker (static_cast<nite::UserTracker*>(m_userTracker_ptr))
#define MAX_USERS 10
#define FILL_JOINT_DATA(_J1, _J2)                                  \
	obs->_J1.x = user.getSkeleton().getJoint(_J2).getPosition().x; \
	obs->_J1.y = user.getSkeleton().getJoint(_J2).getPosition().y; \
	obs->_J1.z = user.getSkeleton().getJoint(_J2).getPosition().z; \
	obs->_J1.conf = user.getSkeleton().getJoint(_J2).getPositionConfidence();

#if MRPT_HAS_NITE2
#include "NiTE.h"
#pragma comment(lib, "NiTE2.lib")
#endif

string jointNames[] = {
	"head",		   "neck",		 "torso",	 "left_shoulder", "left_elbow",
	"left_hand",   "left_hip",   "left_knee", "left_foot",	 "right_shoulder",
	"right_elbow", "right_hand", "right_hip", "right_knee",	"right_foot"};

/*-------------------------------------------------------------
					CSkeletonTracker
-------------------------------------------------------------*/
CSkeletonTracker::CSkeletonTracker() : m_timeStartTT(), m_sensorPose()

{
	m_sensorLabel = "skeletonTracker";

#if MRPT_HAS_OPENNI2 && MRPT_HAS_NITE2
	m_skeletons_ptr = new nite::SkeletonState[MAX_USERS];
	m_userTracker_ptr = new nite::UserTracker;
	for (int i = 0; i < MAX_USERS; ++i) skl_states[i] = nite::SKELETON_NONE;

	m_linesToPlot.resize(NUM_LINES);
	m_joint_theta.resize(NUM_JOINTS);
	for (int i = 1; i < NUM_JOINTS; ++i)
		m_joint_theta[i] = (i - 1) * (M_2PI / (NUM_JOINTS - 1));
#else
	THROW_EXCEPTION(
		"MRPT has been compiled with 'BUILD_OPENNI2'=OFF or 'BUILD_NITE2'=OFF, "
		"so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					~CSkeletonTracker
-------------------------------------------------------------*/
CSkeletonTracker::~CSkeletonTracker()
{
#if MRPT_HAS_OPENNI2 && MRPT_HAS_NITE2
	nite::NiTE::shutdown();  // close tracker
	delete[] skl_states;
	m_skeletons_ptr = nullptr;
	delete user_tracker;
	m_userTracker_ptr = nullptr;
#endif
	if (m_win) m_win.reset();
}

/*-------------------------------------------------------------
					processPreviewNone
-------------------------------------------------------------*/
void CSkeletonTracker::processPreviewNone()
{
	using namespace mrpt::opengl;

	// show skeleton data
	if (m_showPreview)
	{
		if (!m_win)
		{
			string caption = string("Preview of ") + m_sensorLabel;
			m_win = mrpt::make_aligned_shared<mrpt::gui::CDisplayWindow3D>(
				caption, 800, 600);

			COpenGLScene::Ptr& scene = m_win->get3DSceneAndLock();
			scene->insert(
				mrpt::make_aligned_shared<CGridPlaneXZ>(-3, 3, 0, 5, -1.5));

			// set camera parameters
			m_win->setCameraElevationDeg(-90);
			m_win->setCameraAzimuthDeg(90);
			m_win->setCameraZoom(4);
			m_win->setCameraPointingToPoint(0, 0, 0);

			// insert initial body
			CSetOfObjects::Ptr body =
				mrpt::make_aligned_shared<CSetOfObjects>();
			body->setName("body");
			for (const auto& jointName : jointNames)
			{
				CSphere::Ptr sph = mrpt::make_aligned_shared<CSphere>(0.03f);
				sph->setColor(0, 1, 0);
				sph->setName(jointName);
				body->insert(sph);
			}

			// insert initial lines
			CSetOfLines::Ptr lines = mrpt::make_aligned_shared<CSetOfLines>();
			lines->setName("lines");
			lines->setColor(0, 0, 1);
			body->insert(lines);

			scene->insert(body);
			m_win->unlockAccess3DScene();
		}

		if (m_win && m_win->isOpen())
		{
			COpenGLScene::Ptr& scene = m_win->get3DSceneAndLock();
			{
				m_win->addTextMessage(
					0.35, 0.9, "Please, adopt this position", TColorf(1, 1, 1),
					"mono", 10, mrpt::opengl::FILL, 0);

				// insert translucid dummy and help text (it will go away when
				// measurements are taken)
				if (!scene->getByName("dummy"))
				{
					const double SCALE = 0.8;
					const double BODY_RADIUS = 0.22 * SCALE;
					const double BODY_LENGTH = 0.8 * SCALE;
					const double ARM_RADIUS = 0.05 * SCALE;
					const double ARM_LENGTH = 0.4 * SCALE;
					const double LEG_RADIUS = 0.1 * SCALE;
					const double LEG_LENGTH = 0.8 * SCALE;
					const double HEAD_RADIUS = 0.15 * SCALE;
					const double ALPHA_CH = 0.8;

					CSetOfObjects::Ptr dummy =
						mrpt::make_aligned_shared<CSetOfObjects>();
					dummy->setName("dummy");
					dummy->setPose(math::TPose3D(0, 0, 0, 0, 0, DEG2RAD(-90)));
					{
						// head
						CSphere::Ptr part =
							mrpt::make_aligned_shared<CSphere>(HEAD_RADIUS);
						part->setColor(1, 1, 1, ALPHA_CH);
						part->setPose(math::TPose3D(
							0, 0, 0.5 * BODY_LENGTH + HEAD_RADIUS, 0, 0, 0));
						dummy->insert(part);
					}
					{
						// body
						CCylinder::Ptr part =
							mrpt::make_aligned_shared<CCylinder>(
								BODY_RADIUS, BODY_RADIUS, BODY_LENGTH);
						part->setColor(1, 1, 1, ALPHA_CH);
						part->setPose(
							math::TPose3D(0, 0, -BODY_LENGTH / 2, 0, 0, 0));
						dummy->insert(part);
					}
					{
						// left arm 0
						CCylinder::Ptr part =
							mrpt::make_aligned_shared<CCylinder>(
								ARM_RADIUS, ARM_RADIUS, ARM_LENGTH);
						part->setColor(1, 1, 1, ALPHA_CH);
						part->setPose(math::TPose3D(
							-BODY_RADIUS, 0, 0.5 * BODY_LENGTH - ARM_RADIUS, 0,
							DEG2RAD(-90), 0));
						dummy->insert(part);
					}
					{
						// left arm 1
						CCylinder::Ptr part =
							mrpt::make_aligned_shared<CCylinder>(
								ARM_RADIUS, ARM_RADIUS, ARM_LENGTH);
						part->setColor(1, 1, 1, ALPHA_CH);
						part->setPose(math::TPose3D(
							-BODY_RADIUS - ARM_LENGTH + ARM_RADIUS, 0,
							0.5 * BODY_LENGTH - ARM_RADIUS, 0, 0, 0));
						dummy->insert(part);
					}
					{
						// right arm 0
						CCylinder::Ptr part =
							mrpt::make_aligned_shared<CCylinder>(
								ARM_RADIUS, ARM_RADIUS, ARM_LENGTH);
						part->setColor(1, 1, 1, ALPHA_CH);
						part->setPose(math::TPose3D(
							BODY_RADIUS, 0, 0.5 * BODY_LENGTH - ARM_RADIUS, 0,
							DEG2RAD(90), 0));
						dummy->insert(part);
					}
					{
						// right arm 1
						CCylinder::Ptr part =
							mrpt::make_aligned_shared<CCylinder>(
								ARM_RADIUS, ARM_RADIUS, ARM_LENGTH);
						part->setColor(1, 1, 1, ALPHA_CH);
						part->setPose(math::TPose3D(
							BODY_RADIUS + ARM_LENGTH - ARM_RADIUS, 0,
							0.5 * BODY_LENGTH - ARM_RADIUS, 0, 0, 0));
						dummy->insert(part);
					}
					{
						// left leg
						CCylinder::Ptr part =
							mrpt::make_aligned_shared<CCylinder>(
								LEG_RADIUS, LEG_RADIUS, LEG_LENGTH);
						part->setColor(1, 1, 1, ALPHA_CH);
						part->setPose(math::TPose3D(
							-BODY_RADIUS + LEG_RADIUS, 0,
							-(0.5 * BODY_LENGTH + LEG_LENGTH), 0, 0, 0));
						dummy->insert(part);
					}
					{
						// right leg
						CCylinder::Ptr part =
							mrpt::make_aligned_shared<CCylinder>(
								LEG_RADIUS, LEG_RADIUS, LEG_LENGTH);
						part->setColor(1, 1, 1, ALPHA_CH);
						part->setPose(math::TPose3D(
							BODY_RADIUS - LEG_RADIUS, 0,
							-(0.5 * BODY_LENGTH + LEG_LENGTH), 0, 0, 0));
						dummy->insert(part);
					}
					scene->insert(dummy);
				}  // end-if
				else
				{
					CSetOfObjects::Ptr dummy =
						std::dynamic_pointer_cast<CSetOfObjects>(
							scene->getByName("dummy"));
					dummy->setVisibility(true);
				}

				// update joints positions
				CSetOfObjects::Ptr body =
					std::dynamic_pointer_cast<CSetOfObjects>(
						scene->getByName("body"));
				ASSERT_(body);

				for (int i = 0; i < NUM_JOINTS; ++i)
				{
					CSphere::Ptr s = std::dynamic_pointer_cast<CSphere>(
						body->getByName(jointNames[i]));
					CPoint3D sphPos;
					if (i == 0)
						sphPos = CPoint3D(0, 0, 0);
					else
					{
						m_joint_theta[i] += M_2PI / (10 * (NUM_JOINTS - 1));
						sphPos.x(0.5 * cos(m_joint_theta[i]));
						sphPos.y(0.5 * sin(m_joint_theta[i]));
						sphPos.z(0.0);
					}
					s->setPose(sphPos);
					s->setColor(1, 0, 0);
					s->setRadius(i == 0 ? 0.07 : 0.03);
				}  // end-for
			}  // end-get3DSceneAndLock
			m_win->unlockAccess3DScene();
			m_win->forceRepaint();
		}  // end if
	}  // end if
}  // end-processPreviewNone

/*-------------------------------------------------------------
					processPreview
-------------------------------------------------------------*/
void CSkeletonTracker::processPreview(
	const mrpt::obs::CObservationSkeleton::Ptr& obs)
{
	using namespace mrpt::opengl;

	// show skeleton data
	if (m_showPreview)
	{
		if (!m_win)
		{
			string caption = string("Preview of ") + m_sensorLabel;
			m_win = mrpt::make_aligned_shared<mrpt::gui::CDisplayWindow3D>(
				caption, 800, 600);

			COpenGLScene::Ptr& scene = m_win->get3DSceneAndLock();
			scene->insert(
				mrpt::make_aligned_shared<CGridPlaneXZ>(-3, 3, 0, 5, -1.5));

			// set camera parameters
			m_win->setCameraElevationDeg(-90);
			m_win->setCameraAzimuthDeg(90);
			m_win->setCameraZoom(4);
			m_win->setCameraPointingToPoint(0, 0, 0);

			// insert initial body
			CSetOfObjects::Ptr body =
				mrpt::make_aligned_shared<CSetOfObjects>();
			body->setName("body");
			for (const auto& jointName : jointNames)
			{
				CSphere::Ptr sph = mrpt::make_aligned_shared<CSphere>(0.03f);
				sph->setColor(0, 1, 0);
				sph->setName(jointName);
				body->insert(sph);
			}

			// insert initial lines
			CSetOfLines::Ptr lines = mrpt::make_aligned_shared<CSetOfLines>();
			lines->setName("lines");
			lines->setColor(0, 0, 1);
			body->insert(lines);

			scene->insert(body);
			m_win->unlockAccess3DScene();
		}

		if (m_win && m_win->isOpen())
		{
			COpenGLScene::Ptr& scene = m_win->get3DSceneAndLock();

			// remove help text and dummy
			m_win->clearTextMessages();
			CSetOfObjects::Ptr dummy = std::dynamic_pointer_cast<CSetOfObjects>(
				scene->getByName("dummy"));
			if (dummy) dummy->setVisibility(false);

			{
				// update joints positions
				CSetOfObjects::Ptr body =
					std::dynamic_pointer_cast<CSetOfObjects>(
						scene->getByName("body"));
				ASSERT_(body);

				for (int i = 0; i < NUM_JOINTS; ++i)
				{
					CObservationSkeleton::TSkeletonJoint j;

					switch (i)
					{
						case 0:
							j = obs->head;
							break;
						case 1:
							j = obs->neck;
							break;
						case 2:
							j = obs->torso;
							break;

						case 3:
							j = obs->left_shoulder;
							break;
						case 4:
							j = obs->left_elbow;
							break;
						case 5:
							j = obs->left_hand;
							break;
						case 6:
							j = obs->left_hip;
							break;
						case 7:
							j = obs->left_knee;
							break;
						case 8:
							j = obs->left_foot;
							break;

						case 9:
							j = obs->right_shoulder;
							break;
						case 10:
							j = obs->right_elbow;
							break;
						case 11:
							j = obs->right_hand;
							break;
						case 12:
							j = obs->right_hip;
							break;
						case 13:
							j = obs->right_knee;
							break;
						case 14:
							j = obs->right_foot;
							break;
					}  // end-switch

					CSphere::Ptr s = std::dynamic_pointer_cast<CSphere>(
						body->getByName(jointNames[i]));
					s->setPose(mrpt::math::TPose3D(
						j.x * 1e-3, j.y * 1e-3, j.z * 1e-3, 0, 0, 0));
					s->setColor(
						std::min(1.0, 2 * (1 - j.conf)),
						std::min(1.0, 2 * j.conf), 0);
					s->setRadius(i == 0 ? 0.07 : 0.03);
				}  // end-for

				// update lines joining joints
				CSetOfLines::Ptr lines = std::dynamic_pointer_cast<CSetOfLines>(
					body->getByName("lines"));
				ASSERT_(lines);

				lines->clear();
				for (int i = 0; i < NUM_LINES; ++i)
				{
					pair<JOINT, JOINT> pair = m_linesToPlot[i];
					CSphere::Ptr s0 = dynamic_pointer_cast<CSphere>(
						body->getByName(jointNames[pair.first]));
					CSphere::Ptr s1 = dynamic_pointer_cast<CSphere>(
						body->getByName(jointNames[pair.second]));
					ASSERT_(s0 && s1);

					lines->appendLine(
						s0->getPoseX(), s0->getPoseY(), s0->getPoseZ(),
						s1->getPoseX(), s1->getPoseY(), s1->getPoseZ());
				}
			}  // end-get3DSceneAndLock
			m_win->unlockAccess3DScene();
			m_win->forceRepaint();
		}  // end if
	}  // end if
}

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CSkeletonTracker::doProcess()
{
#if MRPT_HAS_OPENNI2 && MRPT_HAS_NITE2
	if (m_state == ssError)
	{
		std::this_thread::sleep_for(200ms);
		initialize();
	}

	if (m_state == ssError) return;

	nite::UserTrackerFrameRef userTrackerFrame;
	nite::Status niteRc = user_tracker->readFrame(&userTrackerFrame);

	if (niteRc != nite::STATUS_OK)
	{
		printf("	[Skeleton tracker] Get next frame failed\n");
		return;
	}

	int n_data_ok = 0;
	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
	m_nUsers = users.getSize();
	for (int i = 0; i < m_nUsers; ++i)
	{
		const nite::UserData& user = users[i];

		// update user state
		skl_states[user.getId()] = user.getSkeleton().getState();

		if (user.isNew())
		{
			user_tracker->startSkeletonTracking(user.getId());
			cout << "	[Skeleton tracker] New user found" << endl;
		}
		else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
		{
			cout << "	[Skeleton tracker] User " << user.getId() << " tracked"
				 << endl;
			CObservationSkeleton::Ptr obs =
				mrpt::make_aligned_shared<CObservationSkeleton>();

			// timestamp
			const uint64_t nowUI = userTrackerFrame.getTimestamp();

			uint64_t AtUI = 0;
			if (m_timeStartUI == 0)
			{
				m_timeStartUI = nowUI;
				m_timeStartTT = mrpt::system::now();
			}
			else
				AtUI = nowUI - m_timeStartUI;

			/* Board time is usec */
			const auto AtDO = std::chrono::microseconds(AtUI);
			mrpt::system::TTimeStamp ts = m_timeStartTT + AtDO;
			obs->timestamp = ts;

			// fill joint data
			FILL_JOINT_DATA(head, nite::JOINT_HEAD)
			FILL_JOINT_DATA(neck, nite::JOINT_NECK)
			FILL_JOINT_DATA(torso, nite::JOINT_TORSO)

			FILL_JOINT_DATA(left_shoulder, nite::JOINT_LEFT_SHOULDER)
			FILL_JOINT_DATA(left_elbow, nite::JOINT_LEFT_ELBOW)
			FILL_JOINT_DATA(left_hand, nite::JOINT_LEFT_HAND)
			FILL_JOINT_DATA(left_hip, nite::JOINT_LEFT_HIP)
			FILL_JOINT_DATA(left_knee, nite::JOINT_LEFT_KNEE)
			FILL_JOINT_DATA(left_foot, nite::JOINT_LEFT_FOOT)

			FILL_JOINT_DATA(right_shoulder, nite::JOINT_RIGHT_SHOULDER)
			FILL_JOINT_DATA(right_elbow, nite::JOINT_RIGHT_ELBOW)
			FILL_JOINT_DATA(right_hand, nite::JOINT_RIGHT_HAND)
			FILL_JOINT_DATA(right_hip, nite::JOINT_RIGHT_HIP)
			FILL_JOINT_DATA(right_knee, nite::JOINT_RIGHT_KNEE)
			FILL_JOINT_DATA(right_foot, nite::JOINT_RIGHT_FOOT)

			// sensor label:
			obs->sensorLabel =
				m_sensorLabel + "_" + std::to_string(user.getId());

			appendObservation(obs);
			processPreview(obs);

			m_toutCounter = 0;
			n_data_ok++;
		}  // end-else-if
	}  // end-for

	if (n_data_ok == 0)  // none of the sensors yielded data
		m_toutCounter++;

	if (m_toutCounter > 0)
	{
		processPreviewNone();
		if ((m_toutCounter % 50) == 0)
			cout << "	[Skeleton tracker] Looking for user..." << endl;
	}

	if (m_toutCounter > 2000)
	{
		m_toutCounter = 0;
		m_state = ssError;

		cout << "	[Skeleton tracker] No user found after 2000 attempts ..."
			 << endl;
		nite::NiTE::shutdown();  // close tracker
	}
#else
	THROW_EXCEPTION(
		"MRPT has been compiled with 'BUILD_OPENNI2'=OFF or "
		"'MRPT_HAS_NITE2'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					initialize
-------------------------------------------------------------*/
void CSkeletonTracker::initialize()
{
#if MRPT_HAS_OPENNI2 && MRPT_HAS_NITE2

	// initialize tracker
	nite::NiTE::initialize();
	nite::Status niteRc = user_tracker->create();

	if (niteRc != nite::STATUS_OK)
	{
		printf("Couldn't create user tracker\n");
		m_state = ssError;
	}
	else
	{
		printf("Sucessfully created user tracker \n");
		printf(
			"Start moving around to get detected...\n(PSI pose may be required "
			"for skeleton calibration, depending on the configuration)\n");
		m_state = ssInitializing;
	}
	// initialize preview joints and lines
	if (m_showPreview)
	{
		m_linesToPlot[0] = make_pair(NECK, HEAD);
		m_linesToPlot[1] = make_pair(NECK, TORSO);
		m_linesToPlot[2] = make_pair(NECK, LEFT_SHOULDER);
		m_linesToPlot[3] = make_pair(NECK, RIGHT_SHOULDER);
		m_linesToPlot[4] = make_pair(LEFT_SHOULDER, LEFT_ELBOW);
		m_linesToPlot[5] = make_pair(LEFT_ELBOW, LEFT_HAND);
		m_linesToPlot[6] = make_pair(RIGHT_SHOULDER, RIGHT_ELBOW);
		m_linesToPlot[7] = make_pair(RIGHT_ELBOW, RIGHT_HAND);
		m_linesToPlot[8] = make_pair(TORSO, LEFT_HIP);
		m_linesToPlot[9] = make_pair(TORSO, RIGHT_HIP);
		m_linesToPlot[10] = make_pair(LEFT_HIP, LEFT_KNEE);
		m_linesToPlot[11] = make_pair(LEFT_KNEE, LEFT_FOOT);
		m_linesToPlot[12] = make_pair(RIGHT_HIP, RIGHT_KNEE);
		m_linesToPlot[13] = make_pair(RIGHT_KNEE, RIGHT_FOOT);
	}
#else
	THROW_EXCEPTION(
		"MRPT has been compiled with 'BUILD_OPENNI2'=OFF OR "
		"'MRPT_HAS_NITE2'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					loadConfig_sensorSpecific
-------------------------------------------------------------*/
void CSkeletonTracker::loadConfig_sensorSpecific(
	const mrpt::config::CConfigFileBase& configSource,
	const std::string& iniSection)
{
	m_sensorPose.setFromValues(
		configSource.read_float(iniSection, "pose_x", 0, false),
		configSource.read_float(iniSection, "pose_y", 0, false),
		configSource.read_float(iniSection, "pose_z", 0, false),
		DEG2RAD(configSource.read_float(iniSection, "pose_yaw", 0, false)),
		DEG2RAD(configSource.read_float(iniSection, "pose_pitch", 0, false)),
		DEG2RAD(configSource.read_float(iniSection, "pose_roll", 0, false)));

	m_showPreview =
		configSource.read_bool(iniSection, "showPreview", m_showPreview, false);

	// dump parameters to console
	cout << "---------------------------" << endl;
	cout << "Skeleton Tracker parameters: " << endl;
	cout << "---------------------------" << endl;
	cout << m_sensorPose << endl;
	cout << m_showPreview << endl;
	cout << "---------------------------" << endl << endl;
}

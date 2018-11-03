/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header

#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>

using namespace mrpt::nav;

IMPLEMENTS_SERIALIZABLE(CLogFileRecord, CSerializable, mrpt::nav)

/*---------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
CLogFileRecord::CLogFileRecord()
	: robotPoseLocalization(0, 0, 0),
	  robotPoseOdometry(0, 0, 0),
	  relPoseSense(0, 0, 0),
	  relPoseVelCmd(0, 0, 0),
	  WS_targets_relative(),
	  cur_vel(0, 0, 0),
	  cur_vel_local(0, 0, 0),

	  rel_cur_pose_wrt_last_vel_cmd_NOP(0, 0, 0),
	  rel_pose_PTG_origin_wrt_sense_NOP(0, 0, 0)
{
	infoPerPTG.clear();
	WS_Obstacles.clear();
}

uint8_t CLogFileRecord::serializeGetVersion() const { return 26; }
void CLogFileRecord::serializeTo(mrpt::serialization::CArchive& out) const
{
	uint32_t i, n;

	// Version 0 ---------
	n = infoPerPTG.size();
	out << n;
	for (i = 0; i < n; i++)
	{
		out << infoPerPTG[i].PTG_desc.c_str();

		uint32_t m = infoPerPTG[i].TP_Obstacles.size();
		out << m;
		if (m)
			out.WriteBuffer(
				(const void*)&(*infoPerPTG[i].TP_Obstacles.begin()),
				m * sizeof(infoPerPTG[i].TP_Obstacles[0]));

		out << infoPerPTG[i]
				   .TP_Targets;  // v8: CPoint2D -> TPoint2D. v26: vector
		out << infoPerPTG[i].TP_Robot;  // v17
		out << infoPerPTG[i].timeForTPObsTransformation
			<< infoPerPTG[i].timeForHolonomicMethod;  // made double in v12
		out << infoPerPTG[i].desiredDirection << infoPerPTG[i].desiredSpeed
			<< infoPerPTG[i].evaluation;  // made double in v12
		// removed in v23: out << evaluation_org << evaluation_priority; //
		// added in v21
		out << infoPerPTG[i].HLFR;

		// Version 9: Removed security distances. Added optional field with
		// PTG info.
		const bool there_is_ptg_data = infoPerPTG[i].ptg ? true : false;
		out << there_is_ptg_data;
		if (there_is_ptg_data) out << infoPerPTG[i].ptg;

		// Was: out << infoPerPTG[i].clearance.raw_clearances; // v19
		infoPerPTG[i].clearance.writeToStream(out);  // v25
	}
	out << nSelectedPTG << WS_Obstacles;
	out << WS_Obstacles_original;  // v20

	// Removed v24: out << robotOdometryPose;
	out << robotPoseLocalization << robotPoseOdometry;  // v24

	out << WS_targets_relative;  // v8, v26: vector
	// v16:
	out << ptg_index_NOP << ptg_last_k_NOP;
	out << rel_cur_pose_wrt_last_vel_cmd_NOP
		<< rel_pose_PTG_origin_wrt_sense_NOP;  // v24: CPose2D->TPose2D

	// Removed: v24. out << ptg_last_curRobotVelLocal; // v17
	ptg_last_navDynState.writeToStream(out);  // v24

	if (ptg_index_NOP < 0) out << cmd_vel /*v10*/ << cmd_vel_original;  // v15

	// Previous values: REMOVED IN VERSION #6
	n = robotShape_x.size();
	out << n;
	if (n)
	{
		out.WriteBuffer(
			(const void*)&(*robotShape_x.begin()), n * sizeof(robotShape_x[0]));
		out.WriteBuffer(
			(const void*)&(*robotShape_y.begin()), n * sizeof(robotShape_y[0]));
	}

	// Version 1 ---------
	out << cur_vel << cur_vel_local; /*v10*/
	// out << estimatedExecutionPeriod; // removed v13

	// Version 3 ----------
	for (i = 0; i < infoPerPTG.size(); i++)
	{
		out << infoPerPTG[i]
				   .evalFactors.base;  // v22: this is now a TParameters
	}

	out << nPTGs;  // v4
	// out << timestamp; // removed v13
	out << robotShape_radius;  // v11
	// out << cmd_vel_filterings; // added v12: Removed in v15

	out << values << timestamps;  // v13

	out << relPoseSense << relPoseVelCmd;  // v14, v24 changed CPose2D->TPose2D

	// v15: cmd_vel converted from std::vector<double> into CSerializable
	out << additional_debug_msgs;  // v18

	navDynState.writeToStream(out);  // v24
}

void CLogFileRecord::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15:
		case 16:
		case 17:
		case 18:
		case 19:
		case 20:
		case 21:
		case 22:
		case 23:
		case 24:
		case 25:
		case 26:
		{
			// Version 0 --------------
			uint32_t i, n;

			infoPerPTG.clear();

			in >> n;
			infoPerPTG.resize(n);
			for (i = 0; i < n; i++)
			{
				auto& ipp = infoPerPTG[i];
				in >> ipp.PTG_desc;

				int32_t m;
				in >> m;
				ipp.TP_Obstacles.resize(m);
				if (m)
					in.ReadBufferFixEndianness(&(*ipp.TP_Obstacles.begin()), m);

				ipp.TP_Targets.clear();
				if (version >= 8)
				{
					if (version >= 26)
					{
						in >> ipp.TP_Targets;
					}
					else
					{
						mrpt::math::TPoint2D trg;
						in >> trg;
						ipp.TP_Targets.push_back(trg);
					}
				}
				else
				{
					mrpt::poses::CPoint2D pos;
					in >> pos;
					ipp.TP_Targets.emplace_back(pos.x(), pos.y());
				}
				if (version >= 17)
					in >> ipp.TP_Robot;
				else
					ipp.TP_Robot = mrpt::math::TPoint2D(0, 0);

				if (version >= 12)
				{
					in >> ipp.timeForTPObsTransformation >>
						ipp.timeForHolonomicMethod;
					in >> ipp.desiredDirection >> ipp.desiredSpeed >>
						ipp.evaluation;
				}
				else
				{
					in.ReadAsAndCastTo<float, double>(
						ipp.timeForTPObsTransformation);
					in.ReadAsAndCastTo<float, double>(
						ipp.timeForHolonomicMethod);
					in.ReadAsAndCastTo<float, double>(ipp.desiredDirection);
					in.ReadAsAndCastTo<float, double>(ipp.desiredSpeed);
					in.ReadAsAndCastTo<float, double>(ipp.evaluation);
				}
				if (version >= 21 && version < 23)
				{
					double evaluation_org, evaluation_priority;
					in >> evaluation_org >> evaluation_priority;
				}

				in >> ipp.HLFR;

				if (version >= 9)  // Extra PTG info
				{
					ipp.ptg.reset();

					bool there_is_ptg_data;
					in >> there_is_ptg_data;
					if (there_is_ptg_data)
						ipp.ptg = std::dynamic_pointer_cast<
							CParameterizedTrajectoryGenerator>(in.ReadObject());
				}

				if (version >= 19)
				{
					if (version < 25)
					{
						std::vector<std::map<double, double>> raw_clearances;
						in >> raw_clearances;
						ipp.clearance.resize(
							raw_clearances.size(), raw_clearances.size());
						for (size_t k = 0; k < raw_clearances.size(); k++)
							ipp.clearance.get_path_clearance_decimated(k) =
								raw_clearances[k];
					}
					else
					{
						ipp.clearance.readFromStream(in);
					}
				}
				else
				{
					ipp.clearance.clear();
				}
			}

			in >> nSelectedPTG >> WS_Obstacles;
			if (version >= 20)
			{
				in >> WS_Obstacles_original;  // v20
			}
			else
			{
				WS_Obstacles_original = WS_Obstacles;
			}

			if (version < 24)
			{
				mrpt::poses::CPose2D robotOdometryPose;
				in >> robotOdometryPose;
				robotPoseOdometry = robotOdometryPose.asTPose();
				robotPoseLocalization = robotOdometryPose.asTPose();
			}
			else
			{
				in >> robotPoseLocalization >> robotPoseOdometry;  // v24
			}

			WS_targets_relative.clear();
			if (version >= 8)
			{
				if (version >= 26)
				{
					in >> WS_targets_relative;
				}
				else
				{
					mrpt::math::TPoint2D trg;
					in >> trg;
					WS_targets_relative.emplace_back(trg);
				}
			}
			else
			{
				mrpt::poses::CPoint2D pos;
				in >> pos;
				WS_targets_relative.emplace_back(
					mrpt::math::TPoint2D(pos.x(), pos.y()));
			}

			if (version >= 16)
			{
				in >> ptg_index_NOP >> ptg_last_k_NOP;
				if (version >= 24)
				{
					in >> rel_cur_pose_wrt_last_vel_cmd_NOP >>
						rel_pose_PTG_origin_wrt_sense_NOP;
				}
				else
				{
					mrpt::poses::CPose2D crel_cur_pose_wrt_last_vel_cmd_NOP,
						crel_pose_PTG_origin_wrt_sense_NOP;
					in >> crel_cur_pose_wrt_last_vel_cmd_NOP >>
						crel_pose_PTG_origin_wrt_sense_NOP;
					rel_cur_pose_wrt_last_vel_cmd_NOP =
						crel_cur_pose_wrt_last_vel_cmd_NOP.asTPose();
					rel_pose_PTG_origin_wrt_sense_NOP =
						crel_pose_PTG_origin_wrt_sense_NOP.asTPose();
				}
			}
			else
			{
				ptg_index_NOP = -1;
			}
			if (version >= 17 && version < 24)
			{
				in >> ptg_last_navDynState.curVelLocal;
			}
			if (version >= 24)
			{
				ptg_last_navDynState.readFromStream(in);
			}

			if (version >= 10)
			{
				if (version >= 15)
				{
					if (ptg_index_NOP < 0) in >> cmd_vel;
				}
				else
				{
					std::vector<double> vel;
					in >> vel;
					if (vel.size() == 2)
						cmd_vel = mrpt::kinematics::CVehicleVelCmd::Ptr(
							new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
					else
						cmd_vel = mrpt::kinematics::CVehicleVelCmd::Ptr(
							new mrpt::kinematics::CVehicleVelCmd_Holo);
					for (size_t k = 0; k < cmd_vel->getVelCmdLength(); k++)
						cmd_vel->setVelCmdElement(i, vel[k]);
				}
			}
			else
			{
				float v, w;
				in >> v >> w;
				cmd_vel = mrpt::kinematics::CVehicleVelCmd::Ptr(
					new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
				cmd_vel->setVelCmdElement(0, v);
				cmd_vel->setVelCmdElement(0, w);
			}

			if (version >= 15 && ptg_index_NOP < 0) in >> cmd_vel_original;

			if (version < 13)
			{
				float old_exec_time;
				in >> old_exec_time;
				values["executionTime"] = old_exec_time;
			}

			if (version < 6)
			{
				mrpt::math::CVectorFloat prevV, prevW, prevSelPTG;

				// Previous values: (Removed in version 6)
				in >> n;
				prevV.resize(n);
				if (n) in.ReadBufferFixEndianness(&(*prevV.begin()), n);

				in >> n;
				prevW.resize(n);
				if (n) in.ReadBufferFixEndianness(&(*prevW.begin()), n);

				in >> n;
				prevSelPTG.resize(n);
				if (n) in.ReadBufferFixEndianness(&(*prevSelPTG.begin()), n);
			}

			in >> n;
			robotShape_x.resize(n);
			robotShape_y.resize(n);
			if (n)
			{
				in.ReadBufferFixEndianness(&(*robotShape_x.begin()), n);
				in.ReadBufferFixEndianness(&(*robotShape_y.begin()), n);
			}

			if (version > 0)
			{  // Version 1 --------------
				if (version >= 10)
				{
					in >> cur_vel >> cur_vel_local;
				}
				else
				{
					float actual_v, actual_w;
					in >> actual_v >> actual_w;
					cur_vel = mrpt::math::TTwist2D(0, 0, 0);
					cur_vel_local =
						mrpt::math::TTwist2D(actual_v, .0, actual_w);
				}
			}
			else
			{  // Default values for old versions:
				cur_vel = mrpt::math::TTwist2D(0, 0, 0);
			}

			if (version < 13 && version > 1)
			{
				float old_estim_period;
				in >> old_estim_period;
				values["estimatedExecutionPeriod"] = old_estim_period;
			}

			for (i = 0; i < infoPerPTG.size(); i++)
			{
				infoPerPTG[i].evalFactors.clear();
			}
			if (version > 2)
			{
				// Version 3..22 ----------
				for (i = 0; i < infoPerPTG.size(); i++)
				{
					if (version < 22)
					{
						in >> n;
						for (unsigned int j = 0; j < n; j++)
						{
							float f;
							in >> f;
							infoPerPTG[i].evalFactors[mrpt::format("f%u", j)] =
								f;
						}
					}
					else
					{
						in >> infoPerPTG[i].evalFactors.base;
					}
				}
			}

			if (version > 3)
			{
				// Version 4 ----------
				in >> nPTGs;
				if (version < 9)  // Old "securityDistances", now unused.
				{
					in >> n;
					float dummy;
					for (i = 0; i < n; i++) in >> dummy;
				}
			}
			else
			{
				nPTGs = infoPerPTG.size();
			}

			if (version > 4)
			{
				if (version < 10)
				{
					int32_t navigatorBehavior;  // removed in v10
					in >> navigatorBehavior;
				}

				if (version < 6)  // Removed in version 6:
				{
					mrpt::poses::CPoint2D doorCrossing_P1, doorCrossing_P2;
					in >> doorCrossing_P1 >> doorCrossing_P2;
				}
			}

			if (version > 6 && version < 13)
			{
				mrpt::system::TTimeStamp tt;
				in >> tt;
				timestamps["tim_start_iteration"] = tt;
			}

			if (version >= 11)
			{
				in >> robotShape_radius;
			}
			else
			{
				robotShape_radius = 0.5;
			}

			if (version >= 12 && version < 15)
			{
				std::vector<std::vector<double>> dummy_cmd_vel_filterings;
				in >> dummy_cmd_vel_filterings;
			}

			if (version >= 13)
			{
				in >> values >> timestamps;
			}
			else
			{
				values.clear();
				timestamps.clear();
			}

			if (version >= 14)
			{
				if (version >= 24)
				{
					in >> relPoseSense >> relPoseVelCmd;
				}
				else
				{
					mrpt::poses::CPose2D crelPoseSense, crelPoseVelCmd;
					in >> crelPoseSense >> crelPoseVelCmd;
					relPoseSense = crelPoseSense.asTPose();
					relPoseVelCmd = crelPoseVelCmd.asTPose();
				}
			}
			else
			{
				relPoseSense = relPoseVelCmd = mrpt::math::TPose2D(0, 0, 0);
			}

			if (version >= 18)
				in >> additional_debug_msgs;
			else
				additional_debug_msgs.clear();

			if (version >= 24)
				navDynState.readFromStream(in);
			else
			{
				navDynState =
					CParameterizedTrajectoryGenerator::TNavDynamicState();
				navDynState.curVelLocal = cur_vel_local;
				if (!WS_targets_relative.empty())
					navDynState.relTarget = WS_targets_relative[0];
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

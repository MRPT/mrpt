/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/stl_serialization.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>

using namespace mrpt::nav;

IMPLEMENTS_SERIALIZABLE( CLogFileRecord, CSerializable,mrpt::nav )


/*---------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
CLogFileRecord::CLogFileRecord() :
    nPTGs     ( 0 )
{
	infoPerPTG.clear();
	WS_Obstacles.clear();
}

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CLogFileRecord::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 19;
	else
	{
		uint32_t	i,n;

		// Version 0 ---------
		n = infoPerPTG.size();
		out << n;
		for (i=0;i<n;i++)
		{
			out << infoPerPTG[i].PTG_desc.c_str();

			uint32_t m = infoPerPTG[i].TP_Obstacles.size();
			out << m;
			if (m) out.WriteBuffer((const void*)&(*infoPerPTG[i].TP_Obstacles.begin()), m * sizeof(infoPerPTG[i].TP_Obstacles[0]));

			out << infoPerPTG[i].TP_Target;  // v8: CPoint2D -> TPoint2D
			out << infoPerPTG[i].TP_Robot; // v17
			out << infoPerPTG[i].timeForTPObsTransformation << infoPerPTG[i].timeForHolonomicMethod; // made double in v12
			out << infoPerPTG[i].desiredDirection << infoPerPTG[i].desiredSpeed << infoPerPTG[i].evaluation; // made double in v12
			out << *infoPerPTG[i].HLFR;

			// Version 9: Removed security distances. Added optional field with PTG info.
			const bool there_is_ptg_data = infoPerPTG[i].ptg.present();
			out << there_is_ptg_data;
			if (there_is_ptg_data)
				out << infoPerPTG[i].ptg;

			out << infoPerPTG[i].clearance.raw_clearances; // v19
		}
		out << nSelectedPTG << WS_Obstacles << robotOdometryPose << WS_target_relative /*v8*/;
		// v16:
		out << ptg_index_NOP << ptg_last_k_NOP  << rel_cur_pose_wrt_last_vel_cmd_NOP << rel_pose_PTG_origin_wrt_sense_NOP;
		out << ptg_last_curRobotVelLocal; // v17

		if (ptg_index_NOP<0)
			out << cmd_vel /*v10*/ << cmd_vel_original; // v15

		// Previous values: REMOVED IN VERSION #6
		n = robotShape_x.size();
		out << n;
		if (n) {
			out.WriteBuffer((const void*)&(*robotShape_x.begin()), n*sizeof(robotShape_x[0]));
			out.WriteBuffer((const void*)&(*robotShape_y.begin()), n*sizeof(robotShape_y[0]));
		}

		// Version 1 ---------
		out << cur_vel<< cur_vel_local; /*v10*/
		//out << estimatedExecutionPeriod; // removed v13

		// Version 3 ----------
		for (i=0;i<infoPerPTG.size();i++)
		{
			n = infoPerPTG[i].evalFactors.size();

			out << n;
			for (unsigned int j=0;j<n;j++)
				out << infoPerPTG[i].evalFactors[j];
		}

		out << nPTGs; // v4
		// out << timestamp; // removed v13
		out << robotShape_radius; // v11
		//out << cmd_vel_filterings; // added v12: Removed in v15

		out << values << timestamps; // v13

		out << relPoseSense << relPoseVelCmd; // v14

		// v15: cmd_vel converted from std::vector<double> into CSerializable
		out << additional_debug_msgs; // v18
	}
}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CLogFileRecord::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
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
		{
			// Version 0 --------------
			uint32_t  i,n;

			infoPerPTG.clear();

			in >> n;
			infoPerPTG.resize(n);
			for (i=0;i<n;i++)
			{
				char str[256];
				in >> str;
				infoPerPTG[i].PTG_desc = std::string(str);

				int32_t m;
				in >> m;
				infoPerPTG[i].TP_Obstacles.resize(m);
				if (m) in.ReadBufferFixEndianness( &(*infoPerPTG[i].TP_Obstacles.begin()), m );

				if (version>=8)
					in >> infoPerPTG[i].TP_Target;
				else
				{
					mrpt::poses::CPoint2D pos;
					in >> pos;
					infoPerPTG[i].TP_Target = mrpt::math::TPoint2D(pos);
				}
				if (version >= 17)
					in >> infoPerPTG[i].TP_Robot;
				else infoPerPTG[i].TP_Robot = mrpt::math::TPoint2D(0, 0);

				if (version>=12) {
					in >> infoPerPTG[i].timeForTPObsTransformation >> infoPerPTG[i].timeForHolonomicMethod;
					in >> infoPerPTG[i].desiredDirection >> infoPerPTG[i].desiredSpeed >> infoPerPTG[i].evaluation;
				} else {
					in.ReadAsAndCastTo<float,double>(infoPerPTG[i].timeForTPObsTransformation);
					in.ReadAsAndCastTo<float,double>(infoPerPTG[i].timeForHolonomicMethod);
					in.ReadAsAndCastTo<float,double>(infoPerPTG[i].desiredDirection);
					in.ReadAsAndCastTo<float,double>(infoPerPTG[i].desiredSpeed);
					in.ReadAsAndCastTo<float,double>(infoPerPTG[i].evaluation);
				}

				in >> infoPerPTG[i].HLFR;

				if (version>=9) // Extra PTG info
				{
					infoPerPTG[i].ptg.clear();

					bool there_is_ptg_data;
					in >> there_is_ptg_data;
					if (there_is_ptg_data)
						infoPerPTG[i].ptg = mrpt::nav::CParameterizedTrajectoryGeneratorPtr( in.ReadObject() );
				}

				if (version >= 19) {
					in >> infoPerPTG[i].clearance.raw_clearances;
				}
				else {
					infoPerPTG[i].clearance.raw_clearances.clear();
				}
			}

			in >> nSelectedPTG >> WS_Obstacles >> robotOdometryPose;

			if (version>=8)
				in >> WS_target_relative;
			else
			{
				mrpt::poses::CPoint2D pos;
				in >> pos;
				WS_target_relative = mrpt::math::TPoint2D(pos);
			}

			if (version >= 16) {
				in >> ptg_index_NOP >> ptg_last_k_NOP >> rel_cur_pose_wrt_last_vel_cmd_NOP >> rel_pose_PTG_origin_wrt_sense_NOP;
			}
			else {
				ptg_index_NOP = -1;
			}
			if (version >= 17)
				in >> ptg_last_curRobotVelLocal; // v17
			else
				ptg_last_curRobotVelLocal = mrpt::math::TTwist2D(0, 0, 0);

			if (version >= 10) {
				if (version >= 15) {
					if (ptg_index_NOP<0)
						in >> cmd_vel;
				}
				else {
					std::vector<double> vel;
					in >> vel;
					if (vel.size() == 2)
						cmd_vel = mrpt::kinematics::CVehicleVelCmdPtr(new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
					else cmd_vel = mrpt::kinematics::CVehicleVelCmdPtr(new mrpt::kinematics::CVehicleVelCmd_Holo);
					for (size_t i = 0; i < cmd_vel->getVelCmdLength(); i++)
						cmd_vel->setVelCmdElement(i, vel[i]);
				}
			}
			else {
				float v, w;
				in >> v >> w;
				cmd_vel = mrpt::kinematics::CVehicleVelCmdPtr(new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
				cmd_vel->setVelCmdElement(0, v);
				cmd_vel->setVelCmdElement(0, w);
			}

			if (version>=15 && ptg_index_NOP<0)
				in >> cmd_vel_original;

			if (version < 13) {
				float old_exec_time;  in >> old_exec_time;
				values["executionTime"] = old_exec_time;
			}

			if (version<6)
			{
				mrpt::math::CVectorFloat prevV,prevW,prevSelPTG;

				// Previous values: (Removed in version 6)
				in >> n;
				prevV.resize(n);
				if (n) in.ReadBufferFixEndianness( &(*prevV.begin()),n);

				in >> n;
				prevW.resize(n);
				if (n) in.ReadBufferFixEndianness( &(*prevW.begin()),n);

				in >> n;
				prevSelPTG.resize(n);
				if (n) in.ReadBufferFixEndianness( &(*prevSelPTG.begin()),n);
			}

			in >> n;
			robotShape_x.resize(n);
			robotShape_y.resize(n);
			if (n) {
				in.ReadBufferFixEndianness( &(*robotShape_x.begin()), n);
				in.ReadBufferFixEndianness( &(*robotShape_y.begin()), n);
			}

			if (version > 0)
			{	// Version 1 --------------
				if (version >= 10) {
					in >> cur_vel >> cur_vel_local;
				}
				else {
					float actual_v, actual_w;
					in >> actual_v >> actual_w;
					cur_vel = mrpt::math::TTwist2D(0,0,0);
					cur_vel_local= mrpt::math::TTwist2D(actual_v, .0, actual_w );
				}
			}
			else
			{	// Default values for old versions:
				cur_vel = mrpt::math::TTwist2D(0,0,0);
			}

			if (version < 13 && version>1) {
				float old_estim_period;  in >> old_estim_period;
				values["estimatedExecutionPeriod"] = old_estim_period;
			}

			if (version > 2)
			{
				// Version 3 ----------
				for (i=0;i<infoPerPTG.size();i++)
				{

					in >> n;
					infoPerPTG[i].evalFactors.resize(n);
					for (unsigned int j=0;j<n;j++)
						in >> infoPerPTG[i].evalFactors[j];
				}

			}
			else
			{
				for (i=0;i<infoPerPTG.size();i++)
					infoPerPTG[i].evalFactors.resize(0);
			}

			if (version > 3)
			{
				// Version 4 ----------
				in >> nPTGs;
				if (version <9)  // Old "securityDistances", now unused.
				{
					in >> n;
					float dummy;
					for (i=0;i<n;i++)
						in >> dummy;
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
					int32_t navigatorBehavior; // removed in v10
					in >> navigatorBehavior;
				}

				if (version<6)  // Removed in version 6:
				{
					mrpt::poses::CPoint2D doorCrossing_P1,doorCrossing_P2;
					in >> doorCrossing_P1 >> doorCrossing_P2;
				}
			}

			if (version>6 && version<13) {
				mrpt::system::TTimeStamp tt; in >> tt;
				timestamps["tim_start_iteration"] = tt;
			}

			if (version>=11) {
				in >> robotShape_radius;
			} else {
				robotShape_radius = 0.5;
			}

			if (version >= 12 && version<15) {
				std::vector<std::vector<double> > dummy_cmd_vel_filterings;
				in >> dummy_cmd_vel_filterings;
			}

			if (version >= 13) {
				in >> values >> timestamps;
			}
			else {
				values.clear();
				timestamps.clear();
			}

			if (version >= 14) {
				in >> relPoseSense >> relPoseVelCmd;
			}
			else {
				relPoseSense = relPoseVelCmd = mrpt::poses::CPose2D();
			}

			if (version>=18) 
			     in >> additional_debug_msgs;
			else additional_debug_msgs.clear();

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CRobotSimulator_H
#define CRobotSimulator_H
#warning mrpt::utils::CRobotSimulator is deprecated and will be removed. Use mrpt::kinematics::CVehicleSimul_DiffDriven instead.
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
namespace mrpt
{
	namespace utils
	{
		class CRobotSimulator : public kinematics::CVehicleSimul_DiffDriven
		{
			public:
			CRobotSimulator(float TAU=0, float DELAY=0){setDelayModelParams(TAU,DELAY);} 
			void getOdometry(poses::CPose2D &pose) const {pose = getCurrentOdometricPose();}
			void getRealPose(poses::CPose2D &pose) const {pose = getCurrentGTPose();}
			void getOdometry(math::TPose2D &pose) const {pose = getCurrentOdometricPose();}
			void getRealPose(math::TPose2D &pose) const {pose = getCurrentGTPose();}
                        void setOdometry(const math::TPose2D &pose) {setCurrentOdometricPose(pose);}
                        void setRealPose(const math::TPose2D &pose) {setCurrentGTPose(pose);}
			double getX() const{return getCurrentGTPose().x;}
			double getY() const{return getCurrentGTPose().y;}
			double getPHI() const{return getCurrentGTPose().phi;}
			double getT() const{return getTime();}
			void simulateInterval(double At){ simulateOneTimeStep(At);}
			void resetOdometry(const poses::CPose2D pose = poses::CPose2D()){setCurrentOdometricPose(pose);}
		};
	}
}
#endif

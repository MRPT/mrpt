/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/planners/TPath.h>
#include <mrpt/system/filesystem.h> // directoryExists(), ...
#include <mrpt/system/os.h>

using namespace mrpt::nav;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace std;

TPath::TPath()
{
    //ctor
}

TPath::~TPath()
{
    //dtor
}

/*---------------------------------------------------------------
            compute the path length using Euclidean metric
  ---------------------------------------------------------------*/
double TPath::TPlannedPath::lengthEuclidean() const
{
    if (this->size()==0)//(path.size()==0)
    {
        std::cout << "info from -TPath::lengthEuclidean- message: the path size is 0 " << std::endl;
        return 0.0;
    }
    double L = 0;
    for (size_t i=0; i<this->size()-1; i++)
        L +=sqrt(square((*this)[i+1].p.x-(*this)[i].p.x)+square((*this)[i+1].p.y-(*this)[i].p.y));

    return L;
}

/*---------------------------------------------------------------
            compute the path length using psm metric
  ---------------------------------------------------------------*/
double TPath::TPlannedPath::lengthPsm() const
{
    if (this->size()==0)
    {
        std::cout << "info from -TPath::lengthEuclidean- message: the path size is 0 " << std::endl;
        return 0.0;
    }
    double L = 0;

    for (size_t i=0; i<this->size()-1; i++)
        L +=(*this)[i].ptg_dist;

return L;
}

/*---------------------------------------------------------------
            save path to file
  ---------------------------------------------------------------*/
bool  TPath::TPlannedPath::save_to_text_file(const std::string &file) const
{

    MRPT_TODO("Add a flag to activate the log saving")
    std::string save_folder = "./nav.logs/paths/";
    if (!mrpt::system::directoryExists(save_folder.c_str()))
		{
			mrpt::system::createDirectory( "./nav.logs"); //if I take it out it will create the subdirectories?
			mrpt::system::createDirectory( save_folder.c_str());
		}
    std::string file_name = save_folder+file;
	std::FILE	*f=os::fopen(file_name.c_str(),"wt");
	if (!f) return false;

//mrpt::system::createDirectory() //if it doesn't exist
    if (this->size()==0)
    {
        std::cout << "info from -TPath::save_to_text_file- message: the path size is 0 " << std::endl;
        os::fclose(f);
        return false;
    }

    TPath tpath;
	os::fprintf(f,"%% Path file --- \n%% euclidean path length = %02f [m],\n%% PTG path length = %02f [psm]\n%% Is approximated = %i --> 0 the target is reached, 1 the solution is approximated\n",
                                (*this).lengthEuclidean(), (*this).lengthPsm(), (*this).isApproximate);
//in the file max_v, max_w are always 0 beause they are taken from the first motion assigned to 0 in the first step
	os::fprintf(f,"%% x [m]  y [m]   phi [deg] ptg_index   K   ptg_dist [psm]   v [m/s]  w [rad/s]\n");

	for (unsigned int i=0;i<this->size();i++)
		os::fprintf(f,  "%02f   %02f  %02f  %d  %d  %02f %02f %02f\n",
                        (*this)[i].p.x, (*this)[i].p.y, RAD2DEG((*this)[i].p.phi),
                        (*this)[i].ind_ptg, (*this)[i].K, (*this)[i].ptg_dist,
                        (*this)[i].max_v, (*this)[i].max_w );

	os::fclose(f);/**/
	return true;

}

/*---------------------------------------------------------------
            get the closest path index to a robot pose
  ---------------------------------------------------------------*/

  int TPath::TPlannedPath::getClosestPathIndex (mrpt::math::TPose2D &robotPose) const
  {

    try
	{

        mrpt::nav::TPath::TPlannedPath m_planned_path_ = *this;
        mrpt::nav::TPath::TPathData	    next_planned_point;  // next target
        //mrpt::reactivenav::TPath::TPathData	    last_planned_point;  // target

        if (m_planned_path_.empty())// || m_planned_path_.size()<2)
            return 0;//m_planned_path_.size();// if there is no plan or the plan is too short just return the last point

        TPose2D  robotPose_=robotPose;

        {
            int closest_path_index=0;  //the first condition will be always true since < than infinity
			float robot2path_distance=std::numeric_limits<float>::infinity();
            for (size_t i=0; i<this->size()-1; i++)  //for all points in the path
			    {
                    next_planned_point = m_planned_path_.front();
                    m_planned_path_.erase(m_planned_path_.begin());
                    if( (CPose2D(next_planned_point.p).distance2DTo(robotPose.x, robotPose.y))<robot2path_distance)
                        {closest_path_index = i;
                        robot2path_distance=CPose2D(next_planned_point.p).distance2DTo(robotPose.x, robotPose.y);
                        }
 			    }
            return closest_path_index;
        }
	}
	catch (std::exception &e)
	{
		cerr <<"[TPath::TPlannedPath::getClosestPathIndex] Exception:"<<endl;
		cerr << e.what() << endl;
		return 0;
	}
  }

/*---------------------------------------------------------------
            get the next breadcrumb point
  ---------------------------------------------------------------*/
bool TPath::TPlannedPath::getBreadcrumbPoint(mrpt::nav::TPath::TPathData &out_next_point,
                                             mrpt::math::TPose2D &robotPose,
                                             mrpt::math::TPose2D &m_target_pose,
                                             double breadcrumb_dist)
{
    try
	{
    mrpt::nav::TPath::TPathData	    next_planned_point;  // next target
	mrpt::nav::TPath::TPathData	    last_planned_point;  // target
    mrpt::nav::TPath::TPlannedPath    m_planned_path_=*this;

    // we have proper localization
    // Acquire the latest path plan --------------
    {
        if (m_planned_path_.empty() || m_planned_path_.size()<2)
        {
            // There's no plan: Stop the robot:
            //planned_path_time = INVALID_TIMESTAMP;
            // and return the target pose as the next reactive pose
            //next_reatctive_point=m_target_pose;
            next_planned_point.p.x = robotPose.x;//m_target_pose.x;//robotPose.x;
            next_planned_point.p.y = robotPose.y;//m_target_pose.y;//robotPose.y;
            next_planned_point.p.phi = robotPose.phi;//m_target_pose.phi;//robotPose.phi;

			out_next_point = next_planned_point;//next_reatctive_point = robotPose;  //if there is no plan just return the robot pose
			return false;
			//don't send any command if you don't have a plan!
        }
        else  // Only update our copy if needed:
        {
            next_planned_point = m_planned_path_.front();
            last_planned_point = m_planned_path_.back();
        }
    } // end of CS

    // Acquire the current estimate of the robot state:
	// ----------------------------------------------------
    int starting_point = 0;
    if (m_planned_path_.size()>3)
        starting_point = m_planned_path_.getClosestPathIndex(robotPose);//getClosestPathIndex();

	for (size_t i=starting_point; i<m_planned_path_.size(); i++)  //for all points in the path beginning from the closest
        {
         next_planned_point = m_planned_path_[i];

			    // Compute target in coordinates relative to the robot just now:
				const CPose2D trg_rel = CPose2D(next_planned_point.p) - CPose2D(robotPose);
                float dist_p2t = CPose2D(m_target_pose).distance2DTo(robotPose.x, robotPose.y);

                if(abs(dist_p2t)<breadcrumb_dist)  //define a parameter for the maximum distance to the target
				{
                  out_next_point = last_planned_point;//next_reatctive_point=m_target_pose;  //if we are close to the target just return the target as the pose
				  return true;
				}

                float dist_p2p = CPose2D(next_planned_point.p).distance2DTo(robotPose.x, robotPose.y);
                if(abs(dist_p2p)<breadcrumb_dist)
                    this->erase(this->begin());   //this point will be not followed and it will be eliminated !!!                }
                else
				{
                    out_next_point = next_planned_point;//next_reatctive_point = CPose2D(next_planned_point.p);
					return true;
				}
			}
        //}//end - for all points in the path

	// Don't have next point.
	return false;

    }
	catch (std::exception &e)
	{
		cerr <<"[TPath::TPlannedPath::getBreadcrumbPoint] Exception:"<<endl;
		cerr << e.what() << endl;
		return false;
	}

}


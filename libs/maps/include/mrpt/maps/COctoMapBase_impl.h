/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// This file is to be included from <mrpt/maps/COctoMapBase.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/maps/CPointsMap.h>

namespace mrpt
{
	namespace maps
	{
		template <class OCTREE,class OCTREE_NODE>
		bool COctoMapBase<OCTREE,OCTREE_NODE>::internal_build_PointCloud_for_observation(const mrpt::obs::CObservation *obs,const mrpt::poses::CPose3D *robotPose, octomap::point3d &sensorPt, octomap::Pointcloud &scan) const
		{
			using namespace mrpt::poses;
			using namespace mrpt::obs;

			scan.clear();

			CPose3D		robotPose3D;
			if (robotPose) // Default values are (0,0,0)
				robotPose3D = (*robotPose);

			if ( IS_CLASS(obs,CObservation2DRangeScan) )
			{
			   /********************************************************************
						OBSERVATION TYPE: CObservation2DRangeScan
				********************************************************************/
				const CObservation2DRangeScan	*o = static_cast<const CObservation2DRangeScan*>( obs );

				// Build a points-map representation of the points from the scan (coordinates are wrt the robot base)

				// Sensor_pose = robot_pose (+) sensor_pose_on_robot
				CPose3D sensorPose(UNINITIALIZED_POSE);
				sensorPose.composeFrom(robotPose3D,o->sensorPose);
				sensorPt = octomap::point3d(sensorPose.x(),sensorPose.y(),sensorPose.z());

				const CPointsMap *scanPts = o->buildAuxPointsMap<mrpt::maps::CPointsMap>();
				const size_t nPts = scanPts->size();

				// Transform 3D point cloud:
				scan.reserve(nPts);

				mrpt::math::TPoint3Df pt;
				for (size_t i=0;i<nPts;i++)
				{
					// Load the next point:
					scanPts->getPointFast(i,pt.x,pt.y,pt.z);

					// Translation:
					double gx,gy,gz;
					robotPose3D.composePoint(pt.x,pt.y,pt.z,  gx,gy,gz);

					// Add to this map:
					scan.push_back(gx,gy,gz);
				}
				return true;
			}
			else if ( IS_CLASS(obs,CObservation3DRangeScan) )
			{
			   /********************************************************************
						OBSERVATION TYPE: CObservation3DRangeScan
				********************************************************************/
				const CObservation3DRangeScan	*o = static_cast<const CObservation3DRangeScan*>( obs );

				// Build a points-map representation of the points from the scan (coordinates are wrt the robot base)
				if (!o->hasPoints3D)
					return false;

				// Sensor_pose = robot_pose (+) sensor_pose_on_robot
				CPose3D sensorPose(UNINITIALIZED_POSE);
				sensorPose.composeFrom(robotPose3D,o->sensorPose);
				sensorPt = octomap::point3d(sensorPose.x(),sensorPose.y(),sensorPose.z());

				o->load(); // Just to make sure the points are loaded from an external source, if that's the case...
				const size_t sizeRangeScan = o->points3D_x.size();

				// Transform 3D point cloud:
				scan.reserve(sizeRangeScan);

				// For quicker access to values as "float" instead of "doubles":
				mrpt::math::CMatrixDouble44  H;
				robotPose3D.getHomogeneousMatrix(H);
				const float	m00 = H.get_unsafe(0,0);
				const float	m01 = H.get_unsafe(0,1);
				const float	m02 = H.get_unsafe(0,2);
				const float	m03 = H.get_unsafe(0,3);
				const float	m10 = H.get_unsafe(1,0);
				const float	m11 = H.get_unsafe(1,1);
				const float	m12 = H.get_unsafe(1,2);
				const float	m13 = H.get_unsafe(1,3);
				const float	m20 = H.get_unsafe(2,0);
				const float	m21 = H.get_unsafe(2,1);
				const float	m22 = H.get_unsafe(2,2);
				const float	m23 = H.get_unsafe(2,3);

				mrpt::math::TPoint3Df pt;
				for (size_t i=0;i<sizeRangeScan;i++)
				{
					pt.x = o->points3D_x[i];
					pt.y = o->points3D_y[i];
					pt.z = o->points3D_z[i];

					// Valid point?
					if ( pt.x!=0 || pt.y!=0 || pt.z!=0 )
					{
						// Translation:
						const float gx = m00*pt.x + m01*pt.y + m02*pt.z + m03;
						const float gy = m10*pt.x + m11*pt.y + m12*pt.z + m13;
						const float gz = m20*pt.x + m21*pt.y + m22*pt.z + m23;

						// Add to this map:
						scan.push_back(gx,gy,gz);
					}
				}
				return true;
			}

			return false;
		}

		template <class OCTREE,class OCTREE_NODE>
		bool COctoMapBase<OCTREE,OCTREE_NODE>::isEmpty() const
		{
			return m_octomap.size()==1;
		}

		template <class OCTREE,class OCTREE_NODE>
		void COctoMapBase<OCTREE,OCTREE_NODE>::saveMetricMapRepresentationToFile(const std::string	&filNamePrefix) const
		{
			MRPT_START

			// Save as 3D Scene:
			{
				mrpt::opengl::COpenGLScene scene;
				mrpt::opengl::CSetOfObjectsPtr obj3D = mrpt::opengl::CSetOfObjects::Create();

				this->getAs3DObject(obj3D);

				scene.insert(obj3D);

				const std::string fil = filNamePrefix + std::string("_3D.3Dscene");
				mrpt::utils::CFileOutputStream	f(fil);
				f << scene;
			}

			// Save as ".bt" file (a binary format from the octomap lib):
			{
				const std::string fil = filNamePrefix + std::string("_binary.bt");
				const_cast<OCTREE*>(&m_octomap)->writeBinary(fil);
			}
			MRPT_END
		}

		template <class OCTREE,class OCTREE_NODE>
		double COctoMapBase<OCTREE,OCTREE_NODE>::internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom )
		{
			octomap::point3d     sensorPt;
			octomap::Pointcloud  scan;

			if (!internal_build_PointCloud_for_observation(obs,&takenFrom, sensorPt, scan))
				return 0; // Nothing to do.

			octomap::OcTreeKey key;
			const size_t N=scan.size();

			double log_lik = 0;
			for (size_t i=0;i<N;i+=likelihoodOptions.decimation)
			{
				if (m_octomap.coordToKeyChecked(scan.getPoint(i), key))
				{
					octree_node_t *node = m_octomap.search(key,0 /*depth*/);
					if (node)
						log_lik += std::log(node->getOccupancy());
				}
			}

			return log_lik;
		}

		template <class OCTREE,class OCTREE_NODE>
		bool COctoMapBase<OCTREE,OCTREE_NODE>::getPointOccupancy(const float x,const float y,const float z, double &prob_occupancy) const
		{
			octomap::OcTreeKey key;
			if (m_octomap.coordToKeyChecked(octomap::point3d(x,y,z), key))
			{
				octree_node_t *node = m_octomap.search(key,0 /*depth*/);
				if (!node) return false;

				prob_occupancy = node->getOccupancy();
				return true;
			}
			else return false;
		}

		template <class OCTREE,class OCTREE_NODE>
		void COctoMapBase<OCTREE,OCTREE_NODE>::insertPointCloud(const CPointsMap &ptMap, const float sensor_x,const float sensor_y,const float sensor_z)
		{
			MRPT_START
			const octomap::point3d sensorPt(sensor_x,sensor_y,sensor_z);
			size_t N;
			const float *xs,*ys,*zs;
			ptMap.getPointsBuffer(N,xs,ys,zs);
			for (size_t i=0;i<N;i++)
				m_octomap.insertRay(sensorPt, octomap::point3d(xs[i],ys[i],zs[i]), insertionOptions.maxrange,insertionOptions.pruning);
			MRPT_END
		}

		template <class OCTREE,class OCTREE_NODE>
		bool COctoMapBase<OCTREE,OCTREE_NODE>::castRay(const mrpt::math::TPoint3D & origin,const mrpt::math::TPoint3D & direction,mrpt::math::TPoint3D & end,bool ignoreUnknownCells,double maxRange) const
		{
			octomap::point3d _end;

			const bool ret=m_octomap.castRay(
				octomap::point3d(origin.x,origin.y,origin.z),
				octomap::point3d(direction.x,direction.y,direction.z),
				_end,ignoreUnknownCells,maxRange);

			end.x = _end.x();
			end.y = _end.y();
			end.z = _end.z();
			return ret;
		}



		/*---------------------------------------------------------------
						TInsertionOptions
		 ---------------------------------------------------------------*/
		template <class OCTREE,class OCTREE_NODE>
		COctoMapBase<OCTREE,OCTREE_NODE>::TInsertionOptions::TInsertionOptions(COctoMapBase<OCTREE,OCTREE_NODE> &parent) :
			maxrange (-1.),
			pruning  (true),
			m_parent (&parent),
			// Default values from octomap:
			occupancyThres (0.5),
			probHit(0.7),
			probMiss(0.4),
			clampingThresMin(0.1192),
			clampingThresMax(0.971)
		{
		}

		template <class OCTREE,class OCTREE_NODE>
		COctoMapBase<OCTREE,OCTREE_NODE>::TInsertionOptions::TInsertionOptions() :
			maxrange (-1.),
			pruning  (true),
			m_parent (NULL),
			// Default values from octomap:
			occupancyThres (0.5),
			probHit(0.7),
			probMiss(0.4),
			clampingThresMin(0.1192),
			clampingThresMax(0.971)
		{
		}

		template <class OCTREE,class OCTREE_NODE>
		COctoMapBase<OCTREE,OCTREE_NODE>::TLikelihoodOptions::TLikelihoodOptions() :
			decimation ( 1 )
		{
		}

		template <class OCTREE,class OCTREE_NODE>
		void COctoMapBase<OCTREE,OCTREE_NODE>::TLikelihoodOptions::writeToStream(mrpt::utils::CStream &out) const
		{
			const int8_t version = 0;
			out << version;
			out << decimation;
		}

		template <class OCTREE,class OCTREE_NODE>
		void COctoMapBase<OCTREE,OCTREE_NODE>::TLikelihoodOptions::readFromStream(mrpt::utils::CStream &in)
		{
			int8_t version;
			in >> version;
			switch(version)
			{
				case 0:
				{
					in >> decimation;
				}
				break;
				default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
			}
		}


		/*---------------------------------------------------------------
							dumpToTextStream
		  ---------------------------------------------------------------*/
		template <class OCTREE,class OCTREE_NODE>
		void  COctoMapBase<OCTREE,OCTREE_NODE>::TInsertionOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
		{
			out.printf("\n----------- [COctoMapBase<>::TInsertionOptions] ------------ \n\n");

			LOADABLEOPTS_DUMP_VAR(maxrange,double);
			LOADABLEOPTS_DUMP_VAR(pruning,bool);

			LOADABLEOPTS_DUMP_VAR(getOccupancyThres(),double);
			LOADABLEOPTS_DUMP_VAR(getProbHit(),double);
			LOADABLEOPTS_DUMP_VAR(getProbMiss(),double);
			LOADABLEOPTS_DUMP_VAR(getClampingThresMin(),double);
			LOADABLEOPTS_DUMP_VAR(getClampingThresMax(),double);

			out.printf("\n");
		}

		template <class OCTREE,class OCTREE_NODE>
		void  COctoMapBase<OCTREE,OCTREE_NODE>::TLikelihoodOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
		{
			out.printf("\n----------- [COctoMapBase<>::TLikelihoodOptions] ------------ \n\n");

			LOADABLEOPTS_DUMP_VAR(decimation,int);
		}

		/*---------------------------------------------------------------
							loadFromConfigFile
		  ---------------------------------------------------------------*/
		template <class OCTREE,class OCTREE_NODE>
		void  COctoMapBase<OCTREE,OCTREE_NODE>::TInsertionOptions::loadFromConfigFile(
			const mrpt::utils::CConfigFileBase  &iniFile,
			const std::string &section)
		{
			MRPT_LOAD_CONFIG_VAR(maxrange,double, iniFile,section);
			MRPT_LOAD_CONFIG_VAR(pruning,bool, iniFile,section);

			MRPT_LOAD_CONFIG_VAR(occupancyThres,double, iniFile,section);
			MRPT_LOAD_CONFIG_VAR(probHit,double, iniFile,section);
			MRPT_LOAD_CONFIG_VAR(probMiss,double, iniFile,section);
			MRPT_LOAD_CONFIG_VAR(clampingThresMin,double, iniFile,section);
			MRPT_LOAD_CONFIG_VAR(clampingThresMax,double, iniFile,section);

			// Set loaded options into the actual octomap object, if any:
			this->setOccupancyThres(occupancyThres);
			this->setProbHit(probHit);
			this->setProbMiss(probMiss);
			this->setClampingThresMin(clampingThresMin);
			this->setClampingThresMax(clampingThresMax);
		}

		template <class OCTREE,class OCTREE_NODE>
		void  COctoMapBase<OCTREE,OCTREE_NODE>::TLikelihoodOptions::loadFromConfigFile(
			const mrpt::utils::CConfigFileBase  &iniFile,
			const std::string &section)
		{
			MRPT_LOAD_CONFIG_VAR(decimation,int,iniFile,section);
		}

		/*  COctoMapColoured */
		template <class OCTREE,class OCTREE_NODE>
		void COctoMapBase<OCTREE,OCTREE_NODE>::TRenderingOptions::writeToStream(mrpt::utils::CStream &out) const
		{
			const int8_t version = 0;
			out << version;
			out << 	generateGridLines << generateOccupiedVoxels << visibleOccupiedVoxels
				<< generateFreeVoxels << visibleFreeVoxels;
		}

		template <class OCTREE,class OCTREE_NODE>
		void COctoMapBase<OCTREE,OCTREE_NODE>::TRenderingOptions::readFromStream(mrpt::utils::CStream &in)
		{
			int8_t version;
			in >> version;
			switch(version)
			{
				case 0:
				{
					in >> generateGridLines >> generateOccupiedVoxels >> visibleOccupiedVoxels
						>> generateFreeVoxels >> visibleFreeVoxels;
				}
				break;
				default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
			}
		}


	} // End of namespace
} // End of namespace



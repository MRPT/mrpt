/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/wrap2pi.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/round.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/utils/mrpt_stdint.h>    // compiler-independent version of "stdint.h"
#include <mrpt/nav/link_pragmas.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>  // STL+ library

namespace mrpt { namespace opengl { class CSetOfLines; } }

namespace mrpt
{
namespace nav
{
	/** \defgroup nav_tpspace TP-Space and PTG classes
	  * \ingroup mrpt_nav_grp
	  */
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CParameterizedTrajectoryGenerator, mrpt::utils::CSerializable, NAV_IMPEXP)

	/** This is the base class for any user-defined PTG.
	 *  There is a class factory interface in CParameterizedTrajectoryGenerator::CreatePTG.
	 *
	 * Papers:
	 *  - J.L. Blanco, J. Gonzalez-Jimenez, J.A. Fernandez-Madrigal, "Extending Obstacle Avoidance Methods through Multiple Parameter-Space Transformations", Autonomous Robots, vol. 24, no. 1, 2008. http://ingmec.ual.es/~jlblanco/papers/blanco2008eoa_DRAFT.pdf
	 *
	 * Changes history:
	 *	- 30/JUN/2004: Creation (JLBC)
	 *	- 16/SEP/2004: Totally redesigned.
	 *	- 15/SEP/2005: Totally rewritten again, for integration into MRPT Applications Repository.
	 *	- 19/JUL/2009: Simplified to use only STL data types, and created the class factory interface.
	 *	- MAY/2016: Refactored into CParameterizedTrajectoryGenerator, CPTG_DiffDrive_CollisionGridBased, PTG classes renamed.
	 *
	 *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP CParameterizedTrajectoryGenerator : 
		public mrpt::utils::CSerializable,
		public mrpt::utils::CLoadableOptions
	{
		DEFINE_VIRTUAL_SERIALIZABLE(CParameterizedTrajectoryGenerator)
	public:
		CParameterizedTrajectoryGenerator(); //!< Default ctor. Must call `loadFromConfigFile()` before initialization
		virtual ~CParameterizedTrajectoryGenerator() //!<  Destructor 

		{ }

		/** The class factory for creating a PTG from a list of parameters in a section of a given config file (physical file or in memory).
		  *  Possible parameters are:
		  *	  - Those explained in CParameterizedTrajectoryGenerator::loadFromConfigFile()
		  *	  - Those explained in the specific PTG being created (see list of derived classes)
		  *
		  * `ptgClassName` can be any PTG class name which has been registered as any other
		  * mrpt::utils::CSerializable class.
		  *
		  * \exception std::logic_error On invalid or missing parameters.
		  */
		static CParameterizedTrajectoryGenerator * CreatePTG(const std::string &ptgClassName, const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection,  const std::string &sKeyPrefix);

		/** @name Virtual interface of each PTG implementation
		 *  @{ */
		virtual std::string getDescription() const = 0 ; //!< Gets a short textual description of the PTG and its parameters

	protected:
		/** Must be called after setting all PTG parameters and before requesting converting obstacles to TP-Space, inverseMap_WS2TP(), etc. */
		virtual void internal_initialize(const std::string & cacheFilename = std::string(), const bool verbose = true) = 0;
		/** This must be called to de-initialize the PTG if some parameter is to be changed. After changing it, call initialize again */
		virtual void internal_deinitialize() = 0;
	public:

		/** Computes the closest (alpha,d) TP coordinates of the trajectory point closest to the Workspace (WS) 
		 *   Cartesian coordinates (x,y), relative to the current robot frame.
		  * \param[in] x X coordinate of the query point, relative to the robot frame.
		  * \param[in] y Y coordinate of the query point, relative to the robot frame.
		  * \param[out] out_k Trajectory parameter index (discretized alpha value, 0-based index).
		  * \param[out] out_d Trajectory distance, normalized such that D_max becomes 1.
		  *
		  * \return true if the distance between (x,y) and the actual trajectory point is below the given tolerance (in meters).
		  */
		virtual bool inverseMap_WS2TP(double x, double y, int &out_k, double &out_normalized_d, double tolerance_dist = 0.10) const = 0;

		/** Returns the same than inverseMap_WS2TP() but without any additional cost. The default implementation
		  * just calls inverseMap_WS2TP() and discards (k,d). */
		virtual bool PTG_IsIntoDomain(double x, double y ) const {
			int k; double d;
			return inverseMap_WS2TP(x,y,k,d);
		}

		/** Converts a discretized "alpha" value into a feasible motion command or action. See derived classes for the meaning of these actions */
		virtual void directionToMotionCommand( uint16_t k, std::vector<double> &out_action_cmd ) const = 0;

		/** Callback whenever we have new info about the velocity state of the robot right now. May be used by some PTGs and discarded by others.
		  * \param[in] curVelLocal The current robot velocities in the local frame of reference (+X: forwards, omega: clockwise rotation) */
		virtual void updateCurrentRobotVel(const mrpt::math::TTwist2D &curVelLocal) = 0;

		virtual void setRefDistance(const double refDist) { refDistance=refDist; }

		/** Access path `k` ([0,N-1]=>[-pi,pi] in alpha): number of discrete "steps" along the trajectory.
		  * May be actual steps from a numerical integration or an arbitrary small length for analytical PTGs.
		  * \sa getAlphaValuesCount() */
		virtual size_t getPathStepCount(uint16_t k) const = 0;

		/** Access path `k` ([0,N-1]=>[-pi,pi] in alpha): pose of the vehicle at discrete step `step`.
		  * \sa getPathStepCount(), getAlphaValuesCount() */
		virtual void getPathPose(uint16_t k, uint16_t step, mrpt::math::TPose2D &p) const = 0;

		/** Access path `k` ([0,N-1]=>[-pi,pi] in alpha): traversed distance at discrete step `step`.
		  * \return Distance in pseudometers (real distance, NOT normalized to [0,1] for [0,refDist])
		  * \sa getPathStepCount(), getAlphaValuesCount() */
		virtual double getPathDist(uint16_t k, uint16_t step) const = 0;

		/** Access path `k` ([0,N-1]=>[-pi,pi] in alpha): largest step count for which the traversed distance is < `dist`
		  * \param[in] dist Distance in pseudometers (real distance, NOT normalized to [0,1] for [0,refDist])
		  * \return false if no step fulfills the condition for the given trajectory `k` (e.g. out of reference distance).
		  * Note that, anyway, the maximum distance (closest point) is returned in `out_step`.
		  * \sa getPathStepCount(), getAlphaValuesCount() */
		virtual bool getPathStepForDist(uint16_t k, double dist, uint16_t &out_step) const = 0;

		/** Updates the radial map of closest TP-Obstacles given a single obstacle point at (ox,oy)
		  * \param [in,out] tp_obstacles A vector of length `getAlphaValuesCount()`, initialized with `initTPObstacles()` (collision-free ranges, in "pseudometers", un-normalized).
		  * \param [in] ox Obstacle point (X), relative coordinates wrt origin of the PTG.
		  * \param [in] oy Obstacle point (Y), relative coordinates wrt origin of the PTG.
		  * \note The length of tp_obstacles is not checked for efficiency since this method is potentially called thousands of times per
		  *  navigation timestap, so it is left to the user responsibility to provide a valid buffer.
		  */
		virtual void updateTPObstacle(double ox, double oy, std::vector<double> &tp_obstacles) const = 0;

		/** Loads a set of default parameters into the PTG. Users normally will call `loadFromConfigFile()` instead, this method is provided 
		  * exclusively for the PTG-configurator tool. */
		virtual void loadDefaultParams();

		/** @} */  // --- end of virtual methods

		static std::string OUTPUT_DEBUG_PATH_PREFIX; //!< The path used as defaul output in, for example, debugDumpInFiles. (Default="./reactivenav.logs/")

		/** Must be called after setting all PTG parameters and before requesting converting obstacles to TP-Space, inverseMap_WS2TP(), etc. */
		void initialize(const std::string & cacheFilename = std::string(), const bool verbose = true);
		/** This must be called to de-initialize the PTG if some parameter is to be changed. After changing it, call initialize again */
		void deinitialize();
		/** Returns true if `initialize()` has been called and there was no errors, so the PTG is ready to be queried for paths, obstacles, etc. */
		bool isInitialized() const;

		/** Get the number of different, discrete paths in this family */
		uint16_t getAlphaValuesCount() const { return m_alphaValuesCount; }
		/** Get the number of different, discrete paths in this family */
		uint16_t getPathCount() const { return m_alphaValuesCount; }

		/** Alpha value for the discrete corresponding value \sa alpha2index */
		double index2alpha( uint16_t k ) const {
			if (k>=m_alphaValuesCount) throw std::runtime_error("PTG: alpha value out of range!");
			return M_PI * (-1.0 + 2.0 * (k+0.5)/m_alphaValuesCount );
		}

		/** Discrete index value for the corresponding alpha value \sa index2alpha */
		uint16_t alpha2index( double alpha ) const;

		inline double getRefDistance() const { return refDistance; }

		/** Resizes and populates the initial appropriate contents in a vector of tp-obstacles (collision-free ranges, in "pseudometers", un-normalized). \sa updateTPObstacle()  */
		void initTPObstacles(std::vector<double> &TP_Obstacles) const;

		/** When used in path planning, a multiplying factor (default=1.0) for the scores for this PTG. Assign values <1 to PTGs with low priority. */
		double getScorePriority() const { return m_score_priority; }
		void setScorePriorty(double prior) { m_score_priority = prior; }

		/** Returns the representation of one trajectory of this PTG as a 3D OpenGL object (a simple curved line).
		  * \param[in] k The 0-based index of the selected trajectory (discrete "alpha" parameter).
		  * \param[out] gl_obj Output object.
		  * \param[in] decimate_distance Minimum distance between path points (in meters).
		  * \param[in] max_path_distance If >0, cut the path at this distance (in meters).
		  */
		virtual void renderPathAsSimpleLine(const uint16_t k,mrpt::opengl::CSetOfLines &gl_obj,const float decimate_distance = 0.1f,const float max_path_distance = 0.0f) const;

		/** Dump PTG trajectories in four text files: `./reactivenav.logs/PTGs/PTG%i_{x,y,phi,d}.txt`
		  * Text files are loadable from MATLAB/Octave, and can be visualized with the script `[MRPT_DIR]/scripts/viewPTG.m` 
		  * \note The directory "./reactivenav.logs/PTGs" will be created if doesn't exist.
		  * \return false on any error writing to disk.
		  * \sa OUTPUT_DEBUG_PATH_PREFIX
		  */
		bool debugDumpInFiles(const std::string &ptg_name) const;

		/** Parameters accepted by this base class:
		 *   - `${sKeyPrefix}num_paths`: The number of different paths in this family (number of discrete `alpha` values).
		 *   - `${sKeyPrefix}ref_distance`: The maximum distance in PTGs [meters]
		 *   - `${sKeyPrefix}score_priority`: When used in path planning, a multiplying factor (default=1.0) for the scores for this PTG. Assign values <1 to PTGs with low priority.
		 */
		virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) MRPT_OVERRIDE;
		virtual void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const MRPT_OVERRIDE;


		/** Auxiliary function for rendering */
		virtual void add_robotShape_to_setOfLines(mrpt::opengl::CSetOfLines &gl_shape, const mrpt::poses::CPose2D &origin = mrpt::poses::CPose2D ()) const  = 0;

protected:
		double    refDistance;
		uint16_t  m_alphaValuesCount; //!< The number of discrete values for "alpha" between -PI and +PI.
		double    m_score_priority;

		bool      m_is_initialized;

		virtual void internal_readFromStream(mrpt::utils::CStream &in);
		virtual void internal_writeToStream(mrpt::utils::CStream &out) const;

	}; // end of class
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CParameterizedTrajectoryGenerator, mrpt::utils::CSerializable, NAV_IMPEXP )


	typedef std::vector<mrpt::nav::CParameterizedTrajectoryGenerator*>  TListPTGs;      //!< A list of PTGs (bare pointers)
	typedef std::vector<mrpt::nav::CParameterizedTrajectoryGeneratorPtr>  TListPTGPtr;  //!< A list of PTGs (smart pointers)


	/** Base class for all PTGs using a 2D polygonal robot shape model.
	 *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP CPTG_RobotShape_Polygonal : public CParameterizedTrajectoryGenerator
	{
	public:
		CPTG_RobotShape_Polygonal() : m_robotShape() {}
		virtual ~CPTG_RobotShape_Polygonal() {}

		/** @name Robot shape
		  * @{ **/
		/** Robot shape must be set before initialization, either from ctor params or via this method. */
		void setRobotShape(const mrpt::math::CPolygon & robotShape) {
			m_robotShape = robotShape;
			internal_processNewRobotShape();
		}
		const mrpt::math::CPolygon & getRobotShape() const { return m_robotShape; }
		/** @} */

		void add_robotShape_to_setOfLines(mrpt::opengl::CSetOfLines &gl_shape, const mrpt::poses::CPose2D &origin = mrpt::poses::CPose2D ()) const  MRPT_OVERRIDE;
	protected:
		virtual void internal_processNewRobotShape() = 0; //!< Will be called whenever the robot shape is set / updated
		mrpt::math::CPolygon m_robotShape;
		void loadShapeFromConfigFile(const mrpt::utils::CConfigFileBase & source,const std::string & section);
		void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const;
		void internal_shape_loadFromStream(mrpt::utils::CStream &in);
		void internal_shape_saveToStream(mrpt::utils::CStream &out) const;
		/** Loads a set of default parameters; provided  exclusively for the PTG-configurator tool. */
		void loadDefaultParams();
	};

	/** Base class for all PTGs using a 2D circular robot shape model.
	 *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP CPTG_RobotShape_Circular : public CParameterizedTrajectoryGenerator
	{
	public:
		CPTG_RobotShape_Circular() : m_robotRadius(.0) {}
		virtual ~CPTG_RobotShape_Circular() {}

		/** @name Robot shape
		  * @{ **/
		/** Robot shape must be set before initialization, either from ctor params or via this method. */
		void setRobotShapeRadius(const double robot_radius) {
			m_robotRadius = robot_radius;
			internal_processNewRobotShape();
		}
		double getRobotShapeRadius() const { return m_robotRadius; }
		/** @} */
		void add_robotShape_to_setOfLines(mrpt::opengl::CSetOfLines &gl_shape, const mrpt::poses::CPose2D &origin = mrpt::poses::CPose2D ()) const  MRPT_OVERRIDE;
	protected:
		virtual void internal_processNewRobotShape() = 0; //!< Will be called whenever the robot shape is set / updated
		double m_robotRadius;
		void loadShapeFromConfigFile(const mrpt::utils::CConfigFileBase & source,const std::string & section);
		void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const;
		void internal_shape_loadFromStream(mrpt::utils::CStream &in);
		void internal_shape_saveToStream(mrpt::utils::CStream &out) const;
		/** Loads a set of default parameters; provided  exclusively for the PTG-configurator tool. */
		void loadDefaultParams();
	};

	void NAV_IMPEXP registerAllNavigationClasses();
}
}

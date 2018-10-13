/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/wrap2pi.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/core/round.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/math/CPolygon.h>
#include <cstdint>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>

namespace mrpt
{
namespace opengl
{
class CSetOfLines;
}
}  // namespace mrpt

namespace mrpt
{
namespace nav
{
/** Defines behaviors for where there is an obstacle *inside* the robot shape
 *right at the beginning of a PTG trajectory.
 *\ingroup nav_tpspace
 * \sa Used in CParameterizedTrajectoryGenerator::COLLISION_BEHAVIOR
 */
enum PTG_collision_behavior_t
{
	/** Favor getting back from too-close (almost collision) obstacles. */
	COLL_BEH_BACK_AWAY = 0,
	/** Totally dissallow any movement if there is any too-close (almost
	   collision) obstacles. */
	COLL_BEH_STOP
};

/** \defgroup nav_tpspace TP-Space and PTG classes
 * \ingroup mrpt_nav_grp
 */

/** This is the base class for any user-defined PTG.
 *  There is a class factory interface in
 *CParameterizedTrajectoryGenerator::CreatePTG.
 *
 * Papers:
 *  - J.L. Blanco, J. Gonzalez-Jimenez, J.A. Fernandez-Madrigal, "Extending
 *Obstacle Avoidance Methods through Multiple Parameter-Space Transformations",
 *Autonomous Robots, vol. 24, no. 1, 2008.
 *http://ingmec.ual.es/~jlblanco/papers/blanco2008eoa_DRAFT.pdf
 *
 * Changes history:
 *	- 30/JUN/2004: Creation (JLBC)
 *	- 16/SEP/2004: Totally redesigned.
 *	- 15/SEP/2005: Totally rewritten again, for integration into MRPT
 *Applications Repository.
 *	- 19/JUL/2009: Simplified to use only STL data types, and created the class
 *factory interface.
 *	- MAY/2016: Refactored into CParameterizedTrajectoryGenerator,
 *CPTG_DiffDrive_CollisionGridBased, PTG classes renamed.
 *	- 2016-2018: Many features added to support "PTG continuation", dynamic
 *paths depending on vehicle speeds, etc.
 *
 *  \ingroup nav_tpspace
 */
class CParameterizedTrajectoryGenerator
	: public mrpt::serialization::CSerializable,
	  public mrpt::config::CLoadableOptions
{
	DEFINE_VIRTUAL_SERIALIZABLE(CParameterizedTrajectoryGenerator)
   public:
	/** Default ctor. Must call `loadFromConfigFile()` before initialization */
	CParameterizedTrajectoryGenerator();
	/**  Destructor  */
	~CParameterizedTrajectoryGenerator() override = default;
	/** The class factory for creating a PTG from a list of parameters in a
	 *section of a given config file (physical file or in memory).
	 *  Possible parameters are:
	 *	  - Those explained in
	 *CParameterizedTrajectoryGenerator::loadFromConfigFile()
	 *	  - Those explained in the specific PTG being created (see list of
	 *derived classes)
	 *
	 * `ptgClassName` can be any PTG class name which has been registered as
	 *any other
	 * mrpt::serialization::CSerializable class.
	 *
	 * \exception std::logic_error On invalid or missing parameters.
	 */
	static CParameterizedTrajectoryGenerator* CreatePTG(
		const std::string& ptgClassName,
		const mrpt::config::CConfigFileBase& cfg, const std::string& sSection,
		const std::string& sKeyPrefix);

	/** @name Virtual interface of each PTG implementation
	 *  @{ */
	/** Gets a short textual description of the PTG and its parameters */
	virtual std::string getDescription() const = 0;

   protected:
	/** Must be called after setting all PTG parameters and before requesting
	 * converting obstacles to TP-Space, inverseMap_WS2TP(), etc. */
	virtual void internal_initialize(
		const std::string& cacheFilename = std::string(),
		const bool verbose = true) = 0;
	/** This must be called to de-initialize the PTG if some parameter is to be
	 * changed. After changing it, call initialize again */
	virtual void internal_deinitialize() = 0;

   public:
	/** Computes the closest (alpha,d) TP coordinates of the trajectory point
	 * closest to the Workspace (WS)
	 *   Cartesian coordinates (x,y), relative to the current robot frame.
	 * \param[in] x X coordinate of the query point, relative to the robot
	 * frame.
	 * \param[in] y Y coordinate of the query point, relative to the robot
	 * frame.
	 * \param[out] out_k Trajectory parameter index (discretized alpha value,
	 * 0-based index).
	 * \param[out] out_d Trajectory distance, normalized such that D_max
	 * becomes 1.
	 *
	 * \return true if the distance between (x,y) and the actual trajectory
	 * point is below the given tolerance (in meters).
	 */
	virtual bool inverseMap_WS2TP(
		double x, double y, int& out_k, double& out_normalized_d,
		double tolerance_dist = 0.10) const = 0;

	/** Returns the same than inverseMap_WS2TP() but without any additional
	 * cost. The default implementation
	 * just calls inverseMap_WS2TP() and discards (k,d). */
	virtual bool PTG_IsIntoDomain(double x, double y) const
	{
		int k;
		double d;
		return inverseMap_WS2TP(x, y, k, d);
	}

	/** Returns true if a given TP-Space point maps to a unique point in
	 * Workspace, and viceversa. Default implementation returns `true`. */
	virtual bool isBijectiveAt(uint16_t k, uint32_t step) const { return true; }
	/** Converts a discretized "alpha" value into a feasible motion command or
	 * action. See derived classes for the meaning of these actions */
	virtual mrpt::kinematics::CVehicleVelCmd::Ptr directionToMotionCommand(
		uint16_t k) const = 0;

	/** Returns an empty kinematic velocity command object of the type supported
	 * by this PTG.
	 * Can be queried to determine the expected kinematic interface of the PTG.
	 */
	virtual mrpt::kinematics::CVehicleVelCmd::Ptr
		getSupportedKinematicVelocityCommand() const = 0;

	/** Dynamic state that may affect the PTG path parameterization. \ingroup
	 * nav_reactive  */
	struct TNavDynamicState
	{
		/** Current vehicle velocity (local frame of reference) */
		mrpt::math::TTwist2D curVelLocal;
		/** Current relative target location */
		mrpt::math::TPose2D relTarget;
		/** Desired relative speed [0,1] at target. Default=0 */
		double targetRelSpeed{0};

		TNavDynamicState();
		bool operator==(const TNavDynamicState& o) const;
		inline bool operator!=(const TNavDynamicState& o) const
		{
			return !(*this == o);
		}
		void writeToStream(mrpt::serialization::CArchive& out) const;
		void readFromStream(mrpt::serialization::CArchive& in);
	};

   protected:
	/** Invoked when `m_nav_dyn_state` has changed; gives the PTG the
	 * opportunity to react and parameterize paths depending on the
	 * instantaneous conditions */
	virtual void onNewNavDynamicState() = 0;

   public:
	virtual void setRefDistance(const double refDist) { refDistance = refDist; }
	/** Access path `k` ([0,N-1]=>[-pi,pi] in alpha): number of discrete "steps"
	 * along the trajectory.
	 * May be actual steps from a numerical integration or an arbitrary small
	 * length for analytical PTGs.
	 * \sa getAlphaValuesCount() */
	virtual size_t getPathStepCount(uint16_t k) const = 0;

	/** Access path `k` ([0,N-1]=>[-pi,pi] in alpha): pose of the vehicle at
	 * discrete step `step`.
	 * \sa getPathStepCount(), getAlphaValuesCount() */
	virtual void getPathPose(
		uint16_t k, uint32_t step, mrpt::math::TPose2D& p) const = 0;

	/** Access path `k` ([0,N-1]=>[-pi,pi] in alpha): traversed distance at
	 * discrete step `step`.
	 * \return Distance in pseudometers (real distance, NOT normalized to [0,1]
	 * for [0,refDist])
	 * \sa getPathStepCount(), getAlphaValuesCount() */
	virtual double getPathDist(uint16_t k, uint32_t step) const = 0;

	/** Returns the duration (in seconds) of each "step"
	 * \sa getPathStepCount() */
	virtual double getPathStepDuration() const = 0;

	/** Returns the maximum linear velocity expected from this PTG [m/s] */
	virtual double getMaxLinVel() const = 0;
	/** Returns the maximum angular velocity expected from this PTG [rad/s] */
	virtual double getMaxAngVel() const = 0;

	/** Access path `k` ([0,N-1]=>[-pi,pi] in alpha): largest step count for
	 * which the traversed distance is < `dist`
	 * \param[in] dist Distance in pseudometers (real distance, NOT normalized
	 * to [0,1] for [0,refDist])
	 * \return false if no step fulfills the condition for the given trajectory
	 * `k` (e.g. out of reference distance).
	 * Note that, anyway, the maximum distance (closest point) is returned in
	 * `out_step`.
	 * \sa getPathStepCount(), getAlphaValuesCount() */
	virtual bool getPathStepForDist(
		uint16_t k, double dist, uint32_t& out_step) const = 0;

	/** Updates the radial map of closest TP-Obstacles given a single obstacle
	 * point at (ox,oy)
	 * \param [in,out] tp_obstacles A vector of length `getAlphaValuesCount()`,
	 * initialized with `initTPObstacles()` (collision-free ranges, in
	 * "pseudometers", un-normalized).
	 * \param [in] ox Obstacle point (X), relative coordinates wrt origin of
	 * the PTG.
	 * \param [in] oy Obstacle point (Y), relative coordinates wrt origin of
	 * the PTG.
	 * \note The length of tp_obstacles is not checked for efficiency since
	 * this method is potentially called thousands of times per
	 *  navigation timestap, so it is left to the user responsibility to
	 * provide a valid buffer.
	 * \note `tp_obstacles` must be initialized with initTPObstacle() before
	 * call.
	 */
	virtual void updateTPObstacle(
		double ox, double oy, std::vector<double>& tp_obstacles) const = 0;

	/** Like updateTPObstacle() but for one direction only (`k`) in TP-Space.
	 * `tp_obstacle_k` must be initialized with initTPObstacleSingle() before
	 * call (collision-free ranges, in "pseudometers", un-normalized). */
	virtual void updateTPObstacleSingle(
		double ox, double oy, uint16_t k, double& tp_obstacle_k) const = 0;

	/** Loads a set of default parameters into the PTG. Users normally will call
	 * `loadFromConfigFile()` instead, this method is provided
	 * exclusively for the PTG-configurator tool. */
	virtual void loadDefaultParams();

	/** Returns true if it is possible to stop sending velocity commands to the
	 * robot and, still, the
	 * robot controller will be able to keep following the last sent trajectory
	 * ("NOP" velocity commands).
	 * Default implementation returns "false". */
	virtual bool supportVelCmdNOP() const;

	/** Returns true if this PTG takes into account the desired velocity at
	 * target. \sa updateNavDynamicState() */
	virtual bool supportSpeedAtTarget() const { return false; }
	/** Only for PTGs supporting supportVelCmdNOP(): this is the maximum time
	 * (in seconds) for which the path
	 * can be followed without re-issuing a new velcmd. Note that this is only
	 * an absolute maximum duration,
	 * navigation implementations will check for many other conditions. Default
	 * method in the base virtual class returns 0.
	 * \param path_k Queried path `k` index  [0,N-1] */
	virtual double maxTimeInVelCmdNOP(int path_k) const;

	/** Returns the actual distance (in meters) of the path, discounting
	 * possible circular loops of the path (e.g. if it comes back to the
	 * origin).
	 * Default: refDistance */
	virtual double getActualUnloopedPathLength(uint16_t k) const
	{
		return this->refDistance;
	}

	/** Query the PTG for the relative priority factor (0,1) of this PTG, in
	 * comparison to others, if the k-th path is to be selected. */
	virtual double evalPathRelativePriority(
		uint16_t k, double target_distance) const
	{
		return 1.0;
	}

	/** Returns an approximation of the robot radius. */
	virtual double getMaxRobotRadius() const = 0;
	/** Returns true if the point lies within the robot shape. */
	virtual bool isPointInsideRobotShape(
		const double x, const double y) const = 0;

	/** Evals the clearance from an obstacle (ox,oy) in coordinates relative to
	 * the robot center. Zero or negative means collision. */
	virtual double evalClearanceToRobotShape(
		const double ox, const double oy) const = 0;

	/** @} */  // --- end of virtual methods

	/** To be invoked by the navigator *before* each navigation step, to let the
	 * PTG to react to changing dynamic conditions. * \sa
	 * onNewNavDynamicState(), m_nav_dyn_state  */
	void updateNavDynamicState(
		const TNavDynamicState& newState, const bool force_update = false);
	const TNavDynamicState& getCurrentNavDynamicState() const
	{
		return m_nav_dyn_state;
	}

	/** The path used as defaul output in, for example, debugDumpInFiles.
	 * (Default="./reactivenav.logs/") */
	static std::string& OUTPUT_DEBUG_PATH_PREFIX();

	/** Must be called after setting all PTG parameters and before requesting
	 * converting obstacles to TP-Space, inverseMap_WS2TP(), etc. */
	void initialize(
		const std::string& cacheFilename = std::string(),
		const bool verbose = true);
	/** This must be called to de-initialize the PTG if some parameter is to be
	 * changed. After changing it, call initialize again */
	void deinitialize();
	/** Returns true if `initialize()` has been called and there was no errors,
	 * so the PTG is ready to be queried for paths, obstacles, etc. */
	bool isInitialized() const;

	/** Get the number of different, discrete paths in this family */
	uint16_t getAlphaValuesCount() const { return m_alphaValuesCount; }
	/** Get the number of different, discrete paths in this family */
	uint16_t getPathCount() const { return m_alphaValuesCount; }
	/** Alpha value for the discrete corresponding value \sa alpha2index */
	double index2alpha(uint16_t k) const;
	static double index2alpha(uint16_t k, const unsigned int num_paths);

	/** Discrete index value for the corresponding alpha value \sa index2alpha
	 */
	uint16_t alpha2index(double alpha) const;
	static uint16_t alpha2index(double alpha, const unsigned int num_paths);

	inline double getRefDistance() const { return refDistance; }
	/** Resizes and populates the initial appropriate contents in a vector of
	 * tp-obstacles (collision-free ranges, in "pseudometers", un-normalized).
	 * \sa updateTPObstacle()  */
	void initTPObstacles(std::vector<double>& TP_Obstacles) const;
	void initTPObstacleSingle(uint16_t k, double& TP_Obstacle_k) const;

	/** When used in path planning, a multiplying factor (default=1.0) for the
	 * scores for this PTG. Assign values <1 to PTGs with low priority. */
	double getScorePriority() const { return m_score_priority; }
	void setScorePriorty(double prior) { m_score_priority = prior; }
	unsigned getClearanceStepCount() const { return m_clearance_num_points; }
	void setClearanceStepCount(const unsigned res)
	{
		m_clearance_num_points = res;
	}

	unsigned getClearanceDecimatedPaths() const
	{
		return m_clearance_decimated_paths;
	}
	void setClearanceDecimatedPaths(const unsigned num)
	{
		m_clearance_decimated_paths = num;
	}

	/** Returns the representation of one trajectory of this PTG as a 3D OpenGL
	 * object (a simple curved line).
	 * \param[in] k The 0-based index of the selected trajectory (discrete
	 * "alpha" parameter).
	 * \param[out] gl_obj Output object.
	 * \param[in] decimate_distance Minimum distance between path points (in
	 * meters).
	 * \param[in] max_path_distance If >=0, cut the path at this distance (in
	 * meters).
	 */
	virtual void renderPathAsSimpleLine(
		const uint16_t k, mrpt::opengl::CSetOfLines& gl_obj,
		const double decimate_distance = 0.1,
		const double max_path_distance = -1.0) const;

	/** Dump PTG trajectories in four text files:
	 * `./reactivenav.logs/PTGs/PTG%i_{x,y,phi,d}.txt`
	 * Text files are loadable from MATLAB/Octave, and can be visualized with
	 * the script `[MRPT_DIR]/scripts/viewPTG.m`
	 * \note The directory "./reactivenav.logs/PTGs" will be created if doesn't
	 * exist.
	 * \return false on any error writing to disk.
	 * \sa OUTPUT_DEBUG_PATH_PREFIX
	 */
	bool debugDumpInFiles(const std::string& ptg_name) const;

	/** Parameters accepted by this base class:
	 *   - `${sKeyPrefix}num_paths`: The number of different paths in this
	 * family (number of discrete `alpha` values).
	 *   - `${sKeyPrefix}ref_distance`: The maximum distance in PTGs [meters]
	 *   - `${sKeyPrefix}score_priority`: When used in path planning, a
	 * multiplying factor (default=1.0) for the scores for this PTG. Assign
	 * values <1 to PTGs with low priority.
	 */
	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& cfg,
		const std::string& sSection) override;
	void saveToConfigFile(
		mrpt::config::CConfigFileBase& cfg,
		const std::string& sSection) const override;

	/** Auxiliary function for rendering */
	virtual void add_robotShape_to_setOfLines(
		mrpt::opengl::CSetOfLines& gl_shape,
		const mrpt::poses::CPose2D& origin = mrpt::poses::CPose2D()) const = 0;

	/** Defines the behavior when there is an obstacle *inside* the robot shape
	 * right at the beginning of a PTG trajectory.
	 * Default value: COLL_BEH_BACK_AWAY
	 */
	static PTG_collision_behavior_t& COLLISION_BEHAVIOR();

	/** Must be called to resize a CD to its correct size, before calling
	 * updateClearance() */
	void initClearanceDiagram(ClearanceDiagram& cd) const;

	/** Updates the clearance diagram given one (ox,oy) obstacle point, in
	 * coordinates relative
	 * to the PTG path origin.
	 * \param[in,out] cd The clearance will be updated here.
	 * \sa m_clearance_dist_resolution
	 */
	void updateClearance(
		const double ox, const double oy, ClearanceDiagram& cd) const;
	void updateClearancePost(
		ClearanceDiagram& cd, const std::vector<double>& TP_obstacles) const;

   protected:
	double refDistance{.0};
	/** The number of discrete values for "alpha" between -PI and +PI. */
	uint16_t m_alphaValuesCount{0};
	double m_score_priority{1.0};
	/** Number of steps for the piecewise-constant approximation of clearance
	 * from TPS distances [0,1] (Default=5) \sa updateClearance() */
	uint16_t m_clearance_num_points{5};
	/** Number of paths for the decimated paths analysis of clearance */
	uint16_t m_clearance_decimated_paths{15};
	/** Updated before each nav step by  */
	TNavDynamicState m_nav_dyn_state;
	/** Update in updateNavDynamicState(), contains the path index (k) for the
	 * target. */
	uint16_t m_nav_dyn_state_target_k;

	static const uint16_t INVALID_PTG_PATH_INDEX = static_cast<uint16_t>(-1);

	bool m_is_initialized{false};

	/** To be called by implementors of updateTPObstacle() and
	 * updateTPObstacleSingle() to
	 * honor the user settings regarding COLLISION_BEHAVIOR.
	 * \param new_tp_obs_dist The newly determiend collision-free ranges, in
	 * "pseudometers", un-normalized, for some "k" direction.
	 * \param inout_tp_obs The target where to store the new TP-Obs distance,
	 * if it fulfills the criteria determined by the collision behavior.
	 */
	void internal_TPObsDistancePostprocess(
		const double ox, const double oy, const double new_tp_obs_dist,
		double& inout_tp_obs) const;

	virtual void internal_readFromStream(mrpt::serialization::CArchive& in);
	virtual void internal_writeToStream(
		mrpt::serialization::CArchive& out) const;

   public:
	/** Evals the robot clearance for each robot pose along path `k`, for the
	 * real distances in
	 * the key of the map<>, then keep in the map value the minimum of its
	 * current stored clearance,
	 * or the computed clearance. In case of collision, clearance is zero.
	 * \param treat_as_obstacle true: normal use for obstacles; false: compute
	 * shortest distances to a target point (no collision)
	 */
	virtual void evalClearanceSingleObstacle(
		const double ox, const double oy, const uint16_t k,
		ClearanceDiagram::dist2clearance_t& inout_realdist2clearance,
		bool treat_as_obstacle = true) const;

};  // end of class

/** A list of PTGs (smart pointers) */
using TListPTGPtr =
	std::vector<mrpt::nav::CParameterizedTrajectoryGenerator::Ptr>;

/** Base class for all PTGs using a 2D polygonal robot shape model.
 *  \ingroup nav_tpspace
 */
class CPTG_RobotShape_Polygonal : public CParameterizedTrajectoryGenerator
{
   public:
	CPTG_RobotShape_Polygonal();
	~CPTG_RobotShape_Polygonal() override;

	/** @name Robot shape
	 * @{ **/
	/** Robot shape must be set before initialization, either from ctor params
	 * or via this method. */
	void setRobotShape(const mrpt::math::CPolygon& robotShape);
	const mrpt::math::CPolygon& getRobotShape() const { return m_robotShape; }
	double getMaxRobotRadius() const override;
	double evalClearanceToRobotShape(
		const double ox, const double oy) const override;
	/** @} */
	bool isPointInsideRobotShape(const double x, const double y) const override;
	void add_robotShape_to_setOfLines(
		mrpt::opengl::CSetOfLines& gl_shape,
		const mrpt::poses::CPose2D& origin =
			mrpt::poses::CPose2D()) const override;

   protected:
	/** Will be called whenever the robot shape is set / updated */
	virtual void internal_processNewRobotShape() = 0;
	mrpt::math::CPolygon m_robotShape;
	double m_robotMaxRadius{.01};
	void loadShapeFromConfigFile(
		const mrpt::config::CConfigFileBase& source,
		const std::string& section);
	void saveToConfigFile(
		mrpt::config::CConfigFileBase& cfg,
		const std::string& sSection) const override;
	void internal_shape_loadFromStream(mrpt::serialization::CArchive& in);
	void internal_shape_saveToStream(mrpt::serialization::CArchive& out) const;
	/** Loads a set of default parameters; provided  exclusively for the
	 * PTG-configurator tool. */
	void loadDefaultParams() override;
};

/** Base class for all PTGs using a 2D circular robot shape model.
 *  \ingroup nav_tpspace
 */
class CPTG_RobotShape_Circular : public CParameterizedTrajectoryGenerator
{
   public:
	CPTG_RobotShape_Circular();
	~CPTG_RobotShape_Circular() override;

	/** @name Robot shape
	 * @{ **/
	/** Robot shape must be set before initialization, either from ctor params
	 * or via this method. */
	void setRobotShapeRadius(const double robot_radius);
	double getRobotShapeRadius() const { return m_robotRadius; }
	double getMaxRobotRadius() const override;
	double evalClearanceToRobotShape(
		const double ox, const double oy) const override;
	/** @} */
	void add_robotShape_to_setOfLines(
		mrpt::opengl::CSetOfLines& gl_shape,
		const mrpt::poses::CPose2D& origin =
			mrpt::poses::CPose2D()) const override;
	bool isPointInsideRobotShape(const double x, const double y) const override;

   protected:
	/** Will be called whenever the robot shape is set / updated */
	virtual void internal_processNewRobotShape() = 0;
	double m_robotRadius{.0};
	void loadShapeFromConfigFile(
		const mrpt::config::CConfigFileBase& source,
		const std::string& section);
	void saveToConfigFile(
		mrpt::config::CConfigFileBase& cfg,
		const std::string& sSection) const override;
	void internal_shape_loadFromStream(mrpt::serialization::CArchive& in);
	void internal_shape_saveToStream(mrpt::serialization::CArchive& out) const;
	/** Loads a set of default parameters; provided  exclusively for the
	 * PTG-configurator tool. */
	void loadDefaultParams() override;
};
}  // namespace nav
}  // namespace mrpt

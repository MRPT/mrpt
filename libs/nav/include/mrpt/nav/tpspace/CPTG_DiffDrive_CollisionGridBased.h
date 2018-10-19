/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mrpt
{
namespace nav
{
/** \addtogroup nav_tpspace
 * @{  */

/** Trajectory points in C-Space for non-holonomic robots \sa
 * CPTG_DiffDrive_CollisionGridBased */
struct TCPoint
{
	TCPoint() = default;
	TCPoint(
		const float x_, const float y_, const float phi_, const float t_,
		const float dist_, const float v_, const float w_)
		: x(x_), y(y_), phi(phi_), t(t_), dist(dist_), v(v_), w(w_)
	{
	}
	float x, y, phi, t, dist, v, w;
};
using TCPointVector = std::vector<TCPoint>;
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& o, const mrpt::nav::TCPoint& p);
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& i, mrpt::nav::TCPoint& p);

/** Base class for all PTGs suitable to non-holonomic, differentially-driven (or
 * Ackermann) vehicles
 * based on numerical integration of the trajectories and collision
 * look-up-table.
 * Regarding `initialize()`: in this this family of PTGs, the method builds the
 * collision grid or load it from a cache file.
 * Collision grids must be calculated before calling getTPObstacle(). Robot
 * shape must be set before initializing with setRobotShape().
 * The rest of PTG parameters should have been set at the constructor.
 */
class CPTG_DiffDrive_CollisionGridBased : public CPTG_RobotShape_Polygonal
{
   public:
	/** The main method to be implemented in derived classes: it defines the
	 * differential-driven differential equation */
	virtual void ptgDiffDriveSteeringFunction(
		float alpha, float t, float x, float y, float phi, float& v,
		float& w) const = 0;

	/** @name Virtual interface of each PTG implementation
	 *  @{ */
	// getDescription(): remains to be defined in derived classes.
	// setParams() to be defined in derived classses.

	/** The default implementation in this class relies on a look-up-table.
	 * Derived classes may redefine this to closed-form expressions, when they
	 * exist.
	 * See full docs in base class
	 * CParameterizedTrajectoryGenerator::inverseMap_WS2TP() */
	bool inverseMap_WS2TP(
		double x, double y, int& out_k, double& out_d,
		double tolerance_dist = 0.10) const override;

	/** In this class, `out_action_cmd` contains: [0]: linear velocity (m/s),
	 * [1]: angular velocity (rad/s).
	 * See more docs in
	 * CParameterizedTrajectoryGenerator::directionToMotionCommand() */
	mrpt::kinematics::CVehicleVelCmd::Ptr directionToMotionCommand(
		uint16_t k) const override;
	mrpt::kinematics::CVehicleVelCmd::Ptr getSupportedKinematicVelocityCommand()
		const override;

	/** Launches an exception in this class: it is not allowed in numerical
	 * integration-based PTGs to change the reference distance
	 * after initialization. */
	void setRefDistance(const double refDist) override;

	// Access to PTG paths (see docs in base class)
	size_t getPathStepCount(uint16_t k) const override;
	void getPathPose(
		uint16_t k, uint32_t step, mrpt::math::TPose2D& p) const override;
	double getPathDist(uint16_t k, uint32_t step) const override;
	bool getPathStepForDist(
		uint16_t k, double dist, uint32_t& out_step) const override;
	double getPathStepDuration() const override;
	double getMaxLinVel() const override { return V_MAX; }
	double getMaxAngVel() const override { return W_MAX; }
	void updateTPObstacle(
		double ox, double oy, std::vector<double>& tp_obstacles) const override;
	void updateTPObstacleSingle(
		double ox, double oy, uint16_t k, double& tp_obstacle_k) const override;

	/** This family of PTGs ignores the dynamic states */
	void onNewNavDynamicState() override
	{
		// Do nothing.
	}

	/** @} */  // --- end of virtual methods

	double getMax_V() const { return V_MAX; }
	double getMax_W() const { return W_MAX; }

   protected:
	CPTG_DiffDrive_CollisionGridBased();

	void internal_processNewRobotShape() override;
	void internal_initialize(
		const std::string& cacheFilename = std::string(),
		const bool verbose = true) override;
	void internal_deinitialize() override;

	/** Possible values in "params" (those in CParameterizedTrajectoryGenerator,
	 * which is called internally, plus):
	 *   - `${sKeyPrefix}resolution`: The cell size
	 *   - `${sKeyPrefix}v_max`, ``${sKeyPrefix}w_max`: Maximum robot speeds.
	 *   - `${sKeyPrefix}shape_x{0,1,2..}`, ``${sKeyPrefix}shape_y{0,1,2..}`:
	 * Polygonal robot shape [Optional, can be also set via
	 * `setRobotPolygonShape()`]
	 *
	 * See docs of derived classes for additional parameters in setParams()
	 */
	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& cfg,
		const std::string& sSection) override;
	void saveToConfigFile(
		mrpt::config::CConfigFileBase& cfg,
		const std::string& sSection) const override;

	void loadDefaultParams() override;

	double V_MAX{.0}, W_MAX{.0};
	double turningRadiusReference{.10};
	std::vector<TCPointVector> m_trajectory;
	double m_resolution{0.05};
	double m_stepTimeDuration{0.01};

	void internal_readFromStream(mrpt::serialization::CArchive& in) override;
	void internal_writeToStream(
		mrpt::serialization::CArchive& out) const override;

	/** Numerically solve the diferential equations to generate a family of
	 * trajectories */
	void simulateTrajectories(
		float max_time, float max_dist, unsigned int max_n, float diferencial_t,
		float min_dist, float* out_max_acc_v = nullptr,
		float* out_max_acc_w = nullptr);

	/**  A list of all the pairs (alpha,distance) such as the robot collides at
	 *that cell.
	 *  - map key   (uint16_t) -> alpha value (k)
	 *	 - map value (float)    -> the MINIMUM distance (d), in meters,
	 *associated with that "k".
	 */
	using TCollisionCell = std::vector<std::pair<uint16_t, float>>;

	/** An internal class for storing the collision grid  */
	class CCollisionGrid : public mrpt::containers::CDynamicGrid<TCollisionCell>
	{
	   private:
		CPTG_DiffDrive_CollisionGridBased const* m_parent;

	   public:
		CCollisionGrid(
			float x_min, float x_max, float y_min, float y_max,
			float resolution, CPTG_DiffDrive_CollisionGridBased* parent)
			: mrpt::containers::CDynamicGrid<TCollisionCell>(
				  x_min, x_max, y_min, y_max, resolution),
			  m_parent(parent)
		{
		}
		~CCollisionGrid() override = default;
		/** Save to file, true = OK */
		bool saveToFile(
			mrpt::serialization::CArchive* fil,
			const mrpt::math::CPolygon& computed_robotShape) const;
		/** Load from file,  true = OK */
		bool loadFromFile(
			mrpt::serialization::CArchive* fil,
			const mrpt::math::CPolygon& current_robotShape);

		/** For an obstacle (x,y), returns a vector with all the pairs (a,d)
		 * such as the robot collides */
		const TCollisionCell& getTPObstacle(
			const float obsX, const float obsY) const;

		/** Updates the info into a cell: It updates the cell only if the
		 *distance d for the path k is lower than the previous value:
		 *	\param cellInfo The index of the cell
		 * \param k The path index (alpha discreet value)
		 * \param d The distance (in TP-Space, range 0..1) to collision.
		 */
		void updateCellInfo(
			const unsigned int icx, const unsigned int icy, const uint16_t k,
			const float dist);

	};  // end of class CCollisionGrid

	// Save/Load from files.
	bool saveColGridsToFile(
		const std::string& filename,
		const mrpt::math::CPolygon& computed_robotShape) const;  // true = OK
	bool loadColGridsFromFile(
		const std::string& filename,
		const mrpt::math::CPolygon& current_robotShape);  // true = OK

	/** The collision grid */
	CCollisionGrid m_collisionGrid;

	/** Specifies the min/max values for "k" and "n", respectively.
	 * \sa m_lambdaFunctionOptimizer
	 */
	struct TCellForLambdaFunction
	{
		TCellForLambdaFunction()
			: k_min(std::numeric_limits<uint16_t>::max()),
			  k_max(std::numeric_limits<uint16_t>::min()),
			  n_min(std::numeric_limits<uint32_t>::max()),
			  n_max(std::numeric_limits<uint32_t>::min())
		{
		}

		uint16_t k_min, k_max;
		uint32_t n_min, n_max;

		bool isEmpty() const
		{
			return k_min == std::numeric_limits<uint16_t>::max();
		}
	};

	/** This grid will contain indexes data for speeding-up the default,
	 * brute-force lambda function */
	mrpt::containers::CDynamicGrid<TCellForLambdaFunction>
		m_lambdaFunctionOptimizer;
};

/** @} */
}  // namespace nav
namespace typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NAMESPACE(TCPoint, mrpt::nav)
}  // namespace typemeta
}  // namespace mrpt

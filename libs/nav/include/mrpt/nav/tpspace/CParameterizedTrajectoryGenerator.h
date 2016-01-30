/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CParameterizedTrajectoryGenerator_H
#define CParameterizedTrajectoryGenerator_H

#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/TParameters.h>
#include <mrpt/nav/link_pragmas.h>
#include <mrpt/utils/mrpt_stdint.h>    // compiler-independent version of "stdint.h"
#include <limits>  // numeric_limits

namespace mrpt
{
  namespace opengl { class CSetOfLines; }

  namespace nav
  {
	/** \defgroup nav_tpspace TP-Space and PTG classes
	  * \ingroup mrpt_nav_grp
	  */

	/** Trajectory points in C-Space \sa CParameterizedTrajectoryGenerator */
	struct NAV_IMPEXP TCPoint
	{
		TCPoint() {}
		TCPoint(const float	x_,const float	y_,const float	phi_,
				const float	t_,const float	dist_,
				const float	v_,const float	w_) :
			x(x_), y(y_), phi(phi_), t(t_), dist(dist_), v(v_), w(w_)
		{}
		float x, y, phi,t, dist,v,w;
	};
	typedef std::vector<TCPoint> TCPointVector;

	/** This is the base class for any user-defined PTG.
	 *   The class factory interface in CParameterizedTrajectoryGenerator::CreatePTG.
	 *
	 * Papers:
	 *  - J.L. Blanco, J. Gonzalez-Jimenez, J.A. Fernandez-Madrigal, "Extending Obstacle Avoidance Methods through Multiple Parameter-Space Transformations", Autonomous Robots, vol. 24, no. 1, 2008. http://ingmec.ual.es/~jlblanco/papers/blanco2008eoa_DRAFT.pdf
	 *
	 * Changes history:
	 *		- 30/JUN/2004: Creation (JLBC)
	 *		- 16/SEP/2004: Totally redesigned.
	 *		- 15/SEP/2005: Totally rewritten again, for integration into MRPT Applications Repository.
	 *		- 19/JUL/2009: Simplified to use only STL data types, and created the class factory interface.
	 *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP  CParameterizedTrajectoryGenerator
	{
	public:
		MRPT_MAKE_ALIGNED_OPERATOR_NEW
	protected:
        /** Constructor: possible values in "params":
		 *   - ref_distance: The maximum distance in PTGs
		 *   - resolution: The cell size
		 *   - v_max, w_max: Maximum robot speeds.
		 *
		 * See docs of derived classes for additional parameters:
		 */
		CParameterizedTrajectoryGenerator(const mrpt::utils::TParameters<double> &params);

		/** Initialized the collision grid with the given size and resolution. */
		void initializeCollisionsGrid(float refDistance,float resolution);

	public:
		/** The class factory for creating a PTG from a list of parameters "params".
		  *  Possible values in "params" are:
		  *	  - "PTG_type": It's an integer number such as "1" -> CPTG1, "2"-> CPTG2, etc...
		  *	  - Those explained in CParameterizedTrajectoryGenerator::CParameterizedTrajectoryGenerator
		  *	  - Those explained in the specific PTG being created (e.g. CPTG1, CPTG2, etc...)
		  *
		  * \exception std::logic_error On invalid or missing parameters.
		  */
		static CParameterizedTrajectoryGenerator * CreatePTG(const mrpt::utils::TParameters<double> &params);

		/** Gets a short textual description of the PTG and its parameters */
		virtual std::string getDescription() const = 0 ;

		/** Destructor */
		virtual ~CParameterizedTrajectoryGenerator() {  }

		/** The main method: solves the diferential equation to generate a family of parametrical trajectories.
			*/
		void simulateTrajectories(
				uint16_t	    alphaValuesCount,
				float			max_time,
				float			max_dist,
				unsigned int	max_n,
				float			diferencial_t,
				float			min_dist,
				float			*out_max_acc_v = NULL,
				float			*out_max_acc_w = NULL);

		/** Saves the simulated trajectories and other parameters to a target stream */
		void saveTrajectories( mrpt::utils::CStream &out ) const;
		/** Loads the simulated trajectories and other parameters from a target stream. \return The PTG textual description */
		virtual std::string loadTrajectories( mrpt::utils::CStream &in );

		/** Computes the closest (alpha,d) TP coordinates of the trajectory point closest to the Workspace (WS) Cartesian coordinates (x,y).
		  * \param[in] x X coordinate of the query point.
		  * \param[in] y Y coordinate of the query point.
		  * \param[out] out_k Trajectory parameter index (discretized alpha value, 0-based index).
		  * \param[out] out_d Trajectory distance, normalized such that D_max becomes 1.
		  *
		  * \return true if the distance between (x,y) and the actual trajectory point is below the given tolerance (in meters).
		  * \note The default implementation in CParameterizedTrajectoryGenerator relies on a look-up-table. Derived classes may redefine this to closed-form expressions, when they exist.
		  */
		virtual bool inverseMap_WS2TP(float x, float y, int &out_k, float &out_d, float tolerance_dist = 0.10f) const;

		/** The "lambda" function, see paper for info. It takes the (a,d) pair that is closest to a given location. */
		MRPT_DECLARE_DEPRECATED_FUNCTION( \
			"Use inverseMap_WS2TP() instead", \
			void lambdaFunction( float x, float y, int &out_k, float &out_d ) \
		);

		/** Converts an "alpha" value (into the discrete set) into a feasible motion command.
			*/
		void directionToMotionCommand( uint16_t k, float &out_v, float &out_w );

		uint16_t getAlfaValuesCount() const { return m_alphaValuesCount; };
		size_t getPointsCountInCPath_k(uint16_t k)  const { return CPoints[k].size(); };

		/** Returns the C-Space coordinates (pose) when the robot has transversed a distance \a d along trajectory index \k. Returns (0,0,0) if out of bounds. */
		void   getCPointWhen_d_Is ( float d, uint16_t k, float &x, float &y, float &phi, float &t, float *v = NULL, float *w = NULL );

		float  GetCPathPoint_x( uint16_t k, int n ) const { return CPoints[k][n].x; }
		float  GetCPathPoint_y( uint16_t k, int n ) const { return CPoints[k][n].y; }
		float  GetCPathPoint_phi(uint16_t k, int n ) const { return CPoints[k][n].phi; }
		float  GetCPathPoint_t( uint16_t k, int n ) const { return CPoints[k][n].t; }
		float  GetCPathPoint_d( uint16_t k, int n ) const { return CPoints[k][n].dist; }
		float  GetCPathPoint_v( uint16_t k, int n ) const { return CPoints[k][n].v; }
		float  GetCPathPoint_w( uint16_t k, int n ) const { return CPoints[k][n].w; }

		float   getMax_V() const { return V_MAX; }
		float   getMax_W() const { return W_MAX; }
		float   getMax_V_inTPSpace() const { return maxV_inTPSpace; }

		/** Alfa value for the discrete corresponding value.
		 * \sa alpha2index
		 */
		float  index2alpha( uint16_t k ) const
		{
			return (float)(M_PI * (-1 + 2 * (k+0.5f) / ((float)m_alphaValuesCount) ));
		}

		/** Discrete index value for the corresponding alpha value.
		 * \sa index2alpha
		 */
		uint16_t  alpha2index( float alpha ) const
		{
			if (alpha>M_PI)  alpha-=(float)M_2PI;
			if (alpha<-M_PI) alpha+=(float)M_2PI;
			return (uint16_t)(0.5f*(m_alphaValuesCount*(1+alpha/M_PI) - 1));
		}

		/** Dump PTG trajectories in a binary file "./reactivenav.logs/PTGs/PTG%i.dat", with "%i" being the user-supplied parameter "nPT",
		  * and in FIVE text files: "./reactivenav.logs/PTGs/PTG%i_{x,y,phi,t,d}.txt".
		  *
		  * Text files are loadable from MATLAB/Octave, and can be visualized with the script [MRPT_DIR]/scripts/viewPTG.m ,
		  *  also online: http://mrpt.googlecode.com/svn/trunk/scripts/viewPTG.m
		  *
		  * \note The directory "./reactivenav.logs/PTGs" will be created if doesn't exist.
		  * \return false on any error writing to disk.
		  */
		bool debugDumpInFiles(const int nPT);

		/** Returns the representation of one trajectory of this PTG as a 3D OpenGL object (a simple curved line).
		  * \param[in] k The 0-based index of the selected trajectory (discrete "alpha" parameter).
		  * \param[out] gl_obj Output object.
		  * \param[in] decimate_distance Minimum distance between path points (in meters).
		  * \param[in] max_path_distance If >0, cut the path at this distance (in meters).
		  */
		void renderPathAsSimpleLine(
			const uint16_t k,
			mrpt::opengl::CSetOfLines &gl_obj,
			const float decimate_distance = 0.1f,
			const float max_path_distance = 0.0f) const;


		/**  A list of all the pairs (alpha,distance) such as the robot collides at that cell.
		  *  - map key   (uint16_t) -> alpha value (k)
		  *	 - map value (float)    -> the MINIMUM distance (d), in meters, associated with that "k".
		  */
		//typedef std::map<uint16_t,float> TCollisionCell;
		typedef std::vector<std::pair<uint16_t,float> > TCollisionCell;

		/** An internal class for storing the collision grid  */
		class NAV_IMPEXP CColisionGrid : public mrpt::utils::CDynamicGrid<TCollisionCell>
		{
		private:
				CParameterizedTrajectoryGenerator const * m_parent;

		public:
				CColisionGrid(float x_min, float x_max,float y_min, float y_max, float resolution, CParameterizedTrajectoryGenerator* parent )
				  : mrpt::utils::CDynamicGrid<TCollisionCell>(x_min,x_max,y_min,y_max,resolution),
				    m_parent(parent)
				{
				}
				virtual ~CColisionGrid() { }

				bool    saveToFile( mrpt::utils::CStream* fil, const mrpt::math::CPolygon & computed_robotShape );	//!< Save to file, true = OK
				bool    loadFromFile( mrpt::utils::CStream* fil, const mrpt::math::CPolygon & current_robotShape );	//!< Load from file,  true = OK

				/** For an obstacle (x,y), returns a vector with all the pairs (a,d) such as the robot collides.
					*/
				const TCollisionCell & getTPObstacle( const float obsX, const float obsY) const;

				/** Updates the info into a cell: It updates the cell only if the distance d for the path k is lower than the previous value:
				  *	\param cellInfo The index of the cell
				  * \param k The path index (alpha discreet value)
				  * \param d The distance (in TP-Space, range 0..1) to collision.
				  */
				void updateCellInfo( const unsigned int icx, const unsigned int icy, const uint16_t k, const float dist );

		}; // end of class CColisionGrid

		/** The collision grid */
		CColisionGrid	m_collisionGrid;

		// Save/Load from files.
		bool    SaveColGridsToFile( const std::string &filename, const mrpt::math::CPolygon & computed_robotShape );	// true = OK
		bool    LoadColGridsFromFile( const std::string &filename, const mrpt::math::CPolygon & current_robotShape ); // true = OK


		float	refDistance;

		/** The main method to be implemented in derived classes */
		virtual void PTG_Generator( float alpha, float t, float x, float y, float phi, float &v, float &w) = 0;

		/** To be implemented in derived classes */
		virtual bool PTG_IsIntoDomain( float x, float y ) = 0;

protected:
		/** Protected constructor for CPTG_Dummy; does not init collision grid. Not for normal usage */
		CParameterizedTrajectoryGenerator() : m_collisionGrid(-1,1,-1,1,0.5,this) { }

		float			V_MAX, W_MAX;
		float			turningRadiusReference;
		std::vector<TCPointVector>	CPoints;

		/** Specifies the min/max values for "k" and "n", respectively.
		  * \sa m_lambdaFunctionOptimizer
		  */
		struct TCellForLambdaFunction
		{
			TCellForLambdaFunction() :
				k_min( std::numeric_limits<uint16_t>::max() ),
				k_max( std::numeric_limits<uint16_t>::min() ),
				n_min( std::numeric_limits<uint32_t>::max() ),
				n_max( std::numeric_limits<uint32_t>::min() )
			{}

			uint16_t k_min,k_max;
			uint32_t n_min,n_max;

			bool isEmpty() const { return k_min==std::numeric_limits<uint16_t>::max();	}
		};

		/** This grid will contain indexes data for speeding-up the default, brute-force lambda function.
		  */
		mrpt::utils::CDynamicGrid<TCellForLambdaFunction>	m_lambdaFunctionOptimizer;

		// Computed from simulations while generating trajectories:
		float	maxV_inTPSpace;

		/** The number of discrete values for "alpha" between -PI and +PI.
		  */
		uint16_t  m_alphaValuesCount;

		/** Free all the memory buffers */
		void    FreeMemory();

	}; // end of class
	typedef stlplus::smart_ptr<CParameterizedTrajectoryGenerator> CParameterizedTrajectoryGeneratorPtr; //!< Smart pointer to a PTG

	typedef std::vector<mrpt::nav::CParameterizedTrajectoryGenerator*>  TListPTGs;      //!< A list of PTGs (bare pointers)
	typedef std::vector<mrpt::nav::CParameterizedTrajectoryGeneratorPtr>  TListPTGPtr;  //!< A list of PTGs (smart pointers)


	mrpt::utils::CStream NAV_IMPEXP & operator << (mrpt::utils::CStream& o, const mrpt::nav::TCPoint & p);
	mrpt::utils::CStream NAV_IMPEXP & operator >> (mrpt::utils::CStream& i, mrpt::nav::TCPoint & p);


	/** A dummy PTG, used mainly to call loadTrajectories() without knowing the exact derived PTG class and still be able to analyze the trajectories. */
	class NAV_IMPEXP  CPTG_Dummy : public CParameterizedTrajectoryGenerator
	{
	public:
		// See base class docs
		CPTG_Dummy() : CParameterizedTrajectoryGenerator() {}
		virtual ~CPTG_Dummy() { }
		virtual std::string getDescription() const { return m_text_description; }
		virtual std::string loadTrajectories( mrpt::utils::CStream &in ) 
		{
			m_text_description = CParameterizedTrajectoryGenerator::loadTrajectories(in);
			return m_text_description;
		}
		virtual void PTG_Generator( float alpha, float t, float x, float y, float phi, float &v, float &w) { throw std::runtime_error("Should not call this method in a dummy PTG!");  }
		virtual bool PTG_IsIntoDomain( float x, float y )  { throw std::runtime_error("Should not call this method in a dummy PTG!");  }

	private:
		std::string m_text_description;
	};

  }
	namespace utils
	{
		// Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME_NAMESPACE(TCPoint,mrpt::nav)
	}

}


#endif


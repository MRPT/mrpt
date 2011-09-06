/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CParameterizedTrajectoryGenerator_H
#define CParameterizedTrajectoryGenerator_H

#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/TParameters.h>
#include <mrpt/reactivenav/link_pragmas.h>

namespace mrpt
{
  namespace reactivenav
  {
	  using namespace mrpt::utils;

	/** This is the base class for any user defined PTG.
	 *   The class factory interface in CParameterizedTrajectoryGenerator::CreatePTG.
	 *
	 * Changes history:
	 *		- 30/JUN/2004: Creation (JLBC)
	 *		- 16/SEP/2004: Totally redesigned.
	 *		- 15/SEP/2005: Totally rewritten again, for integration into MRPT Applications Repository.
	 *		- 19/JUL/2009: Simplified to use only STL data types, and created the class factory interface.
	 *  \ingroup mrpt_reactivenav_grp
	 */
	class REACTIVENAV_IMPEXP  CParameterizedTrajectoryGenerator
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	protected:
        /** Constructor: possible values in "params":
		 *   - ref_distance: The maximum distance in PTGs
		 *   - resolution: The cell size
		 *   - v_max, w_max: Maximum robot speeds.
		 *   - system_TAU, system_DELAY (Optional): Robot dynamics
		 */
        CParameterizedTrajectoryGenerator(const TParameters<double> &params);

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
		static CParameterizedTrajectoryGenerator * CreatePTG(const TParameters<double> &params);

		/** Gets a short textual description of the PTG and its parameters.
		  */
		virtual std::string getDescription() const = 0 ;

        /** Destructor */
        virtual ~CParameterizedTrajectoryGenerator() {  }

        /** The main method: solves the diferential equation to generate a family of parametrical trajectories.
		 */
        void simulateTrajectories(
				uint16_t	alfaValuesCount,
				float			max_time,
				float			max_dist,
				unsigned int	max_n,
				float			diferencial_t,
				float			min_dist,
				float			*out_max_acc_v = NULL,
				float			*out_max_acc_w = NULL);

        /** The "lambda" function, see paper for info. It takes the (a,d) pair that is closest to a given location.
		 */
        virtual void lambdaFunction( float x, float y, int &out_k, float &out_d );

        /** Converts an "alfa" value (into the discrete set) into a feasible motion command.
		 */
        void directionToMotionCommand( uint16_t k, float &out_v, float &out_w );

        size_t getAlfaValuesCount() { return alfaValuesCount; };
        size_t getPointsCountInCPath_k(uint16_t k) { return CPoints[k].size(); };

        void   getCPointWhen_d_Is ( float d, uint16_t k, float &x, float &y, float &phi, float &t, float *v = NULL, float *w = NULL );

        float  GetCPathPoint_x( uint16_t k, int n ){ return CPoints[k][n].x; }
        float  GetCPathPoint_y( uint16_t k, int n ){ return CPoints[k][n].y; }
        float  GetCPathPoint_phi(uint16_t k, int n ){ return CPoints[k][n].phi; }
        float  GetCPathPoint_t( uint16_t k, int n ){ return CPoints[k][n].t; }
        float  GetCPathPoint_d( uint16_t k, int n ){ return CPoints[k][n].dist; }
        float  GetCPathPoint_v( uint16_t k, int n ){ return CPoints[k][n].v; }
        float  GetCPathPoint_w( uint16_t k, int n ){ return CPoints[k][n].w; }

        void    allocMemForVerticesData( int nVertices );

        void    setVertex_xy( uint16_t k, int n, int m, float x, float y )
        {
			vertexPoints_x[k][ n*nVertices + m ] = x;
            vertexPoints_y[k][ n*nVertices + m ] = y;
        }

        float  getVertex_x( uint16_t k, int n, int m )
        {
                int idx = n*nVertices + m;
//                assert( idx>=0);assert(idx<nVertices * nPointsInEachPath[k] );
				return vertexPoints_x[k][idx];
        }

        float  getVertex_y( uint16_t k, int n, int m )
        {
                int idx = n*nVertices + m;
//                assert( idx>=0);assert(idx<nVertices * nPointsInEachPath[k] );
				return vertexPoints_y[k][idx];
        }

		float*  getVertixesArray_x( uint16_t k, int n )
		{
                int idx = n*nVertices;
				return &vertexPoints_x[k][idx];
		}

		float*  getVertixesArray_y( uint16_t k, int n )
		{
                int idx = n*nVertices;
				return &vertexPoints_y[k][idx];
		}

		unsigned int getVertixesCount() { return nVertices; }

        float   getMax_V() { return V_MAX; }
        float   getMax_W() { return W_MAX; }
        float   getMax_V_inTPSpace() { return maxV_inTPSpace; }

		/** Alfa value for the discrete corresponding value.
		 * \sa alfa2index
		 */
		float  index2alfa( uint16_t k )
		{
			return (float)(M_PI * (-1 + 2 * (k+0.5f) / ((float)alfaValuesCount) ));
		}

		/** Discrete index value for the corresponding alfa value.
		 * \sa index2alfa
		 */
		uint16_t  alfa2index( float alfa )
		{
			if (alfa>M_PI)  alfa-=(float)M_2PI;
			if (alfa<-M_PI) alfa+=(float)M_2PI;
			return (uint16_t)(0.5f*(alfaValuesCount*(1+alfa/M_PI) - 1));
		}

		/** Dump PTG trajectories in files in directory "./PTGs/".
		 */
        void    debugDumpInFiles(int nPT);

		/**  A list of all the pairs (alpha,distance) such as the robot collides at that cell.
		  *  - map key   (uint16_t) -> alpha value (k)
		  *	 - map value (float)    -> the MINIMUM distance (d), in meters, associated with that "k".
		  */
		typedef std::map<uint16_t,float> TCollisionCell;

		/** An internal class for storing the collision grid  */
		class REACTIVENAV_IMPEXP CColisionGrid : public mrpt::utils::CDynamicGrid<TCollisionCell>
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

				bool    saveToFile( mrpt::utils::CStream* fil );	//!< Save to file, true = OK
                bool    loadFromFile( mrpt::utils::CStream* fil );	//!< Load from file,  true = OK

                /** For an obstacle (x,y), returns a vector with all the pairs (a,d) such as the robot collides.
				  */
				const TCollisionCell & getTPObstacle( const float obsX, const float obsY) const;

                /** Updates the info into a cell: It updates the cell only if the distance d for the path k is lower than the previous value:
				  *	\param cellInfo The index of the cell
				  * \param k The path index (alfa discreet value)
				  * \param d The distance (in TP-Space, range 0..1) to collision.
				  */
                void updateCellInfo( const unsigned int icx, const unsigned int icy, const uint16_t k, const float dist );
        };

		/** The collision grid */
		CColisionGrid	m_collisionGrid;

        // Save/Load from files.
		bool    SaveColGridsToFile( const std::string &filename );	// true = OK
        bool    LoadColGridsFromFile( const std::string &filename ); // true = OK


		float	refDistance;

		/** The main method to be implemented in derived classes.
		 */
        virtual void PTG_Generator( float alfa, float t, float x, float y, float phi, float &v, float &w) = 0;

		/** To be implemented in derived classes:
		  */
		virtual bool PTG_IsIntoDomain( float x, float y ) = 0;

protected:
		// Given a-priori:
        float			V_MAX, W_MAX;
		float			TAU, DELAY;

		float			turningRadiusReference;

		/** Specifies the min/max values for "k" and "n", respectively.
		  * \sa m_lambdaFunctionOptimizer
		  */
		struct TCellForLambdaFunction
		{
			TCellForLambdaFunction()
			{
				k_min=k_max=n_min=n_max=-1;
			}

			int		k_min,k_max,n_min,n_max;
		};

		/** This grid will contain indexes data for speeding-up the default, brute-force lambda function.
		  */
		CDynamicGrid<TCellForLambdaFunction>	m_lambdaFunctionOptimizer;

		// Computed from simulations while generating trajectories:
		float	maxV_inTPSpace;

        bool    flag1, flag2;

		/** The number of discrete values for ALFA between -PI and +PI.
		  */
        unsigned int     alfaValuesCount;

		/** The trajectories in the C-Space:
		  */
        struct TCPoint
        {
			TCPoint(	float	x,
						float	y,
						float	phi,
						float	t,
						float	dist,
						float	v,
						float	w)
			{
				this->x = x;
				this->y = y;
				this->phi = phi;
				this->t = t;
				this->dist = dist;
				this->v = v;
				this->w = w;
			};

			float  x, y, phi,t, dist,v,w;
        };
		typedef std::vector<TCPoint> TCPointVector;
		std::vector<TCPointVector>	CPoints;

		/** The shape of the robot along the trajectories:
		  */
		std::vector<vector_float> vertexPoints_x,vertexPoints_y;
        int     nVertices;

		/** Free all the memory buffers:
		  */
        void    FreeMemory();

	}; // end of class

	/** A type for lists of PTGs */
	typedef std::vector<mrpt::reactivenav::CParameterizedTrajectoryGenerator*>  TListPTGs;

  }
}


#endif


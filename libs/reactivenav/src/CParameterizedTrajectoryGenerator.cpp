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


#include <mrpt/reactivenav.h>  // Precomp header

//#if defined(_MSC_VER)
//	#pragma warning(disable:4267)
//#endif

#include <mrpt/utils/CStartUpClassesRegister.h>
extern mrpt::utils::CStartUpClassesRegister  mrpt_reactivenav_class_reg;
const int dumm = mrpt_reactivenav_class_reg.do_nothing(); // Avoid compiler removing this class in static linking


#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/geometry.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::reactivenav;
using namespace mrpt::system;
using namespace std;

/** Constructor: possible values in "params":
 *   - ref_distance: The maximum distance in PTGs
 *   - resolution: The cell size
 *   - v_max, w_max: Maximum robot speeds.
 *   - system_TAU, system_DELAY (Optional): Robot dynamics
 */
CParameterizedTrajectoryGenerator::CParameterizedTrajectoryGenerator(const TParameters<double> &params) :
	m_collisionGrid(-1,1,-1,1,0.5,this)
{
	this->refDistance	= params["ref_distance"];
	this->V_MAX			= params["v_max"];
	this->W_MAX			= params["w_max"];
	this->TAU			= params.has("system_TAU") ? params["system_TAU"] : 0;
	this->DELAY			= params.has("system_DELAY") ? params["system_DELAY"] : 0;

    alfaValuesCount=0;
    nVertices = 0;
	turningRadiusReference = 0.10f;

	initializeCollisionsGrid( refDistance, params["resolution"] );
}

/*---------------------------------------------------------------
					Class factory
  ---------------------------------------------------------------*/
CParameterizedTrajectoryGenerator * CParameterizedTrajectoryGenerator::CreatePTG(const TParameters<double> &params)
{
	MRPT_START
	const int nPTG = static_cast<int>( params["PTG_type"] );
	switch(nPTG)
	{
	case 1: return new CPTG1(params);
	case 2: return new CPTG2(params);
	case 3: return new CPTG3(params);
	case 4: return new CPTG4(params);
	case 5: return new CPTG5(params);
	case 6: return new CPTG6(params);
	case 7: return new CPTG7(params);
	default:
		THROW_EXCEPTION_CUSTOM_MSG1("Unknown PTG_type=%i",nPTG)
	};
	MRPT_END
}

/*---------------------------------------------------------------
					initializeCollisionsGrid
  ---------------------------------------------------------------*/
void CParameterizedTrajectoryGenerator::initializeCollisionsGrid(float refDistance,float resolution)
{
	m_collisionGrid.setSize( -refDistance,refDistance,-refDistance,refDistance,resolution );
}

/*---------------------------------------------------------------
					FreeMemory
  ---------------------------------------------------------------*/
void CParameterizedTrajectoryGenerator::FreeMemory()
{
	if (alfaValuesCount)
	{
		// Free trajectories:
		CPoints.clear();

		// And the shape of the robot along them:
		vertexPoints_x.clear();
		vertexPoints_y.clear();

		// Signal an empty PTG:
		alfaValuesCount = 0;
	}
}

/*---------------------------------------------------------------
					allocMemFoVerticesData
  ---------------------------------------------------------------*/
void CParameterizedTrajectoryGenerator::allocMemForVerticesData( int nVertices )
{
		vertexPoints_x.resize(alfaValuesCount);
		vertexPoints_y.resize(alfaValuesCount);

		// Alloc the exact number of items, all of them set to 0:
        for (unsigned int i=0;i<alfaValuesCount;i++)
		{
			vertexPoints_x[i].resize( nVertices * getPointsCountInCPath_k(i), 0 );
			vertexPoints_y[i].resize( nVertices * getPointsCountInCPath_k(i), 0 );
		}

		// Save it:
        this->nVertices= nVertices;
}

/*---------------------------------------------------------------
					simulateTrajectories
	Solve trajectories and fill cells.
  ---------------------------------------------------------------*/
void CParameterizedTrajectoryGenerator::simulateTrajectories(
        uint16_t	alfaValuesCount,
        float			max_time,
        float			max_dist,
        unsigned int	max_n,
        float			diferencial_t,
        float			min_dist,
        float			*out_max_acc_v,
        float			*out_max_acc_w)
{
        // Primero, liberar memoria:
        FreeMemory();

        // The number of discreet values for ALFA:
        this->alfaValuesCount = alfaValuesCount;

		// Reserve the size in the buffers:
		CPoints.resize( alfaValuesCount );

        // Calcular maxima distancia del contorno del robot (para 1 calculo auxiliar)
        float  radio_max_robot=1.0;    // Aprox.

        // Buffer auxiliar:
//        TCPoint         *points = (TCPoint*)calloc(max_n, sizeof(TCPoint));
//        int             nPoints;
		TCPointVector	points;

        float          alfa;
        float          x,y,phi,v,w,cmd_v,cmd_w,t, dist, girado;
        float          _x,_y,_phi;     // De la iteracion anteriormente guardada
        float          ult_dist, ult_dist1, ult_dist2;

		// For the grid:
		float		   x_min = 1e3f, x_max = -1e3;
		float		   y_min = 1e3f, y_max = -1e3;

		// Para averiguar las maximas ACELERACIONES lineales y angulares:
		float			max_acc_lin, max_acc_ang;
		float			acc_lin, acc_ang;

        int				k;

		maxV_inTPSpace = 0;
		max_acc_lin = max_acc_ang = 0;

	try
	{
        for (k=0;k<alfaValuesCount;k++)
        {
                // Simular / calcular trayectoria con "alfa":
                // --------------------------------------------------------
                alfa = index2alfa( k );

				points.clear();
                t = dist = girado = 0.0;
                x = y = phi = v = w =
                _x = _y = _phi = 0.0;
                flag1 = flag2 = false;

                // Ventana deslizante de ultimos comandos:
                vector<float>   last_cmd_vs, last_cmd_ws;
                vector<float>   last_vs, last_ws;
                int             M = 5;

				// cmd_v[i] = cmd_v[k-i]
				last_cmd_vs.clear();last_cmd_ws.clear();
                last_cmd_vs.resize(M,0);
                last_cmd_ws.resize(M,0);

				// cmd_v[i] = cmd_v[k-i]
                last_vs.clear();last_ws.clear();
				last_vs.resize(M,0);
                last_ws.resize(M,0);

                // ________________________________________
                // Parametros del sistema:
                //
                //            (1-alfa)·z(-NDELAY)
                //  H(z) = ------------------------
                //            1 -  z(-1)·alfa
                //
                //    alfa = exp( -1 / d ), d = TAU/T
                // ________________________________________
                int             N_Delay = round(DELAY / diferencial_t);
                if (TAU==0)	       TAU=0.01f;
                double          filter_alfa = exp(-1/(TAU/diferencial_t));

                // Add the first, initial point:
				points.push_back( TCPoint(	x,
											y,
											phi,
											t,
											dist,
											v,
											w
											) );
                // Simular hasta que se cumpla algo de esto:
                while ( t < max_time && dist < max_dist && points.size() < max_n && fabs(girado) < 1.95 * M_PI )
                {
						// Max. aceleraciones:
						if (t>1)
						{
							acc_lin = fabs( (last_vs[0]-last_vs[1])/diferencial_t);
							acc_ang = fabs( (last_ws[0]-last_ws[1])/diferencial_t);

							if (acc_lin>max_acc_lin)
									max_acc_lin = acc_lin;
							if (acc_ang>max_acc_ang)
									max_acc_ang =acc_ang;
						}

                        // Calcular nuevo comando de (v,w):
						PTG_Generator( alfa,t, x, y, phi, cmd_v,cmd_w );

                        if (t==0)
                                maxV_inTPSpace=max(maxV_inTPSpace,(float)( sqrt( square(cmd_v) + square(cmd_w*turningRadiusReference) ) ) );

                        // FILTRAR ----------------------------------
                        for (int i=M-1;i>=1;i--)
                        {
                                last_cmd_vs[i]=last_cmd_vs[i-1];
                                last_cmd_ws[i]=last_cmd_ws[i-1];
                                last_vs[i]=last_vs[i-1];
                                last_ws[i]=last_ws[i-1];
                        }
                        last_vs[0] = v;
                        last_ws[0] = w;
                        last_cmd_vs[0] = cmd_v;
                        last_cmd_ws[0] = cmd_w;

                        // Procesar respuesta del sistema para tener (v,w) "reales":
                        v = (float)(last_cmd_vs[ N_Delay ]*(1-filter_alfa) + filter_alfa*last_vs[1]);
                        w = (float)(last_cmd_ws[ N_Delay ]*(1-filter_alfa) + filter_alfa*last_ws[1]);

                        // -------------------------------------------

                        // Ecuacion en diferencias:
                        x += cos(phi)* v * diferencial_t;
                        y += sin(phi)* v * diferencial_t;
                        phi+= w * diferencial_t;

                        // Contadores:
                        girado += w * diferencial_t;

                        float v_inTPSpace = sqrt( square(v)+square(w*turningRadiusReference) );

                        dist += v_inTPSpace  * diferencial_t;

                        t += diferencial_t;

                        // Si nos hemos movido suficiente, guardar esta muestra:
                        ult_dist1 = sqrt( square( _x - x )+square( _y - y  ) );
                        ult_dist2 = fabs( radio_max_robot* ( _phi - phi ) );
                        ult_dist = max( ult_dist1, ult_dist2 );

                        if (ult_dist > min_dist)
                        {
							// Set the (v,w) to the last record:
							points.back().v = v;
							points.back().w = w;

							// And add the new record:
							points.push_back( TCPoint(	x,
														y,
														phi,
														t,
														dist,
														v,
														w
														) );

							// For the next iter:
                            _x = x;
                            _y = y;
                            _phi = phi;
                        }

					// for the grid:
					x_min = min(x_min,x); x_max = max(x_max,x);
					y_min = min(y_min,y); y_max = max(y_max,y);
                }

				// Add the final point:
				// -------------------------------------
				points.back().v = v;
				points.back().w = w;
				points.push_back( TCPoint(	x,
											y,
											phi,
											t,
											dist,
											v,
											w
											) );

                // Guardar datos en la estructura:
                // --------------------------------------------------------
                CPoints[k] = points;

        }       // for "k"

		// Poner las aceleraciones:
		if (out_max_acc_v) *out_max_acc_v = max_acc_lin;
        if (out_max_acc_w) *out_max_acc_w = max_acc_ang;

		// --------------------------------------------------------
		// Build the speeding-up grid for lambda function:
		// --------------------------------------------------------
		m_lambdaFunctionOptimizer.setSize(
			x_min-0.5f,x_max+0.5f,
			y_min-0.5f,y_max+0.5f,  0.25f);

		TCellForLambdaFunction	defCell;
		defCell.k_max = defCell.k_min = defCell.n_max = defCell.n_min = -1;
		m_lambdaFunctionOptimizer.fill( defCell );

		for (k=0;k<alfaValuesCount;k++)
		{
			for (int n=0;n<(int)CPoints[k].size();n++)
			{
				TCellForLambdaFunction	*cell = m_lambdaFunctionOptimizer.cellByPos(CPoints[k][n].x,CPoints[k][n].y);
				ASSERT_(cell);
				if (cell->k_max==-1)
				{
					// First time: copy
					cell->k_min =
					cell->k_max = k;
					cell->n_min =
					cell->n_max = n;
				}
				else
				{
					// Keep limits:
					cell->k_min = min( cell->k_min, k );
					cell->k_max = max( cell->k_max, k );
					cell->n_min = min( cell->n_min, n );
					cell->n_max = max( cell->n_max, n );
				}
			}
		}
	}
	catch(...)
	{
		std::cout << format("[CParameterizedTrajectoryGenerator::simulateTrajectories] Simulation aborted: unexpected exception!\n");
	}

}

/*---------------------------------------------------------------
					directionToMotionCommand
  ---------------------------------------------------------------*/
void CParameterizedTrajectoryGenerator::directionToMotionCommand( uint16_t k, float &v, float &w )
{
	PTG_Generator( index2alfa(k),0, 0, 0, 0, v, w );
}

/*---------------------------------------------------------------
					getCPointWhen_d_Is
  ---------------------------------------------------------------*/
void CParameterizedTrajectoryGenerator::getCPointWhen_d_Is (
		float		d,
		uint16_t k,
		float		&x,
		float		&y,
		float		&phi,
		float		&t,
                float *v,
                float *w)
{
        unsigned int     n=0;

        if (k>=alfaValuesCount)
		{
			x=y=phi=0;
			return;  // Por si acaso
		}

		while ( n < (CPoints[k].size()-1) && CPoints[k][n].dist<d )
                n++;

        x=CPoints[k][n].x;
        y=CPoints[k][n].y;
        phi=CPoints[k][n].phi;
        t=CPoints[k][n].t;
        if (v) *v =CPoints[k][n].v;
        if (w) *w =CPoints[k][n].w;
}

/*---------------------------------------------------------------
						debugDumpInFiles
  ---------------------------------------------------------------*/
void CParameterizedTrajectoryGenerator::debugDumpInFiles( int nPT )
{
	char str[100];

//#define alsoDumpForMATLAB

	mrpt::system::createDirectory( "./reactivenav.logs" );
	mrpt::system::createDirectory( "./reactivenav.logs/PTGs" );

	os::sprintf(str,100, "./reactivenav.logs/PTGs/PTG%u.dat",nPT);
	FILE* f = os::fopen(str,"wb");

#ifdef alsoDumpForMATLAB
	sprintf(str, "./reactivenav.logs/PTGs/PTG%u_x.txt",nPT); FILE* fx = fopen(str,"wt");
	sprintf(str, "./reactivenav.logs/PTGs/PTG%u_y.txt",nPT); FILE* fy = fopen(str,"wt");
	sprintf(str, "./reactivenav.logs/PTGs/PTG%u_p.txt",nPT); FILE* fp = fopen(str,"wt");
	sprintf(str, "./reactivenav.logs/PTGs/PTG%u_t.txt",nPT); FILE* ft = fopen(str,"wt");
	sprintf(str, "./reactivenav.logs/PTGs/PTG%u_d.txt",nPT); FILE* fd = fopen(str,"wt");
#endif

	int		nPaths = getAlfaValuesCount();
	//int    maxPoints=0;
	int    k;

#ifdef alsoDumpForMATLAB
	// Version texto:
	for (k=0;k<nPaths;k++)
		maxPoints = max( maxPoints, getPointsCountInCPath_k(k) );

	for ( k=0;k<nPaths;k++)
	{
		for (int n=0;n< maxPoints;n++)
		{
				int nn;

				nn= min( n, getPointsCountInCPath_k(k)-1 );

				fprintf(fx,"%0.02f ", GetCPathPoint_x(k,nn) );
				fprintf(fy,"%0.02f ", GetCPathPoint_y(k,nn) );
				fprintf(fp,"%0.02f ", GetCPathPoint_phi(k,nn) );
				fprintf(ft,"%0.02f ", GetCPathPoint_t(k,nn) );
				fprintf(fd,"%0.02f ", GetCPathPoint_d(k,nn) );
		}
		fprintf(fx,"\n" );fprintf(fy,"\n" );fprintf(fp,"\n" );fprintf(ft,"\n" );fprintf(fd,"\n" );
	}
	fclose(fx);fclose(fy);fclose(fp);fclose(ft);fclose(fd);
#endif

	size_t wr;

	// Version binaria:
	for ( k=0;k<nPaths;k++)
	{
		int     nPoints = getPointsCountInCPath_k(k);
		float   fl;
		wr=fwrite( &nPoints ,sizeof(int),1 , f ); ASSERT_(wr>0);
		for (int n=0;n<nPoints;n++)
		{
				fl = GetCPathPoint_x(k,n); wr=fwrite(&fl,sizeof(float),1,f);ASSERT_(wr>0);
				fl = GetCPathPoint_y(k,n); wr=fwrite(&fl,sizeof(float),1,f);ASSERT_(wr>0);
				fl = GetCPathPoint_phi(k,n); wr=fwrite(&fl,sizeof(float),1,f);ASSERT_(wr>0);
				fl = GetCPathPoint_t(k,n); wr=fwrite(&fl,sizeof(float),1,f);ASSERT_(wr>0);
				fl = GetCPathPoint_d(k,n); wr=fwrite(&fl,sizeof(float),1,f);ASSERT_(wr>0);
		}
	}

	os::fclose(f);
}

/*---------------------------------------------------------------
					getTPObstacle
  ---------------------------------------------------------------*/
const CParameterizedTrajectoryGenerator::TCollisionCell & CParameterizedTrajectoryGenerator::CColisionGrid::getTPObstacle(
	const float obsX, const float obsY) const
{
	static const TCollisionCell  emptyCell;
	const TCollisionCell *cell = cellByPos(obsX,obsY);
	return cell!=NULL ? *cell : emptyCell;
}

/*---------------------------------------------------------------
	Updates the info into a cell: It updates the cell only
	  if the distance d for the path k is lower than the previous value:
  ---------------------------------------------------------------*/
void CParameterizedTrajectoryGenerator::CColisionGrid::updateCellInfo(
	const unsigned int icx,
	const unsigned int icy,
	const uint16_t k,
	const float dist )
{
	TCollisionCell *cell = cellByIndex(icx,icy);
	if (!cell) return;

	TCollisionCell::iterator itK = cell->find(k);

	if (itK==cell->end())
	{	// New entry:
		(*cell)[k] = dist;
	}
	else
	{	// Only update that "k" if the distance is shorter now:
		if (dist<itK->second)
			itK->second = dist;
	}
}


/*---------------------------------------------------------------
					Save to file
  ---------------------------------------------------------------*/
bool CParameterizedTrajectoryGenerator::SaveColGridsToFile( const std::string &filename )
{
	try
	{
		CFileGZOutputStream   fo(filename);
		if (!fo.fileOpenCorrectly()) return false;

		const uint32_t n = 1; // for backwards compatibility...
		fo << n;
		return m_collisionGrid.saveToFile(&fo);
	}
	catch (...)
	{
		return false;
	}
}

/*---------------------------------------------------------------
					Load from file
  ---------------------------------------------------------------*/
bool CParameterizedTrajectoryGenerator::LoadColGridsFromFile( const std::string &filename )
{
	try
	{
		CFileGZInputStream   fi(filename);
		if (!fi.fileOpenCorrectly()) return false;

		uint32_t n;
		fi >> n;

		if (n!=1)
		{
			//std::cerr << format("[LoadColGridsFromFile] WARNING: n!=1 --> return false;\n");
			return false;
		}

		return m_collisionGrid.loadFromFile(&fi);
	}
	catch(...)
	{
		return false;
	}
}

const uint32_t COLGRID_FILE_MAGIC = 0xC0C0C0C0;

/*---------------------------------------------------------------
					Save to file
  ---------------------------------------------------------------*/
bool CParameterizedTrajectoryGenerator::CColisionGrid::saveToFile( CStream *f )
{
	try
	{
		if (!f) return false;

		*f << COLGRID_FILE_MAGIC;
		*f << m_parent->getDescription();

		*f << m_x_min << m_x_max << m_y_min << m_y_max;
		*f << m_resolution;
		*f << m_map;
		return true;
	}
	catch(...)
	{
		return false;
	}
}

/*---------------------------------------------------------------
						loadFromFile
  ---------------------------------------------------------------*/
bool CParameterizedTrajectoryGenerator::CColisionGrid::loadFromFile( CStream *f )
{
	try
	{
        if (!f) return false;

		// Return false if the file contents doesn't match what we expected:
		uint32_t file_magic;
		*f >> file_magic;

		if (COLGRID_FILE_MAGIC!=file_magic)
			return false;

		const std::string expected_desc = m_parent->getDescription();
		std::string desc;
		*f >> desc;

		if (desc!=expected_desc)
			return false;

        // Datos descriptivos de la rejilla:
        float	ff;
		*f >> ff; if(ff!=m_x_min) return false;
		*f >> ff; if(ff!=m_x_max) return false;
		*f >> ff; if(ff!=m_y_min) return false;
		*f >> ff; if(ff!=m_y_max) return false;
		*f >> ff; if(ff!=m_resolution) return false;

		*f >> m_map;
		return true;
	}
	catch(...)
	{
		return false;
	}
}


/*---------------------------------------------------------------
                lambdaFunction
  ---------------------------------------------------------------*/
void CParameterizedTrajectoryGenerator::lambdaFunction( float x, float y, int &k_out, float &d_out )
{
	// Esta en la zona donde las trayectorias son curvas:
	//   comparar con simulaciones

	// -------------------------------------------------------------------
	// Optimization: (24-JAN-2007 @ Jose Luis Blanco):
	//  Use a "grid" to determine the range of [k,d] values to check!!
	//  If the point (x,y) is not found in the grid, then directly skip
	//  to the next step.
	// -------------------------------------------------------------------
	int		k_min = 0;
	int		k_max = alfaValuesCount-1;
	int		n_min = 0;
	int		n_max = -1; // This is to force that, if no cell contains the area of interest in the first loop below, we skip straight to the next part.

	// Cell indexes:
	int		cx0 = m_lambdaFunctionOptimizer.x2idx(x);
	int		cy0 = m_lambdaFunctionOptimizer.y2idx(y);

	// (cx,cy)
	bool	firstCell = true;
	for (int cx=cx0-1;cx<=cx0+1;cx++)
	{
		for (int cy=cy0-1;cy<=cy0+1;cy++)
		{
			TCellForLambdaFunction	*cell = m_lambdaFunctionOptimizer.cellByIndex(cx,cy);
			if (cell)
			{
				if (cell->k_max!=-1)
				{
					if (firstCell)
					{
						k_min = cell->k_min;	k_max = cell->k_max;
						n_min = cell->n_min;	n_max = cell->n_max;
						firstCell = false;
					}
					else
					{
						k_min = min(cell->k_min,k_min);	k_max = max(cell->k_max,k_max);
						n_min = min(cell->n_min,n_min);	n_max = max(cell->n_max,n_max);
					}
				}
			}
		}
	}

	// Try to find a closest point to the paths:
	// ----------------------------------------------
	int     selected_k = -1;
	float	selected_d= 0;
	float   selected_dist = std::numeric_limits<float>::max();

	if (n_max>=n_min) // Otherwise, don't even lose time checking...
	{
		for (int k=k_min;k<=k_max;k++)
		{
			const int n_max_this = min( int(CPoints[k].size())-1, n_max);

			for (int n = n_min;n<=n_max_this; n++)
			{
				const float dist_a_punto= square( CPoints[k][n].x - x ) + square( CPoints[k][n].y - y );
				if (dist_a_punto<selected_dist)
				{
					selected_dist = dist_a_punto;
					selected_k = k;
					selected_d = CPoints[k][n].dist;
				}
			}
		}
	}

	if (selected_k!=-1)
	{
		k_out = selected_k;
		d_out = selected_d / refDistance;
		//cerr << "selected_d:" << selected_d << " refDistance:"<< refDistance << " d_out:" << d_out << "k_out:"<<k_out<<endl;
		return;
	}

	// If not found, compute an extrapolation!
	// ----------------------------------------------

	// ------------------------------------------------------------------------------------
	// Given a point (x,y), compute the "k_closest" whose extrapolation
	//  is closest to the point, and the associated "d_closest" distance,
	//  which can be normalized by "1/refDistance" to get TP-Space distances.
	// ------------------------------------------------------------------------------------
	selected_dist = std::numeric_limits<float>::max();
	for ( int k=0;k<static_cast<int>(alfaValuesCount);k++)
	{
		const int n = int (CPoints[k].size()) -1;
		const float dist_a_punto = square( CPoints[k][n].dist ) + square( CPoints[k][n].x - x ) + square( CPoints[k][n].y - y );

		if (dist_a_punto<selected_dist)
		{
			selected_dist = dist_a_punto;
			selected_k = k;
			selected_d = dist_a_punto;
		}
	}

	selected_d = std::sqrt(selected_d);

	k_out = selected_k;
	d_out = selected_d / refDistance;

	//cerr << "extrapol: selected_d:" << selected_d << " refDistance:"<< refDistance << " d_out:" << d_out << "k_out:"<<k_out<<endl;
}



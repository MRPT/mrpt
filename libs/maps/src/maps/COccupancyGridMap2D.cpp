/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/maps.h>  // Precompiled header



// Force size_x being a multiple of 16 cells
//#define		ROWSIZE_MULTIPLE_16

#include <mrpt/math/utils.h>
#include <mrpt/random.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/opengl.h>
#include <mrpt/utils/CEnhancedMetaFile.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CMappedImage.h>

#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservationRange.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/poses/CPosePDFGaussian.h>

#include <mrpt/math/utils.h>

#include <algorithm>
#include <cassert>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

IMPLEMENTS_SERIALIZABLE(COccupancyGridMap2D, CMetricMap,mrpt::slam)


std::vector<float>		COccupancyGridMap2D::entropyTable;

#define	MAX_H		0.69314718055994531f	// ln(2)

/** A lookup table to compute occupancy probabilities in [0,1] from integer log-odds values in the cells, using \f$ p(m_{xy}) = \frac{1}{1+exp(-log_odd)} \f$.
  */
std::vector<float>		COccupancyGridMap2D::logoddsTable;

std::vector<uint8_t>	COccupancyGridMap2D::logoddsTable_255;

std::vector<COccupancyGridMap2D::cellType> 	COccupancyGridMap2D::p2lTable;

float *		COccupancyGridMap2D::logoddsTablePtr		= NULL;
uint8_t *	COccupancyGridMap2D::logoddsTable_255Ptr	= NULL;
COccupancyGridMap2D::cellType * 	COccupancyGridMap2D::p2lTablePtr			= NULL;


/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
COccupancyGridMap2D::COccupancyGridMap2D(
	 float		min_x,
	 float		max_x,
	 float		min_y,
	 float		max_y,
	 float		resolution) :
		map(),
		size_x(0),size_y(0),
		x_min(),x_max(),y_min(),y_max(), resolution(),
		precomputedLikelihood(),
		precomputedLikelihoodToBeRecomputed(true),
		m_basis_map(),
		m_voronoi_diagram(),
		m_is_empty(true),
		voroni_free_threshold(),
		updateInfoChangeOnly(),
		insertionOptions(),
		likelihoodOptions(),
		likelihoodOutputs(),
		CriticalPointsList()
{
	MRPT_START;

	// Create look-up table:
	// -----------------------------------
#ifdef	OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS
	size_t  	desiSize = (1<<16);
#else
	size_t  	desiSize = (1<<8);
#endif

	if (logoddsTable.size()!= desiSize )
	{
		logoddsTable.resize( desiSize );
		logoddsTable_255.resize( desiSize );
		for (int i=OCCGRID_CELLTYPE_MIN;i<=OCCGRID_CELLTYPE_MAX;i++)
		{
			float f = 1.0f / (1.0f + exp( - i * OCCGRID_LOGODD_K_INV ) );
			unsigned int idx =  -OCCGRID_CELLTYPE_MIN+i;
			logoddsTable[idx] = f;
			logoddsTable_255[idx] = (uint8_t)(f*255.0f);
		}
		logoddsTablePtr = &logoddsTable[0];
		logoddsTable_255Ptr = &logoddsTable_255[0];

		// Build the p2lTable as well:
		p2lTable.resize( OCCGRID_P2LTABLE_SIZE+1 );
		double K = 1.0 / OCCGRID_P2LTABLE_SIZE;
		for (int j=0;j<=OCCGRID_P2LTABLE_SIZE;j++)
		{
			double p = j*K;
			if (p==0)
				p=1e-14;
			else if (p==1)
				p=1-1e-14;

			double logodd = log(p)-log(1-p);
			int   L = round(logodd * OCCGRID_LOGODD_K);
			if (L>OCCGRID_CELLTYPE_MAX)
				L=OCCGRID_CELLTYPE_MAX;
			else if (L<OCCGRID_CELLTYPE_MIN)
				L=OCCGRID_CELLTYPE_MIN;
			p2lTable[j] = L;
		}
		p2lTablePtr = &p2lTable[0];
	}

	setSize(min_x,max_x,min_y,max_y, resolution,0.5f );

	MRPT_END;
}


/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
COccupancyGridMap2D::~COccupancyGridMap2D()
{
	freeMap();
}


/*---------------------------------------------------------------
						setSize
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::setSize(
	float		x_min,
	float		x_max,
	float		y_min,
	float		y_max,
	float		resolution,
	float		default_value)
{
	MRPT_START;

	ASSERT_(resolution>0)
	ASSERT_(x_max>x_min && y_max>y_min)
	ASSERT_(default_value>=0 && default_value<=1)

    // Liberar primero:
    freeMap();

	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;

	// Adjust sizes to adapt them to full sized cells acording to the resolution:
	x_min = resolution*round(x_min/resolution);
	y_min = resolution*round(y_min/resolution);
	x_max = resolution*round(x_max/resolution);
	y_max = resolution*round(y_max/resolution);

    // Set parameters:
    this->resolution = resolution;
    this->x_min = x_min;
    this->x_max = x_max;
    this->y_min = y_min;
    this->y_max = y_max;

	// Now the number of cells should be integers:
	size_x = round((x_max-x_min)/resolution);
	size_y = round((y_max-y_min)/resolution);

#ifdef	ROWSIZE_MULTIPLE_16
	// map rows must be 16 bytes aligned:
	if (0!=(size_x % 16))
	{
		size_x = ((size_x >> 4)+1) << 4;
        x_max = x_min + size_x * resolution;
	}
	size_x = round((x_max-x_min)/resolution);
	ASSERT_(0==(size_x % 16));
#endif

    // Cells memory:
    map.resize(size_x*size_y,p2l(default_value));

	// Free these buffers also:
	m_basis_map.clear();
	m_voronoi_diagram.clear();

	m_is_empty=true;

	MRPT_END;
}

/*---------------------------------------------------------------
						ResizeGrid
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::resizeGrid(float new_x_min,float new_x_max,float new_y_min,float new_y_max,float new_cells_default_value, bool additionalMargin) MRPT_NO_THROWS
{
	unsigned int			extra_x_izq=0,extra_y_arr=0,new_size_x=0,new_size_y=0;
	std::vector<cellType>	new_map;

	if( new_x_min > new_x_max )
	{
		printf("[COccupancyGridMap2D::resizeGrid] Warning!! Ignoring call, since: x_min=%f  x_max=%f\n", new_x_min, new_x_max);
		return;
	}
	if( new_y_min > new_y_max )
	{
		printf("[COccupancyGridMap2D::resizeGrid] Warning!! Ignoring call, since: y_min=%f  y_max=%f\n", new_y_min, new_y_max);
		return;
	}

	// Required?
	if (new_x_min>=x_min &&
		new_y_min>=y_min &&
		new_x_max<=x_max &&
		new_y_max<=y_max)	return;

	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;

	// Add an additional margin:
	if (additionalMargin)
	{
		if (new_x_min<x_min) new_x_min= floor(new_x_min-4);
		if (new_x_max>x_max) new_x_max= ceil(new_x_max+4);
		if (new_y_min<y_min) new_y_min= floor(new_y_min-4);
		if (new_y_max>y_max) new_y_max= ceil(new_y_max+4);
	}

	// We do not support grid shrinking... at least stay the same:
	new_x_min = min( new_x_min, x_min);
	new_x_max = max( new_x_max, x_max);
	new_y_min = min( new_y_min, y_min);
	new_y_max = max( new_y_max, y_max);


	// Adjust sizes to adapt them to full sized cells acording to the resolution:
	if (fabs(new_x_min/resolution - round(new_x_min/resolution))>0.05f )
		new_x_min = resolution*round(new_x_min/resolution);
	if (fabs(new_y_min/resolution - round(new_y_min/resolution))>0.05f )
		new_y_min = resolution*round(new_y_min/resolution);
	if (fabs(new_x_max/resolution - round(new_x_max/resolution))>0.05f )
		new_x_max = resolution*round(new_x_max/resolution);
	if (fabs(new_y_max/resolution - round(new_y_max/resolution))>0.05f )
		new_y_max = resolution*round(new_y_max/resolution);

	// Change size: 4 sides extensions:
	extra_x_izq = round((x_min-new_x_min) / resolution);
	extra_y_arr = round((y_min-new_y_min) / resolution);

	new_size_x = round((new_x_max-new_x_min) / resolution);
	new_size_y = round((new_y_max-new_y_min) / resolution);

	assert( new_size_x>=size_x+extra_x_izq );

#ifdef	ROWSIZE_MULTIPLE_16
	// map rows must be 16 bytes aligned:
	size_t old_new_size_x = new_size_x;  // Debug
	if (0!=(new_size_x % 16))
	{
		int size_x_incr = 16 - (new_size_x % 16);
        //new_x_max = new_x_min + new_size_x * resolution;
        new_x_max += size_x_incr * resolution;
	}
	new_size_x = round((new_x_max-new_x_min)/resolution);
	assert(0==(new_size_x % 16));
#endif

	// Reservar la nueva memoria:
    new_map.resize(new_size_x*new_size_y, p2l(new_cells_default_value));

	// Copiar todas las filas del mapa antiguo dentro del nuevo:
	{
		cellType  	*dest_ptr = &new_map[extra_x_izq + extra_y_arr*new_size_x];
		cellType  	*src_ptr  = &map[0];
		size_t 		row_size = size_x*sizeof(cellType);

		for (size_t y = 0;y<size_y;y++)
		{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
			assert( dest_ptr+row_size-1 <= &new_map[new_map.size()-1] );
			assert( src_ptr+row_size-1 <= &map[map.size()-1] );
#endif
			memcpy( dest_ptr, src_ptr, row_size );
			dest_ptr += new_size_x;
			src_ptr  += size_x;
		}
	}

    // Copiar ya los nuevos valores al objeto:
    x_min = new_x_min;
    x_max = new_x_max;
    y_min = new_y_min;
    y_max = new_y_max;

    size_x = new_size_x;
    size_y = new_size_y;

	// Free old map, replace by new one:
	map.swap( new_map );

	// Free the other buffers:
	m_basis_map.clear();
	m_voronoi_diagram.clear();
}

/*---------------------------------------------------------------
						freeMap
  ---------------------------------------------------------------*/
void COccupancyGridMap2D::freeMap()
{
	MRPT_START;

	// Free map and sectors
    map.clear();

	m_basis_map.clear();
	m_voronoi_diagram.clear();

    size_x=size_y=0;

	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;

	m_is_empty=true;

	MRPT_END;
}

/*---------------------------------------------------------------
				Build_VoronoiDiagram
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::buildVoronoiDiagram(
			float		threshold,
			float		robot_size,
			int			x1,
			int			x2,
			int			y1,
			int			y2)
{
        // The whole map?
        if (!x1 && !x2 && !y1 && !y2)
        {
                x1=y1=0;
                x2=size_x-1;
                y2=size_y-1;
        }
        else
        {
                x1=max(0,x1);
                y1=max(0,y1);
                x2=min(x2,static_cast<int>(size_x)-1);
                y2=min(y2,static_cast<int>(size_y)-1);
        }

        int robot_size_units= round(100*robot_size / resolution);

/*   ¡¡DE RECUERDO!!
        // En cada celda se guarda un 0 si no pertenece al diagrama de voronoi,
        //   o si pertenece, la distancia hasta los obstaculos mas cercanos,
        //   la "clearance" (en unidades enteras de centesimas de celdas)
        int             *voronoi_diagram;
*/
        // Iniciar voronoi:

		m_voronoi_diagram.setSize(x_min,x_max, y_min,y_max, resolution);  // assign(size_x*size_y,0);
		ASSERT_EQUAL_(m_voronoi_diagram.getSizeX(), size_x);
		ASSERT_EQUAL_(m_voronoi_diagram.getSizeY(), size_y);
		m_voronoi_diagram.fill(0);

        // threshold en prob. de ocupado. Nosotros guardamos prob. de libre.
//        voroni_free_threshold= 1.0f - threshold;
        voroni_free_threshold= 1.0f - threshold;


        int     basis_x[2],basis_y[2];
        int     nBasis;
        int     Clearance;

        // Construir: (Lo gordo esta en la funcion "ComputeClearance")
		int x;
        for (x=x1;x<=x2;x++)
         for (int y=y1;y<=y2;y++)
         {
                Clearance = computeClearance(x,y,basis_x,basis_y,&nBasis);

                if (Clearance > robot_size_units )
                        setVoroniClearance(x,y,Clearance );
         }

         // Limpiar: Hacer que los trazos sean de grosor 1:
         //  Si un punto del diagrama esta rodeada de mas de 2
         //   puntos tb del diagrama, eliminarlo:
         int    nDiag;
         for (x=x1;x<=x2;x++)
          for (int y=y1;y<=y2;y++)
           if ( getVoroniClearance(x,y) )
           {
                nDiag=0;
                for (int xx=x-1;xx<=(x+1);xx++)
                 for (int yy=y-1;yy<=(y+1);yy++)
                  if (getVoroniClearance(xx,yy)) nDiag++;

                // Eliminar?
                if (nDiag>3)
                        setVoroniClearance(x,y,0);
           }

}

/*---------------------------------------------------------------
					findCriticalPoints
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::findCriticalPoints( float filter_distance )
{
        int     clear_xy, clear;

        int     filter_dist = round(filter_distance / resolution);
        int     min_clear_near, max_clear_near;


        // Resize basis-points map & set to zero:
		m_basis_map.setSize(x_min,x_max, y_min,y_max, resolution);	//m_basis_map.assign(size_x*size_y, 0);
		ASSERT_EQUAL_(m_basis_map.getSizeX(), size_x);
		ASSERT_EQUAL_(m_basis_map.getSizeY(), size_y);
		m_basis_map.fill(0);

        // Lista temporary de candidatos:
		std::vector<int>	temp_x,temp_y, temp_clear, temp_borrar;

        // Escanear en busqueda de puntos criticos:
        // ---------------------------------------------
        for (int x=1;x<(static_cast<int>(size_x)-1);x++)
          for (int y=1;y<(static_cast<int>(size_y)-1);y++)
            if ( 0!=(clear_xy=getVoroniClearance(x,y)) )
                {
                        // Ver si es un punto critico:
                        int nVecinosVoroni = 0;
                        min_clear_near = max_clear_near = clear_xy;

                        for (int xx=x-2;xx<=(x+2);xx++)
                          for (int yy=y-2;yy<=(y+2);yy++)
                          {
                                if ( 0!=(clear = getVoroniClearance(xx,yy)) )
                                {
                                        nVecinosVoroni++;
                                        min_clear_near = min( min_clear_near , clear );
                                        max_clear_near = max( max_clear_near , clear );
                                }
                          }

                          // Al menos tiene q haber 2 puntos mas alrededor:
                          if (nVecinosVoroni>=3 && min_clear_near==clear_xy && max_clear_near!=clear_xy )
                          {
                                // Add to temp list:
                                temp_x.push_back( x );
                                temp_y.push_back( y );
                                temp_clear.push_back( clear_xy );
                                temp_borrar.push_back( 0 );
                          }

                }


        // Filtrar: Hayar los "basis points". Si dos coindicen, dejar solo
        //  el crit.pt. con menos clearance:

        // Hacer la lista de todos los basis points:
		std::vector<int>       basis1_x,basis1_y, basis2_x,basis2_y;
		unsigned i;
        for (i=0;i<temp_x.size();i++)
        {
                int     basis_x[2];
                int     basis_y[2];
                int     nBasis;

                computeClearance(temp_x[i],temp_y[i],basis_x,basis_y,&nBasis);

                if (nBasis==2)
                {
                        basis1_x.push_back(basis_x[0]);
                        basis1_y.push_back(basis_y[0]);

                        basis2_x.push_back(basis_x[1]);
                        basis2_y.push_back(basis_y[1]);
                }
        }

       // Ver basis que coincidan:
       for (i=0;i<(((temp_x.size()))-1);i++)
        if (!temp_borrar[i])
         for (unsigned int j=i+1;j<temp_x.size();j++)
          if (!temp_borrar[j])
          {
                int ax,ay;

                // i1-j1
                ax = basis1_x[i]-basis1_x[j];
                ay = basis1_y[i]-basis1_y[j];
                bool i1j1= (sqrt(1.0f*ax*ax+ay*ay)<filter_dist);

                // i1-j2
                ax = basis1_x[i]-basis2_x[j];
                ay = basis1_y[i]-basis2_y[j];
                bool i1j2= (sqrt(1.0f*ax*ax+ay*ay)<filter_dist);

                // i2-j1
                ax = basis2_x[i]-basis1_x[j];
                ay = basis2_y[i]-basis1_y[j];
                bool i2j1= (sqrt(1.0f*ax*ax+ay*ay)<filter_dist);

                // i2-j2
                ax = basis2_x[i]-basis2_x[j];
                ay = basis2_y[i]-basis2_y[j];
                bool i2j2= (sqrt(1.0f*ax*ax+ay*ay)<filter_dist);


                // Si coincide, eliminar el de mas "dist."
                if ( (i1j1 && i2j2) || (i1j2 && i2j1) )
                {
                        if ( temp_clear[i]<temp_clear[j] )
                                temp_borrar[j]=1;
                        else    temp_borrar[i]=1;
                }

          }


        // Copiar ya a la lista definitiva:
        // ----------------------------------------------------------
        CriticalPointsList.clearance.clear();
        CriticalPointsList.x.clear();
        CriticalPointsList.y.clear();
        CriticalPointsList.x_basis1.clear();
        CriticalPointsList.y_basis1.clear();
        CriticalPointsList.x_basis2.clear();
        CriticalPointsList.y_basis2.clear();

        for (i=0;i<temp_x.size();i++)
        {
                if (!temp_borrar[i])
                {
                        CriticalPointsList.x.push_back( temp_x[i] );
                        CriticalPointsList.y.push_back( temp_y[i] );
                        CriticalPointsList.clearance.push_back( temp_clear[i] );

                        // Add to the basis points as well:
                        setBasisCell( temp_x[i],temp_y[i] ,1);
                }
        }
}

/*---------------------------------------------------------------
    Calcula la "clearance" de una celda, y devuelve sus
       dos (primeros) "basis"
    -Devuelve la "clearance" en unidades de centesimas de "celdas"
    -basis_x/y deben dar sitio para 2 int's

    - Devuelve no cero solo si la celda pertenece a Voroni

    Si se pone "GetContourPoint"=true, no se devuelven los puntos
     ocupados como basis, sino los libres mas cercanos (Esto
     se usa para el calculo de regiones)

 Sirve para calcular diagramas de Voronoi y crit. points,etc...
  ---------------------------------------------------------------*/
int  COccupancyGridMap2D::computeClearance( int cx, int cy, int *basis_x, int *basis_y, int *nBasis, bool GetContourPoint ) const
{
	static const cellType	thresholdCellValue = p2l(0.5f);

	// Si la celda esta ocupada, clearance de cero!
	if ( static_cast<unsigned>(cx)>=size_x || static_cast<unsigned>(cy)>=size_y )
		return 0;

	if ( map[cx+cy*size_y]<thresholdCellValue )
		return 0;

	// Truco para acelerar MUCHO:
	//  Si miramos un punto junto al mirado antes,
	//   usar sus resultados, xk SEGURO que no hay obstaculos
	//   mucho antes:
	static int  ultimo_cx = -10, ultimo_cy = -10;
	int     estimated_min_free_circle;
	static int ultimo_free_circle;

	if ( abs(ultimo_cx-cx)<=1 && abs(ultimo_cy-cy)<=1)
			estimated_min_free_circle = max(1,ultimo_free_circle - 3);
	else
			estimated_min_free_circle = 1;

	ultimo_cx = cx;
	ultimo_cy = cy;

	// Tabla de circulos:
	#define N_CIRCULOS  100
	static bool tabla_construida = false;
	static int     nEntradasCirculo[N_CIRCULOS];
	static int     circ_PrimeraEntrada[N_CIRCULOS];
	static int     circs_x[32000],circs_y[32000];

	if (!tabla_construida)
	{
			tabla_construida=true;
			int     indice_absoluto = 0;
			for (int i=0;i<N_CIRCULOS;i++)
			{
					int             nPasos = round(1+(M_2PI*i)); // Estimacion de # de entradas (luego seran menos)
					float           A = 0;
					float           AA = (2.0f*M_PIf / nPasos);
					register int    ult_x=0,x,ult_y=0,y;
					int             nEntradas = 0;

					circ_PrimeraEntrada[i] = indice_absoluto;

					while (A<2*M_PI)
					{
							x =  round( i*cos( A ) );
							y =  round( i*sin( A ) );

							if ((x!=ult_x || y!=ult_y) && !(x==i && y==0) )
							{
									circs_x[indice_absoluto]=x;
									circs_y[indice_absoluto++]=y;

									nEntradas++;
									ult_x=x;
									ult_y=y;
							}

							A+=AA;
					}

					nEntradasCirculo[i]=nEntradas;

			}

	}


	// La celda esta libre. Buscar en un circulo creciente hasta dar
	//  dar con el obstaculo mas cercano:
	*nBasis=0;
	int tam_circ;

	int    vueltas_extra = 2;

	for (tam_circ=estimated_min_free_circle;tam_circ<N_CIRCULOS && (!(*nBasis) || vueltas_extra );tam_circ++)
	{
			int     nEnts = nEntradasCirculo[tam_circ];
			bool    dentro_obs = false;
			int     idx = circ_PrimeraEntrada[tam_circ];

			for (int j=0;j<nEnts && (*nBasis)<2;j++,idx++)
			{
					register int xx = cx+circs_x[idx];
					register int yy = cy+circs_y[idx];

				   if (xx>=0 && xx<static_cast<int>(size_x) && yy>=0 && yy<static_cast<int>(size_y))
				   {
					//if ( getCell(xx,yy)<=voroni_free_threshold )
					if ( map[xx+yy*size_y]<thresholdCellValue )
					{
							if (!dentro_obs)
							{
									dentro_obs = true;

									// Esta el 2o punto separado del 1o??
									bool pasa;

									if (!(*nBasis))
											pasa = true;
									else
									{
											int ax = basis_x[0] - xx;
											int ay = basis_y[0] - yy;
											pasa = sqrt(1.0f*ax*ax+ay*ay)>(1.75f*tam_circ);
									}

									if (pasa)
									{
											basis_x[*nBasis] = cx+circs_x[idx];
											basis_y[*nBasis] = cy+circs_y[idx];
											(*nBasis)++;
									}
							}
					}
					else
							dentro_obs = false;
				   }
			}

			// Si solo encontramos 1 obstaculo, 1 sola vuelta extra mas:
			if (*nBasis)
			{
				if (*nBasis==1) vueltas_extra--;
					else vueltas_extra=0;
			}
	}

	// Estimacion para siguiente punto:
	ultimo_free_circle = tam_circ;

	if (*nBasis>=2)
	{
			if (GetContourPoint)
			{
					unsigned char vec;
					int dx, dy, dir_predilecta,dir;

					// Hayar punto libre mas cercano al basis 0:
					dx = cx - basis_x[0];
					dy = cy - basis_y[0];
					if (abs(dx)>abs(dy))
							if (dx>0)       dir_predilecta=4;
							else            dir_predilecta=3;
					else
							if (dy>0)       dir_predilecta=1;
							else            dir_predilecta=6;

					vec =  GetNeighborhood( basis_x[0],  basis_y[0] );
					dir = -1;
					if (!(vec & (1<<dir_predilecta))) dir = dir_predilecta;
					else if (!(vec & (1<<1))) dir = 1;
					else if (!(vec & (1<<3))) dir = 3;
					else if (!(vec & (1<<4))) dir = 4;
					else if (!(vec & (1<<6))) dir = 6;
					if (dir!=-1)
					{
					 vec = GetNeighborhood(basis_x[0]+direccion_vecino_x[dir],basis_y[0]+direccion_vecino_y[dir]);
					 if (vec!=0x00 && vec!=0xFF)
					 {
							basis_x[0]+=direccion_vecino_x[dir];
							basis_y[0]+=direccion_vecino_y[dir];
					 }
					}

					// Hayar punto libre mas cercano al basis 1:
					dx = cx - basis_x[1];
					dy = cy - basis_y[1];
					if (abs(dx)>abs(dy))
							if (dx>0)       dir_predilecta=4;
							else            dir_predilecta=3;
					else
							if (dy>0)       dir_predilecta=1;
							else            dir_predilecta=6;

					vec =  GetNeighborhood( basis_x[1],  basis_y[1] );
					dir = -1;
					if (!(vec & (1<<dir_predilecta))) dir = dir_predilecta;
					else if (!(vec & (1<<1))) dir = 1;
					else if (!(vec & (1<<3))) dir = 3;
					else if (!(vec & (1<<4))) dir = 4;
					else if (!(vec & (1<<6))) dir = 6;
					if (dir!=-1)
					{
					 vec = GetNeighborhood(basis_x[1]+direccion_vecino_x[dir],basis_y[1]+direccion_vecino_y[dir]);
					 if (vec!=0x00 && vec!=0xFF)
					 {
							basis_x[1]+=direccion_vecino_x[dir];
							basis_y[1]+=direccion_vecino_y[dir];
					 }
					}
			}

			return tam_circ*100;
	}
	else    return 0;
}

/*---------------------------------------------------------------
						laserScanSimulator

 Simulates a range scan into the current grid map.
   The simulated scan is stored in a CObservation2DRangeScan object, which is also used
    to pass some parameters: all previously stored characteristics (as aperture,...) are
	  taken into account for simulation. Only a few more parameters are needed. Additive gaussian noise can be optionally added to the simulated scan.
		inout_Scan [IN/OUT] This must be filled with desired parameters before calling, and will contain the scan samples on return.
		robotPose [IN] The robot pose in this map coordinates. Recall that sensor pose relative to this robot pose must be specified in the observation object.
		threshold [IN] The minimum occupancy threshold to consider a cell to be occupied, for example 0.5.
		N [IN] The count of range scan "rays", by default to 361.
		noiseStd [IN] The standard deviation of measurement noise. If not desired, set to 0.
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::laserScanSimulator(
		CObservation2DRangeScan	        &inout_Scan,
		const CPose2D					&robotPose,
		float						    threshold,
		size_t							N,
		float						    noiseStd,
		unsigned int				    decimation,
		float							angleNoiseStd ) const
{
	MRPT_START

	ASSERT_(decimation>=1);

	// Sensor pose in global coordinates
	CPose3D		sensorPose3D = CPose3D(robotPose) + inout_Scan.sensorPose;
	// Aproximation: grid is 2D !!!
	CPose2D		sensorPose(sensorPose3D);

    // Scan size:
    inout_Scan.scan.resize(N);
    inout_Scan.validRange.resize(N);

    double  A, AA;
	if (inout_Scan.rightToLeft)
	{
		A = sensorPose.phi() - 0.5*inout_Scan.aperture;
		AA = (inout_Scan.aperture / N);
	}
	else
	{
		A = sensorPose.phi() + 0.5*inout_Scan.aperture;
		AA = -(inout_Scan.aperture / N);
	}

    const float free_thres = 1.0f - threshold;
    const unsigned int max_ray_len = round(inout_Scan.maxRange / resolution);

    for (size_t i=0;i<N;i+=decimation,A+=AA*decimation)
    {
    	bool valid;
    	simulateScanRay(
			sensorPose.x(),sensorPose.y(),A,
			inout_Scan.scan[i],valid,
			max_ray_len, free_thres,
			noiseStd, angleNoiseStd );
		inout_Scan.validRange[i] = valid ? 1:0;
    }

	MRPT_END
}

void  COccupancyGridMap2D::sonarSimulator(
		CObservationRange	        &inout_observation,
		const CPose2D				&robotPose,
		float						threshold,
		float						rangeNoiseStd,
		float						angleNoiseStd) const
{
    const float free_thres = 1.0f - threshold;
    const unsigned int max_ray_len = round(inout_observation.maxSensorDistance / resolution);

	for (CObservationRange::iterator itR=inout_observation.begin();itR!=inout_observation.end();++itR)
	{
		const CPose2D sensorAbsolutePose = CPose2D( CPose3D(robotPose) + CPose3D(itR->sensorPose) );

    	// For each sonar cone, simulate several rays and keep the shortest distance:
    	ASSERT_(inout_observation.sensorConeApperture>0)
    	size_t nRays = round(1+ inout_observation.sensorConeApperture / DEG2RAD(1.0) );

    	double direction = sensorAbsolutePose.phi() - 0.5*inout_observation.sensorConeApperture;
		const double Adir = inout_observation.sensorConeApperture / nRays;

		float min_detected_obs=0;
    	for (size_t i=0;i<nRays;i++, direction+=Adir )
    	{
			bool valid;
			float sim_rang;
			simulateScanRay(
				sensorAbsolutePose.x(), sensorAbsolutePose.y(), direction,
				sim_rang, valid,
				max_ray_len, free_thres,
				rangeNoiseStd, angleNoiseStd );

			if (valid && (sim_rang<min_detected_obs || !i))
				min_detected_obs = sim_rang;
    	}
    	// Save:
    	itR->sensedDistance = min_detected_obs;
	}
}

inline void COccupancyGridMap2D::simulateScanRay(
	const double start_x,const double start_y,const double angle_direction,
	float &out_range,bool &out_valid,
	const unsigned int max_ray_len,
	const float threshold_free,
	const double noiseStd, const double angleNoiseStd ) const
{
	const double A_ = angle_direction + randomGenerator.drawGaussian1D_normalized()*angleNoiseStd;

	// Unit vector in the directorion of the ray:
#ifdef HAVE_SINCOS
	double Arx,Ary;
	::sincos(A_, &Ary,&Arx);
	Arx*=resolution;
	Ary*=resolution;
#else
	const double Arx =  cos(A_)*resolution;
	const double Ary =  sin(A_)*resolution;
#endif

	// Ray tracing, until collision, out of the map or out of range:
	unsigned int ray_len=0;
	unsigned int firstUnknownCellDist=max_ray_len+1;
	double rx=start_x;
	double ry=start_y;
	float hitCellOcc = 0.5f;
	int x, y=y2idx(ry);

	while ( (x=x2idx(rx))>=0 && (y=y2idx(ry))>=0 &&
			 x<static_cast<int>(size_x) && y<static_cast<int>(size_y) && (hitCellOcc=getCell(x,y))>threshold_free &&
			 ray_len<max_ray_len  )
	{
		if ( fabs(hitCellOcc-0.5)<0.01f )
			mrpt::utils::keep_min(firstUnknownCellDist, ray_len );

		rx+=Arx;
		ry+=Ary;
		ray_len++;
	}

	// Store:
	// Check out of the grid?
	// Tip: if x<0, (unsigned)(x) will also be >>> size_x ;-)
	if (fabs(hitCellOcc-0.5)<0.01f || static_cast<unsigned>(x)>=size_x || static_cast<unsigned>(y)>=size_y )
	{
		out_valid = false;

		if (firstUnknownCellDist<ray_len)
				out_range = firstUnknownCellDist*resolution;
		else	out_range = ray_len*resolution;
	}
	else
	{ 	// No: The normal case:
		out_range = ray_len*resolution;
		out_valid = ray_len<max_ray_len;
		// Add additive Gaussian noise:
		if (noiseStd>0 && out_valid)
			out_range+=  noiseStd*randomGenerator.drawGaussian1D_normalized();
	}
}

/*---------------------------------------------------------------
    Devuelve un BYTE, con bits=1 si el vecino esta ocupado:
  Asociacion de numero de bit a vecinos:

                0       1       2
                3       X       4
                5       6       7
  ---------------------------------------------------------------*/
inline unsigned char  COccupancyGridMap2D::GetNeighborhood( int cx, int cy ) const
{
        unsigned char res=0;

        if (getCell(cx-1,cy-1)<=voroni_free_threshold) res |= (1<<0);
        if (getCell( cx ,cy-1)<=voroni_free_threshold) res |= (1<<1);
        if (getCell(cx+1,cy-1)<=voroni_free_threshold) res |= (1<<2);
        if (getCell(cx-1, cy )<=voroni_free_threshold) res |= (1<<3);
        if (getCell(cx+1, cy )<=voroni_free_threshold) res |= (1<<4);
        if (getCell(cx-1,cy+1)<=voroni_free_threshold) res |= (1<<5);
        if (getCell( cx ,cy+1)<=voroni_free_threshold) res |= (1<<6);
        if (getCell(cx+1,cy+1)<=voroni_free_threshold) res |= (1<<7);

        return res;
}

/*---------------------------------------------------------------
Devuelve el indice 0..7 de la direccion, o -1 si no es valida:
                0       1       2
                3       X       4
                5       6       7
  ---------------------------------------------------------------*/
int  COccupancyGridMap2D::direction2idx(int dx, int dy)
{
        switch (dx)
        {
                case -1:
                        switch(dy)
                        {
                                case -1: return 0;
                                case  0: return 3;
                                case  1: return 5;
                                default: return -1;
                        };
                case 0:
                        switch(dy)
                        {
                                case -1: return 1;
                                case  1: return 6;
                                default: return -1;
                        };
                case  1:
                        switch(dy)
                        {
                                case -1: return 2;
                                case  0: return 4;
                                case  1: return 7;
                                default: return -1;
                        };
                default: return -1;
        };

}

/*---------------------------------------------------------------
					saveAsBitmapFile
  ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::saveAsBitmapFile(const std::string &file) const
{
	MRPT_START;

	CImage			img;
	getAsImage(img);
	return img.saveToFile(file);

	MRPT_END;
}


/*---------------------------------------------------------------
					getAsImage
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::getAsImage(
	utils::CImage	&img,
	bool verticalFlip,
	bool forceRGB,
	bool tricolor ) const
{
	if (!tricolor)
	{
		if (!forceRGB)
		{	// 8bit gray-scale
			img.resize(size_x,size_y,1,true); //verticalFlip);
			const cellType	*srcPtr = &map[0];
			unsigned char	*destPtr;
			for (unsigned int y=0;y<size_y;y++)
			{
				if (!verticalFlip)
						destPtr = img(0,size_y-1-y);
				else 	destPtr = img(0,y);
				for (unsigned int x=0;x<size_x;x++)
				{
					*destPtr++ = l2p_255(*srcPtr++);
				}
			}
		}
		else
		{	// 24bit RGB:
			img.resize(size_x,size_y,3,true); //verticalFlip);
			const cellType	*srcPtr = &map[0];
			unsigned char	*destPtr;
			for (unsigned int y=0;y<size_y;y++)
			{
				if (!verticalFlip)
						destPtr = img(0,size_y-1-y);
				else 	destPtr = img(0,y);
				for (unsigned int x=0;x<size_x;x++)
				{
					uint8_t c = l2p_255(*srcPtr++);
					*destPtr++ = c;
					*destPtr++ = c;
					*destPtr++ = c;
				}
			}
		}
	}
	else
	{
		// TRICOLOR: 0, 0.5, 1
		if (!forceRGB)
		{	// 8bit gray-scale
			img.resize(size_x,size_y,1,true); //verticalFlip);
			const cellType	*srcPtr = &map[0];
			unsigned char	*destPtr;
			for (unsigned int y=0;y<size_y;y++)
			{
				if (!verticalFlip)
						destPtr = img(0,size_y-1-y);
				else 	destPtr = img(0,y);
				for (unsigned int x=0;x<size_x;x++)
				{
					uint8_t c = l2p_255(*srcPtr++);
					if (c<120)
						c=0;
					else if (c>136)
						c=255;
					else c = 127;
					*destPtr++ = c;
				}
			}
		}
		else
		{	// 24bit RGB:
			img.resize(size_x,size_y,3,true); //verticalFlip);
			const cellType	*srcPtr = &map[0];
			unsigned char	*destPtr;
			for (unsigned int y=0;y<size_y;y++)
			{
				if (!verticalFlip)
						destPtr = img(0,size_y-1-y);
				else 	destPtr = img(0,y);
				for (unsigned int x=0;x<size_x;x++)
				{
					uint8_t c = l2p_255(*srcPtr++);
					if (c<120)
						c=0;
					else if (c>136)
						c=255;
					else c = 127;

					*destPtr++ = c;
					*destPtr++ = c;
					*destPtr++ = c;
				}
			}
		}
	}
}

/*---------------------------------------------------------------
					getAsImageFiltered
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::getAsImageFiltered(
	utils::CImage	&img,
	bool verticalFlip,
	bool forceRGB ) const
{
	getAsImage(img,verticalFlip,forceRGB);

	// Do filtering to improve the noisy peaks in grids:
	// ----------------------------------------------------
#if 0
	CTicTac  t;
#endif
	if (insertionOptions.CFD_features_gaussian_size!=0) 	img.filterGaussianInPlace( round( insertionOptions.CFD_features_gaussian_size ) );
	if (insertionOptions.CFD_features_median_size!=0) 		img.filterMedianInPlace( round( insertionOptions.CFD_features_median_size ) );
#if 0
	cout << "[COccupancyGridMap2D::getAsImageFiltered] Filtered in: " << t.Tac()*1000 << " ms" << endl;
#endif
}


/*---------------------------------------------------------------
					writeToStream
	Implements the writing to a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 5;
	else
	{
		// Version 3: Change to log-odds. The only change is in the loader, when translating
		//   from older versions.

		// Version 2: Save OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS/16BITS
#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
		out << uint8_t(8);
#else
		out << uint8_t(16);
#endif

		out << size_x << size_y << x_min << x_max << y_min << y_max << resolution;
		ASSERT_(size_x*size_y==map.size());
		out.WriteBuffer(&map[0], sizeof(map[0])*size_x*size_y);


		// insertionOptions:
		out <<	insertionOptions.mapAltitude
			<<	insertionOptions.useMapAltitude
			<<	insertionOptions.maxDistanceInsertion
			<<	insertionOptions.maxOccupancyUpdateCertainty
			<<	insertionOptions.considerInvalidRangesAsFreeSpace
			<<	insertionOptions.decimation
			<<	insertionOptions.horizontalTolerance;

		// Likelihood:
		out	<<	(int32_t)likelihoodOptions.likelihoodMethod
			<<	likelihoodOptions.LF_stdHit
			<<	likelihoodOptions.LF_zHit
			<<	likelihoodOptions.LF_zRandom
			<<	likelihoodOptions.LF_maxRange
			<<	likelihoodOptions.LF_decimation
			<<	likelihoodOptions.LF_maxCorrsDistance
			<<	likelihoodOptions.LF_alternateAverageMethod
			<<	likelihoodOptions.MI_exponent
			<<	likelihoodOptions.MI_skip_rays
			<<	likelihoodOptions.MI_ratio_max_distance
			<<	likelihoodOptions.rayTracing_useDistanceFilter
			<<	likelihoodOptions.rayTracing_decimation
			<<	likelihoodOptions.rayTracing_stdHit
			<<	likelihoodOptions.consensus_takeEachRange
			<<	likelihoodOptions.consensus_pow
			<<	likelihoodOptions.OWA_weights
			<<	likelihoodOptions.enableLikelihoodCache;

		// Insertion as 3D:
		out << m_disableSaveAs3DObject;

		// Version 4:
		out << insertionOptions.CFD_features_gaussian_size
		    << insertionOptions.CFD_features_median_size;

		// Version: 5;
		out << insertionOptions.wideningBeamsWithDistance;

	}
}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::readFromStream(CStream &in, int version)
{
	m_is_empty = false;

	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		{
#			ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
				const uint8_t	MyBitsPerCell = 8;
#			else
				const uint8_t	MyBitsPerCell = 16;
#			endif

			uint8_t		bitsPerCellStream;

			// Version 2: OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS/16BITS
			if (version>=2)
					in >> bitsPerCellStream;
			else	bitsPerCellStream = MyBitsPerCell;  // Old versinons: hope it's the same...

			uint32_t        new_size_x,new_size_y;
			float           new_x_min,new_x_max,new_y_min,new_y_max;
			float			new_resolution;

			//resetFeaturesCache();

			in >> new_size_x >> new_size_y >> new_x_min >> new_x_max >> new_y_min >> new_y_max >> new_resolution;

			setSize(new_x_min,new_x_max,new_y_min,new_y_max,new_resolution,0.5);

			ASSERT_(size_x*size_y==map.size());

			if (bitsPerCellStream==MyBitsPerCell)
			{
				// Perfect:
				in.ReadBuffer(&map[0], sizeof(map[0])*map.size());
			}
			else
			{
				// We must do a conversion...
#			ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
				// We are 8-bit, stream is 16-bit
				ASSERT_(bitsPerCellStream==16);
				std::vector<uint16_t>    auxMap( map.size() );
				in.ReadBuffer(&auxMap[0], sizeof(auxMap[0])*auxMap.size());

				size_t  i, N = map.size();
				uint8_t         *ptrTrg = (uint8_t*)&map[0];
				const uint16_t  *ptrSrc = (const uint16_t*)&auxMap[0];
				for (i=0;i<N;i++)
					*ptrTrg++ = (*ptrSrc++) >> 8;
#			else
				// We are 16-bit, stream is 8-bit
				ASSERT_(bitsPerCellStream==8);
				std::vector<uint8_t>    auxMap( map.size() );
				in.ReadBuffer(&auxMap[0], sizeof(auxMap[0])*auxMap.size());

				size_t  i, N = map.size();
				uint16_t       *ptrTrg = (uint16_t*)&map[0];
				const uint8_t  *ptrSrc = (const uint8_t*)&auxMap[0];
				for (i=0;i<N;i++)
					*ptrTrg++ = (*ptrSrc++) << 8;
#			endif
			}

			// If we are converting an old dump, convert from probabilities to log-odds:
			if (version<3)
			{
				size_t  i, N = map.size();
				cellType  *ptr = &map[0];
				for (i=0;i<N;i++)
				{
					double p = cellTypeUnsigned(*ptr) * (1.0f/0xFF);
					if (p<0)
						p=0;
					if (p>1)
						p=1;
					*ptr++ = p2l( p );
				}
			}

			// For the precomputed likelihood trick:
			precomputedLikelihoodToBeRecomputed = true;

			if (version>=1)
			{
				// insertionOptions:
				in  >>	insertionOptions.mapAltitude
					>>	insertionOptions.useMapAltitude
					>>	insertionOptions.maxDistanceInsertion
					>>	insertionOptions.maxOccupancyUpdateCertainty
					>>	insertionOptions.considerInvalidRangesAsFreeSpace
					>>	insertionOptions.decimation
					>>	insertionOptions.horizontalTolerance;

				// Likelihood:
				int32_t		i;
				in 	>>	i; likelihoodOptions.likelihoodMethod = static_cast<TLikelihoodMethod>(i);
				in	>>	likelihoodOptions.LF_stdHit
					>>	likelihoodOptions.LF_zHit
					>>	likelihoodOptions.LF_zRandom
					>>	likelihoodOptions.LF_maxRange
					>>	likelihoodOptions.LF_decimation
					>>	likelihoodOptions.LF_maxCorrsDistance
					>>	likelihoodOptions.LF_alternateAverageMethod
					>>	likelihoodOptions.MI_exponent
					>>	likelihoodOptions.MI_skip_rays
					>>	likelihoodOptions.MI_ratio_max_distance
					>>	likelihoodOptions.rayTracing_useDistanceFilter
					>>	likelihoodOptions.rayTracing_decimation
					>>	likelihoodOptions.rayTracing_stdHit
					>>	likelihoodOptions.consensus_takeEachRange
					>>	likelihoodOptions.consensus_pow
					>>	likelihoodOptions.OWA_weights
					>>	likelihoodOptions.enableLikelihoodCache;

				// Insertion as 3D:
				in  >> m_disableSaveAs3DObject;
			}

			if (version>=4)
			{
				in >> insertionOptions.CFD_features_gaussian_size
				   >> insertionOptions.CFD_features_median_size;
			}

			if (version>=5)
			{
				in >> insertionOptions.wideningBeamsWithDistance;
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


/*---------------------------------------------------------------
  Computes the entropy and related values of this grid map.
	out_H The target variable for absolute entropy, computed as:<br><center>H(map)=Sum<sub>x,y</sub>{ -p(x,y)·ln(p(x,y)) -(1-p(x,y))·ln(1-p(x,y)) }</center><br><br>
	out_I The target variable for absolute "information", defining I(x) = 1 - H(x)
	out_mean_H The target variable for mean entropy, defined as entropy per square meter: mean_H(map) = H(map) / (Map length x (meters))·(Map length y (meters))
	out_mean_I The target variable for mean information, defined as information per square meter: mean_I(map) = I(map) / (Map length x (meters))·(Map length y (meters))
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::computeEntropy( TEntropyInfo &info ) const
{
	unsigned long					i;
	float							h,p;
	std::vector<unsigned long>		histogram;


#ifdef	OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
	unsigned int								N = 256;
#else
	unsigned int								N = 65536;
#endif

	// Compute the entropy table: The entropy for each posible cell value
	// ----------------------------------------------------------------------
	if (entropyTable.size()!=N)
	{
		entropyTable.resize(N,0);
		for (i=0;i<N;i++)
		{
			p = l2p(static_cast<cellType>(i));
			h = H(p)+H(1-p);

			// Cell's probabilities rounding problem fixing:
			if (i==0 || i==(N-1)) h=0;
			if (h>(MAX_H - 1e-4)) h=MAX_H;

			entropyTable[i] = h;
		}
	}

	// Initialize the global results:
	info.H = info.I = 0;
	info.effectiveMappedCells = 0;


	info.H = info.I = 0;
	info.effectiveMappedCells = 0;
	for ( std::vector<cellType>::const_iterator it=map.begin();it!=map.end();++it)
	{
		cellTypeUnsigned  i = static_cast<cellTypeUnsigned>(*it);
		h = entropyTable[ i ];
		info.H+= h;
		if (h<(MAX_H-0.001f))
		{
			info.effectiveMappedCells++;
			info.I-=h;
		}
	}

	// The info: (See ref. paper EMMI in IROS 2006)
	info.I /= MAX_H;
	info.I += info.effectiveMappedCells;

	// Mean values:
	// ------------------------------------------
	info.effectiveMappedArea = info.effectiveMappedCells * resolution*resolution;
	if (info.effectiveMappedCells)
	{
		info.mean_H = info.H / info.effectiveMappedCells;
		info.mean_I = info.I / info.effectiveMappedCells;
	}
	else
	{
		info.mean_H = 0;
		info.mean_I = 0;
	}
}

/*---------------------------------------------------------------
					Entropy aux. function
 ---------------------------------------------------------------*/
double  COccupancyGridMap2D::H(double p)
{
	if (p==0 || p==1)	return 0;
	else				return -p*log(p);
}

/*---------------------------------------------------------------
					loadFromBitmapFile
 Load a 8-bits, black & white bitmap file as a grid map. It will be loaded such as coordinates (0,0) falls just in the middle of map.
\param file The file to be loaded.
\param resolution The size of a pixel (cell), in meters. Recall cells are always squared, so just a dimension is needed.
\return False on any error.
 ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::loadFromBitmapFile(
	const std::string	&file,
	float			resolution,
	float			xCentralPixel,
	float			yCentralPixel)
{
	MRPT_START;

	CImage		imgFl;
	if (!imgFl.loadFromFile(file,0))
		return false;

	m_is_empty = false;
	return loadFromBitmap(imgFl,resolution, xCentralPixel, yCentralPixel);

	MRPT_END;
}

/*---------------------------------------------------------------
					loadFromBitmap
 ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::loadFromBitmap(const mrpt::utils::CImage &imgFl, float resolution, float xCentralPixel, float yCentralPixel)
{
	MRPT_START;

	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;

	size_t bmpWidth = imgFl.getWidth();
	size_t bmpHeight = imgFl.getHeight();

	if (size_x!=bmpWidth || size_y!=bmpHeight)
	{
		// Middle of bitmap?
		if (xCentralPixel<-1 || yCentralPixel<=-1)
		{
			xCentralPixel = imgFl.getWidth() / 2.0f;
			yCentralPixel = imgFl.getHeight() / 2.0f;
		}

		// Resize grid:
		float new_x_max = (imgFl.getWidth() - xCentralPixel) * resolution;
		float new_x_min = - xCentralPixel * resolution;
		float new_y_max = (imgFl.getHeight() - yCentralPixel) * resolution;
		float new_y_min = - yCentralPixel * resolution;

		setSize(new_x_min,new_x_max,new_y_min,new_y_max,resolution);
	}

	// And load cells content:
	for (size_t x=0;x<bmpWidth;x++)
		for (size_t y=0;y<bmpHeight;y++)
		{
			float f = imgFl.getAsFloat(x,bmpHeight-1-y);
			f = std::max(0.01f,f);
			f = std::min(0.99f,f);
			setCell(x,y,f);
		}

	m_is_empty = false;
	return true;

	MRPT_END;
}

/** Local stucture used in the next method */
struct TLocalPoint
{
	float x,y; int cx, cy;
};

/*---------------------------------------------------------------
					insertObservation

Insert the observation information into this map.
 ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::internal_insertObservation(
		const CObservation	*obs,
		const CPose3D			*robotPose)
{
// 	MRPT_START;   // Avoid "try" since we use "alloca"

#define FRBITS	9

	CPose2D		robotPose2D;
	CPose3D		robotPose3D;

	// This is required to indicate the grid map has changed!
	//resetFeaturesCache();
	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;

	if (robotPose)
	{
		robotPose2D = CPose2D(*robotPose);
		robotPose3D = (*robotPose);
	}
	else
	{
		// Default values are (0,0,0)
	}

	if ( CLASS_ID(CObservation2DRangeScan)==obs->GetRuntimeClass())
	{
	/********************************************************************

				OBSERVATION TYPE: CObservation2DRangeScan

		********************************************************************/
		const CObservation2DRangeScan	*o = static_cast<const CObservation2DRangeScan*>( obs );
		CPose3D						sensorPose3D = robotPose3D + o->sensorPose;
		CPose2D						laserPose( sensorPose3D );

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		bool		reallyInsert = o->isPlanarScan( insertionOptions.horizontalTolerance );
		unsigned int decimation = insertionOptions.decimation;

		// Check the altitude of the map (if feature enabled!)
		if ( insertionOptions.useMapAltitude &&
				fabs(insertionOptions.mapAltitude - sensorPose3D.z() ) > 0.001 )
		{
			reallyInsert = false;
		}

		// Manage horizontal scans, but with the sensor bottom-up:
		//  Use the z-axis direction of the transformed Z axis of the sensor coordinates:
		bool sensorIsBottomwards = sensorPose3D.getHomogeneousMatrixVal().get_unsafe(2,2) < 0;

		if ( reallyInsert )
		{
			// ---------------------------------------------
			//		Insert the scan as simple rays:
			// ---------------------------------------------
			int								cx,cy,N =  o->scan.size();
			float							px,py;
			double							A, dAK;

			// Parameters values:
			const float 	maxDistanceInsertion 	= insertionOptions.maxDistanceInsertion;
			const bool		invalidAsFree			= insertionOptions.considerInvalidRangesAsFreeSpace;
			float		new_x_max, new_x_min;
			float		new_y_max, new_y_min;
			float		last_valid_range	= maxDistanceInsertion;

			float		maxCertainty		= insertionOptions.maxOccupancyUpdateCertainty;
			cellType    logodd_observation  = p2l(maxCertainty);
			cellType    logodd_observation_occupied = 3*logodd_observation;

			// Assure minimum change in cells!
			if (logodd_observation<=0)
				logodd_observation=1;

			cellType    logodd_thres_occupied = OCCGRID_CELLTYPE_MIN+logodd_observation_occupied;
			cellType    logodd_thres_free     = OCCGRID_CELLTYPE_MAX-logodd_observation;


			int		K = updateInfoChangeOnly.enabled ? updateInfoChangeOnly.laserRaysSkip : decimation;
			size_t	idx,nRanges = o->scan.size();
			float	curRange=0;

			// Start position:
			px = laserPose.x();
			py = laserPose.y();

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
			MRPT_CHECK_NORMAL_NUMBER(px);
			MRPT_CHECK_NORMAL_NUMBER(py);
#endif

			// Here we go! Now really insert changes in the grid:
			if ( !insertionOptions.wideningBeamsWithDistance )
			{
				// Method: Simple rays:
				// -------------------------------------

				// Reserve a temporary block of memory on the stack with "alloca": this memory has NOT to be deallocated,
				//  so it's ideal for an efficient, small buffer:
				float	*scanPoints_x = (float*) mrpt_alloca( sizeof(float) * nRanges );
				float	*scanPoints_y = (float*) mrpt_alloca( sizeof(float) * nRanges );

				float 	*scanPoint_x,*scanPoint_y;


				if (o->rightToLeft ^ sensorIsBottomwards )
				{
					A  = laserPose.phi() - 0.5 * o->aperture;
					dAK = K* o->aperture / N;
				}
				else
				{
					A  = laserPose.phi() + 0.5 * o->aperture;
					dAK = - K*o->aperture / N;
				}


				new_x_max = -(numeric_limits<float>::max)();
				new_x_min =  (numeric_limits<float>::max)();
				new_y_max = -(numeric_limits<float>::max)();
				new_y_min =  (numeric_limits<float>::max)();

				for (idx=0, scanPoint_x=scanPoints_x,scanPoint_y=scanPoints_y;idx<nRanges;idx+=K,scanPoint_x++,scanPoint_y++)
				{
					if ( o->validRange[idx] )
					{
						curRange = o->scan[idx];
						float R = min(maxDistanceInsertion,curRange);

						*scanPoint_x = px + cos(A)* R;
						*scanPoint_y = py + sin(A)* R;
						last_valid_range = curRange;
					}
					else
					{
						if (invalidAsFree)
						{
							// Invalid range:
							float R = min(maxDistanceInsertion,0.5f*last_valid_range);
							*scanPoint_x = px + cos(A)* R;
							*scanPoint_y = py + sin(A)* R;
						}
						else
						{
							*scanPoint_x = px;
							*scanPoint_y = py;
						}
					}
					A+=dAK;

					// Asjust size (will not change if not required):
					new_x_max = max( new_x_max, *scanPoint_x );
					new_x_min = min( new_x_min, *scanPoint_x );
					new_y_max = max( new_y_max, *scanPoint_y );
					new_y_min = min( new_y_min, *scanPoint_y );
				}

				// Add an extra margin:
				float securMargen = 15*resolution;

				if (new_x_max>x_max-securMargen)
						new_x_max+= 2*securMargen;
				else	new_x_max = x_max;
				if (new_x_min<x_min+securMargen)
						new_x_min-= 2;
				else	new_x_min = x_min;

				if (new_y_max>y_max-securMargen)
						new_y_max+= 2*securMargen;
				else	new_y_max = y_max;
				if (new_y_min<y_min+securMargen)
						new_y_min-= 2;
				else	new_y_min = y_min;

				// -----------------------
				//   Resize to make room:
				// -----------------------
				resizeGrid(new_x_min,new_x_max, new_y_min,new_y_max,0.5);

				// For updateCell_fast methods:
				cellType  *theMapArray = &map[0];
				unsigned  theMapSize_x = size_x;

				int  cx0 = x2idx(px);		// Remember: This must be after the resizeGrid!!
				int  cy0 = y2idx(py);


				// Insert rays:
				for (idx=0;idx<nRanges;idx+=K)
				{
					if ( !o->validRange[idx] && !invalidAsFree ) continue;

					// Starting position: Laser position
					cx = cx0;
					cy = cy0;

					// Target, in cell indexes:
					int trg_cx = x2idx(scanPoints_x[idx]);
					int trg_cy = y2idx(scanPoints_y[idx]);

	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
					// The x> comparison implicitly holds if x<0
					ASSERT_( static_cast<unsigned int>(trg_cx)<size_x && static_cast<unsigned int>(trg_cy)<size_y );
	#endif

					// Use "fractional integers" to approximate float operations
					//  during the ray tracing:
					int Acx  = trg_cx - cx;
					int Acy  = trg_cy - cy;

					int Acx_ = abs(Acx);
					int Acy_ = abs(Acy);

					int nStepsRay = max( Acx_, Acy_ );
					if (!nStepsRay) continue; // May be...

					// Integers store "float values * 128"
					float  N_1 = 1.0f / nStepsRay;   // Avoid division twice.

					// Increments at each raytracing step:
					int  frAcx = round( (Acx<< FRBITS) * N_1 );  //  Acx*128 / N
					int  frAcy = round( (Acy<< FRBITS) * N_1 );  //  Acy*128 / N

					int frCX = cx << FRBITS;
					int frCY = cy << FRBITS;

					for (int nStep = 0;nStep<nStepsRay;nStep++)
					{
						updateCell_fast_free(cx,cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );

						frCX += frAcx;
						frCY += frAcy;

						cx = frCX >> FRBITS;
						cy = frCY >> FRBITS;
					}

					// And finally, the occupied cell at the end:
					// Only if:
					//  - It was a valid ray, and
					//  - The ray was not truncated
					if ( o->validRange[idx] && o->scan[idx]<maxDistanceInsertion )
						updateCell_fast_occupied(trg_cx,trg_cy, logodd_observation_occupied, logodd_thres_occupied, theMapArray, theMapSize_x );

				}  // End of each range

				mrpt_alloca_free( scanPoints_x );
				mrpt_alloca_free( scanPoints_y );

			}  // end insert with simple rays
			else
			{
				// ---------------------------------
				//  		Widen rays
				// Algorithm in: http://www.mrpt.org/Occupancy_Grids
				// ---------------------------------
				if (o->rightToLeft ^ sensorIsBottomwards )
				{
					A  = laserPose.phi() - 0.5 * o->aperture;
					dAK = K* o->aperture / N;
				}
				else
				{
					A  = laserPose.phi() + 0.5 * o->aperture;
					dAK = - K*o->aperture / N;
				}

				new_x_max = -(numeric_limits<float>::max)();
				new_x_min =  (numeric_limits<float>::max)();
				new_y_max = -(numeric_limits<float>::max)();
				new_y_min =  (numeric_limits<float>::max)();

				last_valid_range	= maxDistanceInsertion;
				for (idx=0;idx<nRanges;idx+=K)
				{
					float scanPoint_x,scanPoint_y;
					if ( o->validRange[idx] )
					{
						curRange = o->scan[idx];
						float R = min(maxDistanceInsertion,curRange);

						scanPoint_x = px + cos(A)* R;
						scanPoint_y = py + sin(A)* R;
						last_valid_range = curRange;
					}
					else
					{
						if (invalidAsFree)
						{
							// Invalid range:
							float R = min(maxDistanceInsertion,0.5f*last_valid_range);
							scanPoint_x = px + cos(A)* R;
							scanPoint_y = py + sin(A)* R;
						}
						else
						{
							scanPoint_x = px;
							scanPoint_y = py;
						}
					}
					A+=dAK;

					// Asjust size (will not change if not required):
					new_x_max = max( new_x_max, scanPoint_x );
					new_x_min = min( new_x_min, scanPoint_x );
					new_y_max = max( new_y_max, scanPoint_y );
					new_y_min = min( new_y_min, scanPoint_y );
				}

				// Add an extra margin:
				float securMargen = 15*resolution;

				if (new_x_max>x_max-securMargen)
						new_x_max+= 2*securMargen;
				else	new_x_max = x_max;
				if (new_x_min<x_min+securMargen)
						new_x_min-= 2;
				else	new_x_min = x_min;

				if (new_y_max>y_max-securMargen)
						new_y_max+= 2*securMargen;
				else	new_y_max = y_max;
				if (new_y_min<y_min+securMargen)
						new_y_min-= 2;
				else	new_y_min = y_min;

				// -----------------------
				//   Resize to make room:
				// -----------------------
				resizeGrid(new_x_min,new_x_max, new_y_min,new_y_max,0.5);

				// For updateCell_fast methods:
				cellType  *theMapArray = &map[0];
				unsigned  theMapSize_x = size_x;

				//int  cx0 = x2idx(px);		// Remember: This must be after the resizeGrid!!
				//int  cy0 = y2idx(py);


				// Now go and insert the triangles of each beam:
				// -----------------------------------------------
				if (o->rightToLeft ^ sensorIsBottomwards )
				{
					A  = laserPose.phi() - 0.5 * o->aperture;
					dAK = K* o->aperture / N;
				}
				else
				{
					A  = laserPose.phi() + 0.5 * o->aperture;
					dAK = - K*o->aperture / N;
				}

				// Insert the rays:
				// ------------------------------------------
				// Vertices of the triangle: In meters
				TLocalPoint P0,P1,P2, P1b;

				last_valid_range	= maxDistanceInsertion;

				const double dA_2 = 0.5 * o->aperture / N;
				for (idx=0;idx<nRanges; idx+=K, A+=dAK)
				{
					float	theR;		// The range of this beam
					if ( o->validRange[idx] )
					{
						curRange = o->scan[idx];
						last_valid_range = curRange;
						theR = min(maxDistanceInsertion,curRange);
					}
					else
					{
						// Invalid range:
						if (invalidAsFree)
						{
							theR = min(maxDistanceInsertion,0.5f*last_valid_range);
						}
						else continue; // Nothing to do
					}
					if (theR < resolution) continue; // Range must be larger than a cell...
					theR -= resolution;	// Remove one cell of length, which will be filled with "occupied" later.

					/* ---------------------------------------------------------
					      Fill one triangle with vertices: P0,P1,P2
					   --------------------------------------------------------- */
					P0.x = px;
					P0.y = py;

					P1.x = px + cos(A-dA_2) * theR;
					P1.y = py + sin(A-dA_2) * theR;

					P2.x = px + cos(A+dA_2) * theR;
					P2.y = py + sin(A+dA_2) * theR;

					// Order the vertices by the "y": P0->bottom, P2: top
					if (P2.y<P1.y) std::swap(P2,P1);
					if (P2.y<P0.y) std::swap(P2,P0);
					if (P1.y<P0.y) std::swap(P1,P0);


					// In cell indexes:
					P0.cx = x2idx( P0.x );	P0.cy = y2idx( P0.y );
					P1.cx = x2idx( P1.x );	P1.cy = y2idx( P1.y );
					P2.cx = x2idx( P2.x );	P2.cy = y2idx( P2.y );

	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
					// The x> comparison implicitly holds if x<0
					ASSERT_( static_cast<unsigned int>(P0.cx)<size_x && static_cast<unsigned int>(P0.cy)<size_y );
					ASSERT_( static_cast<unsigned int>(P1.cx)<size_x && static_cast<unsigned int>(P1.cy)<size_y );
					ASSERT_( static_cast<unsigned int>(P2.cx)<size_x && static_cast<unsigned int>(P2.cy)<size_y );
	#endif

					struct { int frX,frY; int cx,cy; } R1,R2;	// Fractional coords of the two rays:

					// Special case: one single row
					if (P0.cy==P2.cy && P0.cy==P1.cy)
					{
						// Optimized case:
						int min_cx = min3(P0.cx,P1.cx,P2.cx);
						int max_cx = max3(P0.cx,P1.cx,P2.cx);

						for (int ccx=min_cx;ccx<=max_cx;ccx++)
							updateCell_fast_free(ccx,P0.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
					}
					else
					{
						// The intersection point P1b in the segment P0-P2 at the "y" of P1:
						P1b.y = P1.y;
						P1b.x = P0.x + (P1.y-P0.y) * (P2.x-P0.x) / (P2.y-P0.y);

						P1b.cx= x2idx( P1b.x );	P1b.cy= y2idx( P1b.y );


						// Use "fractional integers" to approximate float operations during the ray tracing:
						// Integers store "float values * 128"
						const int Acx01 = P1.cx - P0.cx;
						const int Acy01 = P1.cy - P0.cy;
						const int Acx01b = P1b.cx - P0.cx;
						//const int Acy01b = P1b.cy - P0.cy;  // = Acy01

						// Increments at each raytracing step:
						const float inv_N_01 = 1.0f / ( max3(abs(Acx01),abs(Acy01),abs(Acx01b)) + 1 );	// Number of steps ^ -1
						const int  frAcx01 = round( (Acx01<< FRBITS) * inv_N_01 );  //  Acx*128 / N
						const int  frAcy01 = round( (Acy01<< FRBITS) * inv_N_01 );  //  Acy*128 / N
						const int  frAcx01b = round((Acx01b<< FRBITS)* inv_N_01 );  //  Acx*128 / N

						// ------------------------------------
						// First sub-triangle: P0-P1-P1b
						// ------------------------------------
						R1.cx  = P0.cx;
						R1.cy  = P0.cy;
						R1.frX = P0.cx << FRBITS;
						R1.frY = P0.cy << FRBITS;

						int frAx_R1=0, frAx_R2=0; //, frAy_R2;
						int frAy_R1 = frAcy01;

						// Start R1=R2 = P0... unlesss P0.cy == P1.cy, i.e. there is only one row:
						if (P0.cy!=P1.cy)
						{
							R2 = R1;
							//  R1 & R2 follow the edges: P0->P1  & P0->P1b
							//  R1 is forced to be at the left hand:
							if (P1.x<P1b.x)
							{
								// R1: P0->P1
								frAx_R1 = frAcx01;
								frAx_R2 = frAcx01b;
							}
							else
							{
								// R1: P0->P1b
								frAx_R1 = frAcx01b;
								frAx_R2 = frAcx01;
							}
						}
						else
						{
							R2.cx  = P1.cx;
							R2.cy  = P1.cy;
							R2.frX = P1.cx << FRBITS;
							//R2.frY = P1.cy << FRBITS;
						}

						int last_insert_cy = -1;
						//int last_insert_cx = -1;
						do
						{
							if (last_insert_cy!=R1.cy) // || last_insert_cx!=R1.cx)
							{
								last_insert_cy = R1.cy;
							//	last_insert_cx = R1.cx;

								for (int ccx=R1.cx;ccx<=R2.cx;ccx++)
									updateCell_fast_free(ccx,R1.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
							}

							R1.frX += frAx_R1;    R1.frY += frAy_R1;
							R2.frX += frAx_R2;    // R1.frY += frAcy01;

							R1.cx = R1.frX >> FRBITS;
							R1.cy = R1.frY >> FRBITS;
							R2.cx = R2.frX >> FRBITS;
						} while ( R1.cy < P1.cy );

						// ------------------------------------
						// Second sub-triangle: P1-P1b-P2
						// ------------------------------------

						// Use "fractional integers" to approximate float operations during the ray tracing:
						// Integers store "float values * 128"
						const int Acx12  = P2.cx - P1.cx;
						const int Acy12  = P2.cy - P1.cy;
						const int Acx1b2 = P2.cx - P1b.cx;
						//const int Acy1b2 = Acy12

						// Increments at each raytracing step:
						const float inv_N_12 = 1.0f / ( max3(abs(Acx12),abs(Acy12),abs(Acx1b2)) + 1 );	// Number of steps ^ -1
						const int  frAcx12 = round( (Acx12<< FRBITS) * inv_N_12 );  //  Acx*128 / N
						const int  frAcy12 = round( (Acy12<< FRBITS) * inv_N_12 );  //  Acy*128 / N
						const int  frAcx1b2 = round((Acx1b2<< FRBITS)* inv_N_12 );  //  Acx*128 / N

						//struct { int frX,frY; int cx,cy; } R1,R2;	// Fractional coords of the two rays:
						// R1, R2 follow edges P1->P2 & P1b->P2
						// R1 forced to be at the left hand
						frAy_R1 = frAcy12;
						if (!frAy_R1)
							frAy_R1 = 2 << FRBITS;	// If Ay=0, force it to be >0 so the "do...while" loop below ends in ONE iteration.

						if (P1.x<P1b.x)
						{
							// R1: P1->P2,  R2: P1b->P2
							R1.cx  = P1.cx;
							R1.cy  = P1.cy;
							R2.cx  = P1b.cx;
							R2.cy  = P1b.cy;
							frAx_R1 = frAcx12;
							frAx_R2 = frAcx1b2;
						}
						else
						{
							// R1: P1b->P2,  R2: P1->P2
							R1.cx  = P1b.cx;
							R1.cy  = P1b.cy;
							R2.cx  = P1.cx;
							R2.cy  = P1.cy;
							frAx_R1 = frAcx1b2;
							frAx_R2 = frAcx12;
						}

						R1.frX = R1.cx << FRBITS;
						R1.frY = R1.cy << FRBITS;
						R2.frX = R2.cx << FRBITS;
						R2.frY = R2.cy << FRBITS;

						last_insert_cy=-100;
						//last_insert_cx=-100;

						do
						{
							if (last_insert_cy!=R1.cy) // || last_insert_cx!=R1.cx)
							{
							//	last_insert_cx = R1.cx;
								last_insert_cy = R1.cy;
								for (int ccx=R1.cx;ccx<=R2.cx;ccx++)
									updateCell_fast_free(ccx,R1.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
							}

							R1.frX += frAx_R1;    R1.frY += frAy_R1;
							R2.frX += frAx_R2;    // R1.frY += frAcy01;

							R1.cx = R1.frX >> FRBITS;
							R1.cy = R1.frY >> FRBITS;
							R2.cx = R2.frX >> FRBITS;
						} while ( R1.cy <= P2.cy );

					} // end of free-area normal case (not a single row)

					// ----------------------------------------------------
					// The final occupied cells along the edge P1<->P2
					// Only if:
					//  - It was a valid ray, and
					//  - The ray was not truncated
					// ----------------------------------------------------
					if ( o->validRange[idx] && o->scan[idx]<maxDistanceInsertion )
					{
						theR += resolution;

						P1.x = px + cos(A-dA_2) * theR;
						P1.y = py + sin(A-dA_2) * theR;

						P2.x = px + cos(A+dA_2) * theR;
						P2.y = py + sin(A+dA_2) * theR;

						P1.cx = x2idx( P1.x );	P1.cy = y2idx( P1.y );
						P2.cx = x2idx( P2.x );	P2.cy = y2idx( P2.y );

		#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
						// The x> comparison implicitly holds if x<0
						ASSERT_( static_cast<unsigned int>(P1.cx)<size_x && static_cast<unsigned int>(P1.cy)<size_y );
						ASSERT_( static_cast<unsigned int>(P2.cx)<size_x && static_cast<unsigned int>(P2.cy)<size_y );
		#endif

						// Special case: Only one cell:
						if (P2.cx==P1.cx && P2.cy==P1.cy)
						{
							updateCell_fast_occupied(P1.cx,P1.cy, logodd_observation_occupied, logodd_thres_occupied, theMapArray, theMapSize_x );
						}
						else
						{
							// Use "fractional integers" to approximate float operations during the ray tracing:
							// Integers store "float values * 128"
							const int AcxE  = P2.cx - P1.cx;
							const int AcyE  = P2.cy - P1.cy;

							// Increments at each raytracing step:
							const int nSteps = ( max(abs(AcxE),abs(AcyE)) + 1 );
							const float inv_N_12 = 1.0f / nSteps;	// Number of steps ^ -1
							const int  frAcxE = round( (AcxE<< FRBITS) * inv_N_12 );  //  Acx*128 / N
							const int  frAcyE = round( (AcyE<< FRBITS) * inv_N_12 );  //  Acy*128 / N

							R1.cx  = P1.cx;
							R1.cy  = P1.cy;
							R1.frX = R1.cx << FRBITS;
							R1.frY = R1.cy << FRBITS;

							for (int nStep=0;nStep<=nSteps;nStep++)
							{
								updateCell_fast_occupied(R1.cx,R1.cy, logodd_observation_occupied, logodd_thres_occupied, theMapArray, theMapSize_x );

								R1.frX += frAcxE;
								R1.frY += frAcyE;
								R1.cx = R1.frX >> FRBITS;
								R1.cy = R1.frY >> FRBITS;
							}

						} // end do a line

					} // end if we must set occupied cells

				}  // End of each range

			}  // end insert with beam widening

			// Finished:
			return true;
		}
		else
		{
			// A non-horizontal scan:
			return false;
		}
	}
	else if ( CLASS_ID(CObservationRange)==obs->GetRuntimeClass())
	{
		const CObservationRange *o = static_cast<const CObservationRange*>( obs );
		CPose3D spose;
		o->getSensorPose(spose);
		CPose3D						sensorPose3D = robotPose3D + spose;
		CPose2D						laserPose( sensorPose3D );

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		bool		reallyInsert = true;
		unsigned int decimation = insertionOptions.decimation;

		// Check the altitude of the map (if feature enabled!)
		if ( insertionOptions.useMapAltitude &&
				fabs(insertionOptions.mapAltitude - sensorPose3D.z() ) > 0.001 )
		{
			reallyInsert = false;
		}
	    if ( reallyInsert )
		{
		    // ---------------------------------------------
			//		Insert the scan as simple rays:
			// ---------------------------------------------

			//int		/*cx,cy,*/ N =  o->sensedData.size();
			float	px,py;
			double	A, dAK;

			// Parameters values:
			const float 	maxDistanceInsertion 	= insertionOptions.maxDistanceInsertion;
			const bool		invalidAsFree			= insertionOptions.considerInvalidRangesAsFreeSpace;
			float		new_x_max, new_x_min;
			float		new_y_max, new_y_min;
			float		last_valid_range	= maxDistanceInsertion;

			float		maxCertainty		= insertionOptions.maxOccupancyUpdateCertainty;
			cellType    logodd_observation  = p2l(maxCertainty);
			cellType    logodd_observation_occupied = 3*logodd_observation;

			// Assure minimum change in cells!
			if (logodd_observation<=0)
				logodd_observation=1;

			cellType    logodd_thres_occupied = OCCGRID_CELLTYPE_MIN+logodd_observation_occupied;
			cellType    logodd_thres_free     = OCCGRID_CELLTYPE_MAX-logodd_observation;


			int		K = updateInfoChangeOnly.enabled ? updateInfoChangeOnly.laserRaysSkip : decimation;
			size_t	idx,nRanges = o->sensedData.size();
			float	curRange=0;

			// Start position:
			px = laserPose.x();
			py = laserPose.y();

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
			MRPT_CHECK_NORMAL_NUMBER(px);
			MRPT_CHECK_NORMAL_NUMBER(py);
#endif
			// ---------------------------------
			//  		Widen rays
			// Algorithm in: http://www.mrpt.org/Occupancy_Grids
			// FIXME: doesn't support many different poses in one measurement
			// ---------------------------------
			A  = laserPose.phi();
			dAK = 0;

			new_x_max = -(numeric_limits<float>::max)();
			new_x_min =  (numeric_limits<float>::max)();
			new_y_max = -(numeric_limits<float>::max)();
			new_y_min =  (numeric_limits<float>::max)();

			last_valid_range	= maxDistanceInsertion;

			for (idx=0;idx<nRanges;idx+=K)
			{
				float scanPoint_x,scanPoint_y;
				if ( o->sensedData[idx].sensedDistance < maxDistanceInsertion )
				{
					curRange = o->sensedData[idx].sensedDistance;
					float R = min(maxDistanceInsertion,curRange);

					scanPoint_x = px + cos(A)* R;
					scanPoint_y = py + sin(A)* R;
					last_valid_range = curRange;
				}
				else
				{
					if (invalidAsFree)
					{
						// Invalid range:
						float R = min(maxDistanceInsertion,0.5f*last_valid_range);
						scanPoint_x = px + cos(A)* R;
						scanPoint_y = py + sin(A)* R;
					}
					else
					{
						scanPoint_x = px;
						scanPoint_y = py;
					}
				}
				A+=dAK;

				// Asjust size (will not change if not required):
				new_x_max = max( new_x_max, scanPoint_x );
				new_x_min = min( new_x_min, scanPoint_x );
				new_y_max = max( new_y_max, scanPoint_y );
				new_y_min = min( new_y_min, scanPoint_y );
			}

			// Add an extra margin:
			float securMargen = 15*resolution;

			if (new_x_max>x_max-securMargen)
					new_x_max+= 2*securMargen;
			else	new_x_max = x_max;
			if (new_x_min<x_min+securMargen)
					new_x_min-= 2;
			else	new_x_min = x_min;

			if (new_y_max>y_max-securMargen)
					new_y_max+= 2*securMargen;
			else	new_y_max = y_max;
			if (new_y_min<y_min+securMargen)
					new_y_min-= 2;
			else	new_y_min = y_min;

			// -----------------------
			//   Resize to make room:
			// -----------------------
			resizeGrid(new_x_min,new_x_max, new_y_min,new_y_max,0.5);

			// For updateCell_fast methods:
			cellType  *theMapArray = &map[0];
			unsigned  theMapSize_x = size_x;

			//int  cx0 = x2idx(px);		// Remember: This must be after the resizeGrid!!
			//int  cy0 = y2idx(py);


			// Now go and insert the triangles of each beam:
			// -----------------------------------------------
			A  = laserPose.phi() - 0.5 * o->sensorConeApperture;
			dAK = 0;

			// Insert the rays:
			// ------------------------------------------
			// Vertices of the triangle: In meters
			TLocalPoint P0,P1,P2, P1b;

			last_valid_range	= maxDistanceInsertion;

			const double dA_2 = 0.5 * o->sensorConeApperture;
			for (idx=0;idx<nRanges; idx+=K, A+=dAK)
			{
				float	theR;		// The range of this beam
				if ( o->sensedData[idx].sensedDistance < maxDistanceInsertion )
				{
					curRange = o->sensedData[idx].sensedDistance;
					last_valid_range = curRange;
					theR = min(maxDistanceInsertion,curRange);
				}
				else
				{
					// Invalid range:
					if (invalidAsFree)
					{
						theR = min(maxDistanceInsertion,0.5f*last_valid_range);
					}
					else continue; // Nothing to do
				}
				if (theR < resolution) continue; // Range must be larger than a cell...
				theR -= resolution;	// Remove one cell of length, which will be filled with "occupied" later.

				/* ---------------------------------------------------------
				      Fill one triangle with vertices: P0,P1,P2
				   --------------------------------------------------------- */
				P0.x = px;
				P0.y = py;

				P1.x = px + cos(A-dA_2) * theR;
				P1.y = py + sin(A-dA_2) * theR;

				P2.x = px + cos(A+dA_2) * theR;
				P2.y = py + sin(A+dA_2) * theR;

				// Order the vertices by the "y": P0->bottom, P2: top
				if (P2.y<P1.y) std::swap(P2,P1);
				if (P2.y<P0.y) std::swap(P2,P0);
				if (P1.y<P0.y) std::swap(P1,P0);


				// In cell indexes:
				P0.cx = x2idx( P0.x );	P0.cy = y2idx( P0.y );
				P1.cx = x2idx( P1.x );	P1.cy = y2idx( P1.y );
				P2.cx = x2idx( P2.x );	P2.cy = y2idx( P2.y );

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
				// The x> comparison implicitly holds if x<0
				ASSERT_( static_cast<unsigned int>(P0.cx)<size_x && static_cast<unsigned int>(P0.cy)<size_y );
				ASSERT_( static_cast<unsigned int>(P1.cx)<size_x && static_cast<unsigned int>(P1.cy)<size_y );
				ASSERT_( static_cast<unsigned int>(P2.cx)<size_x && static_cast<unsigned int>(P2.cy)<size_y );
#endif

				struct { int frX,frY; int cx,cy; } R1,R2;	// Fractional coords of the two rays:

				// Special case: one single row
				if (P0.cy==P2.cy && P0.cy==P1.cy)
				{
					// Optimized case:
					int min_cx = min3(P0.cx,P1.cx,P2.cx);
					int max_cx = max3(P0.cx,P1.cx,P2.cx);

					for (int ccx=min_cx;ccx<=max_cx;ccx++)
						updateCell_fast_free(ccx,P0.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
				}
				else
				{
					// The intersection point P1b in the segment P0-P2 at the "y" of P1:
					P1b.y = P1.y;
					P1b.x = P0.x + (P1.y-P0.y) * (P2.x-P0.x) / (P2.y-P0.y);

					P1b.cx= x2idx( P1b.x );	P1b.cy= y2idx( P1b.y );


					// Use "fractional integers" to approximate float operations during the ray tracing:
					// Integers store "float values * 128"
					const int Acx01 = P1.cx - P0.cx;
					const int Acy01 = P1.cy - P0.cy;
					const int Acx01b = P1b.cx - P0.cx;
					//const int Acy01b = P1b.cy - P0.cy;  // = Acy01

					// Increments at each raytracing step:
					const float inv_N_01 = 1.0f / ( max3(abs(Acx01),abs(Acy01),abs(Acx01b)) + 1 );	// Number of steps ^ -1
					const int  frAcx01 = round( (Acx01<<FRBITS) * inv_N_01 );  //  Acx*128 / N
					const int  frAcy01 = round( (Acy01<<FRBITS) * inv_N_01 );  //  Acy*128 / N
					const int  frAcx01b = round((Acx01b<<FRBITS)* inv_N_01 );  //  Acx*128 / N

					// ------------------------------------
					// First sub-triangle: P0-P1-P1b
					// ------------------------------------
					R1.cx  = P0.cx;
					R1.cy  = P0.cy;
					R1.frX = P0.cx <<FRBITS;
					R1.frY = P0.cy <<FRBITS;

					int frAx_R1=0, frAx_R2=0; //, frAy_R2;
					int frAy_R1 = frAcy01;

					// Start R1=R2 = P0... unlesss P0.cy == P1.cy, i.e. there is only one row:
					if (P0.cy!=P1.cy)
					{
						R2 = R1;
						//  R1 & R2 follow the edges: P0->P1  & P0->P1b
						//  R1 is forced to be at the left hand:
						if (P1.x<P1b.x)
						{
							// R1: P0->P1
							frAx_R1 = frAcx01;
							frAx_R2 = frAcx01b;
						}
						else
						{
							// R1: P0->P1b
							frAx_R1 = frAcx01b;
							frAx_R2 = frAcx01;
						}
					}
					else
					{
						R2.cx  = P1.cx;
						R2.cy  = P1.cy;
						R2.frX = P1.cx <<FRBITS;
						//R2.frY = P1.cy <<FRBITS;
					}
					int last_insert_cy = -1;
					//int last_insert_cx = -1;
					do
					{
						if (last_insert_cy!=R1.cy) // || last_insert_cx!=R1.cx)
						{
							last_insert_cy = R1.cy;
						//	last_insert_cx = R1.cx;

							for (int ccx=R1.cx;ccx<=R2.cx;ccx++)
								updateCell_fast_free(ccx,R1.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
						}

						R1.frX += frAx_R1;    R1.frY += frAy_R1;
						R2.frX += frAx_R2;    // R1.frY += frAcy01;

						R1.cx = R1.frX >> FRBITS;
						R1.cy = R1.frY >> FRBITS;
						R2.cx = R2.frX >> FRBITS;
					} while ( R1.cy < P1.cy );
					// ------------------------------------
					// Second sub-triangle: P1-P1b-P2
					// ------------------------------------

					// Use "fractional integers" to approximate float operations during the ray tracing:
					// Integers store "float values * 128"
					const int Acx12  = P2.cx - P1.cx;
					const int Acy12  = P2.cy - P1.cy;
					const int Acx1b2 = P2.cx - P1b.cx;
					//const int Acy1b2 = Acy12

					// Increments at each raytracing step:
					const float inv_N_12 = 1.0f / ( max3(abs(Acx12),abs(Acy12),abs(Acx1b2)) + 1 );	// Number of steps ^ -1
					const int  frAcx12 = round( (Acx12<<FRBITS) * inv_N_12 );  //  Acx*128 / N
					const int  frAcy12 = round( (Acy12<<FRBITS) * inv_N_12 );  //  Acy*128 / N
					const int  frAcx1b2 = round((Acx1b2<<FRBITS)* inv_N_12 );  //  Acx*128 / N

					//struct { int frX,frY; int cx,cy; } R1,R2;	// Fractional coords of the two rays:
					// R1, R2 follow edges P1->P2 & P1b->P2
					// R1 forced to be at the left hand
					frAy_R1 = frAcy12;
					if (!frAy_R1)
						frAy_R1 = 2 <<FRBITS;	// If Ay=0, force it to be >0 so the "do...while" loop below ends in ONE iteration.

					if (P1.x<P1b.x)
					{
						// R1: P1->P2,  R2: P1b->P2
						R1.cx  = P1.cx;
						R1.cy  = P1.cy;
						R2.cx  = P1b.cx;
						R2.cy  = P1b.cy;
						frAx_R1 = frAcx12;
						frAx_R2 = frAcx1b2;
					}
					else
					{
						// R1: P1b->P2,  R2: P1->P2
						R1.cx  = P1b.cx;
						R1.cy  = P1b.cy;
						R2.cx  = P1.cx;
						R2.cy  = P1.cy;
						frAx_R1 = frAcx1b2;
						frAx_R2 = frAcx12;
					}

					R1.frX = R1.cx <<FRBITS;
					R1.frY = R1.cy <<FRBITS;
					R2.frX = R2.cx <<FRBITS;
					R2.frY = R2.cy <<FRBITS;

					last_insert_cy=-100;
					//last_insert_cx=-100;

					do
					{
						if (last_insert_cy!=R1.cy) // || last_insert_cx!=R1.cx)
						{
						//	last_insert_cx = R1.cx;
							last_insert_cy = R1.cy;
							for (int ccx=R1.cx;ccx<=R2.cx;ccx++)
								updateCell_fast_free(ccx,R1.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
						}

						R1.frX += frAx_R1;    R1.frY += frAy_R1;
						R2.frX += frAx_R2;    // R1.frY += frAcy01;

						R1.cx = R1.frX >> FRBITS;
						R1.cy = R1.frY >> FRBITS;
						R2.cx = R2.frX >> FRBITS;
					} while ( R1.cy <= P2.cy );

				} // end of free-area normal case (not a single row)

				// ----------------------------------------------------
				// The final occupied cells along the edge P1<->P2
				// Only if:
				//  - It was a valid ray, and
				//  - The ray was not truncated
				// ----------------------------------------------------
				if ( o->sensedData[idx].sensedDistance < maxDistanceInsertion )
				{
					theR += resolution;

					P1.x = px + cos(A-dA_2) * theR;
					P1.y = py + sin(A-dA_2) * theR;

					P2.x = px + cos(A+dA_2) * theR;
					P2.y = py + sin(A+dA_2) * theR;

					P1.cx = x2idx( P1.x );	P1.cy = y2idx( P1.y );
					P2.cx = x2idx( P2.x );	P2.cy = y2idx( P2.y );

	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
					// The x> comparison implicitly holds if x<0
					ASSERT_( static_cast<unsigned int>(P1.cx)<size_x && static_cast<unsigned int>(P1.cy)<size_y );
					ASSERT_( static_cast<unsigned int>(P2.cx)<size_x && static_cast<unsigned int>(P2.cy)<size_y );
	#endif

					// Special case: Only one cell:
					if (P2.cx==P1.cx && P2.cy==P1.cy)
					{
						updateCell_fast_occupied(P1.cx,P1.cy, logodd_observation_occupied, logodd_thres_occupied, theMapArray, theMapSize_x );
					}
					else
					{
						// Use "fractional integers" to approximate float operations during the ray tracing:
						// Integers store "float values * 128"
						const int AcxE  = P2.cx - P1.cx;
						const int AcyE  = P2.cy - P1.cy;

						// Increments at each raytracing step:
						const int nSteps = ( max(abs(AcxE),abs(AcyE)) + 1 );
						const float inv_N_12 = 1.0f / nSteps;	// Number of steps ^ -1
						const int  frAcxE = round( (AcxE<<FRBITS) * inv_N_12 );  //  Acx*128 / N
						const int  frAcyE = round( (AcyE<<FRBITS) * inv_N_12 );  //  Acy*128 / N

						R1.cx  = P1.cx;
						R1.cy  = P1.cy;
						R1.frX = R1.cx <<FRBITS;
						R1.frY = R1.cy <<FRBITS;

						for (int nStep=0;nStep<=nSteps;nStep++)
						{
							updateCell_fast_occupied(R1.cx,R1.cy, logodd_observation_occupied, logodd_thres_occupied, theMapArray, theMapSize_x );

							R1.frX += frAcxE;
							R1.frY += frAcyE;
							R1.cx = R1.frX >> FRBITS;
							R1.cy = R1.frY >> FRBITS;
						}

					} // end do a line

				} // end if we must set occupied cells

			}  // End of each range

			return true;
		} // end reallyInsert
		else
		    return false;
	}
	else
	{
		/********************************************************************
				OBSERVATION TYPE: Unknown
		********************************************************************/
		return false;
	}

//	MRPT_END
}

/*---------------------------------------------------------------
 Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
	takenFrom The robot's pose the observation is supposed to be taken from.
	obs The observation.
 This method returns a likelihood in the range [0,1].
 ---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood(
			const CObservation		*obs,
			const CPose3D			&takenFrom3D )
{
	// Ignore laser scans if they are not planar or they are not
	//  at the altitude of this grid map:
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	{
		const CObservation2DRangeScan	*scan = static_cast<const CObservation2DRangeScan*>(obs);
		if (!scan->isPlanarScan(insertionOptions.horizontalTolerance))
			return -10;
		if (insertionOptions.useMapAltitude &&
			fabs(insertionOptions.mapAltitude - scan->sensorPose.z() ) > 0.01 )
			return -10;

		// OK, go on...
	}

	// Execute according to the selected method:
	// --------------------------------------------
	CPose2D   takenFrom = CPose2D(takenFrom3D);  // 3D -> 2D, we are in a gridmap...

	switch (likelihoodOptions.likelihoodMethod)
	{
	default:
	case lmRayTracing:
		return computeObservationLikelihood_rayTracing(obs,takenFrom);

	case lmMeanInformation:
		return computeObservationLikelihood_MI(obs,takenFrom);

	case lmConsensus:
		return computeObservationLikelihood_Consensus(obs,takenFrom);

	case lmCellsDifference:
		return computeObservationLikelihood_CellsDifference(obs,takenFrom);

	case lmLikelihoodField_Thrun:
		return computeObservationLikelihood_likelihoodField_Thrun(obs,takenFrom);

	case lmLikelihoodField_II:
		return computeObservationLikelihood_likelihoodField_II(obs,takenFrom);

	case lmConsensusOWA:
		return computeObservationLikelihood_ConsensusOWA(obs,takenFrom);
	};

}

/*---------------------------------------------------------------
			computeObservationLikelihood_Consensus
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_Consensus(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	double		likResult = 0;

	// This function depends on the observation type:
	// -----------------------------------------------------
	if ( obs->GetRuntimeClass() != CLASS_ID(CObservation2DRangeScan) )
	{
		//THROW_EXCEPTION("This method is defined for 'CObservation2DRangeScan' classes only.");
		return 1e-3;
	}
	// Observation is a laser range scan:
	// -------------------------------------------
	const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );

	// Insert only HORIZONTAL scans, since the grid is supposed to
	//  be a horizontal representation of space.
	if ( ! o->isPlanarScan(insertionOptions.horizontalTolerance) ) return 0.5f;		// NO WAY TO ESTIMATE NON HORIZONTAL SCANS!!

	// Assure we have a 2D points-map representation of the points from the scan:
	const CPointsMap *compareMap = o->buildAuxPointsMap<mrpt::slam::CPointsMap>();

	// Observation is a points map:
	// -------------------------------------------
	size_t			Denom=0;
//	int			Acells = 1;
	CPoint2D	pointGlobal,pointLocal;


	// Get the points buffers:

	//	compareMap.getPointsBuffer( n, xs, ys, zs );
	const size_t n = compareMap->size();

	for (size_t i=0;i<n;i+=likelihoodOptions.consensus_takeEachRange)
	{
		// Get the point and pass it to global coordinates:
		compareMap->getPoint(i,pointLocal);
		pointGlobal = takenFrom + pointLocal;

		int		cx0 = x2idx( pointGlobal.x() );
		int		cy0 = y2idx( pointGlobal.y() );

/**/
		likResult += 1-getCell_nocheck(cx0,cy0);
		Denom++;
	}
	if (Denom)	likResult/=Denom;
	likResult = pow(likResult, static_cast<double>( likelihoodOptions.consensus_pow ) );
/** /
		int		cxMin = max(0,cx0 - Acells);
		int		cxMax = min(size_x-1,cx0 + Acells);
		int		cyMin = max(0,cy0 - Acells);
		int		cyMax = min(size_y-1,cy0 + Acells);

		for (int cx=cxMin;cx<=cxMax;cx++)
		{
			for (int cy=cyMin;cy<=cyMax;cy++)
			{
				Denom++;
				likResult += 1-getCell_nocheck(cx,cy);
			} // cy
		} // cx
	} // for each range point
	if (Denom)	likResult/=Denom;
	likResult = pow(likResult, (double) likelihoodOptions.consensus_pow);
/ **/

	return log(likResult);
}

/*---------------------------------------------------------------
			computeObservationLikelihood_ConsensusOWA
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_ConsensusOWA(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	double		likResult = 0;

	// This function depends on the observation type:
	// -----------------------------------------------------
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	{
		//THROW_EXCEPTION("This method is defined for 'CObservation2DRangeScan' classes only.");
		return 1e-3;
	}
	// Observation is a laser range scan:
	// -------------------------------------------
	const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );

	// Insert only HORIZONTAL scans, since the grid is supposed to
	//  be a horizontal representation of space.
	if ( ! o->isPlanarScan(insertionOptions.horizontalTolerance) ) return 0.5;		// NO WAY TO ESTIMATE NON HORIZONTAL SCANS!!

	// Assure we have a 2D points-map representation of the points from the scan:
	CPointsMap::TInsertionOptions	insOpt;
	insOpt.minDistBetweenLaserPoints	= -1;		// ALL the laser points

	const CPointsMap *compareMap = o->buildAuxPointsMap<mrpt::slam::CPointsMap>( &insOpt );

	// Observation is a points map:
	// -------------------------------------------
	int				Acells = 1;
	CPoint2D		pointGlobal,pointLocal;

	// Get the points buffers:
	const size_t n = compareMap->size();

	// Store the likelihood values in this vector:
	likelihoodOutputs.OWA_pairList.clear();
	for (size_t i=0;i<n;i++)
	{
		// Get the point and pass it to global coordinates:
		compareMap->getPoint(i,pointLocal);
		pointGlobal = takenFrom + pointLocal;

		int		cx0 = x2idx( pointGlobal.x() );
		int		cy0 = y2idx( pointGlobal.y() );

		int		cxMin = max(0,cx0 - Acells);
		int		cxMax = min(static_cast<int>(size_x)-1,cx0 + Acells);
		int		cyMin = max(0,cy0 - Acells);
		int		cyMax = min(static_cast<int>(size_y)-1,cy0 + Acells);

		double	lik = 0;

		for (int cx=cxMin;cx<=cxMax;cx++)
			for (int cy=cyMin;cy<=cyMax;cy++)
				lik += 1-getCell_nocheck(cx,cy);

		int		nCells = (cxMax-cxMin+1)*(cyMax-cyMin+1);
		ASSERT_(nCells>0);
		lik/=nCells;

		TPairLikelihoodIndex	element;
		element.first = lik;
		element.second = pointGlobal;
		likelihoodOutputs.OWA_pairList.push_back( element );
	} // for each range point

	// Sort the list of likelihood values, in descending order:
	// ------------------------------------------------------------
	std::sort(likelihoodOutputs.OWA_pairList.begin(),likelihoodOutputs.OWA_pairList.end());

	// Cut the vector to the highest "likelihoodOutputs.OWA_length" elements:
	size_t	M = likelihoodOptions.OWA_weights.size();
	ASSERT_( likelihoodOutputs.OWA_pairList.size()>=M );

	likelihoodOutputs.OWA_pairList.resize(M);
	likelihoodOutputs.OWA_individualLikValues.resize( M );
	likResult = 0;
	for (size_t k=0;k<M;k++)
	{
		likelihoodOutputs.OWA_individualLikValues[k] = likelihoodOutputs.OWA_pairList[k].first;
		likResult+= likelihoodOptions.OWA_weights[k] * likelihoodOutputs.OWA_individualLikValues[k];
	}

	return log(likResult);
}

/*---------------------------------------------------------------
			computeObservationLikelihood_CellsDifference
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_CellsDifference(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	 double		ret = 0.5;

	 // This function depends on the observation type:
	 // -----------------------------------------------------
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	 {
		 // Observation is a laser range scan:
		 // -------------------------------------------
		const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		if (!o->isPlanarScan(insertionOptions.horizontalTolerance)) return 0.5;	// NO WAY TO ESTIMATE NON HORIZONTAL SCANS!!

	 // Build a copy of this occupancy grid:
		COccupancyGridMap2D		compareGrid(takenFrom.x()-10,takenFrom.x()+10,takenFrom.y()-10,takenFrom.y()+10,resolution);
		CPose3D					robotPose(takenFrom);
		int						Ax, Ay;

		// Insert in this temporary grid:
		compareGrid.insertionOptions.maxDistanceInsertion			= insertionOptions.maxDistanceInsertion;
		compareGrid.insertionOptions.maxOccupancyUpdateCertainty	= 0.95f;
		o->insertObservationInto( &compareGrid, &robotPose );

		// Save Cells offset between the two grids:
		Ax = round((x_min - compareGrid.x_min) / resolution);
		Ay = round((y_min - compareGrid.y_min) / resolution);

		int			nCellsCompared = 0;
		float		cellsDifference = 0;
		int			x0 = max(0,Ax);
		int			y0 = max(0,Ay);
		int			x1 = min(compareGrid.size_x, size_x+Ax);
		int			y1 = min(compareGrid.size_y, size_y+Ay);

		for (int x=x0;x<x1;x+=1)
		{
			for (int y=y0;y<y1;y+=1)
			{
				float	xx = compareGrid.idx2x(x);
				float	yy = compareGrid.idx2y(y);

				float	c1 = getPos(xx,yy);
				float	c2 = compareGrid.getCell(x,y);
				if ( c2<0.45f || c2>0.55f )
				{
					nCellsCompared++;
					if ((c1>0.5 && c2<0.5) || (c1<0.5 && c2>0.5))
						cellsDifference++;
				}
			}
		}
		ret = 1 - cellsDifference / (nCellsCompared);
	 }
	 return log(ret);
}

/*---------------------------------------------------------------
			computeObservationLikelihood_MI
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_MI(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	MRPT_START;

 	CPose3D			poseRobot(takenFrom);
	double			res;

	// Dont modify the grid, only count the changes in Information
	updateInfoChangeOnly.enabled = true;
	insertionOptions.maxDistanceInsertion*= likelihoodOptions.MI_ratio_max_distance;

	// Reset the new information counters:
	updateInfoChangeOnly.cellsUpdated = 0;
	updateInfoChangeOnly.I_change = 0;
	updateInfoChangeOnly.laserRaysSkip = likelihoodOptions.MI_skip_rays;

	// Insert the observation (It will not be really inserted, only the information counted)
	insertObservation(obs,&poseRobot);

	// Compute the change in I aported by the observation:
	double	newObservation_mean_I;
	if (updateInfoChangeOnly.cellsUpdated)
			newObservation_mean_I = updateInfoChangeOnly.I_change / updateInfoChangeOnly.cellsUpdated;
	else	newObservation_mean_I = 0;

	// Let the normal mode enabled, i.e. the grid can be updated
	updateInfoChangeOnly.enabled = false;
	insertionOptions.maxDistanceInsertion/=likelihoodOptions.MI_ratio_max_distance;


	res = pow(newObservation_mean_I, static_cast<double>(likelihoodOptions.MI_exponent) );

	return log(res);

	MRPT_END;
 }

double	 COccupancyGridMap2D::computeObservationLikelihood_rayTracing(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	 double		ret=0;

	 // This function depends on the observation type:
	 // -----------------------------------------------------
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	 {
		 // Observation is a laser range scan:
		 // -------------------------------------------
		const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );
		 CObservation2DRangeScan		simulatedObs;

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		if (!o->isPlanarScan(insertionOptions.horizontalTolerance)) return 0.5;	// NO WAY TO ESTIMATE NON HORIZONTAL SCANS!!

		 // The number of simulated rays will be original range scan rays / DOWNRATIO
		 int		decimation = likelihoodOptions.rayTracing_decimation;
		 int		nRays     = o->scan.size();

		 // Perform simulation using same parameters than real observation:
		 simulatedObs.aperture = o->aperture;
		 simulatedObs.maxRange = o->maxRange;
		 simulatedObs.rightToLeft = o->rightToLeft;
		 simulatedObs.sensorPose = o->sensorPose;

		 // Performs the scan simulation:
		 laserScanSimulator(
			simulatedObs,		// The in/out observation
			takenFrom,			// robot pose
			0.45f,				// Cells threshold
			nRays,				// Scan length
			0,
			decimation	);

		 /** /
		 {
			FILE	*f;

			f=os::fopen("scan_sim.txt","wt");
			for (int i=0;i<nRays;i++) os::fprintf(f,"%f ",simulatedObs.validRange[i] ? simulatedObs.scan[i]:0);
			os::fclose(f);
			f=os::fopen("scan_meas.txt","wt");
			for (i=0;i<nRays;i++) os::fprintf(f,"%f ",o->validRange[i] ? o->scan[i]:0);
			os::fclose(f);
		 }
		 / **/

		 double		stdLaser   = likelihoodOptions.rayTracing_stdHit;
		 double		stdSqrt2 = sqrt(2.0f) * stdLaser;

		 // Compute likelihoods:
		 ret = 1;
		 //bool		useDF = likelihoodOptions.rayTracing_useDistanceFilter;
		 float		r_sim,r_obs;
		 double		likelihood;

		 for (int j=0;j<nRays;j+=decimation)
		 {
			// Simulated and measured ranges:
			r_sim = simulatedObs.scan[j];
			r_obs = o->scan[ j ];

			// Is a valid range?
			if ( o->validRange[j] )
			{
				likelihood = 0.1/o->maxRange + 0.9*exp( -square( min((float)fabs(r_sim-r_obs),2.0f)/stdSqrt2) );
				ret += log(likelihood);
				//printf("Sim=%f\tReal=%f\tlik=%f\n",r_sim,r_obs,likelihood);
			}

			//if ( r_sim>0 &&
			//	simulatedObs.validRange[j] &&
			//	o->validRange[j] )
			//{
			//	// Consider this measurement?
			//	if (!useDF || r_obs>=r_sim)
			//	{
			//		// UPDATE ----------------------
			//		likelihood = 0.1 + 0.9*exp( -square((r_sim-r_obs)/stdSqrt2) );
			//		ret *= likelihood;
			//		//printf("Sim=%f\tReal=%f\tlik=%f\n",r_sim,r_obs,likelihood);
			//		// -----------------------------
			//	}
			//}
			//else
			//{
			//	// Likelihood of unknown range:
			//	ret *= 1/o->maxRange;
			//}
		 }
	 }

	//printf("\t\t\t\tLIKELIHOOD=%e\n",ret);

	 return ret;
}
/**/

/*---------------------------------------------------------------
			computeObservationLikelihood_likelihoodField_Thrun
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_likelihoodField_Thrun(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	MRPT_START;

	double		ret=0;

	// This function depends on the observation type:
	// -----------------------------------------------------
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	{
		// Observation is a laser range scan:
		// -------------------------------------------
		const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		if (!o->isPlanarScan(insertionOptions.horizontalTolerance)) return -10;

		// Assure we have a 2D points-map representation of the points from the scan:
		CPointsMap::TInsertionOptions		opts;
		opts.minDistBetweenLaserPoints	= resolution*0.5f;
		opts.isPlanarMap				= true; // Already filtered above!
		opts.horizontalTolerance		= insertionOptions.horizontalTolerance;

		// Compute the likelihood of the points in this grid map:
		ret = computeLikelihoodField_Thrun( o->buildAuxPointsMap<mrpt::slam::CPointsMap>(&opts), &takenFrom );

	} // end of observation is a scan range 2D

	return ret;

	MRPT_END;

}

/*---------------------------------------------------------------
		computeObservationLikelihood_likelihoodField_II
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_likelihoodField_II(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	MRPT_START;

	double		ret=0;

	// This function depends on the observation type:
	// -----------------------------------------------------
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	{
		// Observation is a laser range scan:
		// -------------------------------------------
		const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		if (!o->isPlanarScan(insertionOptions.horizontalTolerance)) return 0.5f;	// NO WAY TO ESTIMATE NON HORIZONTAL SCANS!!

		// Assure we have a 2D points-map representation of the points from the scan:

		// Compute the likelihood of the points in this grid map:
		ret = computeLikelihoodField_II( o->buildAuxPointsMap<mrpt::slam::CPointsMap>(), &takenFrom );

	} // end of observation is a scan range 2D

	return ret;

	MRPT_END;

}


/*---------------------------------------------------------------
					computeLikelihoodField_Thrun
 ---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeLikelihoodField_Thrun( const CPointsMap	*pm, const CPose2D *relativePose )
{
	MRPT_START;

	double		ret;
	size_t		N = pm->getPointsCount();
	int		K = (int)ceil(likelihoodOptions.LF_maxCorrsDistance/*m*/ / resolution);	// The size of the checking area for matchings:

	bool		Product_T_OrSum_F = !likelihoodOptions.LF_alternateAverageMethod;

	if (!N)
	{
		//printf("[COccupancyGridMap2D::computeLikelihoodField_Thrun] There are no points in the map!!!");
		//mrpt::system::pause();
		return -100; // No way to estimate this likelihood!!
	}

	// Compute the likelihoods for each point:
	ret = 0; //Product_T_OrSum_F ? 1e300:0;

	float		stdHit	= likelihoodOptions.LF_stdHit;
	float		zHit	= likelihoodOptions.LF_zHit;
	float		zRandom	= likelihoodOptions.LF_zRandom;
	float		zRandomMaxRange	= likelihoodOptions.LF_maxRange;
	float		zRandomTerm = zRandom / zRandomMaxRange;
	float		Q = -0.5f / square(stdHit);
	int			M = 0;

	unsigned int	size_x_1 = size_x-1;
	unsigned int	size_y_1 = size_y-1;

	// Aux. variables for the "for j" loop:
	double		thisLik;
	double		maxCorrDist_sq = square(likelihoodOptions.LF_maxCorrsDistance);
	double		minimumLik = zRandomTerm  + zHit * exp( Q * maxCorrDist_sq );
	double		ccos,ssin;
	float		occupiedMinDist;

#define LIK_LF_CACHE_INVALID    (66)

    if (likelihoodOptions.enableLikelihoodCache)
    {
        // Reset the precomputed likelihood values map
        if (precomputedLikelihoodToBeRecomputed)
        {
			if (map.size())
					precomputedLikelihood.assign( map.size(),LIK_LF_CACHE_INVALID);
			else	precomputedLikelihood.clear();

			precomputedLikelihoodToBeRecomputed = false;
        }
    }

	cellType	thresholdCellValue = p2l(0.5f);
	int			decimation = likelihoodOptions.LF_decimation;

	const double _resolution = this->resolution;
	const double constDist2DiscrUnits = 100 / (_resolution * _resolution);
	const double constDist2DiscrUnits_INV = 1.0 / constDist2DiscrUnits;


	if (N<10) decimation = 1;

	TPoint2D	pointLocal;
	TPoint2D	pointGlobal;

	for (size_t j=0;j<N;j+= decimation)
	{

		occupiedMinDist = maxCorrDist_sq; // The max.distance

		// Get the point and pass it to global coordinates:
		if (relativePose)
		{
			pm->getPoint(j,pointLocal);
			//pointGlobal = *relativePose + pointLocal;
#ifdef HAVE_SINCOS
			::sincos(relativePose->phi(), &ssin,&ccos);
#else
			ccos = cos(relativePose->phi());
			ssin = sin(relativePose->phi());
#endif
			pointGlobal.x = relativePose->x() + pointLocal.x * ccos - pointLocal.y * ssin;
			pointGlobal.y = relativePose->y() + pointLocal.x * ssin + pointLocal.y * ccos;
		}
		else
		{
			pm->getPoint(j,pointGlobal);
		}

		// Point to cell indixes
		int cx = x2idx( pointGlobal.x );
		int cy = y2idx( pointGlobal.y );

		// Precomputed table:
		// Tip: Comparison cx<0 is implicit in (unsigned)(x)>size...
		if ( static_cast<unsigned>(cx)>=size_x_1 || static_cast<unsigned>(cy)>=size_y_1 )
		{
			// We are outside of the map: Assign the likelihood for the max. correspondence distance:
			thisLik = minimumLik;
		}
		else
		{
			// We are into the map limits:
            if (likelihoodOptions.enableLikelihoodCache)
            {
                thisLik = precomputedLikelihood[ cx+cy*size_x ];
            }

			if (!likelihoodOptions.enableLikelihoodCache || thisLik==LIK_LF_CACHE_INVALID )
			{
				// Compute now:
				// -------------
				// Find the closest occupied cell in a certain range, given by K:
				int xx1 = max(0,cx-K);
				int xx2 = min(size_x_1,(unsigned)(cx+K));
				int yy1 = max(0,cy-K);
				int yy2 = min(size_y_1,(unsigned)(cy+K));

				/** /
				for (yy=yy1;yy<=yy2;yy++)
					for (xx=xx1;xx<=xx2;xx++)
						if ( map[xx+yy*size_x] < thresholdCellValue )
							occupiedMinDist = min( occupiedMinDist, square(idx2x(xx)-pointGlobal_x)+square(idx2y(yy)-pointGlobal_y) );
				*/

				// Optimized code: this part will be invoked a *lot* of times:
				{
					cellType  *mapPtr  = &map[xx1+yy1*size_x]; // Initial pointer position
					unsigned   incrAfterRow = size_x - ((xx2-xx1)+1);

					signed int Ax0 = 10*(xx1-cx);
					signed int Ay  = 10*(yy1-cy);

					unsigned int occupiedMinDistInt = mrpt::utils::round( maxCorrDist_sq * constDist2DiscrUnits );

					for (int yy=yy1;yy<=yy2;yy++)
					{
						unsigned int Ay2 = square((unsigned int)(Ay)); // Square is faster with unsigned.
						signed short Ax=Ax0;
						cellType  cell;

						for (int xx=xx1;xx<=xx2;xx++)
						{
							if ( (cell =*mapPtr++) < thresholdCellValue )
							{
								unsigned int d = square((unsigned int)(Ax)) + Ay2;
								keep_min(occupiedMinDistInt, d);
							}
							Ax += 10;
						}
						// Go to (xx1,yy++)
						mapPtr += incrAfterRow;
						Ay += 10;
					}

					occupiedMinDist = occupiedMinDistInt * constDist2DiscrUnits_INV ;
				}

				thisLik = zRandomTerm  + zHit * exp( Q * occupiedMinDist );

                if (likelihoodOptions.enableLikelihoodCache)
                    // And save it into the table and into "thisLik":
                    precomputedLikelihood[ cx+cy*size_x ] = thisLik;
			}
		}

		// Update the likelihood:
		if (Product_T_OrSum_F)
		{
			ret += log(thisLik);
		}
		else
		{
			ret += thisLik;
			M++;
		}
	} // end of for each point in the scan

	if (!Product_T_OrSum_F)
		ret = log( ret / M );

	return ret;

	MRPT_END;
}

/*---------------------------------------------------------------
					computeLikelihoodField_II
 ---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeLikelihoodField_II( const CPointsMap	*pm, const CPose2D *relativePose )
{
	MRPT_START;

	double		ret;
	size_t		N = pm->getPointsCount();

	if (!N) return 1e-100; // No way to estimate this likelihood!!

	// Compute the likelihoods for each point:
	ret = 0;
//	if (likelihoodOptions.LF_alternateAverageMethod)
//			ret = 0;
//	else	ret = 1;

	TPoint2D	pointLocal,pointGlobal;

	float		zRandomTerm = 1.0f / likelihoodOptions.LF_maxRange;
	float		Q = -0.5f / square( likelihoodOptions.LF_stdHit );

	// Aux. cell indixes variables:
	int			cx,cy;
	size_t		j;
	int			cx0,cy0;
	int			cx_min, cx_max;
	int			cy_min, cy_max;
	int			maxRangeInCells = (int)ceil(likelihoodOptions.LF_maxCorrsDistance / resolution);
	int			nCells = 0;

	// -----------------------------------------------------
	// Compute around a window of neigbors around each point
	// -----------------------------------------------------
	for (j=0;j<N;j+= likelihoodOptions.LF_decimation)
	{
		// Get the point and pass it to global coordinates:
		// ---------------------------------------------
		if (relativePose)
		{
			pm->getPoint(j,pointLocal);
			pointGlobal = *relativePose + pointLocal;
		}
		else
		{
			pm->getPoint(j,pointGlobal);
		}

		// Point to cell indixes:
		// ---------------------------------------------
		cx0 = x2idx( pointGlobal.x );
		cy0 = y2idx( pointGlobal.y );

		// Compute the range of cells to compute:
		// ---------------------------------------------
		cx_min = max( cx0-maxRangeInCells,0);
		cx_max = min( cx0+maxRangeInCells,static_cast<int>(size_x));
		cy_min = max( cy0-maxRangeInCells,0);
		cy_max = min( cy0+maxRangeInCells,static_cast<int>(size_y));

//		debugImg.rectangle(cx_min,cy_min,cx_max,cy_max,0xFF0000 );

		// Compute over the window of cells:
		// ---------------------------------------------
		double  lik = 0;
		for (cx=cx_min;cx<=cx_max;cx++)
		{
			for (cy=cy_min;cy<=cy_max;cy++)
			{
				float	P_free = getCell(cx,cy);
				float	termDist = exp(Q*(square(idx2x(cx)-pointGlobal.x)+square(idx2y(cy)-pointGlobal.y) ));

				lik += P_free	  * zRandomTerm +
					   (1-P_free) * termDist;
			} // end for cy
		} // end for cx

		// Update the likelihood:
		if (likelihoodOptions.LF_alternateAverageMethod)
				ret += lik;
		else	ret += log(lik/((cy_max-cy_min+1)*(cx_max-cx_min+1)));
		nCells++;

	} // end of for each point in the scan

	if (likelihoodOptions.LF_alternateAverageMethod && nCells>0)
			ret = log(ret/nCells);
	else	ret = ret/nCells;

	/** /
	char	str[100];
	os::sprintf(str,100,"LIK=%e",ret);
	win.setWindowTitle( str );
	debugImg.setOriginTopLeft(false);
	win.showImage(debugImg);
	win.setPos(500,0);
	win.waitForKey();
	/ **/

	return ret;

	MRPT_END;
}

/*---------------------------------------------------------------
							clear
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::internal_clear()
{
	setSize( -10,10,-10,10,getResolution());
	//resetFeaturesCache();
	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;
}

/*---------------------------------------------------------------
							fill
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::fill(float default_value)
{
	cellType		defValue = p2l( default_value );
	for (std::vector<cellType>::iterator	it=map.begin();it<map.end();++it)
		*it = defValue;
	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;
	//resetFeaturesCache();
}

/*---------------------------------------------------------------
					updateCell
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::updateCell(int x,int y, float v)
{
	// Tip: if x<0, (unsigned)(x) will also be >>> size_x ;-)
	if (static_cast<unsigned int>(x)>=size_x || static_cast<unsigned int>(y)>=size_y)
		return;

	// Get the current contents of the cell:
	cellType	&theCell = map[x+y*size_x];

	// Compute the new Bayesian-fused value of the cell:
	if ( updateInfoChangeOnly.enabled )
	{
		float	old	= l2p(theCell);
		float		new_v	= 1 / ( 1 + (1-v)*(1-old)/(old*v) );
		updateInfoChangeOnly.cellsUpdated++;
		updateInfoChangeOnly.I_change+= 1-(H(new_v)+H(1-new_v))/MAX_H;
	}
	else
	{
		cellType obs = p2l(v);  // The observation: will be >0 for free, <0 for occupied.
		if (obs>0)
		{
			if ( theCell>(OCCGRID_CELLTYPE_MAX-obs) )
					theCell = OCCGRID_CELLTYPE_MAX; // Saturate
			else	theCell += obs;
		}
		else
		{
			if ( theCell<(OCCGRID_CELLTYPE_MIN-obs) )
					theCell = OCCGRID_CELLTYPE_MIN; // Saturate
			else	theCell += obs;
		}
	}
}


/*---------------------------------------------------------------
							subSample
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::subSample( int downRatio )
{
	std::vector<cellType>		newMap;

	ASSERT_(downRatio>0);

	resolution*=downRatio;

	int		newSizeX = round((x_max-x_min)/resolution);
	int		newSizeY = round((y_max-y_min)/resolution);

	newMap.resize(newSizeX*newSizeY);

	for (int x=0;x<newSizeX;x++)
	{
		for (int y=0;y<newSizeY;y++)
		{
			float	newCell = 0;

			for (int xx=0;xx<downRatio;xx++)
				for (int yy=0;yy<downRatio;yy++)
					newCell+= getCell(x*downRatio+xx, y*downRatio+yy);

			newCell /= (downRatio*downRatio);

			newMap[ x + y*newSizeX ] = p2l(newCell);
		}
	}


	setSize(x_min,x_max,y_min,y_max,resolution);
	map = newMap;


}

/*---------------------------------------------------------------
							computeMatchingWith
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::computeMatchingWith2D(
    const CMetricMap								*otherMap2,
    const CPose2D									&otherMapPose_,
    float									maxDistForCorrespondence,
    float									maxAngularDistForCorrespondence,
    const CPose2D									&angularDistPivotPoint,
    TMatchingPairList						&correspondences,
    float									&correspondencesRatio,
    float									*sumSqrDist,
    bool									onlyKeepTheClosest,
	bool									onlyUniqueRobust) const
{
	MRPT_START;

	ASSERT_(otherMap2->GetRuntimeClass()->derivedFrom( CLASS_ID(CPointsMap) ));
	const CPointsMap			*otherMap = static_cast<const CPointsMap*>(otherMap2);

	const TPose2D  otherMapPose = TPose2D(otherMapPose_);

	size_t							nLocalPoints = otherMap->getPointsCount();
	float							_sumSqrDist=0;
	float							meanSquareError, meanSquareErrorMax;
	std::vector<float>				x_locals(nLocalPoints), y_locals(nLocalPoints),z_locals(nLocalPoints);
	std::vector<float>::const_iterator	otherMap_x_it,otherMap_y_it,otherMap_z_it;
	std::vector<float>::iterator	x_locals_it,y_locals_it,z_locals_it;
	float							sin_phi = sin(otherMapPose.phi);
	float							cos_phi = cos(otherMapPose.phi);
	size_t							nOtherMapPointsWithCorrespondence = 0;	// Number of points with one corrs. at least
	size_t							nTotalCorrespondences = 0;				// Total number of corrs
	float							maxDistForCorrespondenceSquared;
	float							min_dist,this_dist;
	bool							thisLocalHasCorr;
	register float					x_local, y_local,z_local;
	int								cx,cy,cx_min,cx_max,cy_min,cy_max;	// For the cells search
	unsigned int					localIdx;
	float							residual_x,residual_y;

	// The number of cells to look around each point:
	int								cellsSearchRange = round( maxDistForCorrespondence / resolution );


	// Initially there are no correspondences:
	correspondences.clear();
	correspondencesRatio = 0;
	meanSquareError = meanSquareErrorMax = 0;

	// Hay mapa local?
	if (!nLocalPoints)  return;  // No

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	// -----------------------------------------------------------
	float	local_x_min=0,local_x_max=0,local_y_min=0,local_y_max=0;

	// Translate all local map points:
	for (x_locals_it=x_locals.begin(),
			y_locals_it=y_locals.begin(),
			z_locals_it=z_locals.begin(),
			otherMap_x_it=otherMap->x.begin(),
			otherMap_y_it=otherMap->y.begin(),
			otherMap_z_it=otherMap->z.begin();
			x_locals_it<x_locals.end();
			++x_locals_it,++y_locals_it,++z_locals_it,++otherMap_x_it,++otherMap_y_it)
	{
		// Girar y desplazar cada uno de los puntos del local map:
		*x_locals_it = otherMapPose.x + cos_phi* (*otherMap_x_it) - sin_phi*(*otherMap_y_it);
		*y_locals_it = otherMapPose.y + sin_phi* (*otherMap_x_it) + cos_phi*(*otherMap_y_it);
		*z_locals_it = /* otherMapPose.z +*/ (*otherMap_z_it);

		// mantener el max/min de los puntos:
		local_x_min = min(local_x_min,*x_locals_it);
		local_x_max = max(local_x_max,*x_locals_it);
		local_y_min = min(local_y_min,*y_locals_it);
		local_y_max = max(local_y_max,*y_locals_it);
	}

	// If the local map is entirely out of the grid,
	//   do not even try to match them!!
	if (local_x_min> x_max ||
		local_x_max< x_min ||
		local_y_min> y_max ||
		local_y_max< y_min) return;		// Matching is NULL!


	cellType	thresholdCellValue = p2l(0.5f);

	// For each point in the other map:
	for (localIdx=0,
			x_locals_it=x_locals.begin(),
			y_locals_it=y_locals.begin(),
			z_locals_it=z_locals.begin(),
			otherMap_x_it=otherMap->x.begin(),
			otherMap_y_it=otherMap->y.begin(),
			otherMap_z_it=otherMap->z.begin();
			x_locals_it<x_locals.end();
			++x_locals_it,++y_locals_it,++z_locals_it,++otherMap_x_it,++otherMap_y_it,++otherMap_z_it,++localIdx)
	{
		// Starting value:
		maxDistForCorrespondenceSquared = square( maxDistForCorrespondence );

		// For speed-up:
		x_local = *x_locals_it;
		y_local = *y_locals_it;
		z_local = *z_locals_it;

		// Look for the occupied cell closest from the map point:
		min_dist = 1e6;
		TMatchingPair		closestCorr;

		// Get the indexes of cell where the point falls:
		cx=x2idx(x_local);
		cy=y2idx(y_local);

		// Get the rectangle to look for into:
		cx_min = max(0, cx - cellsSearchRange );
		cx_max = min(static_cast<int>(size_x)-1, cx + cellsSearchRange );
		cy_min = max(0, cy - cellsSearchRange );
		cy_max = min(static_cast<int>(size_y)-1, cy + cellsSearchRange );

		// Will be set to true if a corrs. is found:
		thisLocalHasCorr = false;


		// Look in nearby cells:
		for (cx=cx_min;cx<=cx_max;cx++)
		{
			for (cy=cy_min;cy<=cy_max;cy++)
			{
				// Is an occupied cell?
				if ( map[cx+cy*size_x] < thresholdCellValue )//  getCell(cx,cy)<0.49)
				{
					residual_x = idx2x(cx)- x_local;
					residual_y = idx2y(cy)- y_local;

					// Compute max. allowed distance:
					maxDistForCorrespondenceSquared = square(
								maxAngularDistForCorrespondence * angularDistPivotPoint.distance2DTo(x_local,y_local) +
								maxDistForCorrespondence );

					// Square distance to the point:
					this_dist = square( residual_x ) + square( residual_y );

					if (this_dist<maxDistForCorrespondenceSquared)
					{
						if (!onlyKeepTheClosest)
						{
							// save the correspondence:
							nTotalCorrespondences++;
							TMatchingPair   mp;
							mp.this_idx = cx+cy*size_x;
							mp.this_x = idx2x(cx);
							mp.this_y = idx2y(cy);
							mp.this_z = z_local;
							mp.other_idx = localIdx;
							mp.other_x = *otherMap_x_it;
							mp.other_y = *otherMap_y_it;
							mp.other_z = *otherMap_z_it;
							correspondences.push_back( mp );
						}
						else
						{
							// save the closest only:
							if (this_dist<min_dist)
							{
								min_dist = this_dist;

								closestCorr.this_idx = cx+cy*size_x;
								closestCorr.this_x = idx2x(cx);
								closestCorr.this_y = idx2y(cy);
								closestCorr.this_z = z_local;
								closestCorr.other_idx = localIdx;
								closestCorr.other_x = *otherMap_x_it;
								closestCorr.other_y = *otherMap_y_it;
								closestCorr.other_z = *otherMap_z_it;
							}
						}

						// At least one:
						thisLocalHasCorr = true;
					}
				}
			}
		} // End of find closest nearby cell

		// save the closest correspondence:
		if (onlyKeepTheClosest && (min_dist<maxDistForCorrespondenceSquared))
		{
			nTotalCorrespondences++;
			correspondences.push_back( closestCorr );
		}

		// At least one corr:
		if (thisLocalHasCorr)
		{
			nOtherMapPointsWithCorrespondence++;

			// Accumulate the MSE:
			_sumSqrDist+= min_dist;
		}

	}	// End "for each local point"...

	correspondencesRatio = nOtherMapPointsWithCorrespondence / static_cast<float>(nLocalPoints);

	// If requested, copy sum of squared distances to output pointer:
	// -------------------------------------------------------------------
	if (sumSqrDist) *sumSqrDist = _sumSqrDist;


//		os::fclose(fDebug);

	MRPT_END;

}

/*---------------------------------------------------------------
	Initilization of values, don't needed to be called directly.
  ---------------------------------------------------------------*/
COccupancyGridMap2D::TInsertionOptions::TInsertionOptions() :
	mapAltitude							( 0 ),
	useMapAltitude						( false ),
	maxDistanceInsertion				(  15.0f ),
	maxOccupancyUpdateCertainty			(  0.65f ),
	considerInvalidRangesAsFreeSpace	(  true ),
	decimation							( 1 ),
	horizontalTolerance					( DEG2RAD(0.05) ),

	CFD_features_gaussian_size			( 1 ),
	CFD_features_median_size			( 3 ),

	wideningBeamsWithDistance			( false )
{
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR(mapAltitude,float,  					iniFile, section );
	MRPT_LOAD_CONFIG_VAR(maxDistanceInsertion,float,  			iniFile, section );
	MRPT_LOAD_CONFIG_VAR(maxOccupancyUpdateCertainty,float,  	iniFile, section );
	MRPT_LOAD_CONFIG_VAR(useMapAltitude,bool,  					iniFile, section );
	MRPT_LOAD_CONFIG_VAR(considerInvalidRangesAsFreeSpace,bool,	iniFile, section );
	MRPT_LOAD_CONFIG_VAR(decimation,int,  						iniFile, section );
	MRPT_LOAD_CONFIG_VAR_DEGREES(horizontalTolerance, 		 	iniFile, section );

	MRPT_LOAD_CONFIG_VAR(CFD_features_gaussian_size,float,  	iniFile, section );
	MRPT_LOAD_CONFIG_VAR(CFD_features_median_size,float,  	iniFile, section );
	MRPT_LOAD_CONFIG_VAR(wideningBeamsWithDistance,bool,  	iniFile, section );
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::TInsertionOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [COccupancyGridMap2D::TInsertionOptions] ------------ \n\n");

	LOADABLEOPTS_DUMP_VAR(mapAltitude, float)
	LOADABLEOPTS_DUMP_VAR(maxDistanceInsertion, float)
	LOADABLEOPTS_DUMP_VAR(maxOccupancyUpdateCertainty, float)
	LOADABLEOPTS_DUMP_VAR(useMapAltitude, bool)
	LOADABLEOPTS_DUMP_VAR(considerInvalidRangesAsFreeSpace, bool)
	LOADABLEOPTS_DUMP_VAR(decimation, int)
	LOADABLEOPTS_DUMP_VAR(horizontalTolerance, float)
	LOADABLEOPTS_DUMP_VAR(CFD_features_gaussian_size, float)
	LOADABLEOPTS_DUMP_VAR(CFD_features_median_size, float)
	LOADABLEOPTS_DUMP_VAR(wideningBeamsWithDistance, bool)

	out.printf("\n");
}

/*---------------------------------------------------------------
	Initilization of values, don't needed to be called directly.
  ---------------------------------------------------------------*/
COccupancyGridMap2D::TLikelihoodOptions::TLikelihoodOptions() :
	likelihoodMethod				( lmLikelihoodField_Thrun),

	LF_stdHit						( 0.35f ),
	LF_zHit							( 0.95f ),
	LF_zRandom						( 0.05f ),
	LF_maxRange						( 81.0f ),
	LF_decimation					( 5 ),
	LF_maxCorrsDistance				( 0.3f ),
	LF_alternateAverageMethod		( false ),

	MI_exponent						( 2.5f ),
	MI_skip_rays					( 10 ),
	MI_ratio_max_distance			( 1.5f ),

	rayTracing_useDistanceFilter	( true ),
	rayTracing_decimation			( 10 ),
	rayTracing_stdHit				( 1.0f ),

	consensus_takeEachRange			( 1 ),
	consensus_pow					( 5 ),
	OWA_weights						(100,1/100.0f),

	enableLikelihoodCache           ( true )
{
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::TLikelihoodOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR_CAST(likelihoodMethod, int, TLikelihoodMethod, iniFile, section);

    enableLikelihoodCache               = iniFile.read_bool(section,"enableLikelihoodCache",enableLikelihoodCache);

	LF_stdHit							= iniFile.read_float(section,"LF_stdHit",LF_stdHit);
	LF_zHit								= iniFile.read_float(section,"LF_zHit",LF_zHit);
	LF_zRandom							= iniFile.read_float(section,"LF_zRandom",LF_zRandom);
	LF_maxRange							= iniFile.read_float(section,"LF_maxRange",LF_maxRange);
	LF_decimation						= iniFile.read_int(section,"LF_decimation",LF_decimation);
	LF_maxCorrsDistance					= iniFile.read_float(section,"LF_maxCorrsDistance",LF_maxCorrsDistance);
	LF_alternateAverageMethod			= iniFile.read_bool(section,"LF_alternateAverageMethod",LF_alternateAverageMethod);

	MI_exponent							= iniFile.read_float(section,"MI_exponent",MI_exponent);
	MI_skip_rays						= iniFile.read_int(section,"MI_skip_rays",MI_skip_rays);
	MI_ratio_max_distance				= iniFile.read_float(section,"MI_ratio_max_distance",MI_ratio_max_distance);

	rayTracing_useDistanceFilter		= iniFile.read_bool(section,"rayTracing_useDistanceFilter",rayTracing_useDistanceFilter);
	rayTracing_stdHit					= iniFile.read_float(section,"rayTracing_stdHit",rayTracing_stdHit);

	consensus_takeEachRange				= iniFile.read_int(section,"consensus_takeEachRange",consensus_takeEachRange);
	consensus_pow						= iniFile.read_float(section,"consensus_pow",consensus_pow);

	iniFile.read_vector(section,"OWA_weights",OWA_weights,OWA_weights);
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::TLikelihoodOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [COccupancyGridMap2D::TLikelihoodOptions] ------------ \n\n");

	out.printf("likelihoodMethod                        = ");
	switch (likelihoodMethod)
	{
	case lmMeanInformation: out.printf("lmMeanInformation"); break;
	case lmRayTracing: out.printf("lmRayTracing"); break;
	case lmConsensus: out.printf("lmConsensus"); break;
	case lmCellsDifference: out.printf("lmCellsDifference"); break;
	case lmLikelihoodField_Thrun: out.printf("lmLikelihoodField_Thrun"); break;
	case lmLikelihoodField_II: out.printf("lmLikelihoodField_II"); break;
	case lmConsensusOWA: out.printf("lmConsensusOWA"); break;
	default:
		out.printf("UNKNOWN!!!"); break;
	}
	out.printf("\n");

	out.printf("enableLikelihoodCache                   = %c\n",	enableLikelihoodCache ? 'Y':'N');

	out.printf("LF_stdHit                               = %f\n",	LF_stdHit );
	out.printf("LF_zHit                                 = %f\n",	LF_zHit );
	out.printf("LF_zRandom                              = %f\n",	LF_zRandom );
	out.printf("LF_maxRange                             = %f\n",	LF_maxRange );
	out.printf("LF_decimation                           = %u\n",	LF_decimation );
	out.printf("LF_maxCorrsDistance                     = %f\n",	LF_maxCorrsDistance );
	out.printf("LF_alternateAverageMethod               = %c\n",	LF_alternateAverageMethod ? 'Y':'N');
	out.printf("MI_exponent                             = %f\n",	MI_exponent );
	out.printf("MI_skip_rays                            = %u\n",	MI_skip_rays );
	out.printf("MI_ratio_max_distance                   = %f\n",	MI_ratio_max_distance );
	out.printf("rayTracing_useDistanceFilter            = %c\n",	rayTracing_useDistanceFilter ? 'Y':'N');
	out.printf("rayTracing_decimation                   = %u\n",	rayTracing_decimation );
	out.printf("rayTracing_stdHit                       = %f\n",	rayTracing_stdHit );
	out.printf("consensus_takeEachRange                 = %u\n",	consensus_takeEachRange );
	out.printf("consensus_pow                           = %.02f\n", consensus_pow);
	out.printf("OWA_weights   = [");
	for (size_t i=0;i<OWA_weights.size();i++)
	{
		if (i<3 || i>(OWA_weights.size()-3))
			out.printf("%.03f ",OWA_weights[i]);
		else if (i==3 && OWA_weights.size()>6)
			out.printf(" ... ");
	}
	out.printf("] (size=%u)\n",(unsigned)OWA_weights.size());
	out.printf("\n");
}

/*---------------------------------------------------------------
					isEmpty
  ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::isEmpty() const
{
	return m_is_empty;
}


/*---------------------------------------------------------------
				saveAsBitmapTwoMapsWithCorrespondences
  ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences(
	const std::string						&fileName,
	const COccupancyGridMap2D				*m1,
	const COccupancyGridMap2D				*m2,
	const TMatchingPairList		&corrs)
{
	MRPT_START;

	CImage			img1,img2;
	CImage			img(10,10,3,true);
	TColor 			lineColor;
	unsigned int	i,n , lx1, ly1, lx2, ly2, Ay1, Ay2;
	unsigned int	px, py;

	lineColor = TColor::red;

	// The individual maps:
	// ---------------------------------------------
	m1->getAsImage( img1, false );
	m2->getAsImage( img2, false );
	lx1 = img1.getWidth();	ly1 = img1.getHeight();
	lx2 = img2.getWidth();	ly2 = img2.getHeight();

	// The map with the lowest height has to be vertically aligned:
	if (ly1>ly2)
	{
		Ay1 = 0;
		Ay2 = (ly1-ly2)/2;
	}
	else
	{
		Ay2 = 0;
		Ay1 = (ly2-ly1)/2;
	}


	// Compute the size of the composite image:
	// ---------------------------------------------
	img.resize(lx1 + lx2 + 1, max(ly1,ly2), 3, true );
	img.filledRectangle(0,0,img.getWidth()-1,img.getHeight()-1, TColor::black );	// background: black
	img.drawImage(0,Ay1,img1);
	img.drawImage(lx1+1,Ay2,img2);

	// Draw the features:
	// ---------------------------------------------
	n = corrs.size();
	lineColor = TColor::black;
	for (i=0;i<n;i++)
	{
		// In M1:
		px = m1->x2idx( corrs[i].this_x );
		py = Ay1+ly1-1- m1->y2idx( corrs[i].this_y );
		img.rectangle(px-10,py-10,px+10,py+10,lineColor);
		img.rectangle(px-11,py-11,px+11,py+11,lineColor);

		// In M2:
		px = lx1+1 + m2->x2idx( corrs[i].other_x );
		py = Ay2+ly2-1- m2->y2idx( corrs[i].other_y );
		img.rectangle(px-10,py-10,px+10,py+10,lineColor);
		img.rectangle(px-11,py-11,px+11,py+11,lineColor);
	}

	// Draw the correspondences as lines:
	// ---------------------------------------------
	for (i=0;i<n;i++)
	{
		lineColor = TColor(
			static_cast<long>(randomGenerator.drawUniform(0,255.0f)),
			static_cast<long>(randomGenerator.drawUniform(0,255.0f)),
			static_cast<long>(randomGenerator.drawUniform(0,255.0f)) );

		img.line(
			m1->x2idx( corrs[i].this_x ),
//				lx1+1+ m1->x2idx( corrs[i].this_x ),
			Ay1+ly1-1- m1->y2idx( corrs[i].this_y ),
			lx1+1+ m2->x2idx( corrs[i].other_x ),
			Ay2+ly2-1-m2->y2idx( corrs[i].other_y ),
            lineColor);
	} // i

	return img.saveToFile(fileName.c_str() );

	MRPT_END;
}

/*---------------------------------------------------------------
				saveAsEMFTwoMapsWithCorrespondences
  ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::saveAsEMFTwoMapsWithCorrespondences(
	const std::string						&fileName,
	const COccupancyGridMap2D				*m1,
	const COccupancyGridMap2D				*m2,
	const TMatchingPairList		&corrs)
{
	MRPT_START;

	CEnhancedMetaFile				emf(fileName,1);
	CImage			img1,img2;
	TColor			lineColor;
	unsigned int	i,n , lx1, ly1, lx2, ly2, Ay1, Ay2;
	unsigned int	px, py;


	lineColor = TColor::red;

	// The individual maps:
	// ---------------------------------------------
#ifdef MRPT_OS_WINDOWS
	m1->getAsImage( img1, true );
	m2->getAsImage( img2, true );
#else
	m1->getAsImage( img1, false );		// Linux: emulated EMF is different.
	m2->getAsImage( img2, false );
#endif
	lx1 = img1.getWidth();	ly1 = img1.getHeight();
	lx2 = img2.getWidth();	ly2 = img2.getHeight();

	// The map with the lowest height has to be vertically aligned:
	if (ly1>ly2)
	{
		Ay1 = 0;
		Ay2 = (ly1-ly2)/2;
	}
	else
	{
		Ay2 = 0;
		Ay1 = (ly2-ly1)/2;
	}


	// Draw the pair of maps:
	// ---------------------------------------------
	emf.drawImage(0,Ay1,img1);
	emf.drawImage(lx1+1,Ay2,img2);

	// Draw the features:
	// ---------------------------------------------
	n = corrs.size();
	lineColor = TColor::black;
	for (i=0;i<n;i++)
	{
		// In M1:
		px = m1->x2idx( corrs[i].this_x );
		py = Ay1+ly1-1- m1->y2idx( corrs[i].this_y );
		emf.rectangle(px-10,py-10,px+10,py+10,lineColor);
		emf.rectangle(px-11,py-11,px+11,py+11,lineColor);

		// In M2:
		px = lx1+1 + m2->x2idx( corrs[i].other_x );
		py = Ay2+ly2-1- m2->y2idx( corrs[i].other_y );
		emf.rectangle(px-10,py-10,px+10,py+10,lineColor);
		emf.rectangle(px-11,py-11,px+11,py+11,lineColor);
	}

/** /
	// Draw the correspondences as lines:
	// ---------------------------------------------
	for (i=0;i<n;i++)
	{
		lineColor =
			((unsigned long)RandomUni(0,255.0f)) +
			(((unsigned long)RandomUni(0,255.0f)) << 8 ) +
			(((unsigned long)RandomUni(0,255.0f)) << 16 );

		emf.line(
			m1->x2idx( corrs[i].this_x ),
			Ay1+ly1-1- m1->y2idx( corrs[i].this_y ),
			lx1+1+ m2->x2idx( corrs[i].other_x ),
			Ay2+ly2-1-m2->y2idx( corrs[i].other_y ),
            lineColor);
	} // i
/ **/

	// Draw the correspondences as text labels:
	// ---------------------------------------------
	char	str[100];
	for (i=0;i<n;i++)
	{
		os::sprintf(str,100,"%i",i);

		emf.textOut(
			m1->x2idx( corrs[i].this_x ) - 10 ,
			Ay1+ly1-1- m1->y2idx( corrs[i].this_y ) - 25,
			str, TColor::black );

		emf.textOut(
			lx1+1+ m2->x2idx( corrs[i].other_x ) - 10,
			Ay2+ly2-1-m2->y2idx( corrs[i].other_y ) - 25,
			str,TColor::black );
	} // i

	return true;

	MRPT_END;
}

/*---------------------------------------------------------------
				getAs3DObject
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::getAs3DObject(mrpt::opengl::CSetOfObjectsPtr	&outSetOfObj ) const
{
	if (m_disableSaveAs3DObject)
		return;

	MRPT_START;

	opengl::CTexturedPlanePtr	outObj = opengl::CTexturedPlane::Create();

	outObj->setPlaneCorners(x_min,x_max,y_min,y_max);

	outObj->setLocation(0,0, insertionOptions.mapAltitude );

	// Create the color & transparecy (alpha) images:
	CImage			imgColor(size_x,size_y,1);
	CImage			imgTrans(size_x,size_y,1);


	unsigned int x,y;

	const cellType		*srcPtr = &map[0];
	unsigned char		*destPtr_color;
	unsigned char		*destPtr_trans;

	for (y=0;y<size_y;y++)
	{
		destPtr_color = imgColor(0,y);
		destPtr_trans = imgTrans(0,y);
		for (x=0;x<size_x;x++)
		{
			uint8_t  cell255 = l2p_255(*srcPtr++);
			*destPtr_color++ = cell255;

			int8_t   auxC = (int8_t)((signed short)cell255)-128;
			*destPtr_trans++ = auxC>0 ? (auxC << 1) : ((-auxC) << 1);
		}
	}

	outObj->assignImage_fast( imgColor,imgTrans );
	outSetOfObj->insert( outObj );

	MRPT_END;
}


/*---------------------------------------------------------------
				operator <
  ---------------------------------------------------------------*/
bool mrpt::slam::operator < (const COccupancyGridMap2D::TPairLikelihoodIndex &e1, const COccupancyGridMap2D::TPairLikelihoodIndex &e2)
{
	return e1.first > e2.first;
}


/*---------------------------------------------------------------
				computeClearance
  ---------------------------------------------------------------*/
float  COccupancyGridMap2D::computeClearance( float x, float y, float maxSearchDistance ) const
{
	int			xx1 = max(0,x2idx(x-maxSearchDistance));
	int			xx2 = min(static_cast<unsigned>( size_x-1),static_cast<unsigned>( x2idx(x+maxSearchDistance)) );
	int			yy1 = max(0,y2idx(y-maxSearchDistance));
	int			yy2 = min(static_cast<unsigned>( size_y-1),static_cast<unsigned>( y2idx(y+maxSearchDistance)));

	int			cx = x2idx(x);
	int			cy = y2idx(y);

	int			xx,yy;
	float		clearance_sq = square(maxSearchDistance);
	cellType	thresholdCellValue = p2l(0.5f);

	// At least 1 free cell nearby!
	bool		atLeastOneFree = false;
	for (xx=cx-1;!atLeastOneFree && xx<=cx+1;xx++)
		for (yy=cy-1;!atLeastOneFree && yy<=cy+1;yy++)
			if (getCell(xx,yy)>0.505f)
				atLeastOneFree = true;


	if (!atLeastOneFree)
		return 0;

	for (xx=xx1;xx<=xx2;xx++)
		for (yy=yy1;yy<=yy2;yy++)
			if (map[xx+yy*size_x]<thresholdCellValue)
				clearance_sq = min( clearance_sq, square(resolution)*(square(xx-cx)+square(yy-cy)) );

	return sqrt(clearance_sq);
}

/*---------------------------------------------------------------
				computePathCost
  ---------------------------------------------------------------*/
float  COccupancyGridMap2D::computePathCost( float x1, float y1, float x2, float y2 ) const
{
	float	sumCost = 0;

	float	dist = sqrt( square(x1-x2)+square(y1-y2) );
	int	nSteps = round(1.5f * dist / resolution);

	for (int i=0;i<nSteps;i++)
	{
		float	x = x1 + (x2-x1)*i/static_cast<float>(nSteps);
		float	y = y1 + (y2-y1)*i/static_cast<float>(nSteps);
		sumCost += getPos( x,y );
	}

	if (nSteps)
			return sumCost/static_cast<float>(nSteps);
	else	return 0;
}

/*---------------------------------------------------------------
  Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
 * \param  otherMap					  [IN] The other map to compute the matching with.
 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
 * \param  minDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
 * \param  minMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
 *
 * \return The matching ratio [0,1]
 * \sa computeMatchingWith2D
  --------------------------------------------------------------- */
float  COccupancyGridMap2D::compute3DMatchingRatio(
		const CMetricMap						*otherMap,
		const CPose3D							&otherMapPose,
		float									minDistForCorr,
		float									minMahaDistForCorr
		) const
{
	MRPT_UNUSED_PARAM(otherMap);
	MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(minDistForCorr);
	MRPT_UNUSED_PARAM(minMahaDistForCorr);

	return 0;
}

/*---------------------------------------------------------------
					auxParticleFilterCleanUp
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::auxParticleFilterCleanUp()
{

}

/*---------------------------------------------------------------
					auxParticleFilterCleanUp
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::saveMetricMapRepresentationToFile(
	const std::string	&filNamePrefix
	) const
{
	std::string		fil( filNamePrefix + std::string(".png") );
	saveAsBitmapFile( fil  );

	fil = filNamePrefix + std::string("_limits.txt");
	CMatrix LIMITS(1,4);
	LIMITS(0,0) = x_min;
	LIMITS(0,1) = x_max;
	LIMITS(0,2) = y_min;
	LIMITS(0,3) = y_max;
	LIMITS.saveToTextFile( fil, MATRIX_FORMAT_FIXED );
}



/** Returns true if this map is able to compute a sensible likelihood function for this observation (i.e. an occupancy grid map cannot with an image).
 * \param obs The observation.
 * \sa computeObservationLikelihood
 */
bool COccupancyGridMap2D::canComputeObservationLikelihood( const CObservation *obs )
{
	// Ignore laser scans if they are not planar or they are not
	//  at the altitude of this grid map:
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	{
		const CObservation2DRangeScan		*scan = static_cast<const CObservation2DRangeScan*>( obs );

		if (!scan->isPlanarScan(insertionOptions.horizontalTolerance))
			return false;
		if (insertionOptions.useMapAltitude &&
			fabs(insertionOptions.mapAltitude - scan->sensorPose.z() ) > 0.01 )
			return false;

		// OK, go on...
		return true;
	}
	else // Is not a laser scanner...
	{
		return false;
	}
}


void COccupancyGridMap2D::OnPostSuccesfulInsertObs(const CObservation *)
{
	m_is_empty = false;
}

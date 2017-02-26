/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/utils/round.h>  // round()

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

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
	if (!x1 && !x2 && !y1 && !y2) {
		x1=y1=0;
		x2=size_x-1;
		y2=size_y-1;
	}
	else {
		x1=max(0,x1);
		y1=max(0,y1);
		x2=min(x2,static_cast<int>(size_x)-1);
		y2=min(y2,static_cast<int>(size_y)-1);
	}

	int robot_size_units= round(100*robot_size / resolution);

	/* We store 0 in cells NOT belonging to Voronoi, or the closest distance
 * to obstacle otherwise, the "clearance" in "int" distance units.
 */
	m_voronoi_diagram.setSize(x_min,x_max, y_min,y_max, resolution);  // assign(size_x*size_y,0);
	ASSERT_EQUAL_(m_voronoi_diagram.getSizeX(), size_x);
	ASSERT_EQUAL_(m_voronoi_diagram.getSizeY(), size_y);
	m_voronoi_diagram.fill(0);

	// freeness threshold
	voroni_free_threshold= 1.0f - threshold;

	int     basis_x[2],basis_y[2];
	int     nBasis;

	// Build Voronoi:
	for (int x=x1;x<=x2;x++) {
		for (int y=y1;y<=y2;y++)
		{
			const int Clearance = computeClearance(x,y,basis_x,basis_y,&nBasis);

			if (Clearance > robot_size_units )
				setVoroniClearance(x,y,Clearance );
		}
	}

	// Limpiar: Hacer que los trazos sean de grosor 1:
	//  Si un punto del diagrama esta rodeada de mas de 2
	//   puntos tb del diagrama, eliminarlo:
	int    nDiag;
	for (int x=x1;x<=x2;x++) {
		for (int y=y1;y<=y2;y++) {
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

	// Temp list of candidate
	std::vector<int>	temp_x,temp_y, temp_clear, temp_borrar;

	// Scan for critical points
	// ---------------------------------------------
	for (int x=1;x<(static_cast<int>(size_x)-1);x++) {
		for (int y=1;y<(static_cast<int>(size_y)-1);y++) {
			if ( 0!=(clear_xy=getVoroniClearance(x,y)) )
			{
				// Is this a critical point?
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

				// At least 2 more neighbors
				if (nVecinosVoroni>=3 && min_clear_near==clear_xy && max_clear_near!=clear_xy )
				{
					// Add to temp list:
					temp_x.push_back( x );
					temp_y.push_back( y );
					temp_clear.push_back( clear_xy );
					temp_borrar.push_back( 0 );
				}

			}
		}
	}

	// Filter: find "basis points". If two coincide, leave the one with the shortest clearance.
	std::vector<int> basis1_x,basis1_y, basis2_x,basis2_y;
	for (unsigned i=0;i<temp_x.size();i++)
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
	for (unsigned i=0;i<(((temp_x.size()))-1);i++) {
		if (!temp_borrar[i]) {
			for (unsigned int j=i+1;j<temp_x.size();j++) {
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
			}
		}
	}

	// Copy to permanent list:
	// ----------------------------------------------------------
	CriticalPointsList.clearance.clear();
	CriticalPointsList.x.clear();
	CriticalPointsList.y.clear();
	CriticalPointsList.x_basis1.clear();
	CriticalPointsList.y_basis1.clear();
	CriticalPointsList.x_basis2.clear();
	CriticalPointsList.y_basis2.clear();

	for (unsigned i=0;i<temp_x.size();i++)
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
					int             ult_x=0,x,ult_y=0,y;
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
					int xx = cx+circs_x[idx];
					int yy = cy+circs_y[idx];

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


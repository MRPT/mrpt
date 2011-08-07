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

#include <mrpt/math.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::reactivenav;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CLogFileRecord_ND, CHolonomicLogFileRecord,mrpt::reactivenav )



/**  Initialize the parameters of the navigator, from some
*    configuration file, or default values if filename is set to NULL.
*/
CHolonomicND::CHolonomicND(const mrpt::utils::CConfigFileBase *INI_FILE )
{
	last_selected_sector = -1;
	if (INI_FILE!=NULL)
        initialize( *INI_FILE );
}

/*---------------------------------------------------------------
						initialize
  ---------------------------------------------------------------*/
void  CHolonomicND::initialize( const mrpt::utils::CConfigFileBase &INI_FILE )
{
    MRPT_START

	const std::string section("ND_CONFIG");

	// Default values:
	WIDE_GAP_SIZE_PERCENT               = 0.50f;
	MAX_SECTOR_DIST_FOR_D2_PERCENT      = 0.25f;
	RISK_EVALUATION_SECTORS_PERCENT     = 0.10f;
	RISK_EVALUATION_DISTANCE            = 0.4f;
	TOO_CLOSE_OBSTACLE                  = 0.15f;
	TARGET_SLOW_APPROACHING_DISTANCE    = 0.20f;

    // Load from config text:
	MRPT_LOAD_CONFIG_VAR(WIDE_GAP_SIZE_PERCENT,double,  INI_FILE,section );
	MRPT_LOAD_CONFIG_VAR(MAX_SECTOR_DIST_FOR_D2_PERCENT,double,  INI_FILE,section );
	MRPT_LOAD_CONFIG_VAR(RISK_EVALUATION_SECTORS_PERCENT,double,  INI_FILE,section );
	MRPT_LOAD_CONFIG_VAR(RISK_EVALUATION_DISTANCE,double,  INI_FILE,section );
	MRPT_LOAD_CONFIG_VAR(TOO_CLOSE_OBSTACLE,double,  INI_FILE,section );
	MRPT_LOAD_CONFIG_VAR(TARGET_SLOW_APPROACHING_DISTANCE,double,  INI_FILE,section );

    INI_FILE.read_vector(section,"factorWeights", vector_double(0), factorWeights, true );
    ASSERT_(factorWeights.size()==4);

    MRPT_END
}


/*---------------------------------------------------------------
						Navigate
  ---------------------------------------------------------------*/
void  CHolonomicND::navigate(
				poses::CPoint2D	&target,
				vector_double	&obstacles,
				double			maxRobotSpeed,
				double			&desiredDirection,
				double			&desiredSpeed,
				CHolonomicLogFileRecordPtr &logRecord)
{
	TGapArray			gaps;
	TSituations			situation;
	int					selectedSector;
	double				riskEvaluation;
	CLogFileRecord_NDPtr log;
	double				evaluation;

	// Create a log record for returning data.
	if (!logRecord.present())
	{
		log = CLogFileRecord_ND::Create();
		logRecord = log;
	}


	// Search gaps:
    gaps.clear();
	gapsEstimator(	obstacles,
					target,
					gaps );


	// Select best gap:
	searchBestGap(	obstacles,
					1.0f,
					gaps,
					target,
					selectedSector,
					evaluation,
					situation,
					riskEvaluation,
					log);

	if (situation == SITUATION_NO_WAY_FOUND)
	{
		// No way found!
		desiredDirection = 0;
		desiredSpeed = 0;
	}
	else
	{
		// A valid movement:
		desiredDirection = (double)(M_PI*(-1 + 2*(0.5f+selectedSector)/((double)obstacles.size())));

		// Speed control: Reduction factors
		// ---------------------------------------------
		double		targetNearnessFactor = max(0.20, min(1.0, 1.0-exp(-(target.norm()+0.01)/TARGET_SLOW_APPROACHING_DISTANCE)));
		//printf(" TARGET NEARNESS = %f\n",targetNearnessFactor);
		double		riskFactor = min(1.0, riskEvaluation / RISK_EVALUATION_DISTANCE );

		desiredSpeed = maxRobotSpeed * min(riskFactor,targetNearnessFactor);
	}

	last_selected_sector = selectedSector;

	// LOG --------------------------
	if (log)
	{
		// gaps:
		if (situation != SITUATION_TARGET_DIRECTLY )
		{
			int	i,n = gaps.size();
			log->gaps_ini.resize(n);
			log->gaps_end.resize(n);
			for (i=0;i<n;i++)
			{
				log->gaps_ini[i]  = gaps[i].ini;
				log->gaps_end[i]  = gaps[i].end;
			}
		}
        // Selection:
        log->selectedSector = selectedSector;
        log->evaluation = evaluation;
        log->situation = situation;
        log->riskEvaluation = riskEvaluation;
	}
}


/*---------------------------------------------------------------
						Find gaps in the obtacles.
  ---------------------------------------------------------------*/
void  CHolonomicND::gapsEstimator(
		vector_double		&obstacles,
		poses::CPoint2D		&target,
		TGapArray			&gaps_out )
{
	unsigned int	i,n;
	int				nMaximos=0;
    double			MaximoAbsoluto = -100;
	double			MinimoAbsoluto = 100;
    vector_int		MaximoIdx;
    vector_double	MaximoValor;

    // Hacer una lista con los maximos de las distancias a obs:
    // ----------------------------------------------------------
	MaximoIdx.resize(obstacles.size());
	MaximoValor.resize(obstacles.size());
	n = obstacles.size();

    for (i=1;i<(n-1);i++)
    {
		// Actualizar max. y min. absolutos:
		MaximoAbsoluto= max( MaximoAbsoluto, obstacles[i] );
		MinimoAbsoluto= min( MinimoAbsoluto, obstacles[i] );

		// Buscar maximos locales:
		if ( ( obstacles[i] >= obstacles[i+1] &&
			  obstacles[i] > obstacles[i-1] ) ||
			  ( obstacles[i] > obstacles[i+1] &&
			  obstacles[i] >= obstacles[i-1] ) )
		{
				MaximoIdx[nMaximos] = i;
				MaximoValor[nMaximos++] = obstacles[i];
		}
    }

    //  Crear GAPS:
    // --------------------------------------------------------
	TGapArray    gaps_temp;
   	gaps_temp.reserve( 150 );

	for (double factorUmbral = 0.975f;factorUmbral>=0.04f;factorUmbral-=0.05f)
	{
            double   umbral = factorUmbral* MaximoAbsoluto + (1.0f-factorUmbral)*MinimoAbsoluto;
			bool	dentro = false;
			int		sec_ini=0, sec_end;
			double	maxDist=0;

			for (i=0;i<n;i++)
			{
				if ( !dentro && (!i || obstacles[i]>=umbral) )
				{
					sec_ini = i;
					maxDist = obstacles[i];
					dentro = true;
				}
				else if (dentro && (i==(n-1) || obstacles[i]<umbral ))
				{
					sec_end = i;
					dentro = false;

					if ( (sec_end-sec_ini) > 2 )
					{
						// Add new gap:
						TGap	newGap;
						newGap.ini				= sec_ini;
						newGap.end				= sec_end;
						newGap.entranceDistance = min( obstacles[sec_ini], obstacles[sec_end] );
						newGap.maxDistance		= maxDist;

						gaps_temp.push_back(newGap);
					}
				}

				if (dentro) maxDist = max( maxDist, obstacles[i] );
			}
	}

    // Proceso de eliminacion de huecos redundantes:
    // -------------------------------------------------------------
	std::vector<bool>	borrar_gap;
	borrar_gap.resize( gaps_temp.size() );
        for (i=0;i<gaps_temp.size();i++)
			borrar_gap[i] = false;


    // Eliminar huecos con muy poca profundidad si estan dentro de otros:
	double	maxProfundidad = 0;
	for (i=0;i<gaps_temp.size();i++)
    {
		double profundidad =
				gaps_temp[i].maxDistance -
				gaps_temp[i].entranceDistance;
		maxProfundidad = max(maxProfundidad, profundidad);
	}

	for (i=0;i<gaps_temp.size();i++)
    {
		double profundidad =
				gaps_temp[i].maxDistance -
				gaps_temp[i].entranceDistance;

		if ( profundidad< maxProfundidad / 10.0f )
				borrar_gap[i]=true;
    }


	// Si es muy estrecho, pero hay uno casi igual pero UN POCO mas grande,
	//  borrar el estrecho:
    for (i=0;i<gaps_temp.size();i++)
    {
        int     ini_i = gaps_temp[i].ini;
        int     fin_i = gaps_temp[i].end;
		int		ancho_i = fin_i - ini_i;

		if ( !borrar_gap[i] )
		{
			for (unsigned int j=0;j<gaps_temp.size() && !borrar_gap[i];j++)
			{
				if (i!=j)
				{
					int     ini_j = gaps_temp[j].ini;
					int     fin_j = gaps_temp[j].end;
					int		ancho_j = fin_j - ini_j;

					// j dentro de i y UN POCO mas grande nada mas:
					if (	!borrar_gap[j] &&
							ini_j>=ini_i &&
							fin_j<=fin_i &&
							ancho_i < (0.05f*n) &&
							ancho_j < (0.25f*n)
						)
						borrar_gap[i] = true;
				}
			}
		}
	}

	// Si dentro tiene mas de 1, borrarlo:
   for (i=0;i<gaps_temp.size();i++)
    {
        int     ini_i = gaps_temp[i].ini;
        int     fin_i = gaps_temp[i].end;
		int		nDentro = 0;

		if ( !borrar_gap[i] )
		{
			for (unsigned int j=0;j<gaps_temp.size();j++)
			{
				if (i!=j)
				{
					int     ini_j = gaps_temp[j].ini;
					int     fin_j = gaps_temp[j].end;

					// j dentro de i:
					if (    !borrar_gap[j] &&
							ini_j>=ini_i &&
							fin_j<=fin_i ) nDentro++;
				}
			}
			if (nDentro>1) borrar_gap[i] = true;
		}
	}


	// Uno dentro de otro y practicamente a la misma altura: Eliminarlo tambien:
   for (i=0;i<gaps_temp.size();i++)
    {
		if (!borrar_gap[i])
		{
            double	ent_i = gaps_temp[i].entranceDistance;
            int     ini_i = gaps_temp[i].ini;
            int     fin_i = gaps_temp[i].end;

            double MIN_GAPS_ENTR_DIST = (MaximoAbsoluto-MinimoAbsoluto)/10.0f;

            for (unsigned int j=0;j<gaps_temp.size() && !borrar_gap[i];j++)
				if (i!=j)
				{
                    double	ent_j = gaps_temp[j].entranceDistance;
                    int		ini_j = gaps_temp[j].ini;
                    int		fin_j = gaps_temp[j].end;

                    // j dentro de i y casi misma "altura":
                    if (    !borrar_gap[j] &&
							!borrar_gap[i] &&
                            ini_j>=ini_i &&
                            fin_j<=fin_i &&
                            fabs(ent_i-ent_j)< MIN_GAPS_ENTR_DIST )
                                    borrar_gap[i]=true;
				}
		}
    }

    // Copiar solo huecos no marcados para borrar:
    // ---------------------------------------------------
    gaps_out.clear();
	gaps_out.reserve(15);
    for (i=0;i<gaps_temp.size();i++)
            if ( !borrar_gap[i] )
			{
				// Calcular direccion representativa:
				calcRepresentativeSectorForGap( gaps_temp[i], target, obstacles);

				gaps_out.push_back( gaps_temp[i] );
			}

}

/*---------------------------------------------------------------
						Search the best gap.
  ---------------------------------------------------------------*/
void  CHolonomicND::searchBestGap(
			vector_double		&obstacles,
			double				maxObsRange,
			TGapArray			&in_gaps,
			poses::CPoint2D		&target,
			int					&out_selDirection,
			double				&out_selEvaluation,
			TSituations			&out_situation,
			double				&out_riskEvaluation,
			CLogFileRecord_NDPtr log)
{
	// Para evaluar el risk:
	unsigned int min_risk_eval_sector	= 0;
	unsigned int max_risk_eval_sector	= obstacles.size()-1;
	unsigned int TargetSector		= direction2sector(atan2(target.y(),target.x()),obstacles.size());
	const double TargetDist	= std::max(0.01,target.norm());

    // El "risk" se calcula al final para todos los casos.

    // D1 : Camino directo?
    // --------------------------------------------------------
    const int freeSectorsNearTarget = 10;  // 3
    bool theyAreFree = true, caseD1 = false;
    if (TargetSector>(unsigned int)freeSectorsNearTarget &&
		TargetSector<(unsigned int)(obstacles.size()-freeSectorsNearTarget) )
    {
        for (int j=-freeSectorsNearTarget;j<=freeSectorsNearTarget;j++)
                if (obstacles[ TargetSector + j ]<0.95*TargetDist)
                        theyAreFree = false;
        caseD1 = theyAreFree;
    }

    if (caseD1)
    {
        // S1: Camino libre hacia target:
        out_selDirection	= TargetSector;

		// Si hay mas de una, la que llegue antes
		out_selEvaluation   =	1.0 + std::max( 0.0, (maxObsRange - TargetDist) / maxObsRange );
        out_situation		=	SITUATION_TARGET_DIRECTLY;
    }
    else
    {
        // Evaluar los GAPs (Si no hay ninguno, nada, claro):
        vector_double	gaps_evaluation;
        int				selected_gap		=-1;
        double			selected_gap_eval	= -100;

        evaluateGaps(
                obstacles,
				maxObsRange,
                in_gaps,
                TargetSector,
                TargetDist,
                gaps_evaluation );

		if (log) log->gaps_eval = gaps_evaluation;

        // D2: Hay algun gap que pase por detras del target?
        //   ( y no este demasiado lejos):
        // -------------------------------------------------
        for ( unsigned int i=0;i<in_gaps.size();i++ )
			if ( in_gaps[i].maxDistance >= TargetDist &&
				 abs((int)(in_gaps[i].representative_sector-(int)TargetSector)) <= (int)floor(MAX_SECTOR_DIST_FOR_D2_PERCENT * obstacles.size()) )
					if ( gaps_evaluation[i]>selected_gap_eval )
					{
						selected_gap_eval = gaps_evaluation[i];
						selected_gap = i;
					}


        // Coger el mejor GAP:
        //  (Esto ya solo si no se ha cogido antes)
        if ( selected_gap==-1 )
			for ( unsigned int i=0;i<in_gaps.size();i++ )
				if ( gaps_evaluation[i]>selected_gap_eval )
				{
						selected_gap_eval = gaps_evaluation[i];
						selected_gap = i;
				}
        //  D3:  No es suficientemente bueno? ( o no habia ninguno?)
        // ------------------------------------------------------------
        if ( selected_gap_eval <= 0 )
        {
            // S2: No way found
            // ------------------------------------------------------
            out_selDirection	= 0;
            out_selEvaluation	= 0.0f; // La peor
            out_situation		= SITUATION_NO_WAY_FOUND;
        }
        else
        {
            // El seleccionado:
            TGap    gap = in_gaps[selected_gap];

            int     sectors_to_be_wide = round( WIDE_GAP_SIZE_PERCENT * obstacles.size() );

            out_selDirection	= in_gaps[selected_gap].representative_sector;
            out_selEvaluation	= selected_gap_eval;

            // D4: Es un gap ancho?
            // -----------------------------------------------------
            if ( (gap.end-gap.ini) < sectors_to_be_wide )
            {
                    // S3: Small gap
                    // -------------------------------------------
                    out_situation	= SITUATION_SMALL_GAP;
            }
            else
            {
                    // S4: Wide gap
                    // -------------------------------------------
                    out_situation	= SITUATION_WIDE_GAP;
            }

			// Que el risk no salga del gap:
			min_risk_eval_sector = gap.ini;
			max_risk_eval_sector = gap.end;
        }
    }

    // Calcular minima distancia a obstaculos a corto plazo, en
    //   un intervalo de sectores en torno al sector elegido:
    int     ancho_sectores = round( RISK_EVALUATION_SECTORS_PERCENT * obstacles.size() );
    int     sec_ini = max((int)min_risk_eval_sector, out_selDirection - ancho_sectores );
    int     sec_fin = min((int)max_risk_eval_sector, out_selDirection + ancho_sectores );

    out_riskEvaluation = 0.0;
    for (int i=sec_ini;i<=sec_fin;i++) out_riskEvaluation+= obstacles[ i ];
    out_riskEvaluation /= (sec_fin - sec_ini + 1 );

}


/*---------------------------------------------------------------
	Fills in the representative sector
		field in the gap structure:
  ---------------------------------------------------------------*/
void  CHolonomicND::calcRepresentativeSectorForGap(
	TGap					&gap,
	const poses::CPoint2D	&target,
	const vector_double		&obstacles)
{
    int     sector;
    int     sectors_to_be_wide = round( WIDE_GAP_SIZE_PERCENT * obstacles.size());
	int		TargetSector = direction2sector( atan2(target.y(),target.x()), obstacles.size() );

    if ( (gap.end-gap.ini) < sectors_to_be_wide )
    {
#if	1
		sector = round(0.5f*gap.ini+0.5f*gap.end);
#else
		double	min_dist_obs_near_ini=1, min_dist_obs_near_end=1;
		int		i;
		for ( i= gap.ini;i>=max(0,gap.ini-2);i--)
			min_dist_obs_near_ini = min(min_dist_obs_near_ini, obstacles[i]);
		for ( i= gap.end;i<=min((int)obstacles.size()-1,gap.end+2);i++)
			min_dist_obs_near_end = min(min_dist_obs_near_end, obstacles[i]);
		sector = round((min_dist_obs_near_ini*gap.ini+min_dist_obs_near_end*gap.end)/(min_dist_obs_near_ini+min_dist_obs_near_end));
#endif
    }
    else
    {
        // Para gaps anchos que NO contengan al target, cerca del borde
        //  mas cercano a este:
		//if ( TargetSector < gap.ini || TargetSector > gap.end )
  //      {
            int     dir;
            int     dist_ini = abs( TargetSector - gap.ini );
            int     dist_end = abs( TargetSector - gap.end );
            if (dist_ini<dist_end) {
                    sector = gap.ini;
                    dir = +1; }
            else {
                    sector = gap.end;
                    dir = -1; }
            sector = sector + dir * sectors_to_be_wide/2 ;
    //    }
    //    else
    //    {
    //        // Es un valle ancho con el Target dentro:
    //        // Buscar la maxima "distance" en un rango cerca del target:
    //        int     ini = max( gap.ini, TargetSector - sectors_to_be_wide / 2 );
    //        int     end = min( TargetSector + sectors_to_be_wide / 2, gap.end);

    //        sector = TargetSector;
    //        for (int i = ini;i<=end;i++)
				//if ( obstacles[i] > obstacles[sector] )
				//	sector = i;
    //    }
    }

	keep_max(sector, 0);
	keep_min(sector, (int)obstacles.size()-1);

    gap.representative_sector = sector;
}



/*---------------------------------------------------------------
						Evaluate each gap
  ---------------------------------------------------------------*/
void  CHolonomicND::evaluateGaps(
	const vector_double	&obstacles,
	const double		maxObsRange,
	const TGapArray		&gaps,
	const int			TargetSector,
	const double		TargetDist,
	vector_double		&out_gaps_evaluation )
{
	out_gaps_evaluation.resize( gaps.size());

	double	targetAng = M_PI*(-1 + 2*(0.5+TargetSector)/double(obstacles.size()));
	double	target_x =  TargetDist*cos(targetAng);
	double	target_y =  TargetDist*sin(targetAng);

    for (unsigned int i=0;i<gaps.size();i++)
    {
        // Para referenciarlo mas facilmente:
        const TGap	*gap = &gaps[i];

        double   d;
        d = min( obstacles[ gap->representative_sector ],
				min( maxObsRange,  0.95f*TargetDist) );

		// Las coordenadas (en el TP-Space) representativas del gap:
		double	phi = M_PI*(-1 + 2*(0.5+gap->representative_sector)/double(obstacles.size()));
		double	x =  d*cos(phi);
		double	y =  d*sin(phi);

        // Factor 1: Distancia hasta donde llego por esta GPT:
        // -----------------------------------------------------
		double factor1;
/*		if (gap->representative_sector == TargetSector )
				factor1 = min(TargetDist,obstacles[gap->representative_sector]) / TargetDist;
		else
		{
			if (TargetDist>1)
					factor1 = obstacles[gap->representative_sector] / TargetDist;
			else	factor1 = obstacles[gap->representative_sector];
		}
*/
		// Calcular la distancia media a donde llego por este gap:
		double	meanDist = 0;
		for (int j=gap->ini;j<=gap->end;j++)
			meanDist+= obstacles[j];
		meanDist/= ( gap->end - gap->ini + 1);

		if (abs(gap->representative_sector-TargetSector)<=1 && TargetDist<1)
				factor1 = min(TargetDist,meanDist) / TargetDist;
		else	factor1 = meanDist;

        // Factor 2: Distancia en sectores:
        // -------------------------------------------
        double   dif = fabs(((double)( TargetSector - gap->representative_sector )));
//		if (dif> (0.5f*obstacles.size()) ) dif = obstacles.size() - dif;
		// Solo si NO estan el target y el gap atravesando el alfa = "-pi" o "pi"
		if (dif> (0.5f*obstacles.size()) && (TargetSector-0.5f*obstacles.size())*(gap->representative_sector-0.5f*obstacles.size())<0 )
			dif = obstacles.size() - dif;

        double   factor2= exp(-square( dif / (obstacles.size()/4))) ;

        // Factor3: Para evitar cabeceos entre 2 o mas caminos que sean casi iguales:
        // -------------------------------------------
		double dist = (double)(abs(last_selected_sector - gap->representative_sector));
		//
		if (dist> (0.5f*obstacles.size()) ) dist = obstacles.size() - dist;

		double factor_AntiCab;
		if (last_selected_sector==-1)
				factor_AntiCab = 0;
		else	factor_AntiCab = (dist > 0.10f*obstacles.size()) ? 0.0f:1.0f;

        // Factor3: Minima distancia entre el segmento y el target:
		//  Se valora negativamente el alejarse del target
        // -----------------------------------------------------
		double	closestX,closestY;
        double dist_eucl = math::minimumDistanceFromPointToSegment(
					target_x,
					target_y,
					0,0,
					x,y,
					closestX,closestY);

        double factor3=  ( maxObsRange - min(maxObsRange ,dist_eucl) ) / maxObsRange;

		ASSERT_(factorWeights.size()==4);

		if ( obstacles[gap->representative_sector] < TOO_CLOSE_OBSTACLE ) // Too close to obstacles
				out_gaps_evaluation[i] = 0;
		else	out_gaps_evaluation[i] = (
				  factorWeights[0] * factor1 +
				  factorWeights[1] * factor2 +
				  factorWeights[2] * factor3 +
				  factorWeights[3] * factor_AntiCab ) / (math::sum(factorWeights)) ;

	} // for each gap

}

int  CHolonomicND::direction2sector(double a, int N)
{
 		if (a>M_PI)  a-=(double)M_2PI;
		if (a<-M_PI) a+=(double)M_2PI;

		return round(0.5f*(N*(1+a/M_PI) - 1));
}

/*---------------------------------------------------------------
					writeToStream
	Implements the writing to a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void  CLogFileRecord_ND::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		out << gaps_ini << gaps_end << gaps_eval;
		out << selectedSector << evaluation << riskEvaluation << (uint32_t) situation;
	}
}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  CLogFileRecord_ND::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			int32_t	n;

			in >> n;
			gaps_ini.resize(n);
			gaps_end.resize(n);
			in.ReadBuffer( &(*gaps_ini.begin()), sizeof(gaps_ini[0]) * n );
			in.ReadBuffer( &(*gaps_end.begin()), sizeof(gaps_end[0]) * n );

			in >> n;
			gaps_eval.resize(n);
			in.ReadBuffer( &(*gaps_eval.begin()), sizeof(gaps_eval[0]) * n );

			in >> selectedSector >> evaluation >> riskEvaluation >> n;

			situation = (CHolonomicND::TSituations) n;
		} break;
	case 1:
		{
		    uint32_t    n;
            in >> gaps_ini >> gaps_end >> gaps_eval;
            in >> selectedSector >> evaluation >> riskEvaluation >> n;
            situation  = (CHolonomicND::TSituations) n;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

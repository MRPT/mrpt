/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/slam/CRandomFieldGridMap2D.h>
#include <mrpt/system/os.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/color_maps.h>
#include <mrpt/utils/round.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfTriangles.h>

#include <numeric>

// =========== DEBUG MACROS =============
#define RANDOMFIELDGRIDMAP_VERBOSE		    0
#define RANDOMFIELDGRIDMAP_KF2_VERBOSE		0
// ======================================


using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CRandomFieldGridMap2D, CMetricMap,mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CRandomFieldGridMap2D::CRandomFieldGridMap2D(
	TMapRepresentation	mapType,
	float		x_min,
	float		x_max,
	float		y_min,
	float		y_max,
	float		resolution ) :
		CDynamicGrid<TRandomFieldCell>( x_min,x_max,y_min,y_max,resolution ),
		m_insertOptions_common( NULL ),
		m_mapType(mapType),
		m_cov(0,0),
		m_hasToRecoverMeanAndCov(true),
		m_DM_lastCutOff(0),
		m_average_normreadings_mean(0),
		m_average_normreadings_var(0),
		m_average_normreadings_count(0)

{
	// We can't set "m_insertOptions_common" here via "getCommonInsertOptions()" since
	//  it's a pure virtual method and we're at the constructor.
	// We need all derived classes to call ::clear() in their constructors so we reach internal_clear()
	//  and set there that variable...
}

CRandomFieldGridMap2D::~CRandomFieldGridMap2D()
{
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::internal_clear()
{
	// (Read the note in the constructor above)
	m_insertOptions_common = getCommonInsertOptions();  // Get the pointer from child class

	m_average_normreadings_mean  = 0;
	m_average_normreadings_var   = 0;
	m_average_normreadings_count = 0;

	switch (m_mapType)
	{
	case mrKernelDM:
	case mrKernelDMV:
		{
			// Set the grid to initial values:
			TRandomFieldCell	def(0,0);
			fill( def );
		}
		break;

	case mrKalmanFilter:
		{
			printf("[CRandomFieldGridMap2D::clear] Setting covariance matrix to %ux%u\n",(unsigned int)(m_size_y*m_size_x),(unsigned int)(m_size_y*m_size_x));

			TRandomFieldCell	def(
				m_insertOptions_common->KF_defaultCellMeanValue,		// mean
				m_insertOptions_common->KF_initialCellStd				// std
				);

			fill( def );

			// Reset the covariance matrix:
			m_cov.setSize( m_size_y*m_size_x, m_size_y*m_size_x );


			// And load its default values:
			const double KF_covSigma2 = square(m_insertOptions_common->KF_covSigma);
			const double res2 = square(m_resolution);
			const double std0sqr = square(  m_insertOptions_common->KF_initialCellStd );

			for (size_t i = 0;i<m_cov.getRowCount();i++)
			{
				int		cx1 = ( i % m_size_x );
				int		cy1 = ( i / m_size_x );

				for (size_t j = i;j<m_cov.getColCount();j++)
				{
					int		cx2 = ( j % m_size_x );
					int		cy2 = ( j / m_size_x );

					if (i==j)
					{
						m_cov(i,j) = std0sqr;
					}
					else
					{
						m_cov(i,j) = std0sqr * exp( -0.5 * (res2 * static_cast<double>(square(cx1-cx2) +  square(cy1-cy2)))/KF_covSigma2 );
						m_cov(j,i) = m_cov(i,j);
					}
				} // for j
			} // for i

			//m_cov.saveToTextFile("cov_init.txt",1);
		}
		break;
		// and continue with:
	case mrKalmanApproximate:
		{
			m_hasToRecoverMeanAndCov = true;

			CTicTac	tictac;
			tictac.Tic();

			printf("[CRandomFieldGridMap2D::clear] Resetting compressed cov. matrix and cells\n");
			TRandomFieldCell	def(
				m_insertOptions_common->KF_defaultCellMeanValue,									// mean
				m_insertOptions_common->KF_initialCellStd		// std
				);

			fill( def );

			// Reset the covariance matrix:
			// --------------------------------------
			const signed W = m_insertOptions_common->KF_W_size;
			const size_t N = m_map.size();
			const size_t K = 2*W*(W+1)+1;

			const double KF_covSigma2 = square(m_insertOptions_common->KF_covSigma);
			const double std0sqr = square( m_insertOptions_common->KF_initialCellStd );
			const double res2 = square(m_resolution);

			m_stackedCov.setSize( N, K );

			// Populate it with the initial cov. values:
			// ------------------------------------------
			signed Acx, Acy;
			const double	*ptr_first_row = m_stackedCov.get_unsafe_row(0);

			for (size_t i=0;i<N;i++)
			{
				double	*ptr = m_stackedCov.get_unsafe_row(i);

				if (i==0)
				{
					// 1) current cell
					*ptr++ = std0sqr;

					// 2) W rest of the first row:
					Acy = 0;
					for (Acx=1;Acx<=W;Acx++)
						*ptr++ = std0sqr * exp( -0.5 * (res2 * static_cast<double>(square(Acx) +  square(Acy)))/KF_covSigma2 );

					// 3) The others W rows:
					for (Acy=1;Acy<=W;Acy++)
						for (Acx=-W;Acx<=W;Acx++)
							*ptr++ = std0sqr * exp( -0.5 * (res2 * static_cast<double>(square(Acx) +  square(Acy)))/KF_covSigma2 );
				}
				else
				{
					// Just copy the same:
					memcpy( ptr,ptr_first_row, sizeof(double)*K );
				}
			}

			//printf("[CRandomFieldGridMap2D::clear] %ux%u cells done in %.03fms\n", unsigned(m_size_x),unsigned(m_size_y),1000*tictac.Tac());
		}
		break;

	case mrGMRF_G:
		{
			CTicTac	tictac;
			tictac.Tic();

			printf("[CRandomFieldGridMap2D::clear] Generating Prior based on Gaussian\n");

			// Set the grid to initial values:
			TRandomFieldCell	def(0,0);		// mean, std
			fill( def );

			//Set initial restrictions: L "cell Constraints" + 0 "Observations constraints"
			const uint16_t Gsize = m_insertOptions_common->GMRF_constraintsSize;
			const uint16_t Gside = round((Gsize-1)/2);
			const float Gsigma = m_insertOptions_common->GMRF_constraintsSigma;
			gauss_val.resize(2*Gside);
			const size_t N = m_map.size();

			// Weigths of the constratints
			for (uint16_t i=1;i<=2*Gside;i++)  // Compute larger distances than Gside to account for "diagonal" distances.
				gauss_val[i-1] = exp( -0.5 *i /Gsigma );		//Fixed constraints are modeled as Gaussian

			//Determine the number of Initial constraints --> L+M
			nPriorFactors = 0;		// L
			nObsFactors = 0;		// M

			{
				// Avoid the costly % and / operations:
				size_t cx = 0; // ( j % m_size_x );		// [0, m_size_x-1]
				size_t	cy = 0; // ( j / m_size_x );		// [0, m_size_y-1]
				for (size_t j=0; j<N; j++)
				{
					//Determine the Number of restrictions of the current cell j
					//Determine num of columns out of the gridmap
					size_t outc_left = max( 0 , int(Gside-cx) );
					size_t outc_right = max(int (0) , int (Gside-(m_size_x-cx-1)) );
					size_t outc = outc_left + outc_right;
					//Determine num of rows out of the gridmap
					size_t outr_down = max( 0 , int(Gside-(cy)) );
					size_t outr_up = max(int (0) , int (Gside-(m_size_y-cy-1)) );
					size_t outr = outr_up + outr_down;

					nPriorFactors += (Gsize - outc -1) + (Gsize - outr -1 );   //only vertical and horizontal restrictions

					//Increment (row(x), col(y))
					if (++cx>=m_size_x)
					{
						cx=0;
						cy++;
					}
				}
			}

			nFactors = nPriorFactors + nObsFactors;
			cout << "Generating " << nFactors << "cell constraints for a map size of N=" << N << endl;

			//Reset the vector of maps (Hessian_vm), the gradient, and the vector of active observations
#if EIGEN_VERSION_AT_LEAST(3,1,0)
			H_prior.clear();
			H_prior.reserve(nPriorFactors);
#endif
			g.resize(N);			//Initially the gradient is all 0
			g.fill(0.0);
			activeObs.resize(N);	//No initial Observations

			// Load default values:
			{
				// Avoid the costly % and / operations:
				size_t cx = 0; // ( j % m_size_x );		// [0, m_size_x-1]
				size_t cy = 0; // ( j / m_size_x );		// [0, m_size_y-1]
				size_t count = 0;
				for (size_t j=0; j<N; j++)
				{
					//Determine the Number of restrictions of the current cell j
					//Determine num of columns out of the gridmap
					int outc_left = max( 0 , int(Gside-cx) );
					int outc_right = max(int (0) , int (Gside-(m_size_x-cx-1)) );
					int outc = outc_left + outc_right;
					//Determine num of rows out of the gridmap
					int outr_down = max( 0 , int(Gside-(cy)) );
					int outr_up = max(int (0) , int (Gside-(m_size_y-cy-1)) );
					int outr = outr_up + outr_down;

					size_t nConsFixed_j = (Gsize - outc -1) + (Gsize - outr -1 );   //only vertical and horizontal restrictions

					//Set constraints of cell j with all neighbord cells i
					for (int kr=-(Gside-outr_down); kr<=(Gside-outr_up); kr++ )
					{
						for (int kc=-(Gside-outc_left); kc<=(Gside-outc_right); kc++)
						{
							// get index of cell i
							size_t icx = cx + kc;
							size_t icy = cy + kr;
							size_t i = icx + icy*m_size_x;

							if (j==i)
							{
								//H_ii = Nº constraints * Lambda_cell * (J_ij^2 +1)
#if EIGEN_VERSION_AT_LEAST(3,1,0)
								//std::pair<size_t,float> Hentry (j , nConsFixed_j * m_insertOptions_common->GMRF_lambdaPrior * (square(1.0/gauss_val[abs(kr+kc)]) + 1) );
								//H_vm.at(i).insert(Hentry);

								Eigen::Triplet<double> Hentry(i,j , nConsFixed_j * m_insertOptions_common->GMRF_lambdaPrior * (square(1.0/gauss_val[abs(kr+kc)]) + 1) );
								H_prior.push_back( Hentry );
#endif
							}
							else
							{
								if (kr==0 || kc==0)      //only vertical and horizontal restrictions/constraints
								{
									// H_ji = 2 * Lambda_cell * J_ij
#if EIGEN_VERSION_AT_LEAST(3,1,0)
									Eigen::Triplet<double> Hentry(i,j , -2 * m_insertOptions_common->GMRF_lambdaPrior * 1/gauss_val[abs(kr+kc)-1]);
									H_prior.push_back( Hentry );
#endif
									//g_j = [m(j) - alpha*m(i) ]* lambda
									g[j] += (m_map[j].gmrf_mean - m_map[i].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;

									count++;
								}
							}
						}
					}

					// Increment (row(x), col(y))
					if (++cx>=m_size_x)
					{
						cx=0;
						cy++;
					}
				} // end for "j"
			}

			cout << "		Ready in: " << tictac.Tac() << "s" << endl;
			//Solve system and update map estimation
			updateMapEstimation_GMRF();
		}
		break;

	case mrGMRF_SD:
		{
			CTicTac	tictac;
			tictac.Tic();

			printf("[CRandomFieldGridMap2D::clear] Generating Prior based on 'Squared Differences'\n");
			printf("Initial map dimension: %u cells, x=(%.2f,%.2f) y=(%.2f,%.2f)\n", static_cast<unsigned int>(m_map.size()), getXMin(),getXMax(),getYMin(),getYMax());

			// Set the gasgridmap (m_map) to initial values:
			TRandomFieldCell	def(0,0);		// mean, std
			fill( def );

			mrpt::slam::COccupancyGridMap2D	m_Ocgridmap;
			float res_coef = 1.f; // Default value

			if (this->m_insertOptions_common->GMRF_use_occupancy_information)
			{
				printf("loading simplemap: %s ",m_insertOptions_common->GMRF_simplemap_file.c_str());
				printf("%i \n",m_insertOptions_common->GMRF_simplemap_file.compare(""));

				printf("loading map_image: %s ",m_insertOptions_common->GMRF_gridmap_image_file.c_str());
				printf("%i \n",m_insertOptions_common->GMRF_gridmap_image_file.compare(""));
				//Load Occupancy Gridmap and resize
				if( m_insertOptions_common->GMRF_simplemap_file.compare("") )
				{
					//TSetOfMetricMapInitializers	mapList;
					//mapList.loadFromConfigFile(m_ini,"MetricMap");

					//CMultiMetricMap	 metricMap;
					//metricMap.setListOfMaps( &mapList );

					/*printf("Loading '.simplemap' file...");
					mrpt::slam::CSimpleMap MsimpleMap;
					CFileGZInputStream(this->m_insertOptions_common->GMRF_simplemap_file) >> simpleMap;
					printf("Ok (%u poses)\n",(unsigned)simpleMap.size());*/

					//ASSERT_( simpleMap.size()>0 );

					// Build metric map:
					// ------------------------------
					/*printf("Building metric map(s) from '.simplemap'...");
					m_Ocgridmap.loadFromProbabilisticPosesAndObservations(simpleMap);
					printf("Ok\n");*/

					//ASSERTMSG_(!metricMap.m_gridMaps.empty(),"The simplemap file has no gridmap!");

					//m_gridmap = *metricMap.m_gridMaps[0];
					//res_coef = XXX;
				}
				else if( m_insertOptions_common->GMRF_gridmap_image_file.compare("") )
				{
					//Load from image
					bool loaded = m_Ocgridmap.loadFromBitmapFile(this->m_insertOptions_common->GMRF_gridmap_image_file,this->m_insertOptions_common->GMRF_gridmap_image_res,this->m_insertOptions_common->GMRF_gridmap_image_cx,this->m_insertOptions_common->GMRF_gridmap_image_cy);
					printf ("Occupancy Gridmap loaded : %s",loaded ? "YES" : "NO");
					res_coef = this->getResolution() / this->m_insertOptions_common->GMRF_gridmap_image_res;
				}
				else
					cout << "Neither 'simplemap_file' nor 'gridmap_image_file' found in mission file. Quitting."<< endl;

				//Resize GasMap to match Occupancy Gridmap dimmensions
				printf("Resizing m_map to match Occupancy Gridmap dimensions \n");
				resize(m_Ocgridmap.getXMin(),m_Ocgridmap.getXMax(),m_Ocgridmap.getYMin(),m_Ocgridmap.getYMax(),def,0.0);

				printf("Occupancy Gridmap dimensions: x=(%.2f,%.2f)m y=(%.2f,%.2f)m \n",m_Ocgridmap.getXMin(),m_Ocgridmap.getXMax(),m_Ocgridmap.getYMin(),m_Ocgridmap.getYMax());
				printf("Occupancy Gridmap dimensions: %u cells, cx=%i cy=%i\n\n", static_cast<unsigned int>(m_Ocgridmap.getSizeX()*m_Ocgridmap.getSizeY()), m_Ocgridmap.getSizeX(), m_Ocgridmap.getSizeY());
				printf("New map dimensions: %u cells, x=(%.2f,%.2f) y=(%.2f,%.2f)\n", static_cast<unsigned int>(m_map.size()), getXMin(),getXMax(),getYMin(),getYMax());
				printf("New map dimensions: %u cells, cx=%u cy=%u\n", static_cast<unsigned int>(m_map.size()), static_cast<unsigned int>(getSizeX()), static_cast<unsigned int>(getSizeY()));

				m_Ocgridmap.saveAsBitmapFile("./obstacle_map_from_MRPT.png");
			}

			//m_map number of cells
			const size_t N = m_map.size();

			//Set initial factors: L "prior factors" + 0 "Observation factors"
			nPriorFactors = (this->getSizeX() -1) * this->getSizeY() + this->getSizeX() * (this->getSizeY() -1);		// L = (Nr-1)*Nc + Nr*(Nc-1); Full connected
			nObsFactors = 0;		// M
			nFactors = nPriorFactors + nObsFactors;
			cout << "Generating " << nFactors << "factors for a map size of N=" << N << endl;


#if EIGEN_VERSION_AT_LEAST(3,1,0)
			//Initialize H_prior, gradient = 0, and the vector of active observations = empty
			H_prior.clear();
			H_prior.reserve(nPriorFactors);

			g.resize(N);			//Initially the gradient is all 0
			g.fill(0.0);
			activeObs.resize(N);	//No initial Observations


			//-------------------------------------
			// Load default values for H_prior:
			//-------------------------------------
			if (this->m_insertOptions_common->GMRF_use_occupancy_information)
			{
				printf("LOADING PRIOR BASED ON OCCUPANCY GRIDMAP \n");
				printf("Gas Map Dimmensions: %u x %u cells \n", static_cast<unsigned int>(m_size_x), static_cast<unsigned int>(m_size_y));
				printf("Occupancy map Dimmensions: %i x %i cells \n", m_Ocgridmap.getSizeX(), m_Ocgridmap.getSizeY());
				printf("Res_Coeff = %f pixels/celda",res_coef);

				//Use region growing algorithm to determine the gascell interconnections (Factors)
				size_t cx = 0;
				size_t cy = 0;

				size_t cxoj_min, cxoj_max, cyoj_min, cyoj_max, seed_cxo, seed_cyo;				//Cell j limits in the Occupancy
				size_t cxoi_min, cxoi_max, cyoi_min, cyoi_max, objective_cxo, objective_cyo;	//Cell i limits in the Occupancy
				size_t cxo_min, cxo_max, cyo_min, cyo_max;										//Cell i+j limits in the Occupancy
				//bool first_obs = false;

				for (size_t j=0; j<N; j++)		//For each cell in the gas_map
				{
					// Get cell_j indx-limits in Occuppancy gridmap
					cxoj_min = floor(cx*res_coef);
					cxoj_max = cxoj_min + ceil(res_coef-1);
					cyoj_min = floor(cy*res_coef);
					cyoj_max = cyoj_min + ceil(res_coef-1);

					seed_cxo = cxoj_min + ceil(res_coef/2-1);
					seed_cyo = cyoj_min + ceil(res_coef/2-1);

								//DEBUG
								//If cell occpuped then add observation
								if ( m_Ocgridmap.getCell(seed_cxo,seed_cyo) < 0.5 )
								{
									TobservationGMRF new_obs;
									new_obs.obsValue = 0.0;
									new_obs.Lambda = 10e-5f;
									new_obs.time_invariant = true;	//Obs that will not dissapear with time.
									activeObs[j].push_back(new_obs);
								}

								//// Insert 1 observation in the vector of Active Observations for a free cell
								//if( ( m_Ocgridmap.getCell(seed_cxo,seed_cyo) > 0.5 ) && first_obs )
								//{
								//	TobservationGMRF new_obs;
								//	new_obs.obsValue = 0.0;
								//	new_obs.Lambda = m_insertOptions_common->GMRF_lambdaObs;
								//	activeObs[j].push_back(new_obs);
								//	first_obs = false;
								//	cout << " **** Inserting Observation at cell: " << j << endl;
								//}


					//Factor with the right node: H_ji = - Lamda_prior
					//-------------------------------------------------
					if (cx<(m_size_x-1))
					{
						size_t i = j+1;
						size_t cxi = cx+1;
						size_t cyi = cy;

						// Get cell_i indx-limits in Occuppancy gridmap
						cxoi_min = floor(cxi*res_coef);
						cxoi_max = cxoi_min + ceil(res_coef-1);
						cyoi_min = floor(cyi*res_coef);
						cyoi_max = cyoi_min + ceil(res_coef-1);

						objective_cxo = cxoi_min + ceil(res_coef/2-1);
						objective_cyo = cyoi_min + ceil(res_coef/2-1);

						//Get overall indx of both cells together
						cxo_min = min(cxoj_min, cxoi_min );
						cxo_max = max(cxoj_max, cxoi_max );
						cyo_min = min(cyoj_min, cyoi_min );
						cyo_max = max(cyoj_max, cyoi_max );

						//Check using Region growing if cell j is connected to cell i (Occupancy gridmap)
						if( exist_relation_between2cells(&m_Ocgridmap, cxo_min,cxo_max,cyo_min,cyo_max,seed_cxo,seed_cyo,objective_cxo,objective_cyo))
						{
							Eigen::Triplet<double> Hentry(i,j, - m_insertOptions_common->GMRF_lambdaPrior);
							H_prior.push_back( Hentry );

							//Save relation between cells
							cell_interconnections.insert ( std::pair<size_t,size_t>(j,i) );
							cell_interconnections.insert ( std::pair<size_t,size_t>(i,j) );
						}
					}

					//Factor with the upper node: H_ji = - Lamda_prior
					//-------------------------------------------------
					if (cy<(m_size_y-1))
					{
						size_t i = j+m_size_x;
						size_t cxi = cx;
						size_t cyi = cy+1;

						// Get cell_i indx-limits in Occuppancy gridmap
						cxoi_min = floor(cxi*res_coef);
						cxoi_max = cxoi_min + ceil(res_coef-1);
						cyoi_min = floor(cyi*res_coef);
						cyoi_max = cyoi_min + ceil(res_coef-1);

						objective_cxo = cxoi_min + ceil(res_coef/2-1);
						objective_cyo = cyoi_min + ceil(res_coef/2-1);

						//Get overall indx-limits of both cells together
						cxo_min = min(cxoj_min, cxoi_min );
						cxo_max = max(cxoj_max, cxoi_max );
						cyo_min = min(cyoj_min, cyoi_min );
						cyo_max = max(cyoj_max, cyoi_max );


						//Check using Region growing if cell j is connected to cell i (Occupancy gridmap)
						if( exist_relation_between2cells(&m_Ocgridmap, cxo_min,cxo_max,cyo_min,cyo_max,seed_cxo,seed_cyo,objective_cxo,objective_cyo))
						{
							Eigen::Triplet<double> Hentry(i,j, - m_insertOptions_common->GMRF_lambdaPrior);
							H_prior.push_back( Hentry );

							//Save relation
							cell_interconnections.insert ( std::pair<size_t,size_t>(j,i) );
							cell_interconnections.insert ( std::pair<size_t,size_t>(i,j) );
						}
					}

					//Factors of cell_j: H_jj = Nº factors * Lambda_prior
					//----------------------------------------------------
					std::pair < std::multimap<size_t,size_t>::iterator, std::multimap<size_t,size_t>::iterator > range;
					range = cell_interconnections.equal_range(j);
					size_t nFactors_j = 0;
					while ( range.first!=range.second )
					{
						nFactors_j++;
						range.first++;
					}
					Eigen::Triplet<double> Hentry(j,j , nFactors_j * m_insertOptions_common->GMRF_lambdaPrior );
					H_prior.push_back( Hentry );

					// Increment j coordinates (row(x), col(y))
					if (++cx>=m_size_x)
					{
						cx=0;
						cy++;
					}
				} // end for "j"

				//DEBUG - Save cell interconnections to file
				ofstream myfile;
				myfile.open("MRF.txt");
				for (std::multimap<size_t,size_t>::iterator it=cell_interconnections.begin(); it!=cell_interconnections.end(); ++it)
					myfile << (*it).first << " " << (*it).second << '\n';
				myfile.close();

			}
			else
			{
				cout << "LOADING FULL PRIOR" << endl;
				//---------------------------------------------------------------
				// Load default values for H_prior without Occupancy information:
				//---------------------------------------------------------------
				size_t cx = 0;
				size_t cy = 0;
				size_t count = 0;
				for (size_t j=0; j<N; j++)
				{
					//Factor with the right node: H_ji = - Lamda_prior
					//-------------------------------------------------
					if (cx<(m_size_x-1))
					{
						size_t i = j+1;


						Eigen::Triplet<double> Hentry(i,j, - m_insertOptions_common->GMRF_lambdaPrior);
						H_prior.push_back( Hentry );
						count = count +1;
					}

					//Factor with the above node: H_ji = - Lamda_prior
					//-------------------------------------------------
					if (cy<(m_size_y-1))
					{
						size_t i = j+m_size_x;

						Eigen::Triplet<double> Hentry(i,j, - m_insertOptions_common->GMRF_lambdaPrior);
						H_prior.push_back( Hentry );
						count = count +1;
					}

					//Factors of cell_i: H_ii = Nº factors * Lambda_prior
					//----------------------------------------------------
					size_t nFactors_j = 4 - (cx==0) - (cx==m_size_x-1) - (cy==0) - (cy==m_size_y-1);
					Eigen::Triplet<double> Hentry(j,j , nFactors_j * m_insertOptions_common->GMRF_lambdaPrior );
					H_prior.push_back( Hentry );

					// Increment j coordinates (row(x), col(y))
					if (++cx>=m_size_x)
					{
						cx=0;
						cy++;
					}
				} // end for "j"
			} // end if_use_Occupancy

			cout << "---------- Prior Built in: " << tictac.Tac() << "s ----------" << endl;

			//Solve system and update map estimation
			updateMapEstimation_GMRF();
#endif
		}//end case
		break;
	default:
		cout << "MAP TYPE NOT RECOGNIZED... QUITTING" << endl;
		break;
	} //end switch

}

/*---------------------------------------------------------------
						isEmpty
  ---------------------------------------------------------------*/
bool  CRandomFieldGridMap2D::isEmpty() const
{
	return false;
}


/*---------------------------------------------------------------
					insertObservation_KernelDM_DMV
  ---------------------------------------------------------------*/
/** The implementation of "insertObservation" for Achim Lilienthal's map models DM & DM+V.
* \param normReading Is a [0,1] normalized concentration reading.
* \param is_DMV = false -> map type is Kernel DM; true -> map type is DM+V
*/
void  CRandomFieldGridMap2D::insertObservation_KernelDM_DMV(
	float			normReading,
	const mrpt::math::TPoint2D &point,
	bool             is_DMV )
{
	MRPT_START

	static const TRandomFieldCell defCell(0,0);

	// Assure we have room enough in the grid!
	resize(	point.x - m_insertOptions_common->cutoffRadius*2,
			point.x + m_insertOptions_common->cutoffRadius*2,
			point.y - m_insertOptions_common->cutoffRadius*2,
			point.y + m_insertOptions_common->cutoffRadius*2,
			defCell );

	// Compute the "parzen Gaussian" once only:
	// -------------------------------------------------
	int						Ac_cutoff = round(m_insertOptions_common->cutoffRadius / m_resolution);
	unsigned				Ac_all = 1+2*Ac_cutoff;
	double					minWinValueAtCutOff = exp(-square(m_insertOptions_common->cutoffRadius/m_insertOptions_common->sigma) );

	if ( m_DM_lastCutOff!=m_insertOptions_common->cutoffRadius ||
			m_DM_gaussWindow.size() != square(Ac_all) )
	{
		printf("[CRandomFieldGridMap2D::insertObservation_KernelDM_DMV] Precomputing window %ux%u\n",Ac_all,Ac_all);

		double	dist;
		double	std = m_insertOptions_common->sigma;

		// Compute the window:
		m_DM_gaussWindow.resize(Ac_all*Ac_all);
		m_DM_lastCutOff=m_insertOptions_common->cutoffRadius;

		// Actually the array could be 1/4 of this size, but this
		// way it's easier and it's late night now :-)
		vector<float>::iterator	it = m_DM_gaussWindow.begin();
        for (unsigned cx=0;cx<Ac_all;cx++)
		{
			for (unsigned cy=0;cy<Ac_all;cy++)
			{
				dist = m_resolution * sqrt( static_cast<double>(square( Ac_cutoff+1-cx ) + square( Ac_cutoff+1-cy ) ) );
				*(it++) = std::exp( - square(dist/std) );
			}
		}

		printf("[CRandomFieldGridMap2D::insertObservation_KernelDM_DMV] Done!\n");
	} // end of computing the gauss. window.

	//	Fuse with current content of grid (the MEAN of each cell):
	// --------------------------------------------------------------
	const int sensor_cx = x2idx( point.x );
	const int sensor_cy = y2idx( point.y );
	TRandomFieldCell	*cell;
	vector<float>::iterator	windowIt = m_DM_gaussWindow.begin();

	for (int Acx=-Ac_cutoff;Acx<=Ac_cutoff;Acx++)
	{
		for (int Acy=-Ac_cutoff;Acy<=Ac_cutoff;++Acy, ++windowIt)
		{
			const double windowValue = *windowIt;

			if (windowValue>minWinValueAtCutOff)
			{
				cell = cellByIndex(sensor_cx+Acx,sensor_cy+Acy);
				ASSERT_( cell!=NULL )

				cell->dm_mean_w  += windowValue;
				cell->dm_mean += windowValue * normReading;
				if (is_DMV)
				{
					const double cell_var = square(normReading - computeMeanCellValue_DM_DMV(cell) );
					cell->dmv_var_mean += windowValue * cell_var;
				}
			}
		}
	}

	MRPT_END
}


/*---------------------------------------------------------------
					TInsertionOptionsCommon
 ---------------------------------------------------------------*/
CRandomFieldGridMap2D::TInsertionOptionsCommon::TInsertionOptionsCommon() :
	sigma				( 0.15f ),
	cutoffRadius		( sigma * 3.0 ),
	R_min				( 0 ),
	R_max				( 3 ),
	dm_sigma_omega				( 0.05 ),  // See IROS 2009 paper (a scale parameter for the confidence)

	KF_covSigma					( 0.35f ),		// in meters
	KF_initialCellStd			( 1.0 ),		// std in normalized concentration units
	KF_observationModelNoise	( 0.25f ),		// in normalized concentration units
	KF_defaultCellMeanValue		( 0.25f ),
	KF_W_size					( 4 ),

	GMRF_lambdaPrior			( 0.01f ),		// [GMRF model] The information (Lambda) of fixed map constraints
	GMRF_lambdaObs				( 10.0f ),		// [GMRF model] The initial information (Lambda) of each observation (this information will decrease with time)
	GMRF_lambdaObsLoss			( 1.0f ),		//!< The loss of information of the observations with each iteration

	GMRF_use_occupancy_information	( false ),
	GMRF_simplemap_file				( "" ),
	GMRF_gridmap_image_file			( "" ),
	GMRF_gridmap_image_res			( 0.01f ),
	GMRF_gridmap_image_cx			( 0 ),
	GMRF_gridmap_image_cy			( 0 ),

	GMRF_constraintsSize		( 3	),			// [GMRF model] The size (cells) of the Gaussian window to impose fixed restrictions between cells.
	GMRF_constraintsSigma		( 4.0f )  		// [GMRF model] The sigma of the Gaussian window to impose fixed restrictions between cells.
{
}

/*---------------------------------------------------------------
					internal_dumpToTextStream_common
  ---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::TInsertionOptionsCommon::internal_dumpToTextStream_common(CStream	&out) const
{
	out.printf("sigma                                   = %f\n", sigma);
	out.printf("cutoffRadius                            = %f\n", cutoffRadius);
	out.printf("R_min                                   = %f\n", R_min);
	out.printf("R_max                                   = %f\n", R_max);
	out.printf("dm_sigma_omega	                        = %f\n", dm_sigma_omega);

	out.printf("KF_covSigma                             = %f\n", KF_covSigma);
	out.printf("KF_initialCellStd                       = %f\n", KF_initialCellStd);
	out.printf("KF_observationModelNoise                = %f\n", KF_observationModelNoise);
	out.printf("KF_defaultCellMeanValue                 = %f\n", KF_defaultCellMeanValue);
	out.printf("KF_W_size                               = %u\n", (unsigned)KF_W_size);

	out.printf("GMRF_lambdaPrior						= %f\n", GMRF_lambdaPrior);
	out.printf("GMRF_lambdaObs	                        = %f\n", GMRF_lambdaObs);
	out.printf("GMRF_lambdaObsLoss                      = %f\n", GMRF_lambdaObs);

	out.printf("GMRF_use_occupancy_information			= %s\n", GMRF_use_occupancy_information ? "YES":"NO" );
	out.printf("GMRF_simplemap_file						= %s\n", GMRF_simplemap_file.c_str());
	out.printf("GMRF_gridmap_image_file					= %s\n", GMRF_gridmap_image_file.c_str());
	out.printf("GMRF_gridmap_image_res					= %f\n", GMRF_gridmap_image_res);
	out.printf("GMRF_gridmap_image_cx					= %u\n", static_cast<unsigned int>(GMRF_gridmap_image_cx));
	out.printf("GMRF_gridmap_image_cy					= %u\n", static_cast<unsigned int>(GMRF_gridmap_image_cy));

	out.printf("GMRF_constraintsSize                    = %u\n", (unsigned)GMRF_constraintsSize);
	out.printf("GMRF_constraintsSigma	                = %f\n", GMRF_constraintsSigma);
}

/*---------------------------------------------------------------
					internal_loadFromConfigFile_common
  ---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::TInsertionOptionsCommon::internal_loadFromConfigFile_common(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	sigma					= iniFile.read_float(section.c_str(),"sigma",sigma);
	cutoffRadius			= iniFile.read_float(section.c_str(),"cutoffRadius",cutoffRadius);
	R_min					= iniFile.read_float(section.c_str(),"R_min",R_min);
	R_max					= iniFile.read_float(section.c_str(),"R_max",R_max);
	MRPT_LOAD_CONFIG_VAR(dm_sigma_omega, double,   iniFile, section );

	KF_covSigma				= iniFile.read_float(section.c_str(),"KF_covSigma",KF_covSigma);
	KF_initialCellStd		= iniFile.read_float(section.c_str(),"KF_initialCellStd",KF_initialCellStd);
	KF_observationModelNoise= iniFile.read_float(section.c_str(),"KF_observationModelNoise",KF_observationModelNoise);
	KF_defaultCellMeanValue = iniFile.read_float(section.c_str(),"KF_defaultCellMeanValue",KF_defaultCellMeanValue);
	MRPT_LOAD_CONFIG_VAR(KF_W_size, int,   iniFile, section );

	GMRF_lambdaPrior		= iniFile.read_float(section.c_str(),"GMRF_lambdaPrior",GMRF_lambdaPrior);
	GMRF_lambdaObs			= iniFile.read_float(section.c_str(),"GMRF_lambdaObs",GMRF_lambdaObs);
	GMRF_lambdaObsLoss		= iniFile.read_float(section.c_str(),"GMRF_lambdaObsLoss",GMRF_lambdaObsLoss);

	GMRF_use_occupancy_information	= iniFile.read_bool(section.c_str(),"GMRF_use_occupancy_information",false,false);
	GMRF_simplemap_file				= iniFile.read_string(section.c_str(),"simplemap_file","",false);
	GMRF_gridmap_image_file			= iniFile.read_string(section.c_str(),"gridmap_image_file","",false);
	GMRF_gridmap_image_res			= iniFile.read_float(section.c_str(),"gridmap_image_res",0.01,false);
	GMRF_gridmap_image_cx			= iniFile.read_int(section.c_str(),"gridmap_image_cx",0,false);
	GMRF_gridmap_image_cy			= iniFile.read_int(section.c_str(),"gridmap_image_cy",0,false);

	GMRF_constraintsSigma	= iniFile.read_float(section.c_str(),"GMRF_constraintsSigma",GMRF_constraintsSigma);
	MRPT_LOAD_CONFIG_VAR(GMRF_constraintsSize, int,   iniFile, section );
}



/*---------------------------------------------------------------
					saveAsBitmapFile
 ---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::saveAsBitmapFile(const std::string &filName) const
{
	MRPT_START

	mrpt::utils::CImage img;
	getAsBitmapFile(img);
	img.saveToFile(filName);

	MRPT_END
}

/*---------------------------------------------------------------
					getAsBitmapFile
 ---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::getAsBitmapFile(mrpt::utils::CImage &out_img) const
{
	MRPT_START

	unsigned int	x,y;
	double c;
	const TRandomFieldCell	*cell;

	mrpt::math::CMatrixDouble cells_mat(m_size_y,m_size_x);

	recoverMeanAndCov();	// Only has effects for KF2 method

	for (y=0;y<m_size_y;y++)
	{
		for (x=0;x<m_size_x;x++)
		{
			cell = cellByIndex(x,y);
			ASSERT_( cell!=NULL );

			switch (m_mapType)
			{
			case mrKernelDM:
			case mrKernelDMV:
				c = computeMeanCellValue_DM_DMV(cell);
				break;

			case mrKalmanFilter:
			case mrKalmanApproximate:
				c = cell->kf_mean;
				break;
			case mrGMRF_G:
			case mrGMRF_SD:
				c = cell->gmrf_mean;
				break;

			default:
				THROW_EXCEPTION("Unknown m_mapType!!");
			};

			if (c<0) c=0;
			if (c>1) c=1;

			cells_mat(m_size_y-1-y,x) = c;
		}
	}

	out_img.setFromMatrix(cells_mat, true /* vals are normalized in [0,1] */);

	MRPT_END
}

/*---------------------------------------------------------------
					resize
 ---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::resize(
	float	new_x_min,
	float	new_x_max,
	float	new_y_min,
	float	new_y_max,
	const TRandomFieldCell& defaultValueNewCells,
	float	additionalMarginMeters)
{
	MRPT_START

	size_t		old_sizeX = m_size_x;
	size_t		old_sizeY = m_size_y;
	float		old_x_min = m_x_min;
	float		old_y_min = m_y_min;

	// The parent class method:
	CDynamicGrid<TRandomFieldCell>::resize(new_x_min,new_x_max,new_y_min,new_y_max,defaultValueNewCells,additionalMarginMeters);

	// Do we really resized?
	if ( m_size_x != old_sizeX || m_size_y != old_sizeY )
	{
		// YES:
		// If we are in a Kalman Filter representation, also build the new covariance matrix:
		if (m_mapType==mrKalmanFilter)
		{
			// ------------------------------------------
			//		Update the covariance matrix
			// ------------------------------------------
			size_t			i,j,N = m_size_y*m_size_x;	// The new number of cells
			CMatrixD		oldCov( m_cov );		// Make a copy

			//m_cov.saveToTextFile("__debug_cov_before_resize.txt");

			printf("[CRandomFieldGridMap2D::resize] Resizing from %ux%u to %ux%u (%u cells)\n",
				static_cast<unsigned>(old_sizeX),
				static_cast<unsigned>(old_sizeY),
				static_cast<unsigned>(m_size_x),
				static_cast<unsigned>(m_size_y),
				static_cast<unsigned>(m_size_x*m_size_y) );

			m_cov.setSize( N,N );

			// Compute the new cells at the left and the bottom:
			size_t	Acx_left = round((old_x_min - m_x_min)/m_resolution);
			size_t	Acy_bottom = round((old_y_min - m_y_min)/m_resolution);

			// -------------------------------------------------------
			// STEP 1: Copy the old map values:
			// -------------------------------------------------------
			for (i = 0;i<N;i++)
			{
				size_t	cx1 = i % m_size_x;
				size_t	cy1 = i / m_size_x;

				bool	C1_isFromOldMap =	Acx_left<=cx1 && cx1<(Acx_left+old_sizeX) &&
											Acy_bottom<=cy1 && cy1<(Acy_bottom+old_sizeY);

				if ( C1_isFromOldMap )
				{
					for (j = i;j<N;j++)
					{
						size_t	cx2 = j % m_size_x;
						size_t	cy2 = j / m_size_x;

						bool	C2_isFromOldMap =	Acx_left<=cx2 && cx2<(Acx_left+old_sizeX) &&
													Acy_bottom<=cy2 && cy2<(Acy_bottom+old_sizeY);

						// Were both cells in the old map??? --> Copy it!
						if ( C1_isFromOldMap && C2_isFromOldMap )
						{
							// Copy values for the old matrix:
							unsigned int	idx_c1 = ( (cx1 - Acx_left) + old_sizeX * (cy1 - Acy_bottom ) );
							unsigned int	idx_c2 = ( (cx2 - Acx_left) + old_sizeX * (cy2 - Acy_bottom ) );

							MRPT_START

							ASSERT_( cx1 >= Acx_left );
							ASSERT_( cy1 >= Acy_bottom );
							ASSERT_( (cx1 - Acx_left )<old_sizeX );
							ASSERT_( (cy1 - Acy_bottom )<old_sizeY );

							ASSERT_( cx2 >= Acx_left );
							ASSERT_( cy2 >= Acy_bottom );
							ASSERT_( (cx2 - Acx_left )<old_sizeX );
							ASSERT_( (cy2 - Acy_bottom )<old_sizeY );

							ASSERT_( idx_c1<old_sizeX*old_sizeY );
							ASSERT_( idx_c2<old_sizeX*old_sizeY );

							MRPT_END_WITH_CLEAN_UP( \
								printf("cx1=%u\n",static_cast<unsigned>(cx1)); \
								printf("cy1=%u\n",static_cast<unsigned>(cy1)); \
								printf("cx2=%u\n",static_cast<unsigned>(cx2)); \
								printf("cy2=%u\n",static_cast<unsigned>(cy2)); \
								printf("Acx_left=%u\n",static_cast<unsigned>(Acx_left)); \
								printf("Acy_bottom=%u\n",static_cast<unsigned>(Acy_bottom)); \
								printf("idx_c1=%u\n",static_cast<unsigned>(idx_c1)); \
								printf("idx_c2=%u\n",static_cast<unsigned>(idx_c2)); \
								);

							m_cov(i,j) = oldCov( idx_c1, idx_c2 );
							m_cov(j,i) = m_cov(i,j);

							if (i==j)
								ASSERT_( idx_c1 == idx_c2 );

							if (i==j && m_cov(i,i)<0)
							{
								printf("\ni=%u \nj=%i \ncx1,cy1 = %u,%u \n cx2,cy2=%u,%u \nidx_c1=%u \nidx_c2=%u \nAcx_left=%u \nAcy_bottom=%u \nold_sizeX=%u \n",
									static_cast<unsigned>(i),
									static_cast<unsigned>(j),
									static_cast<unsigned>(cx1),
									static_cast<unsigned>(cy1),
									static_cast<unsigned>(cx2),
									static_cast<unsigned>(cy2),
									static_cast<unsigned>(idx_c1),
									static_cast<unsigned>(idx_c2),
									static_cast<unsigned>(Acx_left),
									static_cast<unsigned>(Acy_bottom),
									static_cast<unsigned>(old_sizeX)
									);
							}
						}

						ASSERT_( !isNaN( m_cov(i,j) ) );

					} // for j
				}
			} // for i

			// -------------------------------------------------------
			// STEP 2: Set default values for new cells
			// -------------------------------------------------------
			for (i=0;i<N;i++)
			{
				size_t	cx1 = i % m_size_x;
				size_t	cy1 = i / m_size_x;

				bool	C1_isFromOldMap =	Acx_left<=cx1 && cx1<(Acx_left+old_sizeX) &&
											Acy_bottom<=cy1 && cy1<(Acy_bottom+old_sizeY);
				for (j=i;j<N;j++)
				{
					size_t	cx2 = j % m_size_x;
					size_t	cy2 = j / m_size_x;

					bool	C2_isFromOldMap =	Acx_left<=cx2 && cx2<(Acx_left+old_sizeX) &&
												Acy_bottom<=cy2 && cy2<(Acy_bottom+old_sizeY);
					double	dist=0;

					// If both cells were NOT in the old map??? --> Introduce default values:
					if ( !C1_isFromOldMap || !C2_isFromOldMap )
					{
						// Set a new starting value:
						if (i==j)
						{
							m_cov(i,i) = square(  m_insertOptions_common->KF_initialCellStd );
						}
						else
						{
							dist = m_resolution*sqrt( static_cast<double>( square(cx1-cx2) +  square(cy1-cy2) ));
							double K = sqrt(m_cov(i,i)*m_cov(j,j));

							if ( isNaN( K ) )
							{
								printf("c(i,i)=%e   c(j,j)=%e\n",m_cov(i,i),m_cov(j,j));
								ASSERT_( !isNaN( K ) );
							}

							m_cov(i,j) = K * exp( -0.5 * square( dist/m_insertOptions_common->KF_covSigma ) );
							m_cov(j,i) = m_cov(i,j);
						}

						ASSERT_( !isNaN( m_cov(i,j) ) );
					}
				} // for j
			} // for i


			//m_cov.saveToTextFile("__debug_cov_after_resize.txt");
			// Resize done!
			printf("[CRandomFieldGridMap2D::resize] Done\n");

		} // end of Kalman Filter map
		else if (m_mapType==mrKalmanApproximate )
		{
			// ------------------------------------------
			//		Approximate-Kalman filter
			// ------------------------------------------

			// Cells with "std" == -1 are new ones, we have to change their std
			//   to "m_insertOptions_common->KF_initialCellStd", then adapt appropriately
			//   the compressed cov. matrix:


			/*printf("[CRandomFieldGridMap2D::resize] Resizing from %ux%u to %ux%u (%u cells)\n",
				static_cast<unsigned>(old_sizeX),
				static_cast<unsigned>(old_sizeY),
				static_cast<unsigned>(m_size_x),
				static_cast<unsigned>(m_size_y),
				static_cast<unsigned>(m_size_x*m_size_y) );*/

			// Adapt the size of the cov. matrix:
			const signed	W = m_insertOptions_common->KF_W_size;
			const size_t	N = m_map.size();
			const size_t	K = 2*W*(W+1)+1;
			ASSERT_( K==m_stackedCov.getColCount() );
			ASSERT_( old_sizeX*old_sizeY== m_stackedCov.getRowCount() );

			// Compute the new cells at the left and the bottom:
			size_t	Acx_left = round((old_x_min - m_x_min)/m_resolution);
			size_t	Acy_bottom = round((old_y_min - m_y_min)/m_resolution);

			m_stackedCov.setSize( N,K );

			// Prepare the template for new cells:
			CVectorDouble template_row(K);
			{
				const double	std0sqr	= square(  m_insertOptions_common->KF_initialCellStd );
				double			*ptr	= &template_row[0];
				const double	res2	= square(m_resolution);
				const double	KF_covSigma2 = square(m_insertOptions_common->KF_covSigma);

				// 1) current cell
				*ptr++ = std0sqr;

				// 2) W rest of the first row:
				int Acx, Acy = 0;
				for (Acx=1;Acx<=W;Acx++)
					*ptr++ = std0sqr * exp( -0.5 * (res2 * static_cast<double>(square(Acx) +  square(Acy)))/KF_covSigma2 );

				// 3) The others W rows:
				for (Acy=1;Acy<=W;Acy++)
					for (Acx=-W;Acx<=W;Acx++)
						*ptr++ = std0sqr * exp( -0.5 * (res2 * static_cast<double>(square(Acx) +  square(Acy)))/KF_covSigma2 );
			}

			// Go thru all the cells, from the bottom to the top so
			//  we don't need to make a temporary copy of the covariances:
			for (size_t i=N-1;i<N;i--) // i<N will become false for "i=-1" ;-)
			{
				int	cx,cy;
				idx2cxcy(i,cx,cy);

				const size_t			old_idx_of_i = (cx-Acx_left) + (cy-Acy_bottom)*old_sizeX;

				TRandomFieldCell	&cell = m_map[i];
				if (cell.kf_std<0)
				{
					// "i" is a new cell, fix its std and fill out the compressed covariance:
					cell.kf_std = m_insertOptions_common->KF_initialCellStd;

					double	*new_row = m_stackedCov.get_unsafe_row(i);
					memcpy( new_row,&template_row[0], sizeof(double)*K );
				}
				else
				{
					// "i" is an existing old cell: just copy the "m_stackedCov" row:
					ASSERT_(old_idx_of_i<m_stackedCov.getRowCount());
					if (old_idx_of_i!=i)		// Copy row only if it's moved
					{
						const double *ptr_old	= m_stackedCov.get_unsafe_row(old_idx_of_i);
						double	*ptr_new		= m_stackedCov.get_unsafe_row(i);
						memcpy( ptr_new,ptr_old, sizeof(double)*K );
					}
				}
			} // end for i
		} // end of Kalman-Approximate map
	}

	MRPT_END
}


/*---------------------------------------------------------------
					insertObservation_KF
  ---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::insertObservation_KF(
	float			normReading,
	const mrpt::math::TPoint2D &point )
{
	MRPT_START

	const TRandomFieldCell defCell(
				m_insertOptions_common->KF_defaultCellMeanValue,	// mean
				m_insertOptions_common->KF_initialCellStd			// std
				);

	// DEBUG
	// Save to file the actual cov_matrix to plot it with matlab
	//m_cov.saveToTextFile( std::string("LOG_ICP-SLAM\_mean_compressed_cov.txt"), MATRIX_FORMAT_FIXED );

	// Assure we have room enough in the grid!
	resize(	point.x - 1,
			point.x + 1,
			point.y - 1,
			point.y + 1,
			defCell );

	// --------------------------------------------------------
	// The Kalman-Filter estimation of the gas grid-map:
	// --------------------------------------------------------

	// Prediction stage of KF:
	// ------------------------------------
	// Nothing to do here (static map)

	// Update stage of KF:
	//  We directly apply optimized formulas arising
	//   from our concrete sensor model.
	// -------------------------------------------------
	int						cellIdx = xy2idx( point.x, point.y );
	TRandomFieldCell	*cell = cellByPos( point.x, point.y );
	ASSERT_(cell!=NULL);
	size_t					N,i,j;

	double		yk = normReading - cell->kf_mean;		// Predicted observation mean
	double		sk = m_cov(cellIdx,cellIdx) + square(m_insertOptions_common->KF_observationModelNoise);	// Predicted observation variance
	double		sk_1 = 1.0 / sk;

	// The kalman gain:
//	CMatrixD		Kk;
//	CVectorDouble	Kk;
//	m_cov.extractCol( cellIdx,Kk );
//	Kk *= 1.0/sk;

	//Kk.saveToTextFile("__debug_Kk.txt");

	std::vector<TRandomFieldCell>::iterator	it;

#if RANDOMFIELDGRIDMAP_VERBOSE
	CTicTac		tictac;
	//cout << "[insertObservation_KF] Sensor: " << point << " measur: " << normReading << endl;
	printf("[insertObservation_KF] Updating mean values...");
	tictac.Tic();
#endif

	// Update mean values:
	// ---------------------------------------------------------
	for (i=0,it = m_map.begin();it!=m_map.end();it++,i++)
		//it->kf_mean =  it->kf_mean + yk * sk_1 * m_cov.get_unsafe(i,cellIdx);
		it->kf_mean +=  yk * sk_1 * m_cov(i,cellIdx);

#if RANDOMFIELDGRIDMAP_VERBOSE
	printf("Done in %.03fms\n",	tictac.Tac()*1000 );
#endif

	// Update covariance matrix values:
	// ---------------------------------------------------------
	N = m_cov.getRowCount();

#if RANDOMFIELDGRIDMAP_VERBOSE
	printf("[insertObservation_KF] Updating covariance matrix...");
	tictac.Tic();
#endif

	// We need to refer to the old cov: make an efficient copy of it:
	//CMatrixD		oldCov( m_cov );
	double	*oldCov	= (double*)/*mrpt_alloca*/malloc( sizeof(double)*N*N );
	double  *oldCov_ptr = oldCov;
	for (i=0;i<N;i++)
	{
		memcpy( oldCov_ptr, m_cov.get_unsafe_row(i) , sizeof(double)*N );
		oldCov_ptr+=N;
	}

//	m_cov.saveToTextFile("bef.txt");

#if RANDOMFIELDGRIDMAP_VERBOSE
	printf("Copy matrix %ux%u: %.06fms\n",	(unsigned)m_cov.getRowCount(), (unsigned)m_cov.getColCount(),  tictac.Tac()*1000 );
#endif

	// The following follows from the expansion of Kalman Filter matrix equations
	// TODO: Add references to some paper (if any)?
	const double *oldCov_row_c = oldCov+cellIdx*N;
	for (i=0;i<N;i++)
	{
		const double oldCov_i_c = oldCov[i*N+cellIdx];
		const double sk_1_oldCov_i_c = sk_1 * oldCov_i_c;

		const double *oldCov_row_i = oldCov+i*N;
		for (j=i;j<N;j++)
		{
			double new_cov_ij = oldCov_row_i[j] - sk_1_oldCov_i_c * oldCov_row_c[j];

			// Make symmetric:
			m_cov.set_unsafe(i,j, new_cov_ij );
			m_cov.set_unsafe(j,i, new_cov_ij );

			// Update the "std" in the cell as well:
			if (i==j)
			{
				if (m_cov(i,i)<0){
					// JL on static_cast<unsigned int>(i): gcc warns (rightly!) that size_t != unsigned int ("%u")
					printf("Wrong insertion in KF! m_cov(%u,%u) = %.5f",static_cast<unsigned int>(i),static_cast<unsigned int>(i),m_cov(i,i));
				}

				ASSERT_( m_cov(i,i)>=0 );
				m_map[ i ].kf_std = sqrt( new_cov_ij );
			}
			//ASSERT_( !isNaN( m_cov(i,j) ) );

		} // j
	} // i

	// Free mem:
	/*mrpt_alloca_*/ free( oldCov );

#if RANDOMFIELDGRIDMAP_VERBOSE
	printf("Done! %.03fms\n",	tictac.Tac()*1000 );
#endif

//	m_cov.saveToTextFile("aft.txt");
	//mrpt::system::pause();

	MRPT_END
}


/*---------------------------------------------------------------
					saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::saveMetricMapRepresentationToFile(
	const std::string	&filNamePrefix ) const
{
	std::string		fil;

	// Save as a bitmap:
	fil = filNamePrefix + std::string("_mean.png");
	saveAsBitmapFile( fil );

	// Save dimensions of the grid (for any mapping algorithm):
	CMatrix DIMs(1,4);
	DIMs(0,0)=m_x_min;
	DIMs(0,1)=m_x_max;
	DIMs(0,2)=m_y_min;
	DIMs(0,3)=m_y_max;

	DIMs.saveToTextFile( filNamePrefix + std::string("_grid_limits.txt"), MATRIX_FORMAT_FIXED, false /* add mrpt header */, "% Grid limits: [x_min x_max y_min y_max]\n" );


	if ( m_mapType == mrKernelDM || m_mapType == mrKernelDMV )
	{
		CMatrix  all_means(m_size_y,m_size_x);
		CMatrix  all_vars(m_size_y,m_size_x);
		CMatrix  all_confs(m_size_y,m_size_x);

		for (size_t y=0;y<m_size_y;y++)
			for (size_t x=0;x<m_size_x;x++)
			{
				const TRandomFieldCell * cell =cellByIndex(x,y);
				all_means(y,x) = computeMeanCellValue_DM_DMV(cell);
				all_vars(y,x)  = computeVarCellValue_DM_DMV(cell);
				all_confs(y,x) = computeConfidenceCellValue_DM_DMV(cell);
			}

		all_means.saveToTextFile( filNamePrefix + std::string("_mean.txt"), MATRIX_FORMAT_FIXED );
		if (m_mapType == mrKernelDMV)
		{
			all_vars.saveToTextFile( filNamePrefix + std::string("_var.txt"), MATRIX_FORMAT_FIXED );
			all_confs.saveToTextFile( filNamePrefix + std::string("_confidence.txt"), MATRIX_FORMAT_FIXED );
		}
	}


	if ( m_mapType == mrKalmanFilter || m_mapType == mrKalmanApproximate )
	{
		recoverMeanAndCov();

		// Save the mean and std matrix:
		CMatrix	MEAN( m_size_y,m_size_x);
		CMatrix	STDs( m_size_y, m_size_x );

		for (size_t i=0;i<m_size_y;i++)
			for (size_t j=0;j<m_size_x;j++)
			{
				MEAN(i,j)=cellByIndex(j,i)->kf_mean;
				STDs(i,j)=cellByIndex(j,i)->kf_std;
			}

		MEAN.saveToTextFile( filNamePrefix + std::string("_mean.txt"), MATRIX_FORMAT_FIXED );
		STDs.saveToTextFile( filNamePrefix + std::string("_cells_std.txt"), MATRIX_FORMAT_FIXED );

		if ( m_mapType == mrKalmanApproximate )
			{
				m_stackedCov.saveToTextFile( filNamePrefix + std::string("_mean_compressed_cov.txt"), MATRIX_FORMAT_FIXED );
			}
		if (m_mapType == mrKalmanFilter)
		{
			// Save the covariance matrix:
			m_cov.saveToTextFile( filNamePrefix + std::string("_mean_cov.txt") );
		}

		// And also as bitmap:
		STDs.normalize();
		CImage	img_cov(STDs, true);
		img_cov.saveToFile(filNamePrefix + std::string("_cells_std.png"), true /* vertical flip */);

		// Save the 3D graphs:
		saveAsMatlab3DGraph( filNamePrefix + std::string("_3D.m") );
	}

	if (m_mapType == mrGMRF_G || m_mapType == mrGMRF_SD)
	{
		// Save the mean and std matrix:
		CMatrix	MEAN( m_size_y, m_size_x );
		CMatrix	STDs( m_size_y, m_size_x );

		for (size_t i=0; i<m_size_y; i++)
		{
			for (size_t j=0; j<m_size_x; j++)
			{
				MEAN(i,j) = cellByIndex(j,i)->gmrf_mean;
				STDs(i,j) = cellByIndex(j,i)->gmrf_std;
			}
		}

		MEAN.saveToTextFile( filNamePrefix + std::string("_mean.txt"), MATRIX_FORMAT_FIXED );
		STDs.saveToTextFile( filNamePrefix + std::string("_cells_std.txt"), MATRIX_FORMAT_FIXED );
	}

}

/*---------------------------------------------------------------
					saveAsMatlab3DGraph
  ---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::saveAsMatlab3DGraph(const std::string  &filName) const
{
	MRPT_START

	const double std_times = 3;

	ASSERT_( m_mapType == mrKalmanFilter || m_mapType==mrKalmanApproximate );

	recoverMeanAndCov();

	FILE	*f= os::fopen( filName.c_str(), "wt" );
	if (!f)
		THROW_EXCEPTION("Couldn't create output file!");

	os::fprintf(f,"%%-------------------------------------------------------\n");
	os::fprintf(f,"%% File automatically generated using the MRPT method:\n");
	os::fprintf(f,"%%'CRandomFieldGridMap2D::saveAsMatlab3DGraph'\n");
	os::fprintf(f,"%%\n");
	os::fprintf(f,"%%                        ~ MRPT ~\n");
	os::fprintf(f,"%%  Jose Luis Blanco Claraco, University of Malaga @ 2006-2007\n");
	os::fprintf(f,"%%  http://www.isa.uma.es/ \n");
	os::fprintf(f,"%%-------------------------------------------------------\n\n");


	unsigned int	cx,cy;
	vector<float>	xs,ys;

	// xs: array of X-axis values
	os::fprintf(f,"xs = [");
	xs.resize( m_size_x );
	for (cx=0;cx<m_size_x;cx++)
	{
		xs[cx] = m_x_min + m_resolution * cx;
		os::fprintf(f,"%f ", xs[cx]);
	}
	os::fprintf(f,"];\n");

	// ys: array of X-axis values
	os::fprintf(f,"ys = [");
	ys.resize( m_size_y );
	for (cy=0;cy<m_size_y;cy++)
	{
		ys[cy] = m_y_min + m_resolution * cy;
		os::fprintf(f,"%f ", ys[cy]);
	}
	os::fprintf(f,"];\n");

	// z_mean: matrix with mean concentration values
	os::fprintf(f,"z_mean = [\n");
	for (cy=0;cy<m_size_y;cy++)
	{
		for (cx=0;cx<m_size_x;cx++)
		{
            const TRandomFieldCell	*cell = cellByIndex( cx,cy );
			ASSERT_( cell!=NULL );
			os::fprintf(f,"%e ", cell->kf_mean  );
		} // for cx

		if (cy<(m_size_y-1))
			os::fprintf(f,"; ...\n");
	} // for cy
	os::fprintf(f,"];\n\n");

	// z_upper: matrix with upper confidence levels for concentration values
	os::fprintf(f,"z_upper = [\n");
	for (cy=0;cy<m_size_y;cy++)
	{
		for (cx=0;cx<m_size_x;cx++)
		{
            const TRandomFieldCell	*cell = cellByIndex( cx,cy );
			ASSERT_( cell!=NULL );
			//os::fprintf(f,"%e ",  min(1.0,max(0.0,  cell->kf_mean + std_times * cell->kf_std ) ));
			os::fprintf(f,"%e ",  max(0.0,  cell->kf_mean + std_times * cell->kf_std ));
		} // for cx

		if (cy<(m_size_y-1))
			os::fprintf(f,"; ...\n");
	} // for cy
	os::fprintf(f,"];\n\n");

	// z_lowe: matrix with lower confidence levels for concentration values
	os::fprintf(f,"z_lower = [\n");
	for (cy=0;cy<m_size_y;cy++)
	{
		for (cx=0;cx<m_size_x;cx++)
		{
			const TRandomFieldCell	*cell = cellByIndex( cx,cy );
			ASSERT_(cell!=NULL);

			os::fprintf(f,"%e ",  min(1.0,max(0.0,  cell->kf_mean - std_times * cell->kf_std ) ));
		} // for cx

		if (cy<(m_size_y-1))
			os::fprintf(f,"; ...\n");
	} // for cy
	os::fprintf(f,"];\n\n");

	// Plot them:
	os::fprintf(f,"hold off;\n");
	os::fprintf(f,"S1 = surf(xs,ys,z_mean);\n");
	os::fprintf(f,"hold on;\n");
	os::fprintf(f,"S2 = surf(xs,ys,z_upper);\n");
	os::fprintf(f,"S3 = surf(xs,ys,z_lower);\n");
	os::fprintf(f,"\n");
	os::fprintf(f,"set(S1,'FaceAlpha',0.9,'EdgeAlpha',0.9);\n");
	os::fprintf(f,"set(S2,'FaceAlpha',0.4,'EdgeAlpha',0.4);\n");
	os::fprintf(f,"set(S3,'FaceAlpha',0.2,'EdgeAlpha',0.2);\n");
	os::fprintf(f,"\n");
	os::fprintf(f,"set(gca,'PlotBoxAspectRatio',[%f %f %f]);\n",
		m_x_max-m_x_min,
		m_y_max-m_y_min,
		4.0 );
	os::fprintf(f,"view(-40,30)\n");
	os::fprintf(f,"axis([%f %f %f %f 0 1]);\n",
		m_x_min,
		m_x_max,
		m_y_min,
		m_y_max
		);


	fclose(f);

	MRPT_END

}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	if (m_disableSaveAs3DObject)
		return;

	//Returns only the mean map
	mrpt::opengl::CSetOfObjectsPtr other_obj = mrpt::opengl::CSetOfObjects::Create();
	getAs3DObject(outObj, other_obj);
}


/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&meanObj, mrpt::opengl::CSetOfObjectsPtr &varObj ) const
{
	if (m_disableSaveAs3DObject)
		return;

	recoverMeanAndCov();		//Only works for KF2 method

	opengl::CSetOfTriangles::TTriangle		triag;

	unsigned int	cx,cy;
	vector<float>	xs,ys;

	// xs: array of X-axis values
	xs.resize( m_size_x );
	for (cx=0;cx<m_size_x;cx++)	xs[cx] = m_x_min + m_resolution * cx;

	// ys: array of Y-axis values
	ys.resize( m_size_y );
	for (cy=0;cy<m_size_y;cy++)	ys[cy] = m_y_min + m_resolution * cy;

	// Draw the surfaces:
	switch(m_mapType)
	{
	case mrGMRF_L:
		THROW_EXCEPTION("Rendering not implemented for map type:'mrGMRF_L'")
		break;

	case mrKalmanFilter:
	case mrKalmanApproximate:
	case mrGMRF_G:
	case mrGMRF_SD:
		{
			// for Kalman models:
			// ----------------------------------
			opengl::CSetOfTrianglesPtr obj_m = opengl::CSetOfTriangles::Create();
			obj_m->enableTransparency(true);
			opengl::CSetOfTrianglesPtr obj_v = opengl::CSetOfTriangles::Create();
			obj_v->enableTransparency(true);

			//  Compute mean max/min values:
			// ---------------------------------------
			double 	maxVal_m=0, minVal_m=1, AMaxMin_m, maxVal_v=0, minVal_v=1, AMaxMin_v;
			double c,v;
			for (cy=1;cy<m_size_y;cy++)
			{
				for (cx=1;cx<m_size_x;cx++)
				{
					const TRandomFieldCell	*cell_xy = cellByIndex( cx,cy ); ASSERT_( cell_xy!=NULL );
					//mean
					c = cell_xy->kf_mean;
					minVal_m = min(minVal_m, c);
					maxVal_m = max(maxVal_m, c);
					//variance
					v = square(cell_xy->kf_std);
					minVal_v = min(minVal_v, v);
					maxVal_v = max(maxVal_v, v);
				}
			}

			AMaxMin_m = maxVal_m - minVal_m;
			if (AMaxMin_m==0) AMaxMin_m=1;
			AMaxMin_v = maxVal_v - minVal_v;
			if (AMaxMin_v==0) AMaxMin_v=1;

			// ---------------------------------------
			//  Compute Maps
			// ---------------------------------------
			triag.a[0]=triag.a[1]=triag.a[2]= 0.75f;	// alpha (transparency)
			for (cy=1;cy<m_size_y;cy++)
			{
				for (cx=1;cx<m_size_x;cx++)
				{
					// Cell values:
					const TRandomFieldCell	*cell_xy = cellByIndex( cx,cy ); ASSERT_( cell_xy!=NULL );
					const TRandomFieldCell	*cell_x_1y = cellByIndex( cx-1,cy ); ASSERT_( cell_x_1y!=NULL );
					const TRandomFieldCell	*cell_xy_1 = cellByIndex( cx,cy-1 ); ASSERT_( cell_xy_1!=NULL );
					const TRandomFieldCell	*cell_x_1y_1 = cellByIndex( cx-1,cy-1 ); ASSERT_( cell_x_1y_1!=NULL );

					// MEAN values
					//-----------------
					double c_xy			= min(1.0,max(0.0, cell_xy->kf_mean) );
					double c_x_1y		= min(1.0,max(0.0, cell_x_1y->kf_mean) );
					double c_xy_1		= min(1.0,max(0.0, cell_xy_1->kf_mean) );
					double c_x_1y_1		= min(1.0,max(0.0, cell_x_1y_1->kf_mean) );

					double col_xy		= c_xy;		//min(1.0,max(0.0, (c_xy-minVal_m)/AMaxMin_m ) );
					double col_x_1y		= c_x_1y;	//min(1.0,max(0.0, (c_x_1y-minVal_m)/AMaxMin_m ) );
					double col_xy_1		= c_xy_1;	//min(1.0,max(0.0, (c_xy_1-minVal_m)/AMaxMin_m ) );
					double col_x_1y_1	= c_x_1y_1; //min(1.0,max(0.0, (c_x_1y_1-minVal_m)/AMaxMin_m ) );

					// Triangle #1:
					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = c_xy;
					triag.x[1] = xs[cx];	triag.y[1] = ys[cy-1];	triag.z[1] = c_xy_1;
					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy-1];	triag.z[2] = c_x_1y_1;
					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
					jet2rgb( col_xy_1,triag.r[1],triag.g[1],triag.b[1] );
					jet2rgb( col_x_1y_1,triag.r[2],triag.g[2],triag.b[2] );
					obj_m->insertTriangle( triag );

					// Triangle #2:
					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = c_xy;
					triag.x[1] = xs[cx-1];	triag.y[1] = ys[cy-1];	triag.z[1] = c_x_1y_1;
					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy];	triag.z[2] = c_x_1y;
					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
					jet2rgb( col_x_1y_1,triag.r[1],triag.g[1],triag.b[1] );
					jet2rgb( col_x_1y,triag.r[2],triag.g[2],triag.b[2] );
					obj_m->insertTriangle( triag );

					// VARIANCE values
					//------------------
					double v_xy			= min(10.0,max(0.0, square(cell_xy->kf_std) ) );
					double v_x_1y		= min(10.0,max(0.0, square(cell_x_1y->kf_std) ) );
					double v_xy_1		= min(10.0,max(0.0, square(cell_xy_1->kf_std) ) );
					double v_x_1y_1		= min(10.0,max(0.0, square(cell_x_1y_1->kf_std) ) );

					col_xy				= v_xy/10;		//min(1.0,max(0.0, (v_xy-minVal_v)/AMaxMin_v ) );
					col_x_1y			= v_x_1y/10;	//min(1.0,max(0.0, (v_x_1y-minVal_v)/AMaxMin_v ) );
					col_xy_1			= v_xy_1/10;	//min(1.0,max(0.0, (v_xy_1-minVal_v)/AMaxMin_v ) );
					col_x_1y_1			= v_x_1y_1/10;	//min(1.0,max(0.0, (v_x_1y_1-minVal_v)/AMaxMin_v ) );


					// Triangle #1:
					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = v_xy;
					triag.x[1] = xs[cx];	triag.y[1] = ys[cy-1];	triag.z[1] = v_xy_1;
					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy-1];	triag.z[2] = v_x_1y_1;
					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
					jet2rgb( col_xy_1,triag.r[1],triag.g[1],triag.b[1] );
					jet2rgb( col_x_1y_1,triag.r[2],triag.g[2],triag.b[2] );
					obj_v->insertTriangle( triag );

					// Triangle #2:
					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = v_xy;
					triag.x[1] = xs[cx-1];	triag.y[1] = ys[cy-1];	triag.z[1] = v_x_1y_1;
					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy];	triag.z[2] = v_x_1y;
					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
					jet2rgb( col_x_1y_1,triag.r[1],triag.g[1],triag.b[1] );
					jet2rgb( col_x_1y,triag.r[2],triag.g[2],triag.b[2] );
					obj_v->insertTriangle( triag );

				} // for cx
			} // for cy
			meanObj->insert( obj_m );
			varObj->insert( obj_v );


//-----------------------------
//Code from JL starts here
//-----------------------------
	//		opengl::CSetOfTrianglesPtr obj = opengl::CSetOfTriangles::Create();
	//		const double  std_times = 2;
	//		const double  z_aspect_ratio = 4;

	//		//  Compute max/min values:
	//		// ---------------------------------------
	//		double	maxVal=0, minVal=1, AMaxMin;
	//		for (cy=1;cy<m_size_y;cy++)
	//		{
	//			for (cx=1;cx<m_size_x;cx++)
	//			{
	//				const TRandomFieldCell	*cell_xy = cellByIndex( cx,cy ); ASSERT_( cell_xy!=NULL );
	//				minVal = min(minVal, cell_xy->kf_mean);
	//				maxVal = max(maxVal, cell_xy->kf_mean);
	//			}
	//		}

	//		AMaxMin = maxVal - minVal;
	//		if (AMaxMin==0) AMaxMin=1;


	//		// ---------------------------------------
	//		//  BOTTOM LAYER:  mean - K*std
	//		// ---------------------------------------
	//		triag.a[0]=triag.a[1]=triag.a[2]= 0.8f;

	//		for (cy=1;cy<m_size_y;cy++)
	//		{
	//			for (cx=1;cx<m_size_x;cx++)
	//			{
	//				// Cell values:
	//				const TRandomFieldCell	*cell_xy = cellByIndex( cx,cy ); ASSERT_( cell_xy!=NULL );
	//				const TRandomFieldCell	*cell_x_1y = cellByIndex( cx-1,cy ); ASSERT_( cell_x_1y!=NULL );
	//				const TRandomFieldCell	*cell_xy_1 = cellByIndex( cx,cy-1 ); ASSERT_( cell_xy_1!=NULL );
	//				const TRandomFieldCell	*cell_x_1y_1 = cellByIndex( cx-1,cy-1 ); ASSERT_( cell_x_1y_1!=NULL );

	//				double c_xy	= min(1.0,max(0.0, cell_xy->kf_mean - std_times*cell_xy->kf_std ) );
	//				double c_x_1y	= min(1.0,max(0.0, cell_x_1y->kf_mean - std_times*cell_x_1y->kf_std ) );
	//				double c_xy_1	= min(1.0,max(0.0, cell_xy_1->kf_mean - std_times*cell_xy_1->kf_std ) );
	//				double c_x_1y_1= min(1.0,max(0.0, cell_x_1y_1->kf_mean - std_times*cell_x_1y_1->kf_std) );

	//				double col_xy		= min(1.0,max(0.0, (cell_xy->kf_mean-minVal)/AMaxMin ) );
	//				double col_x_1y	= min(1.0,max(0.0, (cell_x_1y->kf_mean-minVal)/AMaxMin ) );
	//				double col_xy_1	= min(1.0,max(0.0, (cell_xy_1->kf_mean-minVal)/AMaxMin ) );
	//				double col_x_1y_1	= min(1.0,max(0.0, (cell_x_1y_1->kf_mean-minVal)/AMaxMin ) );

	//				// Triangle #1:
	//				//if ( fabs(c_xy-0.5f)<0.49f ||
	//				//	 fabs(c_x_1y-0.5f)<0.49f ||
	//				//	 fabs(c_xy_1-0.5f)<0.49f ||
	//				//	 fabs(c_x_1y_1-0.5f)<0.49f )
	//				{
	//					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = c_xy;
	//					triag.x[1] = xs[cx];	triag.y[1] = ys[cy-1];	triag.z[1] = c_xy_1;
	//					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy-1];	triag.z[2] = c_x_1y_1;
	//					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
	//					jet2rgb( col_xy_1,triag.r[1],triag.g[1],triag.b[1] );
	//					jet2rgb( col_x_1y_1,triag.r[2],triag.g[2],triag.b[2] );

	//					obj->insertTriangle( triag );
	//				}

	//				// Triangle #2:
	//				//if ( fabs(c_xy-0.5f)<0.49f ||
	//				//	 fabs(c_x_1y-0.5f)<0.49f ||
	//				//	 fabs(c_xy_1-0.5f)<0.49f ||
	//				//	 fabs(c_x_1y_1-0.5f)<0.49f )
	//				{
	//					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = c_xy;
	//					triag.x[1] = xs[cx-1];	triag.y[1] = ys[cy-1];	triag.z[1] = c_x_1y_1;
	//					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy];	triag.z[2] = c_x_1y;

	//					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
	//					jet2rgb( col_x_1y_1,triag.r[1],triag.g[1],triag.b[1] );
	//					jet2rgb( col_x_1y,triag.r[2],triag.g[2],triag.b[2] );

	//					obj->insertTriangle( triag );
	//				}
	//			} // for cx
	//		} // for cy
	///**/
	//		// ---------------------------------------
	//		//  MID LAYER:  mean
	//		// ---------------------------------------
	///**/
	//		triag.a[0]=triag.a[1]=triag.a[2]= 0.8f;
	//		for (cy=1;cy<m_size_y;cy++)
	//		{
	//			for (cx=1;cx<m_size_x;cx++)
	//			{
	//				// Cell values:
	//				const TRandomFieldCell	*cell_xy = cellByIndex( cx,cy ); ASSERT_( cell_xy!=NULL );
	//				const TRandomFieldCell	*cell_x_1y = cellByIndex( cx-1,cy ); ASSERT_( cell_x_1y!=NULL );
	//				const TRandomFieldCell	*cell_xy_1 = cellByIndex( cx,cy-1 ); ASSERT_( cell_xy_1!=NULL );
	//				const TRandomFieldCell	*cell_x_1y_1 = cellByIndex( cx-1,cy-1 ); ASSERT_( cell_x_1y_1!=NULL );

	//				double c_xy	= min(1.0,max(0.0, cell_xy->kf_mean ) );
	//				double c_x_1y	= min(1.0,max(0.0, cell_x_1y->kf_mean ) );
	//				double c_xy_1	= min(1.0,max(0.0, cell_xy_1->kf_mean ) );
	//				double c_x_1y_1= min(1.0,max(0.0, cell_x_1y_1->kf_mean ) );

	//				double col_xy		= min(1.0,max(0.0, cell_xy->kf_mean ) );
	//				double col_xy_1	= min(1.0,max(0.0, cell_xy_1->kf_mean ) );
	//				double col_x_1y	= min(1.0,max(0.0, cell_x_1y->kf_mean ) );
	//				double col_x_1y_1	= min(1.0,max(0.0, cell_x_1y_1->kf_mean ) );

	//				// Triangle #1:
	//				triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = z_aspect_ratio*c_xy;
	//				triag.x[1] = xs[cx];	triag.y[1] = ys[cy-1];	triag.z[1] = z_aspect_ratio*c_xy_1;
	//				triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy-1];	triag.z[2] = z_aspect_ratio*c_x_1y_1;
	//				jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
	//				jet2rgb( col_xy_1,triag.r[1],triag.g[1],triag.b[1] );
	//				jet2rgb( col_x_1y_1,triag.r[2],triag.g[2],triag.b[2] );

	//				obj->insertTriangle( triag );

	//				// Triangle #2:
	//				triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = z_aspect_ratio*c_xy;
	//				triag.x[1] = xs[cx-1];	triag.y[1] = ys[cy-1];	triag.z[1] = z_aspect_ratio*c_x_1y_1;
	//				triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy];	triag.z[2] = z_aspect_ratio*c_x_1y;

	//				jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
	//				jet2rgb( col_x_1y_1,triag.r[1],triag.g[1],triag.b[1] );
	//				jet2rgb( col_x_1y,triag.r[2],triag.g[2],triag.b[2] );
	//				obj->insertTriangle( triag );

	//			} // for cx
	//		} // for cy
	///**/

	//		// ---------------------------------------
	//		//  TOP LAYER:  mean + K*std
	//		// ---------------------------------------
	//		triag.a[0]=triag.a[1]=triag.a[2]= 0.5f;

	//		for (cy=1;cy<m_size_y;cy++)
	//		{
	//			for (cx=1;cx<m_size_x;cx++)
	//			{
	//				// Cell values:
	//				const TRandomFieldCell	*cell_xy = cellByIndex( cx,cy ); ASSERT_( cell_xy!=NULL );
	//				const TRandomFieldCell	*cell_x_1y = cellByIndex( cx-1,cy ); ASSERT_( cell_x_1y!=NULL );
	//				const TRandomFieldCell	*cell_xy_1 = cellByIndex( cx,cy-1 ); ASSERT_( cell_xy_1!=NULL );
	//				const TRandomFieldCell	*cell_x_1y_1 = cellByIndex( cx-1,cy-1 ); ASSERT_( cell_x_1y_1!=NULL );

	//				double c_xy	= min(1.0,max(0.0, cell_xy->kf_mean + std_times*cell_xy->kf_std ) );
	//				double c_x_1y	= min(1.0,max(0.0, cell_x_1y->kf_mean + std_times*cell_x_1y->kf_std ) );
	//				double c_xy_1	= min(1.0,max(0.0, cell_xy_1->kf_mean + std_times*cell_xy_1->kf_std ) );
	//				double c_x_1y_1= min(1.0,max(0.0, cell_x_1y_1->kf_mean + std_times*cell_x_1y_1->kf_std) );

	//				double col_xy		= min(1.0,max(0.0, (cell_xy->kf_mean-minVal)/AMaxMin ) );
	//				double col_x_1y	= min(1.0,max(0.0, (cell_x_1y->kf_mean-minVal)/AMaxMin ) );
	//				double col_xy_1	= min(1.0,max(0.0, (cell_xy_1->kf_mean-minVal)/AMaxMin ) );
	//				double col_x_1y_1	= min(1.0,max(0.0, (cell_x_1y_1->kf_mean-minVal)/AMaxMin ) );

	//				// Triangle #1:
	//				/*if ( fabs(c_xy-0.5f)<0.49f ||
	//					 fabs(c_x_1y-0.5f)<0.49f ||
	//					 fabs(c_xy_1-0.5f)<0.49f ||
	//					 fabs(c_x_1y_1-0.5f)<0.49f )*/
	//				{
	//					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = z_aspect_ratio*c_xy;
	//					triag.x[1] = xs[cx];	triag.y[1] = ys[cy-1];	triag.z[1] = z_aspect_ratio*c_xy_1;
	//					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy-1];	triag.z[2] = z_aspect_ratio*c_x_1y_1;
	//					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
	//					jet2rgb( col_xy_1,triag.r[1],triag.g[1],triag.b[1] );
	//					jet2rgb( col_x_1y_1,triag.r[2],triag.g[2],triag.b[2] );

	//					obj->insertTriangle( triag );
	//				}

	//				// Triangle #2:
	//				/*if ( fabs(c_xy-0.5f)<0.49f ||
	//					 fabs(c_x_1y-0.5f)<0.49f ||
	//					 fabs(c_xy_1-0.5f)<0.49f ||
	//					 fabs(c_x_1y_1-0.5f)<0.49f )*/
	//				{
	//					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = z_aspect_ratio*c_xy;
	//					triag.x[1] = xs[cx-1];	triag.y[1] = ys[cy-1];	triag.z[1] = z_aspect_ratio*c_x_1y_1;
	//					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy];	triag.z[2] = z_aspect_ratio*c_x_1y;

	//					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
	//					jet2rgb( col_x_1y_1,triag.r[1],triag.g[1],triag.b[1] );
	//					jet2rgb( col_x_1y,triag.r[2],triag.g[2],triag.b[2] );

	//					obj->insertTriangle( triag );
	//				}
	//			} // for cx
	//		} // for cy


	//		obj->enableTransparency(true);;
	//		meanObj->insert( obj );
	//		varObj->insert( obj );

//-----------------------------
//Code from JL ends here
//-----------------------------

		}
		break; // end KF models

	case mrKernelDM:
	case mrKernelDMV:
		{
			// Draw for Kernel model:
			// ----------------------------------
			opengl::CSetOfTrianglesPtr obj_m = opengl::CSetOfTriangles::Create();
			obj_m->enableTransparency(true);
			opengl::CSetOfTrianglesPtr obj_v = opengl::CSetOfTriangles::Create();
			obj_v->enableTransparency(true);

			//  Compute mean max/min values:
			// ---------------------------------------
			double 	maxVal_m=0, minVal_m=1, AMaxMin_m, maxVal_v=0, minVal_v=1, AMaxMin_v;
			double c,v;
			for (cy=1;cy<m_size_y;cy++)
			{
				for (cx=1;cx<m_size_x;cx++)
				{
					const TRandomFieldCell	*cell_xy = cellByIndex( cx,cy ); ASSERT_( cell_xy!=NULL );
					//mean
					c = computeMeanCellValue_DM_DMV( cell_xy );
					minVal_m = min(minVal_m, c);
					maxVal_m = max(maxVal_m, c);
					//variance
					v = computeVarCellValue_DM_DMV( cell_xy );
					minVal_v = min(minVal_v, v);
					maxVal_v = max(maxVal_v, v);
				}
			}

			AMaxMin_m = maxVal_m - minVal_m;
			if (AMaxMin_m==0) AMaxMin_m=1;
			AMaxMin_v = maxVal_v - minVal_v;
			if (AMaxMin_v==0) AMaxMin_v=1;

			// ---------------------------------------
			//  Compute Maps
			// ---------------------------------------
			triag.a[0]=triag.a[1]=triag.a[2]= 0.75f;	// alpha (transparency)
			for (cy=1;cy<m_size_y;cy++)
			{
				for (cx=1;cx<m_size_x;cx++)
				{
					// Cell values:
					const TRandomFieldCell	*cell_xy = cellByIndex( cx,cy ); ASSERT_( cell_xy!=NULL );
					const TRandomFieldCell	*cell_x_1y = cellByIndex( cx-1,cy ); ASSERT_( cell_x_1y!=NULL );
					const TRandomFieldCell	*cell_xy_1 = cellByIndex( cx,cy-1 ); ASSERT_( cell_xy_1!=NULL );
					const TRandomFieldCell	*cell_x_1y_1 = cellByIndex( cx-1,cy-1 ); ASSERT_( cell_x_1y_1!=NULL );

					// MEAN values
					//-----------------
					double c_xy			= min(1.0,max(0.0, computeMeanCellValue_DM_DMV(cell_xy) ) );
					double c_x_1y		= min(1.0,max(0.0, computeMeanCellValue_DM_DMV(cell_x_1y) ) );
					double c_xy_1		= min(1.0,max(0.0, computeMeanCellValue_DM_DMV(cell_xy_1) ) );
					double c_x_1y_1		= min(1.0,max(0.0, computeMeanCellValue_DM_DMV(cell_x_1y_1) ) );

					double col_xy		= c_xy;		//min(1.0,max(0.0, (c_xy-minVal_m)/AMaxMin_m ) );
					double col_x_1y		= c_x_1y;	//min(1.0,max(0.0, (c_x_1y-minVal_m)/AMaxMin_m ) );
					double col_xy_1		= c_xy_1;	//min(1.0,max(0.0, (c_xy_1-minVal_m)/AMaxMin_m ) );
					double col_x_1y_1	= c_x_1y_1;	//min(1.0,max(0.0, (c_x_1y_1-minVal_m)/AMaxMin_m ) );

					// Triangle #1:
					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = c_xy;
					triag.x[1] = xs[cx];	triag.y[1] = ys[cy-1];	triag.z[1] = c_xy_1;
					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy-1];	triag.z[2] = c_x_1y_1;
					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
					jet2rgb( col_xy_1,triag.r[1],triag.g[1],triag.b[1] );
					jet2rgb( col_x_1y_1,triag.r[2],triag.g[2],triag.b[2] );

					obj_m->insertTriangle( triag );

					// Triangle #2:
					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = c_xy;
					triag.x[1] = xs[cx-1];	triag.y[1] = ys[cy-1];	triag.z[1] = c_x_1y_1;
					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy];	triag.z[2] = c_x_1y;
					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
					jet2rgb( col_x_1y_1,triag.r[1],triag.g[1],triag.b[1] );
					jet2rgb( col_x_1y,triag.r[2],triag.g[2],triag.b[2] );
					obj_m->insertTriangle( triag );

					// VARIANCE values
					//------------------
					double v_xy			= min(1.0,max(0.0, computeVarCellValue_DM_DMV(cell_xy) ) );
					double v_x_1y		= min(1.0,max(0.0, computeVarCellValue_DM_DMV(cell_x_1y) ) );
					double v_xy_1		= min(1.0,max(0.0, computeVarCellValue_DM_DMV(cell_xy_1) ) );
					double v_x_1y_1		= min(1.0,max(0.0, computeVarCellValue_DM_DMV(cell_x_1y_1) ) );

					col_xy				= v_xy;		//min(1.0,max(0.0, (v_xy-minVal_v)/AMaxMin_v ) );
					col_x_1y			= v_x_1y;	//min(1.0,max(0.0, (v_x_1y-minVal_v)/AMaxMin_v ) );
					col_xy_1			= v_xy_1;	//min(1.0,max(0.0, (v_xy_1-minVal_v)/AMaxMin_v ) );
					col_x_1y_1			= v_x_1y_1;	//min(1.0,max(0.0, (v_x_1y_1-minVal_v)/AMaxMin_v ) );


					// Triangle #1:
					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = v_xy;
					triag.x[1] = xs[cx];	triag.y[1] = ys[cy-1];	triag.z[1] = v_xy_1;
					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy-1];	triag.z[2] = v_x_1y_1;
					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
					jet2rgb( col_xy_1,triag.r[1],triag.g[1],triag.b[1] );
					jet2rgb( col_x_1y_1,triag.r[2],triag.g[2],triag.b[2] );

					obj_v->insertTriangle( triag );

					// Triangle #2:
					triag.x[0] = xs[cx];	triag.y[0] = ys[cy];	triag.z[0] = v_xy;
					triag.x[1] = xs[cx-1];	triag.y[1] = ys[cy-1];	triag.z[1] = v_x_1y_1;
					triag.x[2] = xs[cx-1];	triag.y[2] = ys[cy];	triag.z[2] = v_x_1y;
					jet2rgb( col_xy,triag.r[0],triag.g[0],triag.b[0] );
					jet2rgb( col_x_1y_1,triag.r[1],triag.g[1],triag.b[1] );
					jet2rgb( col_x_1y,triag.r[2],triag.g[2],triag.b[2] );
					obj_v->insertTriangle( triag );


				} // for cx
			} // for cy
			meanObj->insert( obj_m );
			varObj->insert( obj_v );
		}
		break; // end Kernel models
	}; // end switch maptype
}



/*---------------------------------------------------------------
					computeConfidenceCellValue_DM_DMV
 ---------------------------------------------------------------*/
double  CRandomFieldGridMap2D::computeConfidenceCellValue_DM_DMV (const TRandomFieldCell *cell ) const
{
	// A confidence measure:
	return 1.0 - std::exp(-square(cell->dm_mean_w/m_insertOptions_common->dm_sigma_omega));
}

/*---------------------------------------------------------------
					computeMeanCellValue_DM_DMV
 ---------------------------------------------------------------*/
double  CRandomFieldGridMap2D::computeMeanCellValue_DM_DMV (const TRandomFieldCell *cell ) const
{
	// A confidence measure:
	const double alpha = 1.0 - std::exp(-square(cell->dm_mean_w/m_insertOptions_common->dm_sigma_omega));
	const double r_val = (cell->dm_mean_w>0) ? (cell->dm_mean / cell->dm_mean_w) : 0;
	return alpha * r_val + (1-alpha) * m_average_normreadings_mean;
}

/*---------------------------------------------------------------
					computeVarCellValue_DM_DMV
 ---------------------------------------------------------------*/
double CRandomFieldGridMap2D::computeVarCellValue_DM_DMV (const TRandomFieldCell *cell ) const
{
	// A confidence measure:
	const double alpha = 1.0 - std::exp(-square(cell->dm_mean_w/m_insertOptions_common->dm_sigma_omega));
	const double r_val = (cell->dm_mean_w>0) ? (cell->dmv_var_mean / cell->dm_mean_w) : 0;
	return alpha * r_val + (1-alpha) * m_average_normreadings_var;
}

/*---------------------------------------------------------------
					predictMeasurement
  ---------------------------------------------------------------*/
void CRandomFieldGridMap2D::predictMeasurement(
	const double	&x,
	const double	&y,
	double			&out_predict_response,
	double			&out_predict_response_variance )
{
	MRPT_START

	switch (m_mapType)
	{
	case mrKernelDM:
		{
			TRandomFieldCell	*cell = cellByPos( x, y );
			if (!cell)
			{
				out_predict_response = m_average_normreadings_mean;
				out_predict_response_variance = square( m_insertOptions_common->KF_initialCellStd );
			}
			else
			{
				out_predict_response = computeMeanCellValue_DM_DMV(cell);
				out_predict_response_variance = square( m_insertOptions_common->KF_initialCellStd );
			}
		}
		break;
	case mrKernelDMV:
		{
			TRandomFieldCell	*cell = cellByPos( x, y );
			if (!cell)
			{
				out_predict_response = m_average_normreadings_mean;
				out_predict_response_variance = square( m_insertOptions_common->KF_initialCellStd );
			}
			else
			{
				out_predict_response = computeMeanCellValue_DM_DMV(cell);
				out_predict_response_variance = computeVarCellValue_DM_DMV(cell);
			}
		}
		break;

	case mrKalmanFilter:
	case mrKalmanApproximate:
		{
			if (m_hasToRecoverMeanAndCov)	recoverMeanAndCov();	// Just for KF2

			TRandomFieldCell	*cell = cellByPos( x, y );
			if (!cell)
			{
				out_predict_response = m_insertOptions_common->KF_defaultCellMeanValue;
				out_predict_response_variance = square( m_insertOptions_common->KF_initialCellStd ) + square(m_insertOptions_common->KF_observationModelNoise);
			}
			else
			{
				out_predict_response = cell->kf_mean;
				out_predict_response_variance = square( cell->kf_std ) +  square(m_insertOptions_common->KF_observationModelNoise);
			}
		}
		break;

	default:
		THROW_EXCEPTION("Invalid map type.");
	};

	// Un-do the sensor normalization:
	out_predict_response = m_insertOptions_common->R_min + out_predict_response * ( m_insertOptions_common->R_max - m_insertOptions_common->R_min );

	MRPT_END
}


/*---------------------------------------------------------------
					insertObservation_KF2
  ---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::insertObservation_KF2(
	float			normReading,
	const mrpt::math::TPoint2D &point )
{
	MRPT_START

	cout << "Inserting KF2: (" << normReading << ") at Postion" << point << endl;

	const signed	W = m_insertOptions_common->KF_W_size;
	const size_t	K = 2*W*(W+1)+1;
	const size_t	W21	= 2*W+1;
	const size_t	W21sqr	= W21*W21;

	ASSERT_(W>=2);

	m_hasToRecoverMeanAndCov = true;

	const TRandomFieldCell defCell(
				m_insertOptions_common->KF_defaultCellMeanValue,	// mean
				-1	// Just to indicate that cells are new, next changed to: m_insertOptions_common->KF_initialCellStd			// std
				);

	// Assure we have room enough in the grid!
	const double Aspace = (W+1) * m_resolution;

	resize(	point.x - Aspace,
			point.x + Aspace,
			point.y - Aspace,
			point.y + Aspace,
			defCell,
			Aspace );

	// --------------------------------------------------------
	// The Kalman-Filter estimation of the gas grid-map:
	// --------------------------------------------------------
	const size_t	N = m_map.size();

	ASSERT_(K==m_stackedCov.getColCount());
	ASSERT_(N==m_stackedCov.getRowCount());

	// Prediction stage of KF:
	// ------------------------------------
	// Nothing to do here (static map)

	// Update stage of KF:
	//  We directly apply optimized formulas arising
	//   from our concrete sensor model.
	// -------------------------------------------------

	//const double	KF_covSigma2 = square(m_insertOptions_common->KF_covSigma);
	//const double	std0 = m_insertOptions_common->KF_initialCellStd;
	//const double	res2 = square(m_resolution);
	const int				cellIdx = xy2idx( point.x, point.y );
	TRandomFieldCell	*cell = cellByPos( point.x, point.y );
	ASSERT_(cell!=NULL);

	// Predicted observation mean
	double		yk = normReading - cell->kf_mean;

	// Predicted observation variance
	double		sk =
		m_stackedCov(cellIdx,0) +	// Variance of that cell: cov(i,i)
		square(m_insertOptions_common->KF_observationModelNoise);

	double		sk_1 = 1.0 / sk;

	std::vector<TRandomFieldCell>::iterator	it;

#if RANDOMFIELDGRIDMAP_KF2_VERBOSE
	CTicTac		tictac;
	printf("[insertObservation_KF2] Updating mean values...");
	tictac.Tic();
#endif

	// ------------------------------------------------------------
	// Update mean values:
	//  Process only those cells in the vecinity of the cell (c):
	//
	//   What follows is *** REALLY UGLY *** for efficiency, sorry!!  :-)
	// ------------------------------------------------------------
	const int	cx_c = x2idx( point.x );
	const int	cy_c = y2idx( point.y );

	const int	Acx0 = max(-W, -cx_c);
	const int	Acy0 = max(-W, -cy_c);
	const int	Acx1 = min( W, int(m_size_x)-1-cx_c);
	const int	Acy1 = min( W, int(m_size_y)-1-cy_c);

	// We will fill this now, so we already have it for updating the
	//  covariances next:
	CVectorDouble	cross_covs_c_i( W21sqr, 0); // Indexes are relative to the (2W+1)x(2W+1) window centered at "cellIdx".
	vector_int		window_idxs   ( W21sqr, -1 ); // The real-map indexes for each element in the window, or -1 if there are out of the map (for cells close to the border)

	// 1) First, the cells before "c":
	for (int Acy=Acy0;Acy<=0;Acy++)
	{
		int		limit_cx = Acy<0 ? Acx1 : -1;

		size_t  idx = cx_c+Acx0 + m_size_x * ( cy_c+Acy );
		int		idx_c_in_idx = -Acy*W21 - Acx0;

		int		window_idx = Acx0+W + (Acy+W)*W21;

		for (int Acx=Acx0;Acx<=limit_cx;Acx++)
		{
			ASSERT_(idx_c_in_idx>0);
			const double cov_i_c = m_stackedCov(idx,idx_c_in_idx);
			//JGmonroy - review m_stackedCov

			m_map[idx].kf_mean += yk * sk_1 * cov_i_c;

			// Save window indexes:
			cross_covs_c_i[window_idx] = cov_i_c;
			window_idxs[window_idx++]  = idx;

			idx_c_in_idx--;
			idx++;
		}
	}



	// 2) The cell "c" itself, and the rest within the window:
	for (int Acy=0;Acy<=Acy1;Acy++)
	{
		int start_cx = Acy>0 ? Acx0 : 0;

		size_t  idx = cx_c+start_cx + m_size_x * ( cy_c+Acy );
		int		idx_i_in_c;
		if (Acy>0)
				idx_i_in_c= (W+1) + (Acy-1)*W21+ (start_cx + W);	// Rest of rows
		else	idx_i_in_c= 0;	// First row.

		int		window_idx = start_cx+W + (Acy+W)*W21;

		for (int Acx=start_cx;Acx<=Acx1;Acx++)
		{
			ASSERT_(idx_i_in_c>=0 && idx_i_in_c<int(K));

			double cov_i_c = m_stackedCov(cellIdx,idx_i_in_c);
			m_map[idx].kf_mean += yk * sk_1 * cov_i_c;

			// Save window indexes:
			cross_covs_c_i[window_idx] = cov_i_c;
			window_idxs[window_idx++]  = idx;

			idx_i_in_c++;
			idx++;
		}
	}


	// ------------------------------------------------------------
	// Update cross-covariances values:
	//  Process only those cells in the vecinity of the cell (c)
	// ------------------------------------------------------------

	// First, we need the cross-covariances between each cell (i) and
	//  (c) in the window around (c). We have this in "cross_covs_c_i[k]"
	//  for k=[0,(2K+1)^2-1], and the indexes in "window_idxs[k]".
	for (size_t i=0;i<W21sqr;i++)
	{
		const int idx_i = window_idxs[i];
		if (idx_i<0) continue;	// out of the map

		// Extract the x,y indexes:
		int cx_i, cy_i;
		idx2cxcy(idx_i,cx_i,cy_i);

		const double cov_c_i = cross_covs_c_i[i];

		for (size_t j=i;j<W21sqr;j++)
		{
			const int idx_j = window_idxs[j];
			if (idx_j<0) continue;	// out of the map

			int cx_j, cy_j;
			idx2cxcy(idx_j,cx_j,cy_j);

			// The cells (i,j) may be too far from each other:
			const int Ax = cx_j-cx_i;
			if (Ax>W) continue;  // Next cell (may be more rows after the current one...)

			const int Ay = cy_j-cy_i;
			if (Ay>W) break; // We are absolutely sure out of i's window.

			const double cov_c_j = cross_covs_c_i[j];

			int idx_j_in_i;
			if (Ay>0)
					idx_j_in_i = Ax+W + (Ay-1)*W21 + W+1;
			else	idx_j_in_i = Ax; // First row:

			// Kalman update of the cross-covariances:
			double &cov_to_change = m_stackedCov(idx_i,idx_j_in_i);
			double Delta_cov = cov_c_j * cov_c_i * sk_1;
			if (i==j && cov_to_change<Delta_cov)
				THROW_EXCEPTION_CUSTOM_MSG1("Negative variance value appeared! Please increase the size of the window (W).\n(m_insertOptions_common->KF_covSigma=%f)",m_insertOptions_common->KF_covSigma);

			cov_to_change -= Delta_cov;

		} // end for j
	} // end for i


#if RANDOMFIELDGRIDMAP_KF2_VERBOSE
	printf("Done in %.03fms\n",	tictac.Tac()*1000 );
#endif

	// Update covariance matrix values:
	// ---------------------------------------------------------


#if RANDOMFIELDGRIDMAP_KF2_VERBOSE
	printf("[insertObservation_KF2] Updating covariance matrix...");
	tictac.Tic();
#endif



#if RANDOMFIELDGRIDMAP_KF2_VERBOSE
	printf("Done! %.03fms\n",	tictac.Tac()*1000 );
#endif


	MRPT_END
}
/*---------------------------------------------------------------
					recoverMeanAndCov
  ---------------------------------------------------------------*/
void  CRandomFieldGridMap2D::recoverMeanAndCov() const
{
	if (!m_hasToRecoverMeanAndCov || (m_mapType!=mrKalmanApproximate ) ) return;
	m_hasToRecoverMeanAndCov = false;

	// Just recover the std of each cell:
	const size_t	N = m_map.size();
	for (size_t i=0;i<N;i++)
		m_map_castaway_const()[i].kf_std = sqrt( m_stackedCov(i,0) );
}


/*---------------------------------------------------------------
					getMeanAndCov
  ---------------------------------------------------------------*/
void CRandomFieldGridMap2D::getMeanAndCov( CVectorDouble &out_means, CMatrixDouble &out_cov) const
{
	const size_t N = BASE::m_map.size();
	out_means.resize(N);
	for (size_t i=0;i<N;++i)
		out_means[i] = BASE::m_map[i].kf_mean;

	recoverMeanAndCov();
	out_cov = m_cov;
}



/*---------------------------------------------------------------
					getMeanAndSTD
  ---------------------------------------------------------------*/
void CRandomFieldGridMap2D::getMeanAndSTD( CVectorDouble &out_means, CVectorDouble &out_STD) const
{
	const size_t N = BASE::m_map.size();
	out_means.resize(N);
	out_STD.resize(N);

	for (size_t i=0;i<N;++i)
	{
		out_means[i] = BASE::m_map[i].kf_mean;
		out_STD[i] = sqrt(m_stackedCov(i,0));
	}
}


/*---------------------------------------------------------------
					setMeanAndSTD
  ---------------------------------------------------------------*/
void CRandomFieldGridMap2D::setMeanAndSTD( CVectorDouble &in_means, CVectorDouble &in_std)
{
	//Assure dimmensions match
	const size_t N = BASE::m_map.size();
	ASSERT_( N == size_t(in_means.size()) );
	ASSERT_( N == size_t(in_std.size()) );

	m_hasToRecoverMeanAndCov = true;
	for (size_t i=0; i<N; ++i)
	{
		m_map[i].kf_mean = in_means[i];	//update mean values
		m_stackedCov(i,0) = square(in_std[i]);	//update variance values
	}
	recoverMeanAndCov();	//update STD values from cov matrix
}


CRandomFieldGridMap2D::TMapRepresentation	 CRandomFieldGridMap2D::getMapType()
{
	return m_mapType;
}

void CRandomFieldGridMap2D::insertIndividualReading(const float sensorReading,const mrpt::math::TPoint2D & point)
{
	switch (m_mapType)
	{
		case mrKernelDM:           insertObservation_KernelDM_DMV(sensorReading,point, false); break;
		case mrKernelDMV:          insertObservation_KernelDM_DMV(sensorReading,point, true); break;
		case mrKalmanFilter:       insertObservation_KF(sensorReading,point); break;
		case mrKalmanApproximate:  insertObservation_KF2(sensorReading,point);break;
		case mrGMRF_G:			   insertObservation_GMRF(sensorReading,point); break;
		case mrGMRF_SD:			   insertObservation_GMRF(sensorReading,point); break;
	default:
		THROW_EXCEPTION("insertObservation() isn't implemented for selected 'mapType'")
	};
}


/*---------------------------------------------------------------
					insertObservation_GMRF
  ---------------------------------------------------------------*/
void CRandomFieldGridMap2D::insertObservation_GMRF(
	float normReading,
	const mrpt::math::TPoint2D &point )
{

	try{
		//Get index of observed cell
		const int			cellIdx = xy2idx( point.x, point.y );
		TRandomFieldCell	*cell = cellByPos( point.x, point.y );
		ASSERT_(cell!=NULL);

		// Insert observation in the vector of Active Observations
		TobservationGMRF new_obs;
		new_obs.obsValue = normReading;
		new_obs.Lambda = m_insertOptions_common->GMRF_lambdaObs;
		new_obs.time_invariant = false;		//Default behaviour, the obs will lose weight with time.
		activeObs[cellIdx].push_back(new_obs);


	}catch(std::exception e){
		cout << "Exception while Inserting new Observation: "  << e.what() << endl;
	}

	//Solve system and update map estimation
	updateMapEstimation_GMRF();
}


/*---------------------------------------------------------------
					updateMapEstimation_GMRF
  ---------------------------------------------------------------*/
void CRandomFieldGridMap2D::updateMapEstimation_GMRF()
{
#if EIGEN_VERSION_AT_LEAST(3,1,0)
	size_t N = m_map.size();
	const uint16_t Gsize = m_insertOptions_common->GMRF_constraintsSize;
	const uint16_t Gside = round((Gsize-1)/2);

#define DO_PROFILE

#ifdef DO_PROFILE
	static mrpt::utils::CTimeLogger timelogger;
	timelogger.enter("GMRF.build_hessian");
#endif

	//------------------
	//  1- HESSIAN
	//------------------
	//Build Sparse Hessian H, from list of triplets (Hprior):
	ASSERT_(!H_prior.empty())
	std::vector<Eigen::Triplet<double> > H_tri(H_prior.size());
	H_tri.reserve( H_prior.size()+N );
	std::copy( H_prior.begin(), H_prior.end(), H_tri.begin() );

	//Add H_obs
	for (size_t j=0; j<N; j++)
	{
		//Sum the information of all observations on cell j
		std::vector<TobservationGMRF>::iterator ito;
		float Lambda_obs_j = 0.0;
		for (ito = activeObs[j].begin(); ito !=activeObs[j].end(); ++ito)
			Lambda_obs_j += ito->Lambda;

		if (Lambda_obs_j != 0.0)
			H_tri.push_back( Eigen::Triplet<double>(j,j, Lambda_obs_j ) ); // Duplicated entries (from obs + from the prior) will be summed in setFromTriplets()
	}

	Eigen::SparseMatrix<double> Hsparse(N,N);				// declares a column-major sparse matrix type of float
	Hsparse.setFromTriplets(H_tri.begin(), H_tri.end() );

#if 0 // For debug only
	mrpt::math::saveEigenSparseTripletsToFile( "H_tri.txt", H_tri);
#endif


#ifdef DO_PROFILE
	timelogger.leave("GMRF.build_hessian");
	timelogger.enter("GMRF.build_grad");
#endif

	//------------------
	//  2- GRADIENT
	//------------------
	//Reset and Built Gradient Vector
	g.setZero();
	size_t cx = 0;
	size_t cy = 0;
	for (size_t j=0; j<N; j++)
	{
		//A- Gradient due to Observations
		std::vector<TobservationGMRF>::iterator ito;
		for (ito = activeObs[j].begin(); ito !=activeObs[j].end(); ++ito)
			g[j] += ((m_map[j].gmrf_mean - ito->obsValue) * ito->Lambda);


		//B- Gradient due to Prior
		switch (m_mapType)
		{
		case mrGMRF_G:
			{
				//Determine num of columns out of the gridmap
				int outc_left = max( 0 , int(Gside-cx) );
				int outc_right = max(int (0) , int (Gside-(m_size_x-cx-1)) );

				//Determine num of rows out of the gridmap
				int outr_down = max( 0 , int(Gside-(cy)) );
				int outr_up = max(int (0) , int (Gside-(m_size_y-cy-1)) );

				//Gradients between cell j and all neighbord cells i that have constraints
				for (int kr=-(Gside-outr_down); kr<=(Gside-outr_up); kr++ )
				{
					for (int kc=-(Gside-outc_left); kc<=(Gside-outc_right); kc++)
					{
						// get index of cell i
						size_t icx = cx + kc;
						size_t icy = cy + kr;
						size_t i = icx + icy*m_size_x;

						if (j!=i)
						{
							if (kr==0 || kc==0)      //only vertical and horizontal restrictions/constraints
							{
								//Resvisar!!
								g[j] += ( m_map[j].gmrf_mean - (m_map[i].gmrf_mean * gauss_val[abs(kr+kc)-1])) * m_insertOptions_common->GMRF_lambdaPrior;
							}
						}
					}
				}
			}
			break;

		case mrGMRF_SD:
			{
				if (this->m_insertOptions_common->GMRF_use_occupancy_information)
				{
					//Consider only cells correlated with the cell j
					std::pair < std::multimap<size_t,size_t>::iterator, std::multimap<size_t,size_t>::iterator > range;
					range = cell_interconnections.equal_range(j);
					while ( range.first!=range.second )
					{
						size_t cell_i_indx = range.first->second;
						g[j] += ( m_map[j].gmrf_mean - m_map[cell_i_indx].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;
						range.first++;
					}

				}
				else
				{
					//Gradient with all 4 neighbours
					if (cx != 0)	//factor with lef node
						g[j] += ( m_map[j].gmrf_mean - m_map[j-1].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;

					if (cx != (m_size_x-1))	//factor with right node
						g[j] += ( m_map[j].gmrf_mean - m_map[j+1].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;

					if (cy != 0)	//factor with benith node
						g[j] += ( m_map[j].gmrf_mean - m_map[j-m_size_x].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;

					if (cy != (m_size_y-1))	//factor with upper node
						g[j] += ( m_map[j].gmrf_mean - m_map[j+m_size_x].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;
				}
			}
			break;

		default:
			{
				g[j] = 0.0;
				cout << "Gradient estimation ERROR: Method not found" << endl;
			}
		};

		// Increment j coordinates (row(x), col(y))
		if (++cx>=m_size_x)
		{
			cx=0;
			cy++;
		}
	}//end-for

#ifdef DO_PROFILE
	timelogger.leave("GMRF.build_grad");
	timelogger.enter("GMRF.solve");
#endif

	//Cholesky Factorization of Hessian --> Realmente se hace: chol( P * H * inv(P) )
	Eigen::SimplicialLLT< Eigen::SparseMatrix <double> > solver;
	solver.compute(Hsparse);
	// Solve System:    m = m + H\(-g);
	// Note: we solve for (+g) to avoid creating a temporary "-g", then we'll substract the result in m_inc instead of adding it:
	Eigen::VectorXd m_inc = solver.solve(g);

#ifdef DO_PROFILE
	timelogger.leave("GMRF.solve");
	timelogger.enter("GMRF.variance");
#endif

	// VARIANCE SIGMA = inv(P) * inv( P*H*inv(P) ) * P
	//Get triangular supperior P*H*inv(P) = UT' * UT = P * R'*R * inv(P)

	Eigen::SparseMatrix<double> UT = solver.matrixU();

	Eigen::SparseMatrix<double> Sigma(N,N);								//Variance Matrix
	Sigma.reserve(UT.nonZeros());

	// TODO: UT.coeff() implies a heavy time cost in sparse matrices... the following
	//       should be rewritten for efficiency exploiting the knowledge about the nonzero pattern.

	//Apply custom equations to obtain the inverse -> inv( P*H*inv(P) )
	for (int l=N-1; l>=0; l--)
	{
		//Computes variances in the inferior submatrix of "l"
		double subSigmas = 0.0;
		for(size_t j=l+1; j<N; j++)
		{
			if (UT.coeff(l,j) != 0)
			{
				//Compute off-diagonal variances Sigma(j,l) = Sigma(l,j);

				//SUM 1
				double sum = 0.0;
				for(size_t i=l+1; i<=j; i++)
				{
					if( UT.coeff(l,i) !=0 )
					{
						sum += UT.coeff(l,i) * Sigma.coeff(i,j);
					}
				}
				//SUM 2
				for(size_t i=j+1; i<N; ++i)
				{
					if( UT.coeff(l,i) !=0 )
					{
						sum += UT.coeff(l,i) * Sigma.coeff(j,i);
					}
				}
				//Save off-diagonal variance (only Upper triangular)
				Sigma.insert(l,j) = ( -sum/UT.coeff(l,l) );
				subSigmas += UT.coeff(l,j) * Sigma.coeff(l,j);
			}
		}

		Sigma.insert(l,l) = (1/UT.coeff(l,l)) * ( 1/UT.coeff(l,l) - subSigmas );
	}

#ifdef DO_PROFILE
	timelogger.leave("GMRF.variance");
	timelogger.enter("GMRF.copy_to_map");
#endif

	// Update Mean-Variance and force (0 < m_map[i].mean < 1)
	for (size_t j=0; j<N; j++)
	{
		// Recover the diagonal covariance values, undoing the permutation:
		int idx = solver.permutationP().indices().coeff(j);
		const double variance = Sigma.coeff(idx,idx);

		m_map[j].gmrf_std = std::sqrt(variance);
		m_map[j].gmrf_mean -= m_inc[j]; // "-" because we solved for "+grad" instead of "-grad".
		if (m_map[j].gmrf_mean >1) m_map[j].gmrf_mean = 1.0;
		if (m_map[j].gmrf_mean <0) m_map[j].gmrf_mean = 0.0;
	}

	// Update Information/Strength of Active Observations
	//---------------------------------------------------------
	for (size_t j=0; j<activeObs.size(); j++)
	{
		std::vector<TobservationGMRF>::iterator ito = activeObs[j].begin();
		while ( ito!=activeObs[j].end() )
		{
			if (ito->time_invariant==false)
			{
				ito->Lambda -= m_insertOptions_common->GMRF_lambdaObsLoss;
				if (ito->Lambda <= 0.0)
					ito = activeObs[j].erase(ito);
				else
					ito++;
			}else
				ito++;
		}
	}

#ifdef DO_PROFILE
	timelogger.leave("GMRF.copy_to_map");
#endif

	/* //Save Hessian matrix to text file
	Eigen::MatrixXd H = Hsparse.toDense();
	H.saveToTextFile("_Hessian_Matrix_GMRF.txt");
	UT.toDense().saveToTextFile("_UT_GMRF.txt");
	solver.permutationP().toDenseMatrix().saveToTextFile("_P_GMRF.txt");
	solver.permutationPinv().toDenseMatrix().saveToTextFile("_Pinv_GMRF.txt");

	//Save Variance matrix to text file
	Sigma.toDense().saveToTextFile("_Var_GMRF.txt");*/


	//Sigma.toDense().saveToTextFile("_Sigma_GMRF.txt");

	//Hsparse = solver.permutationP() * Hsparse * solver.permutationPinv();
	//Hsparse.toDense().saveToTextFile("_PHPinv_GMRF.txt");

	//Save gradient vector
	//g.saveToTextFile("_gradient_vector_GMRF.txt");
	//
	////Save map
	//Eigen::VectorXd mymap;
	//mymap.resize(N);
	//m_inc.saveToTextFile("_m_inc_GMRF.txt");
	//for (size_t i=0; i<N; i++)
	//	mymap[i] = m_map[i].gmrf_mean;
	//mymap.saveToTextFile("_meanMap_GMRF.txt");
#else
	THROW_EXCEPTION("This method requires Eigen 3.1.0 or above")
#endif
}



bool CRandomFieldGridMap2D::exist_relation_between2cells(
	const mrpt::slam::COccupancyGridMap2D *m_Ocgridmap,
	size_t cxo_min,
	size_t cxo_max,
	size_t cyo_min,
	size_t cyo_max,
	const size_t seed_cxo,
	const size_t seed_cyo,
	const size_t objective_cxo,
	const size_t objective_cyo)
{
	//printf("Checking relation between cells (%i,%i) and (%i,%i)", seed_cxo,seed_cyo,objective_cxo,objective_cyo);

	//Ensure delimited region is within the Occupancy map
	cxo_min = max (cxo_min, (size_t)0);
	cxo_max = min (cxo_max, (size_t)m_Ocgridmap->getSizeX()-1);
	cyo_min = max (cyo_min, (size_t)0);
	cyo_max = min (cyo_max, (size_t)m_Ocgridmap->getSizeY()-1);

	//printf("Under gridlimits cx=(%i,%i) and cy=(%i,%i) \n", cxo_min,cxo_max,cyo_min,cyo_max);

	//Check that seed and objective are inside the delimited Occupancy gridmap
	if( (seed_cxo < cxo_min) || (seed_cxo >= cxo_max) || (seed_cyo < cyo_min) || (seed_cyo >= cyo_max) )
	{
		//cout << "Seed out of bounds (false)" << endl;
		return false;
	}
	if( (objective_cxo < cxo_min) || (objective_cxo >= cxo_max) || (objective_cyo < cyo_min) || (objective_cyo >= cyo_max) )
	{
		//cout << "Objective out of bounds (false)" << endl;
		return false;
	}

	// Check that seed and obj have similar occupancy (0,1)
	if ( (m_Ocgridmap->getCell(seed_cxo,seed_cyo)<0.5) != (m_Ocgridmap->getCell(objective_cxo,objective_cyo)<0.5) )
	{
		//cout << "Seed and objective have diff occupation (false)" << endl;
		return false;
	}


	//Create Matrix for region growing (row,col)
	mrpt::math::CMatrixUInt matExp(cxo_max-cxo_min+1, cyo_max-cyo_min+1);
	//cout << "Matrix creted with dimension:" << matExp.getRowCount() << " x " << matExp.getColCount() << endl;
	//CMatrix matExp(cxo_max-cxo_min+1, cyo_max-cyo_min+1);
	matExp.fill(0);

	//Add seed
	matExp(seed_cxo-cxo_min,seed_cyo-cyo_min) = 1;
	int seedsOld = 0;
	int seedsNew = 1;

	//NOT VERY EFFICIENT!!
	while (seedsOld < seedsNew)
	{
		seedsOld = seedsNew;

		for (size_t col=0; col<matExp.getColCount(); col++)
		{
			for (size_t row=0; row<matExp.getRowCount(); row++)
			{
				//test if cell needs to be expanded
				if( matExp(row,col) == 1)
				{
					matExp(row,col) = 2;	//mark as expanded
					//check if neighbourds have similar occupancy (expand)
					for (int i=-1;i<=1;i++)
					{
						for (int j=-1;j<=1;j++)
						{
							//check that neighbour is inside the map
							if( (int(row)+j>=0) && (int(row)+j<=int(matExp.getRowCount()-1)) && (int(col)+i>=0) && (int(col)+i<=int(matExp.getColCount())-1) )
							{
								if( !( (i==0 && j==0) || !(matExp(row+j,col+i)==0) ))
								{
									//check if expand
									if ( (m_Ocgridmap->getCell(row+cxo_min,col+cyo_min)<0.5) == (m_Ocgridmap->getCell(row+j+cxo_min,col+i+cyo_min)<0.5))
									{
										if ( (row+j+cxo_min == objective_cxo) && (col+i+cyo_min == objective_cyo) )
										{
											//cout << "Connection Success (true)" << endl;
											return true;		//Objective connected
										}
										matExp(row+j,col+i) = 1;
										seedsNew++;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	//if not connection to he objective is found, then return false
	//cout << "Connection not found (false)" << endl;
	return false;
}

float  CRandomFieldGridMap2D::compute3DMatchingRatio(
	const CMetricMap						*otherMap,
	const CPose3D							&otherMapPose,
	float									maxDistForCorr,
	float									maxMahaDistForCorr
	) const
{
	MRPT_UNUSED_PARAM(otherMap); MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(maxDistForCorr); MRPT_UNUSED_PARAM(maxMahaDistForCorr);
	return 0;
}


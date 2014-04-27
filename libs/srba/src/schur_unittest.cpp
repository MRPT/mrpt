/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/srba.h>
#include <mrpt/random.h>

#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::srba;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

typedef RbaEngine<
	kf2kf_poses::SE3,                // Parameterization  KF-to-KF poses
	landmarks::Euclidean3D,          // Parameterization of landmark positions
	observations::MonocularCamera    // Type of observations
	>
	my_rba_t;


struct TGraphInitRandom
{
	TGraphInitRandom(
		const uint32_t random_seed_,
		const size_t nUnknowns_k2k_,
		const size_t nUnknowns_k2f_,
		const double PROB_OBS_)
	:	random_seed(random_seed_),
		nUnknowns_k2k(nUnknowns_k2k_),
		nUnknowns_k2f(nUnknowns_k2f_),
		PROB_OBS(PROB_OBS_)
	{}

	const uint32_t random_seed;
	const size_t nUnknowns_k2k;
	const size_t nUnknowns_k2f;
	const double PROB_OBS; // Probability of observing a landmark (=1: all LMs visible from all KFs)
};

struct TGraphInitManual
{
	TGraphInitManual(
		const size_t nUnknowns_k2k_,
		const size_t nUnknowns_k2f_,
		const bool *visible_)
	:
		nUnknowns_k2k(nUnknowns_k2k_),
		nUnknowns_k2f(nUnknowns_k2f_),
		visible(visible_)
	{}
	const size_t nUnknowns_k2k;
	const size_t nUnknowns_k2f;
	const bool * visible;
};


class SchurTests : public ::testing::Test
{
protected:
	virtual void SetUp()
	{
	}

	virtual void TearDown() {  }

	void test_schur_dense_vs_sparse(
		const TGraphInitRandom  *init_random,
		const TGraphInitManual  *init_manual,
		const double lambda = 1e3 )
	{
		// A linear system object holds the sparse Jacobians for a set of observations.
		my_rba_t::rba_problem_state_t::TLinearSystem  lin_system;

		size_t nUnknowns_k2k=0, nUnknowns_k2f=0;

		if (init_random)
		{
			randomGenerator.randomize(init_random->random_seed);
			nUnknowns_k2k=init_random->nUnknowns_k2k;
			nUnknowns_k2f=init_random->nUnknowns_k2f;
		}
		else
		{
			nUnknowns_k2k=init_manual->nUnknowns_k2k;
			nUnknowns_k2f=init_manual->nUnknowns_k2f;
		}

		// Fill example Jacobians for the test:
		//  * 2 keyframes -> 1 k2k edge (edges=unknowns)
		//  * 6 features with unknown positions.
		//  * 6*2 observations: each feature seen once from each keyframe
		// Note: 6*2*2 = 24 is the minimum >= 1*6+3*6=24 unknowns so
		//   Hessians are invertible
		// -----------------------------------------------------------------
		// Create observations:
		// Don't populate the symbolic structure, just the numeric part.
        static char valid_true = 1; // Just to initialize valid bit pointers to this one.
		{

			lin_system.dh_dAp.setColCount(nUnknowns_k2k);
			lin_system.dh_df.setColCount(nUnknowns_k2f);
			size_t idx_obs = 0;
			for (size_t nKF=0;nKF<=nUnknowns_k2k;nKF++)
			{
				my_rba_t::jacobian_traits_t::TSparseBlocksJacobians_dh_dAp::col_t * dh_dAp_i = (nKF==0) ? NULL : (&lin_system.dh_dAp.getCol(nKF-1));

				for (size_t nLM=0;nLM<nUnknowns_k2f;nLM++)
				{
					// Do we have an observation of feature "nLM" from keyframe "nKF"??
					bool add_this;

					if (init_random)
							add_this = (randomGenerator.drawUniform(0.0,1.0)<=init_random->PROB_OBS);
					else	add_this = init_manual->visible[nKF*nUnknowns_k2f + nLM];

					if (add_this)
					{
						// Create observation: KF[nKF] -> LM[nLM]
						my_rba_t::jacobian_traits_t::TSparseBlocksJacobians_dh_df::col_t & dh_df_j = lin_system.dh_df.getCol(nLM);

						// Random is ok for this test:
						if (dh_dAp_i)
						{
							randomGenerator.drawGaussian1DMatrix( (*dh_dAp_i)[idx_obs].num  );
							(*dh_dAp_i)[idx_obs].sym.is_valid = &valid_true;
						}
						randomGenerator.drawGaussian1DMatrix( dh_df_j[idx_obs].num.setRandom() );
						dh_df_j[idx_obs].sym.is_valid = &valid_true;

						idx_obs++;
					}
				}
			}
			// Debug point:
			//if ( idx_obs != (1+nUnknowns_k2k)*nUnknowns_k2f ) cout << "#k2k: " << nUnknowns_k2k << " #k2f: " << nUnknowns_k2f << " #obs: " << idx_obs << " out of max.=" << (1+nUnknowns_k2k)*nUnknowns_k2f << endl;
		}

		// A default gradient:
		Eigen::VectorXd  minus_grad; // The negative of the gradient.
		const size_t idx_start_f = 6*nUnknowns_k2k;
		const size_t nLMs_scalars = 3*nUnknowns_k2f;
		const size_t nUnknowns_scalars = idx_start_f + nLMs_scalars;

		minus_grad.resize(nUnknowns_scalars);
		minus_grad.setOnes();

	#if 0
		lin_system.dh_dAp.saveToTextFileAsDense("test_dh_dAp.txt");
		lin_system.dh_df.saveToTextFileAsDense("test_dh_df.txt");
	#endif

		// ------------------------------------------------------------
		// 1st) Evaluate Schur naively with dense matrices
		// ------------------------------------------------------------
		CMatrixDouble  dense_dh_dAp, dense_dh_df;
		lin_system.dh_dAp.getAsDense(dense_dh_dAp);
		lin_system.dh_df.getAsDense (dense_dh_df);

		const CMatrixDouble  dense_HAp  = dense_dh_dAp.transpose() * dense_dh_dAp;
		const CMatrixDouble  dense_Hf   = dense_dh_df.transpose()  * dense_dh_df;
		const CMatrixDouble  dense_HApf = dense_dh_dAp.transpose() * dense_dh_df;

		CMatrixDouble  dense_Hf_plus_lambda = dense_Hf;
		for (size_t i=0;i<nLMs_scalars;i++)
			dense_Hf_plus_lambda(i,i)+=lambda;


		// Schur: naive dense computation:
		const CMatrixDouble  dense_HAp_schur  = dense_HAp - dense_HApf*dense_Hf_plus_lambda.inv()*dense_HApf.transpose();
		const Eigen::VectorXd  dense_minus_grad_schur = minus_grad.head(idx_start_f) - dense_HApf*(dense_Hf_plus_lambda.inv())*minus_grad.tail(nLMs_scalars);

	#if 0
		dense_HAp.saveToTextFile("dense_HAp.txt");
		dense_Hf.saveToTextFile("dense_Hf.txt");
		dense_HApf.saveToTextFile("dense_HApf.txt");
	#endif

		// ------------------------------------------------------------
		// 2nd) Evaluate using sparse Schur implementation
		// ------------------------------------------------------------
		// Build a list with ALL the unknowns:
		vector<my_rba_t::jacobian_traits_t::TSparseBlocksJacobians_dh_dAp::col_t*> dh_dAp;
		vector<my_rba_t::jacobian_traits_t::TSparseBlocksJacobians_dh_df::col_t*>  dh_df;

		for (size_t i=0;i<lin_system.dh_dAp.getColCount();i++)
			dh_dAp.push_back( & lin_system.dh_dAp.getCol(i) );

		for (size_t i=0;i<lin_system.dh_df.getColCount();i++)
			dh_df.push_back( & lin_system.dh_df.getCol(i) );

#if 0
	{
		CMatrixDouble Jbin;
		lin_system.dh_dAp.getBinaryBlocksRepresentation(Jbin);
		Jbin.saveToTextFile("test_sparse_jacobs_blocks.txt", mrpt::math::MATRIX_FORMAT_INT );
		lin_system.dh_dAp.saveToTextFileAsDense("test_sparse_jacobs.txt");
	}
#endif

		my_rba_t::hessian_traits_t::TSparseBlocksHessian_Ap  HAp;
		my_rba_t::hessian_traits_t::TSparseBlocksHessian_f   Hf;
		my_rba_t::hessian_traits_t::TSparseBlocksHessian_Apf HApf;  // This one stores in row-compressed form (i.e. it's stored transposed!!!)

		// This resizes and fills in the structs HAp,Hf,HApf from Jacobians:
		my_rba_t::sparse_hessian_build_symbolic(
			HAp,Hf,HApf,
			dh_dAp,dh_df
			);


		my_rba_t rba;

		rba.sparse_hessian_update_numeric(HAp);
		rba.sparse_hessian_update_numeric(Hf);
		rba.sparse_hessian_update_numeric(HApf);

	#if 0
		HAp.saveToTextFileAsDense("HAp.txt", true, true );
		Hf.saveToTextFileAsDense("Hf.txt", true, true);
		HApf.saveToTextFileAsDense("HApf.txt",false, false);
		minus_grad.saveToTextFile("minus_grad_Ap.txt");
		{ ofstream f("lambda.txt"); f << lambda << endl; }
	#endif

		// The constructor builds the symbolic Schur.
		SchurComplement<
			my_rba_t::hessian_traits_t::TSparseBlocksHessian_Ap,
			my_rba_t::hessian_traits_t::TSparseBlocksHessian_f,
			my_rba_t::hessian_traits_t::TSparseBlocksHessian_Apf
			>
			schur_compl(
				HAp,Hf,HApf, // The different symbolic/numeric Hessian
				&minus_grad[0],  // minus gradient of the Ap part
				&minus_grad[idx_start_f]   // minus gradient of the features part
				);

		schur_compl.numeric_build_reduced_system(lambda);
		//cout << "getNumFeaturesFullRank: " << schur_compl.getNumFeaturesFullRank() << endl;

		// ------------------------------------------------------------
		// 3rd) Both must match!
		// ------------------------------------------------------------
		// * HAp: Holds the updated Schur complement Hessian.
		// * minus_grad: Holds the updated Schur complement -gradient.
		CMatrixDouble final_HAp_schur;
		HAp.getAsDense(final_HAp_schur, true /* force symmetry, i.e. replicate the half matrix not stored in sparse form */ );

#if 0
		final_HAp_schur.saveToTextFile("schur_HAp.txt");
#endif


		EXPECT_NEAR( (dense_HAp_schur-final_HAp_schur).array().abs().maxCoeff()/(dense_HAp_schur.array().abs().maxCoeff()),0, 1e-10)
			<< "nUnknowns_k2k=" << nUnknowns_k2k << endl
			<< "nUnknowns_k2f=" << nUnknowns_k2f << endl
			<< "final_HAp_schur:\n" << final_HAp_schur << endl
			<< "dense_HAp_schur:\n" << dense_HAp_schur << endl
			;

		const Eigen::VectorXd final_minus_grad_Ap = minus_grad.head(idx_start_f);
		const double Ap_minus_grad_Ap_max_error = (dense_minus_grad_schur-final_minus_grad_Ap).array().abs().maxCoeff();
		EXPECT_NEAR( Ap_minus_grad_Ap_max_error/dense_minus_grad_schur.array().abs().maxCoeff(),0, 1e-10)
			<< "nUnknowns_k2k=" << nUnknowns_k2k << endl
			<< "nUnknowns_k2f=" << nUnknowns_k2f << endl
			//<< "dense_minus_grad_schur:\n" << dense_minus_grad_schur
			//<< "final_minus_grad_Ap:\n" << final_minus_grad_Ap << endl
			;

	#if 0
		HAp.saveToTextFileAsDense("test_HAp_sparse_schur.txt");
		dense_HAp_schur.saveToTextFile("test_HAp_sparse_schur2.txt");
	#endif

	}

};


// The minimum case: 1 k2k edge, 6 k2f features (with 6*2=12 observations)
TEST_F(SchurTests,DenseVsSparseCheck_1k2k_6k2f)
{
	for (uint32_t random_seed=1;random_seed<5;random_seed++)
	{
		TGraphInitRandom gir(random_seed, 1,6,  1.0 /* Probability of Obs. */);
		test_schur_dense_vs_sparse(&gir,NULL );
	}
}

// other random tests:
TEST_F(SchurTests,DenseVsSparseCheck_1k2k_7k2f)
{
	for (uint32_t random_seed=1;random_seed<15;random_seed++)
	{
		TGraphInitRandom gir(random_seed, 1,7,  1.0 /* Probability of Obs. */);
		test_schur_dense_vs_sparse(&gir,NULL );
	}
}

TEST_F(SchurTests,DenseVsSparseCheck_1k2k_8k2f_random_visible)
{
	for (uint32_t random_seed=1;random_seed<15;random_seed++)
	{
		TGraphInitRandom gir(random_seed, 1,7,  0.95 /* Probability of Obs. */);
		test_schur_dense_vs_sparse(&gir,NULL );
	}
}

TEST_F(SchurTests,DenseVsSparseCheck_1k2k_30k2f)
{
	for (uint32_t random_seed=1;random_seed<15;random_seed++)
	{
		TGraphInitRandom gir(random_seed, 1,30,  1.0 /* Probability of Obs. */);
		test_schur_dense_vs_sparse(&gir,NULL );
	}
}

TEST_F(SchurTests,DenseVsSparseCheck_2k2k_20k2f)
{
	for (uint32_t random_seed=1;random_seed<5;random_seed++)
	{
		TGraphInitRandom gir(random_seed, 2,20, 0.9 /* Probability of Obs. */);
		test_schur_dense_vs_sparse(&gir,NULL );
	}
}

TEST_F(SchurTests,DenseVsSparseCheck_2k2k_some_unobserved)
{
	{
		const bool visibles[] = {
			true,true, true, true, false,
			true,true, true, false, true,
			true,true, false, true, true
		};
		TGraphInitManual gim(2,5,visibles);
		test_schur_dense_vs_sparse(NULL,&gim);
	}
	{
		const bool visibles[] = {
			false,true , false, true , false, true,
			true ,false, true , false, true , false,
			true ,true , true , true , true , true
		};
		TGraphInitManual gim(2,6,visibles);
		test_schur_dense_vs_sparse(NULL,&gim);
	}
}

TEST_F(SchurTests,DenseVsSparseCheck_5k2k_30k2f)
{
	for (uint32_t random_seed=1;random_seed<5;random_seed++)
	{
		TGraphInitRandom gir(random_seed, 5,30, 0.9 /* Probability of Obs. */);
		test_schur_dense_vs_sparse(&gir,NULL );
	}
}

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Note: Matrices unit tests have been split in different files since
// building them with eigen3 eats a lot of RAM and may be a problem while
// compiling in small systems.

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/random.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

#define CHECK_AND_RET_ERROR(_COND_, _MSG_) EXPECT_FALSE(_COND_) << _MSG_;

TEST(Matrices, setSize)
{
  {
    CMatrixFixed<double, 6, 6> M;
    EXPECT_TRUE((M.array() == 0).all());
  }
  {
    CMatrixDouble M(5, 5);
    EXPECT_TRUE((M.array() == 0).all());
  }
  {
    CMatrixDouble M(5, 5);
    M.setSize(6, 5, true /* set new entries to zero*/);
    EXPECT_TRUE((M.array() == 0).all());
  }
  {
    CMatrixDouble M(5, 5);
    M.setSize(10, 5, true /* set new entries to zero*/);
    EXPECT_TRUE((M.array() == 0).all());
  }
  {
    CMatrixDouble M(5, 5);
    M.setSize(5, 6, true /* set new entries to zero*/);
    EXPECT_TRUE((M.array() == 0).all());
  }
  {
    CMatrixDouble M(5, 5);
    M.setSize(6, 6, true /* set new entries to zero*/);
    EXPECT_TRUE((M.array() == 0).all());
  }
  {
    CMatrixDouble M(5, 5);
    M.setSize(10, 10, true /* set new entries to zero*/);
    EXPECT_TRUE((M.array() == 0).all());
  }
}

TEST(Matrices, extractSubmatrixSymmetricalBlocks)
{
  {
    const double vals[] = {1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15,
                           1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15,
                           1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15,
                           1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15};
    const CMatrixFixed<double, 8, 8> M(vals);

    std::vector<size_t> vs;
    vs.push_back(1);
    vs.push_back(3);

    CMatrixDouble E;
    mrpt::math::extractSubmatrixSymmetricalBlocks<2>(M, vs, E);

    const double valsE[] = {3, 4, 7, 8, 10, 11, 14, 15, 3, 4, 7, 8, 10, 11, 14, 15};
    const CMatrixDouble44 E_expected(valsE);

    EXPECT_TRUE(E_expected == E);
  }
}

TEST(Matrices, extractSubmatrixSymmetrical)
{
  {
    const double vals[] = {1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15,
                           1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15,
                           1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15,
                           1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15};
    const CMatrixFixed<double, 8, 8> M(vals);

    std::vector<size_t> vs;
    vs.push_back(2);
    vs.push_back(3);
    vs.push_back(6);
    vs.push_back(7);

    CMatrixDouble E;
    mrpt::math::extractSubmatrixSymmetrical(M, vs, E);

    const double valsE[] = {3, 4, 7, 8, 10, 11, 14, 15, 3, 4, 7, 8, 10, 11, 14, 15};
    const CMatrixDouble44 E_expected(valsE);

    EXPECT_TRUE(E_expected == E);
  }
}

TEST(Matrices, removeColumns)
{
  for (size_t i = 0; i < 6; i++)
  {
    auto M = mrpt::math::CMatrixDouble::Identity(6);
    EXPECT_EQ(M.cols(), 6);
    M.removeColumns({i});
    EXPECT_EQ(M.cols(), 5) << "For {i}=" << i;
  }

  {
    auto M = mrpt::math::CMatrixDouble();  // empty
    EXPECT_EQ(M.cols(), 0);
    EXPECT_ANY_THROW(M.removeColumns({0}));
    EXPECT_ANY_THROW(M.removeColumns({1}));
  }
}

TEST(Matrices, removeRows)
{
  for (size_t i = 0; i < 6; i++)
  {
    auto M = mrpt::math::CMatrixDouble::Identity(6);
    EXPECT_EQ(M.rows(), 6);
    M.removeRows({i});
    EXPECT_EQ(M.rows(), 5) << "For {i}=" << i;
  }

  {
    auto M = mrpt::math::CMatrixDouble();  // empty
    EXPECT_EQ(M.rows(), 0);
    EXPECT_ANY_THROW(M.removeRows({0}));
    EXPECT_ANY_THROW(M.removeRows({1}));
  }
}

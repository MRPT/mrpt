/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

// Note: Matrices unit tests have been split in different files since
// building them with eigen3 eats a lot of RAM and may be a problem while
// compiling in small systems.

#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/matrix_serialization.h>  // serialization of matrices
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

const double dat_A[] = {4, 5, 8, -2, 1, 3};

TEST(Matrices, SerializeCMatrixD)
{
  CMatrixDouble A(3, 2, dat_A);
  CMatrixFixed<double, 3, 2> fA;

  CMatrixD As = CMatrixD(A);

  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch << As;
  membuf.Seek(0);
  arch >> fA;

  EXPECT_NEAR(0, fabs((CMatrixDouble(fA) - A).sum()), 1e-9);

  try
  {
    // Now, if we try to de-serialize into the wrong type, we should get an
    // exception:
    membuf.Seek(0);
    CMatrixFixed<double, 2, 2> fB;
    arch >> fB;  // Wrong size!

    GTEST_FAIL() << "Exception not launched when it was expected!";
  }
  catch (...)
  {  // OK, exception occurred, as expected
  }
}

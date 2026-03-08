/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/math/utils.h>
#include <mrpt/system/CTicTac.h>

#include <Eigen/Dense>
#include <iostream>
#include <sstream>

namespace
{
// ------------------------------------------------------
//				TestChol
// ------------------------------------------------------
void TestChol()
{
  std::stringstream ss;
  ss << "2.0302432e+000  1.4548216e+000  1.5770631e+000  1.7090746e+000  1.6648359e+000\n"
        "1.4548216e+000  1.8885715e+000  2.0234057e+000  1.4591638e+000  1.4335901e+000\n"
        "1.5770631e+000  2.0234057e+000  2.7202930e+000  1.3748003e+000  1.6439239e+000\n"
        "1.7090746e+000  1.4591638e+000  1.3748003e+000  1.6242575e+000  1.2965666e+000\n"
        "1.6648359e+000  1.4335901e+000  1.6439239e+000  1.2965666e+000  1.8411448e+000\n";
  ss.seekg(0);

  mrpt::math::CMatrixFloat A;
  A.loadFromTextFile(ss);
  mrpt::math::CMatrixFloat B;
  A.chol(B);

  std::cout << "Cholesky decomposition result:\n" << B;
}

void TestInitMatrix()
{
  // Initialize a matrix from a C array:
  const double numbers[] = {1, 2, 3, 4, 5, 6};
  mrpt::math::CMatrixDouble M(2, 3, numbers);
  std::cout << "Initialized matrix (I): "
            << "\n"
            << M << "\n";

  const double numbers2[] = {0.5, 4.5, 6.7, 8.9, 15.2};
  mrpt::math::CVectorDouble v1;
  mrpt::math::loadVector(v1, numbers2);
  std::cout << "Initialized double vector: ";
  for (auto x : v1) std::cout << x << " ";
  std::cout << "\n";

  std::vector<int> v2;
  mrpt::math::loadVector(v2, numbers2);
  std::cout << "Initialized int vector: ";
  for (auto x : v2) std::cout << x << " ";
  std::cout << "\n";

  /*	// I/O Test
    CMatrixD  B(M);
    CFileOutputStream("mat.bin") << B;
    CMatrixD  A;
    CFileInputStream("mat.bin") >> A;
    cout << "B:" << endl << B;
    cout << "A:" << endl << A;
  */
}

void TestHCH()
{
  mrpt::math::CMatrixFloat H, C, RES;

  std::cout << "reading H.txt...";
  {
    std::stringstream ss;
    ss << " 0.2839445876680414 -0.8644184738883179 -0.1705788928311085 -0.3517043133524523 "
          "0.5536739169045888\n"
       << " 0.3710637379046813 0.9983356795804031 0.1552920730290511 0.02399147777280302 "
          "1.253196636327196\n"
       << " -0.7535627070532932 -1.578288100885369 0.5409409955881032 -0.09888686980102715 "
          "-0.7969177822619684\n"
       << " -1.056652322493684 -0.7655729902415248 -0.624006495918541 0.2460160436422357 "
          "0.2148069621922654\n";
    ss.seekg(0);
    H.loadFromTextFile(ss);
  }
  std::cout << "ok"
            << "\n";

  std::cout << "reading C.txt...";
  {
    std::stringstream ss;
    ss << " 16.02532733629319 -10.6901035809514 -7.334956807573198 4.785099693778158 "
          "-0.3173027335638579\n"
       << " -10.6901035809514 14.14720370421467 8.536254766332371 4.575182475768872 "
          "-2.378574616738005\n"
       << " -7.334956807573198 8.536254766332371 5.987227584524485 2.440688352895442 "
          "-1.229980647514298\n"
       << " 4.785099693778158 4.575182475768872 2.440688352895442 10.70550665735928 "
          "-3.592282328640898\n"
       << " -0.3173027335638579 -2.378574616738005 -1.229980647514298 -3.592282328640898 "
          "3.261545586070256\n";
    ss.seekg(0);
    C.loadFromTextFile(ss);
  }
  std::cout << "ok"
            << "\n";

  // RES = H * C * H'
  mrpt::math::multiply_HCHt(H, C, RES);
  std::cout << "Saving RES.txt ...";
  RES.saveToTextFile("RES.txt");
  std::cout << "ok"
            << "\n";

  // The same for a column vector:
  {
    std::stringstream ss;
    ss << "-1.15962523058769 \n"
       << "-0.4877050122868786 \n"
       << "-1.057293467588184 \n"
       << "0.03727046302098693 \n"
       << "1.891257803766057\n";
    ss.seekg(0);
    H.loadFromTextFile(ss);
  }
  std::cout << "H*C*(H') = " << mrpt::math::multiply_HCHt_scalar(H, C) << "\n";
  std::cout << "Should be= 31.434 "
            << "\n";

  // The same for a row vector:
  {
    std::stringstream ss;
    ss << " -1.15962523058769 -0.4877050122868786 -1.057293467588184 0.03727046302098693 "
          "1.891257803766057\n";
    ss.seekg(0);
    H.loadFromTextFile(ss);
  }
  std::cout << "Loaded H: "
            << "\n"
            << H;
  std::cout << "H*C*(H') = " << mrpt::math::multiply_HCHt_scalar(H, C) << "\n";
  std::cout << "Should be= 31.434"
            << "\n";
}

void TestMatrixTemplate()
{
  mrpt::math::CMatrixDouble M;

  // --------------------------------------
  {
    std::stringstream ss;
    ss << "1 0 0 \n"
       << "0 1 0\n"
       << "0 0 1\n";
    ss.seekg(0);
    M.loadFromTextFile(ss);
  }
  std::cout << M << "\n";

  mrpt::math::CMatrixDouble eigenVectors;
  std::vector<double> eigenValues;
  M.eig(eigenVectors, eigenValues);
  std::cout << "eigenVectors:\n" << eigenVectors << "\n Eigenvalues:\n";
  for (auto x : eigenValues) std::cout << x << " ";
  std::cout << "\n";

  mrpt::math::CMatrixDouble D;
  D.setDiagonal(eigenValues);

  mrpt::math::CMatrixDouble RES;
  RES = M.asEigen() * D.asEigen() * M.transpose();
  std::cout << "RES:\n" << RES;
}

void TestMatrices()
{
  mrpt::math::CMatrixFloat m, l;
  mrpt::system::CTicTac tictac;
  double t;

  m.setSize(4, 4);
  m(0, 0) = 4;
  m(0, 1) = -2;
  m(0, 2) = -1;
  m(0, 3) = 0;
  m(1, 0) = -2;
  m(1, 1) = 4;
  m(1, 2) = 0;
  m(1, 3) = -1;
  m(2, 0) = -1;
  m(2, 1) = 0;
  m(2, 2) = 4;
  m(2, 3) = -2;
  m(3, 0) = 0;
  m(3, 1) = -1;
  m(3, 2) = -2;
  m(3, 3) = 4;

  std::cout << "Matrix:\n" << m << "\n";

  // I/O test through a text file:
  tictac.Tic();
  {
    std::stringstream ss;
    ss << "4 -2 -1 0\n"
       << "-2 4 0 -1\n"
       << "-1 0 4 -2\n"
       << "0 -1 -2 4\n";
    ss.seekg(0);
    l.loadFromTextFile(ss);
  }
  t = tictac.Tac();
  std::cout << "Read (text file) in " << 1e6 * t << "us:\n" << l << "\n";
  mrpt::math::laplacian(m, l);

  std::cout << "Laplacian:\n" << l << "\n";
}

void TestCov()
{
  // Initialize a matrix from a C array:
  const double numbers[] = {1, 2, 3, 10, 4, 5, 6, 14, 10, -5, -3, 1};
  mrpt::math::CMatrixDouble Mdyn(4, 3, numbers);
  mrpt::math::CMatrixFixed<double, 4, 3> Mfix(numbers);

  std::vector<mrpt::math::CVectorDouble> samples(4);
  for (size_t i = 0; i < 4; i++)
  {
    samples[i].resize(3);
    for (size_t j = 0; j < 3; j++) samples[i][j] = Mdyn(i, j);
  }

  std::cout << "COV (vector of vectors): "
            << "\n"
            << mrpt::math::covVector<std::vector<mrpt::math::CVectorDouble>, Eigen::MatrixXd>(
                   samples)
            << "\n";
  std::cout << "COV (mat fix): "
            << "\n"
            << mrpt::math::cov(Mfix) << "\n";
  std::cout << "COV (mat dyn): "
            << "\n"
            << mrpt::math::cov(Mdyn) << "\n";
}

}  // namespace

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
  try
  {
    TestInitMatrix();
    TestMatrixTemplate();
    TestMatrices();
    TestHCH();
    TestChol();
    TestCov();

    return 0;
  }
  catch (std::exception& e)
  {
    std::cout << "MRPT exception caught: " << e.what() << "\n";
    return -1;
  }
}

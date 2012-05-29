// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2012 Désiré Nuentsa-Wakam <desire.nuentsa_wakam@inria.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

#ifndef EIGEN_PASTIXSUPPORT_H
#define EIGEN_PASTIXSUPPORT_H

namespace Eigen { 

/** \ingroup PaStiXSupport_Module
  * \brief Interface to the PaStix solver
  * 
  * This class is used to solve the linear systems A.X = B via the PaStix library. 
  * The matrix can be either real or complex, symmetric or not.
  *
  * \sa TutorialSparseDirectSolvers
  */

template<typename _MatrixType, bool IsStrSym = false> class PastixLU;
template<typename _MatrixType, int Options> class PastixLLT;
template<typename _MatrixType, int Options> class PastixLDLT;

namespace internal
{
    
  template<class Pastix> struct pastix_traits;

  template<typename _MatrixType>
  struct pastix_traits< PastixLU<_MatrixType> >
  {
    typedef _MatrixType MatrixType;
    typedef typename _MatrixType::Scalar Scalar;
    typedef typename _MatrixType::RealScalar RealScalar;
    typedef typename _MatrixType::Index Index;
  };

  template<typename _MatrixType, int Options>
  struct pastix_traits< PastixLLT<_MatrixType,Options> >
  {
    typedef _MatrixType MatrixType;
    typedef typename _MatrixType::Scalar Scalar;
    typedef typename _MatrixType::RealScalar RealScalar;
    typedef typename _MatrixType::Index Index;
  };

  template<typename _MatrixType, int Options>
  struct pastix_traits< PastixLDLT<_MatrixType,Options> >
  {
    typedef _MatrixType MatrixType;
    typedef typename _MatrixType::Scalar Scalar;
    typedef typename _MatrixType::RealScalar RealScalar;
    typedef typename _MatrixType::Index Index;
  };
  
  void eigen_pastix(pastix_data_t **pastix_data, int pastix_comm, int n, int *ptr, int *idx, float *vals, int *perm, int * invp, float *x, int nbrhs, int *iparm, double *dparm)
  {
    if (n == 0) { ptr = NULL; idx = NULL; vals = NULL; }
    if (nbrhs == 0) x = NULL;
    s_pastix(pastix_data, pastix_comm, n, ptr, idx, vals, perm, invp, x, nbrhs, iparm, dparm); 
  }
  
  void eigen_pastix(pastix_data_t **pastix_data, int pastix_comm, int n, int *ptr, int *idx, double *vals, int *perm, int * invp, double *x, int nbrhs, int *iparm, double *dparm)
  {
    if (n == 0) { ptr = NULL; idx = NULL; vals = NULL; }
    if (nbrhs == 0) x = NULL;
    d_pastix(pastix_data, pastix_comm, n, ptr, idx, vals, perm, invp, x, nbrhs, iparm, dparm); 
  }
  
  void eigen_pastix(pastix_data_t **pastix_data, int pastix_comm, int n, int *ptr, int *idx, std::complex<float> *vals, int *perm, int * invp, std::complex<float> *x, int nbrhs, int *iparm, double *dparm)
  {
    c_pastix(pastix_data, pastix_comm, n, ptr, idx, reinterpret_cast<COMPLEX*>(vals), perm, invp, reinterpret_cast<COMPLEX*>(x), nbrhs, iparm, dparm); 
  }
  
  void eigen_pastix(pastix_data_t **pastix_data, int pastix_comm, int n, int *ptr, int *idx, std::complex<double> *vals, int *perm, int * invp, std::complex<double> *x, int nbrhs, int *iparm, double *dparm)
  {
    if (n == 0) { ptr = NULL; idx = NULL; vals = NULL; }
    if (nbrhs == 0) x = NULL;
    z_pastix(pastix_data, pastix_comm, n, ptr, idx, reinterpret_cast<DCOMPLEX*>(vals), perm, invp, reinterpret_cast<DCOMPLEX*>(x), nbrhs, iparm, dparm); 
  }

  // Convert the matrix  to Fortran-style Numbering
  template <typename MatrixType>
  void EigenToFortranNumbering (MatrixType& mat)
  {
    if ( !(mat.outerIndexPtr()[0]) ) 
    { 
      int i;
      for(i = 0; i <= mat.rows(); ++i)
        ++mat.outerIndexPtr()[i];
      for(i = 0; i < mat.nonZeros(); ++i)
        ++mat.innerIndexPtr()[i];
    }
  }
  
  // Convert to C-style Numbering
  template <typename MatrixType>
  void EigenToCNumbering (MatrixType& mat)
  {
    // Check the Numbering
    if ( mat.outerIndexPtr()[0] == 1 ) 
    { // Convert to C-style numbering
      int i;
      for(i = 0; i <= mat.rows(); ++i)
        --mat.outerIndexPtr()[i];
      for(i = 0; i < mat.nonZeros(); ++i)
        --mat.innerIndexPtr()[i];
    }
  }
  
  // Symmetrize the graph of the input matrix 
  // In : The Input matrix to symmetrize the pattern
  // Out : The output matrix
  // StrMatTrans : The structural pattern of the transpose of In; It is 
  // used to optimize the future symmetrization with the same matrix pattern
  // WARNING It is assumed here that successive calls to this routine are done 
  // with matrices having the same pattern.
  template <typename MatrixType>
  void EigenSymmetrizeMatrixGraph (const MatrixType& In, MatrixType& Out, MatrixType& StrMatTrans, bool& hasTranspose)
  {
    eigen_assert(In.cols()==In.rows() && " Can only symmetrize the graph of a square matrix");
    if (!hasTranspose)
    { //First call to this routine, need to compute the structural pattern of In^T
      StrMatTrans = In.transpose();
      // Set the elements of the matrix to zero 
      for (int i = 0; i < StrMatTrans.rows(); i++) 
      {
        for (typename MatrixType::InnerIterator it(StrMatTrans, i); it; ++it)
          it.valueRef() = 0.0;
      }
      hasTranspose = true;
    }
    Out = (StrMatTrans + In).eval(); 
  }

}

// This is the base class to interface with PaStiX functions. 
// Users should not used this class directly. 
template <class Derived>
class PastixBase
{
  public:
    typedef typename internal::pastix_traits<Derived>::MatrixType _MatrixType;
    typedef _MatrixType MatrixType;
    typedef typename MatrixType::Scalar Scalar;
    typedef typename MatrixType::RealScalar RealScalar;
    typedef typename MatrixType::Index Index;
    typedef Matrix<Scalar,Dynamic,1> Vector;
    
  public:
    
    PastixBase():m_initisOk(false),m_analysisIsOk(false),m_factorizationIsOk(false),m_isInitialized(false) 
    {
      m_pastixdata = 0;
      m_hasTranspose = false;
      PastixInit();
    }
    
    ~PastixBase() 
    {
       PastixDestroy();
    }
    
    // Initialize the Pastix data structure, check the matrix
    void PastixInit(); 
    
    // Compute the ordering and the symbolic factorization
    Derived& analyzePattern (MatrixType& mat);
    
    // Compute the numerical factorization
    Derived& factorize (MatrixType& mat);

    /** \returns the solution x of \f$ A x = b \f$ using the current decomposition of A.
      *
      * \sa compute()
      */
    template<typename Rhs>
    inline const internal::solve_retval<PastixBase, Rhs>
    solve(const MatrixBase<Rhs>& b) const
    {
      eigen_assert(m_isInitialized && "Pastix solver is not initialized.");
      eigen_assert(rows()==b.rows()
                && "PastixBase::solve(): invalid number of rows of the right hand side matrix b");
      return internal::solve_retval<PastixBase, Rhs>(*this, b.derived());
    }
    
    template<typename Rhs,typename Dest>
    bool _solve (const MatrixBase<Rhs> &b, MatrixBase<Dest> &x) const;
    
    /** \internal */
    template<typename Rhs, typename DestScalar, int DestOptions, typename DestIndex>
    void _solve_sparse(const Rhs& b, SparseMatrix<DestScalar,DestOptions,DestIndex> &dest) const
    {
      eigen_assert(m_factorizationIsOk && "The decomposition is not in a valid state for solving, you must first call either compute() or symbolic()/numeric()");
      eigen_assert(rows()==b.rows());
      
      // we process the sparse rhs per block of NbColsAtOnce columns temporarily stored into a dense matrix.
      static const int NbColsAtOnce = 1;
      int rhsCols = b.cols();
      int size = b.rows();
      Eigen::Matrix<DestScalar,Dynamic,Dynamic> tmp(size,rhsCols);
      for(int k=0; k<rhsCols; k+=NbColsAtOnce)
      {
        int actualCols = std::min<int>(rhsCols-k, NbColsAtOnce);
        tmp.leftCols(actualCols) = b.middleCols(k,actualCols);
        tmp.leftCols(actualCols) = derived().solve(tmp.leftCols(actualCols));
        dest.middleCols(k,actualCols) = tmp.leftCols(actualCols).sparseView();
      }
    }
    
    Derived& derived()
    {
      return *static_cast<Derived*>(this);
    }
    const Derived& derived() const
    {
      return *static_cast<const Derived*>(this);
    }

    /** Returns a reference to the integer vector IPARM of PaStiX parameters
      * to modify the default parameters. 
      * The statistics related to the different phases of factorization and solve are saved here as well
      * \sa analyzePattern() factorize()
      */
    Array<Index,IPARM_SIZE,1>& iparm()
    {
      return m_iparm; 
    }
    
    /** Return a reference to a particular index parameter of the IPARM vector 
     * \sa iparm()
     */
    
    int& iparm(int idxparam)
    {
      return m_iparm(idxparam);
    }
    
     /** Returns a reference to the double vector DPARM of PaStiX parameters 
      * The statistics related to the different phases of factorization and solve are saved here as well
      * \sa analyzePattern() factorize()
      */
    Array<RealScalar,IPARM_SIZE,1>& dparm()
    {
      return m_dparm; 
    }
    
    
    /** Return a reference to a particular index parameter of the DPARM vector 
     * \sa dparm()
     */
    
    double& dparm(int idxparam)
    {
      return m_dparm(idxparam);
    }
    
    inline Index cols() const { return m_size; }
    inline Index rows() const { return m_size; }
    
     /** \brief Reports whether previous computation was successful.
      *
      * \returns \c Success if computation was succesful,
      *          \c NumericalIssue if the PaStiX reports a problem
      *          \c InvalidInput if the input matrix is invalid
      *
      * \sa iparm()          
      */
    ComputationInfo info() const
    {
      eigen_assert(m_isInitialized && "Decomposition is not initialized.");
      return m_info;
    }
    
    /** \returns the solution x of \f$ A x = b \f$ using the current decomposition of A.
      *
      * \sa compute()
      */
    template<typename Rhs>
    inline const internal::sparse_solve_retval<PastixBase, Rhs>
    solve(const SparseMatrixBase<Rhs>& b) const
    {
      eigen_assert(m_isInitialized && "Pastix LU, LLT or LDLT is not initialized.");
      eigen_assert(rows()==b.rows()
                && "PastixBase::solve(): invalid number of rows of the right hand side matrix b");
      return internal::sparse_solve_retval<PastixBase, Rhs>(*this, b.derived());
    }
    
  protected:
    // Free all the data allocated by Pastix
    void PastixDestroy()
    {
      eigen_assert(m_initisOk && "The Pastix structure should be allocated first"); 
      m_iparm(IPARM_START_TASK) = API_TASK_CLEAN;
      m_iparm(IPARM_END_TASK) = API_TASK_CLEAN;
      internal::eigen_pastix(&m_pastixdata, MPI_COMM_WORLD, 0, m_mat_null.outerIndexPtr(), m_mat_null.innerIndexPtr(),
               m_mat_null.valuePtr(), m_perm.data(), m_invp.data(), m_vec_null.data(), 1, m_iparm.data(), m_dparm.data());
    }
    
    Derived& compute (MatrixType& mat);
    
    int m_initisOk; 
    int m_analysisIsOk;
    int m_factorizationIsOk;
    bool m_isInitialized;
    mutable ComputationInfo m_info; 
    mutable pastix_data_t *m_pastixdata; // Data structure for pastix
    mutable SparseMatrix<Scalar, ColMajor> m_mat_null; // An input  null matrix
    mutable Matrix<Scalar, Dynamic,1> m_vec_null; // An input null vector
    mutable SparseMatrix<Scalar, ColMajor> m_StrMatTrans; // The transpose pattern of the input matrix
    mutable bool m_hasTranspose; // The transpose of the current matrix has already been computed
    mutable int m_comm; // The MPI communicator identifier
    mutable Matrix<Index,IPARM_SIZE,1> m_iparm; // integer vector for the input parameters
    mutable Matrix<double,DPARM_SIZE,1> m_dparm; // Scalar vector for the input parameters
    mutable Matrix<Index,Dynamic,1> m_perm;  // Permutation vector
    mutable Matrix<Index,Dynamic,1> m_invp;  // Inverse permutation vector
    mutable int m_ordering; // ordering method to use
    mutable int m_amalgamation; // level of amalgamation
    mutable int m_size; // Size of the matrix 
    
  private:
    PastixBase(PastixBase& ) {}
    
}; 

 /** Initialize the PaStiX data structure. 
   *A first call to this function fills iparm and dparm with the default PaStiX parameters
   * \sa iparm() dparm()
   */
template <class Derived>
void PastixBase<Derived>::PastixInit()
{
  m_size = 0; 
  m_iparm.resize(IPARM_SIZE);
  m_dparm.resize(DPARM_SIZE);
  
  m_iparm(IPARM_MODIFY_PARAMETER) = API_NO;
  if(m_pastixdata)
  { // This trick is used to reset the Pastix internal data between successive
    // calls with (structural) different matrices
    PastixDestroy();
    m_pastixdata = 0;
    m_iparm(IPARM_MODIFY_PARAMETER) = API_YES;
    m_hasTranspose = false; 
  }
  
  m_iparm(IPARM_START_TASK) = API_TASK_INIT;
  m_iparm(IPARM_END_TASK) = API_TASK_INIT;
  m_iparm(IPARM_MATRIX_VERIFICATION) = API_NO;
  internal::eigen_pastix(&m_pastixdata, MPI_COMM_WORLD, 0, m_mat_null.outerIndexPtr(), m_mat_null.innerIndexPtr(),
               m_mat_null.valuePtr(), m_perm.data(), m_invp.data(), m_vec_null.data(), 1, m_iparm.data(), m_dparm.data());
  
  m_iparm(IPARM_MATRIX_VERIFICATION) = API_NO;
  
  // Check the returned error
  if(m_iparm(IPARM_ERROR_NUMBER)) {
    m_info = InvalidInput;
    m_initisOk = false;
  }
  else { 
    m_info = Success;
    m_initisOk = true;
  }
}

template <class Derived>
Derived& PastixBase<Derived>::compute(MatrixType& mat)
{
  eigen_assert(mat.rows() == mat.cols() && "The input matrix should be squared");
  typedef typename MatrixType::Scalar Scalar;
  
  // Save the size of the current matrix 
  m_size = mat.rows();
  // Convert the matrix in fortran-style numbering 
  internal::EigenToFortranNumbering(mat);
  analyzePattern(mat);
  factorize(mat);
  m_iparm(IPARM_MATRIX_VERIFICATION) = API_NO;
  if (m_factorizationIsOk) m_isInitialized = true;
  
  //Convert back the matrix -- Is it really necessary here 
  internal::EigenToCNumbering(mat);
  
  return derived();
}


template <class Derived>
Derived& PastixBase<Derived>::analyzePattern(MatrixType& mat)
{
  eigen_assert(m_initisOk && "PastixInit should be called first to set the default  parameters");
  m_size = mat.rows();
  m_perm.resize(m_size);
  m_invp.resize(m_size);
  
  // Convert the matrix in fortran-style numbering 
  internal::EigenToFortranNumbering(mat);
  
  m_iparm(IPARM_START_TASK) = API_TASK_ORDERING;
  m_iparm(IPARM_END_TASK) = API_TASK_ANALYSE;
  
  internal::eigen_pastix(&m_pastixdata, MPI_COMM_WORLD, m_size, mat.outerIndexPtr(), mat.innerIndexPtr(),
               mat.valuePtr(), m_perm.data(), m_invp.data(), m_vec_null.data(), 0, m_iparm.data(), m_dparm.data());
  
  // Check the returned error
  if(m_iparm(IPARM_ERROR_NUMBER)) {
    m_info = NumericalIssue;
    m_analysisIsOk = false;
  }
  else { 
    m_info = Success;
    m_analysisIsOk = true;
  }
  return derived();
}

template <class Derived>
Derived& PastixBase<Derived>::factorize(MatrixType& mat)
{
  eigen_assert(m_analysisIsOk && "The analysis phase should be called before the factorization phase");
  m_iparm(IPARM_START_TASK) = API_TASK_NUMFACT;
  m_iparm(IPARM_END_TASK) = API_TASK_NUMFACT;
  m_size = mat.rows();
  
  // Convert the matrix in fortran-style numbering 
  internal::EigenToFortranNumbering(mat);
  
  internal::eigen_pastix(&m_pastixdata, MPI_COMM_WORLD, m_size, mat.outerIndexPtr(), mat.innerIndexPtr(),
               mat.valuePtr(), m_perm.data(), m_invp.data(), m_vec_null.data(), 0, m_iparm.data(), m_dparm.data());
  
  // Check the returned error
  if(m_iparm(IPARM_ERROR_NUMBER)) {
    m_info = NumericalIssue;
    m_factorizationIsOk = false;
    m_isInitialized = false;
  }
  else { 
    m_info = Success;
    m_factorizationIsOk = true;
    m_isInitialized = true;
  }
  return derived();
}

/* Solve the system */
template<typename Base>
template<typename Rhs,typename Dest>
bool PastixBase<Base>::_solve (const MatrixBase<Rhs> &b, MatrixBase<Dest> &x) const
{
  eigen_assert(m_isInitialized && "The matrix should be factorized first");
  EIGEN_STATIC_ASSERT((Dest::Flags&RowMajorBit)==0,
                     THIS_METHOD_IS_ONLY_FOR_COLUMN_MAJOR_MATRICES);
  int rhs = 1;
  
  x = b; /* on return, x is overwritten by the computed solution */
  
  for (int i = 0; i < b.cols(); i++){
    m_iparm(IPARM_START_TASK) = API_TASK_SOLVE;
    m_iparm(IPARM_END_TASK) = API_TASK_REFINE;
    m_iparm(IPARM_RHS_MAKING) = API_RHS_B;
    internal::eigen_pastix(&m_pastixdata, MPI_COMM_WORLD, x.rows(), m_mat_null.outerIndexPtr(), m_mat_null.innerIndexPtr(),
               m_mat_null.valuePtr(), m_perm.data(), m_invp.data(), &x(0, i), rhs, m_iparm.data(), m_dparm.data());
  }
  // Check the returned error
  if(m_iparm(IPARM_ERROR_NUMBER)) {
    m_info = NumericalIssue; 
    return false;
  }
  else {
    return true;
  }
}

/** \ingroup PaStiXSupport_Module
  * \class PastixLU
  * \brief Sparse direct LU solver based on PaStiX library
  * 
  * This class is used to solve the linear systems A.X = B with a supernodal LU 
  * factorization in the PaStiX library. The matrix A should be squared and nonsingular
  * PaStiX requires that the matrix A has a symmetric structural pattern. 
  * This interface can symmetrize the input matrix otherwise. 
  * The vectors or matrices X and B can be either dense or sparse.
  * 
  * \tparam _MatrixType the type of the sparse matrix A, it must be a SparseMatrix<>
  * \tparam IsStrSym Indicates if the input matrix has a symmetric pattern, default is false
  * NOTE : Note that if the analysis and factorization phase are called separately, 
  * the input matrix will be symmetrized at each call, hence it is advised to 
  * symmetrize the matrix in a end-user program and set \p IsStrSym to true
  * 
  * \sa \ref TutorialSparseDirectSolvers
  * 
  */
template<typename _MatrixType, bool IsStrSym>
class PastixLU : public PastixBase< PastixLU<_MatrixType> >
{
  public:
    typedef _MatrixType MatrixType;
    typedef PastixBase<PastixLU<MatrixType> > Base;
    typedef typename MatrixType::Scalar Scalar;
    typedef SparseMatrix<Scalar, ColMajor> PaStiXType; 
    
  public:
    PastixLU():Base() {}
    
    PastixLU(const MatrixType& matrix):Base()
    {
      compute(matrix);
    }
    /** Compute the LU supernodal factorization of \p matrix. 
      * iparm and dparm can be used to tune the PaStiX parameters. 
      * see the PaStiX user's manual
      * \sa analyzePattern() factorize()
      */
    void compute (const MatrixType& matrix)
    {
      // Pastix supports only column-major matrices with a symmetric pattern
      Base::PastixInit(); 
      PaStiXType temp(matrix.rows(), matrix.cols());
      // Symmetrize the graph of the matrix
      if (IsStrSym)   
        temp = matrix;
      else 
      { 
        internal::EigenSymmetrizeMatrixGraph<PaStiXType>(matrix, temp, m_StrMatTrans, m_hasTranspose);
      }
      m_iparm[IPARM_SYM] = API_SYM_NO;
      m_iparm(IPARM_FACTORIZATION) = API_FACT_LU;
      Base::compute(temp);
    }
    /** Compute the LU symbolic factorization of \p matrix using its sparsity pattern. 
      * Several ordering methods can be used at this step. See the PaStiX user's manual. 
      * The result of this operation can be used with successive matrices having the same pattern as \p matrix
      * \sa factorize()
      */
    void analyzePattern(const MatrixType& matrix)
    {
      
      Base::PastixInit(); 
      /* Pastix supports only column-major matrices with symmetrized patterns */
      SparseMatrix<Scalar, ColMajor> temp(matrix.rows(), matrix.cols());
      // Symmetrize the graph of the matrix
      if (IsStrSym)   
        temp = matrix;
      else 
      { 
        internal::EigenSymmetrizeMatrixGraph<PaStiXType>(matrix, temp, m_StrMatTrans,m_hasTranspose);
      }
      
      m_iparm(IPARM_SYM) = API_SYM_NO;
      m_iparm(IPARM_FACTORIZATION) = API_FACT_LU;
      Base::analyzePattern(temp);
    }

    /** Compute the LU supernodal factorization of \p matrix
      * WARNING The matrix \p matrix should have the same structural pattern 
      * as the same used in the analysis phase.
      * \sa analyzePattern()
      */ 
    void factorize(const MatrixType& matrix)
    {
      /* Pastix supports only column-major matrices with symmetrized patterns */
      SparseMatrix<Scalar, ColMajor> temp(matrix.rows(), matrix.cols());
      // Symmetrize the graph of the matrix
      if (IsStrSym)   
        temp = matrix;
      else 
      { 
        internal::EigenSymmetrizeMatrixGraph<PaStiXType>(matrix, temp, m_StrMatTrans,m_hasTranspose);
      }
      m_iparm(IPARM_SYM) = API_SYM_NO;
      m_iparm(IPARM_FACTORIZATION) = API_FACT_LU;
      Base::factorize(temp);
    }
  protected:
    using Base::m_iparm;
    using Base::m_dparm;
    using Base::m_StrMatTrans;
    using Base::m_hasTranspose;
    
  private:
    PastixLU(PastixLU& ) {}
};

/** \ingroup PaStiXSupport_Module
  * \class PastixLLT
  * \brief A sparse direct supernodal Cholesky (LLT) factorization and solver based on the PaStiX library
  * 
  * This class is used to solve the linear systems A.X = B via a LL^T supernodal Cholesky factorization
  * available in the PaStiX library. The matrix A should be symmetric and positive definite
  * WARNING Selfadjoint complex matrices are not supported in the current version of PaStiX
  * The vectors or matrices X and B can be either dense or sparse
  * 
  * \tparam MatrixType the type of the sparse matrix A, it must be a SparseMatrix<>
  * \tparam UpLo The part of the matrix to use : Lower or Upper. The default is Lower as required by PaStiX
  * 
  * \sa \ref TutorialSparseDirectSolvers
  */
template<typename _MatrixType, int _UpLo>
class PastixLLT : public PastixBase< PastixLLT<_MatrixType, _UpLo> >
{
  public:
    typedef _MatrixType MatrixType;
    typedef PastixBase<PastixLLT<MatrixType, _UpLo> > Base;
    typedef typename MatrixType::Scalar Scalar;
    typedef typename MatrixType::Index Index;
    
  public:
    enum { UpLo = _UpLo };
    PastixLLT():Base() {}
    
    PastixLLT(const MatrixType& matrix):Base()
    {
      compute(matrix);
    }

    /** Compute the L factor of the LL^T supernodal factorization of \p matrix 
      * \sa analyzePattern() factorize()
      */
    void compute (const MatrixType& matrix)
    {
      // Pastix supports only lower, column-major matrices
      Base::PastixInit(); // This is necessary to let PaStiX initialize its data structure between successive calls to compute
      SparseMatrix<Scalar, ColMajor> temp(matrix.rows(), matrix.cols());
      PermutationMatrix<Dynamic,Dynamic,Index> pnull;
      temp.template selfadjointView<Lower>() = matrix.template selfadjointView<UpLo>().twistedBy(pnull);
      m_iparm(IPARM_SYM) = API_SYM_YES;
      m_iparm(IPARM_FACTORIZATION) = API_FACT_LLT;
      Base::compute(temp);
    }

     /** Compute the LL^T symbolic factorization of \p matrix using its sparsity pattern
      * The result of this operation can be used with successive matrices having the same pattern as \p matrix
      * \sa factorize()
      */
    void analyzePattern(const MatrixType& matrix)
    {
      Base::PastixInit(); 
      // Pastix supports only lower, column-major matrices
      SparseMatrix<Scalar, ColMajor> temp(matrix.rows(), matrix.cols());
      PermutationMatrix<Dynamic,Dynamic,Index> pnull;
      temp.template selfadjointView<Lower>() = matrix.template selfadjointView<UpLo>().twistedBy(pnull);
      m_iparm(IPARM_SYM) = API_SYM_YES;
      m_iparm(IPARM_FACTORIZATION) = API_FACT_LLT;
      Base::analyzePattern(temp);
    }
      /** Compute the LL^T supernodal numerical factorization of \p matrix 
        * \sa analyzePattern()
        */
    void factorize(const MatrixType& matrix)
    {
      // Pastix supports only lower, column-major matrices 
      SparseMatrix<Scalar, ColMajor> temp(matrix.rows(), matrix.cols());
      PermutationMatrix<Dynamic,Dynamic,Index> pnull;
      temp.template selfadjointView<Lower>() = matrix.template selfadjointView<UpLo>().twistedBy(pnull);
      m_iparm(IPARM_SYM) = API_SYM_YES;
      m_iparm(IPARM_FACTORIZATION) = API_FACT_LLT;
      Base::factorize(temp);
    }
  protected:
    using Base::m_iparm;
    
  private:
    PastixLLT(PastixLLT& ) {}
};

/** \ingroup PaStiXSupport_Module
  * \class PastixLDLT
  * \brief A sparse direct supernodal Cholesky (LLT) factorization and solver based on the PaStiX library
  * 
  * This class is used to solve the linear systems A.X = B via a LDL^T supernodal Cholesky factorization
  * available in the PaStiX library. The matrix A should be symmetric and positive definite
  * WARNING Selfadjoint complex matrices are not supported in the current version of PaStiX
  * The vectors or matrices X and B can be either dense or sparse
  * 
  * \tparam MatrixType the type of the sparse matrix A, it must be a SparseMatrix<>
  * \tparam UpLo The part of the matrix to use : Lower or Upper. The default is Lower as required by PaStiX
  * 
  * \sa \ref TutorialSparseDirectSolvers
  */
template<typename _MatrixType, int _UpLo>
class PastixLDLT : public PastixBase< PastixLDLT<_MatrixType, _UpLo> >
{
public:
    typedef _MatrixType MatrixType;
    typedef PastixBase<PastixLDLT<MatrixType, _UpLo> > Base; 
    typedef typename MatrixType::Scalar Scalar;
    typedef typename MatrixType::Index Index;
    
  public:
    enum { UpLo = _UpLo };
    PastixLDLT():Base() {}
    
    PastixLDLT(const MatrixType& matrix):Base()
    {
      compute(matrix);
    }

    /** Compute the L and D factors of the LDL^T factorization of \p matrix 
      * \sa analyzePattern() factorize()
      */
    void compute (const MatrixType& matrix)
    {
      Base::PastixInit();
      // Pastix supports only lower, column-major matrices
      SparseMatrix<Scalar, ColMajor> temp(matrix.rows(), matrix.cols());
      PermutationMatrix<Dynamic,Dynamic,Index> pnull;
      temp.template selfadjointView<Lower>() = matrix.template selfadjointView<UpLo>().twistedBy(pnull);
      m_iparm(IPARM_SYM) = API_SYM_YES;
      m_iparm(IPARM_FACTORIZATION) = API_FACT_LDLT;
      Base::compute(temp);
    }

    /** Compute the LDL^T symbolic factorization of \p matrix using its sparsity pattern
      * The result of this operation can be used with successive matrices having the same pattern as \p matrix
      * \sa factorize()
      */
    void analyzePattern(const MatrixType& matrix)
    { 
      Base::PastixInit();
      // Pastix supports only lower, column-major matrices
      SparseMatrix<Scalar, ColMajor> temp(matrix.rows(), matrix.cols());
      PermutationMatrix<Dynamic,Dynamic,Index> pnull;
      temp.template selfadjointView<Lower>() = matrix.template selfadjointView<UpLo>().twistedBy(pnull);
    
      m_iparm(IPARM_SYM) = API_SYM_YES;
      m_iparm(IPARM_FACTORIZATION) = API_FACT_LDLT;
      Base::analyzePattern(temp);
    }
    /** Compute the LDL^T supernodal numerical factorization of \p matrix 
      * 
      */
    void factorize(const MatrixType& matrix)
    {
      // Pastix supports only lower, column-major matrices
      SparseMatrix<Scalar, ColMajor> temp(matrix.rows(), matrix.cols());
      PermutationMatrix<Dynamic,Dynamic,Index> pnull;
      temp.template selfadjointView<Lower>() = matrix.template selfadjointView<UpLo>().twistedBy(pnull);

      m_iparm(IPARM_SYM) = API_SYM_YES;
      m_iparm(IPARM_FACTORIZATION) = API_FACT_LDLT;
      Base::factorize(temp);
    }

  protected:
    using Base::m_iparm;
    
  private:
    PastixLDLT(PastixLDLT& ) {}
};

namespace internal {

template<typename _MatrixType, typename Rhs>
struct solve_retval<PastixBase<_MatrixType>, Rhs>
  : solve_retval_base<PastixBase<_MatrixType>, Rhs>
{
  typedef PastixBase<_MatrixType> Dec;
  EIGEN_MAKE_SOLVE_HELPERS(Dec,Rhs)

  template<typename Dest> void evalTo(Dest& dst) const
  {
    dec()._solve(rhs(),dst);
  }
};

template<typename _MatrixType, typename Rhs>
struct sparse_solve_retval<PastixBase<_MatrixType>, Rhs>
  : sparse_solve_retval_base<PastixBase<_MatrixType>, Rhs>
{
  typedef PastixBase<_MatrixType> Dec;
  EIGEN_MAKE_SPARSE_SOLVE_HELPERS(Dec,Rhs)

  template<typename Dest> void evalTo(Dest& dst) const
  {
    dec()._solve_sparse(rhs(),dst);
  }
};

} // end namespace internal

} // end namespace Eigen

#endif

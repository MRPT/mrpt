/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// -------------------------------------------------------------------------
// Note: This file will be included within the body of Eigen::MatrixBase
// -------------------------------------------------------------------------
public:
	/** @name MRPT plugin: Types
	  *  @{ */
	// size is constant
	enum { static_size = RowsAtCompileTime*ColsAtCompileTime };
	/** @} */


	/** @name MRPT plugin: Basic iterators. These iterators are intended for 1D matrices only, i.e. column or row vectors.
	  *  @{ */
	typedef Scalar* iterator;
	typedef const Scalar* const_iterator;

	EIGEN_STRONG_INLINE iterator begin() { return derived().data(); }
	EIGEN_STRONG_INLINE iterator end()   { return (&(derived().data()[size()-1]))+1; }
	EIGEN_STRONG_INLINE const_iterator begin() const { return derived().data(); }
	EIGEN_STRONG_INLINE const_iterator end() const   { return (&(derived().data()[size()-1]))+1; }

	/** @} */


	/** @name MRPT plugin: Set/get/load/save and other miscelaneous methods
	  *  @{ */

	/*! Fill all the elements with a given value */
	EIGEN_STRONG_INLINE void fill(const Scalar v) { derived().setConstant(v); }

	/*! Fill all the elements with a given value */
	EIGEN_STRONG_INLINE void assign(const Scalar v) { derived().setConstant(v); }
	/*! Resize to N and set all the elements to a given value */
	EIGEN_STRONG_INLINE void assign(size_t N, const Scalar v) { derived().resize(N); derived().setConstant(v); }

	/** Get number of rows */
	EIGEN_STRONG_INLINE size_t getRowCount() const { return rows(); }
	/** Get number of columns */
	EIGEN_STRONG_INLINE size_t getColCount() const { return cols(); }

	/** Make the matrix an identity matrix (the diagonal values can be 1.0 or any other value) */
	EIGEN_STRONG_INLINE void unit(const size_t nRows, const Scalar diag_vals) {
		if (diag_vals==1)
			derived().setIdentity(nRows,nRows);
		else {
			derived().setZero(nRows,nRows);
			derived().diagonal().setConstant(diag_vals);
		}
	}

	/** Make the matrix an identity matrix  */
	EIGEN_STRONG_INLINE void unit() { derived().setIdentity(); }
	/** Make the matrix an identity matrix  */
	EIGEN_STRONG_INLINE void eye() { derived().setIdentity(); }

	/** Set all elements to zero */
	EIGEN_STRONG_INLINE void zeros() { derived().setZero(); }
	/** Resize and set all elements to zero */
	EIGEN_STRONG_INLINE void zeros(const size_t row,const size_t col) { derived().setZero(row,col); }

	/** Resize matrix and set all elements to one */
	EIGEN_STRONG_INLINE void  ones(const size_t row, const size_t col) { derived().setOnes(row,col); }
	/** Set all elements to one */
	EIGEN_STRONG_INLINE void  ones() { derived().setOnes(); }

	/** Fast but unsafe method to obtain a pointer to a given row of the matrix (Use only in time critical applications)
	  * VERY IMPORTANT WARNING: You must be aware of the memory layout, either Column or Row-major ordering.
	  */
	EIGEN_STRONG_INLINE Scalar      * get_unsafe_row(size_t row)       { return &derived().coeffRef(row,0); }
	EIGEN_STRONG_INLINE const Scalar* get_unsafe_row(size_t row) const { return &derived().coeff(row,0); }

	/** Read-only access to one element (Use with caution, bounds are not checked!) */
	EIGEN_STRONG_INLINE Scalar get_unsafe(const size_t row, const size_t col) const {
#ifdef _DEBUG
		return derived()(row,col);
#else
		return derived().coeff(row,col);
#endif
	}
	/** Reference access to one element (Use with caution, bounds are not checked!) */
	EIGEN_STRONG_INLINE Scalar& get_unsafe(const size_t row, const size_t col) { //-V659
#ifdef _DEBUG
		return derived()(row,col);
#else
		return derived().coeffRef(row,col);
#endif
	}
	/** Sets an element  (Use with caution, bounds are not checked!) */
	EIGEN_STRONG_INLINE void set_unsafe(const size_t row, const size_t col, const Scalar val) {
#ifdef _DEBUG
		derived()(row,col) = val;
#else
		derived().coeffRef(row,col) = val;
#endif
	}

	/** Insert an element at the end of the container (for 1D vectors/arrays) */
	EIGEN_STRONG_INLINE void push_back(Scalar val)
	{
		const Index N = size();
		derived().conservativeResize(N+1);
		coeffRef(N) = val;
	}

	EIGEN_STRONG_INLINE bool isSquare() const { return cols()==rows(); }
	EIGEN_STRONG_INLINE bool isSingular(const Scalar absThreshold = 0) const { return std::abs(derived().determinant())<absThreshold; }

	/** Read a matrix from a string in Matlab-like format, for example "[1 0 2; 0 4 -1]"
	  *  The string must start with '[' and end with ']'. Rows are separated by semicolons ';' and
	  *  columns in each row by one or more whitespaces ' ', commas ',' or tabs '\t'.
	  *
	  *  This format is also used for CConfigFile::read_matrix.
	  *
	  *  This template method can be instantiated for matrices of the types: int, long, unsinged int, unsigned long, float, double, long double
	  *
	  * \return true on success. false if the string is malformed, and then the matrix will be resized to 0x0.
	  * \sa inMatlabFormat, CConfigFile::read_matrix
	  */
	bool fromMatlabStringFormat(const std::string &s, std::ostream *dump_errors_here = NULL);
	// Method implemented in eigen_plugins_impl.h

	/** Dump matrix in matlab format.
	  *  This template method can be instantiated for matrices of the types: int, long, unsinged int, unsigned long, float, double, long double
	  * \sa fromMatlabStringFormat
	  */
	std::string  inMatlabFormat(const size_t decimal_digits = 6) const;
	// Method implemented in eigen_plugins_impl.h

	/** Save matrix to a text file, compatible with MATLAB text format (see also the methods of matrix classes themselves).
		* \param theMatrix It can be a CMatrixTemplate or a CMatrixFixedNumeric.
		* \param file The target filename.
		* \param fileFormat See TMatrixTextFileFormat. The format of the numbers in the text file.
		* \param appendMRPTHeader Insert this header to the file "% File generated by MRPT. Load with MATLAB with: VAR=load(FILENAME);"
		* \param userHeader Additional text to be written at the head of the file. Typically MALAB comments "% This file blah blah". Final end-of-line is not needed.
		* \sa loadFromTextFile, CMatrixTemplate::inMatlabFormat, SAVE_MATRIX
		*/
	void saveToTextFile(
		const std::string &file,
		mrpt::math::TMatrixTextFileFormat fileFormat = mrpt::math::MATRIX_FORMAT_ENG,
		bool    appendMRPTHeader = false,
		const std::string &userHeader = std::string()
		) const;
	// Method implemented in eigen_plugins_impl.h

	/** Load matrix from a text file, compatible with MATLAB text format.
	  *  Lines starting with '%' or '#' are interpreted as comments and ignored.
	  * \sa saveToTextFile, fromMatlabStringFormat
	  */
	void  loadFromTextFile(const std::string &file);
	// Method implemented in eigen_plugins_impl.h

	//! \overload
	void  loadFromTextFile(std::istream &_input_text_stream);
	// Method implemented in eigen_plugins_impl.h

	EIGEN_STRONG_INLINE void multiplyColumnByScalar(size_t c, Scalar s) { derived().col(c)*=s; }
	EIGEN_STRONG_INLINE void multiplyRowByScalar(size_t r, Scalar s)    { derived().row(r)*=s; }

	EIGEN_STRONG_INLINE void swapCols(size_t i1,size_t i2) { derived().col(i1).swap( derived().col(i2) ); }
	EIGEN_STRONG_INLINE void swapRows(size_t i1,size_t i2) { derived().row(i1).swap( derived().row(i2) ); }

	EIGEN_STRONG_INLINE size_t countNonZero() const { return ((*static_cast<const Derived*>(this))!= 0).count(); }

	/** [VECTORS OR MATRICES] Finds the maximum value
	  * \exception std::exception On an empty input container
	  */
	EIGEN_STRONG_INLINE Scalar maximum() const
	{
		if (size()==0) throw std::runtime_error("maximum: container is empty");
		return derived().maxCoeff();
	}

	/** [VECTORS OR MATRICES] Finds the minimum value
	  * \sa maximum, minimum_maximum
	  * \exception std::exception On an empty input container */
	EIGEN_STRONG_INLINE Scalar minimum() const
	{
		if (size()==0) throw std::runtime_error("minimum: container is empty");
		return derived().minCoeff();
	}

	/** [VECTORS OR MATRICES] Compute the minimum and maximum of a container at once
	  * \sa maximum, minimum
	  * \exception std::exception On an empty input container */
	EIGEN_STRONG_INLINE void minimum_maximum(
		Scalar & out_min,
		Scalar & out_max) const
	{
		out_min = minimum();
		out_max = maximum();
	}


	/** [VECTORS ONLY] Finds the maximum value (and the corresponding zero-based index) from a given container.
	  * \exception std::exception On an empty input vector
	  */
	EIGEN_STRONG_INLINE Scalar maximum(size_t *maxIndex) const
	{
		if (size()==0) throw std::runtime_error("maximum: container is empty");
		Index idx;
		const Scalar m = derived().maxCoeff(&idx);
		if (maxIndex) *maxIndex = idx;
		return m;
	}

	/** [VECTORS OR MATRICES] Finds the maximum value (and the corresponding zero-based index) from a given container.
	  * \exception std::exception On an empty input vector
	  */
	void find_index_max_value(size_t &u,size_t &v,Scalar &valMax) const
	{
		if (cols()==0 || rows()==0) throw std::runtime_error("find_index_max_value: container is empty");
		Index idx1,idx2;
		valMax = derived().maxCoeff(&idx1,&idx2);
		u = idx1; v = idx2;
	}


	/** [VECTORS ONLY] Finds the minimum value (and the corresponding zero-based index) from a given container.
	  * \sa maximum, minimum_maximum
	  * \exception std::exception On an empty input vector  */
	EIGEN_STRONG_INLINE Scalar minimum(size_t *minIndex) const
	{
		if (size()==0) throw std::runtime_error("minimum: container is empty");
		Index idx;
		const Scalar m =derived().minCoeff(&idx);
		if (minIndex) *minIndex = idx;
		return m;
	}

	/** [VECTORS ONLY] Compute the minimum and maximum of a container at once
	  * \sa maximum, minimum
	  * \exception std::exception On an empty input vector */
	EIGEN_STRONG_INLINE void minimum_maximum(
		Scalar & out_min,
		Scalar & out_max,
		size_t *minIndex,
		size_t *maxIndex) const
	{
		out_min = minimum(minIndex);
		out_max = maximum(maxIndex);
	}

	/** Compute the norm-infinite of a vector ($f[ ||\mathbf{v}||_\infnty $f]), ie the maximum absolute value of the elements. */
	EIGEN_STRONG_INLINE Scalar norm_inf() const { return lpNorm<Eigen::Infinity>(); }

	/** Compute the square norm of a vector/array/matrix (the Euclidean distance to the origin, taking all the elements as a single vector). \sa norm */
	EIGEN_STRONG_INLINE Scalar squareNorm() const { return squaredNorm(); }

	/*! Sum all the elements, returning a value of the same type than the container  */
	EIGEN_STRONG_INLINE Scalar sumAll() const { return derived().sum(); }

	/** Computes the laplacian of this square graph weight matrix.
	  *  The laplacian matrix is L = D - W, with D a diagonal matrix with the degree of each node, W the
	  */
	template<typename OtherDerived>
	EIGEN_STRONG_INLINE void laplacian(Eigen::MatrixBase <OtherDerived>& ret) const
	{
		if (rows()!=cols()) throw std::runtime_error("laplacian: Defined for square matrixes only");
		const Index N = rows();
		ret = -(*this);
		for (Index i=0;i<N;i++)
		{
			Scalar deg = 0;
			for (Index j=0;j<N;j++) deg+= derived().coeff(j,i);
			ret.coeffRef(i,i)+=deg;
		}
	}


	/** Changes the size of matrix, maintaining its previous content as possible and padding with zeros where applicable.
	  *  **WARNING**: MRPT's add-on method \a setSize() pads with zeros, while Eigen's \a resize() does NOT (new elements are undefined).
	  */
	EIGEN_STRONG_INLINE void setSize(size_t row, size_t col)
	{
#ifdef _DEBUG
		if ((Derived::RowsAtCompileTime!=Eigen::Dynamic && Derived::RowsAtCompileTime!=int(row)) || (Derived::ColsAtCompileTime!=Eigen::Dynamic && Derived::ColsAtCompileTime!=int(col))) {
			std::stringstream ss; ss << "setSize: Trying to change a fixed sized matrix from " << rows() << "x" << cols() << " to " << row << "x" << col;
			throw std::runtime_error(ss.str());
		}
#endif
		const Index oldCols = cols();
		const Index oldRows = rows();
		const int nNewCols = int(col) - int(cols());
		const int nNewRows = int(row) - int(rows());
		::mrpt::math::detail::TAuxResizer<Eigen::MatrixBase<Derived>,SizeAtCompileTime>::internal_resize(*this,row,col);
		if (nNewCols>0) derived().block(0,oldCols,row,nNewCols).setZero();
		if (nNewRows>0) derived().block(oldRows,0,nNewRows,col).setZero();
	}

	/** Efficiently computes only the biggest eigenvector of the matrix using the Power Method, and returns it in the passed vector "x". */
	template <class OUTVECT>
	void largestEigenvector(
		OUTVECT &x,
		Scalar resolution = Scalar(0.01),
		size_t maxIterations = 6,
		int    *out_Iterations = NULL,
		float  *out_estimatedResolution = NULL ) const
	{
		// Apply the iterative Power Method:
		size_t iter=0;
		const Index n = rows();
		x.resize(n);
		x.setConstant(1);  // Initially, set to all ones, for example...
		Scalar dif;
		do  // Iterative loop:
		{
			Eigen::Matrix<Scalar,Derived::RowsAtCompileTime,1> xx = (*this) * x;
			xx *= Scalar(1.0/xx.norm());
			dif = (x-xx).array().abs().sum();	// Compute diference between iterations:
			x = xx;	// Set as current estimation:
			iter++; // Iteration counter:
		} while (iter<maxIterations && dif>resolution);
		if (out_Iterations) *out_Iterations=static_cast<int>(iter);
		if (out_estimatedResolution) *out_estimatedResolution=dif;
	}

	/** Combined matrix power and assignment operator */
	MatrixBase<Derived>& operator ^= (const unsigned int pow)
	{
		if (pow==0)
			derived().setIdentity();
		else
		for (unsigned int i=1;i<pow;i++)
			derived() *= derived();
		return *this;
	}

	/** Scalar power of all elements to a given power, this is diferent of ^ operator. */
	EIGEN_STRONG_INLINE void scalarPow(const Scalar s) { (*this)=array().pow(s); }

	/** Checks for matrix type */
	EIGEN_STRONG_INLINE bool isDiagonal() const
	{
		for (Index c=0;c<cols();c++)
			for (Index r=0;r<rows();r++)
				if (r!=c && coeff(r,c)!=0)
					return false;
		return true;
	}

	/** Finds the maximum value in the diagonal of the matrix. */
	EIGEN_STRONG_INLINE Scalar maximumDiagonal() const { return diagonal().maxCoeff(); }

	/** Computes the mean of the entire matrix
	  * \sa meanAndStdAll */
	EIGEN_STRONG_INLINE double mean() const
	{
		if ( size()==0) throw std::runtime_error("mean: Empty container.");
		return derived().sum()/static_cast<double>(size());
	}

	/** Computes a row with the mean values of each column in the matrix and the associated vector with the standard deviation of each column.
	  * \sa mean,meanAndStdAll \exception std::exception If the matrix/vector is empty.
	  * \param unbiased_variance Standard deviation is sum(vals-mean)/K, with K=N-1 or N for unbiased_variance=true or false, respectively.
	  */
	template <class VEC>
	void  meanAndStd(
		VEC &outMeanVector,
		VEC &outStdVector,
		const bool unbiased_variance = true ) const
	{
		const size_t N = rows();
		if (N==0) throw std::runtime_error("meanAndStd: Empty container.");
		const double N_inv = 1.0/N;
		const double N_    = unbiased_variance ? (N>1 ? 1.0/(N-1) : 1.0) : 1.0/N;
		outMeanVector.resize(cols());
		outStdVector.resize(cols());
		for (Index i=0;i<cols();i++)
		{
			outMeanVector[i]= this->col(i).array().sum() * N_inv;
			outStdVector[i] = std::sqrt( (this->col(i).array()-outMeanVector[i]).square().sum() * N_ );
		}
	}

	/** Computes the mean and standard deviation of all the elements in the matrix as a whole.
	  * \sa mean,meanAndStd  \exception std::exception If the matrix/vector is empty.
	  * \param unbiased_variance Standard deviation is sum(vals-mean)/K, with K=N-1 or N for unbiased_variance=true or false, respectively.
	  */
	void  meanAndStdAll(
		double &outMean,
		double &outStd,
		const bool unbiased_variance = true )  const
	{
		const size_t N = size();
		if (N==0) throw std::runtime_error("meanAndStdAll: Empty container.");
		const double N_ = unbiased_variance ? (N>1 ? 1.0/(N-1) : 1.0) : 1.0/N;
		outMean = derived().array().sum()/static_cast<double>(size());
		outStd  = std::sqrt( (this->array() - outMean).square().sum()*N_);
	}

	/** Insert matrix "m" into this matrix at indices (r,c), that is, (*this)(r,c)=m(0,0) and so on */
	template <typename MAT>
	EIGEN_STRONG_INLINE void insertMatrix(size_t r,size_t c, const MAT &m) { derived().block(r,c,m.rows(),m.cols())=m; }

	template <typename MAT>
	EIGEN_STRONG_INLINE void insertMatrixTranspose(size_t r,size_t c, const MAT &m) { derived().block(r,c,m.cols(),m.rows())=m.adjoint(); }

	template <typename MAT> EIGEN_STRONG_INLINE void insertRow(size_t nRow, const MAT & aRow) { this->row(nRow) = aRow; }
	template <typename MAT> EIGEN_STRONG_INLINE void insertCol(size_t nCol, const MAT & aCol) { this->col(nCol) = aCol; }

	template <typename R> void insertRow(size_t nRow, const std::vector<R> & aRow)
	{
		if (static_cast<Index>(aRow.size())!=cols()) throw std::runtime_error("insertRow: Row size doesn't fit the size of this matrix.");
		for (Index j=0;j<cols();j++)
			coeffRef(nRow,j) = aRow[j];
	}
	template <typename R> void insertCol(size_t nCol, const std::vector<R> & aCol)
	{
		if (static_cast<Index>(aCol.size())!=rows()) throw std::runtime_error("insertRow: Row size doesn't fit the size of this matrix.");
		for (Index j=0;j<rows();j++)
			coeffRef(j,nCol) = aCol[j];
	}

	/** Remove columns of the matrix.*/
	EIGEN_STRONG_INLINE void removeColumns(const std::vector<size_t> &idxsToRemove)
	{
		std::vector<size_t> idxs = idxsToRemove;
		std::sort( idxs.begin(), idxs.end() );
		std::vector<size_t>::iterator itEnd = std::unique( idxs.begin(), idxs.end() );
		idxs.resize( itEnd - idxs.begin() );

		unsafeRemoveColumns( idxs );
	}

	/** Remove columns of the matrix. The unsafe version assumes that, the indices are sorted in ascending order. */
	EIGEN_STRONG_INLINE void unsafeRemoveColumns(const std::vector<size_t> &idxs)
	{
		size_t k = 1;
		for (std::vector<size_t>::const_reverse_iterator it = idxs.rbegin(); it != idxs.rend(); ++it, ++k)
		{
			const size_t nC = cols() - *it - k;
			if( nC > 0 )
				derived().block(0,*it,rows(),nC) = derived().block(0,*it+1,rows(),nC).eval();
		}
		derived().conservativeResize(NoChange,cols()-idxs.size());
	}

	/** Remove rows of the matrix. */
	EIGEN_STRONG_INLINE void removeRows(const std::vector<size_t> &idxsToRemove)
	{
		std::vector<size_t> idxs = idxsToRemove;
		std::sort( idxs.begin(), idxs.end() );
		std::vector<size_t>::iterator itEnd = std::unique( idxs.begin(), idxs.end() );
		idxs.resize( itEnd - idxs.begin() );

		unsafeRemoveRows( idxs );
	}

	/** Remove rows of the matrix. The unsafe version assumes that, the indices are sorted in ascending order. */
	EIGEN_STRONG_INLINE void unsafeRemoveRows(const std::vector<size_t> &idxs)
	{
		size_t k = 1;
		for (std::vector<size_t>::reverse_iterator it = idxs.rbegin(); it != idxs.rend(); ++it, ++k)
		{
			const size_t nR = rows() - *it - k;
			if( nR > 0 )
				derived().block(*it,0,nR,cols()) = derived().block(*it+1,0,nR,cols()).eval();
		}
		derived().conservativeResize(rows()-idxs.size(),NoChange);
	}

	/** Transpose */
	EIGEN_STRONG_INLINE const AdjointReturnType t() const { return derived().adjoint(); }

	EIGEN_STRONG_INLINE PlainObject inv() const { PlainObject outMat = derived().inverse().eval(); return outMat; }
	template <class MATRIX> EIGEN_STRONG_INLINE void inv(MATRIX &outMat) const { outMat = derived().inverse().eval(); }
	template <class MATRIX> EIGEN_STRONG_INLINE void inv_fast(MATRIX &outMat) const { outMat = derived().inverse().eval(); }
	EIGEN_STRONG_INLINE Scalar det() const { return derived().determinant(); }

	/** @} */  // end miscelaneous


	/** @name MRPT plugin: Multiply and extra addition functions
		@{ */

	EIGEN_STRONG_INLINE bool empty() const { return this->getColCount()==0 || this->getRowCount()==0; }

	/*! Add c (scalar) times A to this matrix: this += A * c  */
	template<typename OTHERMATRIX> EIGEN_STRONG_INLINE void add_Ac(const OTHERMATRIX &m,const Scalar c)	{ (*this)+=c*m; }
	/*! Substract c (scalar) times A to this matrix: this -= A * c  */
	template<typename OTHERMATRIX> EIGEN_STRONG_INLINE void substract_Ac(const OTHERMATRIX &m,const Scalar c)	{ (*this) -= c*m; }

	/*! Substract A transposed to this matrix: this -= A.adjoint() */
	template<typename OTHERMATRIX> EIGEN_STRONG_INLINE void substract_At(const OTHERMATRIX &m)	{ (*this) -= m.adjoint(); }

	/*! Substract n (integer) times A to this matrix: this -= A * n  */
	template<typename OTHERMATRIX> EIGEN_STRONG_INLINE void substract_An(const OTHERMATRIX& m, const size_t n)	{ this->noalias() -= n * m; }

	/*! this += A + A<sup>T</sup>  */
	template<typename OTHERMATRIX> EIGEN_STRONG_INLINE void add_AAt(const OTHERMATRIX &A) { this->noalias() += A; this->noalias() += A.adjoint(); }

	/*! this -= A + A<sup>T</sup>  */ \
	template<typename OTHERMATRIX> EIGEN_STRONG_INLINE void substract_AAt(const OTHERMATRIX &A)	{ this->noalias() -= A; this->noalias() -= A.adjoint(); }


	template <class MATRIX1,class MATRIX2> EIGEN_STRONG_INLINE void multiply( const MATRIX1& A, const MATRIX2 &B ) /*!< this = A * B */ { (*this)= A*B; }

	template <class MATRIX1,class MATRIX2>
	EIGEN_STRONG_INLINE void multiply_AB( const MATRIX1& A, const MATRIX2 &B ) /*!< this = A * B */ {
		(*this)= A*B;
	}

	template <typename MATRIX1,typename MATRIX2>
	EIGEN_STRONG_INLINE void multiply_AtB(const MATRIX1 &A,const MATRIX2 &B) /*!< this=A^t * B */ {
		*this = A.adjoint() * B;
	}

	/*! Computes the vector vOut = this * vIn, where "vIn" is a column vector of the appropriate length. */
	template<typename OTHERVECTOR1,typename OTHERVECTOR2>
	EIGEN_STRONG_INLINE void multiply_Ab(const OTHERVECTOR1 &vIn,OTHERVECTOR2 &vOut,bool accumToOutput = false) const {
		if (accumToOutput) vOut.noalias() += (*this) * vIn;
		else vOut = (*this) * vIn;
	}

	/*! Computes the vector vOut = this<sup>T</sup> * vIn, where "vIn" is a column vector of the appropriate length. */ \
	template<typename OTHERVECTOR1,typename OTHERVECTOR2>
	EIGEN_STRONG_INLINE void multiply_Atb(const OTHERVECTOR1 &vIn,OTHERVECTOR2 &vOut,bool accumToOutput = false) const {
		if (accumToOutput) vOut.noalias() += this->adjoint() * vIn;
		else vOut = this->adjoint() * vIn;
	}

	template <typename MAT_C, typename MAT_R>
	EIGEN_STRONG_INLINE void multiply_HCHt(const MAT_C &C,MAT_R &R,bool accumResultInOutput=false) const /*!< R = this * C * this<sup>T</sup>  */ {
		if (accumResultInOutput)
		      R.noalias() += (*this) * C * this->adjoint();
		else  R.noalias()  = (*this) * C * this->adjoint();
	}

	template <typename MAT_C, typename MAT_R>
	EIGEN_STRONG_INLINE void multiply_HtCH(const MAT_C &C,MAT_R &R,bool accumResultInOutput=false) const /*!< R = this<sup>T</sup> * C * this  */ {
		if (accumResultInOutput)
		      R.noalias() += this->adjoint() * C * (*this);
		else  R.noalias()  = this->adjoint() * C * (*this);
	}

	/*! R = H * C * H<sup>T</sup> (with a vector H and a symmetric matrix C) In fact when H is a vector, multiply_HCHt_scalar and multiply_HtCH_scalar are exactly equivalent */
	template <typename MAT_C>
	EIGEN_STRONG_INLINE Scalar multiply_HCHt_scalar(const MAT_C &C) const {
		return ( (*this) * C * this->adjoint() ).eval()(0,0);
	}

	/*! R = H<sup>T</sup> * C * H (with a vector H and a symmetric matrix C) In fact when H is a vector, multiply_HCHt_scalar and multiply_HtCH_scalar are exactly equivalent */
	template <typename MAT_C>
	EIGEN_STRONG_INLINE Scalar multiply_HtCH_scalar(const MAT_C &C) const {
		return ( this->adjoint() * C * (*this) ).eval()(0,0);
	}

	/*! this = C * C<sup>T</sup> * f (with a matrix C and a scalar f). */
	template<typename MAT_A>
	EIGEN_STRONG_INLINE void multiply_AAt_scalar(const MAT_A &A,typename MAT_A::Scalar f)	{
		*this = (A * A.adjoint()) * f;
	}

	/*! this = C<sup>T</sup> * C * f (with a matrix C and a scalar f). */
	template<typename MAT_A> EIGEN_STRONG_INLINE void multiply_AtA_scalar(const MAT_A &A,typename MAT_A::Scalar f)	{
		*this = (A.adjoint() * A) * f;
	}

	/*! this = A * skew(v), with \a v being a 3-vector (or 3-array) and skew(v) the skew symmetric matrix of v (see mrpt::math::skew_symmetric3) */
	template <class MAT_A,class SKEW_3VECTOR> void multiply_A_skew3(const MAT_A &A,const SKEW_3VECTOR &v) {
		mrpt::math::multiply_A_skew3(A,v,*this); }

	/*! this = skew(v)*A, with \a v being a 3-vector (or 3-array) and skew(v) the skew symmetric matrix of v (see mrpt::math::skew_symmetric3) */
	template <class SKEW_3VECTOR,class MAT_A> void multiply_skew3_A(const SKEW_3VECTOR &v,const MAT_A &A) {
		mrpt::math::multiply_skew3_A(v,A,*this); }

	/** outResult = this * A
	  */
	template <class MAT_A,class MAT_OUT>
	EIGEN_STRONG_INLINE void multiply_subMatrix(const MAT_A &A,MAT_OUT &outResult,const size_t A_cols_offset,const size_t A_rows_offset,const size_t A_col_count) const  {
		outResult = derived() * A.block(A_rows_offset,A_cols_offset,derived().cols(),A_col_count);
	}

	template <class MAT_A,class MAT_B,class MAT_C>
	void multiply_ABC(const MAT_A &A, const MAT_B &B, const MAT_C &C) /*!< this = A*B*C  */ {
		*this = A*B*C;
	}

	template <class MAT_A,class MAT_B,class MAT_C>
	void multiply_ABCt(const MAT_A &A, const MAT_B &B, const MAT_C &C) /*!< this = A*B*(C<sup>T</sup>) */ {
		*this = A*B*C.adjoint();
	}

	template <class MAT_A,class MAT_B,class MAT_C>
	void multiply_AtBC(const MAT_A &A, const MAT_B &B, const MAT_C &C) /*!< this = A(<sup>T</sup>)*B*C */ {
		*this = A.adjoint()*B*C;
	}

	template <class MAT_A,class MAT_B>
	EIGEN_STRONG_INLINE void multiply_ABt(const MAT_A &A,const MAT_B &B) /*!< this = A * B<sup>T</sup> */ {
		*this = A*B.adjoint();
	}

	template <class MAT_A>
	EIGEN_STRONG_INLINE void multiply_AAt(const MAT_A &A) /*!< this = A * A<sup>T</sup> */ {
		*this = A*A.adjoint();
	}

	template <class MAT_A>
	EIGEN_STRONG_INLINE void multiply_AtA(const MAT_A &A) /*!< this = A<sup>T</sup> * A */ {
		*this = A.adjoint()*A;
	}

	template <class MAT_A,class MAT_B>
	EIGEN_STRONG_INLINE void multiply_result_is_symmetric(const MAT_A &A,const MAT_B &B) /*!< this = A * B (result is symmetric) */ {
		*this = A*B;
	}

	/** @} */  // end multiply functions


	/** @name MRPT plugin: Eigenvalue / Eigenvectors
	    @{  */

	/** [For square matrices only] Compute the eigenvectors and eigenvalues (sorted), both returned as matrices: eigenvectors are the columns in "eVecs", and eigenvalues in ascending order as the diagonal of "eVals".
	  *   \note Warning: Only the real part of complex eigenvectors and eigenvalues are returned.
	  *   \sa eigenVectorsSymmetric, eigenVectorsVec
	  *  \return false on error
	  */
	template <class MATRIX1,class MATRIX2>
	EIGEN_STRONG_INLINE bool eigenVectors( MATRIX1 & eVecs, MATRIX2 & eVals ) const;
	// Implemented in eigen_plugins_impl.h (can't be here since Eigen::SelfAdjointEigenSolver isn't defined yet at this point.

	/** [For square matrices only] Compute the eigenvectors and eigenvalues (sorted), eigenvectors are the columns in "eVecs", and eigenvalues are returned in in ascending order in the vector "eVals".
	  *   \note Warning: Only the real part of complex eigenvectors and eigenvalues are returned.
	  *   \sa eigenVectorsSymmetric, eigenVectorsVec
	  *  \return false on error
	  */
	template <class MATRIX1,class VECTOR1>
	EIGEN_STRONG_INLINE bool eigenVectorsVec( MATRIX1 & eVecs, VECTOR1 & eVals ) const;
	// Implemented in eigen_plugins_impl.h

	/** [For square matrices only] Compute the eigenvectors and eigenvalues (sorted), and return only the eigenvalues in the vector "eVals".
	  *   \note Warning: Only the real part of complex eigenvectors and eigenvalues are returned.
	  *   \sa eigenVectorsSymmetric, eigenVectorsVec
	  */
	template <class VECTOR>
	EIGEN_STRONG_INLINE void eigenValues( VECTOR & eVals ) const
	{
		PlainObject eVecs;
		eVecs.resizeLike(*this);
		this->eigenVectorsVec(eVecs,eVals);
	}

	/** [For symmetric matrices only] Compute the eigenvectors and eigenvalues (in no particular order), both returned as matrices: eigenvectors are the columns, and eigenvalues \sa eigenVectors
	  */
	template <class MATRIX1,class MATRIX2>
	EIGEN_STRONG_INLINE void eigenVectorsSymmetric( MATRIX1 & eVecs, MATRIX2 & eVals ) const;
	// Implemented in eigen_plugins_impl.h (can't be here since Eigen::SelfAdjointEigenSolver isn't defined yet at this point.

	/** [For symmetric matrices only] Compute the eigenvectors and eigenvalues (in no particular order), both returned as matrices: eigenvectors are the columns, and eigenvalues \sa eigenVectorsVec
	  */
	template <class MATRIX1,class VECTOR1>
	EIGEN_STRONG_INLINE void eigenVectorsSymmetricVec( MATRIX1 & eVecs, VECTOR1 & eVals ) const;
	// Implemented in eigen_plugins_impl.h


	/** @} */  // end eigenvalues



	/** @name MRPT plugin: Linear algebra & decomposition-based methods
	    @{ */

	/** Cholesky M=U<sup>T</sup> * U decomposition for simetric matrix (upper-half of the matrix will be actually ignored) */
	template <class MATRIX> EIGEN_STRONG_INLINE bool chol(MATRIX &U) const
	{
		Eigen::LLT<PlainObject> Chol = derived().template selfadjointView<Eigen::Lower>().llt();
		if (Chol.info()==Eigen::NoConvergence)
			return false;
		U = PlainObject(Chol.matrixU());
		return true;
	}

	/** Gets the rank of the matrix via the Eigen::ColPivHouseholderQR method
	  * \param threshold If set to >0, it's used as threshold instead of Eigen's default one.
	  */
	EIGEN_STRONG_INLINE size_t rank(double threshold=0) const
	{
		Eigen::ColPivHouseholderQR<PlainObject> QR = this->colPivHouseholderQr();
		if (threshold>0) QR.setThreshold(threshold);
		return QR.rank();
	}

	/** @} */   // end linear algebra



	/** @name MRPT plugin: Scalar and element-wise extra operators
	    @{ */

	/** Scales all elements such as the minimum & maximum values are shifted to the given values */
	void normalize(Scalar valMin, Scalar valMax)
	{
		if (size()==0) return;
		Scalar curMin,curMax;
		minimum_maximum(curMin,curMax);
		Scalar minMaxDelta = curMax - curMin;
		if (minMaxDelta==0) minMaxDelta = 1;
		const Scalar minMaxDelta_ = (valMax-valMin)/minMaxDelta;
		this->array() = (this->array()-curMin)*minMaxDelta_+valMin;
	}
	//! \overload
	inline void adjustRange(Scalar valMin, Scalar valMax) { normalize(valMin,valMax); }

	/** @} */  // end Scalar


	/** Extract one row from the matrix into a row vector */
	template <class OtherDerived> EIGEN_STRONG_INLINE void extractRow(size_t nRow, Eigen::EigenBase<OtherDerived> &v, size_t startingCol = 0) const {
		v = derived().block(nRow,startingCol,1,cols()-startingCol);
	}
	//! \overload
	template <typename T> inline void extractRow(size_t nRow, std::vector<T> &v, size_t startingCol = 0) const {
		const size_t N = cols()-startingCol;
		v.resize(N);
		for (size_t i=0;i<N;i++) v[i]=(*this)(nRow,startingCol+i);
	}
	/** Extract one row from the matrix into a column vector */
	template <class VECTOR> EIGEN_STRONG_INLINE void extractRowAsCol(size_t nRow, VECTOR &v, size_t startingCol = 0) const
	{
		v = derived().adjoint().block(startingCol,nRow,cols()-startingCol,1);
	}


	/** Extract one column from the matrix into a column vector */
	template <class VECTOR> EIGEN_STRONG_INLINE void extractCol(size_t nCol, VECTOR &v, size_t startingRow = 0) const {
		v = derived().block(startingRow,nCol,rows()-startingRow,1);
	}
	//! \overload
	template <typename T> inline void extractCol(size_t nCol, std::vector<T> &v, size_t startingRow = 0) const {
		const size_t N = rows()-startingRow;
		v.resize(N);
		for (size_t i=0;i<N;i++) v[i]=(*this)(startingRow+i,nCol);
	}

	template <class MATRIX> EIGEN_STRONG_INLINE void extractMatrix(const size_t firstRow, const size_t firstCol, MATRIX &m) const
	{
		m = derived().block(firstRow,firstCol,m.rows(),m.cols());
	}
	template <class MATRIX> EIGEN_STRONG_INLINE void extractMatrix(const size_t firstRow, const size_t firstCol, const size_t nRows, const size_t nCols, MATRIX &m) const
	{
		m.resize(nRows,nCols);
		m = derived().block(firstRow,firstCol,nRows,nCols);
	}

	/** Get a submatrix, given its bounds: first & last column and row (inclusive). */
	template <class MATRIX>
	EIGEN_STRONG_INLINE void extractSubmatrix(const size_t row_first,const size_t row_last,const size_t col_first,const size_t col_last,MATRIX &out) const
	{
		out.resize(row_last-row_first+1,col_last-col_first+1);
		out = derived().block(row_first,col_first,row_last-row_first+1,col_last-col_first+1);
	}

	/** Get a submatrix from a square matrix, by collecting the elements M(idxs,idxs), where idxs is a sequence {block_indices(i):block_indices(i)+block_size-1} for all "i" up to the size of block_indices.
	  *  A perfect application of this method is in extracting covariance matrices of a subset of variables from the full covariance matrix.
	  * \sa extractSubmatrix, extractSubmatrixSymmetrical
	  */
	template <class MATRIX>
	void extractSubmatrixSymmetricalBlocks(
		const size_t 			block_size,
		const std::vector<size_t>  	&block_indices,
		MATRIX& out) const
	{
		if (block_size<1) throw std::runtime_error("extractSubmatrixSymmetricalBlocks: block_size must be >=1");
		if (cols()!=rows()) throw std::runtime_error("extractSubmatrixSymmetricalBlocks: Matrix is not square.");

		const size_t N = block_indices.size();
		const size_t nrows_out=N*block_size;
		out.resize(nrows_out,nrows_out);
		if (!N) return; // Done
		for (size_t dst_row_blk=0;dst_row_blk<N; ++dst_row_blk )
		{
			for (size_t dst_col_blk=0;dst_col_blk<N; ++dst_col_blk )
			{
#if defined(_DEBUG)
				if (block_indices[dst_col_blk]*block_size + block_size-1>=size_t(cols())) throw std::runtime_error("extractSubmatrixSymmetricalBlocks: Indices out of range!");
#endif
				out.block(dst_row_blk * block_size,dst_col_blk * block_size, block_size,block_size)
				=
				derived().block(block_indices[dst_row_blk] * block_size, block_indices[dst_col_blk] * block_size, block_size,block_size);
			}
		}
	}


	/** Get a submatrix from a square matrix, by collecting the elements M(idxs,idxs), where idxs is the sequence of indices passed as argument.
	  *  A perfect application of this method is in extracting covariance matrices of a subset of variables from the full covariance matrix.
	  * \sa extractSubmatrix, extractSubmatrixSymmetricalBlocks
	  */
	template <class MATRIX>
	void extractSubmatrixSymmetrical(
		const std::vector<size_t>  	&indices,
		MATRIX& out) const
	{
		if (cols()!=rows()) throw std::runtime_error("extractSubmatrixSymmetrical: Matrix is not square.");

		const size_t N = indices.size();
		const size_t nrows_out=N;
		out.resize(nrows_out,nrows_out);
		if (!N) return; // Done
		for (size_t dst_row_blk=0;dst_row_blk<N; ++dst_row_blk )
			for (size_t dst_col_blk=0;dst_col_blk<N; ++dst_col_blk )
				out.coeffRef(dst_row_blk,dst_col_blk) = this->coeff(indices[dst_row_blk],indices[dst_col_blk]);
	}


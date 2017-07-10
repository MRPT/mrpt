/////////////////////////////////////////////////////////////////////////////
// Name:        matrix2d.h
// Author:      John Labenski
// Created:     07/01/02
// Copyright:   John Labenski, 2002
// License:     wxWidgets v2
/////////////////////////////////////////////////////////////////////////////

#ifndef __WXTHINGS_MATRIX2D_H__
#define __WXTHINGS_MATRIX2D_H__

#include "wx/things/thingdef.h"
#include "wx/object.h"
#include "wx/arrstr.h"
#include "wx/gdicmn.h" // for wxSize

class WXDLLEXPORT wxArrayInt;

//----------------------------------------------------------------------------
//  wxMatrix2D - a generic double valued 2D ref counted matrix class
//----------------------------------------------------------------------------
class wxMatrix2D : public wxObject
{
public:
    wxMatrix2D() : wxObject() {}
    wxMatrix2D(const wxMatrix2D &matrix, bool full_copy = false) : wxObject() { Create(matrix, full_copy); }
    wxMatrix2D(int width, int height, bool zero = true) : wxObject() { Create(width, height, zero); }
    wxMatrix2D(int width, int height, const double *data) : wxObject() { Create(width, height, data); }
    wxMatrix2D(int width, int height, double *data, bool static_data) : wxObject() { Create(width, height, data, static_data); }

    virtual ~wxMatrix2D();

    // Create a wxMatrix2D by Ref(ing) or doing a full unrefed copy after first
    //  unrefing the internal data
    bool Create(const wxMatrix2D &matrix, bool full_copy = false);
    // Create an empty matrix and zero it if zero == true
    bool Create(int width, int height, bool zero = true);
    // Create a matrix by memcpy(ing) the input data
    bool Create(int width, int height, const double *matrix);
    // Create a matrix by using malloc(ed) data and free(ing) it when done if
    //   static_data == true. If static_data == false then the data must exist
    //   for the life of this class.
    bool Create(int width, int height, double *matrix, bool static_data);

    // Create the matrix as a square identity matrix with ones along the diagonal.
    bool CreateIdentity(int size);

    // Make a full copy, not just a refed one
    bool Copy( const wxMatrix2D &matrix );

    // destroy the refed data, reducing the ref count by 1
    void Destroy();
    // Is this matrix created and has a valid width and height
    bool Ok() const;

    // Check if the matrix is exactly equal to this one using memcmp, either
    //  matrix may not be Ok() and be of different sizes, returning false.
    //  Note: the equality operator only checks if the ref data is the same.
    bool IsEqual(const wxMatrix2D& matrix) const;

    // Get the size of the matrix, width = # cols, height = # rows
    int GetWidth() const;
    int GetHeight() const;
    wxSize GetSize() const;
    bool PointInMatrix( int x, int y ) const;

    // Get/Set the value of a matrix element, zero based indexes
    double GetValue(int x, int y) const;
    void SetValue( int x, int y, double value );

    // Insert matrix at this row position, matricies must have same number of cols
    wxMatrix2D InsertRows(int row, const wxMatrix2D& matrix) const;
    // Insert matrix at this col position, matricies must have same number of rows
    wxMatrix2D InsertCols(int col, const wxMatrix2D& matrix) const;
    // Append new rows to the matrix, matricies must have same number of cols
    wxMatrix2D AppendRows(const wxMatrix2D& matrix) const { return InsertRows(-1, matrix); }
    // Append new cols to the matrix, matricies must have same number of rows
    wxMatrix2D AppendCols(const wxMatrix2D& matrix) const { return InsertCols(-1, matrix); }

    // Get a sub matrix inside of the this one, returns a new matrix
    wxMatrix2D SubMatrix(const wxRect& rect) const;
    // Get a sub matrix of rows inside of the this one, returns a new matrix
    wxMatrix2D SubRows(int start_row, int end_row) const;
    // Get a sub matrix of cols inside of the this one, returns a new matrix
    wxMatrix2D SubCols(int start_col, int end_col) const;
    // Reshape the matrix by setting a new width and height. The data is not
    //  changed. Internally the data is stored as a linear array, such that
    //  element = x + y * width.
    bool Reshape(int width, int height);

    // Get the data array as a pointer of size width*height
    double* GetData() const;
    // Set the values for all elements to the given one
    bool Fill( double value );

    // Add this matrix to another matrix element by element, must have same dimensions
    wxMatrix2D Add(const wxMatrix2D &b) const;
    // Add this single value to all elements, returns a new matrix with the result
    wxMatrix2D Add(double scalar) const;
    // Matrix multiplication, returns a new matrix with the result
    wxMatrix2D Mult(const wxMatrix2D &b) const;
    // Hadamard matrix multiplication, element-wise multiplication, returns a new matrix
    wxMatrix2D MultElement(const wxMatrix2D &b) const;
    // Multiply each element of the matrix by this value, returns a new matrix
    wxMatrix2D Mult(double scalar) const;
    // Raise each element of the matrix to this power, returns a new matrix
    wxMatrix2D Pow(double p) const;
    // Raise each element of the matrix to n^(element value), returns a new matrix
    wxMatrix2D PowN(double n) const;
    // Take the natural log of each element of the matrix, returns a new matrix
    wxMatrix2D Log() const;
    // Take the log base 10 of each element of the matrix, returns a new matrix
    wxMatrix2D Log10() const;
    // Set each element to e^(element value), returns a new matrix
    wxMatrix2D Exp() const;
    // Set each element to 10^(element value), returns a new matrix
    //    see PowN()
    //wxMatrix2D Exp10() const;

    // Get the sum of all the elements in the matrix
    double Sum() const;
    // Get the trace of the matrix, the sum of the diagonal elements. The
    //   matrix should be square, but this just adds diagnals from upper left
    //   to lower right.
    double Trace() const;

    // Swap the rows and cols, returns a new matrix
    wxMatrix2D Transpose() const;
    // Rotate the matrix by clockwise*(90 degrees), returns a new matrix
    //  clockwise can be negative to rotate counter-clockwise
    wxMatrix2D Rotate90( int clockwise ) const;
    // Mirror the matrix either horizontally or vertically, returns a new matrix
    wxMatrix2D Mirror( bool horizontally = true ) const;

    // rotate matrix by 45 deg, must be square and sides odd, returns a new matrix
    wxMatrix2D RotateSquare45( bool clockwise ) const;

    // normalize the sum of the values to this
    void Normalize( double sum = 1.0 );

    // print to a string, separating cols and rows by given strings
    wxString ToString(const wxString& colSep = wxT(" "),
                      const wxString& rowSep = wxT("\n")) const;

    // Load a file from disk
    bool LoadFile( const wxString &filename, const wxArrayInt *cols = nullptr );
    wxArrayString m_file_comments;

    friend wxMatrix2D operator+(wxMatrix2D &a, wxMatrix2D &b) { return a.Add(b); }
    friend wxMatrix2D operator+(double scalar, wxMatrix2D &b) { return b.Add(scalar); }
    friend wxMatrix2D operator*(wxMatrix2D &a, wxMatrix2D &b) { return a.Mult(b); }
    friend wxMatrix2D operator*(double scalar, wxMatrix2D &a) { return a.Mult(scalar); }

    wxMatrix2D& operator = (const wxMatrix2D& matrix)
    {
        if ( (*this) != matrix ) wxObject::Ref(matrix);
        return *this;
    }

    bool operator == (const wxMatrix2D& matrix)
    {
        if (!Ok() || !matrix.Ok()) return false;
        return GetData() == matrix.GetData();
    }
    bool operator != (const wxMatrix2D& matrix) { return !(*this == matrix); }

private:
    DECLARE_DYNAMIC_CLASS(wxMatrix2D)
};

#endif // __WXTHINGS_MATRIX2D_H__

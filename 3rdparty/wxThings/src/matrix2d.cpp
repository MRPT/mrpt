/////////////////////////////////////////////////////////////////////////////
// Name:        matrix2d.cpp
// Author:      John Labenski
// Created:     07/01/02
// Copyright:   John Labenski, 2002
// License:     wxWidgets v2
/////////////////////////////////////////////////////////////////////////////

// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

#include "wx/things/matrix2d.h"

#ifndef WX_PRECOMP
    #include "wx/object.h"
    #include "wx/string.h"
    #include "wx/dynarray.h"    // for wxArrayInt
    #include "wx/utils.h"
    #include "wx/msgdlg.h"
#endif

#include "wx/file.h"
#include "wx/wfstream.h"
#include "wx/txtstrm.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

//----------------------------------------------------------------------------
//  wxMatrix2DRefData
//----------------------------------------------------------------------------

class wxMatrix2DRefData : public wxObjectRefData
{
public:
    wxMatrix2DRefData();
    ~wxMatrix2DRefData();

    int     m_width;
    int     m_height;
    double *m_data;
    bool    m_static;
};

wxMatrix2DRefData::wxMatrix2DRefData()
{
    m_width  = 0;
    m_height = 0;
    m_data   = (double*)nullptr;
    m_static = false;
}

wxMatrix2DRefData::~wxMatrix2DRefData()
{
    if (m_data && !m_static) free(m_data);
}

#define M_MATRIXDATA ((wxMatrix2DRefData*)m_refData)

//----------------------------------------------------------------------------
//  wxMatrix2D
//----------------------------------------------------------------------------
IMPLEMENT_DYNAMIC_CLASS( wxMatrix2D, wxObject )

wxMatrix2D::~wxMatrix2D()
{
    // let the MatrixRefData destroy itself
}

bool wxMatrix2D::Create(const wxMatrix2D &source, bool full_copy)
{
    wxCHECK_MSG( source.Ok() && (&source != this), false, wxT("Invalid matrix") );

    UnRef();
    if (full_copy)
        Copy(source);
    else
        Ref(source);

    return Ok();
}

bool wxMatrix2D::Create(int width, int height, bool zero)
{
    wxCHECK_MSG((width > 0) && (height > 0), false, wxT("Invalid matrix size"));

    UnRef();
    m_refData = new wxMatrix2DRefData();

    M_MATRIXDATA->m_data = (double*)malloc( width*height*sizeof(double) );
    if (M_MATRIXDATA->m_data)
    {
        M_MATRIXDATA->m_width  = width;
        M_MATRIXDATA->m_height = height;

        if (zero)
            memset(M_MATRIXDATA->m_data, 0, width*height*sizeof(double));

        return true;
    }

    UnRef();
    return false;
}

bool wxMatrix2D::Create(int width, int height, const double *data)
{
    wxCHECK_MSG((width > 0)&&(height > 0)&&(data != (double*)nullptr), false,
                wxT("Invalid wxMatrix2D size or data is NULL"));

    UnRef();

    m_refData = new wxMatrix2DRefData();

    M_MATRIXDATA->m_data = (double*)malloc( width*height*sizeof(double) );

    if (M_MATRIXDATA->m_data)
    {
        memcpy( M_MATRIXDATA->m_data, data, width*height*sizeof(double) );
        M_MATRIXDATA->m_width = width;
        M_MATRIXDATA->m_height = height;
        return true;
    }

    UnRef();
    return false;
}

bool wxMatrix2D::Create(int width, int height, double *data, bool static_data)
{
    wxCHECK_MSG((width > 0)&&(height > 0)&&(data != (double*)nullptr), false,
                wxT("Invalid wxMatrix2D size or data is NULL"));

    UnRef();

    m_refData = new wxMatrix2DRefData();
    M_MATRIXDATA->m_data = data;

    if (M_MATRIXDATA->m_data)
    {
        M_MATRIXDATA->m_width  = width;
        M_MATRIXDATA->m_height = height;
        M_MATRIXDATA->m_static = static_data;
        return true;
    }

    UnRef();
    return false;
}

bool wxMatrix2D::CreateIdentity(int size)
{
    wxCHECK_MSG(size > 0, false, wxT("Invalid matrix size"));
    if (!Create(size, size, true)) return false;

    double* data = M_MATRIXDATA->m_data;
    for (int i = 0; i < size; ++i) data[i + i*size] = 1.0;

    return true;
}

bool wxMatrix2D::Copy( const wxMatrix2D &matrix )
{
    wxCHECK_MSG(matrix.Ok(), false, wxT("Invalid matrix"));

    if (!Ok() || (M_MATRIXDATA->m_width  != matrix.GetWidth()) ||
                 (M_MATRIXDATA->m_height != matrix.GetHeight()))
    {
        Create(matrix.GetWidth(), matrix.GetHeight());
        if (!Ok()) return false;
    }

    memcpy( M_MATRIXDATA->m_data, matrix.GetData(),
            M_MATRIXDATA->m_width*M_MATRIXDATA->m_height*sizeof(double) );

    return true;
}

void wxMatrix2D::Destroy()
{
    UnRef();
}

bool wxMatrix2D::Ok() const
{
    return M_MATRIXDATA != nullptr;
}

bool wxMatrix2D::IsEqual(const wxMatrix2D& matrix) const
{
    if (!Ok() || !matrix.Ok()) return false;
    if (GetSize() != matrix.GetSize()) return false;

    return memcmp(M_MATRIXDATA->m_data, matrix.GetData(),
                  M_MATRIXDATA->m_width*M_MATRIXDATA->m_height*sizeof(double)) == 0;
}

int wxMatrix2D::GetWidth() const
{
    wxCHECK_MSG( Ok(), 0, wxT("invalid matrix, wxMatrix2D::GetWidth()") );
    return M_MATRIXDATA->m_width;
}

int wxMatrix2D::GetHeight() const
{
    wxCHECK_MSG( Ok(), 0, wxT("invalid matrix, wxMatrix2D::GetHeight()") );
    return M_MATRIXDATA->m_height;
}

wxSize wxMatrix2D::GetSize() const
{
    wxCHECK_MSG( Ok(), wxSize(0, 0), wxT("invalid matrix, wxMatrix2D::GetSize()") );
    return wxSize(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height);
}

bool wxMatrix2D::PointInMatrix( int x, int y ) const
{
    wxCHECK_MSG(Ok(), false, wxT("Invalid wxMatrix2D"));
    return (x>=0) && (y>=0) && (x<M_MATRIXDATA->m_width) && (y<M_MATRIXDATA->m_height);
}

double wxMatrix2D::GetValue(int x, int y) const
{
    wxCHECK_MSG(PointInMatrix(x, y), 0.0, wxT("Invalid wxMatrix2D"));
    return M_MATRIXDATA->m_data[x + y*M_MATRIXDATA->m_width];
}

void wxMatrix2D::SetValue( int x, int y, double value )
{
    wxCHECK_RET(PointInMatrix(x, y), wxT("Invalid wxMatrix2D or point in matrix"));
    M_MATRIXDATA->m_data[x + y*M_MATRIXDATA->m_width] = value;
}

wxMatrix2D wxMatrix2D::InsertRows(int row, const wxMatrix2D& matrix) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok() && matrix.Ok(), temp, wxT("Invalid wxMatrix2D"));

    int width = M_MATRIXDATA->m_width, height = M_MATRIXDATA->m_height;

    wxCHECK_MSG(width == matrix.GetWidth(), temp, wxT("Number of cols must match in InsertRows"));
    wxCHECK_MSG((row >= -1) && (row <= height), temp, wxT("Invalid wxMatrix2D"));

    if (!temp.Create(width, height + matrix.GetHeight(), false))
        return temp;

    if (row < 0) row = height; // fix row to be positive, from 0 - height

    int size        = width*height;
    int matrix_size = matrix.GetWidth()*matrix.GetHeight();
    double *data        = GetData();
    double *matrix_data = matrix.GetData();
    double *temp_data   = temp.GetData();

    if (row < 1)                // prepend the matrix to the beginning
    {
        memcpy(temp_data, matrix_data, sizeof(double)*matrix_size);
        memcpy(&temp_data[matrix_size], data, sizeof(double)*size);
    }
    else if (row >= height)     // append the matrix to the end
    {
        memcpy(temp_data, data, sizeof(double)*size);
        memcpy(&temp_data[size], matrix_data, sizeof(double)*matrix_size);
    }
    else                        // insert the matrix in the middle
    {
        memcpy(temp_data, data, sizeof(double)*width*row);
        memcpy(&temp_data[width*row], matrix_data, sizeof(double)*matrix_size);
        memcpy(&temp_data[width*row+matrix_size], &data[width*row], sizeof(double)*(size-width*row));
    }

    return temp;
}

wxMatrix2D wxMatrix2D::InsertCols(int col, const wxMatrix2D& matrix) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok() && matrix.Ok(), temp, wxT("Invalid wxMatrix2D"));

    int j, width = M_MATRIXDATA->m_width, height = M_MATRIXDATA->m_height;

    wxCHECK_MSG(height == matrix.GetHeight(), temp, wxT("Number of rows must match in InsertRows"));
    wxCHECK_MSG((col >= -1) && (col <= width), temp, wxT("Invalid wxMatrix2D"));

    if (!temp.Create(width + matrix.GetWidth(), height, false))
        return temp;

    if (col < 0) col = width; // fix col to be positive, from 0 - width

    int matrix_width = matrix.GetWidth();
    int temp_width   = temp.GetWidth();
    double *data        = GetData();
    double *matrix_data = matrix.GetData();
    double *temp_data   = temp.GetData();

    for (j = 0; j < height; j++)
    {
        if (col < 1)                // prepend the matrix to the beginning
        {
            memcpy(&temp_data[j*temp_width], &matrix_data[j*matrix_width], sizeof(double)*matrix_width);
            memcpy(&temp_data[j*temp_width+matrix_width], &data[j*width], sizeof(double)*width);
        }
        else if (col >= width)     // append the matrix to the end
        {
            memcpy(&temp_data[j*temp_width], &data[j*width], sizeof(double)*width);
            memcpy(&temp_data[j*temp_width+width], &matrix_data[j*matrix_width], sizeof(double)*matrix_width);
        }
        else                        // insert the matrix in the middle
        {
            memcpy(&temp_data[j*temp_width], &data[j*width], sizeof(double)*col);
            memcpy(&temp_data[j*temp_width + col], &matrix_data[j*matrix_width], sizeof(double)*matrix_width);
            memcpy(&temp_data[j*temp_width + col + matrix_width], &data[j*width+col], sizeof(double)*(width-col));
        }
    }

    return temp;
}

wxMatrix2D wxMatrix2D::SubMatrix(const wxRect& rect) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid wxMatrix2D"));
    wxCHECK_MSG((rect.width > 0) && (rect.height > 0), temp, wxT("Invalid sub matrix rect"));

    int width = M_MATRIXDATA->m_width, height = M_MATRIXDATA->m_height;
    const wxRect r(0, 0, width, height);
    wxCHECK_MSG(r.Intersect(rect) == rect, temp, wxT("Invalid sub matrix"));

    if (!temp.Create(rect.width, rect.height, false))
        return temp;

    double *data      = GetData();
    double *temp_data = temp.GetData();

    int j, data_size = rect.width*sizeof(double);
    for (j = 0; j < rect.height; j++)
    {
        memcpy(&temp_data[j*rect.width], &data[rect.x + (j+rect.y)*width], data_size);
    }

    return temp;
}

wxMatrix2D wxMatrix2D::SubRows(int start_row, int end_row) const
{
    wxCHECK_MSG(Ok(), wxMatrix2D(), wxT("Invalid wxMatrix2D"));
    return SubMatrix(wxRect(0, start_row, M_MATRIXDATA->m_width, end_row-start_row+1));
}

wxMatrix2D wxMatrix2D::SubCols(int start_col, int end_col) const
{
    wxCHECK_MSG(Ok(), wxMatrix2D(), wxT("Invalid wxMatrix2D"));
    return SubMatrix(wxRect(start_col, 0, end_col-start_col+1, M_MATRIXDATA->m_height));
}

bool wxMatrix2D::Reshape(int width, int height)
{
    wxCHECK_MSG(Ok(), false, wxT("Invalid wxMatrix2D"));
    wxCHECK_MSG(width*height == M_MATRIXDATA->m_width*M_MATRIXDATA->m_height, false, wxT("Reshape is not the same size as original"));

    M_MATRIXDATA->m_width  = width;
    M_MATRIXDATA->m_height = height;
    return true;
}

double* wxMatrix2D::GetData() const
{
    wxCHECK_MSG( Ok(), (double*)nullptr, wxT("invalid matrix, wxMatrix2D::GetData()"));
    return M_MATRIXDATA->m_data;
}

bool wxMatrix2D::Fill( double value )
{
    wxCHECK_MSG(Ok(), false, wxT("Invalid wxMatrix2D"));

    double *data = M_MATRIXDATA->m_data;
    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    if (value == 0)
        memset(data, 0, size*sizeof(double));
    else
    {
        for (i = 0; i < size; ++i) *data++ = value;
    }

    return true;
}

wxMatrix2D wxMatrix2D::Add(const wxMatrix2D &B) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok() && B.Ok() && (GetWidth() == B.GetWidth()) && (GetHeight() == B.GetHeight()),
                temp, wxT("Invalid wxMatrix2D"));
    if (!temp.Create(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height, false))
        return temp;

    double *data      = GetData();
    double *b_data    = B.GetData();
    double *temp_data = temp.GetData();

    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    for (i = 0; i < size; ++i)
    {
        *temp_data++ = (*data++) + (*b_data++);
    }

    return temp;
}

wxMatrix2D wxMatrix2D::Add(double scalar) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid matrix"));
    if (!temp.Create(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height, false))
        return temp;

    double *data      = GetData();
    double *temp_data = temp.GetData();
    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    for (i = 0; i < size; ++i) *temp_data++ = scalar + *data++;

    return temp;
}

wxMatrix2D wxMatrix2D::Mult(const wxMatrix2D &B) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok() && B.Ok(), temp, wxT("Invalid matrix"));
    wxCHECK_MSG(GetWidth() == B.GetHeight(), temp, wxT("Invalid matrix dimensions for mult"));

    int b_width = B.GetWidth();
    int i, j, k, width = M_MATRIXDATA->m_width, height = M_MATRIXDATA->m_height;

    if (!temp.Create(b_width, height, true))
        return temp;

    double *data      = GetData();
    double *b_data    = B.GetData();
    double *temp_data = temp.GetData();

    for (j = 0; j < height; ++j)
    {
        for (i = 0; i < b_width; ++i)
        {
            for (k = 0; k < width; ++k)
            {
                temp_data[j+i*b_width] += data[k+i*width]*b_data[j+k*b_width];
            }
        }
    }

    return temp;
}

wxMatrix2D wxMatrix2D::MultElement(const wxMatrix2D &B) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok() && B.Ok(), temp, wxT("Invalid matrix"));
    wxCHECK_MSG(GetSize() == B.GetSize(), temp, wxT("Invalid matrix dimensions"));

    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    if (!temp.Create(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height, false))
        return temp;

    double *data      = GetData();
    double *b_data    = B.GetData();
    double *temp_data = temp.GetData();

    for (i = 0; i < size; ++i)
        *temp_data++ = (*data++) * (*b_data++);

    return temp;
}

wxMatrix2D wxMatrix2D::Mult(double scalar) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid matrix"));
    if (!temp.Create(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height, false))
        return temp;

    double *data = GetData();
    double *temp_data = temp.GetData();
    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    for (i = 0; i < size; ++i) *temp_data++ = scalar * *data++;

    return temp;
}

wxMatrix2D wxMatrix2D::Pow(double p) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid matrix"));
    if (!temp.Create(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height, false))
        return temp;

    double *data = GetData();
    double *temp_data = temp.GetData();
    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    for (i = 0; i < size; ++i) *temp_data++ = pow(*data++, p);

    return temp;
}

wxMatrix2D wxMatrix2D::PowN(double n) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid matrix"));
    if (!temp.Create(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height, false))
        return temp;

    double *data = GetData();
    double *temp_data = temp.GetData();
    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    for (i = 0; i < size; ++i) *temp_data++ = pow(n, *data++);

    return temp;
}

wxMatrix2D wxMatrix2D::Log() const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid matrix"));
    if (!temp.Create(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height, false))
        return temp;

    double *data = GetData();
    double *temp_data = temp.GetData();
    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    for (i = 0; i < size; ++i) *temp_data++ = log(*data++);

    return temp;
}

wxMatrix2D wxMatrix2D::Log10() const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid matrix"));
    if (!temp.Create(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height, false))
        return temp;

    double *data = GetData();
    double *temp_data = temp.GetData();
    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    for (i = 0; i < size; ++i) *temp_data++ = log10(*data++);

    return temp;
}

wxMatrix2D wxMatrix2D::Exp() const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid matrix"));
    if (!temp.Create(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height, false))
        return temp;

    double *data = GetData();
    double *temp_data = temp.GetData();
    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    for (i = 0; i < size; ++i) *temp_data++ = exp(*data++);

    return temp;
}

double wxMatrix2D::Sum() const
{
    wxCHECK_MSG(Ok(), 0, wxT("Invalid matrix"));

    double sum = 0;
    double *data = GetData();
    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    for (i = 0; i < size; ++i) sum += *data++;
    return sum;
}

double wxMatrix2D::Trace() const
{
    wxCHECK_MSG(Ok(), 0, wxT("Invalid matrix"));

    double sum = 0;
    double *data = GetData();
    int width = GetWidth();
    int i, size = wxMin(width, M_MATRIXDATA->m_height);

    for (i = 0; i < size; ++i) sum += data[i+i*width];
    return sum;
}

wxMatrix2D wxMatrix2D::Transpose() const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid matrix"));
    if (!temp.Create(M_MATRIXDATA->m_height, M_MATRIXDATA->m_width, false))
        return temp;

    double *data      = GetData();
    double *temp_data = temp.GetData();
    int i, j, width = M_MATRIXDATA->m_width, height = M_MATRIXDATA->m_height;

    for (j = 0; j < height; ++j)
    {
        for (i = 0; i < width; ++i)
            temp_data[j + i*height] = *data++;
    }

    return temp;
}

wxMatrix2D wxMatrix2D::Rotate90( int clockwise ) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid matrix"));

    clockwise = clockwise % 4; // limit to 0 - 3
    if (clockwise < 0) clockwise += 4;

    // no rotation at all, just copy
    if ((clockwise == 0) || (clockwise == 4))
    {
        temp.Copy(*this);
        return temp;
    }

    if (clockwise == 2)
        temp.Create(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height, false);
    else // == 1 or 3
        temp.Create(M_MATRIXDATA->m_height, M_MATRIXDATA->m_width, false);

    if (!temp.Ok()) return temp;

    double *data      = GetData();
    double *temp_data = temp.GetData();
    int i, j, width = M_MATRIXDATA->m_width, height = M_MATRIXDATA->m_height;

    for (j = 0; j < height; ++j)
    {
        for (i = 0; i < width; ++i)
        {
            if (clockwise == 1)
                temp_data[(i+1)*height - j - 1] = *data;
            else if (clockwise == 2)
                temp_data[width*(height-j-1) +  (width-i-1)] = *data;
            else // == 3
                temp_data[height*(width-1) + j - i*height] = *data;

            ++data;
        }
    }

    return temp;
}

wxMatrix2D wxMatrix2D::Mirror( bool horizontally ) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid matrix"));
    if (!temp.Create(M_MATRIXDATA->m_width, M_MATRIXDATA->m_height, false))
        return temp;

    double *data      = GetData();
    double *temp_data = temp.GetData();
    int i, j, width = M_MATRIXDATA->m_width, height = M_MATRIXDATA->m_height;

    if (horizontally)
    {
        for (j = 0; j < height; ++j)
        {
            temp_data += width - 1;
            for (i = 0; i < width; ++i)
                *temp_data-- = *data++;

            temp_data += width + 1;
        }
    }
    else
    {
        for (i = 0; i < height; ++i)
        {
            temp_data = temp.GetData() + (height-1-i)*width;
            memcpy( temp_data, data, width*sizeof(double) );
            data += width;
        }
    }

    return temp;
}

// lazy inefficient way to do this, maybe fix later
wxMatrix2D wxMatrix2D::RotateSquare45( bool clockwise ) const
{
    wxMatrix2D temp;
    wxCHECK_MSG(Ok(), temp, wxT("Invalid matrix"));

    int width = M_MATRIXDATA->m_width, height = M_MATRIXDATA->m_height;
    wxCHECK_MSG((width == height) || (width%2 == 1), temp, wxT("Non square or odd sided matrix"));

    if (!temp.Create(height, width, false))
        return temp;

    double *data      = GetData();
    double *temp_data = temp.GetData();

    int i, j, ii, jj;
    int i_inc, j_inc, ii_inc, jj_inc;
    int r, s;   // radius inward and circumference

    // set the center value
    temp_data[int(width/2)*(1 + width)] = data[int(width/2)*(1 + width)];

    for (r = 0; r <= height/2; ++r)
    {
        int w = height - r*2;

        i = j = 0;

        ii = w / 2;
        jj = 0;

        i_inc = ii_inc = 1;
        j_inc = jj_inc = 0;

        for (s = 0; s < w*4-4; ++s)
        {
            if (clockwise)
                temp_data[ii+r + (jj+r)*width] = data[i+r + (j+r)*width];
            else
                temp_data[jj+r + (ii+r)*width] = data[j+r + (i+r)*width];

            if ((i == w - 1) && (i_inc == 1))       // upper right going down
            {
                i_inc = 0;
                j_inc = 1;
            }
            else if ((j == w - 1) && (j_inc == 1))  // lower right going left
            {
                i_inc = -1;
                j_inc = 0;
            }
            else if ((i == 0) && (i_inc == -1))     // lower left going up
            {
                i_inc = 0;
                j_inc = -1;
            }

            if ((ii == w - 1) && (ii_inc == 1))     // upper right going down
            {
                ii_inc = 0;
                jj_inc = 1;
            }
            else if ((jj == w - 1) && (jj_inc == 1)) // lower right going left
            {
                ii_inc = -1;
                jj_inc = 0;
            }
            else if ((ii == 0) && (ii_inc == -1))   // lower left going up
            {
                ii_inc = 0;
                jj_inc = -1;
            }
            else if ((jj == 0) && (jj_inc == -1))   // upper left going right
            {
                ii_inc = 1;
                jj_inc = 0;
            }

            i  += i_inc;
            j  += j_inc;
            ii += ii_inc;
            jj += jj_inc;
        }
    }

    return temp;
}

void wxMatrix2D::Normalize( double sum )
{
    wxCHECK_RET(Ok() && (sum != 0.0), wxT("Invalid matrix"));

    double *data = GetData();
    int i, size = M_MATRIXDATA->m_width * M_MATRIXDATA->m_height;

    double current_sum = 0.0;
    for (i = 0; i < size; ++i) current_sum += data[i];
    if ( current_sum == 0.0 ) return;

    current_sum /= sum;
    for (i = 0; i < size; ++i) data[i] /= current_sum;
}

bool wxMatrix2D::LoadFile( const wxString &filename, const wxArrayInt *cols )
{
    if (filename.IsNull()) return false;

    // valid separators for data
//  const char comma = 44;  // comma
    const char tab = 9;     // tab
    const char space = 32;  // space
//  const char cr = 13;     // carrage return

    wxFile loadfile;
    loadfile.Open( filename, wxFile::read );

    if (!loadfile.IsOpened()) return false;

    m_file_comments.Clear();
//  m_file_comments_positions.Clear();

    wxFileInputStream filestream( loadfile );
    wxTextInputStream textstream( filestream );

    int sizeof_data = 4000;
    double *data = (double*)malloc(sizeof_data*sizeof(double));
    if (data == (double*)nullptr) return false;

    wxString line_str, num_str;

    int i, a, b, pos;
    int num_cols = 0;
    int num_rows = 0;
    int num_points = 0;
    int num_lines = 0;
    double point;

    bool fail = false;


    while ( !filestream.Eof() && !fail )
    {
        ++num_lines;

        line_str = textstream.ReadLine();
        if ( filestream.Eof() ) break;

        line_str.Trim(false);
        line_str.Trim(true);


        if (line_str.Left(1) == wxT("#"))
        {
            m_file_comments.Add( line_str );
            //m_file_comments_positions.Add(num_lines);
        }
        else
        {
            ++num_rows;

            // do this once to figure out how many columns of data there is unless already given
            if ( num_cols < 1 )
            {
                for (i = 0; i < 10000; ++i)
                {
                    line_str.Trim(false);
                    a = line_str.Find(space);
                    b = line_str.Find(tab);

                    if ( (a != -1) && (b != -1) ) pos = wxMin( a, b );
                    else if ( a != -1 ) pos = a;
                    else pos = b;

                    if ( pos != -1 ) num_str = line_str.Left( pos );
                    else num_str = line_str;

                    if ( num_str.ToDouble(&point) )
                    {
                        printf("i%d pts%d cols%d rows%d pos%d pt%lf \n", i, num_points, num_cols, num_rows, pos, point); fflush(stdout);
                        //data[i] = point;
                        ++num_cols;
                        line_str = line_str.Right( line_str.Len() - num_str.Len() );
                        ++num_points;
                    }
                    else
                    {
                        i = 10000;
                        break;
                    }
                }
            }
            else
            {
                if ( num_points > sizeof_data - num_cols*2 )
                {
                    data = (double*)realloc( data, sizeof_data+1000 );
                }
                for (i = 0; i < num_cols; ++i)
                {
                    line_str.Trim(false);
                    a = line_str.Find(space);
                    b = line_str.Find(tab);

                    if ( (a != -1) && (b != -1) ) pos = wxMin( a, b );
                    else if ( a != -1 ) pos = a;
                    else pos = b;

                    if ( pos != -1 ) num_str = line_str.Left( pos );
                    else num_str = line_str;

                    if ( num_str.ToDouble(&point) )
                    {
                        printf("i%d pts%d cols%d rows%d pos%d pt%lf \n", i, num_points, num_cols, num_rows, pos, point); fflush(stdout);
                        //data[numpoints*numcolumns + i] = point;
                        line_str = line_str.Right( line_str.Len() - num_str.Len() );
                        ++num_points;
                    }
                    else
                    {
                        // if not just a blank line then data is wrong
                        if (i != 0)
                        {
                            fail = true;
                            wxMessageBox(wxT("# for comments\n7   4\n33  25\n..."),
                                         wxT("Invalid data file format"), wxOK);
                        }
                        i = num_cols;
                        break;
                    }
                }
            }
        }
    }

    // not static
    data = (double*)realloc( data, (num_points+1)*sizeof(double) );
    Create( num_cols, num_rows, data, false );
    loadfile.Close();
    return true;
}

wxString wxMatrix2D::ToString(const wxString& colSep, const wxString& rowSep) const
{
    wxCHECK_MSG(Ok(), wxEmptyString, wxT("Invalid wxMatrix2D"));

    double *data = GetData();
    int i, j, width = M_MATRIXDATA->m_width, height = M_MATRIXDATA->m_height;
    wxString str;

    for (j = 0; j < height; ++j)
    {
        for (i = 0; i < width; ++i)
        {
            str += wxString::Format(wxT("%g"), *data++);
            if (i < width - 1) str += colSep;
        }

        if (j < height - 1) str += rowSep;
    }

    return str;
}

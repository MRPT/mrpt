#ifndef CVD_HAAR_H
#define CVD_HAAR_H

#include <vector>
#include <cmath>
#include <cvd/image.h>

namespace CVD {

namespace Internal {
    template<class It, class TempIt>
    inline void haar1D(It from, int w, TempIt store){
        for(int i = 0; i < w; ++i){
            store[i] = (from[2*i] + from[2*i+1]) * M_SQRT1_2;
            store[i+w] = (from[2*i] - from[2*i+1]) * M_SQRT1_2;
        }
        std::copy(store, store+2*w, from);
    }

    template<class It, class TempIt>
    inline void inv_haar1D(It from, int w, TempIt store){
        for(int i = 0; i < w; ++i){
            store[2*i] = (from[i] + from[w+i]) * M_SQRT1_2;
            store[2*i+1] = (from[i] - from[w+i]) * M_SQRT1_2;
        }
        std::copy(store, store+2*w, from);
    }
}


/// computes the 1D Haar transform of a signal in place. This version takes
/// two iterators, and the data between them will be transformed. Will only work
/// correctly on 2^N data points.
/// @param from iterator pointing to the beginning of the data
/// @param to iterator pointing to the end (after the last element)
/// @ingroup gVision
template<class It>
inline void haar1D(It from, It to){
    std::vector<typename std::iterator_traits<It>::value_type> store(std::distance(from,to), typename std::iterator_traits<It>::value_type());
    for(int w = std::distance(from,to)/2; w > 0; w /= 2)
        Internal::haar1D(from, w, store.begin());
}

/// computes the inverse 1D Haar transform of a signal in place. This version takes
/// two iterators, and the data between them will be transformed. Will only work
/// correctly on 2^N data points.
/// @param from iterator pointing to the beginning of the data
/// @param to iterator pointing to the end (after the last element)
/// @ingroup gVision
template<class It>
inline void inv_haar1D(It from, It to){
    std::vector<typename std::iterator_traits<It>::value_type> store(std::distance(from,to), typename std::iterator_traits<It>::value_type());
    for(int w = 1; w < std::distance(from,to); w *= 2)
        Internal::inv_haar1D(from, w, store.begin());
}

/// computes the 1D Haar transform of a signal in place. Will only work
/// correctly on 2^N data points.
/// @param from iterator pointing to the beginning of the data
/// @param size number of data points, should be 2^N
/// @ingroup gVision
template<class It>
inline void haar1D(It from, int size){
    haar1D(from, from + size);
}

/// computes the inverse 1D Haar transform of a signal in place. Will only work
/// correctly on 2^N data points.
/// @param from iterator pointing to the beginning of the data
/// @param size number of data points, should be 2^N
/// @ingroup gVision
template<class It>
inline void inv_haar1D(It from, int size){
    inv_haar1D(from, from + size);
}

/// computes the 2D Haar transform of a signal in place. Works only with 
/// data with power of two dimensions, 2^N x 2^ M.
/// @param from iterator pointing to the beginning of the data
/// @param width columns of data, should be 2^N
/// @param height rows of data, should be 2^M
/// @param stride offset between rows, if negative will be set to width
/// @ingroup gVision
template<class It>
inline void haar2D(It from, const int width, const int height, int stride = -1){
    if(stride < 0) stride = width;
    typedef typename std::iterator_traits<It>::value_type T;
    std::vector<T> column(height, T());
    std::vector<T> store(std::max(width,height), T());
    int w = width;
    int h = height;
    while(w > 1 || h > 1){
        if(w > 1){
            for(int i = 0; i < h; ++i){
                Internal::haar1D(from + stride * i, w/2, store.begin());
            }
        }
        if(h > 1){
            for(int i = 0; i < w; ++i){
                for(int j = 0; j < h; ++j)
                    column[j] = from[stride * j + i];
                Internal::haar1D(column.begin(), h/2, store.begin());
                for(int j = 0; j < h; ++j)
                    from[stride * j + i] = column[j];
            }
        }
        if(w>1) w/=2;
        if(h>1) h/=2;
    }
}

/// computes the 2D Haar transform of an image in place. Works only with 
/// images with power of two dimensions, 2^N x 2^ M.
/// @param I image to be transformed
/// @ingroup gVision
template<class T>
inline void haar2D( BasicImage<T> & I ){
    haar2D(I.data(), I.size().x, I.size().y, I.row_stride());
}

}

#endif // CVD_HAAR_H

#ifndef CVD_HARRIS_CORNER_H
#define CVD_HARRIS_CORNER_H

#include <vector>
#include <utility>
#include <algorithm>
#include <cmath>
#include <cstdlib>

#include <cvd/image.h>
#include <cvd/convolution.h>

namespace CVD
{
	
	namespace Harris
	{
		///A Utility fuction.
		template<class C> inline C sq(const C& c)
		{
			return c*c;
		}

		///Compute the corner score according to Harris
		///@ingroup gVision
		struct HarrisScore
		{
			static float Compute(float xx, float xy, float yy)
			{
				return (xx * yy - xy * xy) - 0.04 * sq(xx + yy);
			}
		};
		
		///Compute the score according to Shi-Tomasi
		///@ingroup gVision
		struct ShiTomasiScore
		{
			static float Compute(float xx, float xy, float yy)
			{
				float l1 = xx + yy + std::sqrt(sq(xx - yy)+4.0*xy*xy);   
				float l2 = xx + yy - std::sqrt(sq(xx - yy)+4.0*xy*xy);
				return std::min(abs(l1), std::abs(l2));
			}
		};
		
		///Used to save corner positions from harrislike_corner_detect
		///@ingroup gVision
		struct PosInserter
		{
			static void insert(std::vector<ImageRef>& i, const std::pair<float, ImageRef>& p)
			{
				i.push_back(p.second);
			}
		};

		///Used to save corner positions and scores from harrislike_corner_detect
		///@ingroup gVision
		struct PairInserter
		{
			static void insert(std::vector<std::pair<float, ImageRef> >& i, const std::pair<float, ImageRef>& p)
			{
				i.push_back(p);
			}
		};
	}


	/// Generic Harris corner detection function. This can use any scoring metric and
	/// can store corners in any container. The images used to hold the intermediate
	/// results must be passed to this function.
	///
	///@param i Input image.
	///@param c Container holding detected corners
	///@param N Number of corners to detect
	///@param blur Blur radius to use
	///@param sigmas Number of sigmas to use in blur.
	///@param xx Holds the result of blurred, squared X gradient.
	///@param xy Holds the result of blurred, X times Y gradient.
	///@param yy Holds the result of blurred, squared Y gradient.
	///@ingroup gVision
	template<class Score, class Inserter, class C, class B> void harrislike_corner_detect(const BasicImage<B>& i, C& c, unsigned int N, float blur, float sigmas, BasicImage<float>& xx, BasicImage<float>& xy, BasicImage<float>& yy)
	{
		using std::pair;
		using std::greater;
		using std::vector;
		using std::make_pair;

		if(! (i.size() == xx.size() && i.size() ==xy.size() && i.size() == yy.size()))
			throw Exceptions::Convolution::IncompatibleImageSizes("harrislike_corner_detect");

		zeroBorders(xx);
		zeroBorders(xy);
		zeroBorders(yy);

		typedef typename Pixel::traits<B>::wider_type gType;
		
		//Compute gradients
		for(int y=1; y < i.size().y - 1; y++)
			for(int x=1; x < i.size().x - 1; x++)
			{
				
				//FIXME use fast-casting using an arrsy for byte to float conversion.
				gType gx = (gType)i[y][x-1] - i[y][x+1];
				gType gy = (gType)i[y-1][x] - i[y+1][x];
				
				//Compute the gradient moments
				xx[y][x] = gx * gx;
				xy[y][x] = gx * gy;
				yy[y][x] = gy * gy;
			}

		convolveGaussian(xx, xx, blur, sigmas);
		convolveGaussian(xy, xy, blur, sigmas);
		convolveGaussian(yy, yy, blur, sigmas);
		
		//Avoid computing the score along the image borders where the
		//result of the convolution is not valid.
		int	kspread = (int)ceil(sigmas * blur);

		//Compute harris score
		for(int y=kspread; y < i.size().y-kspread; y++)
			for(int x=kspread; x <i.size().x-kspread; x++)
				xx[y][x] = Score::Compute(xx[y][x], xy[y][x], yy[y][x]);

		vector<pair<float, ImageRef> > corner_heap;
		corner_heap.reserve(N+1);

		typedef greater<pair<float, ImageRef> > minheap_compare;

		//Keep the N best corner scores, using a min-heap. This allows us to always
		//remove the smallest element, keeping the largest ones.

		//C++ heap functions use std::less to create a max-heap, growing from the
		//beginning of the array. This allows for convenient sorting. pop_heap
		//gets the largest element, and the heap is shrunk by 1. This element
		//is then put on to the end of the array, ie the spot freed up by shrinking
		//the heap by 1. Repeating this procedure will sort the heap, pulling out
		//the largest elements and placing them at the end. The resulting array will
		//then be sorted by std::less.

		//Therefore we need to use std::greater to create a min-heap

		//The first element in the array will be the smallest value

		//Find local maxima
		for(int y=kspread; y < i.size().y-kspread; y++)
			for(int x=kspread; x <i.size().x-kspread; x++)
			{
				float c = xx[y][x];

				if( c > xx[y-1][x-1]  &&
					c > xx[y-1][x+0]  &&
					c > xx[y-1][x+1]  &&
					c > xx[y-0][x-1]  &&
					c > xx[y-0][x+1]  &&
					c > xx[y+1][x-1]  &&
					c > xx[y+1][x+0]  &&
					c > xx[y+1][x+1])
				{
					if(corner_heap.size() <= N  || c > corner_heap[0].first)
					{
						corner_heap.push_back(make_pair(c, ImageRef(x,y)));
						push_heap(corner_heap.begin(), corner_heap.end(), minheap_compare());
					}

					if(corner_heap.size() > N)
					{
						pop_heap(corner_heap.begin(), corner_heap.end(), minheap_compare());
						corner_heap.pop_back();
					}
				}
			}

		for(unsigned int i=0; i < corner_heap.size(); i++)
			Inserter::insert(c, corner_heap[i]);
	}

	template<class C> void harris_corner_detect(const BasicImage<C>& i, std::vector<ImageRef>& c, unsigned int N, float blur=1.0, float sigmas = 3.0)
	{
		Image<float> xx(i.size()), xy(i.size()), yy(i.size());
		harrislike_corner_detect<Harris::HarrisScore, Harris::PosInserter>(i, c, N, blur, sigmas, xx, xy, yy);
	}

	template<class C> void shitomasi_corner_detect(const BasicImage<C>& i, std::vector<ImageRef>& c, unsigned int N, float blur=1.0, float sigmas = 3.0)
	{
		Image<float> xx(i.size()), xy(i.size()), yy(i.size());
		harrislike_corner_detect<Harris::ShiTomasiScore, Harris::PosInserter>(i, c, N, blur, sigmas, xx, xy, yy);
	}
}
#endif

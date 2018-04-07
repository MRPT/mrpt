#ifndef CVD_INCLUDE_MORPHOLOGY_H
#define CVD_INCLUDE_MORPHOLOGY_H

#include <cvd/vision_exceptions.h>
#include <cvd/vision.h>
#include <cvd/image.h>
#include <functional>
#include <map>
#include <vector>


namespace CVD
{
	#ifndef DOXYGEN_IGNORE_INTERNAL
	namespace Internal
	{
		namespace MorphologyHelpers
		{
			using namespace std;
			
			//Compute pointer offsets for a bunch of ImageRef offsets.
			template<class T> vector<ptrdiff_t> offsets(const vector<ImageRef>& v, const BasicImage<T>& s)
			{
				vector<ptrdiff_t> off;

				for(unsigned int i=0; i < v.size(); i++)
					off.push_back(v[i].x + v[i].y * s.row_stride()-1);
				return off;
			}
			
			//Split a list of ImageRefs up in to rows.
			vector<vector<ImageRef> > row_split(const vector<ImageRef>& v, int y_lo, int y_hi);
		}
	}
	#endif

	/// Perform a morphological operation on the image.
	///
	/// At the edge of the image, the structuring element is cropped to the image
	/// boundary. This function is for homogenous structuring elements, so it is
	/// suitable for erosion, dialtion and etc, not hit-and-miss and so on.
	///
	/// For example:
	/// @code
	///     Image<byte> image, eroded;
	///     vector<ImageRef> structuring_element = getDisc(10);
	///     
	///     ...
	///     
	///     morphology(image, structure_element, Erode<byte>(), eroded);
	/// @endcode
	///
	/// Morphology is performed efficiently using an incremental algorithm. As the
	/// structuring element is moved across the images, only pixels on it's edge are
	/// added and removed. Other morphological operators can be added by creating a
	/// class with the following methods:
	///
	/// @code
	/// template<class T> struct Operation
	/// {
	///     void insert(const T&); //Add a pixel
	///     void remove(const T&); //Remove a pixel
	///     void clear(const T&);  //Remove all pixels
	///     T get();               //Get the current value.
	/// }
	/// @endcode
	/// 
	/// Grayscale erode could be implemented with a multiset to store and remove pixels. Get would simply
	/// return the first element in the multiset.
	///
	/// @param in The source image.
	/// @param selem The structuring element. See e.g. getDisc()
	/// @param a_ The morphological operation to perform.  See Morphology
	/// @param out The destination image.
	/// @ingroup gVision
	template<class Accumulator, class T>
	void morphology(const BasicImage<T>& in, const std::vector<ImageRef>& selem, const Accumulator& a_, BasicImage<T>& out)
	{
		using Internal::MorphologyHelpers::offsets;	
		using Internal::MorphologyHelpers::row_split;	
		using std::min;
		using std::max;
		using std::vector;

		if(in.size() != out.size())
			throw Exceptions::Vision::IncompatibleImageSizes(__FUNCTION__);

		//Cases are:
		//
		// Small selem compared to image:
		//   Topleft corner, top row, top right corner
		//   left edge, centre, right edge
		//   etc
		
		////////////////////////////////////////////////////////////////
		//Find the extents of the structuring element
		int x_lo = selem[0].x;
		int x_hi = selem[0].x;
		int y_lo = selem[0].y;
		int y_hi = selem[0].y;

		for(unsigned int i=0; i < selem.size(); i++)
		{
			x_lo = min(x_lo, selem[i].x);
			x_hi = max(x_hi, selem[i].x);
			y_lo = min(y_lo, selem[i].y);
			y_hi = max(y_hi, selem[i].y);
		}

		////////////////////////////////////////////////////////////////	
		//Shift the  structure element by one and find the differeneces
		vector<ImageRef> structure_element = selem;
		vector<ImageRef> shifted;

		sort(structure_element.begin(), structure_element.end());
		for(unsigned int i=0; i < structure_element.size(); i++)
			shifted.push_back(structure_element[i] + ImageRef(1, 0));
		
		vector<ImageRef> add, remove;
		set_difference(shifted.begin(), shifted.end(), structure_element.begin(), structure_element.end(), back_inserter(add));
		set_difference(structure_element.begin(), structure_element.end(), shifted.begin(), shifted.end(), back_inserter(remove));

		/////////////////////////////////////////////////////////////////
		//	
		//Compute the integer offsets to pixels for speed;
		vector<ptrdiff_t> add_off = offsets(add, in);
		vector<ptrdiff_t> remove_off = offsets(remove, in);
		

		/////////////////////////////////////////////////////////////////
		//	
		// Split by rows, to make the top and bottom edges easier.
		//
		//Because of set operations, the ImageRefs are ordered within each row.
		vector<vector<ImageRef> > split_selem = row_split(structure_element, y_lo, y_hi);
		vector<vector<ImageRef> > split_add   = row_split(add, y_lo, y_hi);
		vector<vector<ImageRef> > split_remove= row_split(remove, y_lo, y_hi);
		
		Accumulator acc(a_);
		//If the image is at least as wide as the structuring element
		if(x_hi - x_lo + 1 <= in.size().x)
			for(int y=0; y < in.size().y; y++)
			{
				//Find the rows which overlap with the image. Only work with these rows.
				int startrow = max(0, - y_lo - y);
				int endrow =   split_selem.size() - max(0, y + y_hi - in.size().y+1);
				
				//Figure out the range of the "easy" bit.
				int x_first_full = max(0, -x_lo);                               //This is the first position at which we have a full kernel in the image
				int x_after_last_full = min(in.size().x, in.size().x - x_hi);   //This is one beyone the end of the position where the last kernel fits in the image.

				//Clear the  accumulator
				acc.clear();

				//Fill in the accumulator sitting up against the left hand side of the image.
				for(int i=startrow; i < endrow; i++)
					for(int j=(int)split_selem[i].size()-1; j >=0 && split_selem[i][j].x >=0; j--)
						acc.insert(in[y + split_selem[i][0].y][split_selem[i][j].x]);
				
				out[y][0] = acc.get();

				//Shift the kernel until we get to the point where
				//we can start shifting the kernel without testing to 
				//see it fits withing the image width.
				for(int x=1; x <= x_first_full ; x++)
				{
					for(int i=startrow; i < endrow; i++)
						for(int j=(int)split_remove[i].size()-1; j >=0 && split_remove[i][j].x+x-1 >=0; j--)
							acc.remove(in[y + split_remove[i][0].y][x+split_remove[i][j].x-1]);

					for(int i=startrow; i < endrow; i++)
						for(int j=(int)split_add[i].size()-1; j >=0 && split_add[i][j].x+x-1 >=0; j--)
							acc.insert(in[y + split_add[i][0].y][x+split_add[i][j].x-1]);

					out[y][x] = acc.get();
				}

				//Go through the two incremental kernels to figure out which
				//indices are fit within the image. This removes a test from
				//the following shift section.
				int add_start=0, add_end=0, remove_start=0, remove_end=0;
				for(int i=0; i < startrow; i++)
				{
					add_start+=split_add[i].size();
					remove_start+=split_remove[i].size();
				}
				for(int i=0; i < endrow; i++)
				{
					add_end+=split_add[i].size();
					remove_end+=split_remove[i].size();
				}

				//Shift the kernel in the area which requires no tests.
				for(int x=max(0, -x_lo+1); x < x_after_last_full; x++)
				{
					for(int i=remove_start; i < remove_end; i++)
						acc.remove(*(in[y] + x + remove_off[i]));
					
					for(int i=add_start; i < add_end; i++)
						acc.insert(*(in[y] + x + add_off[i]));
					
					out[y][x] = acc.get();
				}
				
				//Now perform the right hand edge
				for(int x=x_after_last_full; x < in.size().x ; x++)
				{
					for(int i=startrow; i < endrow; i++)
						for(int j=0; j < (int)split_remove[i].size() && split_remove[i][j].x+x-1 < in.size().x; j++)
							acc.remove(in[y + split_remove[i][0].y][x+split_remove[i][j].x-1]);

					for(int i=startrow; i < endrow; i++)
						for(int j=0; j < (int)split_add[i].size() && split_add[i][j].x+x-1 < in.size().x; j++)
							acc.insert(in[y + split_add[i][0].y][x+split_add[i][j].x-1]);

					out[y][x] = acc.get();
				}
			}
		else
		{
			//The image is too narrow to have a clear area in the middle.

			for(int y=0; y < in.size().y; y++)
			{
				//Find the rows which overlap with the image. Only work with these rows.
				int startrow = max(0, - y_lo - y);
				int endrow =   split_selem.size() - max(0, y + y_hi - in.size().y+1);
				
				//Clear the accumulator
				acc.clear();

				//Fill in the accumulator sitting up against the left hand side of the image.
				for(int i=startrow; i < endrow; i++)
					for(int j=0; j < (int)split_selem[i].size(); j++)
					{
						int xp = split_selem[i][j].x;
						if(xp >= 0 && xp < in.size().x)
							acc.insert(in[y + split_selem[i][0].y][xp]);
					}
				
				out[y][0] = acc.get();

				//Shift the kernel using the incrementals
				for(int x=1; x < in.size().x ; x++)
				{
					for(int i=startrow; i < endrow; i++)
						for(int j=0; j < (int)split_remove[i].size(); j++)
						{
							int xp = x + split_remove[i][j].x - 1;
							if(xp >= 0 && xp < in.size().x)
								acc.remove(in[y + split_add[i][0].y][xp]);
						}

					for(int i=startrow; i < endrow; i++)
						for(int j=0; j < (int)split_add[i].size(); j++)
						{
							int xp = x + split_add[i][j].x - 1;
							if(xp >= 0 && xp < in.size().x)
								acc.insert(in[y + split_add[i][0].y][xp]);
						}

					out[y][x] = acc.get();
				}
			}
		}
	}

	#ifndef DOXYGEN_IGNORE_INTERNAL
		namespace Internal
		{
			template<class C, class D> class PerformMorphology{};

			template<class C, class D>  struct ImagePromise<PerformMorphology<C, D> >
			{
				ImagePromise(const BasicImage<C>& im, const D& acc, const std::vector<ImageRef>& s_)
				:i(im),a(acc),s(s_)
				{}

				const BasicImage<C>& i;
				const D& a;
				const std::vector<ImageRef>& s;

				template<class E> void execute(Image<E>& j)
				{
					j.resize(i.size());
					if(i.data() == j.data())
					{
						Image<E> b(j.size());
						morpholog(i, s, a, b);
						j=b;
					}
					else
						morphology(i, s, a, j);
				}
			};
		};

		template<class C, class D> Internal::ImagePromise<Internal::PerformMorphology<C, D> > morphology(const BasicImage<C>& c, const std::vector<ImageRef>& selem, const D& a)
		{
			return Internal::ImagePromise<Internal::PerformMorphology<C, D> >(c, a, selem);
		}
	#else
		
		/// Perform a morphological operation on the image.
		/// 
		/// @param in The source image.
		/// @param selem The structuring element. See e.g. getDisc()
		/// @param a_ The morphological operation to perform.  See Morphology
		/// @param out The destination image.
		/// @ingroup gVision
		Image<T> morphology(const BasicImage<T>& in, const std::vector<ImageRef>& selem, const Accumulator& a_);

	#endif

	///Image morphology operations
	///@ingroup gVision
	namespace Morphology
	{
		///A helper class for performing basic grayscale morphology
		///on an image. The comparator determines the ordering, and hence
		///the morphological operation. See morphology().
		///@ingroup gVision
		template<class T, template<class> class Cmp> struct BasicGray
		{
			private:
				std::map<T, int, Cmp<T> > pix;
			
			public:
				void clear()
				{
					pix.clear();
				}
				void insert(const T& t)
				{
					pix[t]++;
				}

				void remove(const T& t)
				{
					--pix[t];
				}

				T get()
				{
					typedef typename std::map<T, int, Cmp<T> >::iterator it;

					for(it i=pix.begin(); i != pix.end();)
					{
						it old = i;
						i++;

						if(old->second == 0)
							pix.erase(old);
						else
							return old->first;
					}

					assert(0);
					return 0;
				}
		};
		

		///Class for performing greyscale erosion. See morphology().
		///@ingroup gVision
		template<class T> class Erode: public BasicGray<T, std::less>
		{
		};


		///Class for performing greyscale dilation. See morphology().
		///@ingroup gVision
		template<class T> class Dilate: public BasicGray<T, std::greater>
		{
		};


		///Class for performing percentile filtering. See morphology().
		///@ingroup gVision
		template<class T> class Percentile;

		///Class for performing median filtering. See morphology().
		///@ingroup gVision
		template<class T> class Median;

		///A helper class for performing basic grayscale morphology
		///on an image of bytes.
		///See morphology().
		///@ingroup gVision
		struct BasicGrayByte
		{
			protected:
				int histogram[256];
				int total;
			
			public:
				BasicGrayByte()
				{
					clear();
				}

				void clear()
				{
					total=0;
					for(int i=0; i < 256; i++)
						histogram[i] = 0;
				}

				void insert(byte t)
				{
					total++;
					histogram[t]++;
				}

				void remove(byte t)
				{
					total--;
					histogram[t]--;
				}
		};


		///Class for performing greyscale erosion of bytes. See morphology().
		///@ingroup gVision
		template<> class Erode<byte>: public BasicGrayByte
		{
			public:
				byte get()
				{
					for(int j=0; j < 256; j++)
						if(histogram[j])
							return j;
					
					assert(0);
					return 0;
				}
		};

		///Class for performing greyscale dilation of bytes. See morphology().
		///@ingroup gVision
		template<> class Dilate<byte>: public BasicGrayByte
		{
			public:
				byte get()
				{
					for(int j=255; j >=0 ; j--)
						if(histogram[j])
							return j;
					
					assert(0);
					return 0;
				}
		};

		///Class for performing percentile filtering of bytes. See morphology().
		///@ingroup gVision
		template<> class Percentile<byte>: public BasicGrayByte
		{
			private:
				double ptile;

			public:
				Percentile(double p)
				:ptile(p)
				{
				}

				byte get()
				{
					using std::max;

					if(ptile < 0.5)
					{
						int sum=0;
						//because we use a > test below (to work for the 0th ptile)
						//we have to use the scaled threshold -1 otherwise it will
						//not work for the 100th percentile.
						int threshold = max(0, (int)floor(total * ptile+.5)- 1);

						for(int j=0; j < 255; j++)
						{
							sum += histogram[j];

							if(sum > threshold)
								return j;
						}
						
						return 255;
					}
					else
					{
						//Approach from the top for high percentiles
						int sum=0;
						int threshold = max(0, (int)floor(total * (1-ptile)+.5)- 1);

						for(int j=255; j > 0; j--)
						{
							sum += histogram[j];

							if(sum > threshold)
								return j;
						}
						
						return 0;
					}
				}
		};


		///Class for performing percentile filtering of bytes. See morphology().
		///@ingroup gVision
		template<> class Median<byte>: public Percentile<byte>
		{
			public:
				Median()
				:Percentile<byte>(0.5)
				{
				}
		};

		///Class for performing binary morphology. This class is incomplete and
		///used to build actual functions such as BinaryErode, BinaryDilate and BinaryMedian. See morphology().
		///@ingroup gVision
		template<class T> struct BasicBinary
		{
				int t, f;
				BasicBinary()
				:t(0),f(0)
				{}

				void insert(bool b)
				{
					if(b)
						t++;
					else
						f++;
				}

				void remove(bool b)
				{
					if(b)
						t--;
					else
						f--;
				}

				void clear()
				{
					t=f=0;
				}
		};

		///Class for performing binary erosion. See morphology().
		///@ingroup gVision
		template<class T=bool> struct BinaryErode : public BasicBinary<T>
		{
			using BasicBinary<T>::f;
			T get()
			{
				return !f;
			}
		};

		///Class for performing binary dilation. See morphology().
		///@ingroup gVision
		template<class T=bool> struct BinaryDilate : public BasicBinary<T>
		{
			using BasicBinary<T>::t;
			T get()
			{
				return t!= 0;
			}
		};

		///Class for performing binary median filtering. See morphology().
		///@ingroup gVision
		template<class T=bool> struct BinaryMedian : public BasicBinary<T>
		{
			using BasicBinary<T>::t;
			using BasicBinary<T>::f;
			T get()
			{
				return (t > f);
			}
		};
	}

	namespace median{
		//Some helper classes for median
		template<class T> T median4(T a, T b, T c, T d)
		{
			int v[4] = {a, b, c, d};
			std::nth_element(v, v+2, v+4);
			return v[2];
		}

		template<class T> T median4(const BasicImage<T>& im, int r, int c)
		{
			return median4(im[r][c], im[r][c+1], im[r+1][c], im[r+1][c+1]);
		}

		template<class T> T median6(T a, T b, T c, T d, T e, T f)
		{
			int v[6] = {a, b, c, d, e, f};
			std::nth_element(v, v+3, v+6);
			return v[3];
		}

		template<class T> T median6_row(const BasicImage<T>& im, int r, int c)
		{
			return median6(im[r][c], im[r][c+1], im[r][c+2], im[r+1][c], im[r+1][c+1], im[r+1][c+2]);
		}
		template<class T> T median6_col(const BasicImage<T>& im, int r, int c)
		{
			return median6(im[r][c], im[r][c+1], im[r+1][c], im[r+1][c+1], im[r+2][c], im[r+2][c+1]);
		}

	};
	
	#ifndef DOXYGEN_IGNORE_INTERNAL
		//Overload for median filtering of byte images, to special-case 3x3
		void morphology(const BasicImage<byte>& in, const std::vector<ImageRef>& selem, const Morphology::Median<byte>& m, BasicImage<byte>& out);
	#endif

}

#endif

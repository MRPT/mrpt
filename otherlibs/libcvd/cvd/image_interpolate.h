#ifndef CVD_IMAGE_INTERPOLATE_H
#define CVD_IMAGE_INTERPOLATE_H

#include <TooN/TooN.h>
#include <cvd/image.h>
#include <cvd/internal/pixel_operations.h>
#include <cvd/internal/rgb_components.h>
#include <cvd/vector_image_ref.h>
#include <math.h>

namespace CVD
{
	///Classes used to specify the interpolation type
	///for image_interpolate.
	///@ingroup gImage
	namespace Interpolate
	{
	         /** This does not interpolate: it uses the nearest neighbour.
		     
		     The sub pixel to be accessed is \f$p = (x,y)\f$. The nearest pixel is
		     \f$q = ( \operatorname{round}\ x, \operatorname{round}\ y)\f$
		     The interpolated value, \f$v\f$, is \f$v = I(q)\f$
		 */
		class NearestNeighbour{};

		/** This class is for bilinear interpolation.
		    
		    Define \f$p' = ( \operatorname{floor}\ x, \operatorname{floor}\ y)\f$ and
		    \f$\delta = p - p'\f$
		    
		    4 pixels in a square with \f$p'\f$ in the top left corner are taken:
		    \f[\begin{array}{rl}
		    a =& I(p')\\   
		    b =& I(p' + (1,0))\\  
		    c =& I(p' + (0,1))\\  
		    d =& I(p' + (1,1))
		    \end{array}
		    \f]
		    
		    The interpolated value, \f$v\f$, is 
		    \f[v = (1-\delta_y)((1-\delta_x)a + \delta_xb) + \delta_y((1-\delta_x)c + \delta_xd)\f]
		*/
		class Bilinear{};

		/**
 		  This class is for bicubic (not bicubic spline) interpolation.

		  \f[ v = \sum_{m=-1}^2\sum_{n=-1}^2 I(x' + m, y' + n)r(m - \delta_x)r(\delta_y-n) \f]
  		  where:
		    \f[\begin{array}{rl}
		    r(x) =& \frac{1}{6}\left[ p(x+2)^3 - 4p(x+1)^3 + 6p(x)^3 - 4p(x-1)^3 \right]\\
		    p(x) =& \begin{cases}x&x>0\\0&x \le 0\end{cases}
		    \end{array}\f]                                                             
		
		    This algorithm is described in http://astronomy.swin.edu.au/~pbourke/colour/bicubic/ 
		*/
		class Bicubic{};
	};

	#ifdef DOXYGEN_INCLUDE_ONLY_FOR_DOCS
		///This is a generic interpolation class which wraps in image and provides
		///a similar interface for floating point pixel positions.
		///@param I The interpolation type. See CVD::Interpolate for available types.
		///@param P The pixel type.
		///@ingroup gImage
		template<class I, class P> class image_interpolate
		{
		  public:
			///Construct the class.
			///@param i The image to be interpolated.
			image_interpolate(const BasicImage<P>& i);

			///Is this pixel inside the image?
			///@param pos The coordinate to test.
			bool in_image(const TooN::Vector<2>& pos) const;

			///Access the pixel at pos, with interpolation.
			///Bounds checking is the same as for CVD::Image.
			///@param pos The pixel to access
			float_type operator[](const TooN::Vector<2>& pos) const;

			///Return the minimum value for which in_image returns true.
			TooN::Vector<2> min() const;
			///Return the first value for which in_image returns false.
			TooN::Vector<2> max() const;
		};
	#endif



	#ifndef DOXYGEN_IGNORE_INTERNAL

	template<class I, class C> class image_interpolate;

	//Zero order (nearest neighbour)

	template<class C> class image_interpolate<Interpolate::NearestNeighbour, C>
	{
		private:
			const BasicImage<C>* im;

			int round(double d) const
			{
				if(d < 0)
					return (int)ceil(d - .5);
				else
					return (int)floor(d + .5);
			}

			ImageRef to_ir(const TooN::Vector<2>& v) const
			{
				return ImageRef(round(v[0]), round(v[1]));
			}

			typedef typename Pixel::traits<C>::float_type FT;
	
		public:
			image_interpolate(const BasicImage<C>& i)
			:im(&i)
			{}

			bool in_image(const TooN::Vector<2>& pos) const
			{
				return im->in_image(to_ir(pos));
			}

			FT operator[](const TooN::Vector<2>& pos) const
			{
				return (*im)[to_ir(pos)];
			}

			TooN::Vector<2> min() const
			{
				return TooN::makeVector( 0, 0);
			}

			TooN::Vector<2> max() const
			{
				return vec(im->size());
			}

			
	};

	template<class T> class image_interpolate<Interpolate::Bilinear, T>
	{
		private:
			const BasicImage<T>* im;

			TooN::Vector<2> floor(const TooN::Vector<2>& v) const
			{
				return TooN::makeVector( ::floor(v[0]), ::floor(v[1]));
			}

			TooN::Vector<2> ceil(const TooN::Vector<2>& v) const
			{
				return TooN::makeVector( ::ceil(v[0]), ::ceil(v[1]));
			}

			typedef typename Pixel::traits<T>::float_type FT;

		public:
			image_interpolate(const BasicImage<T>& i)
			:im(&i)
			{}

			bool in_image(const TooN::Vector<2>& pos) const
			{
				return im->in_image(ir(floor(pos))) && im->in_image(ir(ceil(pos)));
			}

			FT operator[](const TooN::Vector<2>& pos) const
			{
				TooN::Vector<2> delta =  pos - floor(pos);

				ImageRef p = ir(floor(pos));

				double x = delta[0];
				double y = delta[1];

				FT ret;

				for(unsigned int i=0; i < Pixel::Component<T>::count; i++)
				{
					float a, b=0, c=0, d=0;

					a = Pixel::Component<T>::get((*im)[p + ImageRef(0,0)], i) * (1-x) * (1-y);

					if(x != 0)
						b = Pixel::Component<T>::get((*im)[p + ImageRef(1,0)], i) * x * (1-y);
					
					if(y != 0)
					c = Pixel::Component<T>::get((*im)[p + ImageRef(0,1)], i) * (1-x) * y;

					if(x !=0 && y != 0)
						d = Pixel::Component<T>::get((*im)[p + ImageRef(1,1)], i) * x * y;
					
					Pixel::Component<FT>::get(ret, i) = a + b + c + d;
				}

				return ret;
			}

			TooN::Vector<2> min() const
			{
				return TooN::makeVector( 0, 0);
			}

			TooN::Vector<2> max() const
			{
				return vec(im->size());
			}

	};


	template<class T> class image_interpolate<Interpolate::Bicubic, T>
	{
		private:
			const BasicImage<T>* im;

			float p(float f) const
			{
				return f <0 ? 0 : f;
			}

			float r(float x) const
			{
				return (  pow(p(x+2), 3) - 4 * pow(p(x+1),3) + 6 * pow(p(x), 3) - 4* pow(p(x-1),3))/6;
			}

			typedef typename Pixel::traits<T>::float_type FT;

		public:
			image_interpolate(const BasicImage<T>& i)
			:im(&i)
			{}

			bool in_image(const TooN::Vector<2>& pos) const
			{
				return pos[0] >= 1 && pos[1] >=1 && pos[0] < im->size().x-2 && pos[1] < im->size().y - 2;
			}

			FT operator[](const TooN::Vector<2>& pos) const
			{
				int x = (int)floor(pos[0]);
				int y = (int)floor(pos[1]);
				float dx = pos[0] - x;
				float dy = pos[1] - y;

				//Algorithm as described in http://astronomy.swin.edu.au/~pbourke/colour/bicubic/
				FT ret;
				for(unsigned int i=0; i < Pixel::Component<T>::count; i++)
				{
					float s=0;

					for(int m = -1; m < 3; m++)
						for(int n = -1; n < 3; n++)
							s += Pixel::Component<T>::get((*im)[y+n][x+m], i) * r(m - dx) * r(dy-n);
						
					Pixel::Component<FT>::get(ret, i)= s;
				}

				return ret;
			}

			TooN::Vector<2> min() const
			{
				return TooN::makeVector( 1, 1);
			}

			TooN::Vector<2> max() const
			{
				return TooN::makeVector( im->size().x - 2, im->size().y - 2);
			}

	};
	#endif

}

#endif

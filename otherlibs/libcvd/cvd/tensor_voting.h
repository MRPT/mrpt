#ifndef CVD_INC_TENSOR_VOTE_H
#define CVD_INC_TENSOR_VOTE_H

#include <cvd/image.h>
#include <TooN/TooN.h>
#include <TooN/helpers.h>
#include <vector>
#include <utility>

namespace CVD
{
	
	#ifndef DOXYGEN_IGNORE_INTERNAL
	namespace TensorVoting
	{
		struct TV_coord
		{
			std::ptrdiff_t o;
			int	x;
			int y;
		};

		std::vector<std::pair<TV_coord, TooN::Matrix<2> > > compute_a_tensor_kernel(int radius, double cutoff, double angle, double sigma, double ratio, int row_stride);
		inline unsigned int quantize_half_angle(double r, int num_divs)
		{
			return  ((int)floor((r/M_PI+100) * num_divs + 0.5)) % num_divs;
		}
	}

	#endif

	/**
	This function performs tensor voting on the gradients of an image. The
	voting is performed densely at each pixel, and the contribution of each
	pixel is scaled by its gradient magnitude. The kernel for voting is
	computed as follows.  Consider that there is a point at \f$(0,0)\f$, with
	gradient normal \f$(0,1)\f$. This will make a contribution to the point
	\f$(x,y)\f$.
	
	The arc-length, \f$l\f$, of the arc passing through \f$(0,0)\f$, tangent to
	the gradient at this point and also passing through \f$(x, y)\f$ is:
	\f[
		l = 2 r \theta
	\f]
	Where
	\f[
		\theta = \tan^{-1}\frac{y}{x}
	\f]
	and the radius of the arc, \f$r\f$ is:
	\f[
		r = \frac{x^2 + y^2}{2y}.
	\f]

	The scale of the contribution is:
	\f[
		s = e^{-\frac{l^2}{\sigma^2} - \kappa\frac{\sigma^2}{r^2}}.
	\f]
	Note that this is achieved by scaling \f$x\f$ and \f$y\f$ by \f$\sigma\f$, so
	\f$\kappa\f$ controls the kernel shape independent of the size.
	The complete tensor contribution is therefore:
	\f[
		e^{-\frac{l^2}{\sigma^2} - \kappa\frac{\sigma^2}{r^2}} 
							\left[
								\begin{array}{c}
									\cos 2\theta\\
									\sin 2\theta
								\end{array}
							\right]
							[ \cos 2\theta\ \ \sin 2\theta]
	\f]


	@param image    The image on which to perform tensor voting
	@param sigma    \f$ \sigma \f$
	@param ratio    \f$ \kappa \f$
	@param cutoff   When \f$s\f$ points drop below the cutoff, it is set to zero.
	@param num_divs The voting kernels are quantized by angle in to this many dicisions in the half-circle.
	@ingroup gVision
	**/
	template<class C> Image<TooN::Matrix<2> > dense_tensor_vote_gradients(const BasicImage<C>& image, double sigma, double ratio, double cutoff=0.001, unsigned int num_divs = 4096)
	{
		using TooN::Matrix;
		using std::pair;
		using std::vector;
		using TensorVoting::TV_coord;

		Matrix<2> zero(TooN::Zeros);
		Image<Matrix<2> > field(image.size(), zero);


		//Kernel values go as exp(-x*x / sigma * sigma)
		//So, for cutoff = exp(-x*x / sigma * sigma)
		//ln cutoff = -x*x / sigma*sigma
		//x = sigma * sqrt(-ln cutoff)
		int kernel_radius =  (int)ceil(sigma * sqrt(-log(cutoff)));


		//First, build up the kernels
		vector<vector<pair<TV_coord, Matrix<2> > > > kernels;
		for(unsigned int i=0; i < num_divs; i++)
		{
			double angle =  M_PI * i / num_divs;
			kernels.push_back(TensorVoting::compute_a_tensor_kernel(kernel_radius, cutoff, angle, sigma, ratio, field.row_stride()));
		}
		
		for(int y= kernel_radius; y < field.size().y - kernel_radius; y++)
			for(int x= kernel_radius; x < field.size().x - kernel_radius; x++)
			{
				double gx = ((double)image[y][x+1] - image[y][x-1])/2.;
				double gy = ((double)image[y+1][x] - image[y-1][x])/2.;

				double scale = sqrt(gx*gx + gy*gy);
				unsigned int direction = TensorVoting::quantize_half_angle(M_PI/2 + atan2(gy,gx), num_divs);

				const vector<pair<TV_coord, Matrix<2> > >& kernel = kernels[direction];

				Matrix<2>* p = &field[y][x];
				
				//The matrices are all symmetric, so only use the upper right triangle.
				for(unsigned int i=0; i < kernel.size(); i++)
				{
					p[kernel[i].first.o][0][0] += kernel[i].second[0][0] * scale;
					p[kernel[i].first.o][0][1] += kernel[i].second[0][1] * scale;
					p[kernel[i].first.o][1][1] += kernel[i].second[1][1] * scale;
				}
			}

		//Now do the edges
		for(int y= 1; y < field.size().y-1; y++)
		{
			for(int x= 1; x < field.size().x-1; x++)
			{
				//Skip the middle bit
				if(y >= kernel_radius && y < field.size().y - kernel_radius && x == kernel_radius)
					x = field.size().x - kernel_radius;

				double gx = ((double)image[y][x+1] - image[y][x-1])/2.;
				double gy = ((double)image[y+1][x] - image[y-1][x])/2.;

				double scale = sqrt(gx*gx + gy*gy);
				unsigned int direction = TensorVoting::quantize_half_angle(M_PI/2 + atan(gy / gx), num_divs);

				const vector<pair<TV_coord, Matrix<2> > >& kernel = kernels[direction];

				Matrix<2>* p = &field[y][x];
				
				//The matrices are all symmetric, so only use the upper right triangle.
				for(unsigned int i=0; i < kernel.size(); i++)
				{
					if(kernel[i].first.y+y >= 0 && kernel[i].first.y+y < field.size().y && kernel[i].first.x+x >= 0 && kernel[i].first.x+x < field.size().x)
					{
						p[kernel[i].first.o][0][0] += kernel[i].second[0][0] * scale;
						p[kernel[i].first.o][0][1] += kernel[i].second[0][1] * scale;
						p[kernel[i].first.o][1][1] += kernel[i].second[1][1] * scale;
					}
				}
			}
		}

		//Copy over bits to make the matrices symmetric
		for(Image<Matrix<2> >:: iterator i=field.begin(); i != field.end(); i++)
			(*i)[1][0] = (*i)[0][1];

		return field;
	}
	

	#ifdef CVD_EXPERIMENTAL

	template<class C> Image<TooN::Matrix<2> > dense_tensor_vote_gradients_fast(const BasicImage<C>& image, double sigma, double ratio, double cutoff=0.001, int num_divs = 4096)
	{
		using TooN::Matrix;
		using std::pair;
		using std::make_pair;
		using std::vector;
		using TensorVoting::TV_coord;

		Matrix<2> zero(TooN::Zeros);
		Image<Matrix<2> > ffield(image.size(), zero);
		Image<__m128> field(image.size());
		field.zero();

		
		//In much the same way as dense_tensor_vote_gradients, build up the kernel.
		int kernel_radius =  (int)ceil(sigma * sqrt(-log(cutoff)));
		vector<vector<pair<TV_coord, Matrix<2> > > > matrix_kernels;
		for(int i=0; i < num_divs; i++)
		{
			double angle =  M_PI * i / num_divs;
			matrix_kernels.push_back(TensorVoting::compute_a_tensor_kernel(kernel_radius, cutoff, angle, sigma, ratio, field.row_stride()));
		}


		//Put the kernel in aligned SSE registers.
		//Image<__m128> is used since it guarantees SSE aligned memory.
		vector<vector<int> > kernel_offsets;
		vector<Image<__m128> > kernel_values;
		for(unsigned int i=0; i < matrix_kernels.size(); i++)
		{
			vector<int>    off(matrix_kernels[i].size());
			Image<__m128>  val(ImageRef(matrix_kernels[i].size(), 1));

			for(unsigned int j=0; j < matrix_kernels[i].size(); j++)
			{
				off[j] = matrix_kernels[i][j].first.o;
				Matrix<2>& m = matrix_kernels[i][j].second;
				val.data()[j] = _mm_setr_ps(m[0][0], m[0][1], m[1][0], m[1][1]);
			}

			kernel_offsets.push_back(off);
			kernel_values.push_back(val);
		}

		#pragma omp parallel for
		for(int y= kernel_radius; y < field.size().y - kernel_radius; y++)
			for(int x= kernel_radius; x < field.size().x - kernel_radius; x++)
			{
				float gx = ((float)image[y][x+1] - image[y][x-1])/2.;
				float gy = ((float)image[y+1][x] - image[y-1][x])/2.;

				float scale = sqrt(gx*gx + gy*gy);
				unsigned int direction = TensorVoting::quantize_half_angle(M_PI/2 + atan(gy / gx), num_divs);

				const vector<int> & off = kernel_offsets[direction];
				__m128* val = kernel_values[direction].data();
				__m128* p = &field[y][x];
				__m128 s = _mm_set1_ps(scale);

				for(unsigned int i=0; i < off.size(); i++)
					p[off[i]] = _mm_add_ps(p[off[i]], _mm_mul_ps(val[i], s));
			}

		for(int y=0; y < field.size().y; y++)
			for(int x=0; x < field.size().x; x++)
			{
				float f[4];
				_mm_storeu_ps(f, field[y][x]);
				ffield[y][x][0][0] = f[0];
				ffield[y][x][0][1] = f[1];
				ffield[y][x][1][0] = f[2];
				ffield[y][x][1][1] = f[3];
			}
		
		return ffield;
	}
	#endif

}


#endif

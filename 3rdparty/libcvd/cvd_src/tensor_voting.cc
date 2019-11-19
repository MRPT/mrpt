#include <cvd/image.h>
#include <TooN/TooN.h>
#include <TooN/helpers.h>
#include <cvd/tensor_voting.h>
#include <utility>
#include <vector>

using namespace TooN;
using namespace std;

namespace CVD
{
	namespace TensorVoting
	{

		Matrix<2> rot(double angle)
		{
			Matrix<2> v;
			v[0] = makeVector(cos(angle), sin(angle));
			v[1] = makeVector(-sin(angle), cos(angle));
			return v;
		}
	
		//See the tensor voting documentation for a description of the maths
		pair<Matrix<2>, double> tensor_kernel_element(Vector<2>& g, int x1, int y1, double sigma, double ratio)
		{
			double x = x1 * g[0] + y1 * g[1];
			double y = x1 * g[1] - y1 * g[0];
			x /= sigma;
			y /= sigma;

			if(y == 0)
			{
				Matrix<2> t = g.as_col() * g.as_row();

				if(x ==0)
					return make_pair(t, 1);
				else
					return make_pair(t, exp(-(x * x)));
			}
			else
			{
				double k = 2 * y / (x*x + y*y);
				double r = 1/k;
				double theta = atan(y/x);
				double arclen = 2 * theta * r;

				double scale = exp(-(arclen * arclen + ratio * k*k));
				Vector<2> d = rot(2*theta) * g;

				return  make_pair(d.as_col() * d.as_row(), scale);
			}
		}
		
		//Borrowed from the tag library.
		template<class A, class B> struct refpair
		{
			A& a;
			B& b;
			refpair(A& aa, B& bb)
			:a(aa),b(bb)
			{}

			void operator=(const pair<A,B>& p)
			{
				a=p.first;
				b=p.second;
			}
		};

		template<class A, class B> refpair<A,B> rpair(A&aa, B&bb)
		{
			return refpair<A,B>(aa, bb);
		}

		TV_coord make(int x, int y, int s)
		{
			TV_coord t;
			t.x = x;
			t.y = y;
			t.o = (ptrdiff_t)s * y + x;
			return t;
		}
		
		//Compute a kernel, with small values set to zero, with pointer offsets
		//for the nonzero elements.
		vector<pair<TV_coord, Matrix<2> > > compute_a_tensor_kernel(int radius, double cutoff, double angle, double sigma, double ratio, int row_stride)
		{

			vector<pair<TV_coord, Matrix<2> > > ret;

			Vector<2> g;
			g[0] = cos(angle);
			g[1] = sin(angle);

			for(int y=-radius; y <= radius; y++)
				for(int x=-radius; x <= radius; x++)
				{
					Matrix<2> tensor;
					double scale;
					rpair(tensor, scale) = tensor_kernel_element(g, x, y, sigma, ratio);

					if(scale >= cutoff)
						ret.push_back(make_pair(make(x,y,row_stride), scale * tensor));
				}
			
			return ret;
		}
	}
}	


#include "cvd/utility.h"

namespace CVD
{
	void differences(const float* a, const float* b, float* diff, unsigned int size)
	{
		differences<float, float>(a, b, diff, size);
	}

	void add_multiple_of_sum(const float* a, const float* b, const float& c,  float* out, size_t count)
	{
		add_multiple_of_sum<float, float>(a, b, c, out, count);
	}


	void assign_multiple(const float* a, const float& c,  float* out, unsigned int count)
	{
		assign_multiple<float, float, float>(a, c, out, count);
	}

	double inner_product(const float* a, const float* b, unsigned int count)
	{
		return inner_product<float>(a, b, count);
	}

	double sum_squared_differences(const float* a, const float* b, size_t count)
	{
      return SumSquaredDifferences<double, float,float>::sum_squared_differences(a,b,count);
	}

	void square(const float* in, float* out, size_t count)
	{
		square<float, float>(in, out, count);
	}

	void subtract_square(const float* in, float* out, size_t count)
	{
		subtract_square<float, float>(in, out, count);
	}
}

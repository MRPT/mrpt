#include "cvd/utility.h"

namespace CVD {

    void differences(const int32_t* a, const int32_t* b, int32_t* diff, unsigned int size)
    {
		differences<int32_t, int32_t>(a, b, diff, size);
    }

    void differences(const double* a, const double* b, double* diff, unsigned int size)
    {
		differences<double, double>(a, b, diff, size);
    }

    void add_multiple_of_sum(const double* a, const double* b, const double& c,  double* out, unsigned int count)
    {
		add_multiple_of_sum<double, double>(a, b, c, out, count);
    }

    void assign_multiple(const double* a, const double& c,  double* out, unsigned int count)
    {
		assign_multiple<double, double>(a, c, out, count);
    }

    double inner_product(const double* a, const double* b, unsigned int count)
    {
		return inner_product<double>(a, b, count);
    }

    double sum_squared_differences(const double* a, const double* b, size_t count)
    {
		return SumSquaredDifferences<double,double,double>::sum_squared_differences(a,b,count);
    }

    long long sum_squared_differences(const byte* a, const byte* b, size_t count)
    {
		return SumSquaredDifferences<long long,int,byte>::sum_squared_differences(a,b,count);
    }
    
}

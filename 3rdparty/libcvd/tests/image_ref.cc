#include <cvd/image_ref.h>
#include <iostream>

using namespace std;
using namespace CVD;

constexpr ImageRef foo(ImageRef i)
{
	i += ImageRef(3,3);
	ImageRef j = i * 2;
	return j<<=1;
}

template<int I> struct Fun{};

int main()
{
	constexpr ImageRef i{1,2};
	cout << foo(i) << endl;

	Fun<i[0]>{};
	Fun<i.x>{};

	if(foo(i) != ImageRef(16,20))
		return 1;
}

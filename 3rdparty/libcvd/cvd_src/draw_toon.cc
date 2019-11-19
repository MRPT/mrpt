
#include <cvd/draw.h>
#include <utility>
#include <TooN/helpers.h>
using namespace TooN;
using namespace std;

namespace CVD {

static double len2(const Vector<2>& v)
{
	return v*v;
}	


vector<ImageRef> getSolidEllipse(float r1, float r2, float theta)
{
	vector<ImageRef> e;

	int r = (int) ceil(max(r1, r2) + 1);
	Matrix<2> t;
	t[0] =  makeVector(cos(theta), sin(theta)) / r1;
	t[1] =  makeVector(-sin(theta), cos(theta)) / r2;


	for(int y=-r; y <= r; y++)
		for(int x=-r; x <= r; x++)
			if(len2(t * makeVector(x, y)) <= 1)
				e.push_back(ImageRef(x, y));
	return e;
}

};

#include <cvd/colourspace.h>

namespace CVD
{
namespace ColourSpace
{

void yuv411_to_y(const unsigned char* ip, int npix, unsigned char* op)
{
	const unsigned char* ipe = ip + npix * 6 / 4;
	for(; ip < ipe;)
	{
		//Format is uYYvYY
		op[0] = ip[1];
		op[1] = ip[2];
		op[2] = ip[4];
		op[3] = ip[5];
		op += 4;
		ip += 6;
	}
}
void yuv411_to_rgb_y(const unsigned char* ip, int npix, unsigned char* op, unsigned char *opy)
{
	const unsigned char* ipe = ip + npix * 6 / 4;

	float  rv, guv, bu, u, v;
	unsigned char y1, y2, y3, y4;

	for(; ip < ipe;)
	{
		//Format is uYYvYY
		//Y, 8 but uchar, (u,v) 8 bit uchar-127
		u = *ip - 128;
		v = *(ip+3) - 128;

		rv = 1.140*v;
		guv = -0.394*u -0.581*v;
		bu = 2.028*u;

		y1 = ip[1];
		y2 = ip[2];
		y3 = ip[4];
		y4 = ip[5];

		#define LIM(X) (unsigned char)((X)>255?255:(X))

		op[0] = LIM(y1 + rv);
		op[1] = LIM(y1 + guv);
		op[2] = LIM(y1 + bu);
		opy[0] = y1;
	
		op[3] = LIM(y2 + rv);
		op[4] = LIM(y2 + guv);
		op[5] = LIM(y2 + bu);
		opy[1] = y2;

		op[6] = LIM(y3 + rv);
		op[7] = LIM(y3 + guv);
		op[8] = LIM(y3 + bu);
		opy[2] = y3;

		op[9] = LIM(y4 + rv);
		op[10]= LIM(y4 + guv);
		op[11]= LIM(y4 + bu);
		opy[3] = y4;

		op += 12;
		opy += 4;
		ip += 6;
	}
}

void yuv411_to_rgb(const unsigned char* ip, int npix, unsigned char* op)
{
	const unsigned char* ipe = ip + npix * 6 / 4;

	float  rv, guv, bu, u, v;
	unsigned char y1, y2, y3, y4;

	for(; ip < ipe;)
	{
		//Format is uYYvYY
		//Y, 8 but uchar, (u,v) 8 bit uchar-127
		u = *ip - 128;
		v = *(ip+3) - 128;

		rv = 1.140*v;
		guv = -0.394*u -0.581*v;
		bu = 2.028*u;

		y1 = ip[1];
		y2 = ip[2];
		y3 = ip[4];
		y4 = ip[5];

		#define LIM(X) (unsigned char)((X)>255?255:(X))

		op[0] = LIM(y1 + rv);
		op[1] = LIM(y1 + guv);
		op[2] = LIM(y1 + bu);
	
		op[3] = LIM(y2 + rv);
		op[4] = LIM(y2 + guv);
		op[5] = LIM(y2 + bu);

		op[6] = LIM(y3 + rv);
		op[7] = LIM(y3 + guv);
		op[8] = LIM(y3 + bu);

		op[9] = LIM(y4 + rv);
		op[10]= LIM(y4 + guv);
		op[11]= LIM(y4 + bu);

		op += 12;
		ip += 6;
	}
}

}
}

#include <cvd/distance_transform.h>
#include <random>
#include <cvd/image_io.h>
#include <algorithm>

using namespace CVD;
using namespace std;

int main()
{
	//Create a blank image.
	Image<byte> im(ImageRef(128, 128), 0);
		
	mt19937 engine;
	uniform_int_distribution<int> rand_x(0, im.size().x-1);
	uniform_int_distribution<int> rand_y(0, im.size().y-1);

	//Scatter down 7 points at random.
	for(int i=1; i < 8; i++)
		im[rand_y(engine)][rand_x(engine)]= i;

	Image<int> dt(im.size());
	Image<ImageRef> inverse_dt(im.size());
	
	//Perfom the distance transform
	euclidean_distance_transform_sq(im, dt, inverse_dt);
	
	//Create an output which is the distance transfom of the input,
	//but coloured according to which pixel is closest.
	int largest_distance = *max_element(dt.begin(), dt.end());

	Image<Rgb<byte> > out(im.size());

	for(int y=0; y < im.size().y; y++)
		for(int x=0; x < im.size().x; x++)
		{
			int c = floor(sqrt(dt[y][x]*1.0/largest_distance) * 255 + .5);

			Rgb<byte> r(0,0,0);
			if(im[inverse_dt[y][x]]&1)
				r.red = c;
			if(im[inverse_dt[y][x]]&2)
				r.green = c;
			if(im[inverse_dt[y][x]]&4)
				r.blue = c;

			out[y][x] = r;
		}


	img_save(out, "distance_transform_result.png");

}

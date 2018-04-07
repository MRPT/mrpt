#include <cvd/distance_transform.h>
#include <cvd/image_io.h>
#include <random>
#include <cvd/image_convert.h>
#include <utility>
using namespace CVD;
using namespace std;

int main()
{

    mt19937 engine;
	uniform_int_distribution<int> rand_size(50, 1050);
	//Generate some random point sets.
	for(int i=0; i < 100; i++)
	{
		ImageRef s(rand_size(engine), rand_size(engine));


		Image<byte> in(s);

		Image<Rgb<byte> > o(s);

		o.zero();
		in.zero();

		uniform_int_distribution<int> rand_x(0, in.size().x-1);
		uniform_int_distribution<int> rand_y(0, in.size().y-1);


		for(int i=1; i < 1 + rand()%10000; i++)
			in[rand_y(engine)][rand_x(engine)]= 1;


		Image<int> out(in.size());
		Image<ImageRef> adt(in.size());


		euclidean_distance_transform_sq(in, out, adt);
		
		//Make sure that the closest point matches the distance transform.
		//Note that sometimes it fails to get anything for the closest point
		//Also, note that this doesn't actually test to see if the closest point
		//AND distance are correct, just consistent.
		for(ImageRef p(-1,0); p.next(in.size()); )
		{
			if(adt[p] != ImageRef(-1, -1))
			{
				float d = ((p - adt[p]).mag_squared());

				if(abs(d - out[p]) != 0)
				{
					cerr << "Error.\n";
					cerr << d << endl;
					cout << out[p] << endl;
					cout << adt[p] << endl;
					return 1;
				}	
			}
		}
	}
}

#include <cvd/image_io.h>
#include <cvd/colourmap.h>
using namespace CVD;
using namespace std;

int main()
{
	typedef float C;
	Image<Rgb<C> > i(ImageRef(256,256));

	//Show three colourmaps side by side from low (top of image)
	//to high (bottom of image)
	for(int r=0; r < i.size().y; r++)
		for(int c=0; c < i.size().x; c++)
		{
			if(c < i.size().x / 3.)
				i[r][c] = Colourmap<Rgb<C> >::hot(r * 1.0/i.size().y);
			else if(c < i.size().x * 2. / 3.)
				i[r][c] = Colourmap<Rgb<C> >::jet(r * 1.0/i.size().y);
			else
				i[r][c] = Colourmap<Rgb<C> >::gkr(r * 1.0/i.size().y);
		}

	img_save(i, "colourmaps.png");
}

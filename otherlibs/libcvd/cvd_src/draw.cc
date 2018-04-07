
#include <cvd/draw.h>

namespace CVD {
std::vector<ImageRef> getCircle(int radius) {
    std::vector<ImageRef> points;
    int y = 0;
    for (int x=-radius; x<=0; x++) {
        int nexty2 = radius*radius - ((x+1)*(x+1));
        while (true) {
        	points.push_back(ImageRef(x,y));
        	if (y*y >= nexty2)
        	  break;
        	++y;
        }
    }
    size_t i;
    for (i=points.size()-1;i>0;i--)
        points.push_back(ImageRef(-points[i-1].x, points[i-1].y));
    for (i=points.size()-1;i>1;i--)
        points.push_back(ImageRef(points[i-1].x, -points[i-1].y));
    return points;
}

std::vector<ImageRef> getDisc(float radius)
{
    std::vector<ImageRef> points;

	int r = (int)ceil(radius + 1);

	for(ImageRef p(0,-r); p.y <= r; p.y++)
		for(p.x = -r; p.x <= r; p.x++)
			if(p.mag_squared() <= radius*radius)
				points.push_back(p);


	
	return points;
}

};

#include <cvd/morphology.h>

namespace CVD{
	
	namespace Internal{
		namespace MorphologyHelpers{
			//Split a list of ImageRefs up in to rows.
			vector<vector<ImageRef> > row_split(const vector<ImageRef>& v, int y_lo, int y_hi)
			{
				vector<vector<ImageRef> > rows(y_hi - y_lo + 1);

				for(unsigned int i=0; i < v.size(); i++)
					rows[v[i].y - y_lo].push_back(v[i]);

				return rows;
			}
		}
	}

	void morphology(const BasicImage<byte>& in, const std::vector<ImageRef>& selem, const Morphology::Median<byte>& m, BasicImage<byte>& out)
	{
		//If we happen to be given a 3x3 square, then perform
		//median filtering using the hand coded functions.
		if(selem.size() == 9)
		{
			std::vector<ImageRef> s = selem;
			std::sort(s.begin(), s.end());
			ImageRef box[9] = {
				ImageRef(-1, -1),
				ImageRef( 0, -1),
				ImageRef( 1, -1),
				ImageRef(-1,  0),
				ImageRef( 0,  0),
				ImageRef( 1,  0),
				ImageRef(-1,  1),
				ImageRef( 0,  1),
				ImageRef( 1,  1)};

			if(std::equal(s.begin(), s.end(), box))
			{
				median_filter_3x3(in, out);

				//median_filter_3x3 does not do the edges, so do the 
				//edges with a cropped kernel.

				using median::median4;
				using median::median6_row;
				using median::median6_col;
				out[0][0]                         = median4(in, 0, 0);
				out[0][in.size().x-1]             = median4(in, 0, in.size().x-2);
				out[in.size().y-1][0]             = median4(in, in.size().y-2, 0);
				out[in.size().y-1][in.size().x-1] = median4(in, in.size().y-2, in.size().x-2);

				for(int i=1; i < in.size().x-1; i++)
					out[0][i] = median6_row(in, 0, i-1);

				for(int i=1; i < in.size().x-1; i++)
					out[in.size().y-1][i] = median6_row(in, in.size().y-2, i-1);

				for(int i=1; i < in.size().y-1; i++)
					out[i][0] = median6_col(in, i-1, 0);

				for(int i=1; i < in.size().y-1; i++)
					out[i][in.size().x-1] = median6_col(in, i-1, in.size().x-2);
			}
			else
				morphology<Morphology::Median<byte>, byte >(in , selem, m, out);
		}
		else
			morphology<Morphology::Median<byte>, byte >(in , selem, m, out);
	}


}

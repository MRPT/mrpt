#include "cvd/connected_components.h"
#include <climits> 
#include <algorithm> 
#include <iterator>

using namespace std;
namespace CVD{
static unsigned int root_of(const vector<unsigned int>& parents, unsigned int n) 
{
	while(n != parents[n])
		n = parents[n];
	return n;
}

struct CompareFistIntLessThan
{
	bool operator()(const pair<int, int>& p1, const pair<int, int>& p2)
	{
		return p1.first < p2.first;
	}
};

void connected_components(const vector<ImageRef>& v, vector<vector<ImageRef> >& r)
{
	int current_label = -1;

	//This stores the initial labelling
	//of each pixel.
	vector<vector<ImageRef> > segments;
	
	r.clear();
	if(v.size() == 0)
		return;


	//This stores the connectivity graph, where
	//each element stores its parent. A node storing
	//its own parent is a root node.
	vector<unsigned int> parents;
	
	//This stores the horizontal position and label of the points in the row
	//above. 
	//There are several choices, map<int, int> can map a position to a label, 
	//as can a vector<int> (using a dense representation). An ordered vector<int, int>
	//is equivalent to a map, but much faster, since allocation is much easier, and 
	//points are always entered in order, so insertion is constant time.
	
	//vector<int, int> is slightly slower than vector<int> on some images, but
	//faster on others, especially those where objects are sparse, but rows are 
	//not.
	vector<pair<int, int> > row_above, row_curent;
	vector<pair<int, int> >::iterator begin;
	int prev_row=INT_MIN, prev_x = INT_MIN;
	
	for(unsigned int i=0; i < v.size(); i++)
	{
		ImageRef pos = v[i];
		
		if(pos.y != prev_row)
		{	
			prev_x=INT_MIN;
			row_above.swap(row_curent);
			row_curent.clear();

			if(pos.y-1 > prev_row)
			{
				row_above.clear();
			}

			//Put in a sentinal value to avoid some tests.
			row_above.push_back(make_pair(INT_MAX, -1));

			begin = row_above.begin();
		}


		//Look to see if there is a point above (4-way connectivity)
		//8 way would look above above-left and above-right
		int above_label=-1;

		//Check for contiguous regions first.
		if(begin->first == pos.x)
		{
			above_label = begin->second;
			begin++;
		}
		else
		{
			vector<pair<int, int> >::iterator above;
			above = lower_bound(begin, row_above.end(), make_pair(pos.x, 0), CompareFistIntLessThan());

			if(above->first == pos.x)
			{
				above_label = above->second;

				//There is no point searching from the beginning next time. We may
				//as well search from one past the current point.
				begin = above+1;
			}
			else
				begin = above;
		}

		//If there is nothing above or to the left,
		//then create a new label and corresponding segment.
		if(above_label == -1 && prev_x != pos.x-1)
		{
				parents.resize(parents.size()+1);
				current_label = parents.size()-1;
				parents[current_label] = current_label;

				segments.resize(segments.size() + 1);
		}
		else if(above_label != -1 && above_label != current_label)
		{
			//Parent is different, so take its label.
			//update the chain to the left if it exists.
			if(prev_x == pos.x - 1)
				current_label = parents[current_label] = root_of(parents, above_label);
			else
				current_label = root_of(parents, above_label);
		}


		segments[current_label].push_back(pos);

		row_curent.push_back(make_pair(pos.x, current_label));
		prev_row = v[i].y;
		prev_x = v[i].x;
	}

	//Concatenate all the connected segments.
	vector<unsigned int> components;


	for(unsigned int i=0; i < parents.size(); i++)
	{
		//Flatten
		parents[i] = root_of(parents, i);

		//Record the roots
		if(i == parents[i])
			components.push_back(i);
		else
		{
			//Or append points on to the root segment.
			copy(segments[i].begin(), segments[i].end(), back_inserter<vector<ImageRef> >(segments[parents[i]]));
		}
	}
	
	r.clear();
	r.resize(components.size());

	//Save all the root segments
	for(unsigned int i=0; i < components.size(); i++)
		r[i].swap(segments[components[i]]);

}
}

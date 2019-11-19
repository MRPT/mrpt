#include <iostream>
#include <cvd/connected_components.h>
#include <cvd/image.h>

using namespace std;
using namespace CVD;

int main()
{
	for(int i=0; i < (1<<9); i++)
	{
		cout << "------------------------\n" << i << endl;
		int j=i;
		vector<ImageRef> p;
		for(int y=0; y < 3; y++)
			for(int x=0; x < 3; x++)
			{
				if(j&1)
					p.push_back(ImageRef(x, y));
				j>>=1;
			}

		vector<vector<ImageRef> > v;
		connected_components(p, v);

		Image<char> im(ImageRef(3,3), '.');

		for(unsigned int j=0; j <v.size(); j++)
			for(unsigned int k=0; k <v[j].size(); k++)
				im[v[j][k]]=65+j;
		
		cout << "Created " << v.size() << " groups\n";
		cout.write(im.data(), 3);
		cout << endl;
		cout.write(im.data()+3, 3);
		cout << endl;
		cout.write(im.data()+6, 3);
		cout << endl;
		cout << endl;
		cin.get();
	}

}

#include <cvd/image_io.h>
#include <cvd/tensor_voting.h>
#include <cvd/colourmap.h>

#include <TooN/SymEigen.h>

#include <cxxopts.hpp>

#include <cstring>
#include <cerrno>

using namespace std;
using namespace CVD;
using namespace TooN;
using namespace cxxopts;


int main(int argc, char** argv)
{
	try
	{
		Options options("tensor_voting", "Apply tensor voting to an image.");

		options.add_options()
			("k,kappa", "kappa (larger gives straighter lines)", value<double>())
			("s,sigma", "sigma (larger givers longer kernels)", value<double>())
			("i,input", "input image (default is stdin)", value<string>())
			("o,output", "output image (default is stdout)", value<string>())
			("h,help", "Help.");

		options.parse(argc, argv);

		if(options.count("help"))
		{
			cout << options.help({"", "Group"}) << endl;
			return 0;
		}

		istream* is = &cin;
		ifstream fin;

		if(options.count("input"))
		{
			string fn =  options["input"].as<string>();
			fin.open(fn);
			if(!fin.good())
			{
				cerr << "Error opening \"" << fn << "\": " << strerror(errno) << endl;
				return 1;
			}
			is = &fin;
		}

		ostream* os = &cout;
		ofstream fout;
		ImageType::ImageType t = ImageType::PNM;

		if(options.count("output"))
		{
			string fn =  options["output"].as<string>();
			fout.open(fn);
			if(!fout.good())
			{
				cerr << "Error opening \"" << fn << "\": " << strerror(errno) << endl;
				return 1;
			}

			t = string_to_image_type(fn);

			os = &fout;
		}

		Image<float> im = img_load(*is);
		
		Image<Matrix<2>> tv = dense_tensor_vote_gradients(im, options["sigma"].as<double>(), options["kappa"].as<double>(), 0.02, 256);


		Image<float> r(im.size());
		float hi = 0;

		for(int y=0; y < r.size().y; y++)
			for(int x=0; x < r.size().x; x++)
			{	
				SymEigen<2> s(tv[y][x]);
				r[y][x] =  s.get_evalues()[1] - s.get_evalues()[0];
				hi = max(hi, r[y][x]);
			}

		Image<Rgb<byte>> out(r.size());

		for(int y=0; y < r.size().y; y++)
			for(int x=0; x < r.size().x; x++)
				out[y][x] = Colourmap<Rgb<byte>>::grey(r[y][x] / hi);



		img_save(out, *os, t);


	}
	catch(const cxxopts::OptionException& e)
	{
		cerr << "Error: " << e.what() << endl;
		return 1;
	}
	catch(Exceptions::All e)
	{
		cerr << "Error: " << e.what << endl;
		return 1;
	}
	
}


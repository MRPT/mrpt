#include <stdio.h>
#include <string.h>
#include "cvd/internal/io/cvdimage.h"

#include "cvd/image_io.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <setjmp.h>
#include <algorithm>
#include <set>
#include <climits>

#include <array>
#include <cstdint>

using namespace std;


namespace CVD
{
namespace CVDimage
{

// helper functions for prediction

// Use a simple linewise predictor and compute the residual differences for a
// single line. The predictor uses the value stored @p pred_len bytes in
// the past to predict the current value.
void pred_horizontal_diff(const byte* in, int width, byte* out, int pred_len = 1)
{
	for (int i = 0; i < pred_len; i++)
		out[i] = in[i];

	for (int i = pred_len; i < width; i++)
		out[i] = (256 + in[i] - in[i-pred_len])&255;
}

void pred_horizontal_undiff(const byte* in, int width, byte* out, int pred_len = 1)
{
	for (int i = 0; i < pred_len; i++)
		out[i] = in[i];

	for (int i = pred_len; i < width; i++)
		out[i] = (in[i] + out[i-pred_len])&255;
}

// Use a better 2d predictor where Gonzalez & Woods refer to a proof, stating
// that it is in some sense the best 2d predictor you can get. Again compute
// the residual differences for a single line of pixels. The predictor uses
// the three values stored @p pred_len bytes to the left and one line above
// the current pixel to predict the value. 
void pred_2doptimal_diff(const byte* in, int width, byte* buffer, byte* out, int pred_len = 1)
{
	// simple vertical prediction for first column(s)
	for (int i = 0; i < pred_len; i++) {
		out[i] = (256 + in[i] - buffer[i]) & 255;
	}

	// 0.75 i[x-1][y] + 0.75 i[x][y-1] - 0.5 i[x-1][y-1]
	// buffer is used to store the previous row
	for (int i = pred_len; i < width; i++) {
		int pred = (3*in[i-pred_len] - 2*buffer[i-pred_len] + 3*buffer[i]) / 4;
		out[i] = (256 + in[i] - pred)&255;
		buffer[i-pred_len] = in[i-pred_len];
	}

	// fix remaining values in the buffer
	for (int i = width-pred_len; i < width; i++) {
		buffer[i] = in[i];
	}
}

void pred_2doptimal_undiff(const byte* in, int width, byte* buffer, byte* out, int pred_len = 1)
{
	// vertical prediction for first column(s)
	for (int i = 0; i < pred_len; i++) {
		out[i] = (in[i] + buffer[i]) & 255;
	}

	// see above
	for (int i = pred_len; i < width; i++) {
		int pred = (3*out[i-pred_len] - 2*buffer[i-pred_len] + 3*buffer[i]) / 4;
		out[i] = (in[i] + pred)&255;
		buffer[i-pred_len] = out[i-pred_len];
	}

	// fix remaining values in the buffer
	for (int i = width-pred_len; i < width; i++) {
		buffer[i] = out[i];
	}
}

// Use a 2d predictor for Bayer lines of pixels. In case @p greenfirst is set,
// the row is assumed to be of the form gxgx, otherwise it is xgxg. The
// predictor uses two rows above the current pixel and two columns to the
// left.

// see King-Hong Chung and Yuk-Hee Chan. A Lossless Compression Scheme for
// Bayer Color Filter Array Images. IEEE Transactions on Image Processing, 2007.
// (i don't trust that paper!)
//static const int pred_coeff_green[]={5, 2, 1}; => 24.5549

//static const int pred_coeff_green[]={3, 5, 0}; => 24.4918
//static const int pred_coeff_green[]={6, -4, 6}; => 24.9409
//static const int pred_coeff_green[]={1, 6, 1}; //=> 24.4494
static const int pred_coeff_green[]={2, 4, 2}; //=> 24.3723
//static const int pred_coeff_bluered[]={8, 0, 0}; //=> 24.6116
//static const int pred_coeff_bluered[]={6, -4, 6}; //=> 24.3723
//static const int pred_coeff_bluered[]={3, 2, 3}; //=> 24.3034
//static const int pred_coeff_bluered[]={5, -2, 5}; //=> 24.2141
static const int pred_coeff_bluered[]={4, 0, 4}; //=> 24.1666

// Ideally one would learn the best coefficients from the autocorrelation
// matrix as done in: S Andriani, G Calvagno, D Menon. Lossless Compression of
// Bayer Mask Images using an Optimal Vector Prediction Technique. EUSIPCO 2006.
// Unfortunaly they don't give their final coefficients.

void pred_2dbayer_diff(const byte* in, int width, byte* buffer1, byte* buffer2, byte* out, bool greenfirst)
{
	// simple vertical prediction for first two columns
	out[0] = (256 + in[0] - buffer1[0]) & 255;
	out[1] = (256 + in[1] - buffer1[1]) & 255;

	// for red and blue: 0.75 i[x-2][y] + 0.75 i[x][y-2] - 0.5 i[x-2][y-2]
	// for green: 0.25 i[x-2][y] + 0.5 i[x-1][y-1] - 0.25 i[x+1][y-1]
	// buffer1 is used to store the pre-previous row
	// buffer2 is used to store the previous row
	int i=2;
	if (greenfirst) {
		int pred = (pred_coeff_green[0]*in[i-2] +
			pred_coeff_green[1]*buffer2[i-1] +
			pred_coeff_green[2]*buffer1[i]) / 8;
		out[i] = (256 + in[i] - pred)&255;
		buffer1[i-2] = in[i-2];
		i++;
	}

	for (; i < width; i++) {
		int pred = (pred_coeff_bluered[0]*in[i-2] +
			pred_coeff_bluered[1]*buffer1[i-2] +
			pred_coeff_bluered[2]*buffer1[i]) / 8;
		out[i] = (256 + in[i] - pred)&255;
		buffer1[i-2] = in[i-2];
		i++;
		if (!(i<width)) break;
		pred = (pred_coeff_green[0]*in[i-2] +
			pred_coeff_green[1]*buffer2[i-1] +
			pred_coeff_green[2]*buffer1[i]) / 8;
		out[i] = (256 + in[i] - pred)&255;
		buffer1[i-2] = in[i-2];
	}

	buffer1[width-2] = in[width-2];
	buffer1[width-1] = in[width-1];
}

void pred_2dbayer_undiff(const byte* in, int width, byte* buffer1, byte* buffer2, byte* out, bool greenfirst)
{
	// simple vertical prediction for first two columns
	out[0] = (in[0] + buffer1[0]) & 255;
	out[1] = (in[1] + buffer1[1]) & 255;

	// see above
	int i=2;
	if (greenfirst) {
		int pred = (pred_coeff_green[0]*out[i-2] +
			pred_coeff_green[1]*buffer2[i-1] +
			pred_coeff_green[2]*buffer1[i]) / 8;
		out[i] = (in[i] + pred)&255;
		buffer1[i-2] = out[i-2];
		i++;
	}

	for (; i < width; i++) {
		int pred = (pred_coeff_bluered[0]*out[i-2] +
			pred_coeff_bluered[1]*buffer1[i-2] +
			pred_coeff_bluered[2]*buffer1[i]) / 8;
		out[i] = (in[i] + pred)&255;
		buffer1[i-2] = out[i-2];
		i++;
		if (!(i<width)) break;
		pred = (pred_coeff_green[0]*out[i-2] +
			pred_coeff_green[1]*buffer2[i-1] +
			pred_coeff_green[2]*buffer1[i]) / 8;
		out[i] = (in[i] + pred)&255;
		buffer1[i-2] = out[i-2];
	}

	buffer1[width-2] = out[width-2];
	buffer1[width-1] = out[width-1];
}

// helper functions for huffman coding

struct Huff{
	Huff* zero, *one, *parent;
	int symbol;
	size_t count;

	bool operator()(const Huff* l, const Huff* r) const
	{
		if(l->count < r->count)
			return 1;
		else if(l->count == r->count)
			return l->symbol < r->symbol;
		else
			return 0;
	}
};


// This function creates a histogram of the element counts. The largest #
// count is always 65535 and the smallest nonzero count is at least 1. 0 means
// no counts, so it is ignored in the huffman tree. This way, all the data
// required to rebuild the tree can be stored in 512 bytes.
void create_normalized_hist(const Image<byte>& im, array<size_t,256>& h)
{
        fill(h.begin(), h.end(), 0);

	for(Image<byte>::const_iterator i = im.begin(); i!=im.end(); i++)
		h[*i]++;

	int hi = *max_element(h.begin(), h.end());
	for(unsigned int i=0; i < h.size(); i++)
		if(h[i])
			h[i]  = (((uint64_t)h[i])* 65534 / hi)+1;
}




// Given a histogram of for the symbols to encode, create a tree for a
// corresponding Huffman code
auto create_tree(const array<size_t,256>& h, vector<Huff*>& symbols)
{
	set<Huff*,Huff> table;
	vector<Huff> all;
	//We're going to be using pointers so reserve enough
	//to avoid a resize.
	all.reserve(h.size() * 2 -1);


	for(int i=0; i < 256; i++)
	{
		if(h[i])
		{
			Huff s = {0,0,0,i,h[i]};
			all.emplace_back(s);
			Huff* ss = &all.back();
			table.insert(ss);
			symbols.push_back(ss);
		}
	}

	//Starting negative and incrementing makes equal probability
	//symbols appear in symbol order. There are at most 2x the number of 
	//junk symbols as real ones.
	int junk_symbol=INT_MIN;
	while(table.size() > 1)
	{

		Huff* smallest = *table.begin();
		table.erase(table.begin());

		Huff* next_smallest = *table.begin();
		table.erase(table.begin());

		Huff s = {smallest, next_smallest, 0, junk_symbol++, smallest->count + next_smallest->count};
		all.emplace_back(s);
		Huff* ss = &all.back();
		
		smallest->parent = ss;
		next_smallest->parent=ss;
		table.insert(ss);
	}
	return make_pair(*table.begin(), move(all));
}


typedef uint16_t PackType;
static const int PackBits = sizeof(PackType) * 8;

enum cvd_predictors {
	PRED_HORIZONTAL = 0,
	PRED_2D_OPTIMAL = 2,
	PRED_2D_BAYER_1 = 3,
	PRED_2D_BAYER_2 = 4,
};

struct SortIndex
{
	const array<size_t, 256>& d;

	SortIndex(const array<size_t,256>& aa)
	:d(aa)
	{}

	bool operator()(int a, int b) const
	{
		return d[a] > d[b];
	}
	
};

// given a image (or "some data") and a histogram of the contained symbols,
// create a Huffman tree, encode the data and return the new code
vector<PackType> huff_compress(const Image<byte>& im, const array<size_t,256>& h)
{
	//Create a Huffman compression tree
	vector<Huff*> terminals;

	vector<Huff> all;
	Huff* table;
	tie(table, all) = create_tree(h, terminals);

	//Create the symbols for the tree and store them
	//rather inefficiently in an array, one bit per entry.
	vector<vector<byte> > symbols(256);
	for(unsigned int i=0; i < terminals.size(); i++)
	{
		vector<byte> bits;

		Huff* h = terminals[i];

		int symbol = h->symbol;

		while(h != table)
		{
			Huff* parent = h->parent;

			if(h == parent->one)
				bits.push_back(1);
			else
				bits.push_back(0);
			
			h = h->parent;
		}
		
		reverse(bits.begin(), bits.end());
		symbols[symbol] = bits;

	}

	//Convert the symbols in to a bit packed form.
	//The symbols are packed in to chunks of PackType (uint16_t)
	//For each of the 256 symbols, store the symbol 16 different times with 
	//starting offset by 16 different shifts.

	//This allows the symbols to be efficiently stuffed in to the stream later.
	array<array<array<PackType, 20>, PackBits>, 256> fast_symbols;
	array<array<int, PackBits>, 256> fast_symbols_num_chunks;

	for(unsigned int i=0; i < symbols.size(); i++)
		if(symbols[i].size())
		{
			for(int off=0; off < PackBits; off++)
			{
				fast_symbols_num_chunks[i][off] = 0;

				int o=off;
				PackType chunk = 0;
				for(unsigned int j=0; j < symbols[i].size(); j++)
				{
					chunk |= ((PackType)symbols[i][j]) << (o%PackBits);
					o++;
					if(o % PackBits == 0)
					{
						fast_symbols[i][off][fast_symbols_num_chunks[i][off]++] = chunk;
						chunk = 0;
					}
				}
				
				if(o % PackBits)
					fast_symbols[i][off][fast_symbols_num_chunks[i][off]++] = chunk;
			}
		}

	
	//Now pack the symbols into the array
	vector<PackType> r2;
	r2.reserve(im.size().area()/(2*PackBits));
	int bit=0;
	for(Image<byte>::const_iterator i = im.begin(); i!=im.end(); i++)
	{
		const int off = bit % PackBits;
		
		//Deal with the first (unaligned byte)
		if(off == 0)
			r2.push_back(fast_symbols[*i][0][0]);
		else
			r2.back() |= fast_symbols[*i][off][0];

		//Deal with any remaining bytes
		for(int j=1; j < fast_symbols_num_chunks[*i][off]; j++)
			r2.push_back(fast_symbols[*i][off][j]);

		bit += symbols[*i].size();
	}


	return r2;
}


// given an encoded data stream and a histogram of the encoded symbols, create
// a Huffman tree, decode the data and store it in the image ret.
// No particular effort has been paid to efficiency.
template<class P> void huff_decompress(const vector<P>& b, const array<size_t,256>& h, Image<byte>& ret)
{
	vector<Huff*> terminals;

	vector<Huff> all;
	Huff* table;
	tie(table, all) = create_tree(h, terminals);

	int i=0;	
	for(Image<byte>::iterator r=ret.begin(); r != ret.end(); r++)
	{
		Huff* h = table;

		while(h->one)
		{
			bool bit =  b[i/(sizeof(P) * 8)] & ((P)1 << (i%(sizeof(P) * 8)));
			i++;
			if(bit)
				h = h->one;
			else
				h = h->zero;
		}
		
		*r = h->symbol;	
	}
}




class ReadPimpl
{
	public:
		ReadPimpl(std::istream&);
		long  x_size() const {return xs;}
		long  y_size() const {return ys;}
		void get_raw_pixel_lines(unsigned char*, unsigned long nlines);
		~ReadPimpl();
		string datatype()
		{
			return type;
		}
		
		template<class T> void get_raw_pixel_line(T* d)
		{
			if(datatype() != PNM::type_name<T>::name())
				throw CVD::Exceptions::Image_IO::ReadTypeMismatch(datatype(), PNM::type_name<T>::name());

			get_raw_pixel_lines((unsigned char*)d, 1);
		}

	private:
		void read_header(std::istream& is);
		array<size_t, 256> read_hist(std::istream& is);
		vector<PackType> read_data(std::istream& is);
		void bayer_swap_rows(void);

		long xs, ys;
		int bypp; // bytes per pixel
		int pred_len;
		enum cvd_predictors pred_mode;
		string type;
		Image<byte> diff;
		int row;
		vector<byte> buffer, buffer2;
};

ImageRef reader::size()
{
	return ImageRef(t->x_size(), t->y_size());
}

void reader::get_raw_pixel_line(unsigned char* d)
{
	t->get_raw_pixel_line(d);
}

void reader::get_raw_pixel_line(bayer_bggr* d)
{
	t->get_raw_pixel_line(d);
}

void reader::get_raw_pixel_line(bayer_rggb* d)
{
	t->get_raw_pixel_line(d);
}

void reader::get_raw_pixel_line(bayer_grbg* d)
{
	t->get_raw_pixel_line(d);
}

void reader::get_raw_pixel_line(bayer_gbrg* d)
{
	t->get_raw_pixel_line(d);
}

void reader::get_raw_pixel_line(Rgb<byte>* d)
{
	t->get_raw_pixel_line(d);
}

void reader::get_raw_pixel_line(Rgba<byte>* d)
{
	t->get_raw_pixel_line(d);
}

string reader::datatype()
{
	return t->datatype();
}
string reader::name()
{
	return "CVD";
}
bool reader::top_row_first()
{
	return true;
}

reader::~reader()
{}

reader::reader(std::istream& i)
:t(new ReadPimpl(i))
{}


ReadPimpl::ReadPimpl(istream& in)
{
	pred_mode = PRED_HORIZONTAL;
	row = 0;
	read_header(in);
	array<size_t,256> h = read_hist(in);

	diff.resize(ImageRef(xs*bypp,ys));
	buffer.resize(xs*bypp);
	if ((pred_mode==PRED_2D_BAYER_1)||(pred_mode==PRED_2D_BAYER_2))
		buffer2.resize(xs*bypp);

	vector<PackType> d = read_data(in);
	huff_decompress(d, h, diff);
}

void ReadPimpl::read_header(istream& in)
{
	string tmp;
	getline(in, tmp);
	//cout << "loading: header-id: '" << tmp << "'" << endl;
	if (tmp != "CVD") throw CVD::Exceptions::Image_IO::MalformedImage(string("Error in CVD image: incorrect header ID"));

	// get data type
	getline(in, type);
	//cout << "loading: type: '" << type << "'" << endl;

	// get image dimensions
	in >> xs >> ys;
	//cout << "loading: size: " << xs << "x" << ys << endl;

	// get extra information, comments, compression options
	getline(in, tmp);
	//cout << "loading: extras: '" << tmp << "'" << endl;
	size_t pos = tmp.find("pred=");
	if (pos!=tmp.npos) {
		istringstream sstr(tmp.substr(pos+5));
		int pmode=-1;
		sstr >> pmode;
		pred_mode = (cvd_predictors) pmode;
	}

	//cout << "type: '" << type << "'" << endl;
	if (type== "unsigned char")
		bypp = pred_len = 1;
	else if (type== "CVD::Rgb<unsigned char>")
		bypp = pred_len = 3;
	else if (type== "CVD::Rgba<unsigned char>")
		bypp = pred_len = 4;
	else if ((type== "bayer_bggr")||(type== "bayer_rggb")) {
		bypp = 1; pred_len = 2;
		if ((pred_mode==PRED_2D_OPTIMAL)||(pred_mode==PRED_2D_BAYER_1))
			pred_mode = PRED_2D_BAYER_2;
	} else if ((type== "bayer_grbg")||(type== "bayer_gbrg")) {
		bypp = 1; pred_len = 2;
		if ((pred_mode==PRED_2D_OPTIMAL)||(pred_mode==PRED_2D_BAYER_2))
			pred_mode = PRED_2D_BAYER_1;
	} else throw CVD::Exceptions::Image_IO::MalformedImage(string("Error in CVD image: unknown data type"));
}

array<size_t, 256> ReadPimpl::read_hist(std::istream& is)
{
	array<size_t, 256> h;
	for (unsigned int i = 0; i < h.size(); i++) {
		h[i] = ((is.get() & 255)<<8) | (is.get()&255);
	}
	return h;
}

vector<PackType> ReadPimpl::read_data(std::istream& is)
{
	vector<PackType> data;
	while ((!is.bad())&&(!is.eof())) {
		int bits = PackBits;
		PackType tmp = 0;
		while (bits>0) {
			tmp <<= 8;
			tmp |= is.get()&255;
			bits-=8;
		}
		data.push_back(tmp);
	}
	return data;
}

void ReadPimpl::get_raw_pixel_lines(unsigned char*data, unsigned long nlines)
{
	switch (pred_mode) {
	case PRED_HORIZONTAL:
		for(unsigned int i=0; i < nlines; i++)	{
			pred_horizontal_undiff(diff[row], xs*bypp, data, pred_len);
			data += xs*bypp;
			row++;
		}
		break;
	case PRED_2D_OPTIMAL:
		if (row==0) {		
			pred_horizontal_undiff(diff[row], xs*bypp, data, pred_len);
			memcpy(buffer.data(), data, xs*bypp);
			data += xs*bypp;
			row++;
			nlines--;
		}
		for(unsigned int i=0; i < nlines; i++)	{
			pred_2doptimal_undiff(diff[row], xs*bypp, buffer.data(), data, pred_len);
			data += xs*bypp;
			row++;
		}
		break;
	case PRED_2D_BAYER_1:
	case PRED_2D_BAYER_2:
		//first two rows use horizontal prediction
		while (row<=1) {
			pred_horizontal_undiff(diff[row], xs*bypp, data, pred_len);
			memcpy(buffer.data(), data, xs*bypp);
			bayer_swap_rows();
                        data += xs*bypp;
                        row++;
			nlines--;
			if (nlines==0) break;
		}

		// regular lines
		while (nlines>0) {
			pred_2dbayer_undiff(diff[row], xs*bypp, buffer.data(), buffer2.data(), data, (pred_mode==PRED_2D_BAYER_1));
			bayer_swap_rows();
			data += xs*bypp;
			row++;
			nlines--;
		}
	}
}

void ReadPimpl::bayer_swap_rows(void)
{
	swap(buffer, buffer2);
	if (pred_mode==PRED_2D_BAYER_1) pred_mode=PRED_2D_BAYER_2;
	else if (pred_mode==PRED_2D_BAYER_2) pred_mode=PRED_2D_BAYER_1;
}

ReadPimpl::~ReadPimpl()
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  Compression 
//

////////////////////////////////////////////////////////////////////////////////
//
// CVD writing.
//

class WritePimpl
{
	public:
		WritePimpl(std::ostream&, int  xsize, int ysize, const string& type);
		long x_size() const {return xs;}
		long y_size() const {return ys;}
		void write_raw_pixel_lines(const unsigned char*, unsigned long);
		template<class C> 	void write_raw_pixel_line(const C*);
		~WritePimpl();
		
	private:
		void write_header(std::ostream& os);
		void write_hist(std::ostream& os, const array<size_t, 256>& h);
		void write_data(std::ostream& os, vector<PackType>& data);
		void bayer_swap_rows(void);

		long	xs, ys, row;
		int	bypp; // bytes per pixel :)
		int	pred_len;
		enum cvd_predictors pred_mode;
		std::ostream& 	o;
		string type;
		Image<byte> diff;
		byte *buffer, *buffer2; //size unknown at compile time
};



WritePimpl::WritePimpl(std::ostream& out, int xsize, int ysize, const string& t)
:o(out)
{
	xs = xsize;
	ys = ysize;
	type = t;
	row=0;
	pred_mode = PRED_2D_OPTIMAL;
//	pred_mode = PRED_HORIZONTAL;
	
	if(type == "unsigned char")
		bypp = pred_len = 1;
	else if(type == "CVD::Rgb<unsigned char>")
		bypp = pred_len = 3;
	else if(type == "CVD::Rgba<unsigned char>")
		bypp = pred_len = 4;
	else if((type == "bayer_bggr")||(type == "bayer_rggb"))  {
		bypp = 1; pred_len = 2;
		if (pred_mode==PRED_2D_OPTIMAL) pred_mode = PRED_2D_BAYER_2;
	} else if((type == "bayer_grbg")||(type == "bayer_gbrg"))  {
		bypp = 1; pred_len = 2;
		if (pred_mode==PRED_2D_OPTIMAL) pred_mode = PRED_2D_BAYER_1;
	} else
		throw Exceptions::Image_IO::UnsupportedImageSubType("CVDimage", type);

	diff.resize(ImageRef(xs*bypp,ys));
	buffer = new byte[xs*bypp];
	if ((pred_mode==PRED_2D_BAYER_1)||(pred_mode==PRED_2D_BAYER_2))
		buffer2 = new byte[xs*bypp];
	else
		buffer2 = NULL;
}

void WritePimpl::write_raw_pixel_lines(const unsigned char* data, unsigned long nlines)
{
	if(nlines + row > (unsigned long) ys)
		throw CVD::Exceptions::Image_IO::InternalLibraryError("CVD", "Write past end of image.");
	if (nlines==0) return;

	switch (pred_mode) {
	case PRED_HORIZONTAL:
		for (unsigned int i=0; i < nlines; i++)	{
			pred_horizontal_diff(data, xs*bypp, diff[row], pred_len);
			data += xs*bypp;
			row++;
		}
		break;
	case PRED_2D_OPTIMAL:
		if (row==0) {
			pred_horizontal_diff(data, xs*bypp, diff[row], pred_len);
			memcpy(buffer, data, xs*bypp);
                        data += xs*bypp;
                        row++;
			nlines--;
		}
		for (unsigned int i=0; i < nlines; i++)	{
			pred_2doptimal_diff(data, xs*bypp, buffer, diff[row], pred_len);
			data += xs*bypp;
			row++;
		}
		break;
	case PRED_2D_BAYER_1:
	case PRED_2D_BAYER_2:
		//first two rows use horizontal prediction
		while (row<=1) {
			pred_horizontal_diff(data, xs*bypp, diff[row], pred_len);
			memcpy(buffer, data, xs*bypp);
			bayer_swap_rows();
                        data += xs*bypp;
                        row++;
			nlines--;
			if (nlines==0) break;
		}

		// regular lines
		while (nlines>0) {
			pred_2dbayer_diff(data, xs*bypp, buffer, buffer2, diff[row], (pred_mode==PRED_2D_BAYER_1));
			bayer_swap_rows();
			data += xs*bypp;
			row++;
			nlines--;
		}
	}
	
}

template<class C> 	void WritePimpl::write_raw_pixel_line(const C*d)
{
	if(type != PNM::type_name<C>::name())
		throw CVD::Exceptions::Image_IO::WriteTypeMismatch(type, PNM::type_name<C>::name());

	write_raw_pixel_lines((const unsigned char*)d, 1); 
}

void WritePimpl::bayer_swap_rows(void)
{
	byte *tmp = buffer;
	buffer = buffer2;
	buffer2 = tmp;
	if (pred_mode==PRED_2D_BAYER_1) pred_mode=PRED_2D_BAYER_2;
	else if (pred_mode==PRED_2D_BAYER_2) pred_mode=PRED_2D_BAYER_1;
}

void WritePimpl::write_header(std::ostream& os)
{
	os << "CVD\n" << type << "\n" << xs << " " << ys;
	if (pred_mode==PRED_2D_BAYER_2) pred_mode = PRED_2D_BAYER_1;
	if (pred_mode!=PRED_HORIZONTAL) os << " pred=" << (int)pred_mode;
	os << "\n";
}

void WritePimpl::write_hist(std::ostream& os, const array<size_t, 256>& h)
{
	// have to go through the data anyway, but one might want to copy the
	// two least significant bytes out of each int, store them in a new
	// array, and dump that in just one call to ostream::write()
	for (unsigned int i = 0; i < h.size(); i++) {
		os.put((h[i]>>8)&255);
		os.put(h[i]&255);
	}
}

void WritePimpl::write_data(std::ostream& os, vector<PackType>& data)
{
	// this variant should be safe, but requires one separate call to
	// ostream::put() for each byte
	for (vector<PackType>::const_iterator it = data.begin(); it!=data.end(); ++it) {
		int bits = PackBits;
		while (bits>0) {
			os.put(((*it)>>(bits-8))&255);
			bits-=8;
		}
	}

	// this variant should be faster, but relies on PackType to be 16bit
	//for (vector<PackType>::iterator it = data.begin(); it!=data.end(); ++it) *it = htons(*it);
	//os.write((const char*)(&(data[0])), data.size()*PackBits/8);
}

WritePimpl::~WritePimpl()
{
	delete[] buffer;
	if (buffer2!=NULL) delete[] buffer2;
	write_header(o);

	array<size_t, 256> h;
        create_normalized_hist(diff, h);
	write_hist(o, h);

	vector<PackType> b = huff_compress(diff, h);
	write_data(o, b);
}


////////////////////////////////////////////////////////////////////////////////
//
// Public interfaces to image writing.
//

writer::writer(ostream& o, ImageRef size, const string& s, const std::map<std::string, Parameter<> >&)
:t(new WritePimpl(o, size.x, size.y, s))
{}

writer::~writer()
{}

void writer::write_raw_pixel_line(const byte* data)
{
	t->write_raw_pixel_line(data);
}

void writer::write_raw_pixel_line(const bayer_bggr* data)
{
	t->write_raw_pixel_line(data);
}

void writer::write_raw_pixel_line(const bayer_rggb* data)
{
	t->write_raw_pixel_line(data);
}

void writer::write_raw_pixel_line(const bayer_grbg* data)
{
	t->write_raw_pixel_line(data);
}

void writer::write_raw_pixel_line(const bayer_gbrg* data)
{
	t->write_raw_pixel_line(data);
}

void writer::write_raw_pixel_line(const Rgb<byte>* data)
{
	t->write_raw_pixel_line(data);
}

void writer::write_raw_pixel_line(const Rgba<byte>* data)
{
	t->write_raw_pixel_line(data);
}

}
}

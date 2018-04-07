#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/colourspaces.h>
#include <cvd/colourspace_convert.h>
#include <cvd/image_io.h>

#include <iostream>
#include <sstream>

using namespace std;
using namespace CVD;

template <typename T>
Image<T> create_bayer( const ImageRef & size, const BasicImage<byte> & pattern ){
    assert(pattern.size() == ImageRef(2,2));
    Image<T> result(size);
    for( int y = 0; y < size.y; y += 2 ){
        for(int x = 0; x < size.x; x +=2 ){
            BasicImage<T> sub = result.sub_image(ImageRef(x,y), ImageRef(2,2));
            reinterpret_cast<BasicImage<byte> &>(sub).copy_from(pattern);
        }
    }
    return result;
}

template <typename T>
inline ostream & operator<<( ostream & out, const Image<T> & im ){
    const ImageRef & size = im.size();
    for( int y = 0; y < size.y; y += 1 ){
        for(int x = 0; x < size.x; x +=1 ){
            out << int(im[y][x]) << " ";
        }
        out << endl;
    }
    return out;
}

template <typename T>
Image<Rgb<byte> > create_test( const ImageRef & size, const Image<byte> & red, const Image<byte> & green, const Image<byte> & blue ){
    
    Image<Rgb<byte> > result(size.dot_times(ImageRef(3,2)));
    
    Image<byte> gray = create_bayer<byte>(size, red);
    Image<T> bayer = create_bayer<T>(size, red);
    
    result.sub_image(ImageRef(0,0).dot_times(size), size).copy_from(convert_image<Rgb<byte> >(gray));
    result.sub_image(ImageRef(0,1).dot_times(size), size).copy_from(convert_image<Rgb<byte> >(bayer));
    
    gray = create_bayer<byte>(size, green);
    bayer = create_bayer<T>(size, green);

    result.sub_image(ImageRef(1,0).dot_times(size), size).copy_from(convert_image<Rgb<byte> >(gray));
    result.sub_image(ImageRef(1,1).dot_times(size), size).copy_from(convert_image<Rgb<byte> >(bayer));

    gray = create_bayer<byte>(size, blue);
    bayer = create_bayer<T>(size, blue);
    
    result.sub_image(ImageRef(2,0).dot_times(size), size).copy_from(convert_image<Rgb<byte> >(gray));
    result.sub_image(ImageRef(2,1).dot_times(size), size).copy_from(convert_image<Rgb<byte> >(bayer));
    
    return result;
}

int main(int argc, char ** argv){

    const ImageRef size(8,8);
    const byte value = (argc>1)?atoi(argv[1]):255;
    
    ostringstream sout;
    sout << int(value) << ".png";
    const string postfix = sout.str();
    
    Image<byte> red(ImageRef(2,2));
    Image<byte> green(ImageRef(2,2));
    Image<byte> blue(ImageRef(2,2));

    // bayer rggb
    // RGRGRG
    // GBGBGB
    // RGRGRG
    // GBGBGB
    red[0][0] = value;
    red[0][1] = 0;
    red[1][0] = 0;
    red[1][1] = 0;

    green[0][0] = 0;
    green[0][1] = value;
    green[1][0] = value;
    green[1][1] = 0;

    blue[0][0] = 0;
    blue[0][1] = 0;
    blue[1][0] = 0;
    blue[1][1] = value;

    Image<Rgb<byte> > rggb = create_test<bayer_rggb>(size, red, green, blue);
    // img_save(rggb, "rggb" + postfix);

    // bayer gbrg
    // GBGBGB
    // RGRGRG
    // GBGBGB
    // RGRGRG
    
    red[0][0] = 0;
    red[0][1] = 0;
    red[1][0] = value;
    red[1][1] = 0;

    green[0][0] = value;
    green[0][1] = 0;
    green[1][0] = 0;
    green[1][1] = value;

    blue[0][0] = 0;
    blue[0][1] = value;
    blue[1][0] = 0;
    blue[1][1] = 0;
    
    Image<Rgb<byte> > gbrg = create_test<bayer_gbrg>(size, red, green, blue);
    // img_save(gbrg, "gbrg" + postfix);

    // bayer bggr
    // BGBGBG
    // GRGRGR
    // BGBGBG
    // GRGRGR

    red[0][0] = 0;
    red[0][1] = 0;
    red[1][0] = 0;
    red[1][1] = value;

    green[0][0] = 0;
    green[0][1] = value;
    green[1][0] = value;
    green[1][1] = 0;

    blue[0][0] = value;
    blue[0][1] = 0;
    blue[1][0] = 0;
    blue[1][1] = 0;

    Image<Rgb<byte> > bggr = create_test<bayer_bggr>(size, red, green, blue);
    // img_save(bggr, "bggr" + postfix);
    
    // bayer grbg
    // GRGRGR
    // BGBGBG
    // GRGRGR
    // BGBGBG
    
    red[0][0] = 0;
    red[0][1] = value;
    red[1][0] = 0;
    red[1][1] = 0;

    green[0][0] = value;
    green[0][1] = 0;
    green[1][0] = 0;
    green[1][1] = value;

    blue[0][0] = 0;
    blue[0][1] = 0;
    blue[1][0] = value;
    blue[1][1] = 0;
    
    Image<Rgb<byte> > grbg = create_test<bayer_grbg>(size, red, green, blue);
    // img_save(grbg, "grbg" + postfix);
    
    Image<Rgb<byte> > all(ImageRef(3, 4*2).dot_times(size));
    all.sub_image(ImageRef(0,0).dot_times(size), rggb.size()).copy_from(rggb);
    all.sub_image(ImageRef(0,2).dot_times(size), gbrg.size()).copy_from(gbrg);
    all.sub_image(ImageRef(0,4).dot_times(size), bggr.size()).copy_from(bggr);
    all.sub_image(ImageRef(0,6).dot_times(size), grbg.size()).copy_from(grbg);

    img_save(all, "all" + postfix);
    return 0;
}

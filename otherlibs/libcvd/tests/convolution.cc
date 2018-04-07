

#include <cvd/image.h>
#include <cvd/convolution.h>

using namespace CVD;

int main(int argc, char **argv) {
  Image<float> img(ImageRef(2,5));
  Image<float> out(img.size());
  convolveGaussian(img, out, 1.0);
}

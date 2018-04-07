#include <cvd/brezenham.h>
#include <math.h>

using namespace TooN;

namespace CVD {

Brezenham::Brezenham(Vector<2> dir){
  if(fabs(dir[0]) > fabs(dir[1])){
    step1.y=0;
    if(dir[0] > 0){
      step1.x=1;
      val2=dir[0];
    } else {
      step1.x=-1;
      val2=-dir[0];
    }
    step2.x=0;
    if(dir[1] > 0){
      step2.y =1;
      val1=dir[1];
    } else {
      step2.y=-1;
      val1=-dir[1];
    }
  } else {
    step1.x=0;
    if(dir[1] > 0){
      step1.y=1;
      val2=dir[1];
    } else {
      step1.y=-1;
      val2=-dir[1];
    }
    step2.y=0;
    if(dir[0] > 0){
      step2.x =1;
      val1=dir[0];
    } else {
      step2.x=-1;
      val1=-dir[0];
    }
  }
  residual = val2/2;
}


ImageRef Brezenham::step(){
  if (residual >= val2){
    residual -= val2;
    return step2;
  }
  residual += val1;
  return step1;
}

Brezenham8::Brezenham8(Vector<2> dir){
  if(fabs(dir[0]) > fabs(dir[1])){
    step1.y=0;
    if(dir[0] > 0){
      step1.x=1;
      val2=dir[0];
    } else {
      step1.x=-1;
      val2=-dir[0];
    }
    step2.x=step1.x;
    if(dir[1] > 0){
      step2.y =1;
      val1=dir[1];
    } else {
      step2.y=-1;
      val1=-dir[1];
    }
    my_sideways.x=0;
    my_sideways.y=1;
  } else {
    step1.x=0;
    if(dir[1] > 0){
      step1.y=1;
      val2=dir[1];
    } else {
      step1.y=-1;
      val2=-dir[1];
    }
    step2.y=step1.y;
    if(dir[0] > 0){
      step2.x =1;
      val1=dir[0];
    } else {
      step2.x=-1;
      val1=-dir[0];
    }
    my_sideways.x=1;
    my_sideways.y=0;
  }
  residual = val2/2;
}


ImageRef Brezenham8::step(){
  residual += val1;
  if (residual >= val2){
    residual -= val2;
    return step2;
  }
  return step1;
}




} // namespace CVD


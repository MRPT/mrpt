// See KmUtils.h
//
// Author: David Arthur (darthur@gmail.com), 2009

#include "KmUtils.h"
#include <iostream>
using namespace std;

int __KMeansAssertionFailure(const char *file, int line, const char *expression) {
  cout << "ASSERTION FAILURE, " << file << " line " << line << ":" << endl;
  cout << "  " << expression << endl;
  exit(-1);
}

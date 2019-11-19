#include "cvd/diskbuffer2.h"
#include "cvd_src/config_internal.h"
#include <glob.h>
#include <string>
#include <vector>

using namespace std;
using namespace CVD;

//
// GLOBLIST
// Makes a list of files matching a pattern
//
namespace CVD{
vector<string> globlist(const string& gl)
{
	vector<string> ret;

	glob_t g;
	unsigned int i;
	
	#ifdef CVD_INTERNAL_GLOB_IS_BAD
		glob(gl.c_str(), 0 ,  0 , &g);
	#else
		glob(gl.c_str(), GLOB_BRACE | GLOB_TILDE,  0 , &g);
	#endif

	for(i=0; i < g.gl_pathc; i++)
		ret.push_back(g.gl_pathv[i]);

	globfree(&g);

	if(ret.size() == 1 && ret[0] == "")
		ret.resize(0);

	return ret;
}
}

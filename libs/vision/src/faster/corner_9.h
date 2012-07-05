/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

// ---------------------------------------------------------------------------
// LICENSING: This file is a slightly-modified version of part of libcvd, 
//             released under LGPL 2.1 by Edward Rosten
// ---------------------------------------------------------------------------

template <class C> inline bool is_corner_9(const uint8_t* p, const int w, const int barrier) {
    const int w3 = 3*w;
    const int t = C::prep_t(*p, barrier);
    if (!C::eval(p[-1-w3],t)) { // ???????????????-
        if (!C::eval(p[w3],t)) { // ????????-??????-
            return false;
        } // ????????@??????-
        if (!C::eval(p[1+w3],t)) { // ???????-@??????-
            return false;
        } // ???????@@??????-
        if (!C::eval(p[2+2*w],t)) { // ??????-@@??????-
            return false;
        } // ??????@@@??????-
        if (!C::eval(p[3+w],t)) { // ?????-@@@??????-
            if (!C::eval(p[-2-2*w],t)) { // ?????-@@@?????--
                return false;
            } // ?????-@@@?????@-
            if (!C::eval(p[-3-w],t)) { // ?????-@@@????-@-
                return false;
            } // ?????-@@@????@@-
            if (!C::eval(p[-1+w3],t)) { // ?????-@@@-???@@-
                return false;
            } // ?????-@@@@???@@-
            if (!C::eval(p[-2+2*w],t)) { // ?????-@@@@-??@@-
                return false;
            } // ?????-@@@@@??@@-
            if (!C::eval(p[-3+w],t)) { // ?????-@@@@@-?@@-
                return false;
            } // ?????-@@@@@@?@@-
            if (!C::eval(p[-3],t)) { // ?????-@@@@@@-@@-
                return false;
            } // ?????-@@@@@@@@@-
            return true;
        } // ?????@@@@??????-
        if (!C::eval(p[3],t)) { // ????-@@@@??????-
            if (!C::eval(p[-3-w],t)) { // ????-@@@@????-?-
                return false;
            } // ????-@@@@????@?-
            if (!C::eval(p[-1+w3],t)) { // ????-@@@@-???@?-
                return false;
            } // ????-@@@@@???@?-
            if (!C::eval(p[-2+2*w],t)) { // ????-@@@@@-??@?-
                return false;
            } // ????-@@@@@@??@?-
            if (!C::eval(p[-3+w],t)) { // ????-@@@@@@-?@?-
                return false;
            } // ????-@@@@@@@?@?-
            if (!C::eval(p[-3],t)) { // ????-@@@@@@@-@?-
                return false;
            } // ????-@@@@@@@@@?-
            return true;
        } // ????@@@@@??????-
        if (!C::eval(p[3-w],t)) { // ???-@@@@@??????-
            if (!C::eval(p[-1+w3],t)) { // ???-@@@@@-?????-
                return false;
            } // ???-@@@@@@?????-
            if (!C::eval(p[-2+2*w],t)) { // ???-@@@@@@-????-
                return false;
            } // ???-@@@@@@@????-
            if (!C::eval(p[-3+w],t)) { // ???-@@@@@@@-???-
                return false;
            } // ???-@@@@@@@@???-
            if (!C::eval(p[-3],t)) { // ???-@@@@@@@@-??-
                return false;
            } // ???-@@@@@@@@@??-
            return true;
        } // ???@@@@@@??????-
        if (!C::eval(p[-1+w3],t)) { // ???@@@@@@-?????-
            if (!C::eval(p[-w3],t)) { // -??@@@@@@-?????-
                return false;
            } // @??@@@@@@-?????-
            if (!C::eval(p[1-w3],t)) { // @-?@@@@@@-?????-
                return false;
            } // @@?@@@@@@-?????-
            if (!C::eval(p[2-2*w],t)) { // @@-@@@@@@-?????-
                return false;
            } // @@@@@@@@@-?????-
            return true;
        } // ???@@@@@@@?????-
        if (!C::eval(p[2-2*w],t)) { // ??-@@@@@@@?????-
            if (!C::eval(p[-2+2*w],t)) { // ??-@@@@@@@-????-
                return false;
            } // ??-@@@@@@@@????-
            if (!C::eval(p[-3+w],t)) { // ??-@@@@@@@@-???-
                return false;
            } // ??-@@@@@@@@@???-
            return true;
        } // ??@@@@@@@@?????-
        if (!C::eval(p[-2+2*w],t)) { // ??@@@@@@@@-????-
            if (!C::eval(p[1-w3],t)) { // ?-@@@@@@@@-????-
                return false;
            } // ?@@@@@@@@@-????-
            return true;
        } // ??@@@@@@@@@????-
        return true;
    } // ???????????????@
    if (!C::eval(p[-2-2*w],t)) { // ??????????????-@
        if (!C::eval(p[3+w],t)) { // ?????-????????-@
            return false;
        } // ?????@????????-@
        if (!C::eval(p[1+w3],t)) { // ?????@?-??????-@
            return false;
        } // ?????@?@??????-@
        if (!C::eval(p[2+2*w],t)) { // ?????@-@??????-@
            return false;
        } // ?????@@@??????-@
        if (!C::eval(p[3],t)) { // ????-@@@??????-@
            if (!C::eval(p[-3-w],t)) { // ????-@@@?????--@
                return false;
            } // ????-@@@?????@-@
            if (!C::eval(p[-3],t)) { // ????-@@@????-@-@
                return false;
            } // ????-@@@????@@-@
            if (!C::eval(p[w3],t)) { // ????-@@@-???@@-@
                return false;
            } // ????-@@@@???@@-@
            if (!C::eval(p[-1+w3],t)) { // ????-@@@@-??@@-@
                return false;
            } // ????-@@@@@??@@-@
            if (!C::eval(p[-2+2*w],t)) { // ????-@@@@@-?@@-@
                return false;
            } // ????-@@@@@@?@@-@
            if (!C::eval(p[-3+w],t)) { // ????-@@@@@@-@@-@
                return false;
            } // ????-@@@@@@@@@-@
            return true;
        } // ????@@@@??????-@
        if (!C::eval(p[3-w],t)) { // ???-@@@@??????-@
            if (!C::eval(p[-3],t)) { // ???-@@@@????-?-@
                return false;
            } // ???-@@@@????@?-@
            if (!C::eval(p[w3],t)) { // ???-@@@@-???@?-@
                return false;
            } // ???-@@@@@???@?-@
            if (!C::eval(p[-1+w3],t)) { // ???-@@@@@-??@?-@
                return false;
            } // ???-@@@@@@??@?-@
            if (!C::eval(p[-2+2*w],t)) { // ???-@@@@@@-?@?-@
                return false;
            } // ???-@@@@@@@?@?-@
            if (!C::eval(p[-3+w],t)) { // ???-@@@@@@@-@?-@
                return false;
            } // ???-@@@@@@@@@?-@
            return true;
        } // ???@@@@@??????-@
        if (!C::eval(p[2-2*w],t)) { // ??-@@@@@??????-@
            if (!C::eval(p[w3],t)) { // ??-@@@@@-?????-@
                return false;
            } // ??-@@@@@@?????-@
            if (!C::eval(p[-1+w3],t)) { // ??-@@@@@@-????-@
                return false;
            } // ??-@@@@@@@????-@
            if (!C::eval(p[-2+2*w],t)) { // ??-@@@@@@@-???-@
                return false;
            } // ??-@@@@@@@@???-@
            if (!C::eval(p[-3+w],t)) { // ??-@@@@@@@@-??-@
                return false;
            } // ??-@@@@@@@@@??-@
            return true;
        } // ??@@@@@@??????-@
        if (!C::eval(p[1-w3],t)) { // ?-@@@@@@??????-@
            if (!C::eval(p[w3],t)) { // ?-@@@@@@-?????-@
                return false;
            } // ?-@@@@@@@?????-@
            if (!C::eval(p[-1+w3],t)) { // ?-@@@@@@@-????-@
                return false;
            } // ?-@@@@@@@@????-@
            if (!C::eval(p[-2+2*w],t)) { // ?-@@@@@@@@-???-@
                return false;
            } // ?-@@@@@@@@@???-@
            return true;
        } // ?@@@@@@@??????-@
        if (!C::eval(p[-w3],t)) { // -@@@@@@@??????-@
            if (!C::eval(p[w3],t)) { // -@@@@@@@-?????-@
                return false;
            } // -@@@@@@@@?????-@
            if (!C::eval(p[-1+w3],t)) { // -@@@@@@@@-????-@
                return false;
            } // -@@@@@@@@@????-@
            return true;
        } // @@@@@@@@??????-@
        return true;
    } // ??????????????@@
    if (!C::eval(p[-w3],t)) { // -?????????????@@
        if (!C::eval(p[1+w3],t)) { // -??????-??????@@
            return false;
        } // -??????@??????@@
        if (!C::eval(p[-1+w3],t)) { // -??????@?-????@@
            return false;
        } // -??????@?@????@@
        if (!C::eval(p[w3],t)) { // -??????@-@????@@
            return false;
        } // -??????@@@????@@
        if (!C::eval(p[-2+2*w],t)) { // -??????@@@-???@@
            if (!C::eval(p[2+2*w],t)) { // -?????-@@@-???@@
                return false;
            } // -?????@@@@-???@@
            if (!C::eval(p[3+w],t)) { // -????-@@@@-???@@
                return false;
            } // -????@@@@@-???@@
            if (!C::eval(p[1-w3],t)) { // --???@@@@@-???@@
                return false;
            } // -@???@@@@@-???@@
            if (!C::eval(p[2-2*w],t)) { // -@-??@@@@@-???@@
                return false;
            } // -@@??@@@@@-???@@
            if (!C::eval(p[3-w],t)) { // -@@-?@@@@@-???@@
                return false;
            } // -@@@?@@@@@-???@@
            if (!C::eval(p[3],t)) { // -@@@-@@@@@-???@@
                return false;
            } // -@@@@@@@@@-???@@
            return true;
        } // -??????@@@@???@@
        if (!C::eval(p[-3+w],t)) { // -??????@@@@-??@@
            if (!C::eval(p[2+2*w],t)) { // -?????-@@@@-??@@
                return false;
            } // -?????@@@@@-??@@
            if (!C::eval(p[2-2*w],t)) { // -?-???@@@@@-??@@
                return false;
            } // -?@???@@@@@-??@@
            if (!C::eval(p[3-w],t)) { // -?@-??@@@@@-??@@
                return false;
            } // -?@@??@@@@@-??@@
            if (!C::eval(p[3],t)) { // -?@@-?@@@@@-??@@
                return false;
            } // -?@@@?@@@@@-??@@
            if (!C::eval(p[3+w],t)) { // -?@@@-@@@@@-??@@
                return false;
            } // -?@@@@@@@@@-??@@
            return true;
        } // -??????@@@@@??@@
        if (!C::eval(p[-3],t)) { // -??????@@@@@-?@@
            if (!C::eval(p[3-w],t)) { // -??-???@@@@@-?@@
                return false;
            } // -??@???@@@@@-?@@
            if (!C::eval(p[3],t)) { // -??@-??@@@@@-?@@
                return false;
            } // -??@@??@@@@@-?@@
            if (!C::eval(p[3+w],t)) { // -??@@-?@@@@@-?@@
                return false;
            } // -??@@@?@@@@@-?@@
            if (!C::eval(p[2+2*w],t)) { // -??@@@-@@@@@-?@@
                return false;
            } // -??@@@@@@@@@-?@@
            return true;
        } // -??????@@@@@@?@@
        if (!C::eval(p[-3-w],t)) { // -??????@@@@@@-@@
            if (!C::eval(p[3],t)) { // -???-??@@@@@@-@@
                return false;
            } // -???@??@@@@@@-@@
            if (!C::eval(p[3+w],t)) { // -???@-?@@@@@@-@@
                return false;
            } // -???@@?@@@@@@-@@
            if (!C::eval(p[2+2*w],t)) { // -???@@-@@@@@@-@@
                return false;
            } // -???@@@@@@@@@-@@
            return true;
        } // -??????@@@@@@@@@
        return true;
    } // @?????????????@@
    if (!C::eval(p[-3-w],t)) { // @????????????-@@
        if (!C::eval(p[2+2*w],t)) { // @?????-??????-@@
            return false;
        } // @?????@??????-@@
        if (!C::eval(p[3+w],t)) { // @????-@??????-@@
            return false;
        } // @????@@??????-@@
        if (!C::eval(p[3],t)) { // @???-@@??????-@@
            return false;
        } // @???@@@??????-@@
        if (!C::eval(p[3-w],t)) { // @??-@@@??????-@@
            if (!C::eval(p[-3],t)) { // @??-@@@?????--@@
                return false;
            } // @??-@@@?????@-@@
            if (!C::eval(p[-3+w],t)) { // @??-@@@????-@-@@
                return false;
            } // @??-@@@????@@-@@
            if (!C::eval(p[1+w3],t)) { // @??-@@@-???@@-@@
                return false;
            } // @??-@@@@???@@-@@
            if (!C::eval(p[w3],t)) { // @??-@@@@-??@@-@@
                return false;
            } // @??-@@@@@??@@-@@
            if (!C::eval(p[-1+w3],t)) { // @??-@@@@@-?@@-@@
                return false;
            } // @??-@@@@@@?@@-@@
            if (!C::eval(p[-2+2*w],t)) { // @??-@@@@@@-@@-@@
                return false;
            } // @??-@@@@@@@@@-@@
            return true;
        } // @??@@@@??????-@@
        if (!C::eval(p[2-2*w],t)) { // @?-@@@@??????-@@
            if (!C::eval(p[-3+w],t)) { // @?-@@@@????-?-@@
                return false;
            } // @?-@@@@????@?-@@
            if (!C::eval(p[1+w3],t)) { // @?-@@@@-???@?-@@
                return false;
            } // @?-@@@@@???@?-@@
            if (!C::eval(p[w3],t)) { // @?-@@@@@-??@?-@@
                return false;
            } // @?-@@@@@@??@?-@@
            if (!C::eval(p[-1+w3],t)) { // @?-@@@@@@-?@?-@@
                return false;
            } // @?-@@@@@@@?@?-@@
            if (!C::eval(p[-2+2*w],t)) { // @?-@@@@@@@-@?-@@
                return false;
            } // @?-@@@@@@@@@?-@@
            return true;
        } // @?@@@@@??????-@@
        if (!C::eval(p[1-w3],t)) { // @-@@@@@??????-@@
            if (!C::eval(p[1+w3],t)) { // @-@@@@@-?????-@@
                return false;
            } // @-@@@@@@?????-@@
            if (!C::eval(p[w3],t)) { // @-@@@@@@-????-@@
                return false;
            } // @-@@@@@@@????-@@
            if (!C::eval(p[-1+w3],t)) { // @-@@@@@@@-???-@@
                return false;
            } // @-@@@@@@@@???-@@
            if (!C::eval(p[-2+2*w],t)) { // @-@@@@@@@@-??-@@
                return false;
            } // @-@@@@@@@@@??-@@
            return true;
        } // @@@@@@@??????-@@
        return true;
    } // @????????????@@@
    if (!C::eval(p[1-w3],t)) { // @-???????????@@@
        if (!C::eval(p[-2+2*w],t)) { // @-????????-??@@@
            return false;
        } // @-????????@??@@@
        if (!C::eval(p[-1+w3],t)) { // @-???????-@??@@@
            return false;
        } // @-???????@@??@@@
        if (!C::eval(p[w3],t)) { // @-??????-@@??@@@
            return false;
        } // @-??????@@@??@@@
        if (!C::eval(p[-3+w],t)) { // @-??????@@@-?@@@
            if (!C::eval(p[1+w3],t)) { // @-?????-@@@-?@@@
                return false;
            } // @-?????@@@@-?@@@
            if (!C::eval(p[2+2*w],t)) { // @-????-@@@@-?@@@
                return false;
            } // @-????@@@@@-?@@@
            if (!C::eval(p[2-2*w],t)) { // @--???@@@@@-?@@@
                return false;
            } // @-@???@@@@@-?@@@
            if (!C::eval(p[3-w],t)) { // @-@-??@@@@@-?@@@
                return false;
            } // @-@@??@@@@@-?@@@
            if (!C::eval(p[3],t)) { // @-@@-?@@@@@-?@@@
                return false;
            } // @-@@@?@@@@@-?@@@
            if (!C::eval(p[3+w],t)) { // @-@@@-@@@@@-?@@@
                return false;
            } // @-@@@@@@@@@-?@@@
            return true;
        } // @-??????@@@@?@@@
        if (!C::eval(p[-3],t)) { // @-??????@@@@-@@@
            if (!C::eval(p[1+w3],t)) { // @-?????-@@@@-@@@
                return false;
            } // @-?????@@@@@-@@@
            if (!C::eval(p[3-w],t)) { // @-?-???@@@@@-@@@
                return false;
            } // @-?@???@@@@@-@@@
            if (!C::eval(p[3],t)) { // @-?@-??@@@@@-@@@
                return false;
            } // @-?@@??@@@@@-@@@
            if (!C::eval(p[3+w],t)) { // @-?@@-?@@@@@-@@@
                return false;
            } // @-?@@@?@@@@@-@@@
            if (!C::eval(p[2+2*w],t)) { // @-?@@@-@@@@@-@@@
                return false;
            } // @-?@@@@@@@@@-@@@
            return true;
        } // @-??????@@@@@@@@
        return true;
    } // @@???????????@@@
    if (!C::eval(p[-3],t)) { // @@??????????-@@@
        if (!C::eval(p[3+w],t)) { // @@???-??????-@@@
            return false;
        } // @@???@??????-@@@
        if (!C::eval(p[3],t)) { // @@??-@??????-@@@
            return false;
        } // @@??@@??????-@@@
        if (!C::eval(p[3-w],t)) { // @@?-@@??????-@@@
            return false;
        } // @@?@@@??????-@@@
        if (!C::eval(p[2-2*w],t)) { // @@-@@@??????-@@@
            if (!C::eval(p[-3+w],t)) { // @@-@@@?????--@@@
                return false;
            } // @@-@@@?????@-@@@
            if (!C::eval(p[-2+2*w],t)) { // @@-@@@????-@-@@@
                return false;
            } // @@-@@@????@@-@@@
            if (!C::eval(p[2+2*w],t)) { // @@-@@@-???@@-@@@
                return false;
            } // @@-@@@@???@@-@@@
            if (!C::eval(p[1+w3],t)) { // @@-@@@@-??@@-@@@
                return false;
            } // @@-@@@@@??@@-@@@
            if (!C::eval(p[w3],t)) { // @@-@@@@@-?@@-@@@
                return false;
            } // @@-@@@@@@?@@-@@@
            if (!C::eval(p[-1+w3],t)) { // @@-@@@@@@-@@-@@@
                return false;
            } // @@-@@@@@@@@@-@@@
            return true;
        } // @@@@@@??????-@@@
        return true;
    } // @@??????????@@@@
    if (!C::eval(p[2-2*w],t)) { // @@-?????????@@@@
        if (!C::eval(p[-1+w3],t)) { // @@-??????-??@@@@
            return false;
        } // @@-??????@??@@@@
        if (!C::eval(p[-2+2*w],t)) { // @@-??????@-?@@@@
            return false;
        } // @@-??????@@?@@@@
        if (!C::eval(p[-3+w],t)) { // @@-??????@@-@@@@
            return false;
        } // @@-??????@@@@@@@
        return true;
    } // @@@?????????@@@@
    if (!C::eval(p[-3+w],t)) { // @@@????????-@@@@
        if (!C::eval(p[3-w],t)) { // @@@-???????-@@@@
            return false;
        } // @@@@???????-@@@@
        if (!C::eval(p[3],t)) { // @@@@-??????-@@@@
            return false;
        } // @@@@@??????-@@@@
        return true;
    } // @@@????????@@@@@
    if (!C::eval(p[3-w],t)) { // @@@-???????@@@@@
        if (!C::eval(p[-2+2*w],t)) { // @@@-??????-@@@@@
            return false;
        } // @@@-??????@@@@@@
        return true;
    } // @@@@???????@@@@@
    return true;
}


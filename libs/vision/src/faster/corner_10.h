/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

// ---------------------------------------------------------------------------
// LICENSING: This file is a slightly-modified version of part of libcvd, 
//             released under LGPL 2.1 by Edward Rosten
// ---------------------------------------------------------------------------

template <class C> inline bool is_corner_10(const uint8_t* p, const int w, const int barrier) {
    const int w3 = 3*w;
    const int t = C::prep_t(*p, barrier);
    if (!C::eval(p[-1-w3],t)) { // ???????????????-
        if (!C::eval(p[3+w],t)) { // ?????-?????????-
            return false;
        } // ?????@?????????-
        if (!C::eval(p[2+2*w],t)) { // ?????@-????????-
            return false;
        } // ?????@@????????-
        if (!C::eval(p[-1+w3],t)) { // ?????@@??-?????-
            return false;
        } // ?????@@??@?????-
        if (!C::eval(p[1+w3],t)) { // ?????@@-?@?????-
            return false;
        } // ?????@@@?@?????-
        if (!C::eval(p[w3],t)) { // ?????@@@-@?????-
            return false;
        } // ?????@@@@@?????-
        if (!C::eval(p[-2+2*w],t)) { // ?????@@@@@-????-
            if (!C::eval(p[-w3],t)) { // -????@@@@@-????-
                return false;
            } // @????@@@@@-????-
            if (!C::eval(p[3],t)) { // @???-@@@@@-????-
                return false;
            } // @???@@@@@@-????-
            if (!C::eval(p[1-w3],t)) { // @-??@@@@@@-????-
                return false;
            } // @@??@@@@@@-????-
            if (!C::eval(p[2-2*w],t)) { // @@-?@@@@@@-????-
                return false;
            } // @@@?@@@@@@-????-
            if (!C::eval(p[3-w],t)) { // @@@-@@@@@@-????-
                return false;
            } // @@@@@@@@@@-????-
            return true;
        } // ?????@@@@@@????-
        if (!C::eval(p[-3+w],t)) { // ?????@@@@@@-???-
            if (!C::eval(p[3],t)) { // ????-@@@@@@-???-
                return false;
            } // ????@@@@@@@-???-
            if (!C::eval(p[1-w3],t)) { // ?-??@@@@@@@-???-
                return false;
            } // ?@??@@@@@@@-???-
            if (!C::eval(p[2-2*w],t)) { // ?@-?@@@@@@@-???-
                return false;
            } // ?@@?@@@@@@@-???-
            if (!C::eval(p[3-w],t)) { // ?@@-@@@@@@@-???-
                return false;
            } // ?@@@@@@@@@@-???-
            return true;
        } // ?????@@@@@@@???-
        if (!C::eval(p[3],t)) { // ????-@@@@@@@???-
            if (!C::eval(p[-3],t)) { // ????-@@@@@@@-??-
                return false;
            } // ????-@@@@@@@@??-
            if (!C::eval(p[-3-w],t)) { // ????-@@@@@@@@-?-
                return false;
            } // ????-@@@@@@@@@?-
            if (!C::eval(p[-2-2*w],t)) { // ????-@@@@@@@@@--
                return false;
            } // ????-@@@@@@@@@@-
            return true;
        } // ????@@@@@@@@???-
        if (!C::eval(p[3-w],t)) { // ???-@@@@@@@@???-
            if (!C::eval(p[-3],t)) { // ???-@@@@@@@@-??-
                return false;
            } // ???-@@@@@@@@@??-
            if (!C::eval(p[-3-w],t)) { // ???-@@@@@@@@@-?-
                return false;
            } // ???-@@@@@@@@@@?-
            return true;
        } // ???@@@@@@@@@???-
        if (!C::eval(p[-3],t)) { // ???@@@@@@@@@-??-
            if (!C::eval(p[2-2*w],t)) { // ??-@@@@@@@@@-??-
                return false;
            } // ??@@@@@@@@@@-??-
            return true;
        } // ???@@@@@@@@@@??-
        return true;
    } // ???????????????@
    if (!C::eval(p[-2-2*w],t)) { // ??????????????-@
        if (!C::eval(p[3],t)) { // ????-?????????-@
            return false;
        } // ????@?????????-@
        if (!C::eval(p[3+w],t)) { // ????@-????????-@
            return false;
        } // ????@@????????-@
        if (!C::eval(p[w3],t)) { // ????@@??-?????-@
            return false;
        } // ????@@??@?????-@
        if (!C::eval(p[1+w3],t)) { // ????@@?-@?????-@
            return false;
        } // ????@@?@@?????-@
        if (!C::eval(p[2+2*w],t)) { // ????@@-@@?????-@
            return false;
        } // ????@@@@@?????-@
        if (!C::eval(p[3-w],t)) { // ???-@@@@@?????-@
            if (!C::eval(p[-1+w3],t)) { // ???-@@@@@-????-@
                return false;
            } // ???-@@@@@@????-@
            if (!C::eval(p[-3-w],t)) { // ???-@@@@@@???--@
                return false;
            } // ???-@@@@@@???@-@
            if (!C::eval(p[-2+2*w],t)) { // ???-@@@@@@-??@-@
                return false;
            } // ???-@@@@@@@??@-@
            if (!C::eval(p[-3+w],t)) { // ???-@@@@@@@-?@-@
                return false;
            } // ???-@@@@@@@@?@-@
            if (!C::eval(p[-3],t)) { // ???-@@@@@@@@-@-@
                return false;
            } // ???-@@@@@@@@@@-@
            return true;
        } // ???@@@@@@?????-@
        if (!C::eval(p[2-2*w],t)) { // ??-@@@@@@?????-@
            if (!C::eval(p[-3],t)) { // ??-@@@@@@???-?-@
                return false;
            } // ??-@@@@@@???@?-@
            if (!C::eval(p[-1+w3],t)) { // ??-@@@@@@-??@?-@
                return false;
            } // ??-@@@@@@@??@?-@
            if (!C::eval(p[-2+2*w],t)) { // ??-@@@@@@@-?@?-@
                return false;
            } // ??-@@@@@@@@?@?-@
            if (!C::eval(p[-3+w],t)) { // ??-@@@@@@@@-@?-@
                return false;
            } // ??-@@@@@@@@@@?-@
            return true;
        } // ??@@@@@@@?????-@
        if (!C::eval(p[1-w3],t)) { // ?-@@@@@@@?????-@
            if (!C::eval(p[-1+w3],t)) { // ?-@@@@@@@-????-@
                return false;
            } // ?-@@@@@@@@????-@
            if (!C::eval(p[-2+2*w],t)) { // ?-@@@@@@@@-???-@
                return false;
            } // ?-@@@@@@@@@???-@
            if (!C::eval(p[-3+w],t)) { // ?-@@@@@@@@@-??-@
                return false;
            } // ?-@@@@@@@@@@??-@
            return true;
        } // ?@@@@@@@@?????-@
        if (!C::eval(p[-w3],t)) { // -@@@@@@@@?????-@
            if (!C::eval(p[-1+w3],t)) { // -@@@@@@@@-????-@
                return false;
            } // -@@@@@@@@@????-@
            if (!C::eval(p[-2+2*w],t)) { // -@@@@@@@@@-???-@
                return false;
            } // -@@@@@@@@@@???-@
            return true;
        } // @@@@@@@@@?????-@
        return true;
    } // ??????????????@@
    if (!C::eval(p[-3-w],t)) { // ?????????????-@@
        if (!C::eval(p[1+w3],t)) { // ???????-?????-@@
            return false;
        } // ???????@?????-@@
        if (!C::eval(p[3-w],t)) { // ???-???@?????-@@
            return false;
        } // ???@???@?????-@@
        if (!C::eval(p[3],t)) { // ???@-??@?????-@@
            return false;
        } // ???@@??@?????-@@
        if (!C::eval(p[3+w],t)) { // ???@@-?@?????-@@
            return false;
        } // ???@@@?@?????-@@
        if (!C::eval(p[2+2*w],t)) { // ???@@@-@?????-@@
            return false;
        } // ???@@@@@?????-@@
        if (!C::eval(p[2-2*w],t)) { // ??-@@@@@?????-@@
            if (!C::eval(p[w3],t)) { // ??-@@@@@-????-@@
                return false;
            } // ??-@@@@@@????-@@
            if (!C::eval(p[-3],t)) { // ??-@@@@@@???--@@
                return false;
            } // ??-@@@@@@???@-@@
            if (!C::eval(p[-1+w3],t)) { // ??-@@@@@@-??@-@@
                return false;
            } // ??-@@@@@@@??@-@@
            if (!C::eval(p[-2+2*w],t)) { // ??-@@@@@@@-?@-@@
                return false;
            } // ??-@@@@@@@@?@-@@
            if (!C::eval(p[-3+w],t)) { // ??-@@@@@@@@-@-@@
                return false;
            } // ??-@@@@@@@@@@-@@
            return true;
        } // ??@@@@@@?????-@@
        if (!C::eval(p[1-w3],t)) { // ?-@@@@@@?????-@@
            if (!C::eval(p[-3+w],t)) { // ?-@@@@@@???-?-@@
                return false;
            } // ?-@@@@@@???@?-@@
            if (!C::eval(p[w3],t)) { // ?-@@@@@@-??@?-@@
                return false;
            } // ?-@@@@@@@??@?-@@
            if (!C::eval(p[-1+w3],t)) { // ?-@@@@@@@-?@?-@@
                return false;
            } // ?-@@@@@@@@?@?-@@
            if (!C::eval(p[-2+2*w],t)) { // ?-@@@@@@@@-@?-@@
                return false;
            } // ?-@@@@@@@@@@?-@@
            return true;
        } // ?@@@@@@@?????-@@
        if (!C::eval(p[-w3],t)) { // -@@@@@@@?????-@@
            if (!C::eval(p[w3],t)) { // -@@@@@@@-????-@@
                return false;
            } // -@@@@@@@@????-@@
            if (!C::eval(p[-1+w3],t)) { // -@@@@@@@@-???-@@
                return false;
            } // -@@@@@@@@@???-@@
            if (!C::eval(p[-2+2*w],t)) { // -@@@@@@@@@-??-@@
                return false;
            } // -@@@@@@@@@@??-@@
            return true;
        } // @@@@@@@@?????-@@
        return true;
    } // ?????????????@@@
    if (!C::eval(p[-w3],t)) { // -????????????@@@
        if (!C::eval(p[2+2*w],t)) { // -?????-??????@@@
            return false;
        } // -?????@??????@@@
        if (!C::eval(p[1+w3],t)) { // -?????@-?????@@@
            return false;
        } // -?????@@?????@@@
        if (!C::eval(p[-2+2*w],t)) { // -?????@@??-??@@@
            return false;
        } // -?????@@??@??@@@
        if (!C::eval(p[w3],t)) { // -?????@@-?@??@@@
            return false;
        } // -?????@@@?@??@@@
        if (!C::eval(p[-1+w3],t)) { // -?????@@@-@??@@@
            return false;
        } // -?????@@@@@??@@@
        if (!C::eval(p[-3+w],t)) { // -?????@@@@@-?@@@
            if (!C::eval(p[1-w3],t)) { // --????@@@@@-?@@@
                return false;
            } // -@????@@@@@-?@@@
            if (!C::eval(p[3+w],t)) { // -@???-@@@@@-?@@@
                return false;
            } // -@???@@@@@@-?@@@
            if (!C::eval(p[2-2*w],t)) { // -@-??@@@@@@-?@@@
                return false;
            } // -@@??@@@@@@-?@@@
            if (!C::eval(p[3-w],t)) { // -@@-?@@@@@@-?@@@
                return false;
            } // -@@@?@@@@@@-?@@@
            if (!C::eval(p[3],t)) { // -@@@-@@@@@@-?@@@
                return false;
            } // -@@@@@@@@@@-?@@@
            return true;
        } // -?????@@@@@@?@@@
        if (!C::eval(p[-3],t)) { // -?????@@@@@@-@@@
            if (!C::eval(p[3+w],t)) { // -????-@@@@@@-@@@
                return false;
            } // -????@@@@@@@-@@@
            if (!C::eval(p[2-2*w],t)) { // -?-??@@@@@@@-@@@
                return false;
            } // -?@??@@@@@@@-@@@
            if (!C::eval(p[3-w],t)) { // -?@-?@@@@@@@-@@@
                return false;
            } // -?@@?@@@@@@@-@@@
            if (!C::eval(p[3],t)) { // -?@@-@@@@@@@-@@@
                return false;
            } // -?@@@@@@@@@@-@@@
            return true;
        } // -?????@@@@@@@@@@
        return true;
    } // @????????????@@@
    if (!C::eval(p[-3],t)) { // @???????????-@@@
        if (!C::eval(p[2+2*w],t)) { // @?????-?????-@@@
            return false;
        } // @?????@?????-@@@
        if (!C::eval(p[2-2*w],t)) { // @?-???@?????-@@@
            return false;
        } // @?@???@?????-@@@
        if (!C::eval(p[3-w],t)) { // @?@-??@?????-@@@
            return false;
        } // @?@@??@?????-@@@
        if (!C::eval(p[3+w],t)) { // @?@@?-@?????-@@@
            return false;
        } // @?@@?@@?????-@@@
        if (!C::eval(p[3],t)) { // @?@@-@@?????-@@@
            return false;
        } // @?@@@@@?????-@@@
        if (!C::eval(p[1-w3],t)) { // @-@@@@@?????-@@@
            if (!C::eval(p[1+w3],t)) { // @-@@@@@-????-@@@
                return false;
            } // @-@@@@@@????-@@@
            if (!C::eval(p[-3+w],t)) { // @-@@@@@@???--@@@
                return false;
            } // @-@@@@@@???@-@@@
            if (!C::eval(p[w3],t)) { // @-@@@@@@-??@-@@@
                return false;
            } // @-@@@@@@@??@-@@@
            if (!C::eval(p[-1+w3],t)) { // @-@@@@@@@-?@-@@@
                return false;
            } // @-@@@@@@@@?@-@@@
            if (!C::eval(p[-2+2*w],t)) { // @-@@@@@@@@-@-@@@
                return false;
            } // @-@@@@@@@@@@-@@@
            return true;
        } // @@@@@@@?????-@@@
        return true;
    } // @???????????@@@@
    if (!C::eval(p[1-w3],t)) { // @-??????????@@@@
        if (!C::eval(p[1+w3],t)) { // @-?????-????@@@@
            return false;
        } // @-?????@????@@@@
        if (!C::eval(p[-3+w],t)) { // @-?????@???-@@@@
            return false;
        } // @-?????@???@@@@@
        if (!C::eval(p[w3],t)) { // @-?????@-??@@@@@
            return false;
        } // @-?????@@??@@@@@
        if (!C::eval(p[-1+w3],t)) { // @-?????@@-?@@@@@
            return false;
        } // @-?????@@@?@@@@@
        if (!C::eval(p[-2+2*w],t)) { // @-?????@@@-@@@@@
            return false;
        } // @-?????@@@@@@@@@
        return true;
    } // @@??????????@@@@
    if (!C::eval(p[2-2*w],t)) { // @@-?????????@@@@
        if (!C::eval(p[-3+w],t)) { // @@-????????-@@@@
            return false;
        } // @@-????????@@@@@
        if (!C::eval(p[w3],t)) { // @@-?????-??@@@@@
            return false;
        } // @@-?????@??@@@@@
        if (!C::eval(p[-1+w3],t)) { // @@-?????@-?@@@@@
            return false;
        } // @@-?????@@?@@@@@
        if (!C::eval(p[-2+2*w],t)) { // @@-?????@@-@@@@@
            return false;
        } // @@-?????@@@@@@@@
        return true;
    } // @@@?????????@@@@
    if (!C::eval(p[-3+w],t)) { // @@@????????-@@@@
        if (!C::eval(p[3-w],t)) { // @@@-???????-@@@@
            return false;
        } // @@@@???????-@@@@
        if (!C::eval(p[3],t)) { // @@@@-??????-@@@@
            return false;
        } // @@@@@??????-@@@@
        if (!C::eval(p[3+w],t)) { // @@@@@-?????-@@@@
            return false;
        } // @@@@@@?????-@@@@
        return true;
    } // @@@????????@@@@@
    if (!C::eval(p[-2+2*w],t)) { // @@@???????-@@@@@
        if (!C::eval(p[3-w],t)) { // @@@-??????-@@@@@
            return false;
        } // @@@@??????-@@@@@
        if (!C::eval(p[3],t)) { // @@@@-?????-@@@@@
            return false;
        } // @@@@@?????-@@@@@
        return true;
    } // @@@???????@@@@@@
    if (!C::eval(p[3-w],t)) { // @@@-??????@@@@@@
        if (!C::eval(p[-1+w3],t)) { // @@@-?????-@@@@@@
            return false;
        } // @@@-?????@@@@@@@
        return true;
    } // @@@@??????@@@@@@
    return true;
}


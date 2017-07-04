/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

// ---------------------------------------------------------------------------
// LICENSING: This file is a slightly-modified version of part of libcvd,
//             released under LGPL 2.1 by Edward Rosten
// ---------------------------------------------------------------------------

template <class C>
inline bool is_corner_10(const uint8_t* p, const int w, const int barrier)
{
	const int w3 = 3 * w;
	const int t = C::prep_t(*p, barrier);
	if (!C::eval(p[-1 - w3], t))
	{  // ???????????????-
		if (!C::eval(p[3 + w], t))
		{  // ?????-?????????-
			return false;
		}  // ?????@?????????-
		if (!C::eval(p[2 + 2 * w], t))
		{  // ?????@-????????-
			return false;
		}  // ?????@@????????-
		if (!C::eval(p[-1 + w3], t))
		{  // ?????@@??-?????-
			return false;
		}  // ?????@@??@?????-
		if (!C::eval(p[1 + w3], t))
		{  // ?????@@-?@?????-
			return false;
		}  // ?????@@@?@?????-
		if (!C::eval(p[w3], t))
		{  // ?????@@@-@?????-
			return false;
		}  // ?????@@@@@?????-
		if (!C::eval(p[-2 + 2 * w], t))
		{  // ?????@@@@@-????-
			if (!C::eval(p[-w3], t))
			{  // -????@@@@@-????-
				return false;
			}  // @????@@@@@-????-
			if (!C::eval(p[3], t))
			{  // @???-@@@@@-????-
				return false;
			}  // @???@@@@@@-????-
			if (!C::eval(p[1 - w3], t))
			{  // @-??@@@@@@-????-
				return false;
			}  // @@??@@@@@@-????-
			if (!C::eval(p[2 - 2 * w], t))
			{  // @@-?@@@@@@-????-
				return false;
			}  // @@@?@@@@@@-????-
			if (!C::eval(p[3 - w], t))
			{  // @@@-@@@@@@-????-
				return false;
			}  // @@@@@@@@@@-????-
			return true;
		}  // ?????@@@@@@????-
		if (!C::eval(p[-3 + w], t))
		{  // ?????@@@@@@-???-
			if (!C::eval(p[3], t))
			{  // ????-@@@@@@-???-
				return false;
			}  // ????@@@@@@@-???-
			if (!C::eval(p[1 - w3], t))
			{  // ?-??@@@@@@@-???-
				return false;
			}  // ?@??@@@@@@@-???-
			if (!C::eval(p[2 - 2 * w], t))
			{  // ?@-?@@@@@@@-???-
				return false;
			}  // ?@@?@@@@@@@-???-
			if (!C::eval(p[3 - w], t))
			{  // ?@@-@@@@@@@-???-
				return false;
			}  // ?@@@@@@@@@@-???-
			return true;
		}  // ?????@@@@@@@???-
		if (!C::eval(p[3], t))
		{  // ????-@@@@@@@???-
			if (!C::eval(p[-3], t))
			{  // ????-@@@@@@@-??-
				return false;
			}  // ????-@@@@@@@@??-
			if (!C::eval(p[-3 - w], t))
			{  // ????-@@@@@@@@-?-
				return false;
			}  // ????-@@@@@@@@@?-
			if (!C::eval(p[-2 - 2 * w], t))
			{  // ????-@@@@@@@@@--
				return false;
			}  // ????-@@@@@@@@@@-
			return true;
		}  // ????@@@@@@@@???-
		if (!C::eval(p[3 - w], t))
		{  // ???-@@@@@@@@???-
			if (!C::eval(p[-3], t))
			{  // ???-@@@@@@@@-??-
				return false;
			}  // ???-@@@@@@@@@??-
			if (!C::eval(p[-3 - w], t))
			{  // ???-@@@@@@@@@-?-
				return false;
			}  // ???-@@@@@@@@@@?-
			return true;
		}  // ???@@@@@@@@@???-
		if (!C::eval(p[-3], t))
		{  // ???@@@@@@@@@-??-
			if (!C::eval(p[2 - 2 * w], t))
			{  // ??-@@@@@@@@@-??-
				return false;
			}  // ??@@@@@@@@@@-??-
			return true;
		}  // ???@@@@@@@@@@??-
		return true;
	}  // ???????????????@
	if (!C::eval(p[-2 - 2 * w], t))
	{  // ??????????????-@
		if (!C::eval(p[3], t))
		{  // ????-?????????-@
			return false;
		}  // ????@?????????-@
		if (!C::eval(p[3 + w], t))
		{  // ????@-????????-@
			return false;
		}  // ????@@????????-@
		if (!C::eval(p[w3], t))
		{  // ????@@??-?????-@
			return false;
		}  // ????@@??@?????-@
		if (!C::eval(p[1 + w3], t))
		{  // ????@@?-@?????-@
			return false;
		}  // ????@@?@@?????-@
		if (!C::eval(p[2 + 2 * w], t))
		{  // ????@@-@@?????-@
			return false;
		}  // ????@@@@@?????-@
		if (!C::eval(p[3 - w], t))
		{  // ???-@@@@@?????-@
			if (!C::eval(p[-1 + w3], t))
			{  // ???-@@@@@-????-@
				return false;
			}  // ???-@@@@@@????-@
			if (!C::eval(p[-3 - w], t))
			{  // ???-@@@@@@???--@
				return false;
			}  // ???-@@@@@@???@-@
			if (!C::eval(p[-2 + 2 * w], t))
			{  // ???-@@@@@@-??@-@
				return false;
			}  // ???-@@@@@@@??@-@
			if (!C::eval(p[-3 + w], t))
			{  // ???-@@@@@@@-?@-@
				return false;
			}  // ???-@@@@@@@@?@-@
			if (!C::eval(p[-3], t))
			{  // ???-@@@@@@@@-@-@
				return false;
			}  // ???-@@@@@@@@@@-@
			return true;
		}  // ???@@@@@@?????-@
		if (!C::eval(p[2 - 2 * w], t))
		{  // ??-@@@@@@?????-@
			if (!C::eval(p[-3], t))
			{  // ??-@@@@@@???-?-@
				return false;
			}  // ??-@@@@@@???@?-@
			if (!C::eval(p[-1 + w3], t))
			{  // ??-@@@@@@-??@?-@
				return false;
			}  // ??-@@@@@@@??@?-@
			if (!C::eval(p[-2 + 2 * w], t))
			{  // ??-@@@@@@@-?@?-@
				return false;
			}  // ??-@@@@@@@@?@?-@
			if (!C::eval(p[-3 + w], t))
			{  // ??-@@@@@@@@-@?-@
				return false;
			}  // ??-@@@@@@@@@@?-@
			return true;
		}  // ??@@@@@@@?????-@
		if (!C::eval(p[1 - w3], t))
		{  // ?-@@@@@@@?????-@
			if (!C::eval(p[-1 + w3], t))
			{  // ?-@@@@@@@-????-@
				return false;
			}  // ?-@@@@@@@@????-@
			if (!C::eval(p[-2 + 2 * w], t))
			{  // ?-@@@@@@@@-???-@
				return false;
			}  // ?-@@@@@@@@@???-@
			if (!C::eval(p[-3 + w], t))
			{  // ?-@@@@@@@@@-??-@
				return false;
			}  // ?-@@@@@@@@@@??-@
			return true;
		}  // ?@@@@@@@@?????-@
		if (!C::eval(p[-w3], t))
		{  // -@@@@@@@@?????-@
			if (!C::eval(p[-1 + w3], t))
			{  // -@@@@@@@@-????-@
				return false;
			}  // -@@@@@@@@@????-@
			if (!C::eval(p[-2 + 2 * w], t))
			{  // -@@@@@@@@@-???-@
				return false;
			}  // -@@@@@@@@@@???-@
			return true;
		}  // @@@@@@@@@?????-@
		return true;
	}  // ??????????????@@
	if (!C::eval(p[-3 - w], t))
	{  // ?????????????-@@
		if (!C::eval(p[1 + w3], t))
		{  // ???????-?????-@@
			return false;
		}  // ???????@?????-@@
		if (!C::eval(p[3 - w], t))
		{  // ???-???@?????-@@
			return false;
		}  // ???@???@?????-@@
		if (!C::eval(p[3], t))
		{  // ???@-??@?????-@@
			return false;
		}  // ???@@??@?????-@@
		if (!C::eval(p[3 + w], t))
		{  // ???@@-?@?????-@@
			return false;
		}  // ???@@@?@?????-@@
		if (!C::eval(p[2 + 2 * w], t))
		{  // ???@@@-@?????-@@
			return false;
		}  // ???@@@@@?????-@@
		if (!C::eval(p[2 - 2 * w], t))
		{  // ??-@@@@@?????-@@
			if (!C::eval(p[w3], t))
			{  // ??-@@@@@-????-@@
				return false;
			}  // ??-@@@@@@????-@@
			if (!C::eval(p[-3], t))
			{  // ??-@@@@@@???--@@
				return false;
			}  // ??-@@@@@@???@-@@
			if (!C::eval(p[-1 + w3], t))
			{  // ??-@@@@@@-??@-@@
				return false;
			}  // ??-@@@@@@@??@-@@
			if (!C::eval(p[-2 + 2 * w], t))
			{  // ??-@@@@@@@-?@-@@
				return false;
			}  // ??-@@@@@@@@?@-@@
			if (!C::eval(p[-3 + w], t))
			{  // ??-@@@@@@@@-@-@@
				return false;
			}  // ??-@@@@@@@@@@-@@
			return true;
		}  // ??@@@@@@?????-@@
		if (!C::eval(p[1 - w3], t))
		{  // ?-@@@@@@?????-@@
			if (!C::eval(p[-3 + w], t))
			{  // ?-@@@@@@???-?-@@
				return false;
			}  // ?-@@@@@@???@?-@@
			if (!C::eval(p[w3], t))
			{  // ?-@@@@@@-??@?-@@
				return false;
			}  // ?-@@@@@@@??@?-@@
			if (!C::eval(p[-1 + w3], t))
			{  // ?-@@@@@@@-?@?-@@
				return false;
			}  // ?-@@@@@@@@?@?-@@
			if (!C::eval(p[-2 + 2 * w], t))
			{  // ?-@@@@@@@@-@?-@@
				return false;
			}  // ?-@@@@@@@@@@?-@@
			return true;
		}  // ?@@@@@@@?????-@@
		if (!C::eval(p[-w3], t))
		{  // -@@@@@@@?????-@@
			if (!C::eval(p[w3], t))
			{  // -@@@@@@@-????-@@
				return false;
			}  // -@@@@@@@@????-@@
			if (!C::eval(p[-1 + w3], t))
			{  // -@@@@@@@@-???-@@
				return false;
			}  // -@@@@@@@@@???-@@
			if (!C::eval(p[-2 + 2 * w], t))
			{  // -@@@@@@@@@-??-@@
				return false;
			}  // -@@@@@@@@@@??-@@
			return true;
		}  // @@@@@@@@?????-@@
		return true;
	}  // ?????????????@@@
	if (!C::eval(p[-w3], t))
	{  // -????????????@@@
		if (!C::eval(p[2 + 2 * w], t))
		{  // -?????-??????@@@
			return false;
		}  // -?????@??????@@@
		if (!C::eval(p[1 + w3], t))
		{  // -?????@-?????@@@
			return false;
		}  // -?????@@?????@@@
		if (!C::eval(p[-2 + 2 * w], t))
		{  // -?????@@??-??@@@
			return false;
		}  // -?????@@??@??@@@
		if (!C::eval(p[w3], t))
		{  // -?????@@-?@??@@@
			return false;
		}  // -?????@@@?@??@@@
		if (!C::eval(p[-1 + w3], t))
		{  // -?????@@@-@??@@@
			return false;
		}  // -?????@@@@@??@@@
		if (!C::eval(p[-3 + w], t))
		{  // -?????@@@@@-?@@@
			if (!C::eval(p[1 - w3], t))
			{  // --????@@@@@-?@@@
				return false;
			}  // -@????@@@@@-?@@@
			if (!C::eval(p[3 + w], t))
			{  // -@???-@@@@@-?@@@
				return false;
			}  // -@???@@@@@@-?@@@
			if (!C::eval(p[2 - 2 * w], t))
			{  // -@-??@@@@@@-?@@@
				return false;
			}  // -@@??@@@@@@-?@@@
			if (!C::eval(p[3 - w], t))
			{  // -@@-?@@@@@@-?@@@
				return false;
			}  // -@@@?@@@@@@-?@@@
			if (!C::eval(p[3], t))
			{  // -@@@-@@@@@@-?@@@
				return false;
			}  // -@@@@@@@@@@-?@@@
			return true;
		}  // -?????@@@@@@?@@@
		if (!C::eval(p[-3], t))
		{  // -?????@@@@@@-@@@
			if (!C::eval(p[3 + w], t))
			{  // -????-@@@@@@-@@@
				return false;
			}  // -????@@@@@@@-@@@
			if (!C::eval(p[2 - 2 * w], t))
			{  // -?-??@@@@@@@-@@@
				return false;
			}  // -?@??@@@@@@@-@@@
			if (!C::eval(p[3 - w], t))
			{  // -?@-?@@@@@@@-@@@
				return false;
			}  // -?@@?@@@@@@@-@@@
			if (!C::eval(p[3], t))
			{  // -?@@-@@@@@@@-@@@
				return false;
			}  // -?@@@@@@@@@@-@@@
			return true;
		}  // -?????@@@@@@@@@@
		return true;
	}  // @????????????@@@
	if (!C::eval(p[-3], t))
	{  // @???????????-@@@
		if (!C::eval(p[2 + 2 * w], t))
		{  // @?????-?????-@@@
			return false;
		}  // @?????@?????-@@@
		if (!C::eval(p[2 - 2 * w], t))
		{  // @?-???@?????-@@@
			return false;
		}  // @?@???@?????-@@@
		if (!C::eval(p[3 - w], t))
		{  // @?@-??@?????-@@@
			return false;
		}  // @?@@??@?????-@@@
		if (!C::eval(p[3 + w], t))
		{  // @?@@?-@?????-@@@
			return false;
		}  // @?@@?@@?????-@@@
		if (!C::eval(p[3], t))
		{  // @?@@-@@?????-@@@
			return false;
		}  // @?@@@@@?????-@@@
		if (!C::eval(p[1 - w3], t))
		{  // @-@@@@@?????-@@@
			if (!C::eval(p[1 + w3], t))
			{  // @-@@@@@-????-@@@
				return false;
			}  // @-@@@@@@????-@@@
			if (!C::eval(p[-3 + w], t))
			{  // @-@@@@@@???--@@@
				return false;
			}  // @-@@@@@@???@-@@@
			if (!C::eval(p[w3], t))
			{  // @-@@@@@@-??@-@@@
				return false;
			}  // @-@@@@@@@??@-@@@
			if (!C::eval(p[-1 + w3], t))
			{  // @-@@@@@@@-?@-@@@
				return false;
			}  // @-@@@@@@@@?@-@@@
			if (!C::eval(p[-2 + 2 * w], t))
			{  // @-@@@@@@@@-@-@@@
				return false;
			}  // @-@@@@@@@@@@-@@@
			return true;
		}  // @@@@@@@?????-@@@
		return true;
	}  // @???????????@@@@
	if (!C::eval(p[1 - w3], t))
	{  // @-??????????@@@@
		if (!C::eval(p[1 + w3], t))
		{  // @-?????-????@@@@
			return false;
		}  // @-?????@????@@@@
		if (!C::eval(p[-3 + w], t))
		{  // @-?????@???-@@@@
			return false;
		}  // @-?????@???@@@@@
		if (!C::eval(p[w3], t))
		{  // @-?????@-??@@@@@
			return false;
		}  // @-?????@@??@@@@@
		if (!C::eval(p[-1 + w3], t))
		{  // @-?????@@-?@@@@@
			return false;
		}  // @-?????@@@?@@@@@
		if (!C::eval(p[-2 + 2 * w], t))
		{  // @-?????@@@-@@@@@
			return false;
		}  // @-?????@@@@@@@@@
		return true;
	}  // @@??????????@@@@
	if (!C::eval(p[2 - 2 * w], t))
	{  // @@-?????????@@@@
		if (!C::eval(p[-3 + w], t))
		{  // @@-????????-@@@@
			return false;
		}  // @@-????????@@@@@
		if (!C::eval(p[w3], t))
		{  // @@-?????-??@@@@@
			return false;
		}  // @@-?????@??@@@@@
		if (!C::eval(p[-1 + w3], t))
		{  // @@-?????@-?@@@@@
			return false;
		}  // @@-?????@@?@@@@@
		if (!C::eval(p[-2 + 2 * w], t))
		{  // @@-?????@@-@@@@@
			return false;
		}  // @@-?????@@@@@@@@
		return true;
	}  // @@@?????????@@@@
	if (!C::eval(p[-3 + w], t))
	{  // @@@????????-@@@@
		if (!C::eval(p[3 - w], t))
		{  // @@@-???????-@@@@
			return false;
		}  // @@@@???????-@@@@
		if (!C::eval(p[3], t))
		{  // @@@@-??????-@@@@
			return false;
		}  // @@@@@??????-@@@@
		if (!C::eval(p[3 + w], t))
		{  // @@@@@-?????-@@@@
			return false;
		}  // @@@@@@?????-@@@@
		return true;
	}  // @@@????????@@@@@
	if (!C::eval(p[-2 + 2 * w], t))
	{  // @@@???????-@@@@@
		if (!C::eval(p[3 - w], t))
		{  // @@@-??????-@@@@@
			return false;
		}  // @@@@??????-@@@@@
		if (!C::eval(p[3], t))
		{  // @@@@-?????-@@@@@
			return false;
		}  // @@@@@?????-@@@@@
		return true;
	}  // @@@???????@@@@@@
	if (!C::eval(p[3 - w], t))
	{  // @@@-??????@@@@@@
		if (!C::eval(p[-1 + w3], t))
		{  // @@@-?????-@@@@@@
			return false;
		}  // @@@-?????@@@@@@@
		return true;
	}  // @@@@??????@@@@@@
	return true;
}

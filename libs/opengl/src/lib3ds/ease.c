/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include <lib3ds/ease.h>


/*!
 * \defgroup ease Ease
 */


/*!
 * \ingroup ease
 */
Lib3dsFloat
lib3ds_ease(Lib3dsFloat fp, Lib3dsFloat fc, Lib3dsFloat fn,
  Lib3dsFloat ease_from, Lib3dsFloat ease_to)
{
  Lib3dsDouble s,step;
  Lib3dsDouble tofrom;
  Lib3dsDouble a;

  s=step=(Lib3dsFloat)(fc-fp)/(fn-fp);
  tofrom=ease_to+ease_from;
  if (tofrom!=0.0) {
    if (tofrom>1.0) {
      ease_to=(Lib3dsFloat)(ease_to/tofrom);
      ease_from=(Lib3dsFloat)(ease_from/tofrom);
    }
    a=1.0/(2.0-(ease_to+ease_from));

    if (step<ease_from) s=a/ease_from*step*step;
    else {
      if ((1.0-ease_to)<=step) {
        step=1.0-step;
        s=1.0-a/ease_to*step*step;
      }
      else {
        s=((2.0*step)-ease_from)*a;
      }
    }
  }
  return((Lib3dsFloat)s);
}

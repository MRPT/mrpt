/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include <lib3ds/tcb.h>
#include <lib3ds/io.h>
#include <math.h>


/*!
 * \defgroup tcb Tension/Continuity/Bias Splines
 */


/*!
 * \ingroup tcb 
 */
void
lib3ds_tcb(Lib3dsTcb *p, Lib3dsTcb *pc, Lib3dsTcb *c, Lib3dsTcb *nc, Lib3dsTcb *n,
  Lib3dsFloat *ksm, Lib3dsFloat *ksp, Lib3dsFloat *kdm, Lib3dsFloat *kdp)
{
  Lib3dsFloat tm,cm,cp,bm,bp,tmcm,tmcp,cc;
  Lib3dsFloat dt,fp,fn;

  if (!pc) {
    pc=c;
  }
  if (!nc) {
    nc=c;
  }
  
  fp=fn=1.0f;
  if (p&&n) {
    dt=0.5f*(Lib3dsFloat)(pc->frame-p->frame+n->frame-nc->frame);
    fp=((Lib3dsFloat)(pc->frame-p->frame))/dt;
    fn=((Lib3dsFloat)(n->frame-nc->frame))/dt;
    cc=(Lib3dsFloat)fabs(c->cont);
    fp=fp+cc-cc*fp;
    fn=fn+cc-cc*fn;
  }

  cm=1.0f-c->cont;
  tm=0.5f*(1.0f-c->tens);
  cp=2.0f-cm;
  bm=1.0f-c->bias;
  bp=2.0f-bm;      
  tmcm=tm*cm;
  tmcp=tm*cp;
  *ksm=tmcm*bp*fp;
  *ksp=tmcp*bm*fp;
  *kdm=tmcp*bp*fn;
  *kdp=tmcm*bm*fn;
}


/*!
 * \ingroup tcb 
 */
Lib3dsBool
lib3ds_tcb_read(Lib3dsTcb *tcb, Lib3dsIo *io)
{
  Lib3dsWord flags;
  
  tcb->frame=lib3ds_io_read_intd(io);
  tcb->flags=flags=lib3ds_io_read_word(io);
  if (flags&LIB3DS_USE_TENSION) {
    tcb->tens=lib3ds_io_read_float(io);
  }
  if (flags&LIB3DS_USE_CONTINUITY) {
    tcb->cont=lib3ds_io_read_float(io);
  }
  if (flags&LIB3DS_USE_BIAS) {
    tcb->bias=lib3ds_io_read_float(io);
  }
  if (flags&LIB3DS_USE_EASE_TO) {
    tcb->ease_to=lib3ds_io_read_float(io);
  }
  if (flags&LIB3DS_USE_EASE_FROM) {
    tcb->ease_from=lib3ds_io_read_float(io);
  }
  if (lib3ds_io_error(io)) {
    return(LIB3DS_FALSE);
  }
  return(LIB3DS_TRUE);
}


/*!
 * \ingroup tcb 
 */
Lib3dsBool
lib3ds_tcb_write(Lib3dsTcb *tcb, Lib3dsIo *io)
{
  lib3ds_io_write_intd(io, tcb->frame);
  lib3ds_io_write_word(io, tcb->flags);
  if (tcb->flags&LIB3DS_USE_TENSION) {
    lib3ds_io_write_float(io, tcb->tens);
  }
  if (tcb->flags&LIB3DS_USE_CONTINUITY) {
    lib3ds_io_write_float(io, tcb->cont);
  }
  if (tcb->flags&LIB3DS_USE_BIAS) {
    lib3ds_io_write_float(io, tcb->bias);
  }
  if (tcb->flags&LIB3DS_USE_EASE_TO) {
    lib3ds_io_write_float(io, tcb->ease_to);
  }
  if (tcb->flags&LIB3DS_USE_EASE_FROM) {
    lib3ds_io_write_float(io, tcb->ease_from);
  }
  if (lib3ds_io_error(io)) {
    return(LIB3DS_FALSE);
  }
  return(LIB3DS_TRUE);
}





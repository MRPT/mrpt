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

#include <mrpt/utils/SSE_types.h>
#include "faster_corner_prototypes.h"

#include "faster_corner_utilities.h"
#include "corner_12.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

#if MRPT_HAS_OPENCV

void fast_corner_detect_plain_12(const IplImage* i, TSimpleFeatureList &corners, int b, uint8_t octave)
{
	int y, cb, c_b;
	const uint8_t  *line_max, *line_min;
	const uint8_t* cache_0;

	int pixel[16] = {
		0 + i->widthStep * 3,
		1 + i->widthStep * 3,
		2 + i->widthStep * 2,
		3 + i->widthStep * 1,
		3 + i->widthStep * 0,
		3 + i->widthStep * -1,
		2 + i->widthStep * -2,
		1 + i->widthStep * -3,
		0 + i->widthStep * -3,
		-1 + i->widthStep * -3,
		-2 + i->widthStep * -2,
		-3 + i->widthStep * -1,
		-3 + i->widthStep * 0,
		-3 + i->widthStep * 1,
		-2 + i->widthStep * 2,
		-1 + i->widthStep * 3,
	};

	for(y = 3 ; y < i->height - 3; y++)
	{
		cache_0 = (const uint8_t*) i->imageData + i->widthStep*y + 3; // &i[y][3];
		line_min = cache_0 - 3;
		line_max = (const uint8_t*) i->imageData + i->widthStep*y+i->width-3; //&i[y][i.size().x - 3];


		for(; cache_0 < line_max;cache_0++)
		{
			cb = *cache_0 + b;
			c_b= *cache_0 - b;

			if(*(cache_0 + pixel[8]) > cb)
			 if(*(cache_0 + pixel[0]) > cb)
			  if(*(cache_0 + pixel[3]) > cb)
			   if(*(cache_0 + pixel[6]) > cb)
			    if(*(cache_0 + pixel[2]) > cb)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + 3) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			          else if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              if(*(cache_0 + -3) > cb)
			               if(*(cache_0 + pixel[13]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               if(*(cache_0 + pixel[9]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + -3) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + -3) > cb)
			               if(*(cache_0 + pixel[13]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + -3) > cb)
			               if(*(cache_0 + pixel[13]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              if(*(cache_0 + -3) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + 3) < c_b)
			       if(*(cache_0 + pixel[9]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[5]) > cb)
			               if(*(cache_0 + pixel[7]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + -3) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[15]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[5]) > cb)
			               if(*(cache_0 + pixel[7]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[9]) > cb)
			       if(*(cache_0 + 3) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[10]) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             goto success;
			            else
			             continue;
			           else if(*(cache_0 + pixel[1]) < c_b)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              if(*(cache_0 + -3) > cb)
			               if(*(cache_0 + pixel[13]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + -3) > cb)
			               if(*(cache_0 + pixel[9]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + -3) > cb)
			               if(*(cache_0 + pixel[13]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              goto success;
			             else if(*(cache_0 + pixel[1]) < c_b)
			              if(*(cache_0 + -3) > cb)
			               if(*(cache_0 + pixel[13]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              if(*(cache_0 + pixel[13]) > cb)
			               if(*(cache_0 + -3) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else if(*(cache_0 + pixel[2]) < c_b)
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + pixel[9]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             if(*(cache_0 + pixel[5]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[1]) > cb)
			               goto success;
			              else
			               continue;
			            else
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[5]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + pixel[10]) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            if(*(cache_0 + pixel[5]) > cb)
			             if(*(cache_0 + 3) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			            else
			             if(*(cache_0 + pixel[1]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else if(*(cache_0 + pixel[6]) < c_b)
			    if(*(cache_0 + pixel[13]) > cb)
			     if(*(cache_0 + pixel[2]) > cb)
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           if(*(cache_0 + pixel[15]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[5]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    if(*(cache_0 + -3) > cb)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + 3) > cb)
			             if(*(cache_0 + pixel[5]) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[5]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + 3) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			  else if(*(cache_0 + pixel[3]) < c_b)
			   if(*(cache_0 + pixel[14]) > cb)
			    if(*(cache_0 + pixel[7]) > cb)
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + -3) > cb)
			       if(*(cache_0 + pixel[9]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[15]) > cb)
			            if(*(cache_0 + pixel[6]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[2]) > cb)
			              goto success;
			             else
			              continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + pixel[5]) > cb)
			            if(*(cache_0 + pixel[6]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else
			   if(*(cache_0 + -3) > cb)
			    if(*(cache_0 + pixel[10]) > cb)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			 else if(*(cache_0 + pixel[0]) < c_b)
			  if(*(cache_0 + 3) > cb)
			   if(*(cache_0 + -3) > cb)
			    if(*(cache_0 + pixel[11]) > cb)
			     if(*(cache_0 + pixel[7]) > cb)
			      if(*(cache_0 + pixel[9]) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[3]) > cb)
			            if(*(cache_0 + pixel[2]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[14]) > cb)
			              goto success;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + pixel[3]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else if(*(cache_0 + 3) < c_b)
			   if(*(cache_0 + -3) < c_b)
			    if(*(cache_0 + pixel[2]) < c_b)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[3]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[1]) < c_b)
			             if(*(cache_0 + pixel[6]) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[1]) < c_b)
			             if(*(cache_0 + pixel[13]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[6]) < c_b)
			             if(*(cache_0 + pixel[1]) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[1]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[10]) < c_b)
			              goto success;
			             else
			              if(*(cache_0 + pixel[6]) < c_b)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[6]) > cb)
			             if(*(cache_0 + pixel[10]) < c_b)
			              if(*(cache_0 + pixel[1]) < c_b)
			               if(*(cache_0 + pixel[13]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else if(*(cache_0 + pixel[6]) < c_b)
			             if(*(cache_0 + pixel[1]) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             if(*(cache_0 + pixel[1]) < c_b)
			              if(*(cache_0 + pixel[10]) < c_b)
			               if(*(cache_0 + pixel[13]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			           else if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[1]) < c_b)
			             if(*(cache_0 + pixel[13]) < c_b)
			              if(*(cache_0 + pixel[10]) < c_b)
			               goto success;
			              else
			               if(*(cache_0 + pixel[6]) < c_b)
			                goto success;
			               else
			                continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[1]) < c_b)
			             if(*(cache_0 + pixel[13]) < c_b)
			              if(*(cache_0 + pixel[6]) < c_b)
			               goto success;
			              else
			               if(*(cache_0 + pixel[10]) < c_b)
			                goto success;
			               else
			                continue;
			             else
			              continue;
			            else
			             continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[6]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else
			   continue;
			 else
			  if(*(cache_0 + -3) > cb)
			   if(*(cache_0 + 3) > cb)
			    if(*(cache_0 + pixel[6]) > cb)
			     if(*(cache_0 + pixel[10]) > cb)
			      if(*(cache_0 + pixel[3]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[1]) > cb)
			              if(*(cache_0 + pixel[2]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[1]) > cb)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else
			   continue;
			else if(*(cache_0 + pixel[8]) < c_b)
			 if(*(cache_0 + pixel[0]) > cb)
			  if(*(cache_0 + -3) > cb)
			   if(*(cache_0 + 3) > cb)
			    if(*(cache_0 + pixel[14]) > cb)
			     if(*(cache_0 + pixel[2]) > cb)
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[11]) > cb)
			              goto success;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else if(*(cache_0 + -3) < c_b)
			   if(*(cache_0 + 3) < c_b)
			    if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + pixel[6]) < c_b)
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              if(*(cache_0 + pixel[3]) < c_b)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[3]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + pixel[14]) < c_b)
			              if(*(cache_0 + pixel[15]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
			            if(*(cache_0 + pixel[7]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + pixel[5]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              if(*(cache_0 + pixel[15]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + pixel[15]) < c_b)
			            if(*(cache_0 + pixel[5]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              if(*(cache_0 + pixel[11]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else
			   continue;
			 else if(*(cache_0 + pixel[0]) < c_b)
			  if(*(cache_0 + pixel[3]) > cb)
			   if(*(cache_0 + -3) < c_b)
			    if(*(cache_0 + pixel[7]) < c_b)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[10]) < c_b)
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[2]) < c_b)
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + pixel[1]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + pixel[5]) < c_b)
			              goto success;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[2]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           if(*(cache_0 + pixel[1]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else if(*(cache_0 + pixel[3]) < c_b)
			   if(*(cache_0 + pixel[10]) > cb)
			    if(*(cache_0 + pixel[14]) < c_b)
			     if(*(cache_0 + pixel[5]) < c_b)
			      if(*(cache_0 + pixel[1]) < c_b)
			       if(*(cache_0 + pixel[2]) < c_b)
			        if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[6]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              goto success;
			             else
			              if(*(cache_0 + pixel[7]) < c_b)
			               goto success;
			              else
			               continue;
			            else
			             if(*(cache_0 + pixel[7]) < c_b)
			              if(*(cache_0 + pixel[9]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[7]) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[7]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             if(*(cache_0 + pixel[13]) < c_b)
			              if(*(cache_0 + pixel[7]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else if(*(cache_0 + pixel[10]) < c_b)
			    if(*(cache_0 + pixel[5]) > cb)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[15]) < c_b)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + pixel[6]) < c_b)
			            if(*(cache_0 + pixel[7]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[2]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[1]) < c_b)
			            if(*(cache_0 + pixel[6]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[5]) < c_b)
			     if(*(cache_0 + pixel[2]) > cb)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[14]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              if(*(cache_0 + 3) < c_b)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[2]) < c_b)
			      if(*(cache_0 + pixel[6]) > cb)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              goto success;
			             else
			              if(*(cache_0 + 3) < c_b)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[6]) < c_b)
			       if(*(cache_0 + 3) > cb)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[14]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              if(*(cache_0 + pixel[7]) < c_b)
			               goto success;
			              else
			               if(*(cache_0 + pixel[1]) < c_b)
			                goto success;
			               else
			                continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + pixel[1]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[11]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             if(*(cache_0 + pixel[13]) < c_b)
			              if(*(cache_0 + pixel[14]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            goto success;
			           else
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[14]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + -3) < c_b)
			            if(*(cache_0 + pixel[14]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[7]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              if(*(cache_0 + pixel[11]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else if(*(cache_0 + pixel[1]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + pixel[13]) < c_b)
			              if(*(cache_0 + -3) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              if(*(cache_0 + pixel[1]) < c_b)
			               goto success;
			              else
			               if(*(cache_0 + pixel[7]) < c_b)
			                goto success;
			               else
			                continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[1]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              goto success;
			             else
			              if(*(cache_0 + 3) < c_b)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + -3) < c_b)
			       if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[6]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              if(*(cache_0 + 3) < c_b)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + -3) < c_b)
			      if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[2]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + pixel[6]) < c_b)
			              if(*(cache_0 + pixel[7]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else
			    if(*(cache_0 + pixel[14]) < c_b)
			     if(*(cache_0 + pixel[6]) < c_b)
			      if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + pixel[13]) < c_b)
			              goto success;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[11]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			  else
			   if(*(cache_0 + -3) < c_b)
			    if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + pixel[7]) < c_b)
			      if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + pixel[2]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[6]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[1]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + pixel[5]) < c_b)
			              goto success;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[2]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[1]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			 else
			  if(*(cache_0 + -3) < c_b)
			   if(*(cache_0 + 3) < c_b)
			    if(*(cache_0 + pixel[6]) < c_b)
			     if(*(cache_0 + pixel[10]) < c_b)
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[3]) > cb)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              if(*(cache_0 + pixel[15]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else if(*(cache_0 + pixel[3]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[15]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              if(*(cache_0 + pixel[11]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[3]) > cb)
			             if(*(cache_0 + pixel[14]) < c_b)
			              if(*(cache_0 + pixel[15]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else if(*(cache_0 + pixel[3]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + pixel[15]) < c_b)
			              if(*(cache_0 + pixel[14]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[1]) < c_b)
			             if(*(cache_0 + pixel[3]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              if(*(cache_0 + pixel[15]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + pixel[7]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + pixel[15]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[7]) < c_b)
			              if(*(cache_0 + pixel[11]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else
			   continue;
			else
			 if(*(cache_0 + pixel[0]) > cb)
			  if(*(cache_0 + 3) > cb)
			   if(*(cache_0 + -3) > cb)
			    if(*(cache_0 + pixel[14]) > cb)
			     if(*(cache_0 + pixel[2]) > cb)
			      if(*(cache_0 + pixel[10]) > cb)
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             goto success;
			            else if(*(cache_0 + pixel[11]) < c_b)
			             if(*(cache_0 + pixel[6]) > cb)
			              if(*(cache_0 + pixel[7]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             if(*(cache_0 + pixel[7]) > cb)
			              if(*(cache_0 + pixel[6]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + pixel[3]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[10]) < c_b)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[7]) > cb)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[5]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              if(*(cache_0 + pixel[5]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else
			   continue;
			 else if(*(cache_0 + pixel[0]) < c_b)
			  if(*(cache_0 + 3) < c_b)
			   if(*(cache_0 + -3) < c_b)
			    if(*(cache_0 + pixel[14]) < c_b)
			     if(*(cache_0 + pixel[2]) < c_b)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[3]) < c_b)
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[1]) < c_b)
			            if(*(cache_0 + pixel[10]) < c_b)
			             if(*(cache_0 + pixel[13]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[6]) < c_b)
			             if(*(cache_0 + pixel[13]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[1]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[6]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + pixel[1]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             if(*(cache_0 + pixel[5]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else
			   continue;
			 else
			  continue;

			success:
				corners.push_back_fast((cache_0-line_min)<<octave, y<<octave);
		}
	}
}

#endif

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// ---------------------------------------------------------------------------
// LICENSING: This file is a slightly-modified version of part of libcvd,
//             released under LGPL 2.1 by Edward Rosten
// ---------------------------------------------------------------------------

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/SSE_types.h>
#include "faster_corner_prototypes.h"

#include "faster_corner_utilities.h"
#include "corner_10.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

#if MRPT_HAS_OPENCV

void fast_corner_detect_plain_10(const IplImage* i, TSimpleFeatureList &corners, int b, uint8_t octave, std::vector<size_t> * out_feats_index_by_row)
{
	size_t *ptr_feat_index_by_row;
	if (out_feats_index_by_row) 
	{
		out_feats_index_by_row->resize(i->height);
		ptr_feat_index_by_row = &(*out_feats_index_by_row)[0];
	}
	else {
		ptr_feat_index_by_row = NULL;
	}


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

	// 3 first rows have no features: 
	if (ptr_feat_index_by_row) {
		*ptr_feat_index_by_row++ = corners.size();
		*ptr_feat_index_by_row++ = corners.size();
		*ptr_feat_index_by_row++ = corners.size();
	}

	for(y = 3 ; y < i->height - 3; y++)
	{
		if (ptr_feat_index_by_row)  // save index by row:
			*ptr_feat_index_by_row++=corners.size();

		cache_0 = (const uint8_t*) i->imageData + i->widthStep*y + 3; // &i[y][3];
		line_min = cache_0 - 3;
		line_max = (const uint8_t*) i->imageData + i->widthStep*y+i->width-3; //&i[y][i.size().x - 3];

		for(; cache_0 < line_max;cache_0++)
		{
			cb = *cache_0 + b;
			c_b= *cache_0 - b;

			if(*(cache_0 + pixel[0]) > cb)
			 if(*(cache_0 + pixel[8]) > cb)
			  if(*(cache_0 + pixel[3]) > cb)
			   if(*(cache_0 + pixel[5]) > cb)
			    if(*(cache_0 + pixel[2]) > cb)
			     if(*(cache_0 + pixel[6]) > cb)
			      if(*(cache_0 + 3) > cb)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			        else if(*(cache_0 + pixel[1]) < c_b)
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
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[1]) > cb)
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
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
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
			      else if(*(cache_0 + 3) < c_b)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[7]) > cb)
			               if(*(cache_0 + pixel[9]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			            else
			             if(*(cache_0 + pixel[7]) > cb)
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
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + -3) > cb)
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + pixel[7]) > cb)
			              if(*(cache_0 + pixel[9]) > cb)
			               goto success;
			              else
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			             else
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			            else if(*(cache_0 + pixel[1]) < c_b)
			             if(*(cache_0 + pixel[7]) > cb)
			              if(*(cache_0 + pixel[9]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             if(*(cache_0 + pixel[9]) > cb)
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
			     else if(*(cache_0 + pixel[6]) < c_b)
			      if(*(cache_0 + -3) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + 3) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[7]) > cb)
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
			      if(*(cache_0 + -3) > cb)
			       if(*(cache_0 + pixel[14]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + 3) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + pixel[7]) > cb)
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
			           if(*(cache_0 + pixel[7]) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
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
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else if(*(cache_0 + pixel[2]) < c_b)
			     if(*(cache_0 + -3) > cb)
			      if(*(cache_0 + pixel[9]) > cb)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + 3) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
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
			          if(*(cache_0 + pixel[1]) > cb)
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
			      continue;
			    else
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + pixel[10]) > cb)
			       if(*(cache_0 + -3) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + 3) > cb)
			            goto success;
			           else if(*(cache_0 + 3) < c_b)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else if(*(cache_0 + pixel[6]) < c_b)
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
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[1]) > cb)
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
			          if(*(cache_0 + pixel[14]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
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
			      continue;
			   else if(*(cache_0 + pixel[5]) < c_b)
			    if(*(cache_0 + pixel[13]) > cb)
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + -3) > cb)
			       if(*(cache_0 + pixel[14]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[7]) > cb)
			             goto success;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[2]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
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
			    if(*(cache_0 + -3) > cb)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[9]) > cb)
			             goto success;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[7]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[10]) < c_b)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + 3) > cb)
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
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
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
			  else if(*(cache_0 + pixel[3]) < c_b)
			   if(*(cache_0 + -3) > cb)
			    if(*(cache_0 + pixel[10]) > cb)
			     if(*(cache_0 + pixel[13]) > cb)
			      if(*(cache_0 + pixel[9]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           goto success;
			          else
			           if(*(cache_0 + pixel[1]) > cb)
			            goto success;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
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
			    continue;
			  else
			   if(*(cache_0 + -3) > cb)
			    if(*(cache_0 + pixel[10]) > cb)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           if(*(cache_0 + pixel[5]) > cb)
			            if(*(cache_0 + pixel[6]) > cb)
			             goto success;
			            else
			             continue;
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
			     else if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + 3) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
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
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + 3) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
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
			       else
			        continue;
			      else
			       continue;
			    else
			     continue;
			   else
			    continue;
			 else if(*(cache_0 + pixel[8]) < c_b)
			  if(*(cache_0 + pixel[11]) > cb)
			   if(*(cache_0 + pixel[2]) > cb)
			    if(*(cache_0 + pixel[15]) > cb)
			     if(*(cache_0 + pixel[1]) > cb)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + 3) > cb)
			           goto success;
			          else
			           if(*(cache_0 + pixel[10]) > cb)
			            goto success;
			           else
			            continue;
			         else
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            if(*(cache_0 + pixel[6]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
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
			   else if(*(cache_0 + pixel[2]) < c_b)
			    if(*(cache_0 + pixel[1]) < c_b)
			     if(*(cache_0 + pixel[3]) < c_b)
			      if(*(cache_0 + 3) < c_b)
			       if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[6]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
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
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else if(*(cache_0 + pixel[11]) < c_b)
			   if(*(cache_0 + pixel[6]) > cb)
			    if(*(cache_0 + pixel[14]) > cb)
			     if(*(cache_0 + pixel[3]) > cb)
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
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
			   else if(*(cache_0 + pixel[6]) < c_b)
			    if(*(cache_0 + pixel[10]) > cb)
			     if(*(cache_0 + pixel[1]) > cb)
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + -3) > cb)
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
			       continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + pixel[5]) > cb)
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[2]) > cb)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + -3) > cb)
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
			        continue;
			      else if(*(cache_0 + pixel[7]) < c_b)
			       if(*(cache_0 + pixel[14]) > cb)
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + pixel[3]) > cb)
			            if(*(cache_0 + 3) > cb)
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
			       else if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + -3) < c_b)
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
			       if(*(cache_0 + -3) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + 3) > cb)
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
			        continue;
			     else if(*(cache_0 + pixel[5]) < c_b)
			      if(*(cache_0 + -3) > cb)
			       if(*(cache_0 + pixel[2]) < c_b)
			        if(*(cache_0 + pixel[3]) < c_b)
			         if(*(cache_0 + 3) < c_b)
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
			      else if(*(cache_0 + -3) < c_b)
			       if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[3]) < c_b)
			            goto success;
			           else
			            continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[14]) < c_b)
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
			       if(*(cache_0 + pixel[2]) < c_b)
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
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
			      if(*(cache_0 + pixel[15]) < c_b)
			       if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
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
			     if(*(cache_0 + -3) > cb)
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
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
			       continue;
			     else
			      continue;
			   else
			    if(*(cache_0 + -3) > cb)
			     if(*(cache_0 + pixel[3]) > cb)
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
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
			      continue;
			    else
			     continue;
			  else
			   if(*(cache_0 + pixel[3]) > cb)
			    if(*(cache_0 + pixel[5]) > cb)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[15]) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            goto success;
			           else
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
			       else if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + 3) > cb)
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
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + 3) > cb)
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
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else if(*(cache_0 + pixel[3]) < c_b)
			    if(*(cache_0 + pixel[1]) < c_b)
			     if(*(cache_0 + pixel[10]) < c_b)
			      if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[6]) < c_b)
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
			      continue;
			    else
			     continue;
			   else
			    continue;
			 else
			  if(*(cache_0 + pixel[3]) > cb)
			   if(*(cache_0 + pixel[14]) > cb)
			    if(*(cache_0 + -3) > cb)
			     if(*(cache_0 + pixel[2]) > cb)
			      if(*(cache_0 + 3) > cb)
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           goto success;
			          else
			           if(*(cache_0 + pixel[5]) > cb)
			            goto success;
			           else
			            continue;
			         else if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
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
			      else if(*(cache_0 + 3) < c_b)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
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
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
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
			    else if(*(cache_0 + -3) < c_b)
			     if(*(cache_0 + pixel[6]) > cb)
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
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
			     if(*(cache_0 + pixel[6]) > cb)
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
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
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + 3) > cb)
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
			     else
			      continue;
			   else
			    continue;
			  else if(*(cache_0 + pixel[3]) < c_b)
			   if(*(cache_0 + pixel[2]) > cb)
			    if(*(cache_0 + pixel[9]) > cb)
			     if(*(cache_0 + pixel[1]) > cb)
			      if(*(cache_0 + pixel[10]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + -3) > cb)
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
			     continue;
			   else
			    continue;
			  else
			   if(*(cache_0 + pixel[9]) > cb)
			    if(*(cache_0 + pixel[2]) > cb)
			     if(*(cache_0 + -3) > cb)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
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
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			else if(*(cache_0 + pixel[0]) < c_b)
			 if(*(cache_0 + pixel[8]) > cb)
			  if(*(cache_0 + pixel[2]) > cb)
			   if(*(cache_0 + pixel[10]) > cb)
			    if(*(cache_0 + pixel[6]) > cb)
			     if(*(cache_0 + pixel[7]) > cb)
			      if(*(cache_0 + pixel[9]) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
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
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + 3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + -3) > cb)
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
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + -3) > cb)
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
			     continue;
			   else
			    continue;
			  else if(*(cache_0 + pixel[2]) < c_b)
			   if(*(cache_0 + pixel[13]) > cb)
			    if(*(cache_0 + pixel[6]) > cb)
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + pixel[9]) > cb)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + 3) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[14]) > cb)
			             goto success;
			            else
			             continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
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
			     if(*(cache_0 + pixel[7]) < c_b)
			      if(*(cache_0 + pixel[1]) < c_b)
			       if(*(cache_0 + pixel[3]) < c_b)
			        if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
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
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else if(*(cache_0 + pixel[13]) < c_b)
			    if(*(cache_0 + pixel[3]) > cb)
			     if(*(cache_0 + pixel[10]) > cb)
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + 3) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
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
			     else if(*(cache_0 + pixel[10]) < c_b)
			      if(*(cache_0 + pixel[9]) < c_b)
			       if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + -3) < c_b)
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
			       continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[3]) < c_b)
			     if(*(cache_0 + pixel[15]) < c_b)
			      if(*(cache_0 + pixel[1]) < c_b)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[10]) < c_b)
			         if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + 3) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + -3) < c_b)
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
			       else if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[6]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[10]) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + -3) < c_b)
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
			       else
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + 3) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            if(*(cache_0 + pixel[14]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[10]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + 3) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
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
			      continue;
			    else
			     if(*(cache_0 + pixel[9]) < c_b)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[10]) < c_b)
			         if(*(cache_0 + -3) < c_b)
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
			       continue;
			     else
			      continue;
			   else
			    if(*(cache_0 + pixel[7]) > cb)
			     if(*(cache_0 + pixel[3]) > cb)
			      if(*(cache_0 + pixel[10]) > cb)
			       if(*(cache_0 + 3) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
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
			    else if(*(cache_0 + pixel[7]) < c_b)
			     if(*(cache_0 + pixel[1]) < c_b)
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[6]) < c_b)
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
			       continue;
			     else
			      continue;
			    else
			     continue;
			  else
			   if(*(cache_0 + -3) > cb)
			    if(*(cache_0 + pixel[6]) > cb)
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + pixel[9]) > cb)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + 3) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[14]) > cb)
			             goto success;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[15]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + 3) > cb)
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
			    continue;
			 else if(*(cache_0 + pixel[8]) < c_b)
			  if(*(cache_0 + 3) > cb)
			   if(*(cache_0 + -3) < c_b)
			    if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[15]) < c_b)
			       if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[2]) < c_b)
			            if(*(cache_0 + pixel[3]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[9]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[3]) < c_b)
			            if(*(cache_0 + pixel[2]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
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
			       if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[6]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
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
			   if(*(cache_0 + pixel[2]) > cb)
			    if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + -3) < c_b)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[15]) > cb)
			            if(*(cache_0 + pixel[5]) < c_b)
			             if(*(cache_0 + pixel[6]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else if(*(cache_0 + pixel[15]) < c_b)
			            goto success;
			           else
			            if(*(cache_0 + pixel[6]) < c_b)
			             if(*(cache_0 + pixel[5]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[1]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[5]) < c_b)
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
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
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
			   else if(*(cache_0 + pixel[2]) < c_b)
			    if(*(cache_0 + pixel[6]) > cb)
			     if(*(cache_0 + pixel[13]) < c_b)
			      if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            goto success;
			           else
			            if(*(cache_0 + pixel[5]) < c_b)
			             goto success;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[10]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[10]) < c_b)
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
			    else if(*(cache_0 + pixel[6]) < c_b)
			     if(*(cache_0 + pixel[3]) > cb)
			      if(*(cache_0 + pixel[9]) < c_b)
			       if(*(cache_0 + pixel[10]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[5]) < c_b)
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
			            if(*(cache_0 + pixel[1]) < c_b)
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
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[3]) < c_b)
			      if(*(cache_0 + pixel[5]) > cb)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            if(*(cache_0 + pixel[1]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + pixel[7]) < c_b)
			              if(*(cache_0 + pixel[9]) < c_b)
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
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[5]) < c_b)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
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
			       else if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[10]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[10]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
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
			       if(*(cache_0 + -3) < c_b)
			        if(*(cache_0 + pixel[14]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + pixel[7]) < c_b)
			              if(*(cache_0 + pixel[9]) < c_b)
			               if(*(cache_0 + pixel[10]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else if(*(cache_0 + pixel[1]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + pixel[9]) < c_b)
			              if(*(cache_0 + pixel[7]) < c_b)
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
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[10]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[7]) > cb)
			            if(*(cache_0 + pixel[1]) < c_b)
			             if(*(cache_0 + pixel[14]) < c_b)
			              if(*(cache_0 + pixel[15]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[5]) < c_b)
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
			            if(*(cache_0 + pixel[15]) < c_b)
			             if(*(cache_0 + pixel[1]) < c_b)
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
			       if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + pixel[3]) < c_b)
			            if(*(cache_0 + pixel[5]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[10]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + pixel[3]) > cb)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[10]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else if(*(cache_0 + pixel[3]) < c_b)
			            goto success;
			           else
			            if(*(cache_0 + pixel[10]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[10]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[3]) < c_b)
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
			   else
			    if(*(cache_0 + pixel[11]) < c_b)
			     if(*(cache_0 + pixel[10]) < c_b)
			      if(*(cache_0 + -3) < c_b)
			       if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
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
			        else if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[6]) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
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
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[1]) < c_b)
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
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[6]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
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
			   if(*(cache_0 + -3) < c_b)
			    if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + pixel[2]) < c_b)
			            if(*(cache_0 + pixel[3]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[7]) < c_b)
			            goto success;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + pixel[2]) < c_b)
			            if(*(cache_0 + pixel[1]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[6]) < c_b)
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
			      continue;
			    else
			     continue;
			   else
			    continue;
			 else
			  if(*(cache_0 + pixel[2]) < c_b)
			   if(*(cache_0 + -3) > cb)
			    if(*(cache_0 + pixel[6]) < c_b)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[3]) < c_b)
			         if(*(cache_0 + 3) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
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
			      else if(*(cache_0 + pixel[7]) < c_b)
			       if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
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
			       if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + 3) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
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
			   else if(*(cache_0 + -3) < c_b)
			    if(*(cache_0 + pixel[3]) > cb)
			     if(*(cache_0 + pixel[9]) < c_b)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
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
			       continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[3]) < c_b)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + 3) > cb)
			       if(*(cache_0 + pixel[10]) < c_b)
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[1]) < c_b)
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
			        continue;
			      else if(*(cache_0 + 3) < c_b)
			       if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[6]) < c_b)
			            if(*(cache_0 + pixel[7]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[11]) < c_b)
			            goto success;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[6]) < c_b)
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
			       if(*(cache_0 + pixel[10]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
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
			     if(*(cache_0 + pixel[9]) < c_b)
			      if(*(cache_0 + pixel[10]) < c_b)
			       if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
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
			    if(*(cache_0 + pixel[6]) < c_b)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + 3) < c_b)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
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
			       else if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
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
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
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
			    else
			     continue;
			  else
			   continue;
			else
			 if(*(cache_0 + pixel[8]) > cb)
			  if(*(cache_0 + pixel[10]) > cb)
			   if(*(cache_0 + 3) > cb)
			    if(*(cache_0 + pixel[2]) > cb)
			     if(*(cache_0 + pixel[6]) > cb)
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           goto success;
			          else if(*(cache_0 + pixel[3]) < c_b)
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
			         else if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + -3) > cb)
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
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
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
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
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
			     else
			      continue;
			    else if(*(cache_0 + pixel[2]) < c_b)
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + -3) > cb)
			       if(*(cache_0 + pixel[9]) > cb)
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[3]) > cb)
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
			     if(*(cache_0 + -3) > cb)
			      if(*(cache_0 + pixel[6]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            goto success;
			           else if(*(cache_0 + pixel[5]) < c_b)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[15]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
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
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
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
			   else if(*(cache_0 + 3) < c_b)
			    if(*(cache_0 + pixel[6]) > cb)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[13]) > cb)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
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
			   else
			    if(*(cache_0 + pixel[14]) > cb)
			     if(*(cache_0 + pixel[6]) > cb)
			      if(*(cache_0 + -3) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
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
			       else if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
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
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
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
			   continue;
			 else if(*(cache_0 + pixel[8]) < c_b)
			  if(*(cache_0 + pixel[10]) < c_b)
			   if(*(cache_0 + 3) > cb)
			    if(*(cache_0 + pixel[14]) < c_b)
			     if(*(cache_0 + pixel[6]) < c_b)
			      if(*(cache_0 + -3) < c_b)
			       if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
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
			   else if(*(cache_0 + 3) < c_b)
			    if(*(cache_0 + pixel[6]) < c_b)
			     if(*(cache_0 + -3) > cb)
			      if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[3]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
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
			       else if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
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
			        if(*(cache_0 + pixel[11]) < c_b)
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
			     else if(*(cache_0 + -3) < c_b)
			      if(*(cache_0 + pixel[7]) < c_b)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[2]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
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
			       else if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[3]) < c_b)
			            goto success;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[15]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
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
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[2]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[3]) < c_b)
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
			      if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[3]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
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
			        if(*(cache_0 + pixel[11]) < c_b)
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
			     continue;
			   else
			    if(*(cache_0 + pixel[14]) < c_b)
			     if(*(cache_0 + pixel[6]) < c_b)
			      if(*(cache_0 + -3) < c_b)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
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
			       else if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
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
			         continue;
			       else
			        if(*(cache_0 + pixel[15]) < c_b)
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

	// 3 last rows have no features: 
	if (ptr_feat_index_by_row) {
		*ptr_feat_index_by_row++ = corners.size();
		*ptr_feat_index_by_row++ = corners.size();
		*ptr_feat_index_by_row++ = corners.size();
	}
}

#endif // MRPT_HAS_OPENCV

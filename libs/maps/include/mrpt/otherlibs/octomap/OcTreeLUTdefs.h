/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

// TODO: convert defines to ENUMs

// Lookup table for neighbor search

  //front
#define LUT_N 0
#define LUT_S 1
#define LUT_E 2
#define LUT_W 3
#define LUT_F 4
#define LUT_R 5

  //edge
#define LUT_NW 6
#define LUT_NE 7
#define LUT_SW 8
#define LUT_SE 9
#define LUT_FN 10
#define LUT_RN 11
#define LUT_FS 12
#define LUT_RS 13
#define LUT_FE 14
#define LUT_FW 15
#define LUT_RE 16
#define LUT_RW 17

  //vertex
#define LUT_FNE 18
#define LUT_FNW 19
#define LUT_FSE 20
#define LUT_FSW 21
#define LUT_RNE 22
#define LUT_RNW 23
#define LUT_RSE 24
#define LUT_RSW 25

  //edge rec.-values
#define LUT_NW_TO_W 3
#define LUT_NW_TO_N 6
#define LUT_NE_TO_E 5
#define LUT_NE_TO_N 7
#define LUT_SW_TO_S 7
#define LUT_SW_TO_W 5
#define LUT_SE_TO_E 7
#define LUT_SE_TO_S 8
#define LUT_FN_TO_F 6
#define LUT_FN_TO_N 10
#define LUT_RN_TO_N 11
#define LUT_RN_TO_R 6
#define LUT_FS_TO_F 8
#define LUT_FS_TO_S 11
#define LUT_RS_TO_R 8
#define LUT_RS_TO_S 12
#define LUT_FE_TO_F 10
#define LUT_FE_TO_E 12
#define LUT_FW_TO_F 11
#define LUT_FW_TO_W 12
#define LUT_RE_TO_R 11
#define LUT_RE_TO_E 14
#define LUT_RW_TO_R 12
#define LUT_RW_TO_W 14

  //vertex rec.values
#define LUT_FNE_TO_E 16
#define LUT_FNE_TO_N 18
#define LUT_FNE_TO_NE 11
#define LUT_FNE_TO_F 14
#define LUT_FNE_TO_FN 8
#define LUT_FNE_TO_FE 4

#define LUT_FNW_TO_W 16
#define LUT_FNW_TO_NW 13
#define LUT_FNW_TO_N 19
#define LUT_FNW_TO_FW 4
#define LUT_FNW_TO_F 15
#define LUT_FNW_TO_FN 9

#define LUT_FSE_TO_S 19
#define LUT_FSE_TO_SE 11
#define LUT_FSE_TO_E 18
#define LUT_FSE_TO_FS 8
#define LUT_FSE_TO_F 16
#define LUT_FSE_TO_FE 6

#define LUT_FSW_TO_SW 13
#define LUT_FSW_TO_S 20
#define LUT_FSW_TO_W 18
#define LUT_FSW_TO_FS 9
#define LUT_FSW_TO_FW 6
#define LUT_FSW_TO_F 17

#define LUT_RNE_TO_R 17
#define LUT_RNE_TO_RE 6
#define LUT_RNE_TO_RN 11
#define LUT_RNE_TO_E 20
#define LUT_RNE_TO_N 22
#define LUT_RNE_TO_NE 15

#define LUT_RNW_TO_RW 6
#define LUT_RNW_TO_R 18
#define LUT_RNW_TO_RN 12
#define LUT_RNW_TO_W 20
#define LUT_RNW_TO_NW 17
#define LUT_RNW_TO_N 23

#define LUT_RSE_TO_RS 11
#define LUT_RSE_TO_R 19
#define LUT_RSE_TO_RE 8
#define LUT_RSE_TO_S 23
#define LUT_RSE_TO_SE 15
#define LUT_RSE_TO_E 22

#define LUT_RSW_TO_RS 12
#define LUT_RSW_TO_RW 8
#define LUT_RSW_TO_R 20
#define LUT_RSW_TO_SW 17
#define LUT_RSW_TO_S 24
#define LUT_RSW_TO_W 22

#define LUT_SELF 0

#define LUT_NO_REC 127
  //#define LUT_ 0


/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 19 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/


#ifndef _ARSOUNDPLAYER_H_
#define _ARSOUNDPLAYER_H_


#include "ArFunctor.h"



/** 
 * @brief This class provides a cross-platform interface for playing short sound samples.
 * (Currently implemented for Windows and Linux).
 * @sa For I/O and network transfer of encoded audio, see the ArNetAudio library.
 * @sa ArSoundsQueue
 *
 * @note Uses an external program to play WAV files on Linux. If an environment
 * variable named PLAY_WAV is set, that program is used, otherwise, 'play' from
 * the 'sox' toolset is used.  PLAY_WAV must contain one word (the command; no arguments)
 * A call to playWavFile() will return immediately after
 * 'play' has finished, even though Linux may still be playing back the sound data. In general,
 * this kind of thing is a problem, especially with speech recognition immediately after playing
 * a sound. Ideally, we should be able to truly block until the sound has finished playback.
 * Alas, it is not an ideal world. Another potential pitfall due to the use of
 * an external program invocation: the program you call must not attempt to
 * issue any output.  'play' from the 'sox' toolset automatically supresses
 * normal output if it isn't called from an interactive terminal, but it may
 * still issue some error messages, which will cause it to hang indefinately.
 *  
 */
class ArSoundPlayer
{
 public:
  /** Play a WAV (Windows RIFF) file 
    * @note Uses an external program to play WAV files on Linux. If an environment
    * variable named PLAY_WAV is set, that program is used, otherwise, 'play' from
    * the 'sox' toolset is used. See detailed note in the overview for this
    * cass.
    * @param filename Name of the file to play
    * @param params ignored
    */
  AREXPORT static bool playWavFile(const char* filename, const char* params);

  /** Play a file in some native file format for the compilation platform. */
  AREXPORT static bool playNativeFile(const char* filename, const char* params);

  /** Cancel (interrupt) any current sound or file playback. */
  AREXPORT static void stopPlaying();

  /** Return the static functor for playWavFile */
  AREXPORT static ArRetFunctor2<bool, const char*, const char*> *getPlayWavFileCallback();

  /** Return the static functor for stopPlaying(). */
  AREXPORT static ArFunctor* getStopPlayingCallback();

  /** Play raw uncompressed PCM16 sound data. The format of this data is 
   *  numSamples samples of two bytes each. Each byte pair is a signed little endian
   *  integer.
   *  The sound will be played back at 16kHz, monaurally.
   *  @return false on error, true on success.
   */
  AREXPORT static bool playSoundPCM16(char* data, int numSamples);
 
protected:
  static int myPlayChildPID; ///< Only used on Linux.
  static ArGlobalRetFunctor2<bool, const char*, const char*> ourPlayWavFileCB;
  static ArGlobalFunctor ourStopPlayingCB;

};
    
  
#endif // _ARSOUNDPLAYER_H_

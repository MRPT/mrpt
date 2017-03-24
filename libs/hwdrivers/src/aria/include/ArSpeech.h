/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*  $Id: ArSpeech.h,v 1.11 2006/08/18 19:43:03 reed Exp $ */

#ifndef ARSPEECH_H
#define ARSPEECH_H

#include "ariaTypedefs.h"
#include "ArFunctor.h"

/** @brief Abstract interface to speech synthesis.  
 *
 *  This class defines the abstract interface for speech synthesizers 
 *  used with Aria.
 *  Implementations are provided in the separate ArSpeechSynth_Cepstral and
 *  ArSpeechSynth_Festival libraries.
 *  This class provides a common-denominator
 *  interface. Implementations (especially ArCepstral) may support more
 *  features, or behave differently; refer to their documentation for more
 *  information.
 */

class ArSpeechSynth
{
public:

  /** Don't forget to call this from derived classes. */
  AREXPORT ArSpeechSynth();  

  AREXPORT virtual ~ArSpeechSynth();


  /** Perform  synthesizer initialization, if neccesary. You must call
   * this method. 
   */
  AREXPORT virtual bool init(void) = 0; 


  /** Speaks the given text. 
   * @param str The text to speak.
   * @param params Voice selection criteria expression
   * @param audioOutputCB If not NULL, send synthesized audio data to this
   * callback (may be called several times). Otherwise, play using default
   * AudioCallback, or directly out the speakers
   * @param sampleRate if given, temporarily use this sample rate for this
   * speech, then restore. If 0, use current sample rate.
   */
  AREXPORT virtual bool speak(const char *str, const char* voiceParams, ArRetFunctor2<bool, ArTypes::Byte2*, int>* audioOutputCB, unsigned short sampleRate = 0) = 0;
  AREXPORT virtual bool speak(const char *str, const char* voiceParams = NULL);

  /** Speaks the given string, using current voice and output settings,
   *  taking varargs and a format string (like printf)
   */
  AREXPORT virtual  bool speakf(const char* fmt, ...) = 0; 

  /** If any speech is currently ongoing, interrupt it. 
   */
  AREXPORT virtual void interrupt() = 0; 

  /** @return a functor for use with ArSoundsQueue */
  AREXPORT ArRetFunctorC<bool, ArSpeechSynth>* getInitCallback();

  /** @return a functor for use with ArSoundsQueue */
  AREXPORT ArRetFunctor2C<bool, ArSpeechSynth, const char*, const char*>* getSpeakCallback(void) ;


  /** @return a functor for interrupt() */
  AREXPORT ArFunctorC<ArSpeechSynth>* getInterruptCallback();


  /** Instead of playing synthesized audio using the synthesizer's internal
   *  audio playback, call the given callback when a chunk of audio has
   *  been synthesized. Audio is passed to the callback in the first parameter
   *  as signed 16-bit samples (PCM16).  The sample rate is 16kHz but may be
   *  changed with setAudioSampleRate(). The second parameter is the number
   *  of samples.  The return value from the callback is ignored.
   */
  AREXPORT void setAudioCallback(ArRetFunctor2<bool, ArTypes::Byte2*, int>* cb);


  /** Change audio sample rate (Hz). Normal rate is 16000 Hz.
   *  Suggested values are 8000, 16000, or 44400
   */
  AREXPORT virtual void setAudioSampleRate(int rate) = 0;

  AREXPORT virtual int getAudioSampleRate() = 0;

  /** Lock, if neccesary */
  /*AREXPORT*/ virtual void lock() { }
  /** Unlock, if neccesary */
  /*AREXPORT*/ virtual void unlock() { }

protected:
  ArRetFunctor2C<bool, ArSpeechSynth, const char*, const char*> mySpeakCB;
  ArRetFunctorC<bool, ArSpeechSynth> myInitCB;
  ArFunctorC<ArSpeechSynth> myInterruptCB;
  ArRetFunctor2<bool, ArTypes::Byte2*, int>* myAudioPlaybackCB; ///< If set, send audio to this callback instead of playing it directly
 

};



#endif

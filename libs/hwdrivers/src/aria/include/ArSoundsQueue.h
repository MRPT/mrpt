/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#ifndef ARSOUNDSQUEUE_H
#define ARSOUNDSQUEUE_H


#include "ariaTypedefs.h"
#include "ArASyncTask.h"
#include "ArCondition.h"
#include "ArSpeech.h"
#include <list>
#include <string>
#include <set>

/**
 * @brief This class manages a queue of items to play as WAV files or as text to
 *  speak using a speech synthesizer.
 *
 * The actual playback of sound and speech is done through callbacks
 * (which you can supply in the constructor or afterwards).
 * Some callbacks you can use are provided by classes like ArSoundPlayer (for sound file
 * playback) and ArFestival (from the ArSpeechSynth_Festival library) and ArCepstral
 * (from the ArSpeechSynth_Cepstral librrary) for speech synthesis.
 *
 * Add sound files to the queue with play(), and text to speak with
 * speak(). Use runAsync() to run
 * the processing thread in the background, or run() to run synchronously in the
 * current thread.
 *
 *
 * @sa ArSoundPlayer
 * @sa @ref soundsQueueExample.cpp
 *
 */

class ArSoundsQueue: public ArASyncTask
{
public:

  /** Opaque identifier for the type of an item in the sound queue. Not used
   * during processing, but may be used to search for items in the queue. */
  enum ItemType { SPEECH, SOUND_FILE, SOUND_DATA, OTHER };

  /** Callback for playing a queue item. First argument is the "data", second is
   * item type-specific "parameters". Return true to continue processing more
   * callbacks in a list, false to cancel processing.
   */
  typedef ArRetFunctor2<bool, const char*, const char*> PlayItemFunctor;
  typedef ArFunctor InterruptItemFunctor;

  /** Callback types for determining whether to commence playing an item,
   * or skipping it.
   */
  typedef ArRetFunctor<bool> PlaybackConditionFunctor;

  /** A sound item in the queue, with callbacks for dealing with the
   * item and the data to pass to those callbacks.
   */
  class Item {
  public:
    std::string data;
    ItemType type;
    std::string params;
    int priority;
    std::list<InterruptItemFunctor*> interruptCallbacks;
    std::list<PlayItemFunctor*> playCallbacks;
    std::list<ArFunctor*> doneCallbacks;
    std::list<PlaybackConditionFunctor*> playbackConditionCallbacks;

    AREXPORT Item();
    AREXPORT Item(std::string _data, ItemType _type, std::string _params = "", int priority = 0);
    AREXPORT Item(std::string _data, ItemType _type, std::string _params, int priority, std::list<PlayItemFunctor*> callbacks);

	AREXPORT Item(const ArSoundsQueue::Item& toCopy);

    /** Note: does not compare priority! */
    bool operator==(const Item& other) const
    {
      return (other.type == type && other.params == params && other.data == data);
    }

    /** Called by sound queue to play this item by calling play callbacks. */
    void play();

    /** Called by sound queue to interrupt this item by calling interrupt
     * callbacks. */
    void interrupt();

    /** Called by sound queue thread after playing this item by calling done
     * callbacks. */
    void done();
  };


  AREXPORT ArSoundsQueue();

  /** @deprecated
   *  @see addInitCallback()
   *  @see setSpeakCallback()
   *  @see setInterruptSpeechCallback()
   *  @see setPlayFileCallback
   *  @see setInterruptFileCallback
   */
  AREXPORT ArSoundsQueue(ArRetFunctor<bool> *speakInitCB,
		    PlayItemFunctor *speakCB = 0,
        InterruptItemFunctor *interruptSpeechCB = 0,
		    ArRetFunctor<bool> *playInitCB = 0,
		    PlayItemFunctor *playFileCB = 0,
        InterruptItemFunctor *interruptFileCB = 0);

  /** Set default speech and WAV file callbacks for use
   * by the convenience methods speak() and play().
   * Omit the last three arguments to use
   * callbacks into ArSoundPlayer.
   */
  AREXPORT ArSoundsQueue(ArSpeechSynth* speechSynthesizer,
		    ArRetFunctor<bool> *playInitCB = 0,
		    PlayItemFunctor *playFileCB = 0,
        InterruptItemFunctor *interruptFileCB = 0);

  AREXPORT virtual ~ArSoundsQueue();

  /** Add a callback to be called when the sound queue begins to run in its
   *  thread.  (For example, the speech synthesizers must be initialized in
   *  the same thread as they are used.)
   */
  /*AREXPORT*/ void addInitCallback(ArRetFunctor<bool> *cb) {
    myInitCallbacks.push_back(cb);
  }

  /** @deprecated */
  /*AREXPORT*/ void setSpeakInitCallback(ArRetFunctor<bool> *cb) {
    addInitCallback(cb);
  }

  /** Add (a copy of) the given item to the queue. */
  AREXPORT void addItem(ArSoundsQueue::Item item);

  /** Create a new queue item with the given data and add to the queue. */
  AREXPORT void addItem(ItemType type, const char* data, std::list<PlayItemFunctor*> callbacks, int priority = 0, const char* params = 0);

  /** Return true if all initialization callbacks have completed, false
   * otherwise.
   */
  /*AREXPORT*/bool isInitialized()
  {
    return myInitialized;
  }

  /** @deprecated
   *  @see isPlaying()
   */
  /*AREXPORT*/ bool isSpeakingOrPlaying(void) { return (myPlayingSomething); }

  /// Returns true if an item is currently being played.
  /*AREXPORT*/ bool isPlaying() { return myPlayingSomething; }


  /** @deprecated
   *  @return true if any queue item is being played.
   *  @see isPlaying()
   */
  /*AREXPORT*/ bool isSpeaking() { return myPlayingSomething; }


  /// Begin processing the sounds queue synchronously (in this thread; does not return)
  /*AREXPORT*/ void run(void) { runInThisThread(); }

  /// Begin processing the sounds queue in a background thread
  /*AREXPORT*/ void runAsync(void) { create(false); }

  /** Temporarily stop processing the sounds queue. (Any currently playing sound
      or speech utterance will finish. The sound device may remain open.)
  */
  AREXPORT void pause();

  /// Resume processing the sounds queue
  AREXPORT void resume() ;

  /** @return true if the queue is paused */
  AREXPORT bool isPaused();

  /// If sound is currently being played or speech is being spoken, interrupt it. (but continue processing the queue). SoundFinished callbacks will not be called.
  AREXPORT void interrupt();

  /** Empty the queue.  If a sound is currently playing, it will not be interrupted.
      QueueEmpty callbacks will be called. SoundFinished callbacks will not be
      called.
  */
  AREXPORT void clearQueue();

  /**  End the processing thread.
   * This function is deprecated. Use stopRunning() instead.
   * @deprecated
      This shuts down the sound queue completely. To
      temporarily stop the queue processing, use pause(). To interrupt the
      currently playing sound, use interrupt().
  */
  AREXPORT void stop() ;

  /// Create and return a new a functor for pause(), which other modules can use to pause this sounds
  /// queue.
  /*AREXPORT*/ArFunctor* getPauseCallback()
  {
    return new ArFunctorC<ArSoundsQueue>(this, &ArSoundsQueue::pause);
  }

  /// Create and return a new functor for resume(), which other modules can use to resume this
  ///  sounds queue.
  /*AREXPORT*/ArFunctor* getResumeCallback()
  {
    return new ArFunctorC<ArSoundsQueue>(this, &ArSoundsQueue::resume);
  }


  /// Get the current size of the speech/sound playback queue.
  /*AREXPORT*/size_t getCurrentQueueSize()
  {
    size_t size;
    lock();
    size = myQueue.size();
    unlock();
    return size;
  }

  /** Add a callback functor to be invoked when playback of one sound or speech utterance starts. */
  /*AREXPORT*/void addSoundStartedCallback(ArFunctor* f)
  {
    myStartPlaybackCBList.push_back(f);
  }

  /** Remove a callback functor to be invoked when playback one sound or speech utterance starts. */
  /*AREXPORT*/void remSoundStartedCallback(ArFunctor* f)
  {
    myStartPlaybackCBList.remove(f);
  }

  /** Add a callback functor to be invoked when plackback of one sound or speech
   * utterance begins. */
  /*AREXPORT*/void addSoundFinishedCallback(ArFunctor* f)
  {
    myEndPlaybackCBList.push_back(f);
  }

  /** Remove a callback functor to be invoked when plackback of one sound or
   * speech utterance finishes. */
  /*AREXPORT*/void remSoundFinishedCallback(ArFunctor* f)
  {
    myEndPlaybackCBList.remove(f);
  }

  /** Add a callback functor to be invoked when a the sound queue becomes
   * non-empty, that is, when a block of sounds/speech utterances begins.
   */
  /*AREXPORT*/void addQueueNonemptyCallback(ArFunctor* f)
  {
    myQueueNonemptyCallbacks.push_back(f);
  }

  /** Remove a functor added by addQueueNonemptyCallback(). */
  /*AREXPORT*/void remQueueNonemptyCallback(ArFunctor* f)
  {
    myQueueNonemptyCallbacks.remove(f);
  }

  /** Add a callback functor to be invoked when the sound queue becomes empty
   * and the last sound has finished playing,
   * that is, when a block of sounds/speech utterances ends. This will not
   * be called when the sound queue first begins running.
   */
  /*AREXPORT*/void addQueueEmptyCallback(ArFunctor* f)
  {
    myQueueEmptyCallbacks.push_back(f);
  }

  /** Remove a functor added by addQueueEmptyCallback() */
  /*AREXPORT*/void remQueueEmptyCallback(ArFunctor* f)
  {
    myQueueEmptyCallbacks.remove(f);
  }



  /** Find items waiting in the queue. This is mainly useful in finding
   *  speech text.
   * @param item  Item to search for.
   * @return A set of positions in the queue. 1 indicates the next sound that will
   * play, followed by 2, etc.
   * @note You have a potential race condition if an item is removed from the queue
   * after this method returns, but before you on the information returned.
   * For best results, pause the sound queue while using this information.
   */
  AREXPORT std::set<int> findPendingItems(const char* item);

  /** Remove pending items with the given data and type. */
  AREXPORT void removePendingItems(const char* item, ItemType type);

  /** Remove pending items with the given data. */
  AREXPORT void removePendingItems(const char* data);

  /** Remove pending items with a priority less than that given. */
  AREXPORT void removePendingItems(int priority);

  /** Remove pending items with priority less the given priority and with the given type. */
  AREXPORT void removePendingItems(int priority, ItemType type);

  /** Remove pending items with the given type. */
  AREXPORT void removePendingItems(ItemType type);

  AREXPORT std::string nextItem(ItemType type);
  AREXPORT std::string nextItem(int priority);
  AREXPORT std::string nextItem(ItemType type, int priority);

  /// Convenience methods for special speech synthesis and WAV file queue items:
  //@{

  /**
   * As a convenience, you may set a "default" speech synthesis callback,
   * and then simply use the speak() method to add a speech item to the queue
   * with those callbacks.
   * @sa Item
   */
  /*AREXPORT*/ void setSpeakCallback(PlayItemFunctor *cb) {
    myDefaultSpeakCB = cb;
  }

  /** Set the "default" callback to interrupt a current speech utterance, used
   * by speak()
   * @sa Item
   */
  /*AREXPORT*/ void setInterruptSpeechCallback(InterruptItemFunctor *cb) {
    myDefaultInterruptSpeechCB = cb;
  }

  /** As a convenience, you may set a "default" WAV file playback callback,
   * and then simply use the play() method to add the file to the queue with
   * this callback.
   * @sa Item
   */
  /*AREXPORT*/ void setPlayFileCallback(PlayItemFunctor *cb) {
    myDefaultPlayFileCB = cb;
  }

  /** @deprecated use setPlayFileCallback() */
  /*AREXPORT*/ void setPlayWavFileCallback(PlayItemFunctor* cb) {
    setPlayFileCallback(cb);
  }

  /** Set the "default" callback to interrupt current wav file playback, for use
   * by the play() convenience method.
   * @sa Item
   */
  /*AREXPORT*/ void setInterruptFileCallback(InterruptItemFunctor *cb) {
    myDefaultInterruptFileCB = cb;
  }

  /** @deprecated use setInterruptFileCallback() */
  /*AREXPORT*/ void setInterruptWavFileCallback(InterruptItemFunctor* cb) {
    setInterruptFileCallback(cb);
  }


  //@}


#if !(defined(WIN32) && defined(_MANAGED))

  /** Add a formatted text string (like printf) to the queue configured for default speech
   * synthesis. When reached in the queue while running, the text will be
   * sent to the "speak" callback, if set, otherwise, the "SythesizeToSound" and
   * "PlaySynthesizedSound" callbacks will be used (if they are set.)
   *
   * @param fmt Format string.
   * @param ... Arguments to format into the format string.
   */
  AREXPORT void speak(const char *fmt, ...);

  /** Speak with alternate voice. */
  AREXPORT void speakWithVoice(const char* voice, const char* fmt, ...);

  /** Speak with alternate priority. */
  AREXPORT void speakWithPriority(int priority, const char* fmt, ...);

  /** Add a sound file to the queue for default sound file playback.
   *
   * @param filename_fmt  Format string for determining the filename of the WAV
   * file, same as printf().
   * @param ... If given, arguments to format into the format string.
   */
  AREXPORT void play(const char *filename_fmt, ...);

#endif // MS Managed C++

  /** Return an item set up for speech with previously set default speech
   * callbacks.
   * @param speech Text to speak (optional)
   * @see setSpeechCallback()
   * @see addItem()
   */
  AREXPORT ArSoundsQueue::Item createDefaultSpeechItem(const char* speech = 0);

  /** Return an item set up for sound file playback with previously set default
   * play callbacks.
   * @param filename Filename to set (optional)
   * @see setPlayFileCallback()
   * @see addItem()
   */
  AREXPORT ArSoundsQueue::Item createDefaultFileItem(const char* filename = 0);

  //@}

  /** Set a playback condition functor used for default speech and sound file
   * items */
  /*AREXPORT*/ void setDefaultPlayConditionCB(PlaybackConditionFunctor* f) {
    myDefaultPlayConditionCB = f;
  }

  /// main function for thread
  /** @internal */
  AREXPORT virtual void *runThread(void *arg);

protected:
  bool myInitialized;
  std::list<Item> myQueue;
  ArMutex myQueueMutex;
  void lock() {
    myQueueMutex.lock();
  }
  void unlock() {
    myQueueMutex.unlock();
  }
  bool tryLock() {
    return myQueueMutex.tryLock();
  }

  /// Functors to invoke when we start running in our thread
  std::list< ArRetFunctor<bool>* > myInitCallbacks;

  bool myPlayingSomething;
  Item myLastItem; ///< The current or most recent item played.

  /// Used by speak() and play() convenience methods:
  //@{
  PlayItemFunctor *myDefaultSpeakCB;
  InterruptItemFunctor *myDefaultInterruptSpeechCB;
  PlayItemFunctor *myDefaultPlayFileCB;
  InterruptItemFunctor *myDefaultInterruptFileCB;
  //@}

  int myPauseRequestCount;  ///< Semaphore for doing pausing and resuming of the queue
  ArCondition myPausedCondition;    ///< Condition variable used by the thread to pause

  /// Functors to invoke during queue processing
  //@{
  std::list<ArFunctor*> myStartPlaybackCBList;
  std::list<ArFunctor*> myEndPlaybackCBList;
  std::list<ArFunctor*> myQueueNonemptyCallbacks;
  std::list<ArFunctor*> myQueueEmptyCallbacks;
  //@}

  PlaybackConditionFunctor* myDefaultPlayConditionCB; ///< Used when creating default configured speech and sound file items

  /// Invoke each functor in a list
  void invokeCallbacks(const std::list<ArFunctor*>& lst);

  /// Invoke each functor in a list, but stop if any of them return false.
  void invokeCallbacks(const std::list<ArRetFunctor<bool>*>& lst);

  /**  push item onto queue
   * @sa addItem()
   */
  //@{
  void pushQueueItem(Item item);
  void pushQueueItem_NoLock(Item item);
  //@}

  /** Pop item from queue and return it */
  //@{
  Item popQueueItem();
  Item popQueueItem_NoLock();
  //@}

};

#endif


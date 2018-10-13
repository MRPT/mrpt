/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/mrptEvent.h>
#include <mrpt/system/CObservable.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/gui/keycodes.h>
#include <mrpt/gui/gui_frwds.h>

#include <mutex>
#include <future>

namespace mrpt::gui
{
/** The base class for GUI window classes.
 *
 *   This class can be observed (see mrpt::system::CObserver) for the following
 * events (see mrpt::system::mrptEvent):
 *   - mrpt::gui::mrptEventWindowChar
 *   - mrpt::gui::mrptEventWindowResize
 *   - mrpt::gui::mrptEventMouseDown
 *   - mrpt::gui::mrptEventWindowClosed
 *
 *  See derived classes to check if they emit other additional events.
 *
 *  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked
 * from the wxWidgets internal MRPT thread,
 *    so all your code in the handler must be thread safe.
 * \ingroup mrpt_gui_grp
 */
class CBaseGUIWindow : public mrpt::system::CObservable
{
	friend class CWindowDialog;
	friend class C3DWindowDialog;
	friend class CWindowDialogPlots;

   private:
	/** can be 200,300,400... See WxSubsystem */
	const int m_CMD_CREATE_WIN;
	/** can be 299,399,499... See WxSubsystem */
	const int m_CMD_DESTROY_WIN;
	void* m_winobj_voidptr;

   protected:
	/** This semaphore will be signaled when the wx window is built and ready.
	 */
	mutable std::promise<void> m_threadReady;
	/** This semaphore will be signaled when the wx window is destroyed. */
	mutable std::promise<void> m_windowDestroyed;
	/** The caption of the window */
	std::string m_caption;
	/** The window handle */
	mrpt::void_ptr_noncopy m_hwnd;

	/* Auxiliary */
	volatile bool m_keyPushed;
	volatile int m_keyPushedCode;
	volatile mrptKeyModifier m_keyPushedModifier;

	/** Must be called by child classes just within the constructor. */
	void createWxWindow(unsigned int initialWidth, unsigned int initialHeight);
	/** Must be called by child classes in their destructors. The code cannot be
	 * put into this class' destructor. */
	void destroyWxWindow();

   public:
	/** Read-only access to the wxDialog object. */
	void* getWxObject() { return m_hwnd.get(); }
	/** Called by wx main thread to set m_hwnd to NULL. */
	void notifyChildWindowDestruction();
	/** Called by wx main thread to signal the semaphore that the wx window is
	 * built and ready. */
	void notifySemThreadReady();

   public:
	/** CMD_DESTROY_WIN can be 299,399,499... See WxSubsystem */

	CBaseGUIWindow(
		void* winobj_voidptr, int CMD_CREATE_WIN, int CMD_DESTROY_WIN,
		const std::string& initial_caption = std::string());
	~CBaseGUIWindow() override;

	/** Returns false if the user has already closed the window.
	 */
	bool isOpen();

	/** Resizes the window, stretching the image to fit into the display area.
	 */
	virtual void resize(unsigned int width, unsigned int height) = 0;

	/** Changes the position of the window on the screen.
	 */
	virtual void setPos(int x, int y) = 0;

	/** Changes the window title text.
	 */
	virtual void setWindowTitle(const std::string& str) = 0;

	/** Gets the last x,y pixel coordinates of the mouse. \return False if the
	 * window is closed. */
	virtual bool getLastMousePosition(int& x, int& y) const = 0;

	/** Set cursor style to default (cursorIsCross=false) or to a cross
	 * (cursorIsCross=true) */
	virtual void setCursorCross(bool cursorIsCross) = 0;

	/** Waits for any key to be pushed on the image or the console, and returns
	 * the key code.
	 *  This method remove key strokes previous to its call, so it will always
	 * wait. To get
	 *   the latest pushed key, see
	 *
	 * \param ignoreControlKeys If set to false, any push of shift, cmd,
	 * control, etc... will make this method to return.
	 * \param out_pushModifier If set to !=nullptr, the modifiers of the key
	 * stroke will be saved here.
	 * \return The virtual key code, as defined in mrptKeyCode (a replication
	 * of wxWidgets key codes).
	 *
	 * \sa getPushedKey, Key codes in the enum mrptKeyCode
	 */
	int waitForKey(
		bool ignoreControlKeys = true,
		mrptKeyModifier* out_pushModifier = nullptr);

	/** Returns true if a key has been pushed, without blocking waiting for a
	 * new key being pushed.
	 * \sa waitForKey, clearKeyHitFlag
	 */
	bool keyHit() const { return m_keyPushed; }
	/** Assure that "keyHit" will return false until the next pushed key.
	 * \sa keyHit, waitForKey
	 */
	void clearKeyHitFlag() { m_keyPushed = false; }
	/** Returns the latest pushed key, or 0 if there is no new key stroke.
	 * \param out_pushModifier If set to !=nullptr, the modifiers of the key
	 * stroke will be saved here.
	 * \return The virtual key code, as defined in <mrpt/gui/keycodes.h> (a
	 * replication of wxWidgets key codes).
	 *
	 * \sa keyHit, waitForKey
	 */
	int getPushedKey(mrptKeyModifier* out_pushModifier = nullptr);

};  // End of class def.

/** @name Events common to all GUI windows:
	@{  */

/**  An event sent by a window upon a char pressed by the user.
 *
 *  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked
 * from the wxWidgets internal MRPT thread,
 *    so all your code in the handler must be thread safe.
 */
class mrptEventWindowChar : public mrpt::system::mrptEvent
{
   protected:
	/** Just to allow this class to be polymorphic */
	void do_nothing() override {}

   public:
	inline mrptEventWindowChar(
		CBaseGUIWindow* obj, int _char_code, mrptKeyModifier _key_mod)
		: source_object(obj), char_code(_char_code), key_modifiers(_key_mod)
	{
	}

	CBaseGUIWindow* source_object;
	/** The virtual key code, as defined in <mrpt/gui/keycodes.h> (a replication
	 * of wxWidgets key codes). */
	int char_code;
	/** Modifiers (Shift, Control, etc...) */
	mrptKeyModifier key_modifiers;
};  // End of class def.

/**  An event sent by a window upon resize.
 *
 *  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked
 * from the wxWidgets internal MRPT thread,
 *    so all your code in the handler must be thread safe.
 */
class mrptEventWindowResize : public mrpt::system::mrptEvent
{
   protected:
	/** Just to allow this class to be polymorphic */
	void do_nothing() override {}

   public:
	inline mrptEventWindowResize(
		CBaseGUIWindow* obj, size_t _new_width, size_t _new_height)
		: source_object(obj), new_width(_new_width), new_height(_new_height)
	{
	}

	CBaseGUIWindow* source_object;
	size_t new_width, new_height;
};  // End of class def.

/**  An event sent by a window upon a mouse click, giving the (x,y) pixel
 * coordinates.
 *
 *  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked
 * from the wxWidgets internal MRPT thread,
 *    so all your code in the handler must be thread safe.
 *
 * \sa mrptEventMouseMove
 */
class mrptEventMouseDown : public mrpt::system::mrptEvent
{
   protected:
	/** Just to allow this class to be polymorphic */
	void do_nothing() override {}

   public:
	inline mrptEventMouseDown(
		CBaseGUIWindow* obj, mrpt::img::TPixelCoord _coords, bool _leftButton,
		bool _rightButton)
		: source_object(obj),
		  coords(_coords),
		  leftButton(_leftButton),
		  rightButton(_rightButton)
	{
	}

	CBaseGUIWindow* source_object;
	mrpt::img::TPixelCoord coords;
	bool leftButton;
	bool rightButton;
};  // End of class def.

/**  An event sent by a window when the mouse is moved over it.
 *  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked
 * from the wxWidgets internal MRPT thread,
 *    so all your code in the handler must be thread safe.
 * \sa mrptEventMouseDown
 */
class mrptEventMouseMove : public mrpt::system::mrptEvent
{
   protected:
	/** Just to allow this class to be polymorphic */
	void do_nothing() override {}

   public:
	inline mrptEventMouseMove(
		CBaseGUIWindow* obj, mrpt::img::TPixelCoord _coords, bool _leftButton,
		bool _rightButton)
		: source_object(obj),
		  coords(_coords),
		  leftButton(_leftButton),
		  rightButton(_rightButton)
	{
	}

	CBaseGUIWindow* source_object;
	mrpt::img::TPixelCoord coords;
	bool leftButton;
	bool rightButton;
};  // End of class def.

/**  An event sent by a window upon when it's about to be closed, either
 * manually by the user or programmatically.
 *   The event field member \a allow_close is default by default, but can be
 * set to false in the event callback
 *   to forbid the window to be closed by the user. If the event corresponds to
 * a programatic close, this field is ignored.
 *
 *  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked
 * from the wxWidgets internal MRPT thread,
 *    so all your code in the handler must be thread safe.
 *
 * \sa CBaseGUIWindow
 */
class mrptEventWindowClosed : public mrpt::system::mrptEvent
{
   protected:
	/** Just to allow this class to be polymorphic */
	void do_nothing() override {}

   public:
	inline mrptEventWindowClosed(CBaseGUIWindow* obj, bool _allow_close = true)
		: source_object(obj), allow_close(_allow_close)
	{
	}
	CBaseGUIWindow* source_object;
	bool allow_close;
};  // End of class def.

/**  @} */

}  // namespace mrpt::gui

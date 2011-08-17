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
#ifndef  CBaseGUIWindow_H
#define  CBaseGUIWindow_H

#include <mrpt/synch.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CObservable.h>
#include <mrpt/utils/safe_pointers.h>
#include <mrpt/utils/TPixelCoord.h>
#include <mrpt/gui/keycodes.h>

#include <mrpt/gui/link_pragmas.h>


namespace mrpt
{
	namespace gui
	{
		using namespace mrpt::utils;

		class CWindowDialog;
		class CWindowDialogPlots;
		class C3DWindowDialog;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_LINKAGE( CBaseGUIWindow, GUI_IMPEXP )

		/** The base class for GUI window classes.
		  *
		  *   This class can be observed (see mrpt::utils::CObserver) for the following events (see mrpt::utils::mrptEvent):
		  *   - mrpt::gui::mrptEventWindowChar
		  *   - mrpt::gui::mrptEventWindowResize
		  *   - mrpt::gui::mrptEventMouseDown
		  *
		  *  See derived classes to check if they emit other additional events.
		  *
		  *  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked from the wxWidgets internal MRPT thread,
		  *    so all your code in the handler must be thread safe.
		  * \ingroup mrpt_gui_grp
		  */
		class GUI_IMPEXP CBaseGUIWindow :
			public mrpt::utils::CObject,
			public mrpt::utils::CObservable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_VIRTUAL_SERIALIZABLE( CBaseGUIWindow )

			friend class CWindowDialog;
			friend class C3DWindowDialog;
			friend class CWindowDialogPlots;

		private:
			const int	m_CMD_CREATE_WIN;  //!< can be 200,300,400... See WxSubsystem
			const int	m_CMD_DESTROY_WIN;  //!< can be 299,399,499... See WxSubsystem
			void*		m_winobj_voidptr;

		protected:
			synch::CSemaphore 	m_semThreadReady;	//!< This semaphore will be signaled when the wx window is built and ready.
			synch::CSemaphore 	m_semWindowDestroyed; //!< This semaphore will be signaled when the wx window is destroyed.
			std::string			m_caption;	//!< The caption of the window
			void_ptr_noncopy	m_hwnd;	//!< The window handle

			/* Auxiliary */
			volatile bool             m_keyPushed;
			volatile int              m_keyPushedCode;
			volatile mrptKeyModifier  m_keyPushedModifier;

			void createWxWindow(unsigned int initialWidth, unsigned int initialHeight); //!< Must be called by child classes just within the constructor.
			void destroyWxWindow(); //!< Must be called by child classes in their destructors. The code cannot be put into this class' destructor.

		public:
			void * getWxObject() { return m_hwnd.get(); } //!< Read-only access to the wxDialog object.
			void notifyChildWindowDestruction();	//!< Called by wx main thread to set m_hwnd to NULL.
			void notifySemThreadReady();	//!< Called by wx main thread to signal the semaphore that the wx window is built and ready.

		public:
			/** CMD_DESTROY_WIN can be 299,399,499... See WxSubsystem */

			CBaseGUIWindow(void* winobj_voidptr, int CMD_CREATE_WIN, int CMD_DESTROY_WIN, const std::string &initial_caption = std::string() );
			virtual ~CBaseGUIWindow();

			/** Returns false if the user has already closed the window.
			  */
			bool isOpen();

			/** Resizes the window, stretching the image to fit into the display area.
			 */
			virtual void  resize( unsigned int width, unsigned int height ) = 0;

			/** Changes the position of the window on the screen.
			 */
			virtual void  setPos( int x, int y ) = 0;

			/** Changes the window title text.
			  */
			virtual void setWindowTitle( const std::string &str )=0;

			/** Gets the last x,y pixel coordinates of the mouse. \return False if the window is closed. */
			virtual bool getLastMousePosition(int &x, int &y) const = 0;

			/** Set cursor style to default (cursorIsCross=false) or to a cross (cursorIsCross=true) */
			virtual void setCursorCross(bool cursorIsCross) = 0;

			/** Waits for any key to be pushed on the image or the console, and returns the key code.
			  *  This method remove key strokes previous to its call, so it will always wait. To get
			  *   the latest pushed key, see
			  *
			  * \param ignoreControlKeys If set to false, any push of shift, cmd, control, etc... will make this method to return.
			  * \param out_pushModifier If set to !=NULL, the modifiers of the key stroke will be saved here.
			  * \return The virtual key code, as defined in mrptKeyCode (a replication of wxWidgets key codes).
			  *
			  * \sa getPushedKey, Key codes in the enum mrptKeyCode
			  */
			int  waitForKey(bool ignoreControlKeys = true, mrptKeyModifier *out_pushModifier=NULL);

			/** Returns true if a key has been pushed, without blocking waiting for a new key being pushed.
			  * \sa waitForKey, clearKeyHitFlag
			  */
			bool  keyHit() const { return m_keyPushed; }

			/** Assure that "keyHit" will return false until the next pushed key.
			  * \sa keyHit, waitForKey
			  */
			void  clearKeyHitFlag() { m_keyPushed = false; }

			/** Returns the latest pushed key, or 0 if there is no new key stroke.
			  * \param out_pushModifier If set to !=NULL, the modifiers of the key stroke will be saved here.
			  * \return The virtual key code, as defined in <mrpt/gui/keycodes.h> (a replication of wxWidgets key codes).
			  *
			  * \sa keyHit, waitForKey
			  */
			int getPushedKey(mrptKeyModifier *out_pushModifier=NULL);


		}; // End of class def.


		/** @name Events common to all GUI windows:
			@{  */

		/**  An event sent by a window upon a char pressed by the user.
		  *
		  *  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked from the wxWidgets internal MRPT thread,
		  *    so all your code in the handler must be thread safe.
		  */
		class GUI_IMPEXP mrptEventWindowChar : public mrptEvent
		{
		protected:
			virtual void do_nothing() { } //!< Just to allow this class to be polymorphic
		public:
			inline mrptEventWindowChar(
				CBaseGUIWindow *obj,
				int	_char_code,
				mrptKeyModifier _key_mod
				) : source_object(obj), char_code(_char_code), key_modifiers(_key_mod) { }

			CBaseGUIWindow *source_object;
			int 			char_code; //!< The virtual key code, as defined in <mrpt/gui/keycodes.h> (a replication of wxWidgets key codes).
			mrptKeyModifier key_modifiers; //!< Modifiers (Shift, Control, etc...)
		}; // End of class def.

		/**  An event sent by a window upon resize.
		  *
		  *  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked from the wxWidgets internal MRPT thread,
		  *    so all your code in the handler must be thread safe.
		  */
		class GUI_IMPEXP mrptEventWindowResize : public mrptEvent
		{
		protected:
			virtual void do_nothing() { } //!< Just to allow this class to be polymorphic
		public:
			inline mrptEventWindowResize(
				CBaseGUIWindow *obj,
				size_t _new_width,
				size_t _new_height) : source_object(obj), new_width(_new_width), new_height(_new_width) { }

			CBaseGUIWindow *source_object;
			size_t new_width, new_height;
		}; // End of class def.

		/**  An event sent by a window upon a mouse click, giving the (x,y) pixel coordinates.
		  *
		  *  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked from the wxWidgets internal MRPT thread,
		  *    so all your code in the handler must be thread safe.
		  *
		  * \sa mrptEventMouseDown
		  */
		class GUI_IMPEXP mrptEventMouseDown : public mrptEvent
		{
		protected:
			virtual void do_nothing() { } //!< Just to allow this class to be polymorphic
		public:
			inline mrptEventMouseDown (
				CBaseGUIWindow *obj,
				mrpt::utils::TPixelCoord  _coords,
				bool   _leftButton,
				bool   _rightButton
				) : source_object(obj), coords(_coords), leftButton(_leftButton), rightButton(_rightButton)
			{ }

			CBaseGUIWindow *source_object;
			mrpt::utils::TPixelCoord  coords;
			bool   leftButton;
			bool   rightButton;
		}; // End of class def.

		/**  @} */

	} // End of namespace

} // End of namespace

#endif

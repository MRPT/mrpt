#ifndef GLWINDOW_H
#define GLWINDOW_H

#include <string>
#include <vector>
#include <map>
#include <cvd/image_ref.h>

namespace CVD {

    namespace Exceptions
    {
	/// %Exceptions specific to CVD::GLWindow
	/// @ingroup gException
	namespace GLWindow
	{
	    /// Base class for all CVD::GLWindow exceptions
	    /// @ingroup gException
	    struct All: public CVD::Exceptions::All{
	    };

	    /// An exception occurred during initialisation
	    /// @ingroup gException
	    struct CreationError: public All
	    {
		CreationError(std::string w); ///< Construct from error string
	    };

	    /// An exception occurred during run-time
	    /// @ingroup gException
	    struct RuntimeError: public All
	    {
		RuntimeError(std::string w); ///< Construct from error string
	    };
	}
    }

    /// An object that creates a window and a GL context attached to that window, and manages its events.
    class GLWindow {
    public:
	/// Symbols for mouse buttons and modifiers
	enum MouseButton { BUTTON_LEFT=1, BUTTON_MIDDLE=2, BUTTON_RIGHT=4, BUTTON_MOD_CTRL=8, BUTTON_MOD_SHIFT=0x10, BUTTON_WHEEL_UP=0x20, BUTTON_WHEEL_DOWN=0x40 };
	/// Symbols for window events
	enum EventType { EVENT_CLOSE, EVENT_EXPOSE };

	/// Abstract base class for event handlers.  Subclass this and override to implement a handler.
	class EventHandler {
	public:
	    virtual ~EventHandler() {}
	    /// Called for key press events
	    virtual void on_key_down(GLWindow&, int /*key*/) {}
	    /// Called for key release events
	    virtual void on_key_up(GLWindow& /*win*/, int /*key*/) {}
	    /// Called for mouse movement events
	    virtual void on_mouse_move(GLWindow& /*win*/, ImageRef /*where*/, int /*state*/) {}
	    /// Called for mouse button press events
	    virtual void on_mouse_down(GLWindow& /*win*/, ImageRef /*where*/, int /*state*/, int /*button*/) {}
	    /// Called for mouse button release events
	    virtual void on_mouse_up(GLWindow& /*win*/, ImageRef /*where*/, int /*state*/, int /*button*/) {}
	    /// Called for window resize events
	    virtual void on_resize(GLWindow& /*win*/, ImageRef /*size*/) {}
	    /// Called for general window events (such as EVENT_CLOSE)
	    virtual void on_event(GLWindow& /*win*/, int /*event*/) {}
	};

	struct Event {
	    enum Type { KEY_DOWN, KEY_UP, MOUSE_MOVE, MOUSE_DOWN, MOUSE_UP, RESIZE, EVENT };
	    Type type;
	    int which, state;
	    ImageRef where, size;
	};

	/// A summary of multiple events
	struct EventSummary {
	    EventSummary() : cursor(-1,-1), cursor_moved(false) {}
	    /// key->frequency mapping for key presses and releases
	    std::map<int,int> key_down, key_up;
	    typedef std::map<int,int>::const_iterator key_iterator;
	    /// button->frequency mapping for mouse presses and releases
	    std::map<int,std::pair<ImageRef,int> > mouse_down, mouse_up;
	    typedef std::map<int,std::pair<ImageRef,int> >::const_iterator mouse_iterator;
	    /// Generic window events -> frequency
	    std::map<int,int> events;
	    /// Reset the summary
	    void clear() { *this = EventSummary(); }
	    /// Has escape been pressed or the close button pressed?
	    bool should_quit() const;
	    /// last seen cursor position from mouse_move
	    ImageRef cursor;
	    /// was the cursor moved during the recording of the history
	    bool cursor_moved;
	};

	/// Construct a GLWindow of the given size and colour depth, with the given title.
	/// A double-buffered GL context is associated with the window.
	/// @param size    Window size
	/// @param bpp     Colour depth
	/// @param title   Window title
	/// @param display X11 display string, passed to XOpenDisplay. "" Is used to indicate NULL. This is ignored for non X11 platforms. 
	GLWindow(const ImageRef& size, int bpp=24, const std::string& title="GLWindow", const std::string& display="") {
	  init(size, bpp, title, display);
	}
	///@overload
	GLWindow(const ImageRef& size, const std::string& title, int bpp=24, const std::string& display="") {
	  init(size, bpp, title, display);
	}

	~GLWindow();
	/// Get the size
	ImageRef size() const;
	/// Set the size
	void set_size(const ImageRef &);
	/// Get the position
	ImageRef position() const;
	/// Set the position
	void set_position(const ImageRef &);
	/// Set the mouse cursor position
	void set_cursor_position(const ImageRef& where);
	/// Get the mouse cursor position
	ImageRef cursor_position() const;
	/// Show (or hide) the cursor
	void show_cursor(bool show=true);
	/// Hide the cursor
	void hide_cursor() { show_cursor(false); }
	/// Get the title
	std::string title() const;
	/// Set the title
	void set_title(const std::string& title);
	/// Swap the front and back buffers
	void swap_buffers();
	/// Handle events in the event queue by calling back to the specified handler.
	void handle_events(EventHandler& handler);
	/// Store all events in the event queue into Event objects.
	void get_events(std::vector<Event>& events);
	/// Make a summary of the events in the queue.
	void get_events(EventSummary& summary);
	/// @returns true if events are pending
	bool has_events() const;
	/// Make this GL context active
	void activate();
	/// Make this GL context active
	void make_current() { activate(); }

    struct State;
    private:
	State* state;
	void init(const ImageRef& size, int bpp, const std::string& title, const std::string& display);
    };


}


#endif

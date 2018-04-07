#include <cvd/glwindow.h>
#include <exception>

#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <GL/glx.h>

using namespace CVD;

CVD::Exceptions::GLWindow::CreationError::CreationError(std::string w)
{
    what="GLWindow creation error: " + w;
}

CVD::Exceptions::GLWindow::RuntimeError::RuntimeError(std::string w)
{
    what="GLWindow error: " + w;
}

struct GLWindow::State {
    ImageRef size;
    ImageRef position;
    std::string title;
    Display* display;
    Window window;
    Atom delete_atom;
    Cursor null_cursor;
    GLXContext context;
};

void CVD::GLWindow::init(const ImageRef& size, int bpp, const std::string& title, const std::string& disp)
{
    Display* display = XOpenDisplay(disp==""?NULL:const_cast<char*>(disp.c_str()));
    if (display == 0)
	throw Exceptions::GLWindow::CreationError("Cannot open X display");

    int visualAttributes[] = {
	GLX_RGBA,
	GLX_DOUBLEBUFFER,
	GLX_RED_SIZE,      bpp/3,
	GLX_GREEN_SIZE,    bpp/3,
	GLX_BLUE_SIZE,     bpp/3,
	GLX_DEPTH_SIZE,    8,
	GLX_STENCIL_SIZE, 8,
	None
    };
    XVisualInfo* visualInfo = glXChooseVisual(display, DefaultScreen(display),visualAttributes);
    if(visualAttributes == 0) {
	XCloseDisplay(display);
	throw Exceptions::GLWindow::CreationError("glXChooseVisual failed");
    }

    Window rootWindow = RootWindow(display, visualInfo->screen);
    XWindowAttributes windowAttributes;

    XGetWindowAttributes(display, rootWindow, &windowAttributes);

    XSetWindowAttributes attributes;
    attributes.border_pixel = 0;
    attributes.colormap = XCreateColormap(display, rootWindow, visualInfo->visual, AllocNone);
    attributes.event_mask = KeyPressMask | KeyReleaseMask | ButtonPressMask | ButtonReleaseMask | PointerMotionMask | StructureNotifyMask | ExposureMask;

    Window window = XCreateWindow(display,
				  rootWindow,
				  0, 0, size.x, size.y,
				  0, visualInfo->depth,
				  InputOutput,
				  visualInfo->visual,
				  CWBorderPixel | CWColormap | CWEventMask,
				  &attributes);
    XStoreName(display, window, title.c_str());
    XClassHint classHint;
	char res_name[] = "cvd";
    classHint.res_class = res_name;
    classHint.res_name = (char *)title.c_str();
    XSetClassHint(display, window, &classHint);
    XMapWindow(display, window);
    XEvent ev;
    do {
        XNextEvent(display,&ev);
    } while (ev.type != MapNotify);

    Atom delete_atom = XInternAtom(display, "WM_DELETE_WINDOW", True);
    XSetWMProtocols(display, window, &delete_atom, 1);

    GLXContext context = glXCreateContext(display, visualInfo, 0, True);
    if (context == 0) {
	XDestroyWindow(display, window);
	XCloseDisplay(display);
	throw Exceptions::GLWindow::CreationError("glXCreateContext failed");
    }

    if (glXMakeCurrent(display, window, context) == False) {
	glXDestroyContext(display, context);
	XDestroyWindow(display, window);
	XCloseDisplay(display);
	throw Exceptions::GLWindow::CreationError("glXMakeCurrent failed");
    }
    glLoadIdentity();
    glViewport(0, 0, size.x, size.y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glColor3f(1.0f,1.0f,1.0f);
    glRasterPos2f(-1, 1);
    glOrtho(-0.375, size.x-0.375, size.y-0.375, -0.375, -1 , 1); //offsets to make (0,0) the top left pixel (rather than off the display)
    glPixelZoom(1,-1);

    XColor black = {0, 0, 0, 0, 0, 0};
    XFontStruct* fixed = XLoadQueryFont(display, "-misc-fixed-medium-r-*-*-12-*-*-*-*-*-*-1" );
    Cursor null_cursor = XCreateGlyphCursor(display, fixed->fid, fixed->fid, ' ', ' ', &black, &black);
    XFreeFont(display, fixed);

    state = new State();
    state->size = size;
    state->title = title;
    state->display = display;
    state->window = window;
    state->delete_atom = delete_atom;
    state->null_cursor = null_cursor;
    state->context = context;
}

CVD::GLWindow::~GLWindow()
{
    glXMakeCurrent(state->display, None, 0);
    glXDestroyContext(state->display, state->context);

    XUnmapWindow(state->display, state->window);
    XDestroyWindow(state->display, state->window);
    XCloseDisplay(state->display);
    delete state;
}

ImageRef CVD::GLWindow::size() const { return state->size; }

void CVD::GLWindow::set_size(const ImageRef & s_){
    // we don't set state->size here, so that it changes through the event system
    // and we react to it there
    XResizeWindow(state->display, state->window, s_.x, s_.y);
}

ImageRef CVD::GLWindow::position() const { return state->position; }

void CVD::GLWindow::set_position(const ImageRef & p_){
    state->position = p_;
    XMoveWindow(state->display, state->window, p_.x, p_.y);
	XFlush(state->display);
}

void CVD::GLWindow::set_cursor_position(const ImageRef& where)
{
    XWarpPointer(state->display, None, state->window, 0, 0, 0, 0, where.x, where.y);
}

ImageRef CVD::GLWindow::cursor_position() const
{
    Window wtmp;
    int itmp;
    unsigned int utmp;
    ImageRef where;
    XQueryPointer(state->display, state->window, &wtmp, &wtmp, &itmp, &itmp, &where.x, &where.y, &utmp);
    return where;
}

void CVD::GLWindow::show_cursor(bool show)
{
    if (show)
	XUndefineCursor(state->display, state->window);
    else
	XDefineCursor(state->display, state->window, state->null_cursor);
}

std::string CVD::GLWindow::title() const
{
    return state->title;
}

void CVD::GLWindow::set_title(const std::string& title)
{
    state->title = title;
    XStoreName(state->display, state->window, title.c_str());
}

void CVD::GLWindow::swap_buffers()
{
    glXSwapBuffers(state->display, state->window);
}

inline int convertButton(unsigned int button)
{
  switch (button) {
  case Button1: return GLWindow::BUTTON_LEFT;
  case Button2: return GLWindow::BUTTON_MIDDLE;
  case Button3: return GLWindow::BUTTON_RIGHT;
  case Button4: return GLWindow::BUTTON_WHEEL_UP;
  case Button5: return GLWindow::BUTTON_WHEEL_DOWN;
  }
  return 0;
}

inline int convertButtonState(unsigned int state)
{
  int ret = 0;
  if (state & Button1Mask) ret |= GLWindow::BUTTON_LEFT;
  if (state & Button2Mask) ret |= GLWindow::BUTTON_MIDDLE;
  if (state & Button3Mask) ret |= GLWindow::BUTTON_RIGHT;
  if (state & ControlMask) ret |= GLWindow::BUTTON_MOD_CTRL;
  if (state & ShiftMask) ret |= GLWindow::BUTTON_MOD_SHIFT;
  return ret;
}

void CVD::GLWindow::handle_events(EventHandler& handler)
{
    XEvent event;
	KeySym k;
    while (XPending(state->display)) {
	XNextEvent(state->display, &event);
	switch (event.type) {
	case ButtonPress:
	    handler.on_mouse_down(*this, ImageRef(event.xbutton.x, event.xbutton.y),
				  convertButtonState(event.xbutton.state), convertButton(event.xbutton.button));
	    break;
	case ButtonRelease:
	    handler.on_mouse_up(*this, ImageRef(event.xbutton.x, event.xbutton.y),
				convertButtonState(event.xbutton.state), convertButton(event.xbutton.button));
	    break;
	case MotionNotify:
	    handler.on_mouse_move(*this, ImageRef(event.xmotion.x, event.xmotion.y), convertButtonState(event.xbutton.state));
	    break;
	case KeyPress:
		{
		XLookupString(&event.xkey, 0, 0, &k, 0);
	    handler.on_key_down(*this, k);
	    break;
		}
	case KeyRelease:
		XLookupString(&event.xkey, 0, 0, &k, 0);
	    handler.on_key_up(*this, k);
	    break;
	    //case UnmapNotify: active = 0; break;
	    //case MapNotify: active = 1; break;
	case ConfigureNotify:
	    if (event.xconfigure.width != state->size.x || event.xconfigure.height != state->size.y) {
		activate();
		state->size = ImageRef(event.xconfigure.width, event.xconfigure.height);
		glViewport(0, 0, state->size.x, state->size.y);
		//glRasterPos2f(0,0);
		//glPixelZoom(float(event.xconfigure.width)/myWidth,-float(event.xconfigure.height)/myHeight);
		handler.on_resize(*this, state->size);
	    }
	    break;
	case Expose:
		handler.on_event(*this, EVENT_EXPOSE);
		break;
	case ClientMessage:
	    if (event.xclient.data.l[0] == (int)state->delete_atom)
		handler.on_event(*this, EVENT_CLOSE);
	    else
		handler.on_event(*this, event.xclient.message_type);
	    break;
	default:
	    handler.on_event(*this, event.type);
	    break;
	}
    }
}

class SaveEvents : public GLWindow::EventHandler {
private:
    std::vector<GLWindow::Event>& events;
public:
    SaveEvents(std::vector<GLWindow::Event>& events_) : events(events_) {}
    void on_key_down(GLWindow&, int key) {
	GLWindow::Event e;
	e.type = GLWindow::Event::KEY_DOWN;
	e.which = key;
	events.push_back(e);
    }
    void on_key_up(GLWindow&, int key) {
	GLWindow::Event e;
	e.type = GLWindow::Event::KEY_UP;
	e.which = key;
	events.push_back(e);
    }

    void on_mouse_move(GLWindow&, ImageRef where, int state) {
	GLWindow::Event e;
	e.type = GLWindow::Event::MOUSE_MOVE;
	e.state = state;
	e.where = where;
	events.push_back(e);
    }

    void on_mouse_down(GLWindow&, ImageRef where, int state, int button) {
	GLWindow::Event e;
	e.type = GLWindow::Event::MOUSE_DOWN;
	e.state = state;
	e.which = button;
	e.where = where;
	events.push_back(e);
    }

    void on_mouse_up(GLWindow&, ImageRef where, int state, int button) {
	GLWindow::Event e;
	e.type = GLWindow::Event::MOUSE_UP;
	e.state = state;
	e.which = button;
	e.where = where;
	events.push_back(e);
    }

    void on_resize(GLWindow&, ImageRef size) {
	GLWindow::Event e;
	e.type = GLWindow::Event::RESIZE;
	e.size = size;
	events.push_back(e);
    }

    void on_event(GLWindow&, int event) {
	GLWindow::Event e;
	e.type = GLWindow::Event::EVENT;
	e.which = event;
	events.push_back(e);
    }
};

void CVD::GLWindow::get_events(std::vector<Event>& events)
{
    SaveEvents saver(events);
    handle_events(saver);
}

bool CVD::GLWindow::EventSummary::should_quit() const
{
    return key_down.count(XK_Escape) || events.count(GLWindow::EVENT_CLOSE);
}

class MakeSummary : public GLWindow::EventHandler {
private:
    GLWindow::EventSummary& summary;
public:
    MakeSummary(GLWindow::EventSummary& summary_) : summary(summary_) {}

    void on_key_down(GLWindow&, int key) {	++summary.key_down[key]; }
    void on_key_up(GLWindow&, int key) { ++summary.key_up[key]; }
    void on_mouse_move(GLWindow&, ImageRef where, int) { summary.cursor = where; summary.cursor_moved = true; }
    void on_mouse_down(GLWindow&, ImageRef where, int state, int button) { summary.mouse_down[button] = std::make_pair(where,state); }
    void on_mouse_up(GLWindow&, ImageRef where, int state, int button) { summary.mouse_up[button] = std::make_pair(where,state); }
    void on_event(GLWindow&, int event) { ++summary.events[event]; }
};

void GLWindow::get_events(EventSummary& summary)
{
    summary.cursor = cursor_position();
    MakeSummary ms(summary);
    handle_events(ms);
}

bool CVD::GLWindow::has_events() const
{
    return XPending(state->display);
}

void CVD::GLWindow::activate()
{
    if (glXMakeCurrent(state->display, state->window, state->context) == False)
	throw Exceptions::GLWindow::RuntimeError("glXMakeCurrent failed");
}


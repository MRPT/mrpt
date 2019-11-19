#include <cvd/glwindow.h>
#include <exception>

#include <windows.h>
#include <windowsx.h>
#include <gl/gl.h>

#include <cassert>
#include <map>
#include <iostream>
using namespace std;

namespace CVD {

	Exceptions::GLWindow::CreationError::CreationError(std::string w)
	{
		what="GLWindow creation error: " + w;
	}

	Exceptions::GLWindow::RuntimeError::RuntimeError(std::string w)
	{
		what="GLWindow error: " + w;
	}

	struct GLWindow::State {
		ImageRef size;
		ImageRef position;
		ImageRef size_offset; // offsets dictated by windows decoration to get proper MoveWindows arguments
		ImageRef position_offset;
		std::string title;

		GLWindow * parent;

		HGLRC   hRC;
		HDC     hDC;
		HWND    hWnd;

		bool has_just_closed;
		bool is_closed;
		bool needs_repaint;
	};

	static map<HWND, GLWindow::State> windowMap;

	LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

	static GLWindow::EventHandler * currentHandler = NULL;

	static bool windowClassRegistered = false;

	void GLWindow::init(const ImageRef& size, int bpp, const std::string& title, const std::string&)
	{
		GLuint		PixelFormat;			// Holds The Results After Searching For A Match
		WNDCLASS	wc;						// Windows Class Structure
		DWORD		dwExStyle;				// Window Extended Style
		DWORD		dwStyle;				// Window Style
		RECT		WindowRect;				// Grabs Rectangle Upper Left / Lower Right Values
		WindowRect.left=(long)0;			// Set Left Value To 0
		WindowRect.right=(long)size.x;		// Set Right Value To Requested Width
		WindowRect.top=(long)0;				// Set Top Value To 0
		WindowRect.bottom=(long)size.y;		// Set Bottom Value To Requested Height

		//fullscreen=fullscreenflag;			// Set The Global Fullscreen Flag
		// Grab An Instance For Our Window   
		HINSTANCE hInstance	= GetModuleHandle(NULL);
		if(!windowClassRegistered){
			wc.style			= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;	// Redraw On Size, And Own DC For Window.
			wc.lpfnWndProc		= (WNDPROC) WndProc;					// WndProc Handles Messages
			wc.cbClsExtra		= 0;									// No Extra Window Data
			wc.cbWndExtra		= 0;									// No Extra Window Data
			wc.hInstance		= hInstance;							// Set The Instance
			wc.hIcon			= LoadIcon(NULL, IDI_WINLOGO);			// Load The Default Icon
			wc.hCursor			= LoadCursor(NULL, IDC_ARROW);			// Load The Arrow Pointer
			wc.hbrBackground	= NULL;									// No Background Required For GL
			wc.lpszMenuName		= NULL;									// We Don't Want A Menu
			wc.lpszClassName	= "glwindow";								// Set The Class Name

			if (!RegisterClass(&wc))
				throw Exceptions::GLWindow::CreationError("Failed to register the Window Class.");
			windowClassRegistered = true;
		}

#if 0
		if (fullscreen)												// Attempt Fullscreen Mode?
		{
			DEVMODE dmScreenSettings;								// Device Mode
			memset(&dmScreenSettings,0,sizeof(dmScreenSettings));	// Makes Sure Memory's Cleared
			dmScreenSettings.dmSize=sizeof(dmScreenSettings);		// Size Of The Devmode Structure
			dmScreenSettings.dmPelsWidth	= width;				// Selected Screen Width
			dmScreenSettings.dmPelsHeight	= height;				// Selected Screen Height
			dmScreenSettings.dmBitsPerPel	= bits;					// Selected Bits Per Pixel
			dmScreenSettings.dmFields=DM_BITSPERPEL|DM_PELSWIDTH|DM_PELSHEIGHT;

			// Try To Set Selected Mode And Get Results.  NOTE: CDS_FULLSCREEN Gets Rid Of Start Bar.
			if (ChangeDisplaySettings(&dmScreenSettings,CDS_FULLSCREEN)!=DISP_CHANGE_SUCCESSFUL)
			{
				// If The Mode Fails, Offer Two Options.  Quit Or Use Windowed Mode.
				if (MessageBox(NULL,"The Requested Fullscreen Mode Is Not Supported By\nYour Video Card. Use Windowed Mode Instead?","NeHe GL",MB_YESNO|MB_ICONEXCLAMATION)==IDYES)
				{
					fullscreen=FALSE;		// Windowed Mode Selected.  Fullscreen = FALSE
				}
				else
				{
					// Pop Up A Message Box Letting User Know The Program Is Closing.
					MessageBox(NULL,"Program Will Now Close.","ERROR",MB_OK|MB_ICONSTOP);
					return FALSE;									// Return FALSE
				}
			}
		}

		if (fullscreen)												// Are We Still In Fullscreen Mode?
		{
			dwExStyle=WS_EX_APPWINDOW;								// Window Extended Style
			dwStyle=WS_POPUP;										// Windows Style
			ShowCursor(FALSE);										// Hide Mouse Pointer
		}
		else
#endif
		{
			dwExStyle=WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;			// Window Extended Style
			dwStyle=WS_OVERLAPPEDWINDOW;							// Windows Style
		}

		RECT oldRect = WindowRect;
		AdjustWindowRectEx(&WindowRect, dwStyle, FALSE, dwExStyle);		// Adjust Window To True Requested Size

		// Create The Window
		HWND hWnd;
		if (!(hWnd=CreateWindowEx(	dwExStyle,							// Extended Style For The Window
			"glwindow",							// Class Name
			NULL, //title.c_str(),								// Window Title
			dwStyle |							// Defined Window Style
			WS_CLIPSIBLINGS |					// Required Window Style
			WS_CLIPCHILDREN,					// Required Window Style
			0, 0,								// Window Position
			WindowRect.right-WindowRect.left,	// Calculate Window Width
			WindowRect.bottom-WindowRect.top,	// Calculate Window Height
			NULL,								// No Parent Window
			NULL,								// No Menu
			hInstance,							// Instance
			NULL)))								// Dont Pass Anything To WM_CREATE
		{
			throw Exceptions::GLWindow::CreationError("Window Creation Error.");
		}

		static	PIXELFORMATDESCRIPTOR pfd=				// pfd Tells Windows How We Want Things To Be
		{
			sizeof(PIXELFORMATDESCRIPTOR),				// Size Of This Pixel Format Descriptor
			1,											// Version Number
			PFD_DRAW_TO_WINDOW |						// Format Must Support Window
			PFD_SUPPORT_OPENGL |						// Format Must Support OpenGL
			PFD_DOUBLEBUFFER,							// Must Support Double Buffering
			PFD_TYPE_RGBA,								// Request An RGBA Format
			bpp,										// Select Our Color Depth
			0, 0, 0, 0, 0, 0,							// Color Bits Ignored
			0,											// No Alpha Buffer
			0,											// Shift Bit Ignored
			0,											// No Accumulation Buffer
			0, 0, 0, 0,									// Accumulation Bits Ignored
			32,											// 32Bit Z-Buffer (Depth Buffer)  
			8,											// 8bit Stencil Buffer
			0,											// No Auxiliary Buffer
			PFD_MAIN_PLANE,								// Main Drawing Layer
			0,											// Reserved
			0, 0, 0										// Layer Masks Ignored
		};

		HDC hDC;
		if (!(hDC=GetDC(hWnd)))
			throw Exceptions::GLWindow::CreationError("Can't create a GL Device Context.");

		if (!(PixelFormat=ChoosePixelFormat(hDC,&pfd)))
			throw Exceptions::GLWindow::CreationError("Can't find a suitable PixelFormat.");

		if(!SetPixelFormat(hDC,PixelFormat,&pfd))
			throw Exceptions::GLWindow::CreationError("Can't Set The PixelFormat.");

		HGLRC hRC;
		if (!(hRC=wglCreateContext(hDC)))
			throw Exceptions::GLWindow::CreationError("Can't Create A GL Rendering Context.");

		if(!wglMakeCurrent(hDC,hRC))
			throw Exceptions::GLWindow::CreationError("Can't Activate The GL Rendering Context.");

		ShowWindow(hWnd,SW_SHOW);						// Show The Window
		SetForegroundWindow(hWnd);						// Slightly Higher Priority
		SetFocus(hWnd);									// Sets Keyboard Focus To The Window

		glLoadIdentity();
		glViewport(0, 0, size.x, size.y);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glColor3f(1.0f,1.0f,1.0f);
		glOrtho(0, size.x, size.y, 0, -1 , 1);
		glPixelZoom(1,-1);
		glRasterPos2f(0, 0);

		state = &windowMap[hWnd];
		state->parent = this;
		state->size = size;
		state->title = title;
		state->hRC = hRC;
		state->hDC = hDC;
		state->hWnd = hWnd;

		state->size_offset.x = (WindowRect.right - WindowRect.left) - (oldRect.right - oldRect.left);
		state->size_offset.y = (WindowRect.bottom - WindowRect.top) - (oldRect.bottom - oldRect.top);
		state->position_offset.x = WindowRect.left - oldRect.left;
		state->position_offset.y = WindowRect.top - oldRect.top;

		state->position = -state->position_offset;

		state->has_just_closed = false;
		state->is_closed = false;
		state->needs_repaint = false;

		// handle events to make window appear
		EventSummary summary;
		get_events(summary);

		// CWK: actually set window title
		set_title(title);
	}

	void markGLWindowAsClosed(GLWindow::State* state) {
		if (state == NULL || state->is_closed) {
			return;
		}

		if (!state->is_closed) {
			state->has_just_closed = true;
			state->is_closed = true;
		}
	}

	void closeGLWindow(GLWindow::State* state) {
		if (state == NULL || state->is_closed) {
			return;
		}

		if(state->hRC){
			if (wglGetCurrentContext() == state->hRC)
				if (!wglMakeCurrent(NULL,NULL))	
					throw Exceptions::GLWindow::RuntimeError("Release of DC and RC failed.");
			if (!wglDeleteContext(state->hRC))
				throw Exceptions::GLWindow::RuntimeError("Release Rendering Context failed.");

			state->hRC = NULL;
		}

		if (state->hDC && !ReleaseDC(state->hWnd,state->hDC)) {
			throw Exceptions::GLWindow::RuntimeError("Release Device Context failed.");

			state->hDC = NULL;
		}

		if (state->hWnd && !DestroyWindow(state->hWnd)) {
			throw Exceptions::GLWindow::RuntimeError("Destroy Window failed.");

			// We don't want to set hWnd to NULL here, as we need it in the destructor to remove state from windowMap.
		}

#if 0
		if (!UnregisterClass("glwindow",GetModuleHandle(NULL)))
			throw Exceptions::GLWindow::RuntimeError("Could not unregister Class.");
#endif

		markGLWindowAsClosed(state);
	}

	ImageRef GLWindow::size() const {
		if (state == NULL || state->is_closed) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		return state->size; 
	}

	GLWindow::~GLWindow()
	{
		closeGLWindow(state);
		windowMap.erase(state->hWnd);
		state = NULL;
	}

	void GLWindow::set_size(const ImageRef & s_){
		if (state == NULL || state->is_closed) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		state->size = s_;
		MoveWindow(state->hWnd, state->position.x + state->position_offset.x, state->position.y + state->position_offset.y, state->size.x + state->size_offset.x, state->size.y + state->size_offset.y, FALSE);
	}

	ImageRef GLWindow::position() const { 
		if (state == NULL || state->is_closed) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		return state->position; 
	}

	void GLWindow::set_position(const ImageRef & p_){
		if (state == NULL) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		state->position = p_;
		MoveWindow(state->hWnd, state->position.x + state->position_offset.x, state->position.y + state->position_offset.y, state->size.x + state->size_offset.x, state->size.y + state->size_offset.y, FALSE);
	}

	void GLWindow::set_cursor_position(const ImageRef& where)
	{
		// FIXME
		//XWarpPointer(state->display, None, state->window, 0, 0, 0, 0, where.x, where.y);
	}

	ImageRef GLWindow::cursor_position() const
	{
		if (state == NULL || state->is_closed) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		ImageRef where;
		POINT point;
		GetCursorPos(&point);
		where.x = point.x - state->position.x;
		where.y = point.y - state->position.y;
		return where;
	}

	void GLWindow::show_cursor(bool show)
	{
		if (state == NULL || state->is_closed) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		if (show)
			ShowCursor(TRUE);
		else
			ShowCursor(FALSE);
	}

	std::string GLWindow::title() const
	{
		if (state == NULL || state->is_closed) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		return state->title;
	}

	void GLWindow::set_title(const std::string& title)
	{
		if (state == NULL || state->is_closed) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		state->title = title;
		SetWindowText(state->hWnd, state->title.c_str());
	}

	void GLWindow::swap_buffers()
	{
		if (state == NULL || state->is_closed) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		SwapBuffers(state->hDC);
	}

	inline int convertButtonState(const WPARAM state)
	{
		int ret = 0;
		if (state & MK_LBUTTON) ret |= GLWindow::BUTTON_LEFT;
		if (state & MK_MBUTTON) ret |= GLWindow::BUTTON_MIDDLE;
		if (state & MK_RBUTTON) ret |= GLWindow::BUTTON_RIGHT;
		if (state & MK_CONTROL) ret |= GLWindow::BUTTON_MOD_CTRL;
		if (state & MK_SHIFT)   ret |= GLWindow::BUTTON_MOD_SHIFT;
		return ret;
	}

	inline ImageRef convertPosition(LPARAM param)
	{
		return ImageRef(GET_X_LPARAM(param), GET_Y_LPARAM(param));
	}

	void GLWindow::handle_events(EventHandler& handler)
	{
		if (state != NULL && state->has_just_closed) {
			state->has_just_closed = false;
			handler.on_event(*this, EVENT_CLOSE);
			return;
		}

		if (state == NULL || state->is_closed) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		if (state->needs_repaint) {
			state->needs_repaint = false;
			handler.on_event(*this, EVENT_EXPOSE);
		}

		MSG	msg;

		currentHandler = &handler; // for events only received in Window Procedure

		while(PeekMessage(&msg, state->hWnd, 0, 0, PM_REMOVE)){
			//TranslateMessage(&msg);  // don't care for WM_CHAR/WM_DEADCHAR messages
			switch(msg.message){
			case WM_LBUTTONDOWN:
				handler.on_mouse_down(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_LEFT);
				break;
			case WM_LBUTTONUP:
				handler.on_mouse_up(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_LEFT);
				break;
			case WM_MBUTTONDOWN:
				handler.on_mouse_down(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_MIDDLE);
				break;
			case WM_MBUTTONUP:
				handler.on_mouse_up(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_MIDDLE);
				break;
			case WM_RBUTTONDOWN:
				handler.on_mouse_down(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_RIGHT);
				break;
			case WM_RBUTTONUP:
				handler.on_mouse_up(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam), GLWindow::BUTTON_RIGHT);
				break;
			case WM_MOUSEWHEEL:
				// positive forward, negative backward, FIXME check correspondence to X11 implementation
				handler.on_mouse_up(*this, convertPosition(msg.lParam), convertButtonState(GET_KEYSTATE_WPARAM(msg.wParam)), (GET_WHEEL_DELTA_WPARAM(msg.wParam) > 0) ? GLWindow::BUTTON_WHEEL_UP : GLWindow::BUTTON_WHEEL_DOWN);
				break;
			case WM_MOUSEMOVE:
				handler.on_mouse_move(*this, convertPosition(msg.lParam), convertButtonState(msg.wParam));
				break;
			case WM_KEYDOWN:
				{
					unsigned char state[256];
					GetKeyboardState(state);
					char buffer[4];
					int res = ToAscii((UINT)msg.wParam, (UINT)(msg.lParam & 0xffffff) >> 16, state, (LPWORD)buffer, 0);
					if(res == 1){
						handler.on_key_down(*this, buffer[0]);
					} else {
						handler.on_key_down(*this, (int)msg.wParam);
					}
				}
				break;
			case WM_KEYUP:
				{
					unsigned char state[256];
					GetKeyboardState(state);
					char buffer[4];
					int res = ToAscii((UINT)msg.wParam, (UINT)(msg.lParam & 0xffffff) >> 16, state, (LPWORD)buffer, 0);
					if(res == 1){
						handler.on_key_up(*this, buffer[0]);
					} else {
						handler.on_key_up(*this, (int)msg.wParam);
					}
				}
				break;
			case WM_PAINT:
				// This will never be called here, it will always be in WndProc...
				handler.on_event(*this, EVENT_EXPOSE);
			default:
				DispatchMessage(&msg);
			}
		}

		currentHandler = NULL;
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

	void GLWindow::get_events(std::vector<Event>& events)
	{
		SaveEvents saver(events);
		handle_events(saver);
	}

	bool GLWindow::EventSummary::should_quit() const
	{
		return key_down.count(VK_ESCAPE) || events.count(GLWindow::EVENT_CLOSE);
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
		if (state != NULL && !state->is_closed) {
			summary.cursor = cursor_position();
		}

		MakeSummary ms(summary);
		handle_events(ms);
	}

	bool GLWindow::has_events() const
	{
		if (state == NULL || state->is_closed) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		MSG	msg;
		return PeekMessage(&msg, state->hWnd, 0, 0, PM_NOREMOVE) != 0;
	}

	void GLWindow::activate()
	{
		if (state == NULL || state->is_closed) {
			throw Exceptions::GLWindow::RuntimeError("Window is not open.");
		}

		if(!wglMakeCurrent(state->hDC,state->hRC))
			throw Exceptions::GLWindow::RuntimeError("wglMakeCurrent failed");
	}

	LRESULT CALLBACK WndProc(	HWND	hWnd,			// Handle For This Window
		UINT	uMsg,			// Message For This Window
		WPARAM	wParam,			// Additional Message Information
		LPARAM	lParam)			// Additional Message Information
	{
		switch(uMsg){
		case WM_WINDOWPOSCHANGED:
			if(windowMap.count(hWnd) == 1){
				GLWindow::State & state = windowMap[hWnd];
				WINDOWPOS * pos = (WINDOWPOS *)lParam;
				ImageRef newSize(pos->cx, pos->cy);
				newSize -= state.size_offset;
				if(newSize != state.size){
					state.size = newSize;
					state.parent->activate();
					glViewport(0, 0, state.size.x, state.size.y);
					if(currentHandler != NULL)
						currentHandler->on_resize(*state.parent, state.size);
					else
						cerr << "Event outside of cvd control for " << state.title << endl;
				}
				state.position = ImageRef(pos->x, pos->y) - state.position_offset;
				return 0;
			}
			break;
		case WM_CLOSE:
			if(windowMap.count(hWnd) == 1) {
				GLWindow::State& state = windowMap[hWnd];
				closeGLWindow(&state);

				return 0;
			}

			break;
		case WM_DESTROY:
			if(windowMap.count(hWnd) == 1) {
				GLWindow::State& state = windowMap[hWnd];
				// This should happen as part of closeGLWindow when we receive WM_CLOSE, but it doesn't hurt to double-check.
				markGLWindowAsClosed(&state);

				return 0;
			}

			break;
		case WM_PAINT: 
			if(windowMap.count(hWnd) == 1) {
				GLWindow::State& state = windowMap[hWnd];
				state.needs_repaint = true;
				// We don't want to return 0 here, as we don't actually paint anything straight away 
				// (and so Windows will keep sending messages until we do).
			}

			break;
		}
		// Pass All Unhandled Messages To DefWindowProc
		return DefWindowProc(hWnd,uMsg,wParam,lParam);
	}

} // namespace CVD

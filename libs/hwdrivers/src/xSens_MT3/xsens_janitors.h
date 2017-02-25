/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _JANITORS_H_2006_05_01
#define _JANITORS_H_2006_05_01

// required for gnu c++ compiler due to difference in attribute declarations
#if defined(__GNUC__) && !defined(__cdecl)
#   define __cdecl   // __attribute__((cdecl))   // JLBC @ MRPT: GCC warns about ignored attribute
#   define __stdcall // __attribute__((stdcall)) // JLBC @ MRPT: GCC warns about ignored attribute
#endif

namespace xsens {

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Value restoring janitor class

	This class can be used to make sure that the value that is in the variable at the time
	the janitor is created will be in it again when the janitor leaves scope.
*/
template <class T>
class JanitorRestore {
private:
	T& m_control;
	T  m_value;
	bool m_enabled;
public:
	JanitorRestore<T>(T& control, bool enabl = true) :
		m_control(control), m_value(control), m_enabled(enabl) {}
	~JanitorRestore<T>()
	{
		if (m_enabled)
			m_control = m_value;
	}

	void disable(void)
		{ m_enabled = false; }

	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Memory releasing janitor class

	This class can be used to make sure that the associated pointer is freed when the
	janitor leaves scope.
*/
template <class T>
class JanitorFree {
private:
	T* m_control;
	bool m_enabled;
public:
	JanitorFree<T>(T* control, bool enabl = true) :
		m_control(control), m_enabled(enabl) {}
	~JanitorFree<T>()
	{
		if (m_enabled)
			free(m_control);
	}

	void disable(void)
		{ m_enabled = false; }

	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Memory releasing janitor class

	This class can be used to make sure that the associated object is deleted when the
	janitor leaves scope.
*/
template <class T>
class JanitorDelete {
private:
	T* m_control;
	bool m_enabled;
public:
	JanitorDelete<T>(T* control, bool enabl = true) :
		m_control(control), m_enabled(enabl) {}
	~JanitorDelete<T>()
	{
		if (m_enabled)
			delete m_control;
	}

	void disable(void)
		{ m_enabled = false; }

	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Memory releasing janitor class

	This class can be used to make sure that the associated object is deleted when the
	janitor leaves scope.
*/
template <class T>
class JanitorDeleteArray {
private:
	T* m_control;
	bool m_enabled;
public:
	JanitorDeleteArray<T>(T* control, bool enabl = true) :
		m_control(control), m_enabled(enabl) {}
	~JanitorDeleteArray<T>()
	{
		if (m_enabled)
			delete[] m_control;
	}

	void disable(void)
		{ m_enabled = false; }

	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Class function calling janitor class

	This class can be used to make sure that the given class function is called when the
	janitor leaves scope.
*/
template <class T, typename R = void>
class JanitorClassFunc {
public:
	typedef R (T::*t_func_JanitorClasssFunc)(void);
private:
	T& m_control;
	t_func_JanitorClasssFunc m_funcJCF;
	bool m_enabled;
public:

	JanitorClassFunc<T,R>(T& control, t_func_JanitorClasssFunc func, bool enabl = true) :
		m_control(control), m_funcJCF(func), m_enabled(enabl)
	{
	}
	~JanitorClassFunc<T,R>()
	{
		if (m_enabled)
			(m_control.*m_funcJCF)();
	}

	void disable(void)
		{ m_enabled = false; }

	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Function calling janitor class

	This class can be used to make sure that the given function is called on the given
	object when the janitor leaves scope. Take care that the object is not of a type that
	is destroyed before the function unrolling begins.
*/
template <class T, typename R = void>
class JanitorFunc {
public:
	typedef R (__cdecl * t_func_JanitorFunc)(T);
private:
	T& m_control;
	t_func_JanitorFunc m_funcJF;
	bool m_enabled;
public:

	JanitorFunc<T,R>(t_func_JanitorFunc func, T& control, bool enabl = true) :
		m_funcJF(func), m_control(control), m_enabled(enabl) {}
	~JanitorFunc<T,R>()
	{
		if (m_enabled)
			(*m_funcJF)(m_control);
	}

	void disable(void)
		{ m_enabled = false; }

	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Log / printf-like function calling janitor class

	This class can be used to make sure that the given printf-like function is called with the
	supplied parameter when the janitor leaves scope. Take care that the object is not of a type that
	is destroyed before the function unrolling begins.
*/
template <class T, class C, typename R = void>
class JanitorLogFunc {
public:
	typedef R (__cdecl * t_func_JanitorLogFunc)(const char*,...);
private:
	const char* m_str;
	T& m_control;
	t_func_JanitorLogFunc m_funcJF;
	bool m_enabled;
public:

	JanitorLogFunc<T,C,R>(t_func_JanitorLogFunc func, const char* str, T& control, bool enable = true) :
		m_funcJF(func), m_str(str), m_control(control), m_enabled(enable) {}
	~JanitorLogFunc<T,C,R>()
	{
		if (m_enabled)
			(*m_funcJF)(m_str,(C) m_control);
	}

	void disable(void)
		{ m_enabled = false; }

	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Function calling janitor class

	This class can be used to make sure that the given function is called on the given
	object when the janitor leaves scope. Take care that the object is not of a type that
	is destroyed before the function unrolling begins.
*/
template <class T, typename R = void>
class JanitorFuncStdCall {
public:
	typedef R (__stdcall * t_func_JanitorFuncStdCall)(T);
private:
	T& m_control;
	t_func_JanitorFuncStdCall m_funcJFSC;
	bool m_enabled;
public:

	JanitorFuncStdCall<T,R>(t_func_JanitorFuncStdCall func, T& control, bool enabl = true) :
		m_funcJFSC(func), m_control(control), m_enabled(enabl) {}
	~JanitorFuncStdCall<T,R>()
	{
		if (m_enabled)
			(*m_funcJFSC)(m_control);
	}

	void disable(void)
		{ m_enabled = false; }

	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Value restoring janitor class

	This class can be used to make sure that the value that is in the variable at the time
	the janitor is created will be in it again when the janitor leaves scope.
*/
template <class T>
class JanitorSet {
private:
	T& m_control;
	T  m_value;
	bool m_enabled;
public:
	JanitorSet<T>(T& control, const T& val, bool enabl = true) :
		m_control(control), m_value(val), m_enabled(enabl) {}
	~JanitorSet<T>()
	{
		if (m_enabled)
			m_control = m_value;
	}

	void disable(void)
		{ m_enabled = false; }

	void enable(void)
		{ m_enabled = true; }
};

}	// end of xsens namespace

#endif	// _JANITORS_H_2006_05_01


//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSENS_JANITORS_H
#define XSENS_JANITORS_H

#ifndef __AVR32__
#include <functional>
#endif

// required for gnu c++ compiler versions due to difference in attribute declarations
#if defined(__AVR32__)
#	define __cdecl
#	define __stdcall
#elif defined(_ADI_COMPILER)
#   define __cdecl
#   define __stdcall
#elif defined(__GNUC__) && !defined(HAVE_CDECL)
#	if !defined(__cdecl)
#		if defined(__x86_64__)
#			define __cdecl
#		else
#   define __cdecl __attribute__((cdecl))
#		endif
#	endif
#	if !defined(__stdcall)
#		if defined(__x86_64__)
#			define __stdcall
#		else
#   define __stdcall __attribute__((stdcall))
#endif
#	endif
#endif

namespace xsens {

#ifndef NOT_FOR_PUBLIC_RELEASE
//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Value restoring janitor class
	\details This class can be used to make sure that the value that is in the variable at the time
	the janitor is created will be in it again when the janitor leaves scope.
*/
template <class T>
class JanitorRestore {
private:
	T& m_control;
	T  m_value;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorRestore<T>(T& control, bool enabl = true) :
		m_control(control), m_value(control), m_enabled(enabl) {}

	/*! \brief Destructor
	*/
	~JanitorRestore()
	{
		if (m_enabled)
			m_control = m_value;
	}

	/*! \brief Disables the value restoring
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the value restoring
	*/
	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Memory releasing janitor class
	\details This class can be used to make sure that the associated pointer is freed when the
	janitor leaves scope.
*/
template <class T>
class JanitorFree {
private:
	T* m_control;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorFree<T>(T* control, bool enabl = true) :
		m_control(control), m_enabled(enabl) {}

	/*! \brief Destructor
	*/
	~JanitorFree()
	{
		if (m_enabled)
			free(m_control);
	}

	/*! \brief Disables the memory releasing
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the memory releasing
	*/
	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Memory releasing janitor class
	\details This class can be used to make sure that the associated object is deleted when the
	janitor leaves scope.
*/
template <class T>
class JanitorDelete {
private:
	T* m_control;
	bool m_enabled;
public:

	/*! \brief Default constructor
	*/
	JanitorDelete<T>() :
		m_control(NULL),
		m_enabled(true)
	{

	}

	/*! \brief Constructor
	*/
	JanitorDelete<T>(T* control, bool enabl = true) :
		m_control(control),
		m_enabled(enabl)
	{

	}

	/*! \brief Destructor
	*/
	~JanitorDelete()
	{
		if (m_enabled)
			delete m_control;
	}

	/*! \brief Sets the control object
		\param control The control object
	*/
	void setControl(T* control)
		{ m_control = control; }

	/*! \brief Disables the memory releasing
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the memory releasing
	*/
	void enable(void)
		{ m_enabled = true; }

	/*! \returns A const pointer to the data. This does not detach/copy the data.
	*/
	const T *operator->() const { return m_control;	}

	/*! \returns A pointer to the data. This does not detach/copy the data.
	*/
	T *operator->() { return m_control; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Memory releasing and nulling janitor class
	\details This class can be used to make sure that the associated object is deleted when the
	janitor leaves scope and the referenced pointer is set to NULL.
*/
template <class T>
class JanitorDeleteNull {
private:
	T*& m_control;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorDeleteNull<T>(T*& control, bool enabl = true) :
		m_control(control),
		m_enabled(enabl)
	{

	}

	/*! \brief Destructor
	*/
	~JanitorDeleteNull()
	{
		if (m_enabled)
		{
			delete m_control;
			m_control = 0;
		}
	}

	/*! \brief Disables the memory releasing and nulling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the memory releasing and nulling
	*/
	void enable(void)
		{ m_enabled = true; }

	/*! \returns A const pointer to the data. This does not detach/copy the data.
	*/
	const T *operator->() const { return m_control;	}

	/*! \returns A pointer to the data. This does not detach/copy the data.
	*/
	T *operator->() { return m_control; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Memory releasing janitor class
	\details This class can be used to make sure that the associated object is deleted when the
	janitor leaves scope.
*/
template <class T>
class JanitorDeleteArray {
private:
	T* m_control;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorDeleteArray<T>(T* control, bool enabl = true) :
		m_control(control), m_enabled(enabl) {}

	/*! \brief Destructor
	*/
	~JanitorDeleteArray()
	{
		if (m_enabled)
			delete[] m_control;
	}

	/*! \brief Disables the memory releasing
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the memory releasing
	*/
	void enable(void)
		{ m_enabled = true; }
};
#endif

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Class function calling janitor class
	\details This class can be used to make sure that the given class function is called when the
	janitor leaves scope.
*/
template <class T, typename R = void>
class JanitorClassFunc {
public:
	typedef R (T::*t_func_JanitorClassFunc)(void); //!< A function prototype for a janitor class
private:
	const JanitorClassFunc& operator = (const JanitorClassFunc&);

	T& m_control;
	t_func_JanitorClassFunc m_funcJCF;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorClassFunc<T,R>(T& control, t_func_JanitorClassFunc func, bool enabl = true) :
		m_control(control), m_funcJCF(func), m_enabled(enabl)
	{
	}

	/*! \brief Destructor
	*/
	~JanitorClassFunc()
	{
		if (m_enabled)
			(m_control.*m_funcJCF)();
	}

	/*! \brief Disables the function calling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the function calling
	*/
	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Class function calling janitor class with a parameter
	\details This class can be used to make sure that the given class function is called when the
	janitor leaves scope.
*/
template <class T, typename P1, typename R = void>
class JanitorClassFuncP1 {
public:
	typedef R (T::*t_func_JanitorClassFunc)(P1); //!< A function prototype for a janitor class
private:
	const JanitorClassFuncP1& operator = (const JanitorClassFuncP1&);

	T& m_control;
	P1 m_param1;
	t_func_JanitorClassFunc m_funcJCF;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorClassFuncP1<T,P1,R>(T& control, P1 p1, t_func_JanitorClassFunc func, bool enabl = true) :
		m_control(control), m_param1(p1), m_funcJCF(func), m_enabled(enabl)
	{
	}

	/*! \brief Destructor
	*/
	~JanitorClassFuncP1()
	{
		if (m_enabled)
			(m_control.*m_funcJCF)(m_param1);
	}

	/*! \brief Disables the function calling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enable the function calling
	*/
	void enable(void)
		{ m_enabled = true; }
};

#ifndef NOT_FOR_PUBLIC_RELEASE
//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Function calling janitor class for function with 0 parameters
	\details This class can be used to make sure that the given function is called on the given
	object when the janitor leaves scope. Take care that the object is not of a type that
	is destroyed before the function unrolling begins.
*/
template <typename ResultType = void>
class JanitorFunc0 {
public:
	typedef ResultType (__cdecl * t_func_JanitorFunc)(void); //!< A function prototype for a janitor function
private:
	const JanitorFunc0& operator = (const JanitorFunc0&);
	t_func_JanitorFunc m_funcJF;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorFunc0<ResultType>(t_func_JanitorFunc func, bool enabl = true) :
		m_funcJF(func), m_enabled(enabl) {}

	/*! \brief Destructor
	*/
	~JanitorFunc0()
	{
		if (m_enabled)
			(*m_funcJF)();
	}

	/*! \brief Disables the function calling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the function calling
	*/
	void enable(void)
		{ m_enabled = true; }
};

#ifndef __AVR32__
//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Function calling janitor class for std::function with 0 parameters
	\details This class can be used to make sure that the given function is called on the given
	object when the janitor leaves scope. Take care that the object is not of a type that
	is destroyed before the function unrolling begins.
*/
template <typename ResultType = void>
class JanitorStdFunc0 {
public:
	typedef std::function<ResultType (void)> t_func_JanitorFunc; //!< A function prototype for a janitor function
private:
	const JanitorStdFunc0& operator = (const JanitorStdFunc0&);
	t_func_JanitorFunc m_funcJF;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorStdFunc0<ResultType>(t_func_JanitorFunc func, bool enabl = true) :
		m_funcJF(func), m_enabled(enabl) {}

	/*! \brief Destructor
	*/
	~JanitorStdFunc0()
	{
		if (m_enabled)
			m_funcJF();
	}

	/*! \brief Disables the function calling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the function calling
	*/
	void enable(void)
		{ m_enabled = true; }
};
#endif

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Function calling janitor class for function with 1 parameter
	\details This class can be used to make sure that the given function is called on the given
	object when the janitor leaves scope. Take care that the object is not of a type that
	is destroyed before the function unrolling begins.
*/
template <class ParamType, typename ResultType = void>
class JanitorFunc1 {
public:
	typedef ResultType (__cdecl * t_func_JanitorFunc)(ParamType); //!< A function prototype for a janitor function
private:
	const JanitorFunc1& operator = (const JanitorFunc1&);
	ParamType m_control;
	t_func_JanitorFunc m_funcJF;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorFunc1<ParamType,ResultType>(t_func_JanitorFunc func, ParamType control, bool enabl = true) :
		m_control(control), m_funcJF(func), m_enabled(enabl) {}

	/*! \brief Destructor
	*/
	~JanitorFunc1()
	{
		if (m_enabled)
			(*m_funcJF)(m_control);
	}

	/*! \brief Disables the function calling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the function calling
	*/
	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Function calling janitor class for function with 1 reference parameter
	\details This class can be used to make sure that the given function is called on the given
	object when the janitor leaves scope. Take care that the object is not of a type that
	is destroyed before the function unrolling begins.
*/
template <class ParamType, typename ResultType = void>
class JanitorFunc1R {
public:
	typedef ResultType (__cdecl * t_func_JanitorFunc)(ParamType); //!< A function prototype for a janitor function
private:
	const JanitorFunc1R& operator = (const JanitorFunc1R&);
	ParamType& m_control;
	t_func_JanitorFunc m_funcJF;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorFunc1R<ParamType,ResultType>(t_func_JanitorFunc func, ParamType& control, bool enabl = true) :
		m_control(control), m_funcJF(func), m_enabled(enabl) {}

	/*! \brief Destructor
	*/
	~JanitorFunc1R()
	{
		if (m_enabled)
			(*m_funcJF)(m_control);
	}

	/*! \brief Disables the function calling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the function calling
	*/
	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Function calling janitor class for function with 2 parameters
	\details This class can be used to make sure that the given function is called on the given
	object when the janitor leaves scope. Take care that the object is not of a type that
	is destroyed before the function unrolling begins.
*/
template <class Param1Type, class Param2Type, typename ResultType = void>
class JanitorFunc2 {
public:
	typedef ResultType (__cdecl * t_func_JanitorFunc)(Param1Type,Param2Type); //!< A function prototype for a janitor function
private:
	const JanitorFunc2& operator = (const JanitorFunc2&);
	Param1Type m_control1;
	Param2Type m_control2;
	t_func_JanitorFunc m_funcJF;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorFunc2<Param1Type,Param2Type,ResultType>(t_func_JanitorFunc func, Param1Type control1, Param2Type control2, bool enabl = true) :
		m_funcJF(func), m_control1(control1), m_control2(control2), m_enabled(enabl) {}

	/*! \brief Destructor
	*/
	~JanitorFunc2()
	{
		if (m_enabled)
			(*m_funcJF)(m_control1,m_control2);
	}

	/*! \brief Disables the function calling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the function calling
	*/
	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Function calling janitor class for function with 2 reference parameters
	\details This class can be used to make sure that the given function is called on the given
	object when the janitor leaves scope. Take care that the object is not of a type that
	is destroyed before the function unrolling begins.
*/
template <class Param1Type, class Param2Type, typename ResultType = void>
class JanitorFunc2R {
public:
	typedef ResultType (__cdecl * t_func_JanitorFunc)(Param1Type,Param2Type); //!< A function prototype for a janitor function
private:
	const JanitorFunc2R& operator = (const JanitorFunc2R&);
	Param1Type& m_control1;
	Param2Type& m_control2;
	t_func_JanitorFunc m_funcJF;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorFunc2R<Param1Type,Param2Type,ResultType>(t_func_JanitorFunc func, Param1Type& control1, Param2Type& control2, bool enabl = true) :
		m_funcJF(func), m_control1(control1), m_control2(control2), m_enabled(enabl) {}

	/*! \brief Destructor
	*/
	~JanitorFunc2R()
	{
		if (m_enabled)
			(*m_funcJF)(m_control1,m_control2);
	}

	/*! \brief Disables the function calling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the function calling
	*/
	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Log / printf-like function calling janitor class
	\details This class can be used to make sure that the given printf-like function is called with the
	supplied parameter when the janitor leaves scope. Take care that the object is not of a type that
	is destroyed before the function unrolling begins.
*/
template <class T, class C, typename R = void>
class JanitorLogFunc {
public:
	typedef R (__cdecl * t_func_JanitorLogFunc)(const char*, const char*, ...); //!< A function prototype for a janitor log function
private:
	const JanitorLogFunc& operator = (const JanitorLogFunc&);
	t_func_JanitorLogFunc m_funcJF;
	const char* m_filter;
	const char* m_str;
	const char* m_functionName;
	T& m_control;
	bool m_enabled;
public:

	/*! \brief Constructor with additional 'function name' parameter
	*/
	JanitorLogFunc<T,C,R>(t_func_JanitorLogFunc func, const char *filter, const char *str, const char *functionName, T& control, bool enable = true) :
		m_funcJF(func), m_filter(filter), m_str(str), m_functionName(functionName), m_control(control), m_enabled(enable)
	{}

	/*! \brief Constructor
	*/
	JanitorLogFunc<T,C,R>(t_func_JanitorLogFunc func, const char *filter, const char* str, T& control, bool enable = true) :
		m_funcJF(func), m_filter(filter), m_str(str), m_functionName(0), m_control(control), m_enabled(enable) {}

	/*! \brief Destructor
	*/
	~JanitorLogFunc()
	{
		if (m_enabled)
		{
			if (m_functionName)
				(*m_funcJF)(m_filter, m_str, m_functionName, (C) m_control);
			else
				(*m_funcJF)(m_filter, m_str, (C) m_control);
		}
	}

	/*! \brief Disables the log function calling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the log function calling
	*/
	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Log / printf-like function calling janitor class
	\details This class can be used to make sure that the given printf-like function is called with the
	supplied string when the janitor leaves scope.
*/
template <typename R = void>
class JanitorSimpleLogFunc {
public:
	typedef R (__cdecl * t_func_JanitorSimpleLogFunc)(const char*, const char*, ...); //!< A function prototype for a janitor simple log function
private:
	const JanitorSimpleLogFunc& operator = (const JanitorSimpleLogFunc&);
	t_func_JanitorSimpleLogFunc m_funcJF;
	const char* m_filter;
	const char* m_str;
	const char* m_functionName;
	bool m_enabled;
public:

	/*! \brief Constructor with additional 'function name' parameter
	*/
	JanitorSimpleLogFunc<R>(t_func_JanitorSimpleLogFunc func, const char *filter, const char *str, const char *functionName, bool enable = true) :
		m_funcJF(func), m_filter(filter), m_str(str), m_functionName(functionName), m_enabled(enable)
	{}

	/*! \brief Constructor
	*/
	JanitorSimpleLogFunc<R>(t_func_JanitorSimpleLogFunc func, const char *filter, const char* str, bool enable = true) :
		m_funcJF(func), m_filter(filter), m_str(str), m_functionName(0), m_enabled(enable) {}

	/*! \brief Destructor
	*/
	~JanitorSimpleLogFunc()
	{
		if (m_enabled)
		{
			if (m_functionName)
				(*m_funcJF)(m_filter, m_str, m_functionName);
			else
				(*m_funcJF)(m_filter, m_str);
		}
	}

	/*! \brief Disables the log function calling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the log function calling
	*/
	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Function calling janitor class
	\details This class can be used to make sure that the given function is called on the given
	object when the janitor leaves scope. Take care that the object is not of a type that
	is destroyed before the function unrolling begins.
*/
template <class ParamType, typename ResultType = void>
class JanitorFuncStdCall {
public:
	typedef ResultType (__stdcall * t_func_JanitorFuncStdCall)(ParamType); //!< A function prototype for a calling janitor function
private:
	const JanitorFuncStdCall& operator = (const JanitorFuncStdCall&);
	ParamType& m_control;
	t_func_JanitorFuncStdCall m_funcJFSC;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorFuncStdCall<ParamType,ResultType>(t_func_JanitorFuncStdCall func, ParamType& control, bool enabl = true) :
		m_funcJFSC(func), m_control(control), m_enabled(enabl) {}

	/*! \brief Destructor
	*/
	~JanitorFuncStdCall()
	{
		if (m_enabled)
			(*m_funcJFSC)(m_control);
	}

	/*! \brief Disables the function calling
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the function calling
	*/
	void enable(void)
		{ m_enabled = true; }
};

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Value restoring janitor class
	\details This class can be used to make sure that the value that is in the variable at the time
	the janitor is created will be in it again when the janitor leaves scope.
*/
template <class T>
class JanitorSet {
private:
	const JanitorSet& operator = (const JanitorSet&);
	T& m_control;
	T  m_value;
	bool m_enabled;
public:

	/*! \brief Constructor
	*/
	JanitorSet<T>(T& control, const T& val, bool enabl = true) :
		m_control(control), m_value(val), m_enabled(enabl) {}

	/*! \brief Destructor
	*/
	~JanitorSet()
	{
		if (m_enabled)
			m_control = m_value;
	}

	/*! \brief Disables the value restoring
	*/
	void disable(void)
		{ m_enabled = false; }

	/*! \brief Enables the value restoring
	*/
	void enable(void)
		{ m_enabled = true; }
};


//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Reference counting janitor class
	\details This class can be used to make sure that the associated pointer is deleted when the
	last janitor for it leaves scope.
*/
template <typename T>
class JanitorRCDeallocationPolicy_Delete
{
public:
	/*! \brief Deallocates (deletes) a pointer to an object
		\param p The pointer to an object to delete
	*/
	static void Deallocate(T* p) {delete p;}
protected:
	/*! \brief Destructor
		\note Always use non-virtual protected destructors in policies
	*/
	~JanitorRCDeallocationPolicy_Delete() {}
};

/*! \brief Reference counting janitor class
	\details This class can be used to make sure that the associated pointer is freed when the
	last janitor for it leaves scope.
*/
template <typename T>
class JanitorRCDeallocationPolicy_Free
{
public:
	/*! \brief Deallocates (frees) a pointer to an object
		\param p The pointer to an object to free
	*/
	static void Deallocate(T* p) {free(p);}
protected:
	/*! \brief Destructor
		\note Always use non-virtual protected destructors in policies
	*/
	~JanitorRCDeallocationPolicy_Free() {}
};

/*! \brief Reference counting janitor class
	\details This class can be used to make sure that the associated pointer is deleted/freed when the
	last janitor for it leaves scope.
*/
template <class T, template <class> class DeallocationPolicy>
class JanitorRCDeallocation {
public:
	typedef JanitorRCDeallocation<T, DeallocationPolicy> ThisType;	//!< A type definition for this type
	typedef T ElementType;											//!< A type definition for an element type
	typedef void (*PostDeallocateFunc)();							//!< A function prototype for a post deallocation function

	/*! \brief Constructor with an element type
	*/
	JanitorRCDeallocation(ElementType *target = 0, bool enabled=true)
		: m_target(target)
		, m_refCounter(0)
		, m_enabled(enabled)
	{
		addRef();
	}

	/*! \brief Constructor with this type
	*/
	JanitorRCDeallocation(const ThisType &j, bool enabled=true)
		: m_target(j.m_target)
		, m_refCounter(j.m_refCounter)
		, m_enabled(enabled)
	{
		addRef();
	}

	/*! \brief Destructor
	*/
	virtual ~JanitorRCDeallocation() { removeRef(); }

	/*! \brief Copy the data from \a rhs
		\param rhs The janitor deallication class
		\returns this
	*/
	JanitorRCDeallocation& operator=(const JanitorRCDeallocation &rhs)
	{
		if (m_target != rhs.m_target) {
			removeRef();
			m_target = rhs.m_target;
			m_refCounter = rhs.m_refCounter;
			addRef();
		}
		return *this;
	}
	/*! \returns A const pointer to the data. This does not detach/copy the data.
	*/
	ElementType* operator->() const
	{
		return m_target;
	}

	/*! \returns A reference to the data. This does not detach/copy the data.
	*/
	ElementType& operator*() const
	{
		return *m_target;
	}

	/*! \brief Disables the reference counting
	*/
	void disable(void)
	{
		m_enabled = false;
	}

	/*! \brief Enables the reference counting
	*/
	void enable(void)
	{
		m_enabled = true;
	}

	/*! \returns A pointer to an element type
	*/
	ElementType* get() const throw()
	{
		return m_target;
	}

	/*! \brief Swaps the current object's variables with an other object's variables
		\param other The reference to an other object
	*/
	void swap(JanitorRCDeallocation& other) throw()
	{
		std::swap(m_target, other.m_target);
		std::swap(m_refCounter, other.m_refCounter);
		std::swap(m_enabled, other.m_enabled);
	}

	/*! \brief Swaps an element type with this type
		\details If no argument is passed the object will be reset
		\param p The pointer to an element type
	*/
	void reset(ElementType* p = 0) throw()
	{
		if (m_target != p) {
			ThisType(p).swap(*this);
		}
	}

private:

	/*! \brief Adds a reference to a reference counter
		\note If there was no reference counter, then it will create a new one
	*/
	void addRef(void)
	{
		if (m_target) {
			if (!m_refCounter)
				m_refCounter = new int(0);
			(*m_refCounter)++;
		}
	}

	/*! \brief Removes a reference from a reference counter
		\note If the reference counter has less than one reference, it will delete itself and deallocate an element type
	*/
	void removeRef(void)
	{
		if (m_refCounter) {
			if (*m_refCounter > 1)
				(*m_refCounter)--;
			else {
				delete m_refCounter;
				DeallocationPolicy<ElementType>::Deallocate(m_target);
			}
		}
	}

	ElementType *m_target;
	int *m_refCounter;
	bool m_enabled;
};

/*! \brief A reference counter memory releasing janitor class
*/
template <class T, template <class> class DeallocationPolicy = JanitorRCDeallocationPolicy_Free>
class JanitorRCFree : public JanitorRCDeallocation<T, DeallocationPolicy>
{
public:

	/*! \brief Constructor
	*/
	JanitorRCFree(T *target = 0, bool enabled=true): JanitorRCDeallocation<T, DeallocationPolicy>(target, enabled) {}
};

/*! \brief A reference counter memory releasing janitor class
*/
template <class T, template <class> class DeallocationPolicy = JanitorRCDeallocationPolicy_Delete>
class JanitorRCDelete : public JanitorRCDeallocation<T, DeallocationPolicy>
{
public:

	/*! \brief Constructor
	*/
	JanitorRCDelete(T *target = 0, bool enabled=true): JanitorRCDeallocation<T, DeallocationPolicy>(target, enabled) {}
};

#endif

}

#endif

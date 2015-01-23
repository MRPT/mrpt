#ifndef __BINDINGS_H__
#define __BINDINGS_H__

/* BOOST */
#include <boost/python.hpp>

/* std */
#include <vector>
#include <deque>

/* smart_ptr */
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>

/* macros */
#define MAKE_PTR(class_name) class_<class_name##Ptr>("class_name##Ptr", "class_name smart pointer type", no_init)\
    .def("ctx", &class_name##Ptr_ctx, return_value_policy<reference_existing_object>())\
;\

#define MAKE_PTR_CTX(class_name) class_name& class_name##Ptr_ctx(class_name##Ptr& self)\
{\
    return *self;\
}\


namespace stlplus
{
    // here comes the magic
    template <typename T> T* get_pointer(smart_ptr<T> const& p)
    {
      //notice the const_cast<> at this point
      //for some unknown reason, bp likes to have it like that
      return const_cast<T*>(p.pointer());
    }
}

namespace boost{ namespace python{

    template <class T> struct pointee<stlplus::smart_ptr<T> >
    {
        typedef T type;
    };

} }


// STL list-like containers
void IndexError();

template<class T>
struct stl_list
{
    typedef typename T::value_type V;
    static V& get(T const& x, int i)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) return x[i];
        IndexError();
    }
    static void set(T const& x, int i, V const& v)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) x[i]=v;
        else IndexError();
    }
    static void del(T const& x, int i)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) x.erase(i);
        else IndexError();
    }
    static void add(T const& x, V const& v)
    {
        x.push_back(v);
    }
};

template<class T>
struct stl_deque
{
    typedef typename T::value_type V;
    static V& get(T & x, int i)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) return x[i];
        IndexError();
    }
    static void set(T & x, int i, V const& v)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) x[i]=v;
        else IndexError();
    }
    static void del(T & x, int i)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) x.erase(x.begin() + i);
        else IndexError();
    }
    static void add(T & x, V const& v)
    {
        x.push_back(v);
    }
};


/* exporters */
void export_gui();
void export_opengl();
void export_math();
void export_obs();
void export_maps();
void export_slam();
void export_nav();
void export_poses();
void export_system();
void export_utils();
void export_bayes();

#endif

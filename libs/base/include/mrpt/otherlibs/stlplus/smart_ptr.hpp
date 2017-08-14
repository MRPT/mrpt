/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*
    Modified version of STL+ sources shipped with the Mobile Robot
	Programming Toolkit (MRPT).

	Sources have been modified to support thred-safe smart pointers
	through atomic operations.

	2009, Jose Luis Blanco. University of Malaga.
*/


#ifndef STLPLUS_SMART_PTR
#define STLPLUS_SMART_PTR
////////////////////////////////////////////////////////////////////////////////

//   Author:    Andy Rushton
//   Copyright: (c) Andy Rushton, 2007
//   License:   BSD License, see ../docs/license.html

//   A smart pointer is a memory-managing pointer to an object. If you like, it
//   is a zero-dimensional container.

//   Assignment of smart pointers result in multiple aliases of the same object.
//   The term alias is used to differentiate from conventional pointers because
//   the semantics are different.

//   Aliases can be turned into copies if the pointed-to class supports copying.

//   The base class is smart_ptr_base which defines the common interface. Then
//   there are three subclasses which have the same interface but different copy
//   semantics:

//   - smart_ptr        for simple types and classes which have copy constructors
//   - smart_ptr_clone  for polymorphic class hierarchies which are copied using a clone method
//   - smart_ptr_nocopy for any class that cannot or should not be copied

////////////////////////////////////////////////////////////////////////////////
#include "exceptions.hpp"

#include <mrpt/synch/atomic_incr.h>   // JLB: For atomic_count. See smart_ptr.tpp

namespace stlplus
{

  ////////////////////////////////////////////////////////////////////////////////
  // internals

  template<typename T,typename COUNTER> class smart_ptr_holder;

  ////////////////////////////////////////////////////////////////////////////////
  // Base class
  ////////////////////////////////////////////////////////////////////////////////

  template<typename T, typename C, typename COUNTER> // Typically: COUNTER = mrpt::synch::CAtomicCounter>
  class smart_ptr_base
  {
  public:
    //////////////////////////////////////////////////////////////////////////////
    // member type definitions

    typedef T value_type;
    typedef T& reference;
    typedef const T& const_reference;
    typedef C value_copy;

    //////////////////////////////////////////////////////////////////////////////
    // constructors and destructors

    // create a null pointer
    smart_ptr_base(void);

    // create a pointer containing a *copy* of the object using the template parameter C
    // this copy is taken because the pointer class maintains a dynamically allocated object
    // and the T& may not be (usually is not) dynamically allocated
    explicit smart_ptr_base(const T& data);

    // create a pointer containing a dynamically created object
    // Note: the object must be allocated *by the user* with new
    // constructor form - must be called in the form smart_ptr_base<type> x(new type(args))
    explicit smart_ptr_base(T* data);

    // copy constructor implements aliasing so no copy is made
    explicit smart_ptr_base(const smart_ptr_base<T,C,COUNTER>& r);

    // destructor decrements the reference count and delete only when the last reference is destroyed
    ~smart_ptr_base(void);

    //////////////////////////////////////////////////////////////////////////////
    // logical tests to see if there is anything contained in the pointer since it can be null

    // there are two forms:explicit and implicit
    // implicit: if(!r) or if(r)
    // explicit: if(r.null()) or if(r.present())
    operator bool(void) const;
    bool operator!(void) const;
    bool present(void) const;
    bool null(void) const;

    //////////////////////////////////////////////////////////////////////////////
    // dereference operators and functions

    // dereference the smart pointer to get the object - use in the form *p1
    T& operator*(void);
    const T& operator*(void) const;

    // used as a prefix to a member access to the contained object e.g. p1->print() calls T::print()
    T* operator->(void);
    const T* operator->(void) const;

    //////////////////////////////////////////////////////////////////////////////
    // explicit function forms of the above assignment and dereference operators

    // set the value - note that this does a copy using the C template parameter
    void set_value(const T& data);
    // get the value
    T& value(void);
    const T& value(void) const;

    // set the pointer
    // deletes the previous pointer and adopts the passed pointer instead
    // Note: the object must be allocated *by the user* with new
    // Warning: it is very easy to break the memory management with this operation
    void set(T* data = 0);
    // get the pointer
    T* pointer(void);
    const T* pointer(void) const;

    //////////////////////////////////////////////////////////////////////////////
    // functions to manage aliases

    // make this an alias of the passed object
    inline void alias(const smart_ptr_base<T,C,COUNTER>&);

    // test whether two pointers point to the same object(known as aliasing the object)
    // used in the form if(a.aliases(b))
    inline bool aliases(const smart_ptr_base<T,C,COUNTER>&) const;

    // find the number of aliases - used when you need to know whether an
    // object is still referred to from elsewhere (rare!)
    inline unsigned alias_count(void) const;

    // delete the object and make the pointer null - does not make it unique
    // first, so all other pointers to this will be null too
    inline void clear(void);

    // make the pointer unique and null in one step - does not affect other
    // pointers that were pointing to the same object
    inline void clear_unique(void);

    //////////////////////////////////////////////////////////////////////////////
    // functions that involve copying

    // these functions use the copy functor passed as the template parameter C
    // to copy the object with the right copy semantics. If the copy functor
    // is no_copy, an exception will be thrown.

    // make this pointer unique with respect to any other references to the same object
    // if this pointer is already unique, it does nothing - otherwise it copies the object
    inline void make_unique(void);

    // make this pointer a unique copy of the parameter
    // useful for expressions like p1.copy(p2) which makes p1 a pointer to a unique copy of the contents of p2
    void copy(const smart_ptr_base<T,C,COUNTER>&);

  protected:
    smart_ptr_holder<T,COUNTER>* m_holder;

  public:
    // internal use only - had to make them public because they need to be
    // accessed by routines that could not be made friends
    void* handle(void) const;
    void make_alias(void* handle);
  };

  ////////////////////////////////////////////////////////////////////////////////
  // copy functors implementing the three possible copy semantics

  // constructor_copy uses the copy constructor of the object - used for simple types

  template <typename T>
  class constructor_copy
  {
  public:
    T* operator() (const T& from)
      {
        return new T(from);
      }
  };

  // clone_copy uses the clone method of the object - used for polymorphic types

  template <typename T>
  class clone_copy
  {
  public:
    T* operator() (const T& from)
      {
        return from.clone();
      }
  };

  // no_copy throws an exception - used for types that cannot be copied

  template <typename T>
  class no_copy
  {
  public:
    T* operator() (const T& )
      {
        throw illegal_copy("no_copy functor called");
        return 0;
      }
  };

  ////////////////////////////////////////////////////////////////////////////////
  // smart_ptr        for simple types and classes which have copy constructors

  template <typename T,typename COUNTER = mrpt::synch::CAtomicCounter>
  class smart_ptr : public smart_ptr_base<T, constructor_copy<T>, COUNTER  >
  {
  public:
    smart_ptr(void) {}
    explicit smart_ptr(const T& data) : smart_ptr_base<T, constructor_copy<T>,COUNTER >(data) {}
    explicit smart_ptr(T* data) : smart_ptr_base<T, constructor_copy<T>,COUNTER >(data) {}
    smart_ptr<T>& operator=(const T& data) {this->set_value(data); return *this;}
    smart_ptr<T>& operator=(const smart_ptr<T>& r) {this->alias(r); return *this;}
    ~smart_ptr(void) {}
  };

  ////////////////////////////////////////////////////////////////////////////////
  // smart_ptr_clone  for polymorphic class hierarchies which have a clone method

  template <typename T,typename COUNTER = mrpt::synch::CAtomicCounter>
  class smart_ptr_clone : public smart_ptr_base<T, clone_copy<T>, COUNTER >
  {
  public:
    smart_ptr_clone(void) {}
    explicit smart_ptr_clone(const T& data) : smart_ptr_base<T, clone_copy<T>, COUNTER >(data) {}
    explicit smart_ptr_clone(T* data) : smart_ptr_base<T, clone_copy<T>, COUNTER >(data) {}
    smart_ptr_clone<T>& operator=(const T& data) {this->set_value(data); return *this;}
    smart_ptr_clone<T>& operator=(const smart_ptr_clone<T>& r) {this->alias(r); return *this;}
    ~smart_ptr_clone(void) {}
  };

  ////////////////////////////////////////////////////////////////////////////////
  // smart_ptr_nocopy for any class that cannot or should not be copied

  template <typename T,typename COUNTER = mrpt::synch::CAtomicCounter>
  class smart_ptr_nocopy : public smart_ptr_base<T, no_copy<T>, COUNTER >
  {
  public:
    smart_ptr_nocopy(void) {}
    explicit smart_ptr_nocopy(const T& data) : smart_ptr_base<T, no_copy<T>, COUNTER >(data) {}
    explicit smart_ptr_nocopy(T* data) : smart_ptr_base<T, no_copy<T>, COUNTER >(data) {}
    smart_ptr_nocopy<T>& operator=(const T& data) {this->set_value(data); return *this;}
    smart_ptr_nocopy<T>& operator=(const smart_ptr_nocopy<T>& r) {this->alias(r); return *this;}
    ~smart_ptr_nocopy(void) {}
  };

  ////////////////////////////////////////////////////////////////////////////////

} // end namespace stlplus

#include "smart_ptr.tpp"
#endif

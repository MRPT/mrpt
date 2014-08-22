/*
    The STL+ C++ Library Collection

    Website <http://stlplus.sourceforge.net/> Collection <index.html>


      License Agreement

    <http://www.opensource.org/>

        * License for using the STLplus Library Collection <#license>
        * The Intent of this License <#intent>
        * How to Comply with this License <#compliance>
        * Historical Note <#history>


        License for using the STLplus Library Collection

    *© 1999-2008 Andy Rushton. All rights reserved.*

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above Copyright
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above Copyright
          notice, this list of conditions and the following disclaimer in
          the documentation and/or other materials provided with the
          distribution.
        * Neither the name of the STLplus library nor the names of its
          contributors may be used to endorse or promote products derived
          from this software without specific prior written permission.

    This software is provided by the Copyright holders and contributors "as
    is" and any express or implied warranties, including, but not limited
    to, the implied warranties of merchantability and fitness for a
    particular purpose are disclaimed. In no event shall the Copyright owner
    or contributors be liable for any direct, indirect, incidental, special,
    exemplary, or consequential damages (including, but not limited to,
    procurement of substitute goods or services; loss of use, data, or
    profits; or business interruption) however caused and on any theory of
    liability, whether in contract, strict liability, or tort (including
    negligence or otherwise) arising in any way out of the use of this
    software, even if advised of the possibility of such damage.
*/

/*
    Modified version of STL+ sources shipped with the Mobile Robot 
	Programming Toolkit (MRPT).
	
	Sources have been modified to support thred-safe smart pointers
	through atomic operations.
	
	2009, Jose Luis Blanco. University of Malaga. 
*/

#ifndef MRPT_SMARTPTR_H
#define MRPT_SMARTPTR_H

////////////////////////////////////////////////////////////////////////////////

//   Author:    Andy Rushton
//   Copyright: (c) Andy Rushton, 2007
//   License:   BSD License, see ../docs/license.html

////////////////////////////////////////////////////////////////////////////////

namespace stlplus
{

  ////////////////////////////////////////////////////////////////////////////////
  // internal holder data structure
  ////////////////////////////////////////////////////////////////////////////////

  template<typename T,typename COUNTER>
  class smart_ptr_holder
  {
  private:
	COUNTER	m_count;  //JL: It was...  unsigned m_count;
    T* m_data;

    // make these private to disallow copying because the holder doesn't know how to copy
    inline smart_ptr_holder(const smart_ptr_holder& ) :
      m_count(0), m_data(0)
      {
      }

    inline smart_ptr_holder& operator=(const smart_ptr_holder& )
      {
        return *this;
      }

  public:
    inline smart_ptr_holder(T* p = 0) : 
      m_count(1), m_data(p)
      {
      }

    ~smart_ptr_holder(void)
      {
        clear();
      }

    inline unsigned long count(void) const
      {
        return m_count;
      }

    inline void increment(void)
      {
        ++m_count;
      }

    inline bool decrement(void)
      {
        return (--m_count)==0;
      }

    inline bool null(void)
      {
        return m_data == 0;
      }

    inline void clear(void)
      {
        if(m_data)
          delete m_data;
        m_data = 0;
      }

    inline void set(T* p = 0)
      {
        clear();
        m_data = p;
      }

    inline T*& pointer(void)
      {
        return m_data;
      }

    inline const T* pointer(void) const
      {
        return m_data;
      }

    inline T& value(void)
      {
        return *m_data;
      }

    inline const T& value(void) const
      {
        return *m_data;
      }
  };

  ////////////////////////////////////////////////////////////////////////////////
  // smart_ptr_base class
  ////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////
  // constructors, assignments and destructors

  // create a null pointer
  template <typename T, typename C, typename COUNTER>
  smart_ptr_base<T,C,COUNTER>::smart_ptr_base(void) :
    m_holder(new smart_ptr_holder<T,COUNTER>)
  {
  }

  // create a pointer containing a *copy* of the object pointer
  template <typename T, typename C, typename COUNTER>
  smart_ptr_base<T,C,COUNTER>::smart_ptr_base(const T& data) throw(illegal_copy) :
    m_holder(new smart_ptr_holder<T,COUNTER>)
  {
    m_holder->set(C()(data));
  }

  // create a pointer containing a dynamically created object
  // Note: the object must be allocated *by the user* with new
  // constructor form - must be called in the form smart_ptr<type> x(new type(args))
  template <typename T, typename C, typename COUNTER>
  smart_ptr_base<T,C,COUNTER>::smart_ptr_base(T* data) :
    m_holder(new smart_ptr_holder<T,COUNTER>)
  {
    m_holder->set(data);
  }

  // copy constructor implements counted referencing - no copy is made
  template <typename T, typename C, typename COUNTER>
  smart_ptr_base<T,C,COUNTER>::smart_ptr_base(const smart_ptr_base<T,C,COUNTER>& r) :
    m_holder(0)
  {
    m_holder = r.m_holder;
    m_holder->increment();
  }

  // destructor decrements the reference count and delete only when the last reference is destroyed
  template <typename T, typename C, typename COUNTER>
  smart_ptr_base<T,C,COUNTER>::~smart_ptr_base(void)
  {
    if(m_holder->decrement())
      delete m_holder;
  }

  //////////////////////////////////////////////////////////////////////////////
  // logical tests to see if there is anything contained in the pointer since it can be null

  template <typename T, typename C, typename COUNTER>
  inline bool smart_ptr_base<T,C,COUNTER>::null(void) const
  {
    return m_holder->null();
  }

  template <typename T, typename C, typename COUNTER>
  inline bool smart_ptr_base<T,C,COUNTER>::present(void) const
  {
    return !m_holder->null();
  }

  template <typename T, typename C, typename COUNTER>
  bool smart_ptr_base<T,C,COUNTER>::operator!(void) const
  {
    return m_holder->null();
  }

  template <typename T, typename C, typename COUNTER>
  smart_ptr_base<T,C,COUNTER>::operator bool(void) const
  {
    return !m_holder->null();
  }

  //////////////////////////////////////////////////////////////////////////////
  // dereference operators and functions

  template <typename T, typename C, typename COUNTER>
  inline T& smart_ptr_base<T,C,COUNTER>::operator*(void) throw(null_dereference)
  {
    if (m_holder->null()) throw null_dereference("null pointer dereferenced in smart_ptr::operator*");
    return m_holder->value();
  }

  template <typename T, typename C, typename COUNTER>
  inline const T& smart_ptr_base<T,C,COUNTER>::operator*(void) const throw(null_dereference)
  {
    if (m_holder->null()) throw null_dereference("null pointer dereferenced in smart_ptr::operator*");
    return m_holder->value();
  }

  template <typename T, typename C, typename COUNTER>
  inline T* smart_ptr_base<T,C,COUNTER>::operator->(void) throw(null_dereference)
  {
    if (m_holder->null()) throw null_dereference("null pointer dereferenced in smart_ptr::operator->");
    return m_holder->pointer();
  }

  template <typename T, typename C, typename COUNTER>
  inline const T* smart_ptr_base<T,C,COUNTER>::operator->(void) const throw(null_dereference)
  {
    if (m_holder->null()) throw null_dereference("null pointer dereferenced in smart_ptr::operator->");
    return m_holder->pointer();
  }

  //////////////////////////////////////////////////////////////////////////////
  // explicit function forms of the above assignment dereference operators

  template <typename T, typename C, typename COUNTER>
  inline void smart_ptr_base<T,C,COUNTER>::set_value(const T& data) throw(illegal_copy)
  {
    m_holder->set(C()(data));
  }

  template <typename T, typename C, typename COUNTER>
  inline T& smart_ptr_base<T,C,COUNTER>::value(void) throw(null_dereference)
  {
    if (m_holder->null()) throw null_dereference("null pointer dereferenced in smart_ptr::value");
    return m_holder->value();
  }

  template <typename T, typename C, typename COUNTER>
  inline const T& smart_ptr_base<T,C,COUNTER>::value(void) const throw(null_dereference)
  {
    if (m_holder->null()) throw null_dereference("null pointer dereferenced in smart_ptr::value");
    return m_holder->value();
  }

  template <typename T, typename C, typename COUNTER>
  void smart_ptr_base<T,C,COUNTER>::set(T* data)
  {
    m_holder->set(data);
  }

  template <typename T, typename C, typename COUNTER>
  inline T* smart_ptr_base<T,C,COUNTER>::pointer(void)
  {
    return m_holder->pointer();
  }

  template <typename T, typename C, typename COUNTER>
  inline const T* smart_ptr_base<T,C,COUNTER>::pointer(void) const
  {
    return m_holder->pointer();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // functions to manage counted referencing

  // make this an alias of the passed object
  template <typename T, typename C, typename COUNTER>
  void smart_ptr_base<T,C,COUNTER>::alias(const smart_ptr_base<T,C,COUNTER>& r)
  {
    // make it alias-copy safe - this means that I don't try to do the
    // assignment if r is either the same object or an alias of it
    //   if (m_holder == r.m_holder) return;
    //   if (m_holder->decrement())
    //     delete m_holder;
    //   m_holder = r.m_holder;
    //   m_holder->increment();
    make_alias(r.m_holder);
  }

  template <typename T, typename C, typename COUNTER>
  bool smart_ptr_base<T,C,COUNTER>::aliases(const smart_ptr_base<T,C,COUNTER>& r) const
  {
    return m_holder == r.m_holder;
  }

  template <typename T, typename C, typename COUNTER>
  unsigned smart_ptr_base<T,C,COUNTER>::alias_count(void) const
  {
    return m_holder->count();
  }

  template <typename T, typename C, typename COUNTER>
  void smart_ptr_base<T,C,COUNTER>::clear(void)
  {
    m_holder->clear();
  }

  template <typename T, typename C, typename COUNTER>
  void smart_ptr_base<T,C,COUNTER>::clear_unique(void)
  {
    if (m_holder->count() == 1)
      m_holder->clear();
    else
    {
      m_holder->decrement();
      m_holder = 0;
      m_holder = new smart_ptr_holder<T,COUNTER>;
    }
  }

  template <typename T, typename C, typename COUNTER>
  void smart_ptr_base<T,C,COUNTER>::make_unique(void) throw(illegal_copy)
  {
    if (m_holder->count() > 1)
    {
      smart_ptr_holder<T,COUNTER>* old_holder = m_holder;
      m_holder->decrement();
      m_holder = 0;
      m_holder = new smart_ptr_holder<T,COUNTER>;
      if (old_holder->pointer())
        m_holder->set(C()(old_holder->value()));
    }
  }

  template <typename T, typename C, typename COUNTER>
  void smart_ptr_base<T,C,COUNTER>::copy(const smart_ptr_base<T,C,COUNTER>& data) throw(illegal_copy)
  {
    alias(data);
    make_unique();
  }

  // internal function for distinguishing unique smart_ptr objects
  // used for example in persistence routines

  template <typename T, typename C, typename COUNTER>
  void* smart_ptr_base<T,C,COUNTER>::handle(void) const
  {
    return m_holder;
  }

  template <typename T, typename C, typename COUNTER>
  void smart_ptr_base<T,C,COUNTER>::make_alias(void* handle)
  {
    smart_ptr_holder<T,COUNTER>* r_holder = (smart_ptr_holder<T,COUNTER>*)handle;
    if (m_holder != r_holder)
    {
      if (m_holder->decrement())
        delete m_holder;
      m_holder = r_holder;
      m_holder->increment();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////

} // end namespace stlplus

#endif


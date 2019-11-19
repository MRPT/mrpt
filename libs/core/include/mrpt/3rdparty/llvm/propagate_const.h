//===------------------------ propagate_const -----------------------------===//
////
////                     Originally:
////           The LLVM Compiler Infrastructure
////           Slightly modified for use in MRPT.
////        Note: propagate_const may be standardized.
////     Delete this when std::propagate_const becomes available.
////
//// This file is dual licensed under the MIT and the University of Illinois Open
//// Source Licenses. See LICENSE.TXT for details.
////
////===----------------------------------------------------------------------===//


#pragma once

#include <type_traits>
#include <utility>
#include <functional>

namespace mrpt
{

template <class _Tp>
class propagate_const;

template <class _Up>
inline constexpr
const _Up& get_underlying(const propagate_const<_Up>& __pu) noexcept;

template <class _Up>
inline constexpr
_Up& get_underlying(propagate_const<_Up>& __pu) noexcept;

template <class _Tp>
class propagate_const
{
public:
  typedef std::remove_reference_t<decltype(*std::declval<_Tp&>())> element_type;

  static_assert(!std::is_array<_Tp>::value,
      "Instantiation of propagate_const with an array type is ill-formed.");
  static_assert(!std::is_reference<_Tp>::value,
      "Instantiation of propagate_const with a reference type is ill-formed.");
  static_assert(!(std::is_pointer<_Tp>::value && std::is_function<typename std::remove_pointer<_Tp>::type>::value),
      "Instantiation of propagate_const with a function-pointer type is ill-formed.");
  static_assert(!(std::is_pointer<_Tp>::value && std::is_same<typename std::remove_cv<typename std::remove_pointer<_Tp>::type>::type, void>::value),
      "Instantiation of propagate_const with a pointer to (possibly cv-qualified) void is ill-formed.");

private:
  template <class _Up>
  static constexpr element_type* __get_pointer(_Up* __u)
  {
    return __u;
  }

  template <class _Up>
  static constexpr element_type* __get_pointer(_Up& __u)
  {
    return __get_pointer(__u.get());
  }

  template <class _Up>
  static constexpr const element_type* __get_pointer(const _Up* __u)
  {
    return __u;
  }

  template <class _Up>
  static constexpr const element_type* __get_pointer(const _Up& __u)
  {
    return __get_pointer(__u.get());
  }

  template <class _Up>
  struct __is_propagate_const : std::false_type
  {
  };

  template <class _Up>
  struct __is_propagate_const<propagate_const<_Up>> : std::true_type
  {
  };

  _Tp __t_;

public:

  template <class _Up> friend constexpr const _Up& mrpt::get_underlying(const propagate_const<_Up>& __pu) noexcept;
  template <class _Up> friend constexpr _Up& mrpt::get_underlying(propagate_const<_Up>& __pu) noexcept;

  constexpr propagate_const() = default;

  propagate_const(const propagate_const&) = delete;

  constexpr propagate_const(propagate_const&&) = default;

  template <class _Up, std::enable_if_t<!std::is_convertible<_Up, _Tp>::value &&
                                 std::is_constructible<_Tp, _Up&&>::value,bool> = true>
  explicit constexpr propagate_const(propagate_const<_Up>&& __pu)
      : __t_(std::move(mrpt::get_underlying(__pu)))
  {
  }

  template <class _Up, std::enable_if_t<std::is_convertible<_Up&&, _Tp>::value &&
                                 std::is_constructible<_Tp, _Up&&>::value,bool> = false>
  constexpr propagate_const(propagate_const<_Up>&& __pu)
      : __t_(std::move(mrpt::get_underlying(__pu)))
  {
  }

  template <class _Up, std::enable_if_t<!std::is_convertible<_Up&&, _Tp>::value &&
                                 std::is_constructible<_Tp, _Up&&>::value &&
                                 !__is_propagate_const<std::decay_t<_Up>>::value,bool> = true>
  explicit constexpr propagate_const(_Up&& __u)
      : __t_(std::forward<_Up>(__u))
  {
  }

  template <class _Up, std::enable_if_t<std::is_convertible<_Up&&, _Tp>::value &&
                                 std::is_constructible<_Tp, _Up&&>::value &&
                                 !__is_propagate_const<std::decay_t<_Up>>::value,bool> = false>
  constexpr propagate_const(_Up&& __u)
      : __t_(std::forward<_Up>(__u))
  {
  }

  propagate_const& operator=(const propagate_const&) = delete;

  constexpr propagate_const& operator=(propagate_const&&) = default;

  template <class _Up>
  constexpr propagate_const& operator=(propagate_const<_Up>&& __pu)
  {
    __t_ = std::move(mrpt::get_underlying(__pu));
    return *this;
  }

  template <class _Up, class _Vp = std::enable_if_t<!__is_propagate_const<std::decay_t<_Up>>::value>>
  constexpr propagate_const& operator=(_Up&& __u)
  {
    __t_ = std::forward<_Up>(__u);
    return *this;
  }

  constexpr const element_type* get() const
  {
    return __get_pointer(__t_);
  }

  constexpr element_type* get()
  {
    return __get_pointer(__t_);
  }

  explicit constexpr operator bool() const
  {
    return get() != nullptr;
  }

  constexpr const element_type* operator->() const
  {
    return get();
  }

  template <class _Tp_ = _Tp, class _Up = std::enable_if_t<std::is_convertible<
                                  const _Tp_, const element_type *>::value>>
  constexpr operator const element_type *() const {
    return get();
  }

  constexpr const element_type& operator*() const
  {
    return *get();
  }

  constexpr element_type* operator->()
  {
    return get();
  }

  template <class _Tp_ = _Tp, class _Up = std::enable_if_t<
                                  std::is_convertible<_Tp_, element_type *>::value>>
  constexpr operator element_type *() {
    return get();
  }

  constexpr element_type& operator*()
  {
    return *get();
  }

  constexpr void swap(propagate_const& __pt) noexcept(std::__is_nothrow_swappable<_Tp>::value)
  {
    using std::swap;
    swap(__t_, __pt.__t_);
  }
};


template <class _Tp>

constexpr bool operator==(const propagate_const<_Tp>& __pt, std::nullptr_t)
{
  return mrpt::get_underlying(__pt) == nullptr;
}

template <class _Tp>

constexpr bool operator==(std::nullptr_t, const propagate_const<_Tp>& __pt)
{
  return nullptr == mrpt::get_underlying(__pt);
}

template <class _Tp>

constexpr bool operator!=(const propagate_const<_Tp>& __pt, std::nullptr_t)
{
  return mrpt::get_underlying(__pt) != nullptr;
}

template <class _Tp>

constexpr bool operator!=(std::nullptr_t, const propagate_const<_Tp>& __pt)
{
  return nullptr != mrpt::get_underlying(__pt);
}

template <class _Tp, class _Up>
constexpr bool operator==(const propagate_const<_Tp>& __pt,
                          const propagate_const<_Up>& __pu)
{
  return mrpt::get_underlying(__pt) == mrpt::get_underlying(__pu);
}

template <class _Tp, class _Up>
constexpr bool operator!=(const propagate_const<_Tp>& __pt,
                          const propagate_const<_Up>& __pu)
{
  return mrpt::get_underlying(__pt) != mrpt::get_underlying(__pu);
}

template <class _Tp, class _Up>
constexpr bool operator<(const propagate_const<_Tp>& __pt,
                         const propagate_const<_Up>& __pu)
{
  return mrpt::get_underlying(__pt) < mrpt::get_underlying(__pu);
}

template <class _Tp, class _Up>
constexpr bool operator>(const propagate_const<_Tp>& __pt,
                         const propagate_const<_Up>& __pu)
{
  return mrpt::get_underlying(__pt) > mrpt::get_underlying(__pu);
}

template <class _Tp, class _Up>
constexpr bool operator<=(const propagate_const<_Tp>& __pt,
                          const propagate_const<_Up>& __pu)
{
  return mrpt::get_underlying(__pt) <= mrpt::get_underlying(__pu);
}

template <class _Tp, class _Up>
constexpr bool operator>=(const propagate_const<_Tp>& __pt,
                          const propagate_const<_Up>& __pu)
{
  return mrpt::get_underlying(__pt) >= mrpt::get_underlying(__pu);
}

template <class _Tp, class _Up>
constexpr bool operator==(const propagate_const<_Tp>& __pt, const _Up& __u)
{
  return mrpt::get_underlying(__pt) == __u;
}

template <class _Tp, class _Up>
constexpr bool operator!=(const propagate_const<_Tp>& __pt, const _Up& __u)
{
  return mrpt::get_underlying(__pt) != __u;
}

template <class _Tp, class _Up>
constexpr bool operator<(const propagate_const<_Tp>& __pt, const _Up& __u)
{
  return mrpt::get_underlying(__pt) < __u;
}

template <class _Tp, class _Up>
constexpr bool operator>(const propagate_const<_Tp>& __pt, const _Up& __u)
{
  return mrpt::get_underlying(__pt) > __u;
}

template <class _Tp, class _Up>
constexpr bool operator<=(const propagate_const<_Tp>& __pt, const _Up& __u)
{
  return mrpt::get_underlying(__pt) <= __u;
}

template <class _Tp, class _Up>
constexpr bool operator>=(const propagate_const<_Tp>& __pt, const _Up& __u)
{
  return mrpt::get_underlying(__pt) >= __u;
}


template <class _Tp, class _Up>
constexpr bool operator==(const _Tp& __t, const propagate_const<_Up>& __pu)
{
  return __t == mrpt::get_underlying(__pu);
}

template <class _Tp, class _Up>
constexpr bool operator!=(const _Tp& __t, const propagate_const<_Up>& __pu)
{
  return __t != mrpt::get_underlying(__pu);
}

template <class _Tp, class _Up>
constexpr bool operator<(const _Tp& __t, const propagate_const<_Up>& __pu)
{
  return __t < mrpt::get_underlying(__pu);
}

template <class _Tp, class _Up>
constexpr bool operator>(const _Tp& __t, const propagate_const<_Up>& __pu)
{
  return __t > mrpt::get_underlying(__pu);
}

template <class _Tp, class _Up>
constexpr bool operator<=(const _Tp& __t, const propagate_const<_Up>& __pu)
{
  return __t <= mrpt::get_underlying(__pu);
}

template <class _Tp, class _Up>
constexpr bool operator>=(const _Tp& __t, const propagate_const<_Up>& __pu)
{
  return __t >= mrpt::get_underlying(__pu);
}

template <class _Tp>
constexpr void swap(propagate_const<_Tp>& __pc1, propagate_const<_Tp>& __pc2) noexcept(std::__is_nothrow_swappable<_Tp>::value)
{
  __pc1.swap(__pc2);
}

template <class _Tp>
constexpr const _Tp& get_underlying(const propagate_const<_Tp>& __pt) noexcept
{
  return __pt.__t_;
}

template <class _Tp>
constexpr _Tp& get_underlying(propagate_const<_Tp>& __pt) noexcept
{
  return __pt.__t_;
}

}

namespace std
{

template <class _Tp>
struct hash<mrpt::propagate_const<_Tp>>
{
  typedef size_t result_type;
  typedef mrpt::propagate_const<_Tp> argument_type;

  size_t operator()(const mrpt::propagate_const<_Tp>& __pc1) const
  {
    return std::hash<_Tp>()(mrpt::get_underlying(__pc1));
  }
};

template <class _Tp>
struct equal_to<mrpt::propagate_const<_Tp>>
{
  typedef mrpt::propagate_const<_Tp> first_argument_type;
  typedef mrpt::propagate_const<_Tp> second_argument_type;

  bool operator()(const mrpt::propagate_const<_Tp>& __pc1,
      const mrpt::propagate_const<_Tp>& __pc2) const
  {
    return std::equal_to<_Tp>()(mrpt::get_underlying(__pc1), mrpt::get_underlying(__pc2));
  }
};

template <class _Tp>
struct not_equal_to<mrpt::propagate_const<_Tp>>
{
  typedef mrpt::propagate_const<_Tp> first_argument_type;
  typedef mrpt::propagate_const<_Tp> second_argument_type;

  bool operator()(const mrpt::propagate_const<_Tp>& __pc1,
      const mrpt::propagate_const<_Tp>& __pc2) const
  {
    return std::not_equal_to<_Tp>()(mrpt::get_underlying(__pc1), mrpt::get_underlying(__pc2));
  }
};

template <class _Tp>
struct less<mrpt::propagate_const<_Tp>>
{
  typedef mrpt::propagate_const<_Tp> first_argument_type;
  typedef mrpt::propagate_const<_Tp> second_argument_type;

  bool operator()(const mrpt::propagate_const<_Tp>& __pc1,
      const mrpt::propagate_const<_Tp>& __pc2) const
  {
    return std::less<_Tp>()(mrpt::get_underlying(__pc1), mrpt::get_underlying(__pc2));
  }
};

template <class _Tp>
struct greater<mrpt::propagate_const<_Tp>>
{
  typedef mrpt::propagate_const<_Tp> first_argument_type;
  typedef mrpt::propagate_const<_Tp> second_argument_type;

  bool operator()(const mrpt::propagate_const<_Tp>& __pc1,
      const mrpt::propagate_const<_Tp>& __pc2) const
  {
    return std::greater<_Tp>()(mrpt::get_underlying(__pc1), mrpt::get_underlying(__pc2));
  }
};

template <class _Tp>
struct less_equal<mrpt::propagate_const<_Tp>>
{
  typedef mrpt::propagate_const<_Tp> first_argument_type;
  typedef mrpt::propagate_const<_Tp> second_argument_type;

  bool operator()(const mrpt::propagate_const<_Tp>& __pc1,
      const mrpt::propagate_const<_Tp>& __pc2) const
  {
    return std::less_equal<_Tp>()(mrpt::get_underlying(__pc1), mrpt::get_underlying(__pc2));
  }
};

template <class _Tp>
struct greater_equal<mrpt::propagate_const<_Tp>>
{
  typedef mrpt::propagate_const<_Tp> first_argument_type;
  typedef mrpt::propagate_const<_Tp> second_argument_type;

  bool operator()(const mrpt::propagate_const<_Tp>& __pc1,
      const mrpt::propagate_const<_Tp>& __pc2) const
  {
    return std::greater_equal<_Tp>()(mrpt::get_underlying(__pc1), mrpt::get_underlying(__pc2));
  }
};

}



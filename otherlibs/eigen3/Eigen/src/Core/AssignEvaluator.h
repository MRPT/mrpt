// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2011 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2011 Jitse Niesen <jitse@maths.leeds.ac.uk>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

#ifndef EIGEN_ASSIGN_EVALUATOR_H
#define EIGEN_ASSIGN_EVALUATOR_H

// This implementation is based on Assign.h

namespace internal {
  
/***************************************************************************
* Part 1 : the logic deciding a strategy for traversal and unrolling       *
***************************************************************************/

// copy_using_evaluator_traits is based on assign_traits
// (actually, it's identical)

template <typename Derived, typename OtherDerived>
struct copy_using_evaluator_traits
{
public:
  enum {
    DstIsAligned = Derived::Flags & AlignedBit,
    DstHasDirectAccess = Derived::Flags & DirectAccessBit,
    SrcIsAligned = OtherDerived::Flags & AlignedBit,
    JointAlignment = bool(DstIsAligned) && bool(SrcIsAligned) ? Aligned : Unaligned
  };

private:
  enum {
    InnerSize = int(Derived::IsVectorAtCompileTime) ? int(Derived::SizeAtCompileTime)
              : int(Derived::Flags)&RowMajorBit ? int(Derived::ColsAtCompileTime)
              : int(Derived::RowsAtCompileTime),
    InnerMaxSize = int(Derived::IsVectorAtCompileTime) ? int(Derived::MaxSizeAtCompileTime)
              : int(Derived::Flags)&RowMajorBit ? int(Derived::MaxColsAtCompileTime)
              : int(Derived::MaxRowsAtCompileTime),
    MaxSizeAtCompileTime = Derived::SizeAtCompileTime,
    PacketSize = packet_traits<typename Derived::Scalar>::size
  };

  enum {
    StorageOrdersAgree = (int(Derived::IsRowMajor) == int(OtherDerived::IsRowMajor)),
    MightVectorize = StorageOrdersAgree
                  && (int(Derived::Flags) & int(OtherDerived::Flags) & ActualPacketAccessBit),
    MayInnerVectorize  = MightVectorize && int(InnerSize)!=Dynamic && int(InnerSize)%int(PacketSize)==0
                       && int(DstIsAligned) && int(SrcIsAligned),
    MayLinearize = StorageOrdersAgree && (int(Derived::Flags) & int(OtherDerived::Flags) & LinearAccessBit),
    MayLinearVectorize = MightVectorize && MayLinearize && DstHasDirectAccess
                       && (DstIsAligned || MaxSizeAtCompileTime == Dynamic),
      /* If the destination isn't aligned, we have to do runtime checks and we don't unroll,
         so it's only good for large enough sizes. */
    MaySliceVectorize  = MightVectorize && DstHasDirectAccess
                       && (int(InnerMaxSize)==Dynamic || int(InnerMaxSize)>=3*PacketSize)
      /* slice vectorization can be slow, so we only want it if the slices are big, which is
         indicated by InnerMaxSize rather than InnerSize, think of the case of a dynamic block
         in a fixed-size matrix */
  };

public:
  enum {
    Traversal = int(MayInnerVectorize)  ? int(InnerVectorizedTraversal)
              : int(MayLinearVectorize) ? int(LinearVectorizedTraversal)
              : int(MaySliceVectorize)  ? int(SliceVectorizedTraversal)
              : int(MayLinearize)       ? int(LinearTraversal)
                                        : int(DefaultTraversal),
    Vectorized = int(Traversal) == InnerVectorizedTraversal
              || int(Traversal) == LinearVectorizedTraversal
              || int(Traversal) == SliceVectorizedTraversal
  };

private:
  enum {
    UnrollingLimit      = EIGEN_UNROLLING_LIMIT * (Vectorized ? int(PacketSize) : 1),
    MayUnrollCompletely = int(Derived::SizeAtCompileTime) != Dynamic
                       && int(OtherDerived::CoeffReadCost) != Dynamic
                       && int(Derived::SizeAtCompileTime) * int(OtherDerived::CoeffReadCost) <= int(UnrollingLimit),
    MayUnrollInner      = int(InnerSize) != Dynamic
                       && int(OtherDerived::CoeffReadCost) != Dynamic
                       && int(InnerSize) * int(OtherDerived::CoeffReadCost) <= int(UnrollingLimit)
  };

public:
  enum {
    Unrolling = (int(Traversal) == int(InnerVectorizedTraversal) || int(Traversal) == int(DefaultTraversal))
                ? (
 		    int(MayUnrollCompletely) ? int(CompleteUnrolling)
                  : int(MayUnrollInner)      ? int(InnerUnrolling)
                                             : int(NoUnrolling)
                  )
              : int(Traversal) == int(LinearVectorizedTraversal)
                ? ( bool(MayUnrollCompletely) && bool(DstIsAligned) ? int(CompleteUnrolling) 
                                                                    : int(NoUnrolling) )
              : int(Traversal) == int(LinearTraversal)
                ? ( bool(MayUnrollCompletely) ? int(CompleteUnrolling) 
                                              : int(NoUnrolling) )
              : int(NoUnrolling)
  };

#ifdef EIGEN_DEBUG_ASSIGN
  static void debug()
  {
    EIGEN_DEBUG_VAR(DstIsAligned)
    EIGEN_DEBUG_VAR(SrcIsAligned)
    EIGEN_DEBUG_VAR(JointAlignment)
    EIGEN_DEBUG_VAR(InnerSize)
    EIGEN_DEBUG_VAR(InnerMaxSize)
    EIGEN_DEBUG_VAR(PacketSize)
    EIGEN_DEBUG_VAR(StorageOrdersAgree)
    EIGEN_DEBUG_VAR(MightVectorize)
    EIGEN_DEBUG_VAR(MayLinearize)
    EIGEN_DEBUG_VAR(MayInnerVectorize)
    EIGEN_DEBUG_VAR(MayLinearVectorize)
    EIGEN_DEBUG_VAR(MaySliceVectorize)
    EIGEN_DEBUG_VAR(Traversal)
    EIGEN_DEBUG_VAR(UnrollingLimit)
    EIGEN_DEBUG_VAR(MayUnrollCompletely)
    EIGEN_DEBUG_VAR(MayUnrollInner)
    EIGEN_DEBUG_VAR(Unrolling)
  }
#endif
};

/***************************************************************************
* Part 2 : meta-unrollers
***************************************************************************/

/************************
*** Default traversal ***
************************/

template<typename DstEvaluatorType, typename SrcEvaluatorType, int Index, int Stop>
struct copy_using_evaluator_DefaultTraversal_CompleteUnrolling
{
  typedef typename DstEvaluatorType::XprType DstXprType;
  
  enum {
    outer = Index / DstXprType::InnerSizeAtCompileTime,
    inner = Index % DstXprType::InnerSizeAtCompileTime
  };

  EIGEN_STRONG_INLINE static void run(DstEvaluatorType &dstEvaluator, 
				      SrcEvaluatorType &srcEvaluator)
  {
    dstEvaluator.copyCoeffByOuterInner(outer, inner, srcEvaluator);
    copy_using_evaluator_DefaultTraversal_CompleteUnrolling
      <DstEvaluatorType, SrcEvaluatorType, Index+1, Stop>
      ::run(dstEvaluator, srcEvaluator);
  }
};

template<typename DstEvaluatorType, typename SrcEvaluatorType, int Stop>
struct copy_using_evaluator_DefaultTraversal_CompleteUnrolling<DstEvaluatorType, SrcEvaluatorType, Stop, Stop>
{
  EIGEN_STRONG_INLINE static void run(DstEvaluatorType&, SrcEvaluatorType&) { }
};

template<typename DstEvaluatorType, typename SrcEvaluatorType, int Index, int Stop>
struct copy_using_evaluator_DefaultTraversal_InnerUnrolling
{
  EIGEN_STRONG_INLINE static void run(DstEvaluatorType &dstEvaluator, 
				      SrcEvaluatorType &srcEvaluator, 
				      int outer)
  {
    dstEvaluator.copyCoeffByOuterInner(outer, Index, srcEvaluator);
    copy_using_evaluator_DefaultTraversal_InnerUnrolling
      <DstEvaluatorType, SrcEvaluatorType, Index+1, Stop>
      ::run(dstEvaluator, srcEvaluator, outer);
  }
};

template<typename DstEvaluatorType, typename SrcEvaluatorType, int Stop>
struct copy_using_evaluator_DefaultTraversal_InnerUnrolling<DstEvaluatorType, SrcEvaluatorType, Stop, Stop>
{
  EIGEN_STRONG_INLINE static void run(DstEvaluatorType&, SrcEvaluatorType&, int) { }
};

/***********************
*** Linear traversal ***
***********************/

template<typename DstEvaluatorType, typename SrcEvaluatorType, int Index, int Stop>
struct copy_using_evaluator_LinearTraversal_CompleteUnrolling
{
  EIGEN_STRONG_INLINE static void run(DstEvaluatorType &dstEvaluator, 
				      SrcEvaluatorType &srcEvaluator)
  {
    dstEvaluator.copyCoeff(Index, srcEvaluator);
    copy_using_evaluator_LinearTraversal_CompleteUnrolling
      <DstEvaluatorType, SrcEvaluatorType, Index+1, Stop>
      ::run(dstEvaluator, srcEvaluator);
  }
};

template<typename DstEvaluatorType, typename SrcEvaluatorType, int Stop>
struct copy_using_evaluator_LinearTraversal_CompleteUnrolling<DstEvaluatorType, SrcEvaluatorType, Stop, Stop>
{
  EIGEN_STRONG_INLINE static void run(DstEvaluatorType&, SrcEvaluatorType&) { }
};

/**************************
*** Inner vectorization ***
**************************/

template<typename DstEvaluatorType, typename SrcEvaluatorType, int Index, int Stop>
struct copy_using_evaluator_innervec_CompleteUnrolling
{
  typedef typename DstEvaluatorType::XprType DstXprType;
  typedef typename SrcEvaluatorType::XprType SrcXprType;

  enum {
    outer = Index / DstXprType::InnerSizeAtCompileTime,
    inner = Index % DstXprType::InnerSizeAtCompileTime,
    JointAlignment = copy_using_evaluator_traits<DstXprType,SrcXprType>::JointAlignment
  };

  EIGEN_STRONG_INLINE static void run(DstEvaluatorType &dstEvaluator, 
				      SrcEvaluatorType &srcEvaluator)
  {
    dstEvaluator.template copyPacketByOuterInner<Aligned, JointAlignment>(outer, inner, srcEvaluator);
    enum { NextIndex = Index + packet_traits<typename DstXprType::Scalar>::size };
    copy_using_evaluator_innervec_CompleteUnrolling
      <DstEvaluatorType, SrcEvaluatorType, NextIndex, Stop>
      ::run(dstEvaluator, srcEvaluator);
  }
};

template<typename DstEvaluatorType, typename SrcEvaluatorType, int Stop>
struct copy_using_evaluator_innervec_CompleteUnrolling<DstEvaluatorType, SrcEvaluatorType, Stop, Stop>
{
  EIGEN_STRONG_INLINE static void run(DstEvaluatorType&, SrcEvaluatorType&) { }
};

template<typename DstEvaluatorType, typename SrcEvaluatorType, int Index, int Stop>
struct copy_using_evaluator_innervec_InnerUnrolling
{
  EIGEN_STRONG_INLINE static void run(DstEvaluatorType &dstEvaluator, 
				      SrcEvaluatorType &srcEvaluator, 
				      int outer)
  {
    dstEvaluator.template copyPacketByOuterInner<Aligned, Aligned>(outer, Index, srcEvaluator);
    typedef typename DstEvaluatorType::XprType DstXprType;
    enum { NextIndex = Index + packet_traits<typename DstXprType::Scalar>::size };
    copy_using_evaluator_innervec_InnerUnrolling
      <DstEvaluatorType, SrcEvaluatorType, NextIndex, Stop>
      ::run(dstEvaluator, srcEvaluator, outer);
  }
};

template<typename DstEvaluatorType, typename SrcEvaluatorType, int Stop>
struct copy_using_evaluator_innervec_InnerUnrolling<DstEvaluatorType, SrcEvaluatorType, Stop, Stop>
{
  EIGEN_STRONG_INLINE static void run(DstEvaluatorType&, SrcEvaluatorType&, int) { }
};

/***************************************************************************
* Part 3 : implementation of all cases
***************************************************************************/

// copy_using_evaluator_impl is based on assign_impl

template<typename DstXprType, typename SrcXprType,
         int Traversal = copy_using_evaluator_traits<DstXprType, SrcXprType>::Traversal,
         int Unrolling = copy_using_evaluator_traits<DstXprType, SrcXprType>::Unrolling>
struct copy_using_evaluator_impl;

/************************
*** Default traversal ***
************************/

template<typename DstXprType, typename SrcXprType>
struct copy_using_evaluator_impl<DstXprType, SrcXprType, DefaultTraversal, NoUnrolling>
{
  static void run(DstXprType& dst, const SrcXprType& src)
  {
    typedef typename evaluator<DstXprType>::type DstEvaluatorType;
    typedef typename evaluator<SrcXprType>::type SrcEvaluatorType;
    typedef typename DstXprType::Index Index;

    DstEvaluatorType dstEvaluator(dst);
    SrcEvaluatorType srcEvaluator(src);

    for(Index outer = 0; outer < dst.outerSize(); ++outer) {
      for(Index inner = 0; inner < dst.innerSize(); ++inner) {
	dstEvaluator.copyCoeffByOuterInner(outer, inner, srcEvaluator);
      }
    }
  }
};

template<typename DstXprType, typename SrcXprType>
struct copy_using_evaluator_impl<DstXprType, SrcXprType, DefaultTraversal, CompleteUnrolling>
{
  EIGEN_STRONG_INLINE static void run(DstXprType &dst, const SrcXprType &src)
  {
    typedef typename evaluator<DstXprType>::type DstEvaluatorType;
    typedef typename evaluator<SrcXprType>::type SrcEvaluatorType;

    DstEvaluatorType dstEvaluator(dst);
    SrcEvaluatorType srcEvaluator(src);

    copy_using_evaluator_DefaultTraversal_CompleteUnrolling
      <DstEvaluatorType, SrcEvaluatorType, 0, DstXprType::SizeAtCompileTime>
      ::run(dstEvaluator, srcEvaluator);
  }
};

template<typename DstXprType, typename SrcXprType>
struct copy_using_evaluator_impl<DstXprType, SrcXprType, DefaultTraversal, InnerUnrolling>
{
  typedef typename DstXprType::Index Index;
  EIGEN_STRONG_INLINE static void run(DstXprType &dst, const SrcXprType &src)
  {
    typedef typename evaluator<DstXprType>::type DstEvaluatorType;
    typedef typename evaluator<SrcXprType>::type SrcEvaluatorType;

    DstEvaluatorType dstEvaluator(dst);
    SrcEvaluatorType srcEvaluator(src);

    const Index outerSize = dst.outerSize();
    for(Index outer = 0; outer < outerSize; ++outer)
      copy_using_evaluator_DefaultTraversal_InnerUnrolling
	<DstEvaluatorType, SrcEvaluatorType, 0, DstXprType::InnerSizeAtCompileTime>
        ::run(dstEvaluator, srcEvaluator, outer);
  }
};

/***************************
*** Linear vectorization ***
***************************/

template <bool IsAligned = false>
struct unaligned_copy_using_evaluator_impl
{
  // if IsAligned = true, then do nothing
  template <typename SrcEvaluatorType, typename DstEvaluatorType>
  static EIGEN_STRONG_INLINE void run(const SrcEvaluatorType&, DstEvaluatorType&, 
				      typename SrcEvaluatorType::Index, typename SrcEvaluatorType::Index) {}
};

template <>
struct unaligned_copy_using_evaluator_impl<false>
{
  // MSVC must not inline this functions. If it does, it fails to optimize the
  // packet access path.
#ifdef _MSC_VER
  template <typename DstEvaluatorType, typename SrcEvaluatorType>
  static EIGEN_DONT_INLINE void run(DstEvaluatorType &dstEvaluator, 
				    const SrcEvaluatorType &srcEvaluator, 
				    typename DstEvaluatorType::Index start, 
				    typename DstEvaluatorType::Index end)
#else
  template <typename DstEvaluatorType, typename SrcEvaluatorType>
  static EIGEN_STRONG_INLINE void run(DstEvaluatorType &dstEvaluator, 
				      const SrcEvaluatorType &srcEvaluator, 
				      typename DstEvaluatorType::Index start, 
				      typename DstEvaluatorType::Index end)
#endif
  {
    for (typename DstEvaluatorType::Index index = start; index < end; ++index)
      dstEvaluator.copyCoeff(index, srcEvaluator);
  }
};

template<typename DstXprType, typename SrcXprType>
struct copy_using_evaluator_impl<DstXprType, SrcXprType, LinearVectorizedTraversal, NoUnrolling>
{
  EIGEN_STRONG_INLINE static void run(DstXprType &dst, const SrcXprType &src)
  {
    typedef typename evaluator<DstXprType>::type DstEvaluatorType;
    typedef typename evaluator<SrcXprType>::type SrcEvaluatorType;
    typedef typename DstXprType::Index Index;

    DstEvaluatorType dstEvaluator(dst);
    SrcEvaluatorType srcEvaluator(src);

    const Index size = dst.size();
    typedef packet_traits<typename DstXprType::Scalar> PacketTraits;
    enum {
      packetSize = PacketTraits::size,
      dstIsAligned = int(copy_using_evaluator_traits<DstXprType,SrcXprType>::DstIsAligned),
      dstAlignment = PacketTraits::AlignedOnScalar ? Aligned : dstIsAligned,
      srcAlignment = copy_using_evaluator_traits<DstXprType,SrcXprType>::JointAlignment
    };
    const Index alignedStart = dstIsAligned ? 0 : first_aligned(&dstEvaluator.coeffRef(0), size);
    const Index alignedEnd = alignedStart + ((size-alignedStart)/packetSize)*packetSize;

    unaligned_copy_using_evaluator_impl<dstIsAligned!=0>::run(dstEvaluator, srcEvaluator, 0, alignedStart);

    for(Index index = alignedStart; index < alignedEnd; index += packetSize)
    {
      dstEvaluator.template copyPacket<dstAlignment, srcAlignment>(index, srcEvaluator);
    }

    unaligned_copy_using_evaluator_impl<>::run(dstEvaluator, srcEvaluator, alignedEnd, size);
  }
};

template<typename DstXprType, typename SrcXprType>
struct copy_using_evaluator_impl<DstXprType, SrcXprType, LinearVectorizedTraversal, CompleteUnrolling>
{
  typedef typename DstXprType::Index Index;
  EIGEN_STRONG_INLINE static void run(DstXprType &dst, const SrcXprType &src)
  {
    typedef typename evaluator<DstXprType>::type DstEvaluatorType;
    typedef typename evaluator<SrcXprType>::type SrcEvaluatorType;

    DstEvaluatorType dstEvaluator(dst);
    SrcEvaluatorType srcEvaluator(src);

    enum { size = DstXprType::SizeAtCompileTime,
           packetSize = packet_traits<typename DstXprType::Scalar>::size,
           alignedSize = (size/packetSize)*packetSize };

    copy_using_evaluator_innervec_CompleteUnrolling
      <DstEvaluatorType, SrcEvaluatorType, 0, alignedSize>
      ::run(dstEvaluator, srcEvaluator);
    copy_using_evaluator_DefaultTraversal_CompleteUnrolling
      <DstEvaluatorType, SrcEvaluatorType, alignedSize, size>
      ::run(dstEvaluator, srcEvaluator);
  }
};

/**************************
*** Inner vectorization ***
**************************/

template<typename DstXprType, typename SrcXprType>
struct copy_using_evaluator_impl<DstXprType, SrcXprType, InnerVectorizedTraversal, NoUnrolling>
{
  inline static void run(DstXprType &dst, const SrcXprType &src)
  {
    typedef typename evaluator<DstXprType>::type DstEvaluatorType;
    typedef typename evaluator<SrcXprType>::type SrcEvaluatorType;
    typedef typename DstXprType::Index Index;

    DstEvaluatorType dstEvaluator(dst);
    SrcEvaluatorType srcEvaluator(src);

    const Index innerSize = dst.innerSize();
    const Index outerSize = dst.outerSize();
    const Index packetSize = packet_traits<typename DstXprType::Scalar>::size;
    for(Index outer = 0; outer < outerSize; ++outer)
      for(Index inner = 0; inner < innerSize; inner+=packetSize) {
	dstEvaluator.template copyPacketByOuterInner<Aligned, Aligned>(outer, inner, srcEvaluator);
      }
  }
};

template<typename DstXprType, typename SrcXprType>
struct copy_using_evaluator_impl<DstXprType, SrcXprType, InnerVectorizedTraversal, CompleteUnrolling>
{
  EIGEN_STRONG_INLINE static void run(DstXprType &dst, const SrcXprType &src)
  {
    typedef typename evaluator<DstXprType>::type DstEvaluatorType;
    typedef typename evaluator<SrcXprType>::type SrcEvaluatorType;

    DstEvaluatorType dstEvaluator(dst);
    SrcEvaluatorType srcEvaluator(src);

    copy_using_evaluator_innervec_CompleteUnrolling
      <DstEvaluatorType, SrcEvaluatorType, 0, DstXprType::SizeAtCompileTime>
      ::run(dstEvaluator, srcEvaluator);
  }
};

template<typename DstXprType, typename SrcXprType>
struct copy_using_evaluator_impl<DstXprType, SrcXprType, InnerVectorizedTraversal, InnerUnrolling>
{
  typedef typename DstXprType::Index Index;
  EIGEN_STRONG_INLINE static void run(DstXprType &dst, const SrcXprType &src)
  {
    typedef typename evaluator<DstXprType>::type DstEvaluatorType;
    typedef typename evaluator<SrcXprType>::type SrcEvaluatorType;

    DstEvaluatorType dstEvaluator(dst);
    SrcEvaluatorType srcEvaluator(src);

    const Index outerSize = dst.outerSize();
    for(Index outer = 0; outer < outerSize; ++outer)
      copy_using_evaluator_innervec_InnerUnrolling
	<DstEvaluatorType, SrcEvaluatorType, 0, DstXprType::InnerSizeAtCompileTime>
        ::run(dstEvaluator, srcEvaluator, outer);
  }
};

/***********************
*** Linear traversal ***
***********************/

template<typename DstXprType, typename SrcXprType>
struct copy_using_evaluator_impl<DstXprType, SrcXprType, LinearTraversal, NoUnrolling>
{
  inline static void run(DstXprType &dst, const SrcXprType &src)
  {
    typedef typename evaluator<DstXprType>::type DstEvaluatorType;
    typedef typename evaluator<SrcXprType>::type SrcEvaluatorType;
    typedef typename DstXprType::Index Index;

    DstEvaluatorType dstEvaluator(dst);
    SrcEvaluatorType srcEvaluator(src);

    const Index size = dst.size();
    for(Index i = 0; i < size; ++i)
      dstEvaluator.copyCoeff(i, srcEvaluator);
  }
};

template<typename DstXprType, typename SrcXprType>
struct copy_using_evaluator_impl<DstXprType, SrcXprType, LinearTraversal, CompleteUnrolling>
{
  EIGEN_STRONG_INLINE static void run(DstXprType &dst, const SrcXprType &src)
  {
    typedef typename evaluator<DstXprType>::type DstEvaluatorType;
    typedef typename evaluator<SrcXprType>::type SrcEvaluatorType;

    DstEvaluatorType dstEvaluator(dst);
    SrcEvaluatorType srcEvaluator(src);

    copy_using_evaluator_LinearTraversal_CompleteUnrolling
      <DstEvaluatorType, SrcEvaluatorType, 0, DstXprType::SizeAtCompileTime>
      ::run(dstEvaluator, srcEvaluator);
  }
};

/**************************
*** Slice vectorization ***
***************************/

template<typename DstXprType, typename SrcXprType>
struct copy_using_evaluator_impl<DstXprType, SrcXprType, SliceVectorizedTraversal, NoUnrolling>
{
  inline static void run(DstXprType &dst, const SrcXprType &src)
  {
    typedef typename evaluator<DstXprType>::type DstEvaluatorType;
    typedef typename evaluator<SrcXprType>::type SrcEvaluatorType;
    typedef typename DstXprType::Index Index;

    DstEvaluatorType dstEvaluator(dst);
    SrcEvaluatorType srcEvaluator(src);

    typedef packet_traits<typename DstXprType::Scalar> PacketTraits;
    enum {
      packetSize = PacketTraits::size,
      alignable = PacketTraits::AlignedOnScalar,
      dstAlignment = alignable ? Aligned : int(copy_using_evaluator_traits<DstXprType,SrcXprType>::DstIsAligned) ,
      srcAlignment = copy_using_evaluator_traits<DstXprType,SrcXprType>::JointAlignment
    };
    const Index packetAlignedMask = packetSize - 1;
    const Index innerSize = dst.innerSize();
    const Index outerSize = dst.outerSize();
    const Index alignedStep = alignable ? (packetSize - dst.outerStride() % packetSize) & packetAlignedMask : 0;
    Index alignedStart = ((!alignable) || copy_using_evaluator_traits<DstXprType,SrcXprType>::DstIsAligned) ? 0
                       : first_aligned(&dstEvaluator.coeffRef(0,0), innerSize);

    for(Index outer = 0; outer < outerSize; ++outer)
    {
      const Index alignedEnd = alignedStart + ((innerSize-alignedStart) & ~packetAlignedMask);
      // do the non-vectorizable part of the assignment
      for(Index inner = 0; inner<alignedStart ; ++inner) {
        dstEvaluator.copyCoeffByOuterInner(outer, inner, srcEvaluator);
      }

      // do the vectorizable part of the assignment
      for(Index inner = alignedStart; inner<alignedEnd; inner+=packetSize) {
        dstEvaluator.template copyPacketByOuterInner<dstAlignment, srcAlignment>(outer, inner, srcEvaluator);
      }

      // do the non-vectorizable part of the assignment
      for(Index inner = alignedEnd; inner<innerSize ; ++inner) {
        dstEvaluator.copyCoeffByOuterInner(outer, inner, srcEvaluator);
      }

      alignedStart = std::min<Index>((alignedStart+alignedStep)%packetSize, innerSize);
    }
  }
};

/***************************************************************************
* Part 4 : Entry points
***************************************************************************/

// Based on DenseBase::LazyAssign()

template<typename DstXprType, typename SrcXprType>
const DstXprType& copy_using_evaluator(const DstXprType& dst, const SrcXprType& src)
{
#ifdef EIGEN_DEBUG_ASSIGN
  internal::copy_using_evaluator_traits<DstXprType, SrcXprType>::debug();
#endif
  copy_using_evaluator_impl<DstXprType, SrcXprType>::run(const_cast<DstXprType&>(dst), src);
  return dst;
}

// Based on DenseBase::swap()
// TODO: Chech whether we need to do something special for swapping two
//       Arrays or Matrices.

template<typename DstXprType, typename SrcXprType>
void swap_using_evaluator(const DstXprType& dst, const SrcXprType& src)
{
  copy_using_evaluator(SwapWrapper<DstXprType>(const_cast<DstXprType&>(dst)), src);
}

// Based on MatrixBase::operator+= (in CwiseBinaryOp.h)
template<typename DstXprType, typename SrcXprType>
void add_assign_using_evaluator(const MatrixBase<DstXprType>& dst, const MatrixBase<SrcXprType>& src)
{
  typedef typename DstXprType::Scalar Scalar;
  SelfCwiseBinaryOp<internal::scalar_sum_op<Scalar>, DstXprType, SrcXprType> tmp(dst.const_cast_derived());
  copy_using_evaluator(tmp, src.derived());
}

// Based on ArrayBase::operator+=
template<typename DstXprType, typename SrcXprType>
void add_assign_using_evaluator(const ArrayBase<DstXprType>& dst, const ArrayBase<SrcXprType>& src)
{
  typedef typename DstXprType::Scalar Scalar;
  SelfCwiseBinaryOp<internal::scalar_sum_op<Scalar>, DstXprType, SrcXprType> tmp(dst.const_cast_derived());
  copy_using_evaluator(tmp, src.derived());
}

// TODO: Add add_assign_using_evaluator for EigenBase ?

template<typename DstXprType, typename SrcXprType>
void subtract_assign_using_evaluator(const MatrixBase<DstXprType>& dst, const MatrixBase<SrcXprType>& src)
{
  typedef typename DstXprType::Scalar Scalar;
  SelfCwiseBinaryOp<internal::scalar_difference_op<Scalar>, DstXprType, SrcXprType> tmp(dst.const_cast_derived());
  copy_using_evaluator(tmp, src.derived());
}

template<typename DstXprType, typename SrcXprType>
void subtract_assign_using_evaluator(const ArrayBase<DstXprType>& dst, const ArrayBase<SrcXprType>& src)
{
  typedef typename DstXprType::Scalar Scalar;
  SelfCwiseBinaryOp<internal::scalar_difference_op<Scalar>, DstXprType, SrcXprType> tmp(dst.const_cast_derived());
  copy_using_evaluator(tmp, src.derived());
}

template<typename DstXprType, typename SrcXprType>
void multiply_assign_using_evaluator(const ArrayBase<DstXprType>& dst, const ArrayBase<SrcXprType>& src)
{
  typedef typename DstXprType::Scalar Scalar;
  SelfCwiseBinaryOp<internal::scalar_product_op<Scalar>, DstXprType, SrcXprType> tmp(dst.const_cast_derived());
  copy_using_evaluator(tmp, src.derived());
}

template<typename DstXprType, typename SrcXprType>
void divide_assign_using_evaluator(const ArrayBase<DstXprType>& dst, const ArrayBase<SrcXprType>& src)
{
  typedef typename DstXprType::Scalar Scalar;
  SelfCwiseBinaryOp<internal::scalar_quotient_op<Scalar>, DstXprType, SrcXprType> tmp(dst.const_cast_derived());
  copy_using_evaluator(tmp, src.derived());
}


} // namespace internal

#endif // EIGEN_ASSIGN_EVALUATOR_H

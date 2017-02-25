/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CBINARYRELATION_H_
#define CBINARYRELATION_H_

#include <mrpt/math/CMatrixTemplate.h>
#include <mrpt/math/matrix_adaptors.h>

#include <set>
#include <iterator>
#include <algorithm>
#include <utility>

namespace mrpt	{	namespace math	{
	using std::vector;

	/**
	  * This class models a binary relation through the elements of any given set. I.e. for each pair of elements (A,B) it assigns two values, f(A,B) and f(B,A).
	  * This class is useful when calling the base function is costly, since it acts like a proxy. It's also useful if the relationship values do not correspond
	  * with the return value of a function. Although it theoretically supports objects with non-trivial constructors or destructors (indicated by specifying
	  * "true" as the thrid parameter of the template instantiation), certain operations will cause memory leaks and may even cause undefined behaviour, so it's
	  * reccomended to use only basic types for the parameter U. The parameter T may be any complex object, however, like a smart pointer.
	 * \ingroup mrpt_base_grp
	  */
	template<typename T,typename U,bool UIsObject=false> class CBinaryRelation	{
	private:
		//TODO: VIRTUALIZE INSERTROWSANDCOLS!!! AND REIMPLEMENT IN CMATRIXTEMPLATEOBJECTS.

		typedef typename detail::MatrixWrapper<U,UIsObject>::MatrixType MatrixType;	//!<Matrix type used to store the actual relation.
	public:
		typedef U (*SimpleFunctionByReturnValue)(T,T);	//!< Simple function type, used to initialize chunks of the matrix.
		typedef U (*FunctionByReturnValue)(const T &,const T &);	//!<Function type which obtains the relation value by a return value.
		typedef void (*FunctionByReferencePass)(const T &,const T &,U &);	//!<Function type which obtains the relation value by reference pass.
		typedef typename std::set<T>::const_iterator const_iterator;	//!<Constant iterator through the set elements.
		typedef typename std::set<T>::const_reverse_iterator const_reverse_iterator;	//!<Constant reverse iterator through the set elements.
		typedef CMatrixRowAccessor<U> AccessorForFirstElement;	//!<Accessor type to every value related to any element A, i.e., f(A,x).
		typedef CMatrixColumnAccessor<U> AccessorForSecondElement;	//!<Accessor type to every value related to any element B, i.e., f(x,B).
		typedef CConstMatrixRowAccessor<U> ConstAccessorForFirstElement;	//!<Const accessor type to every value related to any element A, i.e., f(A,x).
		typedef CConstMatrixColumnAccessor<U> ConstAccessorForSecondElement;	//!<Const accessor type to every value related to any element B, i.e., f(x,B).
	private:
		std::set<T> elements;	//!<Actual set of elements.
		MatrixType relation;	//!<Matrix storing the relation.

		/**
		  * Template used to make the function interface independent from the function type.
		  *  (wrapper for the global method - needed to make this compile under GCC).
		  */
		template<typename FunctionType> inline void applyFunction(FunctionType fun,size_t e1,size_t e2,const T &T1,const T &T2)	{
			detail::applyFunction<T,U,UIsObject,FunctionType>(*this,fun,e1,e2,T1,T2);
		}

	public:
		/**
		  * Default constructor, doesn't initialize the relation.
		  */
		explicit inline CBinaryRelation(const std::set<T> &els):elements(els),relation(els.size(),els.size())	{}
		/**
		  * Constructor which initializes the relation using a given function.
		  */
		template<typename FunctionType> inline CBinaryRelation(const std::set<T> &els,FunctionType fun):elements(els),relation(els.size(),els.size())	{
			initializeWith(fun);
		}
		/**
		  * Initialize the whole relation with a given function.
		  */
		template<typename FunctionType> void initializeWith(FunctionType fun)	{
			typename std::set<T>::const_iterator it=elements.begin();
			for (size_t i=0;i<elements.size();++i,++it)	{
				typename std::set<T>::const_iterator jt=elements.begin();
				for (size_t j=0;j<elements.size();++j,++jt) applyFunction(fun,i,j,*it,*jt);
			}
		}
		/**
		  * Initialize the whole relation with a given function, assuming that the relation is symmetrical.
		  */
		template<typename FunctionType> void initializeSymmetricallyWith(FunctionType fun)	{
			typename std::set<T>::const_iterator it=elements.begin();
			for (size_t i=0;i<elements.size();++i,++it)	{
				applyFunction(fun,i,i,*it,*it);
				typename std::set<T>::const_iterator jt=it;
				jt++;
				for (size_t j=i+1;j<elements.size();++j,++jt)	{
					applyFunction(fun,i,j,*it,*jt);
					relation(j,i)=relation(i,j);
				}
			}
		}
		/**
		  * Manually set a relationship value, given the indices.
		  */
		inline void setRelationValue(size_t e1,size_t e2,const U &newVal)	{
			relation.get_unsafe(e1,e2)=newVal;
		}
		/**
		  * Get a relation value, given the indices.
		  */
		inline const U &getRelationValue(size_t e1,size_t e2) const	{
			return relation.get_unsafe(e1,e2);
		}
		inline const U &operator()(size_t e1,size_t e2) const	{
			return getRelationValue(e1,e2);
		}
		/**
		  * Get a reference to a relation value given its indices, which allows both querying and setting the value.
		  */
		inline U &getRelationValue(size_t e1,size_t e2)	{
			return relation.get_unsafe(e1,e2);
		}
		inline U &operator()(size_t e1,size_t e2)	{
			return getRelationValue(e1,e2);
		}
		/**
		  * Manually set a relationship value, given the elements. Returns false if any of the elements is not present.
		  */
		inline bool setRelationValue(const T &t1,const T &t2,const U &newVal)	{
			typename std::set<T>::const_iterator b=elements.begin(),e=elements.end();
			typename std::set<T>::const_iterator it1=std::find(b,e,t1),it2=std::find(b,e,t2);
			if (it1==e||it2==e) return false;
			setRelationValue(static_cast<size_t>(std::distance(b,it1)),static_cast<size_t>(std::distance(b,it2)),newVal);
			return true;
		}
		/**
		  * Get a relation value, given the elements. Throws domain_error if any of the elements is not present.
		  */
		inline U getRelationValue(const T &t1,const T &t2) const	{
			typename std::set<T>::const_iterator b=elements.begin(),e=elements.end();
			typename std::set<T>::const_iterator it1=std::find(b,e,t1),it2=std::find(b,e,t2);
			if (it1==e||it2==e) throw std::domain_error("Element not found");
			return getRelationValue(static_cast<size_t>(std::distance(b,it1)),static_cast<size_t>(std::distance(b,it2)));
		}
		/**
		  * Get a reference to a relation value given the elements, which allows both querying and setting. Throws domain_error if any of the elements is not
		  * present.
		  */
		inline U &getRelationValue(const T &t1,const T &t2)	{
			typename std::set<T>::const_iterator b=elements.begin(),e=elements.end();
			typename std::set<T>::const_iterator it1=std::find(b,e,t1),it2=std::find(b,e,t2);
			if (it1==e||it2==e) throw std::domain_error("Element not found");
			return getRelationValue(static_cast<size_t>(std::distance(b,it1)),static_cast<size_t>(distance(b,it2)));
		}
		/**
		  * Gets an iterator to the starting point of the elements set.
		  */
		inline const_iterator begin() const	{
			return elements.begin();
		}
		/**
		  * Gets an iterator to the ending point of the elements set.
		  */
		inline const_iterator end() const	{
			return elements.end();
		}
		/**
		  * Gets a reverse iterator to the ending point of the elements set.
		  */
		inline const_reverse_iterator rbegin() const	{
			return elements.rbegin();
		}
		/**
		  * Gets a reverse iterator to the starting point of the elements set.
		  */
		inline const_reverse_iterator rend() const	{
			return elements.rend();
		}
		/**
		  * Operator for direct access to a element given its index.
		  */
		T operator[](size_t i) const	{
			ASSERT_BELOW_(i,elements.size())
			typename std::set<T>::const_iterator it=elements.begin();
			std::advance(it,i);
			return *it;
		}
		/**
		  * Gets an accessor for every value related to an element A given its index, i.e., every f(A,x). This accessor is iterable.
		  */
		inline AccessorForFirstElement getRelationFrom(size_t i)	{
			return AccessorForFirstElement(relation,i);
		}
		/**
		  * Gets a constant accessor for every value related to an element A given its index, i.e., every f(A,x). This accessor is iterable.
		  */
		inline ConstAccessorForFirstElement getRelationFrom(size_t i) const	{
			return ConstAccessorForFirstElement(relation,i);
		}
		/**
		  * Gets an accessor for every value related to an element B given its index, i.e., every f(x,B). This accessor is iterable.
		  */
		inline AccessorForSecondElement getRelationTo(size_t i)	{
			return AccessorForSecondElement(relation,i);
		}
		/**
		  * Gets a constant accessor for every value related to an element B given its index, i.e., every f(x,B). This accessor is fully iterable.
		  */
		inline ConstAccessorForSecondElement getRelationTo(size_t i) const	{
			return ConstAccessorForSecondElement(relation,i);
		}
		/**
		  * Gets an iterable accessor for every value related to an element A, i.e., every f(A,x). A domain_error will be thrown if the element is not present.
		  */
		inline AccessorForFirstElement getRelationFrom(const T &t)	{
			typename std::set<T>::const_iterator b=elements.begin(),e=elements.end();
			typename std::set<T>::const_iterator it=std::find(b,e,t);
			if (it==e) throw std::domain_error("Element not found");
			return getRelationFrom(static_cast<size_t>(std::distance(b,it)));
		}
		/**
		  * Gets an iterable constant accessor for every value related to an element A, i.e., every f(A,x). A domain_error will be thrown if the element is not
		  * present.
		  */
		inline ConstAccessorForFirstElement getRelationFrom(const T &t) const	{
			typename std::set<T>::const_iterator b=elements.begin(),e=elements.end();
			typename std::set<T>::const_iterator it=std::find(b,e,t);
			if (it==e) throw std::domain_error("Element not found");
			return getRelationFrom(static_cast<size_t>(std::distance(b,it)));
		}
		inline void getRelationFrom(size_t i,vector<U> &vec)	{
			size_t N=elements.size();
			ASSERT_(i<N);
			vec.resize(N);
			ConstAccessorForFirstElement access(relation,i);
			std::copy(access.begin(),access.end(),vec.begin());
		}
		inline void getRelationFrom(const T &t,vector<U> &vec)	{
			typename std::set<T>::const_iterator b=elements.begin(),e=elements.end();
			typename std::set<T>::const_iterator it=std::find(b,e,t);
			if (it==e) throw std::domain_error("Element not found");
			getRelationFrom(static_cast<size_t>(std::distance(b,it)),vec);
		}
		/**
		  * Gets an iterable accessor for every value related to an element B, i.e., every f(x,B). A domain_error will be thrown if the element is not present.
		  */
		inline AccessorForSecondElement getRelationTo(const T &t)	{
			typename std::set<T>::const_iterator b=elements.begin(),e=elements.end();
			typename std::set<T>::const_iterator it=std::find(b,e,t);
			if (it==e) throw std::domain_error("Element not found");
			return getRelationTo(static_cast<size_t>(std::distance(b,it)));
		}
		/**
		  * Gets an iterable constant accessor for every value related to an alement B, i.e., every f(x,B). A domain_error will be thrown if the element is not
		  * present.
		  */
		inline ConstAccessorForSecondElement getRelationTo(const T &t) const	{
			typename std::set<T>::const_iterator b=elements.begin(),e=elements.end();
			typename std::set<T>::const_iterator it=std::find(b,e,t);
			if (it==e) throw std::domain_error("Element not found");
			return getRelationTo(static_cast<size_t>(std::distance(b,it)));
		}
		inline void getRelationTo(size_t i,vector<U> &vec)	{
			size_t N=elements.size();
			ASSERT_(i<N);
			vec.resize(N);
			ConstAccessorForSecondElement access(relation,i);
			std::copy(access.begin(),access.end(),vec.begin());
		}
		inline void getRelationTo(const T &t,vector<U> &vec)	{
			typename std::set<T>::const_iterator b=elements.begin(),e=elements.end();
			typename std::set<T>::const_iterator it=std::find(b,e,t);
			if (it==e) throw std::domain_error("Element not found");
			getRelationTo(static_cast<size_t>(std::distance(b,it)),vec);
		}
		/**
		  * Removes an element at a concrete position.
		  */
		void removeElementAt(size_t i)	{
			ASSERT_(i<elements.size());
			typename std::set<T>::const_iterator it=elements.begin();
			std::advance(it,i);
			elements.erase(i);
			std::set<size_t> ii;
			ii.insert(i);
			relation.removeRowsAndCols(ii,ii);
		}
		/**
		  * Removes an element. Returns false if the element was not present and thus could'nt be eliminated.
		  */
		bool removeElement(const T &el)	{
			typename std::set<T>::const_iterator b=elements.begin(),e=elements.end();
			typename std::set<T>::const_iterator it=std::find(e,b,el);
			if (it==e) return false;
			removeElementAt(std::distance(b,it));
			return true;
		}
		/**
		  * Removes a set of elements. Returns the number of elements which were actually erased.
		  */
		size_t removeElements(const std::set<T> &vals)	{
			std::set<size_t> positions;
			for (typename std::set<T>::const_iterator it=vals.begin();it!=vals.end();++it)	{
				typename std::set<T>::iterator elsIt=std::find(elements.begin(),elements.end(),*it);
				if (elsIt!=elements.end()) positions.insert(std::distance(elements.begin(),elsIt));
			}
			removeElementsAt(positions);
			return positions.size();
		}
		void removeElementsAt(const std::set<size_t> &poss)	{
			relation.removeRowsAndCols(poss,poss);
			for (std::set<size_t>::const_reverse_iterator it=poss.rbegin();it!=poss.rend();++it)	{
				typename std::set<T>::const_iterator it2=elements.begin();
				std::advance(it2,*it);
				elements.erase(it2);
			}
		}
		/**
		  * Inserts an element. If the element was present, returns false and its current position. If it wasn't, returns true and the position in which it was
		  * inserted.
		  */
		std::pair<bool,size_t> insertElement(const T &el)	{
			std::pair<typename std::set<T>::iterator,bool> ins=elements.insert(el);
			size_t dist=std::distance(elements.begin(),ins.first);
			if (ins.second)	{
				std::multiset<size_t> newEls;
				newEls.insert(dist);
				relation.insertRowsAndCols(newEls,newEls);
				return std::make_pair(true,dist);
			}	else return std::make_pair(false,dist);
		}
		/**
		  * Inserts an element and initializes its relationship values, even if it was already present.
		  */
		template<typename FunctionType> std::pair<bool,size_t> insertElement(const T &el,FunctionType fun)	{
			std::pair<bool,size_t> ins=insertElement(el);
			size_t pos=ins.second;
			for (size_t i=0;i<elements.size();++i)	{
				const T &newEl=operator[](i);
				applyFunction(fun,pos,i,el,newEl);
				applyFunction(fun,i,pos,newEl,el);
			}
			return ins;
		}
		/**
		  * Inserts a set of elements into the relation. Does not initialize the actual relation.
		  */
		size_t insertElements(const std::set<T> &els)	{
			if (els.empty()) return 0;
			//This code is much more complex than it should! Trying, for efficiency, to avoid multiple calls to insertElement makes things a lot harder.
			//It raises the complexity level to N^2, but alleviates it greatly by making a single memory allocation. Multiple calls to insertElement will be
			//faster only if the number of elements in the set is really large.
			std::vector<size_t> added;
			//std::vector<size_t> exist;
			added.reserve(els.size());
			for (typename std::set<T>::const_iterator it=els.begin();it!=els.end();++it)	{
				std::pair<typename std::set<T>::iterator,bool> ins=elements.insert(*it);
				size_t dist=std::distance(elements.begin(),ins.first);
				if (ins.second)	{
					added.push_back(dist);
					for (std::vector<size_t>::iterator it2=added.begin();it2!=added.end();++it2) if (*it2>=dist) ++(*it2);
					//for (std::vector<size_t>::iterator it2=exist.begin();it2!=exist.end();++it2) if (*it2>=dist) ++(*it2);
				}//	else exist.push_back(dist);
			}
			std::sort(added.begin(),added.end());
			for (size_t j=1;j<added.size();++j) added[j]-=j;
			std::multiset<size_t> poss(added.begin(),added.end());
			relation.insertRowsAndCols(poss,poss);
			return added.size();
		}
		/**
		  * Inserts a set of elements into the relation, initializing the actual relation with a given function.
		  */
		template<typename FunctionType> size_t insertElements(const std::set<T> &els,FunctionType fun)	{
			if (els.empty()) return 0;
			size_t howMany=insertElements(els);
			std::set<size_t> poss;
			{
				//Little scope for "begin" and "end"...
				typename std::set<T>::const_iterator begin=elements.begin(),end=elements.end();
				for (typename std::set<T>::const_iterator it=els.begin();it!=els.end();++it) poss.insert(std::distance(begin,find(begin,end,*it)));
			}
			std::set<size_t> nPoss;
			std::set<size_t>::const_iterator begin=poss.begin(),end=poss.end();
			for (size_t i=0;i<elements.size();++i) if (std::find(begin,end,i)==end) nPoss.insert(i);
			vector<const T *> proxy;
			proxy.reserve(poss.size());
			for (std::set<size_t>::const_iterator it=begin;it!=end;++it)	{
				const T &e1=operator[](*it);
				proxy.push_back(&e1);
				size_t i=0;
				for (typename std::set<T>::const_iterator it2=elements.begin();it2!=elements.end();++it2,++i) applyFunction(fun,*it,i,e1,*it2);
			}
			for (std::set<size_t>::const_iterator it=nPoss.begin();it!=nPoss.end();++it)	{
				const T &e1=operator[](*it);
				typename std::vector<const T *>::const_iterator itV=proxy.begin();
				for (std::set<size_t>::const_iterator it2=poss.begin();it2!=poss.end();++it2,++itV) applyFunction(fun,*it,*it2,e1,**itV);
			}
			return howMany;
		}
		/**
		  * Completely resets the relation, using a new set of elements. Does not initialize the relation.
		  */
		void setElements(const std::set<T> &newEls)	{
			relation.setSize(0,0);
			elements=newEls;
			relation.setSize(newEls.size(),newEls.size());
		}
		/**
		  * Returns the amount of elements present in the relation.
		  */
		inline size_t size() const	{
			return elements.size();
		}
	};

	namespace detail {
		// generic version (specialization is after definition of CBinaryRelation):
		template<typename T,typename U,bool UIsObject,typename FunctionType> inline void applyFunction(CBinaryRelation<T,U,UIsObject> &o, FunctionType fun,size_t e1,size_t e2,const T &T1,const T &T2)	{
			o.getRelationValue(e1,e2)=fun(T1,T2);
		}

		/** Template specialization by reference type.
		  */
		template<typename T,typename U,bool UIsObject> inline void applyFunction(CBinaryRelation<T,U,UIsObject> &o,typename CBinaryRelation<T,U,UIsObject>::FunctionByReferencePass fun,size_t e1,size_t e2,const T &T1,const T &T2)	{
			fun(T1,T2,o.getRelationValue(e1,e2));
		}
	}


}}	//End of namespaces
#endif

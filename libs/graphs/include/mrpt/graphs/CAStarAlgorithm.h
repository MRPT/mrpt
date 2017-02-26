/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CASTARALGORITHM_H
#define CASTARALGORITHM_H
#include <map>
#include <vector>
#define _USE_MATH_DEFINES // (For VS to define M_PI, etc. in cmath)
#include <cmath>
#include <mrpt/utils/CTicTac.h>

namespace mrpt
{
	namespace graphs
	{
		/** This class is intended to efficiently solve graph-search problems using heuristics to determine the best path. To use it, a solution class must be defined
		  * so that it contains all the information about any partial or complete solution. Then, a class inheriting from CAStarAlgorithm<Solution class> must also be
		  * implemented, overriding five virtual methods which define the behaviour of the solutions. These methods are isSolutionEnded, isSolutionValid,
		  * generateChildren, getHeuristic and getCost.
		  * Once both classes are generated, each object of the class inheriting from CAStarAlgorithm represents a problem who can be solved by calling
		  * getOptimalSolution. See http://en.wikipedia.org/wiki/A*_search_algorithm for details about how this algorithm works.
		  * \sa CAStarAlgorithm::isSolutionEnded
		  * \sa CAStarAlgorithm::isSolutionValid
		  * \sa CAStarAlgorithm::generateChildren
		  * \sa CAStarAlgorithm::getHeuristic
		  * \sa CAStarAlgorithm::getCost
		  * \ingroup mrpt_graphs_grp
		  */
		template<typename T> class CAStarAlgorithm	{
		public:
			/**
			  * Client code must implement this method.
			  * Returns true if the given solution is complete.
			  */
			virtual bool isSolutionEnded(const T &sol)=0;
			/**
			  * Client code must implement this method.
			  * Returns true if the given solution is acceptable, that is, doesn't violate the problem logic.
			  */
			virtual bool isSolutionValid(const T &sol)=0;
			/**
			  * Client code must implement this method.
			  * Given a partial solution, returns all its children solution, regardless of their validity or completeness.
			  */
			virtual void generateChildren(const T &sol,std::vector<T> &sols)=0;
			/**
			  * Client code must implement this method.
			  * Given a partial solution, estimates the cost of the remaining (unknown) part.
			  * This cost must always be greater or equal to zero, and not greater than the actual cost. Thus, must be 0 if the solution is complete.
			  */
			virtual double getHeuristic(const T &sol)=0;
			/**
			  * Client code must implement this method.
			  * Given a (possibly partial) solution, calculates its cost so far.
			  * This cost must not decrease with each step. That is, a solution cannot have a smaller cost than the previous one from which it was generated.
			  */
			virtual double getCost(const T &sol)=0;
		private:
			/**
			  * Calculates the total cost (known+estimated) of a solution.
			  */
			inline double getTotalCost(const T &sol)	{
				return getHeuristic(sol)+getCost(sol);
			}
		public:
			/**
			  * Finds the optimal solution for a problem, using the A* algorithm. Returns whether an optimal solution was actually found.
			  * Returns 0 if no solution was found, 1 if an optimal solution was found and 2 if a (possibly suboptimal) solution was found but the time lapse ended.
			  */
			int getOptimalSolution(const T &initialSol,T &finalSol,double upperLevel=HUGE_VAL,double maxComputationTime=HUGE_VAL)	{
				//Time measuring object is defined.
				mrpt::utils::CTicTac time;
				time.Tic();
				//The partial solution set is initialized with a single element (the starting solution).
				std::multimap<double,T> partialSols;
				partialSols.insert(std::pair<double,T>(getTotalCost(initialSol),initialSol));
				//The best known solution is set to the upper bound (positive infinite, if there is no given parameter).
				double currentOptimal=upperLevel;
				bool found=false;
				std::vector<T> children;
				//Main loop. Each iteration checks an element of the set, with minimum estimated cost.
				while (!partialSols.empty())	{
					//Return if elapsed time has been reached.
					if (time.Tac()>=maxComputationTime) return found?2:0;
					typename std::multimap<double,T>::iterator it=partialSols.begin();
					double tempCost=it->first;
					//If the minimum estimated cost is higher than the upper bound, then also is every solution in the set. So the algorithm returns immediately.
					if (tempCost>=currentOptimal) return found?1:0;
					T tempSol=it->second;
					partialSols.erase(it);
					//At this point, the solution cost is lesser than the upper bound. So, if the solution is complete, the optimal solution and the upper bound are updated.
					if (isSolutionEnded(tempSol))	{
						currentOptimal=tempCost;
						finalSol=tempSol;
						found=true;
						continue;
					}
					//If the solution is not complete, check for its children. Each one is included in the set only if it's valid and it's not yet present in the set.
					generateChildren(tempSol,children);
					for (typename std::vector<T>::const_iterator it2=children.begin();it2!=children.end();++it2) if (isSolutionValid(*it2))	{
						bool alreadyPresent=false;
						double cost=getTotalCost(*it2);
						typename std::pair<typename std::multimap<double,T>::const_iterator,typename std::multimap<double,T>::const_iterator> range = partialSols.equal_range(cost);
						for (typename std::multimap<double,T>::const_iterator it3=range.first;it3!=range.second;++it3) if (it3->second==*it2)	{
							alreadyPresent=true;
							break;
						}
						if (!alreadyPresent) partialSols.insert(std::pair<double,T>(getTotalCost(*it2),*it2));
					}
				}
				//No more solutions to explore...
				return found?1:0;
			}
		};
	}
}	//End of namespaces
#endif

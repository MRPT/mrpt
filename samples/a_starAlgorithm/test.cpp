/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#define _CRT_SECURE_NO_WARNINGS

#include <mrpt/graphs/CAStarAlgorithm.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>

using namespace std;
using namespace mrpt::graphs;

/**
  * This is a example of problem resolution using the CAStarAlgorithm template. Although this problem is better solved with dynamic programming, it illustrates
  * perfectly how to inherit from that template in order to solve a problem.
  *
  * Let's assume a currency composed of coins whose values are 2, 7, 8 and 19. The problem consists of finding a minimal set of these coins whose total value
  * equals a given amount.
  */

class CCoinDistribution	{
public:
	size_t coins2;
	size_t coins7;
	size_t coins8;
	size_t coins19;
	CCoinDistribution():coins2(0),coins7(0),coins8(0),coins19(0)	{}

	/**
	  * Auxiliary function to calculate the amount of money. Not strictly necessary, but handy.
	  */
	size_t money() const	{
		return 2*coins2+7*coins7+8*coins8+19*coins19;
	}
	/**
	  * Implementing the == operator is mandatory, because it allows the A* algorithm to reject repeated solutions. Actually, the template should not compile if
	  * this operator is not present.
	  */
	inline bool operator==(const CCoinDistribution &mon) const	{
		return (coins2==mon.coins2)&&(coins7==mon.coins7)&&(coins8==mon.coins8)&&(coins19==mon.coins19);
	}
};

/**
  * To use the template, a class must be derived from CAStarAlgorithm<Solution Class>. In this case, the Solution Class is CCoinDistribution.
  */
class CAStarExample:public CAStarAlgorithm<CCoinDistribution>	{
private:
	/**
	  * Problem goal.
	  */
	const size_t N;
public:
	/**
	  * When a class derives from CAStarAlgorithm, its constructor should include all the data that define the specific problem.
	  */
	CAStarExample(size_t goal):N(goal)	{}
	/**
	  * The following five methods must be implemented to use the algorithm:
	  */
	virtual bool isSolutionEnded(const CCoinDistribution &s)	{	//True if the solution is complete.
		return s.money()==N;
	}
	virtual bool isSolutionValid(const CCoinDistribution &s)	{	//True if the solution is valid.
		return s.money()<=N;
	}
	virtual void generateChildren(const CCoinDistribution &s,vector<CCoinDistribution> &sols)	{	//Get all the children of a solution.
		//Complex classes might want to define a copy constructor (note how the vector in the following line is created by cloning this object four times).
		sols=vector<CCoinDistribution>(4,s);
		sols[0].coins2++;	//Misma solución, más una moneda de 2.
		sols[1].coins7++;	//Misma solución, más una moneda de 7.
		sols[2].coins8++;	//Misma solución, más una moneda de 8...
		sols[3].coins19++;	//Y misma solución, más una moneda de 19.
	}
	virtual double getHeuristic(const CCoinDistribution &s)	{	//Heuristic cost of the remaining part of the solution.
		//Check the documentation of CAStarAlgorithm to know which characteristics does this function need to comply.
		return static_cast<double>(N-s.money())/19.0;
	}
	virtual double getCost(const CCoinDistribution &s)	{	//Known cost of the partial solution.
		return s.coins2+s.coins7+s.coins8+s.coins19;
	}
};

/**
  * Main function. Just calls the A* algorithm as many times as needed.
  */
int main(int argc,char **argv)	{
	for (;;)	{
		char text[11];
		printf("Input an integer number to solve a problem, or \"e\" to end.\n");
		if (1!=scanf("%10s",text))  // GCC warning: scanf -> return cannot be ignored!
		{
			printf("Please, input a positive integer.\n\n");
			continue;
		}
		if (strlen(text)==1&&(text[0]=='e'||text[0]=='E')) break;
		int val=atoi(text);
		if (val<=0)	{
			printf("Please, input a positive integer.\n\n");
			continue;
		}
		//The solution objects and the problem are created...
		CCoinDistribution solIni,solFin;	//Initial solution automatically initialized to (0,0,0,0).
		CAStarExample prob(static_cast<size_t>(val));
		switch (prob.getOptimalSolution(solIni,solFin,HUGE_VAL,15))	{
			case 0:
				printf("No solution has been found. Either the number is too small, or the time elapsed has exceeded 15 seconds.\n\n");
				break;
			case 1:
				printf("An optimal solution has been found:\n");
				printf("\t%u coins of 2 piastres.\n\t%u coins of 7 piastres.\n\t%u coins of 8 piastres.\n\t%u coins of 19 piastres.\n\n",(unsigned)solFin.coins2,(unsigned)solFin.coins7,(unsigned)solFin.coins8,(unsigned)solFin.coins19);
				break;
			case 2:
				printf("A solution has been found, although it may not be optimal:\n");
				printf("\t%u coins of 2 piastres.\n\t%u coins of 7 piastres.\n\t%u coins of 8 piastres.\n\t%u coins of 19 piastres.\n\n",(unsigned)solFin.coins2,(unsigned)solFin.coins7,(unsigned)solFin.coins8,(unsigned)solFin.coins19);
				break;
		}
	}
	return 0;
}

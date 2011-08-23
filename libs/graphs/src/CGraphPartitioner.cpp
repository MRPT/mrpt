/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/graphs.h>  // Precompiled headers
#include <mrpt/graphs/CGraphPartitioner.h>

using namespace mrpt;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

bool CGraphPartitioner::DEBUG_SAVE_EIGENVECTOR_FILES = false;
int CGraphPartitioner::debug_file_no = 0;
bool CGraphPartitioner::VERBOSE = false;

/*---------------------------------------------------------------
					SpectralPartition
  ---------------------------------------------------------------*/
void  CGraphPartitioner::SpectralBisection(
			CMatrix					&in_A,
			vector_uint		&out_part1,
			vector_uint		&out_part2,
			float					&out_cut_value,
			bool					forceSimetry )
{
	size_t				nodeCount;		// Nodes count
	size_t    			i,j;
    CMatrix				Adj, eigenVectors,eigenValues;

	// Check matrix is square:
	if (in_A.getColCount() != (nodeCount = in_A.getRowCount()) )
		THROW_EXCEPTION("Weights matrix is not square!!");

	// Shi & Malik's method
    //-----------------------------------------------------------------

    // forceSimetry?
	if (forceSimetry)
	{
		Adj.setSize(nodeCount,nodeCount);
		for (i=0;i<nodeCount;i++)
			for (j=i;j<nodeCount;j++)
				Adj(i,j)=Adj(j,i)= 0.5f*(in_A(i,j)+in_A(j,i));
	}
	else Adj = in_A;

    // Compute eigen-vectors of laplacian:
	CMatrix   LAPLACIAN;
	Adj.laplacian(LAPLACIAN);


    LAPLACIAN.eigenVectors( eigenVectors, eigenValues );

	//  Execute the bisection
    // ------------------------------------
    // Second smallest eigen-vector
    double	mean = 0;
	size_t  colNo = 1; // second smallest
	size_t  nRows = eigenVectors.getRowCount();

	//for (i=0;i<eigenVectors.getColCount();i++) mean+=eigenVectors(colNo,i);
	for (i=0;i<nRows;i++) mean+=eigenVectors(i,colNo);
	mean /= nRows;

	out_part1.clear();
	out_part2.clear();

    for(i=0;i<nRows;i++)
    {
      if ( eigenVectors(i,colNo) >= mean)
			out_part1.push_back( i );
      else	out_part2.push_back( i );
    }

	// Special and strange case: Constant eigenvector: Split nodes in two
	//    equally sized parts arbitrarily:
	// ------------------------------------------------------------------
	if (!out_part1.size() || !out_part2.size())
	{
		out_part1.clear();
		out_part2.clear();
		// Assign 50%-50%:
		for (i=0;i<Adj.getColCount();i++)
			if (i<=Adj.getColCount()/2)
					out_part1.push_back(i);
			else	out_part2.push_back(i);
	}

	// Compute the N-cut value
	out_cut_value = nCut( Adj, out_part1, out_part2);

	// DEBUG
	if (CGraphPartitioner::DEBUG_SAVE_EIGENVECTOR_FILES)
	{
        CMatrix aux(1,1);
        aux(0,0) = out_cut_value;

        eigenVectors.saveToTextFile( format("DEBUG_GRAPHPART_eigvectors_%05u.txt",CGraphPartitioner::debug_file_no) );
        Adj.saveToTextFile( format("DEBUG_GRAPHPART_adj_%05u.txt",CGraphPartitioner::debug_file_no) );
        LAPLACIAN.saveToTextFile( format("DEBUG_GRAPHPART_laplacian_%05u.txt",CGraphPartitioner::debug_file_no) );
        eigenValues.saveToTextFile( format("DEBUG_GRAPHPART_eigenvalues_%05u.txt",CGraphPartitioner::debug_file_no) );
        aux.saveToTextFile( format("DEBUG_GRAPHPART_eigvectors_%05u_ncut.txt",CGraphPartitioner::debug_file_no) );
        debug_file_no++;
	}
}

/*---------------------------------------------------------------
					RecursiveSpectralPartition
  ---------------------------------------------------------------*/
void  CGraphPartitioner::RecursiveSpectralPartition(
	CMatrix					&in_A,
	std::vector<vector_uint>	&out_parts,
	float						threshold_Ncut,
	bool						forceSimetry,
	bool						useSpectralBisection,
	bool						recursive,
	unsigned 					minSizeClusters )
{
	MRPT_START

	size_t				nodeCount;
	vector_uint			p1,p2;
	float				cut_value;
	size_t				i,j;
	CMatrix				Adj;

	out_parts.clear();

	// Check matrix is square:
	if (in_A.getColCount() != (nodeCount = in_A.getRowCount()) )
		THROW_EXCEPTION("Weights matrix is not square!!");

	if (nodeCount==1)
	{
		// Don't split, there is just a node!
		p1.push_back(0);
		out_parts.push_back(p1);
		return;
	}

	// forceSimetry?
	if (forceSimetry)
	{
		Adj.setSize(nodeCount,nodeCount);
		for (i=0;i<nodeCount;i++)
			for (j=i;j<nodeCount;j++)
				Adj(i,j)=Adj(j,i)= 0.5f*(in_A(i,j)+in_A(j,i));
	}
	else Adj = in_A;

	// Make bisection
	if (useSpectralBisection)
			SpectralBisection( Adj, p1, p2, cut_value, false);
	else	exactBisection(Adj, p1, p2, cut_value, false);

	if (CGraphPartitioner::VERBOSE)
		cout << format("Cut:%u=%u+%u,nCut=%.02f->",(unsigned int)nodeCount,(unsigned int)p1.size(),(unsigned int)p2.size(),cut_value);

	// Is it a useful partition?
	if (cut_value>threshold_Ncut || p1.size()<minSizeClusters || p2.size()<minSizeClusters )
	{
		if (CGraphPartitioner::VERBOSE)
			cout << "->NO!" << endl;

		// No:
		p1.clear();
		for (i=0;i<nodeCount;i++) p1.push_back(i);
		out_parts.push_back(p1);
	}
	else
	{
		if (CGraphPartitioner::VERBOSE)
			cout << "->YES!" << endl;

		// Yes:
		std::vector<vector_uint>	p1_parts, p2_parts;

		if (recursive)
		{
			// Split "p1":
			// --------------------------------------------
			// sub-matrix:
			CMatrix A_1( p1.size(),p1.size() );
			for (i=0;i<p1.size();i++)
				for (j=0;j<p1.size();j++)
					A_1(i,j)= in_A(p1[i],p1[j]);

			RecursiveSpectralPartition(A_1,p1_parts, threshold_Ncut, forceSimetry,useSpectralBisection);

			// Split "p2":
			// --------------------------------------------
			// sub-matrix:
			CMatrix A_2( p2.size(),p2.size() );
			for (i=0;i<p2.size();i++)
				for (j=0;j<p2.size();j++)
					A_2(i,j)= in_A(p2[i],p2[j]);

			RecursiveSpectralPartition(A_2,p2_parts, threshold_Ncut, forceSimetry,useSpectralBisection);

			// Build "out_parts" from "p1_parts" + "p2_parts"
			//  taken care of indexes mapping!
			// -------------------------------------------------------------------------------------
			// remap p1 nodes:
			for (i=0;i<p1_parts.size();i++)
			{
				for (j=0;j<p1_parts[i].size();j++)
					p1_parts[i][j] = p1[ p1_parts[i][j] ];
				out_parts.push_back(p1_parts[i]);
			}

			// remap p2 nodes:
			for (i=0;i<p2_parts.size();i++)
			{
				for (j=0;j<p2_parts[i].size();j++)
					p2_parts[i][j] = p2[ p2_parts[i][j] ];

				out_parts.push_back(p2_parts[i]);
			}
		}
		else
		{
			// Force bisection only:
			out_parts.clear();
			out_parts.push_back(p1);
			out_parts.push_back(p2);
		}

	}

	MRPT_END
}

/*---------------------------------------------------------------
						nCut
  ---------------------------------------------------------------*/
float  CGraphPartitioner::nCut(
    const CMatrix					&in_A,
    const vector_uint		&in_part1,
    const vector_uint		&in_part2)
{
	unsigned int	i,j;
	size_t          size1=in_part1.size();
	size_t          size2=in_part2.size();

	// Compute the N-cut value
	// -----------------------------------------------
	float		cut_AB=0;
    for(i=0;i<size1;i++)
		for(j=0;j<size2;j++)
				cut_AB += in_A(in_part1[i],in_part2[j]);

	float assoc_AA = 0;
    for(i=0;i<size1;i++)
		for(j=i;j<size1;j++)
			if (i!=j)
				assoc_AA += in_A(in_part1[i],in_part1[j]);

	float assoc_BB = 0;
    for(i=0;i<size2;i++)
		for(j=i;j<size2;j++)
			if (i!=j)
				assoc_BB += in_A(in_part2[i],in_part2[j]);

	float assoc_AV = assoc_AA + cut_AB;
	float assoc_BV = assoc_BB + cut_AB;

	if (!cut_AB)
			return 0;
	else	return cut_AB/assoc_AV + cut_AB/assoc_BV;

}


/*---------------------------------------------------------------
					exactBisection
  ---------------------------------------------------------------*/
void  CGraphPartitioner::exactBisection(
			CMatrix					&in_A,
			vector_uint		&out_part1,
			vector_uint		&out_part2,
			float					&out_cut_value,
			bool					forceSimetry )
{
	size_t						nodeCount;		// Nodes count
	size_t		    			i,j;
    CMatrix						Adj;
	vector_bool					partition, bestPartition;
	vector_uint					part1,part2;
	float						partCutValue, bestCutValue = 1e+20f;
	bool						end = false;
	bool						carry;

	// Check matrix is square:
	if (in_A.getColCount() != (nodeCount = in_A.getRowCount()) )
		THROW_EXCEPTION("Weights matrix is not square!!");

	ASSERT_(nodeCount>=2);

    // forceSimetry?
	if (forceSimetry)
	{
		Adj.setSize(nodeCount,nodeCount);
		for (i=0;i<nodeCount;i++)
			for (j=i;j<nodeCount;j++)
				Adj(i,j)=Adj(j,i)= 0.5f*(in_A(i,j)+in_A(j,i));
	}
	else Adj = in_A;


	// Brute force: compute all posible partitions:
    //-----------------------------------------------------------------

	// First combination: 1000...0000
	// Last combination:  1111...1110
	partition.clear();
	partition.resize(nodeCount,false);
	partition[0] = true;

	while (!end)
	{
		// Build partitions from binary vector:
		part1.clear();
		part2.clear();

		for (i=0;i<nodeCount;i++)
		{
			if (partition[i])
					part2.push_back(i);
			else	part1.push_back(i);
		}

		// Compute the n-cut:
		partCutValue = nCut(Adj,part1,part2);

		if (partCutValue<bestCutValue)
		{
			bestCutValue = partCutValue;
			bestPartition = partition;
		}

		// Next combo in the binary vector:
		i = 0; carry = false;
		do
		{
            carry = partition[i];
			partition[i]=!partition[i];
			i++;
		} while ( carry && i<nodeCount );

		// End criterion:
		end = true;
		for (i=0;end && i<nodeCount;i++)
			end = end && partition[i];

	}; // End of while


	// Return the best partition:
	out_cut_value = bestCutValue;

	out_part1.clear();
	out_part2.clear();

	for (i=0;i<nodeCount;i++)
	{
		if (bestPartition[i])
				out_part2.push_back(i);
		else	out_part1.push_back(i);
	}

}

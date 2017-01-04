/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#if !defined(CGRAPHPARTITIONER_H)
#error "This file can't be included from outside of CGraphPartitioner.h"
#endif

namespace mrpt
{
namespace graphs
{

/*---------------------------------------------------------------
					SpectralPartition
  ---------------------------------------------------------------*/
template <class GRAPH_MATRIX, typename num_t>
void CGraphPartitioner<GRAPH_MATRIX,num_t>::SpectralBisection(
	GRAPH_MATRIX    &in_A,
	vector_uint		&out_part1,
	vector_uint		&out_part2,
	num_t			&out_cut_value,
	bool			forceSimetry )
{
	size_t  nodeCount;		// Nodes count
    GRAPH_MATRIX	Adj, eigenVectors,eigenValues;

	// Check matrix is square:
	if (in_A.getColCount() != (nodeCount = in_A.getRowCount()) )
		THROW_EXCEPTION("Weights matrix is not square!!");

	// Shi & Malik's method
    //-----------------------------------------------------------------

    // forceSimetry?
	if (forceSimetry)
	{
		Adj.setSize(nodeCount,nodeCount);
		for (size_t i=0;i<nodeCount;i++)
			for (size_t j=i;j<nodeCount;j++)
				Adj(i,j)=Adj(j,i)= 0.5f*(in_A(i,j)+in_A(j,i));
	}
	else Adj = in_A;

    // Compute eigen-vectors of laplacian:
	GRAPH_MATRIX   LAPLACIAN;
	Adj.laplacian(LAPLACIAN);


    LAPLACIAN.eigenVectors( eigenVectors, eigenValues );

	//  Execute the bisection
    // ------------------------------------
    // Second smallest eigen-vector
    double	mean = 0;
	size_t  colNo = 1; // second smallest
	size_t  nRows = eigenVectors.getRowCount();

	//for (i=0;i<eigenVectors.getColCount();i++) mean+=eigenVectors(colNo,i);
	for (size_t i=0;i<nRows;i++) mean+=eigenVectors(i,colNo);
	mean /= nRows;

	out_part1.clear();
	out_part2.clear();

    for(size_t i=0;i<nRows;i++)
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
		for (size_t i=0;i<Adj.getColCount();i++)
			if (i<=Adj.getColCount()/2)
					out_part1.push_back(i);
			else	out_part2.push_back(i);
	}

	// Compute the N-cut value
	out_cut_value = nCut( Adj, out_part1, out_part2);
}

/*---------------------------------------------------------------
					RecursiveSpectralPartition
  ---------------------------------------------------------------*/
template <class GRAPH_MATRIX, typename num_t>
void CGraphPartitioner<GRAPH_MATRIX,num_t>::RecursiveSpectralPartition(
	GRAPH_MATRIX   				&in_A,
	std::vector<vector_uint>	&out_parts,
	num_t						threshold_Ncut,
	bool						forceSimetry,
	bool						useSpectralBisection,
	bool						recursive,
	unsigned 					minSizeClusters,
	const bool verbose )
{
	MRPT_START

	size_t				nodeCount;
	vector_uint			p1,p2;
	num_t				cut_value;
	size_t				i,j;
	GRAPH_MATRIX				Adj;

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

	if (verbose)
		std::cout << format("Cut:%u=%u+%u,nCut=%.02f->",(unsigned int)nodeCount,(unsigned int)p1.size(),(unsigned int)p2.size(),cut_value);

	// Is it a useful partition?
	if (cut_value>threshold_Ncut || p1.size()<minSizeClusters || p2.size()<minSizeClusters )
	{
		if (verbose)
			std::cout << "->NO!" << std::endl;

		// No:
		p1.clear();
		for (i=0;i<nodeCount;i++) p1.push_back(i);
		out_parts.push_back(p1);
	}
	else
	{
		if (verbose)
			std::cout << "->YES!" << std::endl;

		// Yes:
		std::vector<vector_uint>	p1_parts, p2_parts;

		if (recursive)
		{
			// Split "p1":
			// --------------------------------------------
			// sub-matrix:
			GRAPH_MATRIX A_1( p1.size(),p1.size() );
			for (i=0;i<p1.size();i++)
				for (j=0;j<p1.size();j++)
					A_1(i,j)= in_A(p1[i],p1[j]);

			RecursiveSpectralPartition(A_1,p1_parts, threshold_Ncut, forceSimetry,useSpectralBisection, recursive, minSizeClusters);

			// Split "p2":
			// --------------------------------------------
			// sub-matrix:
			GRAPH_MATRIX A_2( p2.size(),p2.size() );
			for (i=0;i<p2.size();i++)
				for (j=0;j<p2.size();j++)
					A_2(i,j)= in_A(p2[i],p2[j]);

			RecursiveSpectralPartition(A_2,p2_parts, threshold_Ncut, forceSimetry,useSpectralBisection, recursive, minSizeClusters);

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
template <class GRAPH_MATRIX, typename num_t>
num_t CGraphPartitioner<GRAPH_MATRIX,num_t>::nCut(
    const GRAPH_MATRIX		&in_A,
    const vector_uint		&in_part1,
    const vector_uint		&in_part2)
{
	unsigned int	i,j;
	size_t          size1=in_part1.size();
	size_t          size2=in_part2.size();

	// Compute the N-cut value
	// -----------------------------------------------
	num_t		cut_AB=0;
    for(i=0;i<size1;i++)
		for(j=0;j<size2;j++)
				cut_AB += in_A(in_part1[i],in_part2[j]);

	num_t assoc_AA = 0;
    for(i=0;i<size1;i++)
		for(j=i;j<size1;j++)
			if (i!=j)
				assoc_AA += in_A(in_part1[i],in_part1[j]);

	num_t assoc_BB = 0;
    for(i=0;i<size2;i++)
		for(j=i;j<size2;j++)
			if (i!=j)
				assoc_BB += in_A(in_part2[i],in_part2[j]);

	num_t assoc_AV = assoc_AA + cut_AB;
	num_t assoc_BV = assoc_BB + cut_AB;

	if (!cut_AB)
			return 0;
	else	return cut_AB/assoc_AV + cut_AB/assoc_BV;

}


/*---------------------------------------------------------------
					exactBisection
  ---------------------------------------------------------------*/
template <class GRAPH_MATRIX, typename num_t>
void CGraphPartitioner<GRAPH_MATRIX,num_t>::exactBisection(
	GRAPH_MATRIX    &in_A,
	vector_uint		&out_part1,
	vector_uint		&out_part2,
	num_t			&out_cut_value,
	bool			forceSimetry )
{
	size_t						nodeCount;		// Nodes count
	size_t		    			i,j;
    GRAPH_MATRIX						Adj;
	vector_bool					partition, bestPartition;
	vector_uint					part1,part2;
	num_t						partCutValue, bestCutValue = std::numeric_limits<num_t>::max();
	bool						end = false;

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
		i = 0;
		bool carry = false;
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

} // end NS
} // end NS

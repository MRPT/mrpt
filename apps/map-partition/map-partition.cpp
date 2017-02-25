/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
    APPLICATION: Map partitioning based on spectral graph
    FILE: map-partition.cpp
    AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	See README.txt for instructions.
  ---------------------------------------------------------------*/

#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace std;

// The ".simplemap" file to process.
string MAP_FILE;

// The command-line passed threshold
float  THRESHOLD_NCUT = -1;


// ------------------------------------------------------
//		Test of maps partitioning
//   (Method in the ICRA06 paper)
// ------------------------------------------------------
void Test()
{
	CSimpleMap		in_map, out_map;
	CTicTac						tictac;
	CIncrementalMapPartitioner	imp;
	std::vector<vector_uint>	parts;

	deleteFilesInDirectory("./MAP-PARTITION_RESULTS");
	createDirectory("./MAP-PARTITION_RESULTS");
	printf("Output files will be saved in directory ./MAP-PARTITION_RESULTS\n");

	// Options:
	imp.options.partitionThreshold			 = 0.90f;
	imp.options.gridResolution				 = 0.10f;
	imp.options.minDistForCorrespondence	 = 0.20f;
	imp.options.minMahaDistForCorrespondence = 10.00f;
	imp.options.useMapMatching               = true;

	if (fileExists("./MAP-PARTITION_CONFIG.ini"))
	{
	    cout << "Loading params from 'MAP-PARTITION_CONFIG.ini'" << endl;
        CConfigFile  cfg("./MAP-PARTITION_CONFIG.ini");
        imp.options.loadFromConfigFile( cfg, "PARAMS" );
	}
	else
	{
	    cout << "Warning: 'MAP-PARTITION_CONFIG.ini' not found. Using default parameters" << endl;
	}

	if (THRESHOLD_NCUT>=0)
		imp.options.partitionThreshold = THRESHOLD_NCUT;

	imp.options.dumpToConsole();

	// Load map from the input file:
	printf("Loading input map:\n%s\n...",MAP_FILE.c_str());
	CFileGZInputStream(MAP_FILE) >> in_map;
	printf("Ok\n");

	// Execute the method:
	printf("\nExecuting the method...\n");

	tictac.Tic();

	const size_t n = in_map.size();
	for (size_t i=0;i<n;i++)
	{
		CSensoryFramePtr sf;
		CPose3DPDFPtr posePDF;

		in_map.get(i,posePDF, sf);

		imp.addMapFrame( sf, posePDF );

		printf("[%u/%u]...",(unsigned int)i,(unsigned int)n);

//		if ((i%1)==0)
		if (i==n-1)
		{
			imp.updatePartitions( parts );
			printf("Map nodes:%u --- %u partitions:\n", (unsigned int)i+1,(unsigned int)parts.size() );
			for (size_t j=0;j<parts.size();j++)
			{
				printf("  Part#%u=", (unsigned int)j);
				for (size_t k=0;k<parts[j].size();k++)
					printf(" %u",parts[j][k]);
				printf("\n");
			}
		}

		//printf("\n");
	}

	printf("Done! in %.03fms\n", (float)(1000*tictac.Tac()) );

	// Save in different maps:
	// ------------------------------------
	FILE	*f=os::fopen( format("MAP-PARTITION_RESULTS/out_partitions_%s.txt", extractFileName(MAP_FILE).c_str() ).c_str() ,"wt");
	for (size_t i=0;i<parts.size();i++)
	{
		for (size_t j=0;j<parts[i].size();j++)
			fprintf(f,"%u ",parts[i][j]);

		fprintf(f,"\n");
	}
	fclose(f);

	printf("Saving output maps...");

	for (size_t i=0;i<parts.size();i++)
	{
		out_map.clear();
		for (size_t j=0;j<parts[i].size();j++)
		{
			CSensoryFramePtr	sf;
			CPose3DPDFPtr		posePDF;

			in_map.get(parts[i][j],posePDF, sf);

			out_map.insert(posePDF, sf);
		}

		CFileOutputStream( format("MAP-PARTITION_RESULTS/out_part#%03u.simplemap",(unsigned)i) ) << out_map;
	}

	printf("Ok\n");

	CMatrixDouble A;
	imp.getAdjacencyMatrix( A);
	A.saveToTextFile( "MAP-PARTITION_RESULTS/matrix_A.txt", MATRIX_FORMAT_FIXED);


	// ------------------------------------------------------------------
	//  Compute the rearranged matrix:
	// ------------------------------------------------------------------

	std::sort( parts.begin(), parts.end() );

	CMatrix			B( A.getRowCount(), A.getColCount() );
	vector_uint		rearrIndexes;
	vector_uint		separations;
	for (size_t i=0;i<parts.size();i++)
	{
		uint32_t  maxIdx = 0;
		for (size_t j=0;j<parts[i].size();j++)
		{
			maxIdx = max(maxIdx, parts[i][j]);
			rearrIndexes.push_back( parts[i][j] );
		}
		separations.push_back( (unsigned int)rearrIndexes.size() );
	}

	for (size_t col=0;col<rearrIndexes.size();col++)
		for (size_t row=0;row<rearrIndexes.size();row++)
			B( row,col ) = A(rearrIndexes[row],rearrIndexes[col] );


	B.saveToTextFile( "MAP-PARTITION_RESULTS/matrix_B.txt" , MATRIX_FORMAT_FIXED);
	{
		gui::CDisplayWindow		win("Adjacency matrix");
		gui::CDisplayWindow		win2(" Rearranged adjacency matrix");
		CImage	img( A, true /* normalized in range [0,1] */ );
		CImage	img2( B, true /* normalized in range [0,1] */ );
		img.saveToFile("MAP-PARTITION_RESULTS/ADJ_MATRIX_BEFORE.png");
		img2.saveToFile("MAP-PARTITION_RESULTS/ADJ_MATRIX_AFTER.png");
		win.showImage( img );
		win2.showImage( img2 );
		win.setPos(20,20);
		win2.setPos((int)(40+A.getColCount()),20);
		cout << "Press any key to continue..." << endl;
		win2.waitForKey();
	}

	// The matlab script below will need "globalmap_grid.png":
	{
		COccupancyGridMap2D  gridmap(-10,10, -10,10, 0.05f);
		cout << "Building global gridmap needed by MATLAB script..."; cout.flush();
		gridmap.loadFromSimpleMap(in_map);
		gridmap.saveMetricMapRepresentationToFile("MAP-PARTITION_RESULTS/globalmap_grid");
		cout << "Done.\n";
	}

	// ------------------------------------------------------------------
	//			Generate MATLAB script
	// ------------------------------------------------------------------
	const std::string matlab_script_filename = "MAP-PARTITION_RESULTS/seePartitionResults.m";
	printf("Generating MATLAB script for visualizing results: %s ...",matlab_script_filename.c_str() );

	f=os::fopen(matlab_script_filename.c_str(),"wt");

	fprintf(f,
		"function []=seePartitionResults()\n"
		"%% Script generated automatically by map-partition - Part of MRPT\n\n"
		"R=0.20; %% Radius of robot shape\n"
		"STEP=4;\n"
		"figure(1); hold on;\n"
		"\nim2=imread('globalmap_grid.png');M = size(im2,1); MM = size(im2,2);\n"
		"for i=1:M,im(i,:)=im2(M-i+1,:);end;\n"
		"D=load('globalmap_grid_limits.txt');\n"
		"imshow(im,'XData',linspace(D(1),D(2),MM),'YData',linspace(D(3),D(4),M));\n"
		"set(gca,'YDir','normal');\n\n"
		"set(gca,'Position',[0 0 1 1]);\n"
		);


	for (size_t i=0;i<parts.size();i++)
	{
		fprintf(f,"%% Partition #%03i\n", (unsigned int)i);
		fprintf(f,"poses=[...\n");

		CPose2D		meanPose;
		for (size_t j=0;j<parts[i].size();j++)
		{
			CSensoryFramePtr	sf;
			CPose3DPDFPtr		posePDF;

			// Get the pose:
			in_map.get(parts[i][j],posePDF, sf);

			meanPose = CPose2D( posePDF->getMeanVal());

			fprintf(f,"       %.03f,%.03f,%.03f",meanPose.x(), meanPose.y(), meanPose.phi() );

			if (j==(parts[i].size()-1))
					fprintf(f,"];\n");
			else	fprintf(f,";...\n");
		}
		fprintf(f,"\n");
		fprintf(f,"figure(1); hold on;\n");
		char	color='k';
		switch (i % 4)
		{
		case 0: color = 'k'; break;
		case 1: color = 'r'; break;
		case 2: color = 'b'; break;
		case 3: color = 'g'; break;
		}
		fprintf(f,"drawRobot(poses(1:STEP:end,:),R,'%c');\n", color);
	}
	fprintf(f,"axis equal;axis tight;\n");

	fprintf(f,"A=load('matrix_A.txt'); B=load('matrix_B.txt');\n");
	fprintf(f,"\n figure(2); subplot(121); imagesc(A); axis equal;colormap(gray);axis([1 length(A) 1 length(A)]);\n");
	fprintf(f,"\n subplot(122);  imagesc(B); axis equal;hold on;colormap(gray);axis([1 length(A) 1 length(A)]);\n");

	// Draw separation lines:
	const size_t N = rearrIndexes.size();
	for (size_t i=0;i<(separations.size()-1);i++)
	{
		fprintf(f,"plot([%i %i],[%i %i],'k');\n", 0, (unsigned int)N-1, separations[i],separations[i] );
		fprintf(f,"plot([%i %i],[%i %i],'k');\n", separations[i],separations[i], 0, (unsigned int)N-1 );
	}

	// Add the auxiliary function: drawRobot()
	fprintf(f,
		"\n"
		"end %% of the main function\n"
		"\n"
		"function [] = drawRobot(poses, radius, style)\n"
		"%%  function [] = drawRobot(poses, radius, style)\n"
		"%%   poses = Nx3, each row=(X,Y,PHI)\n"
		"\n"
		"n = size(poses,1);\n"
		"if (size(poses,2)~=3),\n"
		"    error('Poses must be a Nx3 matrix!');\n"
		"end\n"
		"p = linspace(0,2*pi,100);\n"
		"x = radius*cos(p);\n"
		"y = radius*sin(p);\n"
		"for i=1:n,\n"
		"    plot( poses(i,1)+x, poses(i,2)+y, style );\n"
		"    if (i==1),\n"
		"        hold on;\n"
		"    end;\n"
		"    plot( [poses(i,1) poses(i,1)+radius*cos(poses(i,3))], [poses(i,2) poses(i,2)+radius*sin(poses(i,3))], style );\n"
		"end\n"
		"end  %% end of function\n"
		"\n"
		);

	fclose(f);
	printf("Ok\n");
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		printf(" map-partition version 0.2 - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf("-------------------------------------------------------------------\n");

		// Process arguments:
		if (argc<2)
		{
			printf("Use: map-partition <file.simplemap> [nCut threshold 0-2]\n");
			mrpt::system::pause();
			return -1;
		}

		MAP_FILE = std::string( argv[1] );

		if (argc==3)
		{
			bool err = false;

			if (!sscanf(argv[2],"%f",&THRESHOLD_NCUT) )
				err = true;
			else
				if (THRESHOLD_NCUT<0 || THRESHOLD_NCUT>2)
					err = true;

			if (err)
			{
				printf("Use: map-partition <file.simplemap> [nCut threshold 0-2]\n");
				printf("\n\n Invalid nCut threshold: '%s'\n",argv[2]);
				mrpt::system::pause();
				return -1;
			}
		}


		Test();

		return 0;
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl << "Program finished for an exception!!" << std::endl;
		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		std::cerr << "Untyped exception!!" << std::endl;
		mrpt::system::pause();
		return -1;
	}
}


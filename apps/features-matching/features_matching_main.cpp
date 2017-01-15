/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/utils/metaprogramming.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/system/threads.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::vision;
using namespace mrpt;
using namespace std;

#include "../common/sample_image1.h"
#include "../common/sample_image2.h"

string file1,file2;

bool DemoFeatures()
{
	// Ask for the pair of images:
	cout << "Note: On all questions, [enter] means taking the default value" << endl << endl;

	if (file1.empty())
	{
		cout << "Enter path to image #1 [sample_image1]: ";
		std::getline(cin,file1);
	}
	else cout << "Image #1: " << file1 << endl;

	if (file2.empty())
	{
		cout << "Enter path to image #2 [sample_image2]: ";
		std::getline(cin,file2);
	}
	else cout << "Image #2: " << file2 << endl;

	// --------------------------------------
	// Ask the user for the feature method
	// --------------------------------------
	mrpt::vision::CFeatureExtraction	fext;

	cout << endl
		<< "Detectors:\n"
		"0: KLT\n"
		"1: Harris\n"
		"2: BCD\n"
		"3: SIFT\n"
		"4: SURF\n"
		"6: FAST\n"
		"7: FASTER-9\n"
		"8: FASTER-10\n"
		"9: FASTER-12\n";

	cout << endl << "Select the number for the desired method [3: SIFT]:";

	string sel_method;
	std::getline(cin,sel_method);

	if (sel_method.empty())
	{
		fext.options.featsType = featSIFT;
	}
	else
	{
		fext.options.featsType = TFeatureType( atoi( sel_method.c_str() ) );
	}

	// Compute descriptors:
	TDescriptorType	desc_to_compute = TDescriptorType(-1);


	if ( fext.options.featsType!=featSIFT &&
		 fext.options.featsType!=featSURF )
	{
		cout << endl << "Descriptors:" << endl;
		cout << "-1: None" << endl;
		cout << "0: Patch correlation" << endl;
		cout << "1: SIFT" << endl;
		cout << "2: SURF" << endl;
		cout << "4: Intensity-domain spin image descriptors" << endl;
		cout << "8: Polar image descriptor" << endl;
		cout << "16: Log-Polar image descriptor" << endl;

		cout << endl << "Select the number for the desired method [1: SIFT]:";

		string sel_method;
		std::getline(cin,sel_method);

		if (sel_method.empty())
		{
			desc_to_compute = descSIFT;
		}
		else
		{
			desc_to_compute = TDescriptorType( atoi( sel_method.c_str() ) );
		}
	}

	// Max. num of features:
	cout << endl << "Maximum number of features [150 (default), 0: Infinite]:";
	string sel_num_feats;
	std::getline(cin,sel_num_feats);

	// Max # of features
	const size_t  nFeats =  sel_num_feats.empty() ? int(150) : int(::atoi( sel_num_feats.c_str() ));

	CImage img1,img2;

	if (!file1.empty())
	{
		if (!img1.loadFromFile(file1))
			THROW_EXCEPTION_CUSTOM_MSG1("Error loading file: %s",file1.c_str())
	}
	else
	{
		CMemoryStream buf;
		buf.assignMemoryNotOwn(sample_image1,sizeof(sample_image1));
		buf >> img1;
	}

	if (!file2.empty())
	{
		if (!img2.loadFromFile(file2))
			THROW_EXCEPTION_CUSTOM_MSG1("Error loading file: %s",file2.c_str())
	}
	else
	{
		CMemoryStream buf;
		buf.assignMemoryNotOwn(sample_image2,sizeof(sample_image2));
		buf >> img2;
	}

	// img2.rotateImage(DEG2RAD(20),img2.getWidth()/2,img2.getHeight()/2);

	// Only extract patchs if we are using it: descAny means take the patch:
	if (desc_to_compute != descAny )
		fext.options.patchSize = 0; // Do not extract patch:

	CFeatureList	feats1,feats2;

	CTicTac tictac;


	cout << "Detecting features in image1..."; tictac.Tic();
	fext.detectFeatures(img1,feats1, 0,nFeats );
	cout << tictac.Tac() * 1000 << " ms (" << feats1.size() << " features)\n";

	cout << "Detecting features in image2..."; tictac.Tic();
	fext.detectFeatures(img2,feats2, 0, nFeats);
	cout << tictac.Tac() * 1000 << " ms (" << feats2.size() << " features)\n";

	if (desc_to_compute != TDescriptorType(-1) &&
		desc_to_compute != descAny )
	{
		const size_t N_TIMES = 1;
		//const size_t N_TIMES = 10;

		cout << "Extracting descriptors from image 1..."; tictac.Tic();
		for (size_t timer_loop=0;timer_loop<N_TIMES;timer_loop++)
			fext.computeDescriptors(img1,feats1, desc_to_compute);
		cout << tictac.Tac() * 1000.0 / N_TIMES << " ms" << endl;

		cout << "Extracting descriptors from image 2..."; tictac.Tic();
		for (size_t timer_loop=0;timer_loop<N_TIMES;timer_loop++)
			fext.computeDescriptors(img2,feats2, desc_to_compute);
		cout << tictac.Tac() * 1000.0 / N_TIMES << " ms" << endl;
	}


	CDisplayWindow	win1("Image1"), win2("Image2");

	win1.setPos(10,10);
	win1.showImageAndPoints(img1,feats1, TColor::blue);

	win2.setPos(20+img1.getWidth(),10);
	win2.showImageAndPoints(img2,feats2, TColor::blue);


	cout << "Showing all the features" << endl;
	cout << "Press any key on windows 1 or the console to continue..." << endl;
	win1.waitForKey();

	CDisplayWindowPlots	winPlots("Distance between descriptors");
	winPlots.setPos(10,70+img1.getHeight());
	winPlots.resize(500,200);

	// Another window to show the descriptors themselves:
	CDisplayWindowPtr		winptr2D_descr1,winptr2D_descr2;
	CDisplayWindowPlotsPtr	winptrPlot_descr1,winptrPlot_descr2;

	if (fext.options.featsType == featSIFT )
		desc_to_compute = descSIFT;
	else if (fext.options.featsType == featSURF )
		desc_to_compute = descSURF;

	switch (desc_to_compute)
	{
	case descAny:// Patch
	case descPolarImages:
	case descLogPolarImages:
	case descSpinImages:
		{
			winptr2D_descr1 = CDisplayWindow::Create("Descriptor 1");
			winptr2D_descr1->setPos(550,70+img1.getHeight());
			winptr2D_descr1->resize(220,200);

			winptr2D_descr2 = CDisplayWindow::Create("Descriptor 2");
			winptr2D_descr2->setPos(760,70+img1.getHeight());
			winptr2D_descr2->resize(220,200);
		}
		break;
	case descSIFT:
	case descSURF:
		{
			winptrPlot_descr1 = CDisplayWindowPlots::Create("Descriptor 1");
			winptrPlot_descr1->setPos(550,70+img1.getHeight());
			winptrPlot_descr1->resize(220,200);

			winptrPlot_descr2 = CDisplayWindowPlots::Create("Descriptor 2");
			winptrPlot_descr2->setPos(760,70+img1.getHeight());
			winptrPlot_descr2->resize(220,200);
		}
		break;
   default:
      {
         cerr << "Descriptor specified is not handled yet" << endl;
      }
      break;
	}

	CImage img1_show, img2_show, img2_show_base;

	img1_show.selectTextFont("6x13");
	img2_show.selectTextFont("6x13");
	img2_show_base.selectTextFont("6x13");

	// Show features distances:
	for (unsigned int i1 = 0; i1<feats1.size() && winPlots.isOpen() && win1.isOpen() && win2.isOpen() ;i1++)
	{
		// Compute distances:
		CVectorDouble distances(feats2.size());

		tictac.Tic();
		if (desc_to_compute!=descAny)
		{
			// Ignore rotations
			// feats1[i1]->descriptors.polarImgsNoRotation = true;

			for (unsigned int i2 = 0; i2<feats2.size();i2++)
				distances[i2] = feats1[i1]->descriptorDistanceTo( *feats2[i2] );
		}
		else
		{
			for (unsigned int i2 = 0; i2<feats2.size();i2++)
				distances[i2] = feats1[i1]->patchCorrelationTo( *feats2[i2] );
		}
		cout << "All distances computed in " << 1000.0*tictac.Tac() << " ms" << endl;

		// Show Distances;
		winPlots.plot(distances,".4k","all_dists");

		double min_dist=0,max_dist=0;
		size_t min_dist_idx=0, max_dist_idx=0;
		distances.minimum_maximum(min_dist,max_dist,&min_dist_idx,&max_dist_idx);

		const double dist_std = mrpt::math::stddev(distances);

		cout << "Min. distance=" << min_dist << " for img2 feat #" << min_dist_idx << " .Distances sigma: " << dist_std << endl;

		winPlots.axis(-15,distances.size(),-0.15*max_dist,max_dist*1.15);
		winPlots.plot( CVectorDouble(1,(double)min_dist_idx), CVectorDouble(1,min_dist) ,".8b","best_dists");

		winPlots.setWindowTitle(format("Distances feat #%u -> all others ",i1));


		// Display the current descriptor in its window and the best descriptor from the other image:
		switch(desc_to_compute)
		{
		case descAny: // Patch
		case descPolarImages:
		case descLogPolarImages:
		case descSpinImages:
			{
				CImage auxImg1,auxImg2;
				if (desc_to_compute==descAny) {
					auxImg1 = feats1[i1]->patch; auxImg2 = feats2[min_dist_idx]->patch;
				}
				else if (desc_to_compute==descPolarImages) {
					auxImg1.setFromMatrix( feats1[i1]->descriptors.PolarImg );
					auxImg2.setFromMatrix( feats2[min_dist_idx]->descriptors.PolarImg );
				}else if (desc_to_compute==descLogPolarImages) {
					auxImg1.setFromMatrix( feats1[i1]->descriptors.LogPolarImg );
					auxImg2.setFromMatrix( feats2[min_dist_idx]->descriptors.LogPolarImg );
				}else if (desc_to_compute==descSpinImages)
				{
					{
						const size_t nR = feats1[i1]->descriptors.SpinImg_range_rows;
						const size_t nC = feats1[i1]->descriptors.SpinImg.size()/feats1[i1]->descriptors.SpinImg_range_rows;
						CMatrixFloat M1(nR,nC);
						for (size_t r=0;r<nR;r++)
							for (size_t c=0;c<nC;c++)
								M1(r,c)=feats1[i1]->descriptors.SpinImg[c+r*nC];
						auxImg1.setFromMatrix( M1 );
					}
					{
						const size_t nR = feats2[min_dist_idx]->descriptors.SpinImg_range_rows;
						const size_t nC = feats2[min_dist_idx]->descriptors.SpinImg.size()/feats2[min_dist_idx]->descriptors.SpinImg_range_rows;
						CMatrixFloat M2(nR,nC);
						for (size_t r=0;r<nR;r++)
							for (size_t c=0;c<nC;c++)
								M2(r,c)=feats2[min_dist_idx]->descriptors.SpinImg[c+r*nC];
						auxImg2.setFromMatrix( M2 );
					}
				}

				while (auxImg1.getWidth()<100 && auxImg1.getHeight()<100)
					auxImg1.scaleImage(auxImg1.getWidth()*2,auxImg1.getHeight()*2,IMG_INTERP_NN);
				while (auxImg2.getWidth()<100 && auxImg2.getHeight()<100)
					auxImg2.scaleImage(auxImg2.getWidth()*2,auxImg2.getHeight()*2,IMG_INTERP_NN);
				winptr2D_descr1->showImage( auxImg1 );
				winptr2D_descr2->showImage( auxImg2 );
			}
			break;
			case descSIFT:
				{
					vector<float> v1, v2;
					mrpt::utils::metaprogramming::copy_container_typecasting(feats1[i1]->descriptors.SIFT, v1);
					mrpt::utils::metaprogramming::copy_container_typecasting(feats2[min_dist_idx]->descriptors.SIFT, v2);
					winptrPlot_descr1->plot( v1 );
					winptrPlot_descr2->plot( v2 );
					winptrPlot_descr1->axis_fit();
					winptrPlot_descr2->axis_fit();
				}
				break;
			case descSURF:
				{
					winptrPlot_descr1->plot( feats1[i1]->descriptors.SURF );
					winptrPlot_descr2->plot( feats2[min_dist_idx]->descriptors.SURF);
					winptrPlot_descr1->axis_fit();
					winptrPlot_descr2->axis_fit();
				}
				break;
         default:
            {
               cerr << "Descriptor specified is not handled yet" << endl;
            }
            break;
		}


		// win2: Show only best matches:

		//CFeatureList  feats2_best;
		img2_show_base = img2;

		CVectorDouble xs_best,ys_best;
		for (unsigned int i2 = 0; i2<feats2.size();i2++)
		{
			if (distances[i2]< min_dist + 0.1*dist_std )
			{
				img2_show_base.cross(feats2[i2]->x,feats2[i2]->y, TColor::red,'+',7 );
				//img2_show.drawCircle(feats2[i2]->x,feats2[i2]->y,7, TColor::blue );

				img2_show_base.textOut(feats2[i2]->x+10,feats2[i2]->y-10,format("#%u, dist=%.02f",i2,distances[i2]), TColor::gray);

				xs_best.push_back(i2);
				ys_best.push_back(distances[i2]);
			}
			else
			{
				img2_show_base.cross(feats2[i2]->x,feats2[i2]->y, TColor::gray,'+',3);
			}
		}

		winPlots.plot( xs_best, ys_best,".4b","best_dists2");

		// Show new images in win1 / win2, but with a catchy animation to focus on the features:
		// ------------------------------------------------------------------------------------------
		// win1: Show only the current feature:
		for (unsigned anim_loops = 36;anim_loops>0;anim_loops-=2)
		{
			img1_show = img1;

			img1_show.cross(feats1[i1]->x,feats1[i1]->y, TColor::red,'+',7 );
			img1_show.drawCircle(feats1[i1]->x,feats1[i1]->y,7+anim_loops, TColor::blue );

			img2_show = img2_show_base;
			for (unsigned int i2 = 0; i2<feats2.size();i2++)
			{
				if (distances[i2]< min_dist + 0.1*dist_std )
				{
					img2_show.drawCircle(feats2[i2]->x,feats2[i2]->y,7+anim_loops, TColor::blue );
				}
			}

			win1.showImage(img1_show);
			win2.showImage(img2_show);

			mrpt::system::sleep(10);
		}

		// Wait for the next iteration:
		cout << "Press any key on the distances window or on the console to continue (close any window to exit)..." << endl;
		winPlots.waitForKey();
	}


    return false;
}


int main(int argc, char **argv)
{
	try
	{
		if (argc!=1 && argc!=3)
		{
			cerr << "Usage: " << endl;
			cerr << argv[0] << endl;
			cerr << argv[0] << " <image1> <image2>" << endl;
			return 1;
		}

		if (argc==3)
		{
			file1 = string(argv[1]);
			file2 = string(argv[2]);
		}

		DemoFeatures();
		return 0;
	}
	catch(exception &e)
	{
		cerr << e.what();
		return 1;
	}
}

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/config.h>

#if MRPT_HAS_OPENCV
	#define CV_NO_CVV_IMAGE   // Avoid CImage name crash
	#include <cv.h>
	#include <highgui.h>

	#ifdef CImage	// For old OpenCV versions (<=1.0.0)
	#undef CImage
	#endif
#endif

#ifdef _CH_
#pragma package <opencv>
#endif

#define CV_NO_BACKWARD_COMPATIBILITY

#ifndef _EiC
#include <stdio.h>
#include <ctype.h>
#endif

#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/hwdrivers.h>
#include <mrpt/vision.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace std;

typedef vector_uint patch_ind;
typedef vector<patch_ind> indices_list;
typedef vector_byte triangle;
typedef vector<triangle> triangle_list;


typedef std::vector<CvPoint2D32f> CvPoint2D32fVector;




#define cvSet1 cvSetReal1D
#define cvGet1 cvGetReal1D

#define cvSet2 cvSetReal2D
#define cvGet2 cvGetReal2D


//Sequence has a pointer to an image, and a list of triangle-patches, a list of patch-indices and a mask that contain all triangles.
struct img_seq_data {
	img_seq_data() : img(NULL), mask(NULL)
	{
	}

	triangle_list tri_list;
	indices_list ind_list;
	IplImage *img;
	CvMat *mask;
};
vector<img_seq_data> data_img;
CDisplayWindow   win("Reference"), win1("Video");//, win2("Patch-Tform"), win3("Patch_LM"); //win4("AUX");
CDisplayWindow   win2("Pruebas");
unsigned int f, t;
//IplImage *current;
CArrayDouble<6>	C;
CImage img_aux, imgColor(1,1,3);

std::vector<CArrayDouble<3> >  po1, po2, po3;
CArrayDouble<3> v_init1, v_init2, v_init3;
unsigned int v1, v2, v3;
CvPoint2D32f* center = 0;
//vector_double po1(3), po2(3), po3(3);

//-------------------------------------
//|		NCC-VECTORPATCH FUNCTION      |
//-------------------------------------

//Normalised Cross Correlation between two vector patches
//The Matlab code for that:
//a = a - mean(a);
//b = b - mean(b);
//r = sum(a.*b)/sqrt(sum(a.*a)*sum(b.*b));
//
//double ncc_vector( const vector_byte &patch1, const vector_byte &patch2 )
//{
//	ASSERT_( patch1.size()==patch2.size() );
//
//	double numerator = 0, sum_a = 0, sum_b = 0, result, a_mean, b_mean;
//	a_mean = patch1.mean();
//	b_mean = patch2.mean();
//
//	for(unsigned int i=0; i<patch1.size(); i++)
//	{
//		numerator = numerator + (patch1[i]-a_mean)*(patch2[i]-b_mean);
//		sum_a = sum_a + square(patch1[i]-a_mean);
//		sum_b = sum_b + square(patch2[i]-b_mean);
//	}
//	result=numerator/sqrt(sum_a*sum_b);
//	return result;
//}



//--------------------------------
//|		   COST FUNCTION          |
//---------------------------------
inline size_t floorS(double d)	{
	return static_cast<size_t>(floor(d));
}


//typedef CLevenbergMarquardtTempl<vector_double,vector<img_seq_data> > CMyLevMarq;
typedef CLevenbergMarquardtTempl<vector_double, img_seq_data > CMyLevMarq;


void CostFunction(const vector_double &X, const img_seq_data &last, vector_double &cost)
{
	CMatrixDouble33 tform;
	vector_byte patch2;
	vector_word xy1(3);  xy1[2]=1;
	vector_double xy2(3);
	vector_float xx(3), yy(3);
	unsigned int x, y, i;
	vector_double pt1(3), pt2(3), pt3(3);
	float d1_2, d2_2, d3_2;
	//CImage last_img;
	bool inside_img = true;
	CTicTac	tictac;

	try
	{
		tictac.Tic();
		cost.resize(1);
		//Most generic case (8 parameters)
		tform.set_unsafe(0,0,X[0]); tform.set_unsafe(0,1,X[1]); tform.set_unsafe(0,2,X[2]);
		tform.set_unsafe(1,0,X[3]); tform.set_unsafe(1,1,X[4]); tform.set_unsafe(1,2,X[5]);
		tform.set_unsafe(2,0,X[6]); tform.set_unsafe(2,1,X[7]); tform.set_unsafe(2,2,1);

		tform.multiply_Ab(po1[t],pt1);
		pt1[0] = pt1[0]/pt1[2]; pt1[1] = pt1[1]/pt1[2];
		tform.multiply_Ab(po2[t],pt2);
		pt2[0] = pt2[0]/pt2[2]; pt2[1] = pt2[1]/pt2[2];
		tform.multiply_Ab(po3[t],pt3);
		pt3[0] = pt3[0]/pt3[2]; pt3[1] = pt3[1]/pt3[2];

		xx[0] = pt1[0]; xx[1] = pt2[0]; xx[2] = pt3[0];
		yy[0] = pt1[1]; yy[1] = pt2[1]; yy[2] = pt3[1];

			//Show Image and points
		//win3.showImageAndPoints( last.img, xx, yy);
		//mrpt::system::pause();
		d1_2 = square(pt1[0]-C[0]) + square(pt1[1]-C[1]);
		d2_2 = square(pt2[0]-C[2]) + square(pt2[1]-C[3]);
		d3_2 = square(pt3[0]-C[4]) + square(pt3[1]-C[5]);

		if( d1_2>400 || d2_2>400 || d3_2>400 ){
			cost[0] = 2;
			cout<< tictac.Tac() <<" segs (CostFunction)." <<endl;
			return;
		}

		//Load Patch
		for ( i=0; i<data_img[0].tri_list[t].size(); i++)
		{
			x = data_img[0].ind_list[t][i]%data_img[0].img->widthStep + 1;
			y = data_img[0].ind_list[t][i]/data_img[0].img->widthStep + 1;
			xy1[0] = x;	xy1[1] = y;
			tform.multiply_Ab(xy1,xy2);		//xy2 = xy1 * tform;
			xy2[0]/=xy2[2]; xy2[1]/=xy2[2]; //xy2[2]/=xy2[2];			 //SET THE LAST ELEMENT TO 1.
			if ( xy2[0]>(data_img[0].img->width-2) || xy2[0]<1 || xy2[0]>(data_img[0].img->height-2) || xy2[0]<1 ){
				inside_img = false;
				break;
			}

			patch2.resize(data_img[0].tri_list[t].size());
			patch2[i] = round((unsigned char)(last.img->imageData[floorS(xy2[0]-1)+(floorS(xy2[1])-1)*data_img[0].img->widthStep])*(floorS(xy2[0])+1-xy2[0])*(floorS(xy2[1])+1-xy2[1]) +
                     (unsigned char)(last.img->imageData[floorS(xy2[0]-1)+floorS(xy2[1])*data_img[0].img->widthStep])*(floorS(xy2[0])+1-xy2[0])*(xy2[1]-floorS(xy2[1])) +
                     (unsigned char)(last.img->imageData[floorS(xy2[0])+(floorS(xy2[1])-1)*data_img[0].img->widthStep])*(xy2[0]-floorS(xy2[0]))*(floorS(xy2[1])+1-xy2[1]) +
                     (unsigned char)(last.img->imageData[floorS(xy2[0])+floorS(xy2[1])*data_img[0].img->widthStep])*(xy2[0]-floorS(xy2[0]))*(xy2[1]-floorS(xy2[1])));
		}
		if(inside_img)
			cost[0] = 1 - ncc_vector(data_img[0].tri_list[t], patch2);
		else
			cost[0] = 2;

		cout<< tictac.Tac() <<" segs (CostFunction)." <<endl;
	}
	catch(std::exception &e)
	{
		std::cout << e.what() << std::endl;
		std::cout << "Cost Function couldn't be evaluated" << endl;
	}
	catch(...)
	{
		std::cout << "Runtime error." << std::endl;
		std::cout << "Cost Function couldn't be evaluated" << endl;
	}
}

//--------------------------------
//|	     NEAREST ELEMENT          |
//---------------------------------
//int nearest(

//--------------------------------
//|	    AREA OF TRIANGLE          |
//---------------------------------

double TriangleArea(double ax, double ay, double bx, double by, double cx, double cy)
{
	double area;
	area = 0.5*sqrt( (square(ax-bx)+square(ay-by))*(square(ax-cx)+square(ay-cy)) - square((ax-bx)*(ax-cx)+(ay-by)*(ay-cy)) );
	return area;
}

double TriangleArea(CvPoint2D32f a, CvPoint2D32f b, CvPoint2D32f c)
{
	double area;
	area = 0.5*sqrt( (square(a.x-b.x)+square(a.y-b.y))*(square(a.x-c.x)+square(a.y-c.y)) - square((a.x-b.x)*(a.x-c.x)+(a.y-b.y)*(a.y-c.y)) );
	return area;
}

//--------------------------------
//|	   POINT IN TRIANGLE          |
//---------------------------------

bool PointInTriangle(CvPoint2D32f p, CvPoint2D32f a, CvPoint2D32f b, CvPoint2D32f c)
{
    double cp1, cp2;
	bool s1, s2, s3;

//  same_side(p,a, b,c)

	cp1 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
    cp2 = (c.x - b.x) * (a.y - b.y) - (c.y - b.y) * (a.x - b.x);

    s1 = (cp1 * cp2) >= 0;

//  same_side(p,b, a,c)
    cp1 = (c.x - a.x) * (p.y - a.y) - (c.y - a.y) * (p.x - a.x);
    cp2 = (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);

    s2 = (cp1 * cp2) >= 0;

//  same_side(p,c, a,b)
    cp1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    cp2 = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);

    s3 = (cp1 * cp2) >= 0;

    return( s1 & s2 & s3 );
}

//--------------------------------
//|			LOAD PATCHES          |
//---------------------------------

	//loadPatches(CvMat *Ptos, CvMat *mesh)

	//Takes a matrix with the image points, and another matrix with the triangle mesh to build a mask,
	//which assign to each position of the image the number of the triangle of the referred mesh in which it is,
	//or 0 in the case that it doesn't belong to the mesh.
	//At the same time, it loads the patch vectors of each triangle. Taking the original image as a global variable.


void loadPatches(CFeatureList &Ptos, CMatrix &mesh, CvMat** mask, triangle_list &tri, indices_list &ind, IplImage *ipl)
{
	*mask = cvCreateMat(ipl->height, ipl->width, CV_16UC1);		//Para cuando mask es un puntero
	cvZero(*mask);

	int idx1, idx2, idx3;
	CvPoint2D32f p1, p2, p3;
	int num_facets = mesh.getRowCount();

	for(int i = 1; i <= num_facets; i++)		//CAMBIAR ESTRUCTURA, A UN UNICO FOR PARA EL RECORRIDO DE LA IMAGEN
	{
		idx1 = mesh(i-1,0)-1;
		idx2 = mesh(i-1,1)-1;
		idx3 = mesh(i-1,2)-1;

		p1 = cvPoint2D32f( Ptos[idx1]->x, Ptos[idx1]->y);
		p2 = cvPoint2D32f( Ptos[idx2]->x, Ptos[idx2]->y);
		p3 = cvPoint2D32f( Ptos[idx3]->x, Ptos[idx3]->y);

		//idx1 = (int)cvGet2D(mesh, (i - 1), 0) - 1;
		//idx2 = (int)cvGet2D(mesh, (i - 1), 1) - 1;
		//idx3 = (int)cvGet2D(mesh, (i - 1), 2) - 1;

		//p1 = cvPoint2D32f( cvGet2D(points, idx1, 0), cvGet2(points, idx1, 1));
		//p2 = cvPoint2D32f( cvGet2D(points, idx2, 0), cvGet2(points, idx2, 1));
		//p3 = cvPoint2D32f( cvGet2D(points, idx3, 0), cvGet2(points, idx3, 1));


		for(int y = 1; y <= ipl->height; y++)
		{
			for(int x = 1; x <= ipl->width; x++)
			{
				if(PointInTriangle(cvPoint2D32f(x, y), p1, p2, p3))
				{
					cvSet2((*mask), (y - 1), (x - 1), (double)i );
					tri[i-1].push_back( ipl->imageData[ x - 1 + (y - 1)*ipl->widthStep ] );
					ind[i-1].push_back( x - 1 + (y - 1)*ipl->widthStep );
				}
			}
		}
	}
}

//--------------------------------
//|			FOLLOW-PATCH          |
//---------------------------------
void followPatch(CMatrixDouble33 &tform, vector_byte &tri, IplImage *ipl)
{
	int x, y;
	vector_double xy1(3), xy2(3);
	tri.resize(data_img[0].tri_list[t].size());
	for(unsigned int i = 0; i < data_img[0].ind_list[t].size(); i++)
	{
		x = data_img[0].ind_list[t][i] % data_img[0].img->widthStep +1;
		y = data_img[0].ind_list[t][i] / data_img[0].img->widthStep +1;
		xy1[0] = x; xy1[1] = y; xy1[2] = 1;
		tform.multiply_Ab(xy1,xy2);
		xy2[0]/=xy2[2]; xy2[1]/=xy2[2]; //xy2[2]/=xy2[2];
		//ind[i] = ( xy2[0] + xy2[1]*data_img[0].img->widthStep );

		tri[i] = round((unsigned char)(ipl->imageData[floorS(xy2[0]-1)+(floorS(xy2[1])-1)*data_img[0].img->widthStep])*(floorS(xy2[0])+1-xy2[0])*(floorS(xy2[1])+1-xy2[1]) +
                     (unsigned char)(ipl->imageData[floorS(xy2[0]-1)+floorS(xy2[1])*data_img[0].img->widthStep])*(floorS(xy2[0])+1-xy2[0])*(xy2[1]-floorS(xy2[1])) +
                     (unsigned char)(ipl->imageData[floorS(xy2[0])+(floorS(xy2[1])-1)*data_img[0].img->widthStep])*(xy2[0]-floorS(xy2[0]))*(floorS(xy2[1])+1-xy2[1]) +
                     (unsigned char)(ipl->imageData[floorS(xy2[0])+floorS(xy2[1])*data_img[0].img->widthStep])*(xy2[0]-floorS(xy2[0]))*(xy2[1]-floorS(xy2[1])));
	}
}

//--------------------------------
//|       EXTRACT FEATURES          |
//---------------------------------
//CFeatureList ExtractFeaturesFrame1(CImage img)
//{
//	//CDisplayWindow   wind("1st Frame");
//	CFeatureExtraction	fExt;
//	CFeatureList features, f_out;
//	//CTicTac	tictac;
//
//	cout << "1ST FRAME LOADED" << endl;
//
//	//tictac.Tic();
//
//	//KLT
//	cout << "Extracting KLT features ... [boot.txt]\n";
//	fExt.options.featsType = featKLT;
//	fExt.options.harrisOptions.tile_image = false;
//	fExt.options.patchSize=0;
//	fExt.detectFeatures( img, features, 0,1000);
//
//
////PRINT THE KLT FEATURES IN THE IMAGE
//	//char* cstr = new char [3];
//	//for(unsigned int i = 0; i < features.size(); i++)
//	//{
//	//	sprintf( cstr, "%d", i );
//	//	img.textOut( features.getByID(i)->x, features.getByID(i)->y, cstr, 255 );
//	//}
//	////wind.showImageAndPoints(img, features);
//	////mrpt::system::pause();
//	//delete cstr;
//
//	unsigned int chosen[5] = { 14, 18, 34, 64, 70 };
//	for(unsigned int i = 0; i < 5; i++)
//		f_out.push_back(features.getByID(chosen[i]));
//
//	//wind.showImageAndPoints(img, f_out);
//	//mrpt::system::pause();
//
//	////	CODE TO GET RECURSIVELY 2 VECTORS WITH THE FEATURES COORDINATES
//	//vector_float x, y;
//	//x.resize(features.size());
//	//y.resize(features.size());
//	//for(unsigned int i = 0; i < x.size(); i++)
//	//{
//	//	x[i] = features.getByID(i)->x;
//	//	y[i] = features.getByID(i)->y;
//	//}
//	//wind.showImageAndPoints(img, x, y);
//
//
//
//
//
//	////HARRIS + SURF
//	//cout << "Extracting Harris-SURF features ... [boot.txt]\n";
//	//fExt.options.featsType = featHarris;
//	//fExt.options.harrisOptions.tile_image = false;
//	//fExt.options.patchSize=0;
//	//fExt.detectFeatures( img, features, 0,1000 );
//	//fExt.computeDescriptors( img, features, descSURF );
//	//wind.showImageAndPoints(img, features);
//	//mrpt::system::pause();
//
//	//char* cstr = new char [3];
//	//for(unsigned int i = 0; i < features.size(); i++)
//	//{
//	//	sprintf( cstr, "%d", i );
//	//	img.textOut( features.getByID(i)->x, features.getByID(i)->y, cstr, 255 );
//	//}
//	//wind.showImageAndPoints(img, features);
//	//mrpt::system::pause();
//	//delete cstr;
//
//	//unsigned int chosen[5] = { 14, 18, 34, 64, 70 };
//	//for(unsigned int i = 0; i < 5; i++)
//	//	f_out.push_back(features.getByID(chosen[i]));
//
//	return f_out;
//}

void detectClusteres(CFeatureList &ptos, CvMat* samples, CvMat* clusters, CvMat* centers, int cluster_count )
{
	for(unsigned int i=0; i<ptos.size(); i++)
	{
		cvSet2D( samples, i, 0, cvScalar(ptos[i]->x) );
		cvSet2D( samples, i, 1, cvScalar(ptos[i]->y) );
	}
	cvKMeans2( samples, cluster_count, clusters, cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0 ) , 1, 0, 0, centers );
}



/*---------------------------------------------------------------
		Aux. functions needed by ransac_homographies
 ---------------------------------------------------------------*/
namespace mrpt
{
	namespace math
	{
		template <typename T>
		void  ransacHomography_fit(
			const CMatrixTemplateNumeric<T> &allData,
			const vector_size_t  &useIndices,
			vector< CMatrixTemplateNumeric<T> > &fitModels )
		{
			ASSERT_(useIndices.size()==3);

			try
			{
				//BUILD AFFINE TRANSFORMATION
				CMatrixDouble66		D;
				CArrayDouble<6>		A;
				double dett;
				CArrayDouble<3> pt1, pt2, pt3;

				pt1[0] = allData(0,useIndices[0]); pt1[1] = allData(1,useIndices[0]); pt1[2] = 1;
				pt2[0] = allData(0,useIndices[1]); pt2[1] = allData(1,useIndices[1]); pt2[2] = 1;
				pt3[0] = allData(0,useIndices[2]); pt3[1] = allData(1,useIndices[2]); pt3[2] = 1;

				D.fill(0);																								//D is the inverse of B, B is the original matrix of pts
				dett = (-pt1[0]*pt3[1]+pt1[0]*pt2[1]+pt3[1]*pt2[0]-pt2[1]*pt3[0]-pt2[0]*pt1[1]+pt3[0]*pt1[1]);			//This is the square root of minus det(B) -> sqtr(-det(B))
				D.set_unsafe(0,0, (-pt3[1]+pt2[1])/dett);
				D.set_unsafe(0,2, -(pt1[1]-pt3[1])/dett);
				D.set_unsafe(0,4, (-pt2[1]+pt1[1])/dett);
				D.set_unsafe(1,0, -(pt2[0]-pt3[0])/dett);
				D.set_unsafe(1,2, (pt1[0]-pt3[0])/dett);
				D.set_unsafe(1,4, -(pt1[0]-pt2[0])/dett);
				D.set_unsafe(2,0, (-pt2[1]*pt3[0]+pt3[1]*pt2[0])/dett);
				D.set_unsafe(2,2, -(pt1[0]*pt3[1]-pt3[0]*pt1[1])/dett);
				D.set_unsafe(2,4, (pt1[0]*pt2[1]-pt2[0]*pt1[1])/dett);
				D.set_unsafe(3,1, (-pt3[1]+pt2[1])/dett);
				D.set_unsafe(3,3, -(pt1[1]-pt3[1])/dett);
				D.set_unsafe(3,5, (-pt2[1]+pt1[1])/dett);
				D.set_unsafe(4,1, -(pt2[0]-pt3[0])/dett);
				D.set_unsafe(4,3, (pt1[0]-pt3[0])/dett);
				D.set_unsafe(4,5, -(pt1[0]-pt2[0])/dett);
				D.set_unsafe(5,1, (-pt2[1]*pt3[0]+pt3[1]*pt2[0])/dett);
				D.set_unsafe(5,3, -(pt1[0]*pt3[1]-pt3[0]*pt1[1])/dett);
				D.set_unsafe(5,5, (pt1[0]*pt2[1]-pt2[0]*pt1[1])/dett);
				//D.saveToTextFile("d:\\D.txt");

				C[0] = allData(2,useIndices[0]);
				C[1] = allData(3,useIndices[0]);
				C[2] = allData(2,useIndices[1]);
				C[3] = allData(3,useIndices[1]);
				C[4] = allData(2,useIndices[2]);
				C[5] = allData(3,useIndices[2]);

				D.multiply_Ab(C,A);

				fitModels.resize(1);
				CMatrixTemplateNumeric<T> &TForm = fitModels[0];		//CMatrixDouble33		TForm;
				TForm.setSize(3,3);

				TForm.set_unsafe(0,0, A[0]);
				TForm.set_unsafe(0,1, A[1]);
				TForm.set_unsafe(0,2, A[2]);
				TForm.set_unsafe(1,0, A[3]);
				TForm.set_unsafe(1,1, A[4]);
				TForm.set_unsafe(1,2, A[5]);
				TForm.set_unsafe(2,0, 0);
				TForm.set_unsafe(2,1, 0);
				TForm.set_unsafe(2,2, 1);
			}
			catch(exception &)
			{
				fitModels.clear();
				return;
			}
		}


		template <typename T>
		void ransacHomography_distance(
			const CMatrixTemplateNumeric<T> &allData,
			const vector< CMatrixTemplateNumeric<T> > & testModels,
			const T distanceThreshold,
			unsigned int & out_bestModelIndex,
			vector_size_t & out_inlierIndices )
		{
			ASSERT_( testModels.size()==1 )
			out_bestModelIndex = 0;
			double d;

			const CMatrixDouble &M = testModels[0];

			ASSERT_( size(M,1)==3 && size(M,2)==3 )

			CArrayDouble<3> pt, pt_r;

			const size_t N = size(allData,2);
			out_inlierIndices.clear();
			out_inlierIndices.reserve(100);
			for (size_t i=0;i<N;i++)
			{
				pt[0] = allData.get_unsafe(0,i);
				pt[1] = allData.get_unsafe(1,i);
				pt[2] = 1;
				testModels[0].multiply_Ab( pt, pt_r );
				d = sqrt( square( pt_r[0] - allData(2,i) ) + square(pt_r[1] - allData(3,i) ) );
				if ( d < distanceThreshold )
					out_inlierIndices.push_back(i);
			}

			//Increase Homography Inliers
			if (out_inlierIndices.size() >= 15 )
			{
				//A)Search and store natural vertices in clockwise direction (greatest and smallest x & y coordinates)
				vector_size_t vertices(4,out_inlierIndices[0]);
				for (size_t i=1;i<out_inlierIndices.size();i++)
				{
					if( allData(3,out_inlierIndices[i]) > allData(3,vertices[0]) )
						vertices[0] = out_inlierIndices[i];
					if( allData(2,out_inlierIndices[i]) > allData(2,vertices[1]) )
						vertices[1] = out_inlierIndices[i];
					if( allData(3,out_inlierIndices[i]) < allData(3,vertices[2]) )
						vertices[2] = out_inlierIndices[i];
					if( allData(2,out_inlierIndices[i]) < allData(2,vertices[3]) )
						vertices[3] = out_inlierIndices[i];
				}

				//std::set<size_t> v;		//Este tampoco me vale porque reordena los indices
				//v.insert( vertices.begin(), vertices.end() );
				////ESTA FORMA DE BORRAR REPETICIONES GENERA ERRORES "ALEATORIOS" -> HAY QUE CAMBIARLA
				//for (vector_size_t::iterator it1=vertices.begin(); it1!=(vertices.end()-1); ++it1)	//Erase repeated vertex		//poly.removeRepeatedVertices();
				//{
				//	for (vector_size_t::iterator it2=it1+1; it2!=vertices.end(); ++it2)
				//	{
				//		if( (*it1)==(*it2) )
				//		{
				//			it1 = vertices.erase(it1);
				//			break;
				//		}
				//	}
				//}
				vector_size_t::iterator it1;
				for (size_t i=0; i<vertices.size()-1; ++i)	//Erase repeated vertex		//poly.removeRepeatedVertices();
				{
					for (size_t j=i+1; j<vertices.size(); ++j)
					{
						if( vertices[i]==vertices[j] )
						{
							it1 = vertices.begin()+j;
							vertices.erase(it1);
							break;
						}
					}
				}


				//Look for inliers out the initial polygon and transform the polygon to contain all
				TPolygon2D poly;
				vector<TPoint2D> vertex;
				TPoint2D point;
				size_t seg_id, point_id;
				double dist, max_dist;			//dist & max_dist refer to the nearest line of the contour
				bool vert;
				vertex.resize( vertices.size() );
				for (size_t i=0;i<vertices.size();i++)
				{
					vertex[i] = TPoint2D( allData(2,vertices[i]), allData(3,vertices[i]) );
				}
				poly = TPolygon2D( vertex );

				////PLOT INITIAL POLYGON
				//img_aux.colorImage( imgColor );	// Create a colorimage
				//for(size_t j=0;j<out_inlierIndices.size(); j++)
				//{
				//	imgColor.cross(round(allData(2,out_inlierIndices[j])),round(allData(3,out_inlierIndices[j])),TColor::blue,'+');
				//	//imgColor.textOut(round(allData(2,out_inlierIndices[j])),round(allData(3,out_inlierIndices[j])), format("%d", out_inlierIndices[j]), 255 );
				//}
				//for(size_t j=0;j<(vertices.size()-1); j++)
				//{
				//	imgColor.line( allData(2,vertices[j]), allData(3,vertices[j]), allData(2,vertices[j+1]), allData(3,vertices[j+1]), 255 );
				//	imgColor.textOut(round(allData(2,vertices[j])),round(allData(3,vertices[j])), format("%d", vertices[j]), 255 );
				//}
				//imgColor.textOut(round(allData(2,vertices[vertices.size()-1])),round(allData(3,vertices[vertices.size()-1])), format("%d", vertices[vertices.size()-1]), 255 );
				//imgColor.textOut(round(allData(2,out_inlierIndices[0])),round(allData(3,out_inlierIndices[0])), format("%d", out_inlierIndices[0]), 255 );
				//imgColor.line( allData(2,vertices[vertices.size()-1] ), allData(3,vertices[vertices.size()-1] ), allData(2,vertices[0]), allData(3,vertices[0]), 255 );
				//win2.showImage(imgColor);
				//mrpt::system::pause();


				do{
					max_dist = 0;
					//Start adding the farest points to the polygon (to yhe polygon boundary)
					std::vector<TSegment2D> sgs;
					poly.getAsSegmentList(sgs);
					//TPoint2D cntr;
					//poly.getCenter(cntr);

					for (size_t i=0;i<out_inlierIndices.size();i++) //NO SE EVALUA NADA MAS K EL 1ER PTO
					{
						vert = false;
						for(size_t j=0;j<vertices.size();j++)
						{
							if( vertices[j] == out_inlierIndices[i] )
								vert = true;
						}

						point = TPoint2D( allData(2,out_inlierIndices[i]), allData(3,out_inlierIndices[i]) );
						if( !poly.contains(point) && !vert )
						{
							size_t j = 1;
							for (vector<TSegment2D>::const_iterator it=sgs.begin();it!=sgs.end();++it)	//NO SE EVALUA NADA MAS K EL 1ER PTO
							{
								TLine2D l = TLine2D(*it);
								dist = -( l.signedDistance(point) );
								if( dist > max_dist+1 )		//modifica el poligono para ptos extremadamente proximos->Evitarlo con if( dist > max_dist+1 )
								{
									max_dist = dist;
									point_id = out_inlierIndices[i];
									seg_id = j;
								}
								j++;
							}
						}
					}

					if( max_dist != 0 )
					{
						vertices.insert( vertices.begin()+seg_id, point_id );
						vertex.insert( vertex.begin()+seg_id, TPoint2D( allData(2,vertices[seg_id]), allData(3,vertices[seg_id]) ) );
						poly = TPolygon2D( vertex );
					}

					//img_aux.colorImage( imgColor );	// Create a colorimage
					//for(size_t j=0;j<out_inlierIndices.size(); j++)
					//	imgColor.cross(round(allData(2,out_inlierIndices[j])),round(allData(3,out_inlierIndices[j])),TColor::blue,'+');
					//for(size_t j=0;j<vertices.size()-1; j++)
					//{
					//	imgColor.line( allData(2,vertices[j]), allData(3,vertices[j]), allData(2,vertices[j+1]), allData(3,vertices[j+1]), 255 );
					//	imgColor.textOut(round(allData(2,vertices[j])),round(allData(3,vertices[j])), format("%d", vertices[j]), 255 );
					//}
					//imgColor.line( allData(2,vertices[vertices.size()-1] ), allData(3,vertices[vertices.size()-1] ), allData(2,vertices[0]), allData(3,vertices[0]), 255 );
					//imgColor.textOut(round(allData(2,vertices[vertices.size()-1])),round(allData(3,vertices[vertices.size()-1])), format("%d", vertices[vertices.size()-1]), 255 );
					//win2.showImage(imgColor);
					//mrpt::system::pause();

				}while( max_dist != 0 );

				img_aux.colorImage( imgColor );	// Create a colorimage
				for(size_t j=0;j<out_inlierIndices.size(); j++)
					imgColor.cross(round(allData(2,out_inlierIndices[j])),round(allData(3,out_inlierIndices[j])),TColor::blue,'+');
				for(size_t j=0;j<vertices.size()-1; j++)
				{
					imgColor.line( allData(2,vertices[j]), allData(3,vertices[j]), allData(2,vertices[j+1]), allData(3,vertices[j+1]), 255 );
					imgColor.textOut(round(allData(2,vertices[j])),round(allData(3,vertices[j])), format("%d", vertices[j]), 255 );
				}
				imgColor.line( allData(2,vertices[vertices.size()-1] ), allData(3,vertices[vertices.size()-1] ), allData(2,vertices[0]), allData(3,vertices[0]), 255 );
				imgColor.textOut(round(allData(2,vertices[vertices.size()-1])),round(allData(3,vertices[vertices.size()-1])), format("%d", vertices[vertices.size()-1]), 255 );
				win2.showImage(imgColor);
				mrpt::system::pause();

				//if( vertices.size()==2 )	//Look for the farest inlier
				//{
				//	TSegment2D sgs = TSegment2D( TPoint2D( allData(2,vertices[0]), allData(3,vertices[0]) ), TPoint2D( allData(2,vertices[1]), allData(3,vertices[1]) ) );
				//	double dist, max_dist = 0;
				//	for (size_t i=0;i<out_inlierIndices.size();i++)
				//	{
				//		if( (i != vertices[0]) && (i != vertices[1]) )
				//		{
				//			dist = sgs.distance( TPoint2D( allData(2,out_inlierIndices[i]), allData(2,out_inlierIndices[i]) ) );
				//			if ( dist > max_dist )
				//			{
				//				max_dist = dist;
				//				vertices.push_back(out_inlierIndices[i]);
				//			}
				//		}
				//	}
				//}


				//Check the rest of points inside the polygon
				size_t k = 0;
				size_t outliers = 0;
				vector_size_t more_inlierIndices;
				dist = 0;					//dist is now used to check how consistent are the rest of inside points with the homography
				for (size_t i=0; i<N; i++)
				{
					point = TPoint2D( allData(2,i), allData(3,i) );
					if( i!=out_inlierIndices[k] )
					{
						if( poly.contains(point) )
						{
							pt[0] = allData.get_unsafe(0,i);
							pt[1] = allData.get_unsafe(1,i);
							pt[2] = 1;
							testModels[0].multiply_Ab( pt, pt_r );
							d = sqrt( square( pt_r[0] - allData(2,i) ) + square(pt_r[1] - allData(3,i) ) );
							if ( d < 3*distanceThreshold )
								more_inlierIndices.push_back(i);
							else
							{
								outliers++;
								dist += d;
							}
						}
					}
					else
					{
						if( k!=out_inlierIndices.size()-1 )
							k++;
					}
				}
				dist /= outliers;
				//if( outliers > 15 || dist > 5*3*distanceThreshold )
				//	out_inlierIndices.clear();
				if( (more_inlierIndices.size()>15) && (outliers < 15) && (dist < 5*3*distanceThreshold) )
					out_inlierIndices.insert( out_inlierIndices.end(), more_inlierIndices.begin(), more_inlierIndices.end() );

			}

		}

		/** Return "true" if the selected points are a degenerate (invalid) case.
		  */
		template <typename T>
		bool ransacHomography_degenerate(
			const CMatrixTemplateNumeric<T> &allData,
			const mrpt::vector_size_t &useIndices )
		{
			//double r_a, r_b, r_c, dist;
			//r_a = ( allData.get_unsafe( 3, useIndices[0] ) - allData.get_unsafe( 3, useIndices[1] ) );
			//r_b = ( allData.get_unsafe( 2, useIndices[1] ) - allData.get_unsafe( 2, useIndices[0] ) );
			//r_c = ( allData.get_unsafe( 3, useIndices[1] )*allData.get_unsafe( 2, useIndices[0] ) - allData.get_unsafe( 3, useIndices[0] )*allData.get_unsafe( 2, useIndices[1] ) );
			//dist = r_a*allData.get_unsafe( 2, useIndices[2] ) + r_b*allData.get_unsafe( 3, useIndices[2] ) + r_c;
			//if ( dist < 3 )
			//	return true;
			//else
				return false;
		}
	} // end namespace
} // end namespace


/*---------------------------------------------------------------
					ransac_homographies
 ---------------------------------------------------------------*/
//template <typename NUMTYPE>
void ransac_homographies(
						CFeatureList				ref,
						CFeatureList				vid,
						vector<CMatrixTemplateNumeric<double> >     &out_detected_planes,
						const double			    threshold,
						const size_t				min_inliers_for_valid_plane,
						vector< vector<size_t> >	&plane_ind)
{
	MRPT_START

	ASSERT_( ref.size()==vid.size() )
	if ( ref.empty() ) return;

	out_detected_planes.clear();

	CMatrixTemplateNumeric<double>	  remainingPoints( 4, ref.size() );
	for(unsigned int i=0; i<ref.size(); i++)
	{
		remainingPoints.set_unsafe(0,i,ref[i]->x);
		remainingPoints.set_unsafe(1,i,ref[i]->y);
		remainingPoints.set_unsafe(2,i,vid[i]->x);
		remainingPoints.set_unsafe(3,i,vid[i]->y);
	}

	// ---------------------------------------------
	// For each plane:
	// ---------------------------------------------
	mrpt::vector_size_t				other_inliers, other_inliers2;
	for (;;)
	{
		mrpt::vector_size_t				this_best_inliers;
		CMatrixTemplateNumeric<double>	this_best_model;

		math::RANSAC_Template<double>::execute(
			remainingPoints,
			ransacHomography_fit,
			ransacHomography_distance,
			ransacHomography_degenerate,
			threshold,
			3,  // Minimum set of points
			this_best_inliers,
			this_best_model,
			true, // Verbose
			0.999  // Prob. of good result
			);

		// Is this plane good enough?
		if (this_best_inliers.size()>=min_inliers_for_valid_plane)
		{
			// Add this plane to the output list:
			out_detected_planes.push_back(this_best_model);
			//plane_ind.push_back(this_best_inliers);

			if ( out_detected_planes.size() == 1 )
			{
				plane_ind.push_back(this_best_inliers);
				other_inliers = this_best_inliers;
			}
			else
			{
				mrpt::vector_size_t	this_best_inliers_r;	//Restored Index of Inliers
				this_best_inliers_r.resize( this_best_inliers.size() );
				unsigned int j = 0;
				for(int i=0; i<this_best_inliers.size(); i++)
				{
					if( j==other_inliers.size() )
						this_best_inliers_r[i] = this_best_inliers[i] + j;
					else if( ( this_best_inliers[i] + j < other_inliers[j] ) )
						this_best_inliers_r[i] = this_best_inliers[i] + j;
					else
					{
						j++;
						i--;
					}
					//if(i==(this_best_inliers.size()-2) )
					//	mrpt::system::pause();
				}
				plane_ind.push_back(this_best_inliers_r);

				//Upload other_inliers
				unsigned int i = 0;
				j = 0;
				other_inliers2.resize( other_inliers.size() + this_best_inliers_r.size() );
				while( (j+i) < other_inliers2.size() )
				{
					if ( j==this_best_inliers_r.size() ){
						other_inliers2[i+j] = other_inliers[i];
						i++;
					}else if (i==other_inliers.size()){
						other_inliers2[i+j] = this_best_inliers_r[j];
						j++;
					}else if ( (this_best_inliers_r[j] < other_inliers[i]) ){
						other_inliers2[i+j] = this_best_inliers_r[j];
						j++;
					}else{
						other_inliers2[i+j] = other_inliers[i];
						i++;
					}
					//if( (j+i)==(other_inliers2.size()-2) )
					//	mrpt::system::pause();
				}
				other_inliers = other_inliers2;
			}

			//SHOW IMAGE AND SET OF INLIERS
			//img_aux.colorImage( imgColor );	// Create a colorimage
			//for(size_t j=0;j<this_best_inliers.size(); j++)
			//	imgColor.cross(round(remainingPoints(2, this_best_inliers[j])),round(remainingPoints(3, this_best_inliers[j])),TColor::blue,'+');
			//win2.showImage(imgColor);
			//mrpt::system::pause();

			// Discard the selected points so they are not used again for finding subsequent planes:
			remainingPoints.removeColumns(this_best_inliers);
		}
		else
		{
			break; // Do not search for more planes.
		}
	}

	MRPT_END
}

CFeatureList ExtractFeaturesFrame1(IplImage *img, CFeatureList &f_out)//, double &average_x, double &average_y, double &std_x, double &std_y)
{
	CFeatureList features;
	//CTicTac	tictac;

	cout << "1ST FRAME LOADED" << endl;

	//tictac.Tic();

	//KLT
	cout << "Extracting KLT features ... \n";
	//fExt.options.featsType = featKLT;
	//fExt.options.harrisOptions.tile_image = false;
	//fExt.options.patchSize=0;
	//fExt.detectFeatures( img, features, 0,1000);



	IplImage *grey = 0, *prev_grey = 0; //, *pyramid = 0, *prev_pyramid = 0, *image = 0
	int win_size = 11;
	const int MAX_COUNT = 1000;
	//std::vector<CvPoint2D32fVector>		points(2);
	//unsigned nFeatures = 5;
	//points[0].resize(nFeatures);
	//points[1].resize(nFeatures);
	//std::vector<char>	status;

	CvPoint2D32f* points[2] = {0,0};//, *swap_points
	char* status = 0;
    points[0] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(points[0][0]));
    points[1] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(points[0][0]));
    status = (char*)cvAlloc(MAX_COUNT);

	grey = cvCreateImage( cvGetSize(img), 8, 1 );
	IplImage* eig = cvCreateImage( cvGetSize(grey), 32, 1 );
    IplImage* temp = cvCreateImage( cvGetSize(grey), 32, 1 );
    double quality = 0.01;
    double min_distance = 10;

    //cvCopy( frame, image, 0 );
    //cvCvtColor( img, grey, CV_BGR2GRAY );
	cvCopy( img, grey, 0 );

	int count = 0;
    count = MAX_COUNT;
    cvGoodFeaturesToTrack( grey, eig, temp, points[1], &count,
                           quality, min_distance, 0, 3, 0, 0.04 );
    cvFindCornerSubPix( grey, &points[1][0], count,
        cvSize(win_size,win_size), cvSize(-1,-1),
        cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
    cvReleaseImage( &eig );
    cvReleaseImage( &temp );


	vector_double xx, yy;
	xx.resize(count); yy.resize(count);
	// Array conversion OpenCV->MRPT
	CFeatureList::iterator		itFeat;
	for( int i = 0; i < count ; i++ )
	{
		CFeaturePtr feat = CFeature::Create();
			feat->x			= points[1][i].x;
			feat->y			= points[1][i].y;
			feat->ID = i;
			feat->type = featKLT;
		features.push_back(feat);
		xx[i] = feat->x; yy[i] = feat->y;
	} // end for


	////CLUSTERING
	//int num_clusteres = 15;
	//int sample_count = features.size();
	//CvMat* samples = cvCreateMat( sample_count, 2, CV_32F );
 //   CvMat* clusters = cvCreateMat( sample_count, 1, CV_32S );
	//CvMat* centers = cvCreateMat( num_clusteres, 2, CV_32F );
	//detectClusteres( features, samples, clusters, centers , num_clusteres );
	//CImage image;
	//image.loadFromIplImage(img);
	//for(int i=0; i<num_clusteres; i++)
	//	image.cross( cvGetReal2D(centers, i, 0), cvGetReal2D(centers, i, 1), 255, 'o' );


	//double dist=1000;
	//CFeaturePtr nearest = features.nearest( cvGetReal2D(centers, 0, 0), cvGetReal2D(centers, 0, 1), dist );
	//image.cross(  nearest->x, nearest->y, 0, 'x' );

	cout<< features.size() << " features found\n" <<endl;
	win.showImageAndPoints(grey, features);
	//mrpt::system::pause();

	//FEATURE STATISTICS
//	double average_x = 0, average_y = 0, std_x = 0, std_y = 0;//	average_x = xx.mean();
//	average_y = yy.mean();
//	std_x = xx.std();
//	std_y = yy.std();
//	cout<< average_x <<endl << average_y <<endl << std_x <<endl << std_y <<endl ;
//	unsigned int x_min, x_max, y_min, y_max;
//	x_min = average_x - std_x/5;
//	x_max = average_x + std_x/5;
//	y_min = average_y - std_y/5;
//	y_max = average_y + std_y/5;
//	for(unsigned int i = 0; i < features.size(); i++)
//	{
//		if( features[i]->x > x_min && features[i]->x < x_max && features[i]->y > y_min && features[i]->y < y_max )
//			f_out.push_back(features[i]);
//	}
	//SEARCH WINDOW
//	while( f_out.size()<10 )
//	{
//		x_min -= std_x/10;
//		x_max += std_x/10;
//		y_min -= std_y/10;
//		y_max += std_y/10;
//		for(unsigned int i = 0; i < features.size(); i++){
//			if( features[i]->x > x_min && features[i]->x < x_max && features[i]->y > y_min && features[i]->y < y_max )
//				f_out.push_back(features[i]);
//		}
//	}
//
//	SELECT 3 POINTS
//	double maxArea = 0;
//	for(unsigned int i = 0; i < f_out.size(); i++){
//		for(unsigned int j = i; j < f_out.size(); j++){
//			for(unsigned int k = j; k < f_out.size(); k++){
//				if( TriangleArea( f_out[i]->x, f_out[i]->y, f_out[j]->x, f_out[j]->y, f_out[k]->x, f_out[k]->y ) > maxArea ){
//					maxArea = TriangleArea( f_out[i]->x, f_out[i]->y, f_out[j]->x, f_out[j]->y, f_out[k]->x, f_out[k]->y );
//					v1 = i; v2 = j; v3 = k;
//				}
//			}
//		}
//	}
//	v_init1[0] = f_out[v1]->x; v_init1[1] = f_out[v1]->y; v_init1[2] = 1;
//	v_init2[0] = f_out[v2]->x; v_init2[1] = f_out[v2]->y; v_init2[2] = 1;
//	v_init3[0] = f_out[v3]->x; v_init3[1] = f_out[v3]->y; v_init3[2] = 1;
//
////PRINT THE KLT FEATURES IN THE IMAGE
//	CDisplayWindow   wind("1st Frame");
//	//CImage image;
//	//image.loadFromIplImage(img);
//
//	IplImage *img2 = cvCreateImage(cvSize(img->width, img->height), img->depth, 1);
//	cvCopy(img,img2);
//	// Draw Image and square.
//	CvPoint pt1, pt2, pt3, pt4;
//	CvScalar			color = CV_RGB( 255, 255, 255 );		//color = CV_RGB( r, g, b );
//	pt1 = cvPoint( v_init1[0], v_init1[1] );
//	pt2 = cvPoint( v_init2[0], v_init2[1] );
//	pt3 = cvPoint( v_init3[0], v_init3[1] );
//	cvLine(img2, pt1, pt2, color, 1, 8, 0);
//	cvLine(img2, pt1, pt3, color, 1, 8, 0);
//	cvLine(img2, pt3, pt2, color, 1, 8, 0);
//
//	pt1 = cvPoint( x_min, y_min );
//	pt2 = cvPoint( x_max, y_min );
//	pt3 = cvPoint( x_max, y_max );
//	pt4 = cvPoint( x_min, y_max );
//	cvLine(img2, pt1, pt2, color, 1, 8, 0);
//	cvLine(img2, pt2, pt3, color, 1, 8, 0);
//	cvLine(img2, pt3, pt4, color, 1, 8, 0);
//	cvLine(img2, pt4, pt1, color, 1, 8, 0);
//
//
//	////TextOut the feature ID
//	//for(unsigned int i = 0; i < features.size(); i++)
//	//{
//	//	image.textOut( features.getByID(i)->x, features.getByID(i)->y, format("%d", i), 255 );
//	//}
//	//unsigned int chosen[5] = { 455, 309, 69, 85, 412 };
//	//for(unsigned int i = 0; i < 5; i++)
//	//{
//	//	sprintf( cstr, "%d", chosen[i] );
//	//	image.textOut( features.getByID(chosen[i])->x, features.getByID(chosen[i])->y, cstr, 255 );
//	//}
//
//	cout<< features.size() << " features found\n" <<endl;
//	win.showImageAndPoints(img2, features);
//	mrpt::system::pause();
//
//	//unsigned int chosen[5] = { 14, 18, 34, 64, 70 };
//	//unsigned int chosen[5] = { 455, 309, 69, 85, 412 };
//	//for(unsigned int i = 0; i < 5; i++)
//	//	f_out.push_back(features.getByID(chosen[i]));
//
//	//return f_out;
	return features;
}
//--------------------------------
//|		  MATCH FEATURES          |
//---------------------------------
void matchFeatures(CFeatureList &list1,  CFeatureList &list2, CMatchedFeatureList &matched_out)
{
	//CTicTac	tictac;
	TMatchingOptions opt;
	opt.useXRestriction=false;
	bool epipolar_match = false;
	if(epipolar_match)
	{
		opt.useEpipolarRestriction=true;
		opt.parallelOpticalAxis=false;
		opt.epipolar_TH=2.0;
		opt.maxEDSD_TH=0.18;
		opt.EDSD_RATIO=0.6;
		opt.F.loadFromTextFile("FM_.txt");
	}
	else
	{
		opt.useEpipolarRestriction=false;
		opt.maxEDSD_TH=0.18;
		opt.EDSD_RATIO=0.6;
	}
	cout<<"\nCalculating matches...\n";

	opt.matching_method=TMatchingOptions::mmDescriptorSURF;
	//tictac.Tic();
	mrpt::vision::utils::matchFeatures2( list1, list2, matched_out, opt);
	//cout << "Detected " << data->Harris_matched.size() << " Harris matches in " << endl;
	//cout << format("  %.03fms",tictac.Tac()*1000) << endl;
	matched_out.saveToTextFile("HarrisSURF_match.txt");
}


int main()
{
	unsigned int i, j;
	int count;
	CImage				im1, im2, patch;
	IplImage			*patch_tform, *patch_LM;
	CFeatureList		boot, flow_klt, seed_feat;
	double				dett, distance2, dist, distance = 0;
	CvPoint				pt1, pt2, pt3;
	CvScalar			color = CV_RGB( 255, 255, 255 );		//color = CV_RGB( r, g, b );
	CMatrix				mesh, correlations;
	CMatrixDouble66		B, D;
	CArrayDouble<6>		A;
	CMatrixDouble33		TForm;
	vector_double		optimal_x;
	vector_double		initial_x;
	vector_double		increments_x;
	vector_double		y;
	CMyLevMarq::TResultInfo	info;

	CvCapture *video = cvCaptureFromFile("d:\\test4_proc.avi");

	IplImage *image = 0, *grey = 0, *prev_grey = 0, *pyramid = 0, *prev_pyramid = 0, *swap_temp;

	int win_size = 11;
	const int MAX_COUNT = 500;
	CvPoint2D32f* points[2] = {0,0}, *swap_points;
	char* status = 0;
	int flags = 0, flagsT = 0;
	//int add_remove_pt = 0;
	//CvPoint pt;
	int k;


	try
	{
		//LOAD BOOTING INFORMATION
		//boot.loadFromTextFile("d:\\points.txt");
		//flow_klt.loadFromTextFile("d:\\points.txt");
		mesh.loadFromTextFile("d:\\mesh4.txt");
		img_seq_data aux1;

		IplImage *frame = cvQueryFrame(video);

		ASSERT_(frame);
		ASSERT_(frame->depth==IPL_DEPTH_8U);

		aux1.img = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 1);

		//COLOR DISCRIMINATION
		//IplImage* r = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 1);
		//IplImage* g = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 1);
		//IplImage* b = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 1);
		//IplImage* bk = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 1);
		//cvZero(bk);

		//cvSplit(frame, b, g, r, 0);

		//cvMul(b, bk, b, 1.2);
		//cvMul(g, bk, b, 1.2);
		//cvMul(r, bk, b, 0.6);

		//cvMerge(b, g, r, 0, frame);

		cvCvtColor( frame, aux1.img, CV_BGR2GRAY );

		im1.loadFromIplImage( aux1.img );
		//im1.saveToFile( "D:\\rgb2.bmp" );

		//im1.loadFromFile("D:\\corner.jpg");
		//patch_tform = (IplImage*) im1.getAsIplImage();
		//boot = ExtractFeaturesFrame1(patch_tform);

		// ALLOCATE ALL THE BUFFERS //
		image = cvCreateImage( cvGetSize(frame), 8, 3 );
		image->origin = frame->origin;
		grey = cvCreateImage( cvGetSize(frame), 8, 1 );
		prev_grey = cvCreateImage( cvGetSize(frame), 8, 1 );
		pyramid = cvCreateImage( cvGetSize(frame), 8, 1 );
		prev_pyramid = cvCreateImage( cvGetSize(frame), 8, 1 );
		points[0] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(points[0][0]));
		points[1] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(points[0][0]));
		status = (char*)cvAlloc(MAX_COUNT);
		flags = 0;

		boot = ExtractFeaturesFrame1(aux1.img, seed_feat);//, average_x, average_y, std_x, std_y);
		boot.saveToTextFile("D:\\boot_features.txt");
		count = boot.size();
		for( i = 0; i < count ; i++ )	//Copy boot to flow_klt
		{
			CFeaturePtr feat = CFeature::Create();
				feat->x			= boot[i]->x;
				feat->y			= boot[i]->y;
				feat->ID = i;
				feat->type = featKLT;
				feat.make_unique();
			flow_klt.push_back(feat);
			points[0][i].x = feat->x;
			points[0][i].y = feat->y;
		} // end for

		//boot = ExtractFeaturesFrame1(im1);
		//boot.saveToTextFile("d:\\boot.txt");

		//flow_klt.loadFromTextFile("d:\\boot.txt");
		//flow_klt = ExtractFeaturesFrame1(aux1.img);
		//flow_klt = ExtractFeaturesFrame1(im1);


	//// CREATE AND ALLOCATE ALL THE BUFFERS FOR SINGLE PATCHES//
	//	//std::vector<IplImage> tri_ref( mesh.getRowCount() );
	//	//std::vector<IplImage> tri_vid( mesh.getRowCount() );
	//	//tri_ref = (IplImage*)cvAlloc(numTri*sizeof(tri_ref[0]));
	//	int numFeatTri = 5;
	//	IplImage* tri_ref[ 9 ];
	//	IplImage* tri_vid[ 9 ];
	//	//IplImage* prev_greyT[ 9 ];
	//	CvPoint2D32f* pointsT[2] = {0,0};
	//	char* statusT = 0;
	//	pointsT[0] = (CvPoint2D32f*)cvAlloc(numFeatTri*sizeof(points[0][0]));
	//	pointsT[1] = (CvPoint2D32f*)cvAlloc(numFeatTri*sizeof(points[0][0]));
	//	statusT = (char*)cvAlloc(numFeatTri);
	//	IplImage *pyramidT = 0, *prev_pyramidT = 0;
	//	pyramidT = cvCreateImage( cvGetSize(frame), 8, 1 );
	//	prev_pyramidT = cvCreateImage( cvGetSize(frame), 8, 1 );


	////LOAD INITIAL FEATURES
	//	FILE* file;
	//	fopen_s(&file,"D:\\frame_proc4.fea","r");
	//	int countT;
	//	float x, y;
	//	fscanf_s(file, "%d", &count);

	//	for( i = 0; i < count; i++)
	//	{
	//		CFeaturePtr feat = CFeature::Create();
	//		fscanf_s(file, "%f\t%f", &x, &y);
	//		feat->x			=  x;
	//		feat->y			=  y;
	//		feat->ID = i;
	//		feat->type = featKLT;
	//		boot.push_back(feat);

	//		CFeaturePtr feat2 = feat;
	//		feat2.make_unique();
	//		flow_klt.push_back(feat2);

	//		points[0][i].x = x;
	//		points[0][i].y = y;
	//	}
	//	fclose(file);



		////VERTEX OF REFERENCE TRIANGLE:
		//po1.resize( mesh.getRowCount() );
		//po2.resize( mesh.getRowCount() );
		//po3.resize( mesh.getRowCount() );
		//for(t=0; t<mesh.getRowCount(); t++)
		//{
		//	po1[t][0] = boot[mesh(t,0)-1]->x; po1[t][1] = boot[mesh(t,0)-1]->y; po1[t][2] = 1;
		//	po2[t][0] = boot[mesh(t,1)-1]->x; po2[t][1] = boot[mesh(t,1)-1]->y; po2[t][2] = 1;
		//	po3[t][0] = boot[mesh(t,2)-1]->x; po3[t][1] = boot[mesh(t,2)-1]->y; po3[t][2] = 1;
		//}

		//aux1.tri_list.resize( mesh.getRowCount() );
		//aux1.ind_list.resize( mesh.getRowCount() );
		//loadPatches(boot, mesh, &(aux1.mask), aux1.tri_list, aux1.ind_list, aux1.img);			//LOAD PATCHES

		//data_img.push_back( aux1 );

		//correlations.resize( 400, mesh.getRowCount() );		//->PASARLO A LA ESTRUCTURA DE DATOS	!!!!!!!!!!!!!!!!
		//correlations.fill(0);

		//for( i=0; i<mesh.getRowCount(); i++ )
		//{
		//	pt1 = cvPoint( boot[mesh(i,0)-1]->x, boot[mesh(i,0)-1]->y);
		//	pt2 = cvPoint( boot[mesh(i,1)-1]->x, boot[mesh(i,1)-1]->y);
		//	pt3 = cvPoint( boot[mesh(i,2)-1]->x, boot[mesh(i,2)-1]->y);
		//	cvLine(aux1.img, pt1, pt2, color, 1, 8, 0);
		//	cvLine(aux1.img, pt1, pt3, color, 1, 8, 0);
		//	cvLine(aux1.img, pt3, pt2, color, 1, 8, 0);
		//}
		////win.showImage(aux1.img);
		////cvShowImage( "Reference", aux1.img );
		//im1.loadFromIplImage(aux1.img);

	////NCC STATISTICS (HISTOGRAM)
	//	char* cstr = new char [20];
	//	for( j=0; j<mesh.getRowCount(); j++ )
	//	{
	//		vector_double hist = aux1.tri_list[j].histogram( 0, 255, 256, false);
	//		//std::string fichNam = format( "D:\\histogram%u.txt",(j+1) );
	//		//vectorToTextFile(hist,fichNam);

	//		double media = 0; double length =  hist.sumAll(); //=aux1.tri_list[0].size();
	//		for( i=0; i<256; i++)
	//		{
	//			media = media + i*hist[i]/length;
	//		}
	//		double desviacion, varianza = 0;
	//		for( i=0; i<256; i++)
	//		{
	//			varianza = varianza + square(i-media)*(hist[i]/length);
	//		}
	//		desviacion = sqrt(varianza);
	//		cout<< "Histogram T" <<j << "\taverage: " << media <<"  \tdesviacion: " << desviacion <<endl <<endl;

	//		sprintf( cstr, "%d %.2f", (j+1), desviacion );
	//		im1.textOut( (boot[mesh(j,0)-1]->x + boot[mesh(j,1)-1]->x + boot[mesh(j,2)-1]->x)/3-20, (boot[mesh(j,0)-1]->y + boot[mesh(j,1)-1]->y + boot[mesh(j,2)-1]->y)/3, cstr, 255 );
	//	}
	//	delete cstr;

		//win.showImage(im1);
		//mrpt::system::pause();


		////PATCH FEATURES
		//CFeatureExtraction	fExt;
		//std::vector<CFeatureList> feat_patch_ref( mesh.getRowCount() );
		//std::vector<CFeatureList> feat_patch_vid( mesh.getRowCount() );
		//fExt.options.featsType = featKLT;
		//fExt.options.harrisOptions.tile_image = false;
		//fExt.options.patchSize=0;

		//// DRAW patch_ref
		//for( t=0; t<mesh.getRowCount(); t++ )
		//{
		//	//vector_float xx(3); xx[0] = pt1.x; xx[1] = pt2.x; xx[2] = pt3.x;
		//	//vector_float yy(3); yy[0] = pt1.y; yy[1] = pt2.y; yy[2] = pt3.y;

		//	patch_tform = cvCreateImage(cvSize(data_img[0].img->width, data_img[0].img->height), data_img[0].img->depth, 1);
		//	for(i=0; i<aux1.tri_list[t].size(); i++)
		//	{
		//		patch_tform->imageData[aux1.ind_list[t][i]] = aux1.tri_list[t][i];
		//	}
		//	tri_ref[t] = cvCreateImage(cvSize(data_img[0].img->width, data_img[0].img->height), data_img[0].img->depth, 1);
		//	//prev_greyT[t] = cvCreateImage(cvSize(data_img[0].img->width, data_img[0].img->height), data_img[0].img->depth, 1);
		//	cvCopy( patch_tform, tri_ref[t], 0 );

		//	patch.loadFromIplImage(patch_tform);
		//	//patch.saveToFile( format("D:\\ref%d.bmp", (t+1) ) );
		//	//win3.showImageAndPoints( patch, boot );
		//	//fExt.detectFeatures( tri_ref[t], feat_patch_ref[t], 0, numFeatTri);

		//	IplImage* eig = cvCreateImage( cvGetSize(tri_ref[t]), 8, 1 );
		//	IplImage* temp = cvCreateImage( cvGetSize(tri_ref[t]), 8, 1 );
		//	CvPoint2D32f* puntos = {0};
		//	puntos = (CvPoint2D32f*)cvAlloc(numFeatTri*sizeof(puntos[0]));
		//	double quality = 0.01;
		//	double min_distance = 10;

		//	cvGoodFeaturesToTrack( tri_ref[t], eig, temp, puntos, &numFeatTri,
		//						   quality, min_distance, 0, 3, 0, 0.04 );
		//	cvFindCornerSubPix( tri_ref[t], puntos, numFeatTri,
		//		cvSize(win_size,win_size), cvSize(-1,-1),
		//		cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
		//	for(i=0; i<numFeatTri; i++)
		//	{
		//	CFeaturePtr feat = CFeature::Create();
		//					feat->x			= puntos[i].x;
		//					feat->y			= puntos[i].y;
		//					feat->ID		= 0;
		//					feat->type		= featKLT;
		//					feat_patch_ref[t].push_back(feat);
		//					CFeaturePtr feat2 = feat;
		//					feat2.make_unique();
		//					feat_patch_vid[t].push_back(feat2);
		//					patch.cross( feat_patch_ref[t][i]->x, feat_patch_ref[t][i]->y, 255, '+' );
		//	}
		//	cvReleaseImage( &eig );
		//	cvReleaseImage( &temp );

			////SURF
			//fExt.options.featsType = featHarris;
			//fExt.options.harrisOptions.tile_image = false;
			//fExt.options.patchSize=0;
			//fExt.detectFeatures( tri_ref[t], feat_patch_ref[t], 0, numFeatTri);
			//fExt.computeDescriptors( tri_ref[t], feat_patch_ref[t], descSURF );
			//win2.showImageAndPoints( tri_ref[t], feat_patch_ref[t] );
			//mrpt::system::pause();
			//..........TO FINISH


			//patch.saveToFile( format("D:\\ref%d+p.bmp", (t+1) ) );
		//}

		cvCopy( aux1.img, grey, 0 );
		//cvFindCornerSubPix( grey, points[0], count,
			//	cvSize(win_size,win_size), cvSize(-1,-1),
			//	cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));

		//CHECK FOR A FRAME WITH ENOUGH DISPARITY AND TRACK FEATURES
		img_seq_data aux;

		CV_SWAP( prev_grey, grey, swap_temp );
		frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); //Take one frame each 5
		frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); //Take one frame each 5
		frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); //Take one frame each 5
		//frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); //Take one frame each 5
		//frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); //Take one frame each 5

		ASSERT_(frame);
		ASSERT_(frame->depth==IPL_DEPTH_8U);

		//if( !frame ) break;
		//if (f==200) break;

		aux.img = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 1);
		cvCvtColor( frame, aux.img, CV_BGR2GRAY );
		IplImage *frame_aux = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 1);
		cvCopy( aux.img, frame_aux );

		cvCopy( aux.img, grey, 0 );

		if( count > 0 )
		{
			cvCalcOpticalFlowPyrLK( prev_grey, grey, prev_pyramid, pyramid,
				points[0], points[1], count, cvSize(win_size,win_size), 3, status, 0,
				cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );
			flags |= CV_LKFLOW_PYR_A_READY;

			//for( i = 0; i < count; i++)
			//{
			//	flow_klt[i]->x = points[1][i].x;
			//	flow_klt[i]->y = points[1][i].y;
		//	}

			////CALCULATE AVERAGE DISTANCE OF FLOW
			//for( i=0; i<count; i++ )
			//{
			//	distance += sqrt( square( boot[i]->x - flow_klt[i]->x ) + square( boot[i]->y - flow_klt[i]->y ) );
			//}
			//distance /= count;

			for( i=0; i<count; i++ )
			{
				distance += sqrt( square( points[0][i].x - points[1][i].x ) + square( points[0][i].y - points[1][i].y ) );
			}
			distance /= count;		//Average distance of all features followed
			cout<< "Average distance: " <<distance <<endl;
		}
		else
			cout<< "Error tracking features...\n";

		for( i = 0; i < count; i++)
		{
			flow_klt[i]->x = points[1][i].x;
			flow_klt[i]->y = points[1][i].y;
		}
		flow_klt.saveToTextFile("D:\\flow_klt.txt");
		win1.showImageAndPoints(aux.img, flow_klt);
		//mrpt::system::pause();



		img_aux.loadFromIplImage(aux.img);

		//RANSAC
		ofstream ransac_log;
		ransac_log.open ("d:\\ransac_log.txt");
		vector<CMatrixTemplateNumeric<double> > detected_homographies;
		vector< vector<size_t> > plane_ind;
		const double DIST_THRESHOLD = 0.2;
		const size_t min_points = 15;
		CTicTac	tictac;
		tictac.Tic();
		ransac_homographies( boot, flow_klt, detected_homographies, DIST_THRESHOLD, min_points, plane_ind );
		cout<<"RANSAC Performance in " <<tictac.Tac() <<"s." <<endl;

		for (j=0;j<plane_ind.size();j++)
		{
			ransac_log << endl << detected_homographies[j] << endl ; // (0,0) <<" " << detected_homographies[0](0,1) <<" " << detected_homographies[0](0,2) <<endl;

			for (i=0;i<plane_ind[j].size();i++)
			{
				v_init1[0] = boot[plane_ind[j][i] ]->x;
				v_init1[1] = boot[plane_ind[j][i] ]->y;
				v_init1[2] = 1;
				detected_homographies[j].multiply_Ab( v_init1, v_init2 );
				//dist = sqrt( square( v_init2[0] - v_init1[0] ) + square(v_init2[1] - v_init1[1] ) );
				v_init3[0] = flow_klt[plane_ind[j][i] ]->x;
				v_init3[1] = flow_klt[plane_ind[j][i] ]->y;
				v_init3[2] = sqrt( square( v_init2[0] - v_init3[0] ) + square(v_init2[1] - v_init3[1] ) );
				ransac_log << v_init1 << v_init2 << v_init3 <<endl ;
			}
		}
		ransac_log.close();

		im2.loadFromIplImage(aux.img);
		TColor color;
		for(i=0; i<detected_homographies.size(); i++){
			//CImage imgColor(1,1,3);
			im2.colorImage( imgColor );	// Create a colorimage
			//k = i%4;
			//switch (k){
			//	case 0:
			//		color = TColor::blue;
			//		break;
			//	case 1:
			//		color = TColor::red;
			//		break;
			//	case 2:
			//		color = TColor::green;
			//		break;
			//	case 3:
			//		color = TColor::black;
			//		break;
			//	default:
			//		color = TColor::white;
			//}
			//color = TColor(25*(10-i),25*(20-i),65*(i-4));
			color = TColor::blue;
			for(j=0;j<plane_ind[i].size(); j++)
				imgColor.cross(round(flow_klt[ plane_ind[i][j] ]->x),round(flow_klt[ plane_ind[i][j] ]->y),color,'+');
		win2.showImage(imgColor);
		mrpt::system::pause();
		}


		//EXPAND THE TRIANGLE
		//float dir, dist;
		//CFeaturePtr next;
		//center.x =100; center.y =100;
		//while ( distance<5 )
		//{
		//	next = flow_klt.nearest(center.x, center.y, dist);
		//
		//}

		////SWAP THE TRIANGLE
		//while ( distance>5 )
		//{

		//}



		//CHECK LOCAL PLANARITY (It takes three vertex in a square and verifies if the rest of points in the square fulfill the homography)
		CArrayDouble<3> vs1, vs2, vs3;
		vs1[0] = boot[seed_feat[v1]->ID]->x; vs1[1] = boot[seed_feat[v1]->ID]->y; vs1[2] = 1;
		vs2[0] = boot[seed_feat[v2]->ID]->x; vs2[1] = boot[seed_feat[v2]->ID]->y; vs2[2] = 1;
		vs3[0] = boot[seed_feat[v3]->ID]->x; vs3[1] = boot[seed_feat[v3]->ID]->y; vs3[2] = 1;
		bool seed_plane = false;
		while ( !seed_plane )
		{
			//Build affine transformation
			D.fill(0);
			dett = (-vs1[0]*vs3[1]+vs1[0]*vs2[1]+vs3[1]*vs2[0]-vs2[1]*vs3[0]-vs2[0]*vs1[1]+vs3[0]*vs1[1]);			//This is the square root of minus det(B) -> sqtr(-det(B))
			D.set_unsafe(0,0, (-vs3[1]+vs2[1])/dett);
			D.set_unsafe(0,2, -(vs1[1]-vs3[1])/dett);
			D.set_unsafe(0,4, (-vs2[1]+vs1[1])/dett);
			D.set_unsafe(1,0, -(vs2[0]-vs3[0])/dett);
			D.set_unsafe(1,2, (vs1[0]-vs3[0])/dett);
			D.set_unsafe(1,4, -(vs1[0]-vs2[0])/dett);
			D.set_unsafe(2,0, (-vs2[1]*vs3[0]+vs3[1]*vs2[0])/dett);
			D.set_unsafe(2,2, -(vs1[0]*vs3[1]-vs3[0]*vs1[1])/dett);
			D.set_unsafe(2,4, (vs1[0]*vs2[1]-vs2[0]*vs1[1])/dett);
			D.set_unsafe(3,1, (-vs3[1]+vs2[1])/dett);
			D.set_unsafe(3,3, -(vs1[1]-vs3[1])/dett);
			D.set_unsafe(3,5, (-vs2[1]+vs1[1])/dett);
			D.set_unsafe(4,1, -(vs2[0]-vs3[0])/dett);
			D.set_unsafe(4,3, (vs1[0]-vs3[0])/dett);
			D.set_unsafe(4,5, -(vs1[0]-vs2[0])/dett);
			D.set_unsafe(5,1, (-vs2[1]*vs3[0]+vs3[1]*vs2[0])/dett);
			D.set_unsafe(5,3, -(vs1[0]*vs3[1]-vs3[0]*vs1[1])/dett);
			D.set_unsafe(5,5, (vs1[0]*vs2[1]-vs2[0]*vs1[1])/dett);
			D.saveToTextFile("d:\\D.txt");

			C[0] = flow_klt[seed_feat[v1]->ID]->x;
			C[1] = flow_klt[seed_feat[v1]->ID]->y;
			C[2] = flow_klt[seed_feat[v2]->ID]->x;
			C[3] = flow_klt[seed_feat[v2]->ID]->y;
			C[4] = flow_klt[seed_feat[v3]->ID]->x;
			C[5] = flow_klt[seed_feat[v3]->ID]->y;

			D.multiply_Ab(C,A);
			TForm.set_unsafe(0,0, A[0]);
			TForm.set_unsafe(0,1, A[1]);
			TForm.set_unsafe(0,2, A[2]);
			TForm.set_unsafe(1,0, A[3]);
			TForm.set_unsafe(1,1, A[4]);
			TForm.set_unsafe(1,2, A[5]);
			TForm.set_unsafe(2,0, 0);
			TForm.set_unsafe(2,1, 0);
			TForm.set_unsafe(2,2, 1);


			CArrayDouble<3> pto, pto_r;
			pto[2] = 1;
			distance = 0;
			for(i=0; i<seed_feat.size(); i++)
			{
				if( !(i==v1 || i==v2 || i==v3 ) )
				{
					pto[0] = boot[ seed_feat[i]->ID ]->x; pto[1] = boot[ seed_feat[i]->ID ]->y;
					TForm.multiply_Ab( pto, pto_r );
					distance += sqrt( square( pto_r[0] - flow_klt[seed_feat[i]->ID]->x ) + square(pto_r[1] - flow_klt[seed_feat[i]->ID]->y ) );
				}
			}
			distance /= ( seed_feat.size() - 3 );		//Distancia media de los píxeles del entorno

			if ( distance < 2 )
				seed_plane = true;


//			followPatch(TForm, aux.tri_list[t], aux.img);	//				followPatch(TForm, aux.tri_list[t], aux.ind_list[t], aux.img);

			correlations(2*f-1,t) = ncc_vector(data_img[0].tri_list[t], aux.tri_list[t]);
			cout<< "Affine-correlation\tFrame: " <<f <<"\tTriangle: " <<t+1 <<endl <<correlations(2*f-1,t) <<endl;

		// DRAW PATCH_TFORM
			patch_tform = cvCreateImage(cvSize(data_img[0].img->width, data_img[0].img->height), data_img[0].img->depth, 1);
			for( i=0; i<data_img[0].tri_list[t].size(); i++ )
			{
				patch_tform->imageData[data_img[0].ind_list[t][i]] = aux.tri_list[t][i];
			}
		}

		f=1;
/*		while(true)
		{
			////TEST NCC-VECTOR
			//double test = ncc_vector( data_img[0].tri_list[0], data_img[0].tri_list[0] );
			//cout<< "NCC Test: " <<test <<endl;

			CV_SWAP( prev_grey, grey, swap_temp );

			img_seq_data aux;
			//data_img[f].tri.resize(mesh.getRowCount);
			//data_img[f].img = (IplImage*) im2->getAsIplImage();

			frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); frame = cvQueryFrame(video); //Take one frame each 5
			ASSERT_(frame);
			ASSERT_(frame->depth==IPL_DEPTH_8U);

			if( !frame ) break;
			if (f==200) break;

			aux.img = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 1);
			cvCvtColor( frame, aux.img, CV_BGR2GRAY );
			IplImage *frame_aux = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 1);
			//cvCvtColor( frame, frame_aux, CV_BGR2GRAY );
			cvCopy( aux.img, frame_aux );


			//MRPT TRACKFEATURES-WRAPER
			//im2.loadFromIplImage(frame_aux);
			//trackFeatures( im1, im2, flow_klt2, 11, 11 );

			cvCopy( aux.img, grey, 0 );
			//cvCvtColor( image, grey, CV_BGR2GRAY );

			if( count > 0 )
			{
				cvCalcOpticalFlowPyrLK( prev_grey, grey, prev_pyramid, pyramid,
					points[0], points[1], count, cvSize(win_size,win_size), 3, status, 0,
					cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );
				flags |= CV_LKFLOW_PYR_A_READY;

				// ARRAY CONVERSION OPENCV->MRPT
				//CFeatureList::iterator		itFeat;
				//for( i = 0; i < count ; i++ )
				//{
				//	CFeaturePtr feat = CFeature::Create();
				//		feat->x			= points[1][i].x;
				//		feat->y			= points[1][i].y;
				//		feat->ID = i;
				//		feat->type = featKLT;
				//	flow_klt.push_back(feat);
				//} // end for
				for( i = 0; i < count; i++)
				{
					flow_klt[i]->x = points[1][i].x;
					flow_klt[i]->y = points[1][i].y;
 				}
				win1.showImageAndPoints(aux.img, flow_klt);
				mrpt::system::pause();

				//CALCULATE AVERAGE DISTANCE OF FLOW
				distance = 0;
				for( i=0; i<count; i++ )
				{
					distance += sqrt( square( boot[i]->x - flow_klt[i]->x ) + square( boot[i]->y - flow_klt[i]->y ) );
				}
				distance /= count;

				for( i = k = 0; i < count; i++ )
				{
					if( !status[i] )
						continue;

					//points[1][k++] = points[1][i];
					//cvCircle( image, cvPointFrom32f(points[1][i]), 3, CV_RGB(0,255,0), -1, 8,0);
				}
				//count = k;
			}

			//if( add_remove_pt && count < MAX_COUNT )
			//{
			//    points[1][count++] = cvPointTo32f(pt);
			//    cvFindCornerSubPix( grey, points[1] + count - 1, 1,
			//        cvSize(win_size,win_size), cvSize(-1,-1),
			//        cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
			//    add_remove_pt = 0;
			//}

	//       CV_SWAP( prev_grey, grey, swap_temp );
			CV_SWAP( prev_pyramid, pyramid, swap_temp );
			CV_SWAP( points[0], points[1], swap_points );



			//// Draw Image and mesh.	//frame_aux
			//for( i=0; i<mesh.getRowCount(); i++ )
			//{
			//	pt1 = cvPoint( flow_klt[mesh(i,0)-1]->x, flow_klt[mesh(i,0)-1]->y );
			//	pt2 = cvPoint( flow_klt[mesh(i,1)-1]->x, flow_klt[mesh(i,1)-1]->y );
			//	pt3 = cvPoint( flow_klt[mesh(i,2)-1]->x, flow_klt[mesh(i,2)-1]->y );
			//	cvLine(frame_aux, pt1, pt2, color, 1, 8, 0);
			//	cvLine(frame_aux, pt1, pt3, color, 1, 8, 0);
			//	cvLine(frame_aux, pt3, pt2, color, 1, 8, 0);
			//}
			////cvShowImage( "Video", frame_aux );
			////im2.loadFromIplImage(frame_aux);
			//win1.showImage(frame_aux);
			////mrpt::system::pause();


			// Check if all features have been followed
			CFeatureList::iterator it;
			for(it=flow_klt.begin(); it!=flow_klt.end(); it++)
			{
				if( (*it)->x < 0 )
				{
					cout<< "Feature " <<(*it)->ID << " couldn't be followed." <<endl;
					mrpt::system::pause();
				}
			}
			aux.tri_list.resize( mesh.getRowCount() );
			aux.ind_list.resize( mesh.getRowCount() );
			//loadPatches(flow_klt, mesh, &(aux.mask), aux.tri_list, aux.ind_list, aux.img);

			patch_tform = cvCreateImage(cvSize(data_img[0].img->width, data_img[0].img->height), data_img[0].img->depth, 1);
			patch_LM = cvCreateImage(cvSize(data_img[0].img->width, data_img[0].img->height), data_img[0].img->depth, 1);


			for(t=0; t<mesh.getRowCount(); t++)
			{
				//if(f==1)	//HISTOGRAM CALCULATION
				//{
				//	vector_double hist = aux1.tri_list[t].histogram( 0, 255, 256, false);

				//	double media = 0; double length = hist.sumAll(); //=aux1.tri_list[0].size();
				//	for( i=0; i<256; i++)
				//	{
				//		media = media + i*hist[i]/length;
				//	}
				//	double desviacion, varianza = 0;
				//	for( i=0; i<256; i++)
				//	{
				//		varianza = varianza + square(i-media)*(hist[i]/length);
				//	}
				//	desviacion = sqrt(varianza);
				//	cout<< "\nHistogram T" <<t << "\taverage: " << media <<"  \tdesviacion: " << desviacion <<endl;
				//}//END HISTOGRAM


		////TRANSFORM DATA TYPE TO PERFORM CORRELATION MEASUREMENT
				//for(t=0; t<mesh.getRowCount; t++)
				//{
				//	image = cvCreateMatHeader(1, data_img[f].tri_list[t].size(), CV_8UC1 );
				//	cvCreateData(image);
				//	for(k=0;k<data_img[f].tri_list[t].size();k++)
				//	{
				//		cvSet2D(image, 0, k, data_img[i].tri_list[t][k]);
				//	}
				//}
				//cvMatchTemplate(image, const CvArr* templ, result, CV_TM_CCORR_NORMED);
				//cvRelease(&image);


			// AFFINE MATRIX CALCULATION
				//B.fill(0);
				//for(h=0; h<3; h++)
				//{
				//	B(2*h,0) = boot[mesh(t,h)-1]->x;
				//	B(2*h,1) = boot[mesh(t,h)-1]->y;
				//	B(2*h,2) = 1;
				//	B(2*h+1,3) = boot[mesh(t,h)-1]->x;
				//	B(2*h+1,4) = boot[mesh(t,h)-1]->y;
				//	B(2*h+1,5) = 1;
				//	C[2*h] = flow_klt[mesh(t,h)-1]->x;
				//	C[2*h+1] = flow_klt[mesh(t,h)-1]->y;
				//}
				//B.saveToTextFile("d:\\B.txt");
				//cout << B <<endl << C << endl;
				//B.inv(D); //D.loadFromTextFile("d:\\invB.txt");

				D.fill(0);
				dett = (-po1[t][0]*po3[t][1]+po1[t][0]*po2[t][1]+po3[t][1]*po2[t][0]-po2[t][1]*po3[t][0]-po2[t][0]*po1[t][1]+po3[t][0]*po1[t][1]);			//This is the square root of minus det(B) -> sqtr(-det(B))
				D.set_unsafe(0,0, (-po3[t][1]+po2[t][1])/dett);
				D.set_unsafe(0,2, -(po1[t][1]-po3[t][1])/dett);
				D.set_unsafe(0,4, (-po2[t][1]+po1[t][1])/dett);
				D.set_unsafe(1,0, -(po2[t][0]-po3[t][0])/dett);
				D.set_unsafe(1,2, (po1[t][0]-po3[t][0])/dett);
				D.set_unsafe(1,4, -(po1[t][0]-po2[t][0])/dett);
				D.set_unsafe(2,0, (-po2[t][1]*po3[t][0]+po3[t][1]*po2[t][0])/dett);
				D.set_unsafe(2,2, -(po1[t][0]*po3[t][1]-po3[t][0]*po1[t][1])/dett);
				D.set_unsafe(2,4, (po1[t][0]*po2[t][1]-po2[t][0]*po1[t][1])/dett);
				D.set_unsafe(3,1, (-po3[t][1]+po2[t][1])/dett);
				D.set_unsafe(3,3, -(po1[t][1]-po3[t][1])/dett);
				D.set_unsafe(3,5, (-po2[t][1]+po1[t][1])/dett);
				D.set_unsafe(4,1, -(po2[t][0]-po3[t][0])/dett);
				D.set_unsafe(4,3, (po1[t][0]-po3[t][0])/dett);
				D.set_unsafe(4,5, -(po1[t][0]-po2[t][0])/dett);
				D.set_unsafe(5,1, (-po2[t][1]*po3[t][0]+po3[t][1]*po2[t][0])/dett);
				D.set_unsafe(5,3, -(po1[t][0]*po3[t][1]-po3[t][0]*po1[t][1])/dett);
				D.set_unsafe(5,5, (po1[t][0]*po2[t][1]-po2[t][0]*po1[t][1])/dett);
				D.saveToTextFile("d:\\D.txt");

				for(i=0; i<3; i++)
				{
					C[2*i] = flow_klt[mesh(t,i)-1]->x;
					C[2*i+1] = flow_klt[mesh(t,i)-1]->y;
				}

				D.multiply_Ab(C,A);
				TForm.set_unsafe(0,0, A[0]);
				TForm.set_unsafe(0,1, A[1]);
				TForm.set_unsafe(0,2, A[2]);
				TForm.set_unsafe(1,0, A[3]);
				TForm.set_unsafe(1,1, A[4]);
				TForm.set_unsafe(1,2, A[5]);
				TForm.set_unsafe(2,0, 0);
				TForm.set_unsafe(2,1, 0);
				TForm.set_unsafe(2,2, 1);

				followPatch(TForm, aux.tri_list[t], aux.img);	//				followPatch(TForm, aux.tri_list[t], aux.ind_list[t], aux.img);

				correlations(2*f-1,t) = ncc_vector(data_img[0].tri_list[t], aux.tri_list[t]);
				cout<< "Affine-correlation\tFrame: " <<f <<"\tTriangle: " <<t+1 <<endl <<correlations(2*f-1,t) <<endl;

			// DRAW PATCH_TFORM
				patch_tform = cvCreateImage(cvSize(data_img[0].img->width, data_img[0].img->height), data_img[0].img->depth, 1);
				for( i=0; i<data_img[0].tri_list[t].size(); i++ )
				{
					patch_tform->imageData[data_img[0].ind_list[t][i]] = aux.tri_list[t][i];
				}
				//cvShowImage( "Patch-TForm", patch_tform );
				//patch.loadFromIplImage(patch_tform);
				tri_vid[t] = cvCreateImage(cvSize(data_img[0].img->width, data_img[0].img->height), data_img[0].img->depth, 1);
				cvCopy( patch_tform, tri_vid[t], 0 );
				patch.loadFromIplImage(patch_tform);

				//win2.showImage(patch_tform);
				//patch.saveToFile( format("D:\\patch%d.bmp", t ) );
				win2.showImageAndPoints( patch_tform, boot );
				mrpt::system::pause();

			//DRAW TRANSFORMED PATCHES
				for( i=0; i<data_img[0].tri_list[t].size(); i++ )
					patch_tform->imageData[data_img[0].ind_list[t][i]] = aux.tri_list[t][i];

				pt1 = cvPoint( boot[mesh(t,0)-1]->x, boot[mesh(t,0)-1]->y);
				pt2 = cvPoint( boot[mesh(t,1)-1]->x, boot[mesh(t,1)-1]->y);
				pt3 = cvPoint( boot[mesh(t,2)-1]->x, boot[mesh(t,2)-1]->y);
				cvLine(patch_tform, pt1, pt2, color, 1, 8, 0);
				cvLine(patch_tform, pt1, pt3, color, 1, 8, 0);
				cvLine(patch_tform, pt3, pt2, color, 1, 8, 0);

				if( t==(mesh.getRowCount()-1) )
				{
					im2.loadFromIplImage(patch_tform);
					for( i=0; i<mesh.getRowCount(); i++ )
						im2.textOut( (boot[mesh(i,0)-1]->x + boot[mesh(i,1)-1]->x + boot[mesh(i,2)-1]->x)/3-20, (boot[mesh(i,0)-1]->y + boot[mesh(i,1)-1]->y + boot[mesh(i,2)-1]->y)/3-10, format("%.2f", correlations(2*f-1,i) ), 255 );

					win2.showImageAndPoints( im2, boot );		//im2 and boot has different scales, since the origins are 0 & 1 respectively, therefore the mesh is shifted 1pixel in each axis
					mrpt::system::pause();
				} //END DRAWING


			//FOLLOW PATCH-FEATURES
				countT = feat_patch_ref[t].size();
				for(i=0; i<countT; i++)
				{
					pointsT[0][i].x = feat_patch_vid[t][i]->x;
					pointsT[0][i].y = feat_patch_vid[t][i]->y;
				}
				cvCalcOpticalFlowPyrLK( tri_ref[t], tri_vid[t], prev_pyramidT, pyramidT,
					pointsT[0], pointsT[1], countT, cvSize(win_size,win_size), 3, statusT, 0,
					cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flagsT );
				//flagsT |= CV_LKFLOW_PYR_A_READY;

				distance2 = 0;
				for(i=0; i<countT; i++)
				{
					//CFeaturePtr feat = CFeature::Create();
					//feat->x			= pointsT[1][i].x;
					//feat->y			= pointsT[1][i].y;
					//feat->ID		= i;
					//feat->type		= featKLT;
					//feat_patch_vid[t].push_back(feat);
					feat_patch_vid[t][i]->x = pointsT[1][i].x;
					feat_patch_vid[t][i]->y = pointsT[1][i].y;
					dist = feat_patch_ref[t][i]->x - feat_patch_vid[t][i]->x ;
					distance2 += square( feat_patch_ref[t][i]->x - feat_patch_vid[t][i]->x ) + square( feat_patch_ref[t][i]->y - feat_patch_vid[t][i]->y );
					patch.line( feat_patch_ref[t][i]->x, feat_patch_ref[t][i]->y, feat_patch_vid[t][i]->x, feat_patch_vid[t][i]->y, 255 );
					patch.cross( feat_patch_vid[t][i]->x, feat_patch_vid[t][i]->y, 255, '+' );
				}
				correlations(2*f,t) = distance2 / countT;
				cout<< "distance " <<correlations(2*f,t) <<endl;
				win3.showImageAndPoints( tri_vid[t], feat_patch_vid[t] );
				patch.textOut( (boot[mesh(t,0)-1]->x + boot[mesh(t,1)-1]->x + boot[mesh(t,2)-1]->x)/3+60, (boot[mesh(t,0)-1]->y + boot[mesh(t,1)-1]->y + boot[mesh(t,2)-1]->y)/3, format( "Distance2: %f", correlations(2*f,t) ), 255 );
				patch.saveToFile( format("D:\\vid%d+p.bmp", (t+1) ) );
				mrpt::system::pause();

				CV_SWAP( tri_ref[t], tri_vid[t], swap_temp );		//Reference changes to the previous patch.
				//CV_SWAP( pointsT[0], pointsT[1], swap_points );
				//CV_SWAP( prev_pyramidT, pyramidT, swap_temp );



			//LEVENBERG-MARQUARDT
				//if( correlations(2*f-1,t)<0.85 && correlations(2*f-1,t)>0.6 )	//Then perform Levenberg-Marquardt algorithm
				//{
				//	//%                   [ A D ]
				//	//% [u v] = [x y 1] * [ B E ]	Construct Affine Transformation
				//	//%                   [ C F ]

				//	initial_x.resize(8);
				//	initial_x[0] = A[0];
				//	initial_x[1] = A[1];
				//	initial_x[2] = A[2];
				//	initial_x[3] = A[3];
				//	initial_x[4] = A[4];
				//	initial_x[5] = A[5];
				//	initial_x[6] = 0;
				//	initial_x[7] = 0;

				//	// Increments Vector = [ 0.0001 0.0001 .5 0.0001 0.0001 .5 0.000001 0.000001 ]
				//	increments_x.resize(8, 0.0001 );
				//	increments_x[2] = 0.5; increments_x[5] = 0.5; increments_x[6] = 0.000001; increments_x[7] = 0.000001;
				//	optimal_x.resize(8);
				//	//CMyLevMarq::execute(optimal_x, initial_x, CostFunction, increments_x, data_img, info  );
				//	CTicTac tictac;
				//	tictac.Tic();
				//	CMyLevMarq::execute(optimal_x, initial_x, CostFunction, increments_x, aux, info, false, 200, 1e-3, 1e-10, 1e-10, false );

				//	correlations(2*f,t) = 1 - sqrt(info.final_sqr_err);
				//	//cout<< "LM-correlation\tFrame: " <<f  <<"\tTriangle: " <<t+1 <<endl <<correlations(2*f,t) <<endl;
				//	cout<< "LM-correlation" <<endl <<correlations(2*f,t) <<endl;
				//	cout<< tictac.Tac() <<" seconds\t" << info.iterations_executed <<" iterations\n" <<endl;

				//	TForm.set_unsafe(0,0, optimal_x[0]);
				//	TForm.set_unsafe(0,1, optimal_x[1]);
				//	TForm.set_unsafe(0,2, optimal_x[2]);
				//	TForm.set_unsafe(1,0, optimal_x[3]);
				//	TForm.set_unsafe(1,1, optimal_x[4]);
				//	TForm.set_unsafe(1,2, optimal_x[5]);
				//	TForm.set_unsafe(2,0, optimal_x[6]);
				//	TForm.set_unsafe(2,1, optimal_x[7]);
				//	TForm.set_unsafe(2,2, 1);

				//	//	//DRAW IMAGE AND POINTS
				//	//vector_double pp1(3), pp2(3), pp3(3);
				//	//vector_float xx(3), yy(3);
				//	//TForm.multiply_Ab(po1[t],pp1);
				//	//pp1[0] = pp1[0]/pp1[2]; pp1[1] = pp1[1]/pp1[2];
				//	//TForm.multiply_Ab(po2[t],pp2);
				//	//pp2[0] = pp2[0]/pp2[2]; pp2[1] = pp2[1]/pp2[2];
				//	//TForm.multiply_Ab(po3[t],pp3);
				//	//pp3[0] = pp3[0]/pp3[2]; pp3[1] = pp3[1]/pp3[2];
				//	//xx[0] = pp1[0]; xx[1] = pp2[0]; xx[2] = pp3[0];
				//	//yy[0] = pp1[1]; yy[1] = pp2[1]; yy[2] = pp3[1];
				//	////data_img[f].img = (IplImage*) im2->getAsIplImage();
				//	//win3.showImageAndPoints( aux.img, xx, yy, TColor::red );
				//	//mrpt::system::pause();

				//	vector_byte triangle;
				//	triangle.resize(data_img[0].tri_list[t].size());
				//	followPatch(TForm, triangle, aux.img);
				//	//cout<< ncc( data_img[0].tri_list[t], triangle ) <<endl;

				//	// DRAW PATCH_LM
				//	patch_tform = cvCreateImage(cvSize(data_img[0].img->width, data_img[0].img->height), data_img[0].img->depth, 1);
				//	for(i=0; i<data_img[0].tri_list[t].size(); i++)
				//	{
				//		patch_tform->imageData[data_img[0].ind_list[t][i]] = triangle[i];
				//	}
				//	patch.loadFromIplImage(patch_tform);
				//	win3.showImage(patch);
				//	mrpt::system::pause();


				//	////DRAW LEVENBERG-MARQUARDT TRANSFORMED PATCHES
				//	//for( i=0; i<data_img[0].tri_list[t].size(); i++ )
				//	//	patch_LM->imageData[data_img[0].ind_list[t][i]] = triangle[i];
				//	//
				//	//pt1 = cvPoint( boot[mesh(t,0)-1]->x, boot[mesh(t,0)-1]->y);
				//	//pt2 = cvPoint( boot[mesh(t,1)-1]->x, boot[mesh(t,1)-1]->y);
				//	//pt3 = cvPoint( boot[mesh(t,2)-1]->x, boot[mesh(t,2)-1]->y);
				//	//cvLine(patch_LM, pt1, pt2, color, 1, 8, 0);
				//	//cvLine(patch_LM, pt1, pt3, color, 1, 8, 0);
				//	//cvLine(patch_LM, pt3, pt2, color, 1, 8, 0);
				//	//
				//	//if( t==(mesh.getRowCount()-1) )
				//	//{
				//	//	im2.loadFromIplImage(patch_LM);
				//	//	for( i=0; i<mesh.getRowCount(); i++ )
				//	//	{
				//	//		char* cstr = new char [20];
				//	//		sprintf( cstr, "%.2f", correlations(2*f-1,i) );
				//	//		im2.textOut( (boot[mesh(i,0)-1]->x + boot[mesh(i,1)-1]->x + boot[mesh(i,2)-1]->x)/3-20, (boot[mesh(i,0)-1]->y + boot[mesh(i,1)-1]->y + boot[mesh(i,2)-1]->y)/3-10, cstr, 255 );
				//	//		delete cstr;
				//	//	}
				//	//	win2.showImageAndPoints( im2, boot );		//im2 and boot has different scales, since the origins are 0 & 1 respectively, therefore the mesh is shifted 1pixel in each axis
				//	//	mrpt::system::pause();
				//	//} //END DRAWING

				//} //End If -> Levenberg-Marquardt

			}

			data_img.push_back( aux );

			f++;
		}*/
		cvReleaseCapture( &video );
		correlations.saveToTextFile( "D:\\correlations.txt" );
	}
	catch(std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}

	return 0;
}

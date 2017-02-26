/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "utils.h"
#include "imgfeatures.h"

#include <math.h>

int import_oxfd_features( char*, struct feature** );
int export_oxfd_features( char*, struct feature*, int );
void draw_oxfd_features( IplImage*, struct feature*, int );
void draw_oxfd_feature( IplImage*, struct feature*, CvScalar );

int import_lowe_features( char*, struct feature** );
int export_lowe_features( char*, struct feature*, int );
void draw_lowe_features( IplImage*, struct feature*, int );
void draw_lowe_feature( IplImage*, struct feature*, CvScalar );


/*
Reads image features from file.  The file should be formatted as from
the code provided by the Visual Geometry Group at Oxford:


@param filename location of a file containing image features
@param type determines how features are input.  If \a type is FEATURE_OXFD,
	the input file is treated as if it is from the code provided by the VGG
	at Oxford:

	http://www.robots.ox.ac.uk:5000/~vgg/research/affine/index.html

	If \a type is FEATURE_LOWE, the input file is treated as if it is from
	David Lowe's SIFT code:

	http://www.cs.ubc.ca/~lowe/keypoints
@param features pointer to an array in which to store features

@return Returns the number of features imported from filename or -1 on error
*/
int import_features( char* filename, int type, struct feature** feat )
{
	int n;

	switch( type )
	{
	case FEATURE_OXFD:
		n = import_oxfd_features( filename, feat );
		break;
	case FEATURE_LOWE:
		n = import_lowe_features( filename, feat );
		break;
	default:
		fprintf( stderr, "Warning: import_features(): unrecognized feature" \
				"type, %s, line %d\n", __FILE__, __LINE__ );
		return -1;
	}

	if( n == -1 )
		fprintf( stderr, "Warning: unable to import features from %s,"	\
			" %s, line %d\n", filename, __FILE__, __LINE__ );
	return n;
}



/*
Exports a feature set to a file formatted depending on the type of
features, as specified in the feature struct's type field.

@param filename name of file to which to export features
@param feat feature array
@param n number of features

@return Returns 0 on success or 1 on error
*/
int export_features( char* filename, struct feature* feat, int n )
{
	int r, type;

	if( n <= 0  ||  ! feat )
	{
		fprintf( stderr, "Warning: no features to export, %s line %d\n",
				__FILE__, __LINE__ );
		return 1;
	}
	type = feat[0].type;
	switch( type )
	{
	case FEATURE_OXFD:
		r = export_oxfd_features( filename, feat, n );
		break;
	case FEATURE_LOWE:
		r = export_lowe_features( filename, feat, n );
		break;
	default:
		fprintf( stderr, "Warning: export_features(): unrecognized feature" \
				"type, %s, line %d\n", __FILE__, __LINE__ );
		return -1;
	}

	if( r )
		fprintf( stderr, "Warning: unable to export features to %s,"	\
				" %s, line %d\n", filename, __FILE__, __LINE__ );
	return r;
}


/*
Draws a set of features on an image

@param img image on which to draw features
@param feat array of Oxford-type features
@param n number of features
*/
void draw_features( IplImage* img, struct feature* feat, int n )
{
	int type;

	if( n <= 0  ||  ! feat )
	{
		fprintf( stderr, "Warning: no features to draw, %s line %d\n",
				__FILE__, __LINE__ );
		return;
	}
	type = feat[0].type;
	switch( type )
	{
	case FEATURE_OXFD:
		draw_oxfd_features( img, feat, n );
		break;
	case FEATURE_LOWE:
		draw_lowe_features( img, feat, n );
		break;
	default:
		fprintf( stderr, "Warning: draw_features(): unrecognized feature" \
			" type, %s, line %d\n", __FILE__, __LINE__ );
		break;
	}
}



/*
Calculates the squared Euclidian distance between two feature descriptors.

@param f1 first feature
@param f2 second feature

@return Returns the squared Euclidian distance between the descriptors of
f1 and f2.
*/
double descr_dist_sq( struct feature* f1, struct feature* f2 )
{
	double diff, dsq = 0;
	double* descr1, * descr2;
	int i, d;

	d = f1->d;
	if( f2->d != d )
		return DBL_MAX;
	descr1 = f1->descr;
	descr2 = f2->descr;

	for( i = 0; i < d; i++ )
	{
		diff = descr1[i] - descr2[i];
		dsq += diff*diff;
	}
	return dsq;
}



/***************************** Local Functions *******************************/


/*
Reads image features from file.  The file should be formatted as from
the code provided by the Visual Geometry Group at Oxford:

http://www.robots.ox.ac.uk:5000/~vgg/research/affine/index.html

@param filename location of a file containing image features
@param features pointer to an array in which to store features

@return Returns the number of features imported from filename or -1 on error
*/
int import_oxfd_features( char* filename, struct feature** features )
{
	struct feature* f;
	int i, j, n, d;
	double x, y, a, b, c, dv;
	FILE* file;

	if( ! features )
		fatal_error( "NULL pointer error, %s, line %d",  __FILE__, __LINE__ );

	if( ! ( file = fopen( filename, "r" ) ) )
	{
		fprintf( stderr, "Warning: error opening %s, %s, line %d\n",
				filename, __FILE__, __LINE__ );
		return -1;
	}

	/* read dimension and number of features */
	if( fscanf( file, " %d %d ", &d, &n ) != 2 )
	{
		fprintf( stderr, "Warning: file read error, %s, line %d\n",
				__FILE__, __LINE__ );
		return -1;
	}
	if( d > FEATURE_MAX_D )
	{
		fprintf( stderr, "Warning: descriptor too long, %s, line %d\n",
				__FILE__, __LINE__ );
		return -1;
	}


	f = calloc( n, sizeof(struct feature) );
	for( i = 0; i < n; i++ )
	{
		/* read affine region parameters */
		if( fscanf( file, " %lf %lf %lf %lf %lf ", &x, &y, &a, &b, &c ) != 5 )
		{
			fprintf( stderr, "Warning: error reading feature #%d, %s, line %d\n",
					i+1, __FILE__, __LINE__ );
			free( f );
			return -1;
		}
		f[i].img_pt.x = f[i].x = x;
		f[i].img_pt.y = f[i].y = y;
		f[i].a = a;
		f[i].b = b;
		f[i].c = c;
		f[i].d = d;
		f[i].type = FEATURE_OXFD;

		/* read descriptor */
		for( j = 0; j < d; j++ )
		{
			if( ! fscanf( file, " %lf ", &dv ) )
			{
				fprintf( stderr, "Warning: error reading feature descriptor" \
						" #%d, %s, line %d\n", i+1, __FILE__, __LINE__ );
				free( f );
				return -1;
			}
			f[i].descr[j] = dv;
		}

		f[i].scl = f[i].ori = 0;
		f[i].category = 0;
		f[i].fwd_match = f[i].bck_match = f[i].mdl_match = NULL;
		f[i].mdl_pt.x = f[i].mdl_pt.y = -1;
		f[i].feature_data = NULL;
	}

	if( fclose(file) )
	{
		fprintf( stderr, "Warning: file close error, %s, line %d\n",
				__FILE__, __LINE__ );
		free( f );
		return -1;
	}

	*features = f;
	return n;
}




/*
Exports a feature set to a file formatted as one from the code provided
by the Visual Geometry Group at Oxford:

http://www.robots.ox.ac.uk:5000/~vgg/research/affine/index.html

@param filename name of file to which to export features
@param feat feature array
@param n number of features

@return Returns 0 on success or 1 on error
*/
int export_oxfd_features( char* filename, struct feature* feat, int n )
{
	FILE* file;
	int i, j, d;

	if( n <= 0 )
	{
		fprintf( stderr, "Warning: feature count %d, %s, line %d\n",
				n, __FILE__, __LINE__ );
		return 1;
	}
	if( ! ( file = fopen( filename, "w" ) ) )
	{
		fprintf( stderr, "Warning: error opening %s, %s, line %d\n",
				filename, __FILE__, __LINE__ );
		return 1;
	}

	d = feat[0].d;
	fprintf( file, "%d\n%d\n", d, n );
	for( i = 0; i < n; i++ )
	{
		fprintf( file, "%f %f %f %f %f", feat[i].x, feat[i].y, feat[i].a,
				feat[i].b, feat[i].c );
		for( j = 0; j < d; j++ )
			fprintf( file, " %f", feat[i].descr[j] );
		fprintf( file, "\n" );
	}

	if( fclose(file) )
	{
		fprintf( stderr, "Warning: file close error, %s, line %d\n",
				__FILE__, __LINE__ );
		return 1;
	}

	return 0;
}



/*
Draws Oxford-type affine features

@param img image on which to draw features
@param feat array of Oxford-type features
@param n number of features
*/
void draw_oxfd_features( IplImage* img, struct feature* feat, int n )
{
	CvScalar color = CV_RGB( 255, 255, 255 );
	int i;

	if( img-> nChannels > 1 )
		color = FEATURE_OXFD_COLOR;
	for( i = 0; i < n; i++ )
		draw_oxfd_feature( img, feat + i, color );
}



/*
Draws a single Oxford-type feature

@param img image on which to draw
@param feat feature to be drawn
@param color color in which to draw
*/
void draw_oxfd_feature( IplImage* img, struct feature* feat, CvScalar color )
{
	double m[4] = { feat->a, feat->b, feat->b, feat->c };
	double v[4] = { 0 };
	double e[2] = { 0 };
	CvMat M, V, E;
	double alpha, l1, l2;

	/* compute axes and orientation of ellipse surrounding affine region */
	cvInitMatHeader( &M, 2, 2, CV_64FC1, m, CV_AUTOSTEP );
	cvInitMatHeader( &V, 2, 2, CV_64FC1, v, CV_AUTOSTEP );
	cvInitMatHeader( &E, 2, 1, CV_64FC1, e, CV_AUTOSTEP );
#if CV_MAJOR_VERSION==1
	cvEigenVV( &M, &V, &E, DBL_EPSILON );
#else
	cvEigenVV( &M, &V, &E, DBL_EPSILON, -1,-1 );
#endif
	l1 = 1 / sqrt( e[1] );
	l2 = 1 / sqrt( e[0] );
	alpha = -atan2( v[1], v[0] );
	alpha *= 180 / CV_PI;

	cvEllipse( img, cvPoint( feat->x, feat->y ), cvSize( l2, l1 ), alpha,
				0, 360, CV_RGB(0,0,0), 3, 8, 0 );
	cvEllipse( img, cvPoint( feat->x, feat->y ), cvSize( l2, l1 ), alpha,
				0, 360, color, 1, 8, 0 );
	cvLine( img, cvPoint( feat->x+2, feat->y ), cvPoint( feat->x-2, feat->y ),
			color, 1, 8, 0 );
	cvLine( img, cvPoint( feat->x, feat->y+2 ), cvPoint( feat->x, feat->y-2 ),
			color, 1, 8, 0 );
}



/*
Reads image features from file.  The file should be formatted as from
the code provided by David Lowe:

http://www.cs.ubc.ca/~lowe/keypoints/

@param filename location of a file containing image features
@param features pointer to an array in which to store features

@return Returns the number of features imported from filename or -1 on error
*/
int import_lowe_features( char* filename, struct feature** features )
{
	struct feature* f;
	int i, j, n, d;
	double x, y, s, o, dv;
	FILE* file;

	if( ! features )
		fatal_error( "NULL pointer error, %s, line %d",  __FILE__, __LINE__ );

	if( ! ( file = fopen( filename, "r" ) ) )
	{
		fprintf( stderr, "Warning: error opening %s, %s, line %d\n",
			filename, __FILE__, __LINE__ );
		return -1;
	}

	/* read number of features and dimension */
	if( fscanf( file, " %d %d ", &n, &d ) != 2 )
	{
		fprintf( stderr, "Warning: file read error, %s, line %d\n",
				__FILE__, __LINE__ );
		return -1;
	}
	if( d > FEATURE_MAX_D )
	{
		fprintf( stderr, "Warning: descriptor too long, %s, line %d\n",
				__FILE__, __LINE__ );
		return -1;
	}

	f = calloc( n, sizeof(struct feature) );
	for( i = 0; i < n; i++ )
	{
		/* read affine region parameters */
		if( fscanf( file, " %lf %lf %lf %lf ", &y, &x, &s, &o ) != 4 )
		{
			fprintf( stderr, "Warning: error reading feature #%d, %s, line %d\n",
					i+1, __FILE__, __LINE__ );
			free( f );
			return -1;
		}
		f[i].img_pt.x = f[i].x = x;
		f[i].img_pt.y = f[i].y = y;
		f[i].scl = s;
		f[i].ori = o;
		f[i].d = d;
		f[i].type = FEATURE_LOWE;

		/* read descriptor */
		for( j = 0; j < d; j++ )
		{
			if( ! fscanf( file, " %lf ", &dv ) )
			{
				fprintf( stderr, "Warning: error reading feature descriptor" \
						" #%d, %s, line %d\n", i+1, __FILE__, __LINE__ );
				free( f );
				return -1;
			}
			f[i].descr[j] = dv;
		}

		f[i].a = f[i].b = f[i].c = 0;
		f[i].category = 0;
		f[i].fwd_match = f[i].bck_match = f[i].mdl_match = NULL;
		f[i].mdl_pt.x = f[i].mdl_pt.y = -1;
	}

	if( fclose(file) )
	{
		fprintf( stderr, "Warning: file close error, %s, line %d\n",
				__FILE__, __LINE__ );
		free( f );
		return -1;
	}

	*features = f;
	return n;
}



/*
Exports a feature set to a file formatted as one from the code provided
by David Lowe:

http://www.cs.ubc.ca/~lowe/keypoints/

@param filename name of file to which to export features
@param feat feature array
@param n number of features

@return Returns 0 on success or 1 on error
*/
int export_lowe_features( char* filename, struct feature* feat, int n )
{
	FILE* file;
	int i, j, d;

	if( n <= 0 )
	{
		fprintf( stderr, "Warning: feature count %d, %s, line %d\n",
				n, __FILE__, __LINE__ );
		return 1;
	}
	if( ! ( file = fopen( filename, "w" ) ) )
	{
		fprintf( stderr, "Warning: error opening %s, %s, line %d\n",
				filename, __FILE__, __LINE__ );
		return 1;
	}

	d = feat[0].d;
	fprintf( file, "%d %d\n", n, d );
	for( i = 0; i < n; i++ )
	{
		fprintf( file, "%f %f %f %f", feat[i].y, feat[i].x,
				feat[i].scl, feat[i].ori );
		for( j = 0; j < d; j++ )
		{
			/* write 20 descriptor values per line */
			if( j % 20 == 0 )
				fprintf( file, "\n" );
			fprintf( file, " %d", (int)(feat[i].descr[j]) );
		}
		fprintf( file, "\n" );
	}

	if( fclose(file) )
	{
		fprintf( stderr, "Warning: file close error, %s, line %d\n",
				__FILE__, __LINE__ );
		return 1;
	}

	return 0;
}


/*
Draws Lowe-type features

@param img image on which to draw features
@param feat array of Oxford-type features
@param n number of features
*/
void draw_lowe_features( IplImage* img, struct feature* feat, int n )
{
	CvScalar color = CV_RGB( 255, 255, 255 );
	int i;

	if( img-> nChannels > 1 )
		color = FEATURE_LOWE_COLOR;
	for( i = 0; i < n; i++ )
		draw_lowe_feature( img, feat + i, color );
}



/*
Draws a single Lowe-type feature

@param img image on which to draw
@param feat feature to be drawn
@param color color in which to draw
*/
void draw_lowe_feature( IplImage* img, struct feature* feat, CvScalar color )
{
	int len, hlen, blen, start_x, start_y, end_x, end_y, h1_x, h1_y, h2_x, h2_y;
	double scl, ori;
	double scale = 5.0;
	double hscale = 0.75;
	CvPoint start, end, h1, h2;

	/* compute points for an arrow scaled and rotated by feat's scl and ori */
	start_x = cvRound( feat->x );
	start_y = cvRound( feat->y );
	scl = feat->scl;
	ori = feat->ori;
	len = cvRound( scl * scale );
	hlen = cvRound( scl * hscale );
	blen = len - hlen;
	end_x = cvRound( len *  cos( ori ) ) + start_x;
	end_y = cvRound( len * -sin( ori ) ) + start_y;
	h1_x = cvRound( blen *  cos( ori + CV_PI / 18.0 ) ) + start_x;
	h1_y = cvRound( blen * -sin( ori + CV_PI / 18.0 ) ) + start_y;
	h2_x = cvRound( blen *  cos( ori - CV_PI / 18.0 ) ) + start_x;
	h2_y = cvRound( blen * -sin( ori - CV_PI / 18.0 ) ) + start_y;
	start = cvPoint( start_x, start_y );
	end = cvPoint( end_x, end_y );
	h1 = cvPoint( h1_x, h1_y );
	h2 = cvPoint( h2_x, h2_y );

	cvLine( img, start, end, color, 1, 8, 0 );
	cvLine( img, end, h1, color, 1, 8, 0 );
	cvLine( img, end, h2, color, 1, 8, 0 );
}



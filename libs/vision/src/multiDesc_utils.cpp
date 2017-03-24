/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/multiDesc_utils.h>
#include <mrpt/vision/utils.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/CFeature.h>

#include <mrpt/poses/CPoint3D.h>
//#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/obs/CObservationVisualLandmarks.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/geometry.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
//using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

#ifdef MRPT_OS_WINDOWS
    #include <process.h>
    #include <windows.h>		// TODO: This is temporary!!!
#endif

const int FEAT_FREE = -1;
//const int NOT_ASIG = 0;   // JL: Not used anymore?? (FAMD)
//const int ASG_FEAT = 1;
//const int AMB_FEAT = 2;

/*-------------------------------------------------------------
					insertHashCoeffs
-------------------------------------------------------------*/
void vision::insertHashCoeffs(
                const CFeaturePtr       & feat,
                TQuantizationTable      & qTable )
{
    MRPT_START
    for( int k = 0; k < int(feat->multiScales.size()); ++k )
    {
        for( int m = 0; m < int(feat->multiOrientations[k].size()); ++m )
        {
            int key1 = feat->multiHashCoeffs[k][m][0];
            int key2 = feat->multiHashCoeffs[k][m][1];
            int key3 = feat->multiHashCoeffs[k][m][2];

            bool found = false;
            if( qTable.find(key1)               != qTable.end() &&
                qTable[key1].find(key2)         != qTable[key1].end() &&
                qTable[key1][key2].find(key3)   != qTable[key1][key2].end() )
            {
                // The entry for these keys already exists
                // Check if the qTable already contain this ID and multiScale!
                for( int n = 0; n < int(qTable[key1][key2][key3].size()); ++n )
                {
                    TFeatureID  thisID      = qTable[key1][key2][key3][n].first;
                    double      thisScale   = qTable[key1][key2][key3][n].second;
                    if( thisID == feat->ID && thisScale == feat->multiScales[k] )
                        found = true;
    //                            cout << "Inserting in: " << key1 << "," << key2 << "," << key3 << endl;
                } // end-for
            } // end-if
            if( !found )        // Insert the new coefficients if they haven't been inserted before
                qTable[key1][key2][key3].push_back( make_pair(feat->ID,feat->multiScales[k]) );
        } // end for multiOrientations
    } // end for multiScales
    MRPT_END
} // end-insertHashCoeffs

/*-------------------------------------------------------------
					saveQTableToFile
-------------------------------------------------------------*/
void vision::saveQTableToFile( const TQuantizationTable &qTable, const string &filename )
{
    FILE *f = mrpt::system::os::fopen( filename, "wt" );

    typedef map<int,map<int,map<int,deque<pair<TFeatureID, double> > > > > TQuantizationTable;

    TQuantizationTable::const_iterator                                    it1;
    map<int,map<int,deque<pair<TFeatureID, double> > > >::const_iterator  it2;
    map<int,deque<pair<TFeatureID, double> > >::const_iterator            it3;

    for( it1 = qTable.begin(); it1 != qTable.end(); ++it1 )
        for( it2 = it1->second.begin(); it2 != it1->second.end(); ++it2 )
            for( it3 = it2->second.begin(); it3 != it2->second.end(); ++it3 )
            {
                mrpt::system::os::fprintf( f, "%d\t%d\t%d\t", it1->first, it2->first, it3->first );
                for( int k = 0; k < int(it3->second.size()); ++k )
                    mrpt::system::os::fprintf( f, "%lu\t%.2f\t",static_cast<long unsigned int>(it3->second[k].first), it3->second[k].second );
                mrpt::system::os::fprintf( f, "\n" );
            } // end-for
    mrpt::system::os::fclose( f );
} // end-saveQTableToFile

/*-------------------------------------------------------------
					relocalizeMultiDesc
-------------------------------------------------------------*/
// Return the number of features which have been properly re-matched
TMultiResMatchingOutput vision::relocalizeMultiDesc(
                const CImage                        & image,
                CFeatureList                        & baseList,
                CFeatureList                        & currentList,
                TQuantizationTable                  & qTable,
                const TMultiResDescOptions          & desc_opts,
                const TMultiResDescMatchOptions     & match_opts )
{
    MRPT_START
    const bool PARAR = false;

    int TH = 30;                        // The threshold for searching in the quantization table
    TMultiResMatchingOutput output;
    output.firstListCorrespondences.resize( baseList.size(), -1 );
    output.firstListDistance.resize( baseList.size() );
    output.firstListFoundScales.resize( baseList.size() );
    output.secondListCorrespondences.resize( currentList.size(), -1 );
    output.nMatches = 0;

    TQuantizationTable::iterator                                    it1;
    map<int,map<int,deque<pair<TFeatureID, double> > > >::iterator  it2;
    map<int,deque<pair<TFeatureID, double> > >::iterator            it3;

    vector<TFeatureID>  vID;

    int hpSize  = desc_opts.basePSize/2;             // Half of the patch size
    int w       = image.getWidth();
    int h       = image.getHeight();

    map<TFeatureID, vector<double> > featsToCompareMap;

    int curCounter = 0;
    CFeatureList::iterator it;
    for( it = currentList.begin(); it != currentList.end(); ++it, ++curCounter )
    {
        if( PARAR && (*it)->ID == 193 )
        {
            for( int i = 0; i < (int)(*it)->multiScales.size(); ++i )
                cout << (*it)->multiScales[i] << endl;
        }
        ASSERT_( (*it)->multiHashCoeffs.size() == (*it)->multiScales.size() );

//        (*it)->dumpToConsole();
//        (*it)->patch.saveToFile(format("imgs/patch_ini_%d.jpg", int((*it)->ID)));
//        cout << "done" << endl;

        featsToCompareMap.clear();
        if( !(*it)->descriptors.hasDescriptorMultiSIFT() )
        {
            cout << "[relocalizeMultiDesc] Feature " << (*it)->ID << " in currentList hasn't got any descriptor." << endl;

            // Compute the new descriptors at scale 1.0
            TMultiResDescOptions nopts = desc_opts;
            nopts.scales.resize(1);
            nopts.scales[0] = 1.0;
            if( (*it)->x+hpSize > (w-1) || (*it)->y+hpSize > (h-1) ||
                (*it)->x-hpSize < 0 || (*it)->y-hpSize < 0 )
            {
                cout << "[relocalizeMultiDesc] WARNING: Feature too close to the border. MultiDescriptor computation skipped." << endl;
                continue;
            }
            computeMultiResolutionDescriptors( image, *it, nopts );
        } // end-if

        for( int k = 0; k < int((*it)->multiOrientations[0].size()); ++k )
        {
            /* Where to look for within the quantization table?*/
            /* this is done for each feature and for each main orientation*/
            int c1mn = (*it)->multiHashCoeffs[0][k][0] - TH;
            int c1mx = (*it)->multiHashCoeffs[0][k][0] + TH;

            int c2mn = (*it)->multiHashCoeffs[0][k][1] - TH;
            int c2mx = (*it)->multiHashCoeffs[0][k][1] + TH;

            int c3mn = (*it)->multiHashCoeffs[0][k][2] - TH;
            int c3mx = (*it)->multiHashCoeffs[0][k][2] + TH;

            for( int m1 = c1mn; m1 < c1mx; ++m1 )
            {
                it1 = qTable.find(m1);
                if( it1 != qTable.end() )
                {
                    for( int m2 = c2mn; m2 < c2mx; ++m2 )
                    {
                        it2 = it1->second.find(m2);
                        if( it2 != it1->second.end() )
                        {
                            for( int m3 = c3mn; m3 < c3mx; ++m3 )
                            {
                                it3 = it2->second.find(m3);
                                if( it3 != it2->second.end() )
                                {
                                    for( int n = 0; n < int(it3->second.size()); ++n )
                                    {
                                        featsToCompareMap[qTable[m1][m2][m3][n].first].push_back( qTable[m1][m2][m3][n].second );
                                    } // endfor n
                                } // endif it3
                            } // endfor m3
                        } // endif it2
                    } // endfor m2
                } // endif it1
            } //endfor m1

            // Erase duplicates
            vID.resize( featsToCompareMap.size() ); // To store only the IDs
            int counter = 0;
            for( map< TFeatureID, vector<double> >::iterator nit = featsToCompareMap.begin(); nit != featsToCompareMap.end(); ++nit, ++counter )
            {
                // Remove duplicates
                std::sort( nit->second.begin(), nit->second.end() );

                vector<double>::iterator vit;
                vit = std::unique( nit->second.begin(), nit->second.end() );

                nit->second.resize( vit - nit->second.begin() );

                // Add to the ID vector
                vID[counter] = nit->first;
            }

            // Find the scales where to look
            // Use all orientations for each scale (we don't know a priori what's the change in orientation)
            counter = 0;
            // for each feature to compare
            int minDist     = 1e6;
            int minBaseScl  = 0;
            int minBaseFeat = 0;
            for( map< TFeatureID, vector<double> >::iterator nit = featsToCompareMap.begin(); nit != featsToCompareMap.end(); ++nit, ++counter )
            {
                int baseIdx;
                CFeaturePtr baseFeat = baseList.getByID( nit->first, baseIdx );

//                cout << int((*it)->ID) << " (" << (*it)->x << "," << (*it)->y << ")";
//                cout << " -------------------------------------------------------------" << endl;

                // for each scale within the base feature
                for( int k1 = 0; k1 < int(baseFeat->multiScales.size()); ++k1 )
                {
                    // for each scale from the qTable
                    for( int k2 = 0; k2 < int(nit->second.size()); ++k2 )
                    {
                        if( baseFeat->multiScales[k1] == nit->second[k2] )  // if this scale has been selected
                        {
                            // for each orientation of the feature 1
                            for( int k3 = 0; k3 < int(baseFeat->multiOrientations[k1].size()); ++k3 )
                            {
                                // for each orientation of the feature 2
                                for( int k4 = 0; k4 < int((*it)->multiOrientations[0].size()); ++k4 )
                                {
                                    // check orientation
                                    if( fabs(baseFeat->multiOrientations[k1][k3]-(*it)->multiOrientations[0][k4]) > DEG2RAD(10) )
                                        continue;
//                                    cout << "Orientation check passed" << endl;
//                                    cout << "HASH: " << (*it)->multiHashCoeffs[0][k4] << endl;

                                    // Compute the distance
                                    int dist = 0;
                                    for( int d = 0; d < int(baseFeat->descriptors.multiSIFTDescriptors[k1][k3].size()); ++d )
                                        dist += fabs( float(baseFeat->descriptors.multiSIFTDescriptors[k1][k3][d]) - float((*it)->descriptors.multiSIFTDescriptors[0][k4][d]) );

//                                    cout << nit->first << "[" << baseFeat->multiScales[k1] << "] - ori: " << baseFeat->multiOrientations[k1][k3] << " - HASH: " << baseFeat->multiHashCoeffs[k1][k3] << " - dist: " << dist << endl;
                                    if( dist < minDist )
                                    {
                                        minDist     = dist;     // minimun distance of the features
                                        minBaseFeat = baseIdx;  // index of the base feature
                                        minBaseScl  = k1;       // scale of the base feature
                                    } // end-if
                                } // end-for
                            } // end-for
                        } // end-if
                    } // end-for-k2
                } // end-for-k1
//                if( baseFeat->ID == 32 && (*it)->ID == 9 )
//                    mrpt::system::pause();
            } // end-for-nit
            if( minDist < match_opts.matchingThreshold )
            {
                // We've found a match
                // Store the index of the current feature
                output.firstListCorrespondences[minBaseFeat]    = curCounter;
                output.firstListFoundScales[minBaseFeat]        = minBaseScl;
                output.firstListDistance[minBaseFeat]           = minDist;
                output.secondListCorrespondences[curCounter]    = minBaseFeat;
                output.nMatches++;
            }
        } // end-for orientations
//        if( (*it)->ID == 9 )
//        {
//            baseList.getByID(32)->dumpToConsole();
//            (*it)->dumpToConsole();
//            mrpt::system::pause();
//        }

    } // end-for-it

    return output;

    MRPT_END
} // end-relocalizeMultiDesc

/*-------------------------------------------------------------
					updateBaseList
-------------------------------------------------------------*/
// Updates the following parameters for each feature in Base List:
// Coordinates, the number of frames it has been seen in, the number of frames since the last time it was seen and
// the number of frames it has not been seen. This is done according to the information in vector "idx" and the
// positions of the features in current list.
void vision::updateBaseList(
                CFeatureList                        & baseList,
                const CFeatureList                  & currentList,
                const vector<int>                   & idx )
{
    MRPT_START
    ASSERT_( idx.size() == baseList.size() );

    size_t sz = idx.size();

    CVectorDouble dp(sz), my(sz);
    int counter = 0;
    for( int k = 0; k < (int)sz; ++k )
    {
        if( idx[k] == FEAT_FREE )
        {
            baseList[k]->nTimesNotSeen++;
            baseList[k]->nTimesLastSeen++;
        }
        else
        {
            baseList[k]->nTimesSeen++;
            baseList[k]->nTimesLastSeen = 0;

            // Update position!
            dp[k] = fabs(baseList[k]->depth-currentList[idx[k]]->depth);
//            cout << "dp:" << dp[k] << endl;

            counter++;
        } // end-else
    } // end-for

    double m_dp, std_dp;
    mrpt::math::meanAndStd( dp, m_dp, std_dp );
    cout << "mean&std: " << m_dp << "," << std_dp << endl;

    for( int k = 0; k < (int)idx.size(); ++k )
    {
        if( idx[k] != FEAT_FREE )
        {
            // Update position!
            if( dp[k] < (m_dp+std_dp) )
            {
                baseList[k]->x      = currentList[idx[k]]->x;
                baseList[k]->y      = currentList[idx[k]]->y;
                baseList[k]->p3D    = currentList[idx[k]]->p3D;
            }
        } // end-if
    } // end-for

    MRPT_END
} // updateBaseList

/*-------------------------------------------------------------
					checkScalesAndFindMore
-------------------------------------------------------------*/
// Checks the scales where the matched were found and find more descriptors if they are located at the boundaries of
// the scale range o a certain feature within base list.
// It also deletes the features in base list which are have not been seen for a long time and it wasn't seen enough.
void vision::checkScalesAndFindMore(
                CMatchedFeatureList                 & baseList,
                const CFeatureList                  & currentList,
                const CImage                        & currentImage,
                const TMultiResMatchingOutput       & output,
                const TMultiResDescOptions          & computeOpts,
                const TMultiResDescMatchOptions     & matchOpts )
{
    CMatchedFeatureList::iterator itBase;
    int m = 0;
//    cout << "Base: " << baseList.size() << " and output: " << output.firstListCorrespondences.size() << endl;
    for( itBase = baseList.begin(); itBase!= baseList.end(); ++m )
    {

//        cout << m << " was last seen " << itBase->->first->nTimesLastSeen << " frames ago and has been seen " << itBase->first->nTimesSeen << " times so far." << endl;
        if( itBase->first->nTimesLastSeen > matchOpts.lastSeenThreshold &&
            itBase->first->nTimesSeen < matchOpts.timesSeenThreshold )
        {
//            cnt++;
//            cout << "Deleted feature #" << m << endl;
            itBase = baseList.erase( itBase );
            //++itBase;
        }
        else
        {
            // We've found a tracked match
            // We have found the match in both frames! Check out the scales!
//            if( !(m < (int)output.firstListCorrespondences.size() ))
//            {
//                cout << m << " vs " << (int)output.firstListCorrespondences.size() << endl;
//                ASSERT_( m < (int)output.firstListCorrespondences.size() );
//            }

            int tidx = output.firstListCorrespondences[m];
//            ASSERT_( tidx < (int)currentList.size() );

            if( tidx != FEAT_FREE )
            {
                if( output.firstListFoundScales[m] == 0 )
                {
                    cout << "Left feature " << m << " found in scale 0!" << endl;
//                    currentImage.saveToFile("imgs/current.jpg");
//                    cout << "px: "<< currentList[tidx]->x << ","<< currentList[tidx]->y << endl;
                    int res = computeMoreDescriptors( currentImage, currentList[tidx], itBase->first, true, computeOpts );
                    ASSERT_( itBase->first->descriptors.hasDescriptorMultiSIFT() );
                    if( res == 0) cout << "LF LOWSCALES Out of bounds!!" << endl;
                    //mrpt::system::pause();
                }
                else
                {
                    size_t nscales = itBase->first->multiScales.size()-1;
                    //itBase->first->dumpToConsole();
                    if( output.firstListFoundScales[m] == (int)nscales )
                    {
                        cout << "Left feature " << m << " found in last scale!" << endl;
                        int res = computeMoreDescriptors( currentImage, currentList[tidx], itBase->first, false, computeOpts );
                        if( res == 0) cout << "LF HIGHSCALES Out of bounds!!" << endl;
    //                    mrpt::system::pause();
                    }
                }
            } // end-if
            ++itBase;
        } // end-else
    } // end-for
} // checkScalesAndFindMore

/*-------------------------------------------------------------
                    computeGradient
-------------------------------------------------------------*/
// computeGradient.- Computes both the (approximated) gradient magnitude and orientation for a certain point within the image
// return: true = OK; false = if the keypoint is located at the image border (where the gradient cannot be computed).
// x = col, y = row
bool vision::computeGradient(
                const CImage                        & image,
                const unsigned int                  x,
                const unsigned int                  y,
                double                              & mag,
                double                              & ori )
{
    if( x > 0 && x < image.getWidth()-1 && y > 0 && y < image.getHeight()-1 )
    {
        float d1, d2;
        //----------
        //| 0|-1| 0|
        //----------
        //|-1| 0| 1|
        //----------
        //| 0| 1| 0|
        //----------

        // According to Hess's implementation (Lowe does: d2 = image.getAsFloat(x,y+1) - px4 = image.getAsFloat(x,y-1); )
        d1 = image.getAsFloat(x+1,y) - image.getAsFloat(x-1,y);
        d2 = image.getAsFloat(x,y-1) - image.getAsFloat(x,y+1);

        mag = sqrt( d1*d1 + d2*d2 );
        ori  = atan2( d2, d1 );      // From -pi to pi
        return true;
    }
    else
    {
        cout << "[computeGradient]: Out of bounds in " << x << "," << y << " with image: " << image.getWidth() << "x" << image.getHeight() << endl;
        return false;
    }
} // end-computeGradient

/*-------------------------------------------------------------
					computeMainOrientations
-------------------------------------------------------------*/
// Compute a set of main orientations (if there are more than one) of a certain keypoint within the image.
// return: true = OK; false = keypoint is too close the image margin (there is no space for the whole patch)
bool vision::computeMainOrientations( const CImage &image,
                const unsigned int                  x,
                const unsigned int                  y,
                const unsigned int                  patchSize,
                std::vector<double>                 & orientations,
                const double                        & sigma )
{
    MRPT_START

    // Pre operations
    orientations.clear();

    // Local variables:
    const unsigned int NBINS    = 36;
    const int hPatchSize        = patchSize/2;

    vector<double> oris( NBINS, 0.0 );
    int mx = (int)x, my = (int)y;

    // Check if there's no margin problems when computing the orientation
    if( mx-(hPatchSize+1) < 0 || my-(hPatchSize+1) < 0 || mx+(hPatchSize+1) > (int)(image.getWidth()-1) || my+(hPatchSize+1) > (int)(image.getHeight()-1) )
        return false;      // Feature is too close to the image's border

    // For each pixel within the patch (patchSize should be 29x29):
    // Compute local gradients (magnitude and orientation)
    // The orientation histogram is weighted with a Gaussian with sigma = 7.5 (Cheklov's Thesis)
//    cout << "IM: " << image.getWidth() << " and PS: " << patchSize << endl;
    double exp_denom = 2.0 * sigma *sigma;
    double bin_size = M_2PI/NBINS;
    double mag, ori;
    for( int ii = -hPatchSize; ii <= hPatchSize; ++ii )
        for( int jj = -hPatchSize; jj <= hPatchSize; ++jj )
        {
            if( computeGradient( image, mx+ii, my+jj, mag, ori ) )
            {
                ori += M_PI;        // ori: from 0 to 2*pi
                double w = mag*exp( -( ii*ii + jj*jj ) / exp_denom );

                int bin             = ((int)(floor(ori/bin_size))) % NBINS;    // from 0 to 35
                double bin_center   = bin*bin_size;
                double dif          = ori-bin_center;
                int nxbin;
                if( dif > 0 )
                    nxbin = bin == NBINS-1 ? 0 : bin+1;             // the other involved bin is the next one
                else
                    nxbin = bin == 0 ? NBINS-1 : bin-1;             // the other involved bin is the previous one

                double nbin_center = nxbin*bin_size;
                double dif2        = ori-nbin_center;

                oris[bin]   += w*fabs(dif2)/bin_size;               // dif2 == 0 means that "ori" is equal to "bin_center"
                oris[nxbin] += w*fabs(dif)/bin_size;                // dif == 0 means that "ori" is equal to "nbin_center"
            } // end-if
            else
                return false;
        } // end
    // Search for the maximum
    double         mxori   = 0.0;
    for( unsigned int k = 0; k < oris.size(); ++k )
    {
        if( oris[k] > mxori )
        {
            mxori = oris[k];
        }
    } // end-for

    // Compute the peaks of the histogram of orientations
    double hist_mag_th = 0.8*mxori;
    for( unsigned int k = 0; k < oris.size(); ++k )
    {
        double pv = k == 0 ? oris[oris.size()-1] : oris[k-1];               // Previous peak of the histogram
        double nv = k == oris.size()-1 ? 0 : oris[k+1];                     // Next peak of the histogram

        if( oris[k] > pv && oris[k] > nv && oris[k] > hist_mag_th )         // It this peak is maximum and is over 0.8 of the maximum peak
        {
            // Check this formulae:
            // double A    = (pv-nv)/(4*oris[k]+2*nv-2*pv-4*oris[k]);
            // double peak = A/(1+2*A);

            // Interpolate the position of the peak
            double int_bin = k + 0.5 * (pv-nv)/(pv-2.0*oris[k]+nv); // Hess formulae
			int_bin = ( int_bin < 0 ) ? NBINS + int_bin : ( int_bin >= NBINS ) ? int_bin - NBINS : int_bin;
			double int_ori = ( ( M_2PI * int_bin ) / NBINS ) - M_PI;    // Back to -pi:pi
            orientations.push_back( int_ori );

//            cout << "Ori found: " << int_ori << endl;
        }
    }
    return true;
    MRPT_END
} // end vision::computeMainOrientation

/*-------------------------------------------------------------
					interpolateHistEntry
-------------------------------------------------------------*/
void vision::interpolateHistEntry(
                vector<double>                      & oris,
                const double                        & cbin,
                const double                        & rbin,
                const double                        & obin,
                const double                        & mag,
                const int                           d,
                const int                           n )
{
    // Insert the sample into the orientation histogram taking into account that there is an overlapping
    // of half a bin between consecutive bins.
    // [And the first one overlaps with the last one??? --> Apparently not]

    CTimeLogger logger;
    logger.disable();

    double ncbin = cbin + d/2. - 0.5;
    double nrbin = rbin + d/2. - 0.5;

    int ncbin_i = floor( ncbin );
    int nrbin_i = floor( nrbin );
    int nobin_i = floor( obin );

    double d_c = ncbin_i - ncbin;
    double d_r = nrbin_i - nrbin;
    double d_o = nobin_i - obin;

    for( int k = 0; k < 2; ++k )
        for( int m = 0; m < 2; ++m )
            for( int l = 0; l < 2; ++l )
                if( ncbin_i+k >= 0 && ncbin_i+k < d && nrbin_i+m >= 0 && nrbin_i+m < d )
                {
                    int idx = ((nobin_i+l)%n) + n*(ncbin_i+k) + n*d*(nrbin_i+m);
                    oris[idx] += mag*(1-fabs(d_c+k))*(1-fabs(d_r+m))*(1-fabs(d_o+l));
                }

//    // Spatial bin
//    std::vector<int>    colIndex, rowIndex;
//    std::vector<double> colBinDistance, rowBinDistance;
//
//    logger.enter("Main loop");
//    colIndex.reserve(2);
//    colBinDistance.reserve(2);
//    rowIndex.reserve(2);
//    rowBinDistance.reserve(2);
//    for( int bin = 0; bin < d; bin++ )
//    {
//        double binCenter    = bin-1.5;                          // Center of the bin
//        double colDistance  = fabs( cbin - binCenter );         // Distance to the center of the bin
//        if( colDistance < 1.0 )                                 // If it is close enough
//        {
//            colIndex.push_back( bin );
//            colBinDistance.push_back( 1-colDistance );
//        }
//
//        double rowDistance = fabs( rbin - binCenter );
//        if( rowDistance < 1.0 )
//        {
//            rowIndex.push_back( bin );
//            rowBinDistance.push_back( 1-rowDistance );
//        }
//
//        if( colIndex.size() == 2 && rowIndex.size() == 2 )       // We have found all we need (stop looping)
//            break;
//    } // end for
//
//    logger.leave("Main loop");
//    ASSERT_( colIndex.size() <= 2 && rowIndex.size() <= 2 );
//
//    // Orientation bin
//    std::vector<int>    oriIndex(2);
//    std::vector<double> oriBinDistance(2);
//    oriIndex[0]         = floor( obin );
//    oriIndex[1]         = oriIndex[0] == n-1 ? 0 : oriIndex[0]+1;
//    oriBinDistance[0]   = 1 - (obin-oriIndex[0]);
//    oriBinDistance[1]   = 1 - oriBinDistance[0];
//
//    // The entry can affect to 2 (1x1x2), 4 (2x1x2 or 1x2x2) or 8 (2x2x2) bins
//    // Insert into the histogram
//    logger.enter("Insertion in histogram");
//    for( unsigned int k = 0; k < colIndex.size(); ++k )
//        for( unsigned int l = 0; l < rowIndex.size(); ++l )
//            for( unsigned int m = 0; m < 2; ++m )
//            {
//                if( colIndex[k] >= 0 && rowIndex[l] >= 0 )
//                {
//                    unsigned int idx = (unsigned int)(oriIndex[m] + n*colIndex[k] + n*d*rowIndex[l] );
//                    oris[idx] += mag*colBinDistance[k]*rowBinDistance[l]*oriBinDistance[m];
//                } // end if
//            } // end for
//    logger.leave("Insertion in histogram");
} // end of interpolateHistEntry

/*-------------------------------------------------------------
					computeHistogramOfOrientations
-------------------------------------------------------------*/
// x = col, y = row
void vision::computeHistogramOfOrientations(
                const CImage                        & image,
                const unsigned int                  x,
                const unsigned int                  y,
                const unsigned int                  patchSize,
                const double                        & orientation,
                vector<int32_t>                     & descriptor,
                const TMultiResDescOptions          & opts,
                vector<int32_t>                     & hashCoeffs )
{
    MRPT_START
    CTimeLogger tlogger;
    tlogger.disable();

    // The patch is 29x29
    int     w               = 16;   // width of the used patch
    int     Bp              = 4;    // Number of spatial bins
    int     n               = 8;    // Number of frequential bins (per spatial bin) --> Descriptor size: 4 x 4 x 8 = 128
    double  cos_t           = cos( orientation );
	double  sin_t           = sin( orientation );
	double  bins_per_rad    = n / M_2PI;
	double  exp_denom       = opts.sg3*opts.sg3*2;
	int     radius          = 0.5*patchSize;
	vector<double> oris( Bp*Bp*n, 0.0 );                // Typically 128-D

	// Normalize patch
	CImage nimage, thePatch;
	if( opts.computeHashCoeffs )
	{
        image.extract_patch( thePatch, x-radius, y-radius, patchSize, patchSize );
        normalizeImage( thePatch, nimage );             // Normalize to zero-mean and unit standard deviation
	}

//     For each pixel in the 23x23 patch:
//     a. find its rotated position
//     b. determine its proper spatial bin (and the other affected one)
//     c. rotate its orientation with respect the main orientation
//     d. determine its frequential bin
//     e. compute the weight
//    const double kp = ((double)(Bp+1))/((double)w); // [0.31250] Constant for mapping between -8,8 -> -2.5,2.5
    const double kp = double(Bp)/double(w); // [0.25] Constant for mapping between -8,8 -> -2,2
    double mag, ori;
    double cbin, rbin, obin;
    double c1 = 0.0, c2 = 0.0, c3 = 0.0;

//    FILE *f1 = mrpt::system::os::fopen( "imgs/c1p.txt", "wt" );
//    FILE *f2 = mrpt::system::os::fopen( "imgs/c1n.txt", "wt" );
//    FILE *f3 = mrpt::system::os::fopen( "imgs/c2p.txt", "wt" );
//    FILE *f4 = mrpt::system::os::fopen( "imgs/c2n.txt", "wt" );
//    FILE *f5 = mrpt::system::os::fopen( "imgs/c3p.txt", "wt" );
//    FILE *f6 = mrpt::system::os::fopen( "imgs/c3n.txt", "wt" );
//    FILE *f7 = mrpt::system::os::fopen( "imgs/c0.txt", "wt" );
//    FILE *f8 = mrpt::system::os::fopen( "imgs/c3.txt", "wt" );
//    FILE *f9 = mrpt::system::os::fopen( "imgs/r0.txt", "wt" );
//    FILE *f10 = mrpt::system::os::fopen( "imgs/r3.txt", "wt" );
//    FILE *f11 = mrpt::system::os::fopen( "imgs/out.txt", "wt" );

    for( int c = -radius; c <= radius; ++c )
        for( int r = -radius; r <= radius; ++r )
        {
            cbin = kp*c*cos_t - kp*r*sin_t;
            rbin = kp*c*sin_t + kp*r*cos_t;

            if( cbin > -2.5 && cbin < 2.5 && rbin > -2.5 && rbin < 2.5 )
            {
                tlogger.enter("computeGradient");
                bool res = vision::computeGradient( image, x+c, y+r, mag, ori );
                tlogger.leave("computeGradient");
                if( res )
				{
				    ori -= orientation;
					while( ori < 0.0 )
						ori += M_2PI;
					while( ori >= M_2PI )
						ori -= M_2PI;

					obin = ori*bins_per_rad;
					w = exp( -(c*c + r*r) / exp_denom );
					tlogger.enter("interpolate");
                    vision::interpolateHistEntry( oris, cbin, rbin, obin, mag, Bp, n );
                    tlogger.leave("interpolate");
				} // end if

//				if( cbin < -0.5 )
//                    mrpt::system::os::fprintf( f7, "%d %d\n", y+r, x+c );
//                if( cbin > 0.5 )
//                    mrpt::system::os::fprintf( f8, "%d %d\n", y+r, x+c );
//				if( rbin < -0.5 )
//                    mrpt::system::os::fprintf( f9, "%d %d\n", y+r, x+c );
//                if( rbin > 0.5 )
//                    mrpt::system::os::fprintf( f10, "%d %d\n", y+r, x+c );

				// Compute the hashing coefficients:
				if( opts.computeHashCoeffs )
				{
				    if( cbin < 0 )
				    {
				        c1 += nimage.getAsFloat( c+radius, r+radius );
//				        mrpt::system::os::fprintf( f1, "%d %d\n", y+r, x+c );
				    }
				    else
				    {
				        c1 -= nimage.getAsFloat( c+radius, r+radius );
//				        mrpt::system::os::fprintf( f2, "%d %d\n", y+r, x+c );
				    }
                    if( rbin < 0 )
                    {
                        c2 += nimage.getAsFloat( c+radius, r+radius );
//                        mrpt::system::os::fprintf( f3, "%d %d\n", y+r, x+c );
                    }
                    else
                    {
                        c2 -= nimage.getAsFloat( c+radius, r+radius );
//                        mrpt::system::os::fprintf( f4, "%d %d\n", y+r, x+c );
                    }
                    if( (cbin < 0 && rbin < 0) || (cbin > 0 && rbin > 0) )
                    {
                        c3 += nimage.getAsFloat( c+radius, r+radius );
//                        mrpt::system::os::fprintf( f5, "%d %d\n", y+r, x+c );
                    }
                    else
                    {
                        c3 -= nimage.getAsFloat( c+radius, r+radius );
//                        mrpt::system::os::fprintf( f6, "%d %d\n", y+r, x+c );
                    }
				} // end-if
            } // end
//            else
//                mrpt::system::os::fprintf( f11, "%d %d\n", y+r, x+c );
        } // end-for
//    mrpt::system::os::fclose(f1);
//    mrpt::system::os::fclose(f2);
//    mrpt::system::os::fclose(f3);
//    mrpt::system::os::fclose(f4);
//    mrpt::system::os::fclose(f5);
//    mrpt::system::os::fclose(f6);
//    mrpt::system::os::fclose(f7);
//    mrpt::system::os::fclose(f8);
//    mrpt::system::os::fclose(f9);
//    mrpt::system::os::fclose(f10);
//    mrpt::system::os::fclose(f11);

//    mrpt::system::pause();

    if( opts.computeHashCoeffs )
    {
        hashCoeffs.resize(3);
        hashCoeffs[0] = round(c1);
        hashCoeffs[1] = round(c2);
        hashCoeffs[2] = round(c3);

//        cout << "Hash Coeffs: " << hashCoeffs << endl;
    }

    // Normalize
    tlogger.enter("normalize");
    double sum = 0.0;
    for( unsigned int ii = 0; ii < oris.size(); ++ii )
        sum += oris[ii]*oris[ii];
    sum = 1/sqrt(sum);
    for( unsigned int ii = 0; ii < oris.size(); ++ii )
        oris[ii] *= sum;

    // Crop to "crop_value"
    for( unsigned int ii = 0; ii < oris.size(); ++ii )
        oris[ii] = min(opts.cropValue,oris[ii]);

    // Normalize again -> we have the descriptor!
    for( unsigned int ii = 0; ii < oris.size(); ++ii )
        sum += oris[ii]*oris[ii];
    sum = 1/sqrt(sum);
    for( unsigned int ii = 0; ii < oris.size(); ++ii )
        oris[ii] *= sum;

    // Convert it to std::vector<int>
    descriptor.resize( oris.size() );
    for( unsigned int ii = 0; ii < descriptor.size(); ++ii )
        descriptor[ii] = (int)(255.0f*oris[ii]);

    tlogger.leave("normalize");

    MRPT_END
} // end vision::computeHistogramOfOrientations

/*-------------------------------------------------------------
					matchMultiResolutionFeatures
-------------------------------------------------------------*/
TMultiResMatchingOutput vision::matchMultiResolutionFeatures(
                const CFeatureList                  & list1,
                CFeatureList                        & list2,
                const CImage                        & rightImage,
                const TMultiResDescMatchOptions     & matchOpts,
                const TMultiResDescOptions          & computeOpts )
{
    MRPT_START
    // "List1" has a set of features with their depths computed
    // "List2" has a set of FAST features detected in the next frame (with their depths)
    // We perform a prediction of the "List1" features and try to find its matches within List2
    // --------------------------------------------------------
    // Algortihm summary:
    // --------------------------------------------------------
    // For each feature in List1 we find a search region: by now a fixed window e.g. 20x20 pixels
    // TO DO: Non-maximal supression according to the feature score
    // From the remaining set, we compute the main orientation and check its consistency with the List1 feature
    // NEW: compute the corresponding feature in right image and compute its depth
    // From the remaining set, we compute the descriptor at scale 1.0 and determine the set of scales where to look for
    // If the distance between descriptors is below a certain threshold, we've found a match.
    // --------------------------------------------------------

    // General variables
    CTimeLogger logger;
    logger.disable();

    TMultiResMatchingOutput output;

    // Preliminary tasks
    output.firstListCorrespondences.resize(list1.size(),FEAT_FREE);
    output.secondListCorrespondences.resize(list2.size(),FEAT_FREE);
    output.firstListFoundScales.resize(list1.size(),-1);
    output.firstListDistance.resize(list1.size(),1.0);

    // Local variables
    int hWindow = matchOpts.searchAreaSize/2;               // Half of the search window width
    int mxsize = hWindow+2*matchOpts.lastSeenThreshold;
    int imageW = rightImage.getWidth();                     // Image width
    int imageH = rightImage.getHeight();                    // Image height
    int leftFeatCounter = 0;
    int rightFeatCounter = 0;                               // Counter for features
    int patchSize = computeOpts.basePSize;
    int minScale;
    double maxResponse;
    output.nMatches = 0;
    int ridx = 0;

    CFeatureList::const_iterator it1;
    CFeatureList::iterator it2;
//    cout << "Starting the loop" << endl;
    cout << endl;
    for( leftFeatCounter = 0, it1 = list1.begin(); it1 != list1.end(); ++it1, ++leftFeatCounter )
    {
        if( !(*it1)->descriptors.hasDescriptorMultiSIFT() )
            continue;
//        cout << (*it1)->ID << " with " << endl;
        double sRegionX0 = max( (*it1)->x-min((hWindow+2*(*it1)->nTimesLastSeen), mxsize), 0.0f);
        double sRegionX1 = min( (*it1)->x+min((hWindow+2*(*it1)->nTimesLastSeen), mxsize), (float)imageW );
        double sRegionY0 = max( (*it1)->y-min((hWindow+2*(*it1)->nTimesLastSeen), mxsize), 0.0f);
        double sRegionY1 = min( (*it1)->y+min((hWindow+2*(*it1)->nTimesLastSeen), mxsize), (float)imageH );

        int minDist = 1e6;
        //int minIdx  = -1;
        minScale = -1;
        maxResponse = 0;
        ridx = 0;

        for( rightFeatCounter = 0, it2 = list2.begin(); it2 != list2.end(); ++it2, ++rightFeatCounter )
        {
//            if( (*it2)->ID == 193 )
//            {
//                cout << (*it1)->ID << " with " << (*it2)->ID << endl;
//                mrpt::system::pause();
//            }


            // FILTER 1: Search region
            if( (*it2)->x < sRegionX0 || (*it2)->x > sRegionX1 || (*it2)->y < sRegionY0 || (*it2)->y > sRegionY1 )
            {
//                if( (*it2)->ID == 193 )
//                {
//                    cout << (*it2)->ID << " OOB" << endl;
//                }
                continue;
            }

//            cout << "     " << (*it2)->ID << endl;

            // Compute main orientations
//            logger.enter("cmorientation");
//            (*it2)->multiScales.push_back( 1.0 );
//            (*it2)->multiOrientations.resize( 1 );
//            (*it2)->descriptors.multiSIFTDescriptors.resize( 1 );
//            (*it2)->multiHashCoeffs.resize( 1 );
            vector<double> thisOris;
            if( !vision::computeMainOrientations( rightImage, (*it2)->x, (*it2)->y, patchSize, thisOris, computeOpts.sg2 )  )
            {
//                if( (*it2)->ID == 193 )
//                {
//                    cout << (*it2)->ID << " bad orientation computed" << endl;
//                    mrpt::system::pause();
//                }
                continue;
            }
//            logger.leave("cmorientation");

            // Compute the proper scales where to look for
            int firstScale, lastScale;
            if( matchOpts.useDepthFilter )
                vision::setProperScales( (*it1), (*it2), firstScale, lastScale );
            else
            {
                firstScale = max( 0, (int)matchOpts.lowScl1 );
                lastScale = min( (int)(*it1)->multiScales.size()-1, (int)matchOpts.highScl1 );
            }

//            if( (*it2)->ID == 193 )
//            {
//                cout << "Searching in scales: " << firstScale << " to " << lastScale << endl;
//                mrpt::system::pause();
//            }

            // Search for consistency of the orientation
            for( int k1 = firstScale; k1 <= lastScale; ++k1 )
                for( int k2 = 0; k2 < (int)(*it1)->multiOrientations[k1].size(); ++k2 )
                    for( int k3 = 0; k3 < (int)thisOris.size(); ++k3 )  // FILTER 2: Orientation
                        if( fabs( (*it1)->multiOrientations[k1][k2] - thisOris[k3] ) < matchOpts.oriThreshold ||
                            fabs( M_2PI - fabs( (*it1)->multiOrientations[k1][k2] - thisOris[k3] ) ) < matchOpts.oriThreshold )
                        {
                            // Orientation check passed
                            // FILTER 3: Feature response
                            // has it a better score than its 8 neighbours?
//                            logger.enter("non-max");
                            // TO DO: search the maximum number of neighbours up to 8
                            bool isMax = true;
                            if( list2.size() >= 8 )
                            {
                                std::vector< size_t > out_idx;
                                std::vector< float > out_dist_sqr;
                                maxResponse = (*it2)->response;

                                list2.kdTreeNClosestPoint2DIdx(	(*it2)->x, (*it2)->y, 8, out_idx, out_dist_sqr );
                                for( size_t kdcounter = 0; kdcounter < out_idx.size(); ++kdcounter )
                                {
                                    if( out_dist_sqr[kdcounter] > 1.4142 )
                                        continue;

                                    if( list2[out_idx[kdcounter]]->response > maxResponse )
                                    {
                                        maxResponse = list2[out_idx[kdcounter]]->response;
                                        isMax = false;
                                        break;
                                    }
                                }
                            } // end-if

                            if( !isMax )
                            {
                                cout << "NO ES MAX" << endl;
                                mrpt::system::pause();
                                continue;
                            }

                            // Candidate found at scale "k1" and orientation "k2" (list1) and orientation "k3" (list2)
                            // Compute descriptor for these conditions!

//                            vector<int> desc, hashC;
//                            TMultiResDescOptions auxOpts = computeOpts;
//                            auxOpts.computeHashCoeffs = false;

                            // All the filters have been passed -> we can add all the information into the feature
                            // If it has a previous computed descriptor it is substituted by this one
                            if( (*it2)->multiScales.size() == 0 )
                            {
                                (*it2)->multiScales.resize(1);
                                (*it2)->multiScales[0] = 1.0;
                                (*it2)->multiOrientations.resize(1);
                                (*it2)->descriptors.multiSIFTDescriptors.resize(1);
                                (*it2)->multiHashCoeffs.resize(1);
                            }

                            /* ORIENTATIONS */
                            bool oriFound = false;
                            int wh = 0;
                            for( int co = 0; co < (int)(*it2)->multiOrientations[0].size(); ++co )
                                if( (*it2)->multiOrientations[0][co] == thisOris[k3] )
                                {
                                    wh = co;
                                    oriFound = true;
                                }


                            int dist = 0;
                            if( !oriFound )     // The feature hasn't got this orientation already -> compute the descriptor & coeffs
                            {
//                                if( (*it2)->ID == 193 )
//                                    cout << "Orientation not found -> compute the descriptor." << endl;

                                (*it2)->multiOrientations[0].push_back( thisOris[k3] );

                                vector<int32_t> thisDesc, thisHash;
                                computeHistogramOfOrientations(
                                            rightImage,
                                            (*it2)->x, (*it2)->y,
                                            patchSize,
                                            thisOris[k3],
                                            thisDesc,
                                            computeOpts,
                                            thisHash );

                                /* DESCRIPTORS */
                                (*it2)->descriptors.multiSIFTDescriptors[0].push_back( thisDesc );
                                /* HASH COEFFICIENTS */
                                (*it2)->multiHashCoeffs[0].push_back( thisHash );

                                for( int n = 0; n < int(thisDesc.size()); n++ )
                                    dist += square( (*it1)->descriptors.multiSIFTDescriptors[k1][k2][n] - thisDesc[n] );
                            } // end if
                            else
                            {
//                                if( (*it2)->ID == 193 )
//                                    cout << "Skipping descriptor computation." << endl;

                                for( int n = 0; n < (int)(*it2)->descriptors.multiSIFTDescriptors[0][wh].size(); n++ )
                                    dist += square( (*it1)->descriptors.multiSIFTDescriptors[k1][k2][n] - (*it2)->descriptors.multiSIFTDescriptors[0][wh][n] );
                            } // end else

//                            if( PARAR && (*it2)->ID == 193 )
//                            {
//                                (*it2)->dumpToConsole();
//                                mrpt::system::pause();
//                            }

                            // FILTER 4 for the matching: Descriptor distance
                            if( dist < minDist )
                            {
                                minDist     = dist;
                                ridx        = rightFeatCounter;
                                maxResponse = (*it2)->response;
                                minScale    = k1;

//                                minauxfscale = auxfscale;
//                                minauxlscale = auxlscale;

//                                mindepth1    = (*it1)->depth;
//                                mindepth2    = (*it2)->depth;
                            }

//                            if( (*it2)->ID == 193 )
//                            {
//                                cout << "Matching entre 193 y "<< (*it2)->ID << " entre escalas: " << firstScale << " y " << lastScale << endl;
//                                cout << "OR1: " << (*it1)->multiOrientations[k1][k2] << " OR2: " << (*it2)->multiOrientations[0][k3];
//                                cout << "SCL: " << (*it1)->multiScales[k1] << " DIST: " << dist << endl;
//                            }

                        } // end if
        } // end for it2
        if( minDist < matchOpts.matchingThreshold )
        {
            // AVOID COLLISIONS IN A GOOD MANNER
            int auxIdx = output.secondListCorrespondences[ ridx ];
		    if( auxIdx != FEAT_FREE && minDist < output.firstListDistance[ auxIdx ] )
		    {
                // We've found a better match
                output.firstListDistance[ leftFeatCounter ]         = minDist;
                output.firstListCorrespondences[ leftFeatCounter ]  = ridx;
                output.secondListCorrespondences[ ridx ]            = leftFeatCounter;
                output.firstListFoundScales[ leftFeatCounter ]      = minScale;

                // Undo the previous one
                output.firstListDistance[ auxIdx ]                  = 1.0;
                output.firstListCorrespondences[ auxIdx ]           = FEAT_FREE;
                output.firstListFoundScales[ auxIdx ]               = -1;

//                cout << "Better match at scale: " << minScale << endl;
		    } // end-if
		    else
		    {
////		        cout << "New match found: [R] " << minRightIdx << " with [L] " << minLeftIdx << "(" << minVal << ")" << endl;
		        output.secondListCorrespondences[ ridx ]            = leftFeatCounter;
		        output.firstListCorrespondences[ leftFeatCounter ]  = ridx;
		        output.firstListDistance[ leftFeatCounter ]         = minDist;
		        output.firstListFoundScales[ leftFeatCounter ]      = minScale;
		        output.nMatches++;
//		        cout << "Match found at scale: " << minScale << endl;
		    }
            // Match found
//            leftMatchingIdx.push_back( leftFeatCounter );
//            rightMatchingIdx.push_back( minIdx );
//            outScales.push_back( minScale );
//
//            cout << "Match found: [" << leftFeatCounter << "," << minIdx;
//            cout << "] at scale " << minScale << " [" << minauxfscale << "," << minauxlscale << "]";
//            cout << " with depths: [" << mindepth1 << "," << mindepth2 << "]" << endl;
        } // end if
    } // end for it1

    return output;
    MRPT_END
} // end matchMultiResolutionFeatures

/*-------------------------------------------------------------
					matchMultiResolutionFeatures
-------------------------------------------------------------*/
int  vision::matchMultiResolutionFeatures(
                CMatchedFeatureList                 & mList1,
                CMatchedFeatureList                 & mList2,
                const CImage                        & leftImage,
                const CImage                        & rightImage,
                const TMultiResDescMatchOptions     & matchOpts,
                const TMultiResDescOptions          & computeOpts )
{
    MRPT_START
    // "mList1" has a set of matched features with their depths computed
    // "mList2" has a set of matched FAST features detected in the next frame at scale 1.0 (with their depths)
    // We perform a prediction of the "mList1" features and try to find its matches within mList2
    // --------------------------------------------------------
    // Algortihm summary:
    // --------------------------------------------------------
    // For each feature in List1 we find a search region: by now a fixed window e.g. 20x20 pixels
    // TO DO: Non-maximal supression according to the feature score
    // From the remaining set, we compute the main orientation and check its consistency with the List1 feature
    // NEW: compute the corresponding feature in right image and compute its depth
    // From the remaining set, we compute the descriptor at scale 1.0 and determine the set of scales where to look for
    // If the distance between descriptors is below a certain threshold, we've found a match.
    // --------------------------------------------------------

    // General variables
    CTimeLogger logger;
    //logger.disable();

    ASSERT_( mList1.size() > 0 && mList2.size() > 0 );

    CFeatureList baseList1, baseList2;
    mList1.getBothFeatureLists( baseList1, baseList2 );

    CFeatureList auxList1, auxList2;
    mList2.getBothFeatureLists( auxList1, auxList2 );

    vector<int> scales1, scales2;

    TMultiResMatchingOutput output1, output2;

    // Left image
    output1 = matchMultiResolutionFeatures( baseList1, auxList1, leftImage, matchOpts, computeOpts );
    updateBaseList( baseList1, auxList1, output1.firstListCorrespondences );     //Update counters
    //updateBaseList(leftIdx2,auxList1);

//    CImage auxImg1, auxImg2;
//    auxImg1 = leftImage;
//    int hwsize = matchOpts.searchAreaSize/2;
//    for( int k = 0; k < leftIdx1.size(); ++k )
//        if( leftIdx1[k] != FEAT_FREE )
//        {
//            auxImg1.cross( baseList1[k]->x, baseList1[k]->y, TColor::red, '+' );
//            auxImg1.textOut( baseList1[k]->x, baseList1[k]->y, format("%d", scales1[k]), TColor::red );
//            auxImg1.rectangle( baseList1[k]->x-hwsize, baseList1[k]->y-hwsize, baseList1[k]->x+hwsize, baseList1[k]->y+hwsize, TColor::red );
//        }
//    win1.showImage( auxImg1 );

    // Right image
    output2 = matchMultiResolutionFeatures( baseList2, auxList2, rightImage, matchOpts, computeOpts );
    updateBaseList(baseList2, auxList2, output2.firstListCorrespondences);    //Update counters
    //updateBaseList(rightIdx2,auxList2);

//    auxImg2 = rightImage;
//    for( int k = 0; k < rightIdx1.size(); ++k )
//        if( rightIdx1[k] != FEAT_FREE )
//        {
//            auxImg2.cross( baseList2[k]->x, baseList2[k]->y, TColor::red, '+' );
//            auxImg2.textOut( baseList2[k]->x, baseList2[k]->y, format("%d", scales2[k]), TColor::red );
//            auxImg2.rectangle( baseList2[k]->x-hwsize, baseList2[k]->y-hwsize, baseList2[k]->x+hwsize, baseList2[k]->y+hwsize, TColor::red );
//        }
    //win2.showImage( auxImg2 );
    //mrpt::system::pause();

    cout << "Left matches: " << output1.nMatches << " out of " << baseList1.size() << "," << auxList1.size() << endl;
    cout << "Right matches: " << output2.nMatches << " out of " << baseList2.size() << "," << auxList2.size() << endl;
    cout << "Matched list: " << mList1.size() << endl;

//    for(int k = 0; k < auxList2.size(); ++k)
//    {
//        cout << k;
//        auxList2[k]->dumpToConsole();
//    }
//    mrpt::system::pause();

    CMatchedFeatureList::iterator itMatch;
    int m;
    //int cnt = 0;
    for( m = 0, itMatch = mList1.begin(); itMatch != mList1.end(); ++m )
    {

        //cout << m << " " << endl;
        if( itMatch->first->nTimesLastSeen > matchOpts.lastSeenThreshold || itMatch->second->nTimesLastSeen > matchOpts.lastSeenThreshold )
        {
//            cnt++;
//            cout << "feature " << m << " must be deleted" << endl;
            itMatch = mList1.erase( itMatch );
        }
        else
        {
            // We've found a tracked match
            // We have found the match in both frames! Check out the scales!
            int tidx = output1.firstListCorrespondences[m];
            if( tidx != FEAT_FREE )
            {
                if( scales1[m] == 0 )
                {
                    cout << "Left feature " << m << " found in scale 0!" << endl;
                    int res = computeMoreDescriptors( leftImage, auxList1[tidx], itMatch->first, true, computeOpts );
                    if( res == 0) cout << "LF LOWSCALES Out of bounds!!" << endl;
                    //mrpt::system::pause();
                }
                else
                {
                    int nScales = (int)itMatch->first->multiScales.size();
                    if( scales1[m] == nScales-1 )
                    {
                        cout << "Left feature " << m << " found in last scale!" << endl; //computeMoreDescriptors( auxList1[k1], leftImage, false, itMatch->first );
                        cout << "tidx=" << tidx << endl;
                        int res = computeMoreDescriptors( leftImage, auxList1[tidx], itMatch->first, false, computeOpts );
                        if( res == 0) cout << "LF HIGHSCALES Out of bounds!!" << endl;
    //                    mrpt::system::pause();
                    }
                }
            } // end-if

//            cout << "Left: " << m << " with " <<  << " at scale: " << output1.firstListFoundScales[m] << endl;
//            cout << "Right: " << m << " with " << output2.firstListCorrespondences[m] << " at scale: " << output2.firstListFoundScales[m] << endl;
            tidx = output2.firstListCorrespondences[m];
            if( tidx != FEAT_FREE )
            {
                if( scales2[m] == 0 )
                {
                    cout << "Right feature " << m << " found in scale 0!" << endl; //computeMoreDescriptors( auxList2[k2], rightImage, true, itMatch->second );
                    int res = computeMoreDescriptors( rightImage, auxList2[tidx], itMatch->second, true, computeOpts );
                    if( res == 0) cout << "RF LOWSCALES Out of bounds!!" << endl;
                    //mrpt::system::pause();
                }
                else
                {
                    int nScales = (int)itMatch->second->multiScales.size();
                    if( scales2[m] == nScales-1 )
                    {
                        cout << "Right feature " << m << " found in scale!" << endl; //computeMoreDescriptors( auxList2[k2], rightImage, false, itMatch->second );
                        int res = computeMoreDescriptors( rightImage, auxList2[tidx], itMatch->second, false, computeOpts );
                        if( res == 0) cout << "RF HIGHSCALES Out of bounds!!" << endl;
    //                    mrpt::system::pause();
                    }
                }
            } // end-else
            itMatch++;
        } // end-else
    } // end-for
//    cout << cnt << " features would be deleted." << endl;
    return mList1.size();
    MRPT_END
} // end matchMultiResolutionFeatures

/*-------------------------------------------------------------
					computeMoreDescriptors
-------------------------------------------------------------*/
int vision::computeMoreDescriptors(
                const CImage                        & image,
                const CFeaturePtr                   & inputFeat,
                CFeaturePtr                         & outputFeat,
                const bool                          & lowerScales,
                const TMultiResDescOptions          & opts )
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x211

    MRPT_START
    //**************************************************************************
    // Pre-smooth the image with sigma = sg1 (typically 0.5)
    //**************************************************************************
    cv::Mat tempImg1;
    IplImage aux1;

    const cv::Mat inImg1 = cv::cvarrToMat(image.getAs<IplImage>(),false /*dont copy data*/);

    cv::GaussianBlur( inImg1, tempImg1, cvSize(0,0), opts.sg1 /*sigmaX*/, opts.sg1 /*sigmaY*/ );
    aux1 = tempImg1;
    CImage smLeftImg( &aux1 );
    //--------------------------------------------------------------------------

    unsigned int a = opts.basePSize;

    int largestSize     = lowerScales ? round(a*0.8) : round(a*2.0);
    largestSize         = largestSize%2 ? largestSize : largestSize+1;          // round to the closest odd number
    int hLargestSize    = largestSize/2;

    unsigned int npSize, hpSize;

    // We don't take into account the matching if it is too close to the image border (the largest patch cannot be extracted)
    if( inputFeat->x+hLargestSize > smLeftImg.getWidth()-1 || inputFeat->x-hLargestSize < 0 ||
        inputFeat->y+hLargestSize > smLeftImg.getHeight()-1 || inputFeat->y-hLargestSize < 0 )
            return 0;

    //int iniScale = 0, endScale = 0;
    if( lowerScales )
    {
        vector<double> thisScales(2);
        thisScales[0] = 0.5;
        thisScales[1] = 0.8;

        double baseScale = outputFeat->multiScales[0];

//        cout << " :: Lower scales" << endl;
        outputFeat->multiScales.push_front( thisScales[1]*baseScale );
        outputFeat->multiScales.push_front( thisScales[0]*baseScale );

//        iniScale = 0;
//        endScale = 2;

        for( int k = 1; k >= 0; --k )
        {
            npSize = round( a*thisScales[k] );
            npSize = npSize%2 ? npSize : npSize+1;          // round to the closest odd number
            hpSize = npSize/2;                              // half size of the patch

            CImage tPatch;
            // LEFT IMAGE:
            if( npSize )
                smLeftImg.extract_patch( tPatch, inputFeat->x-hpSize, inputFeat->y-hpSize, npSize, npSize );

            cv::Mat out_mat_patch;
            // The size is a+2xa+2 because we have to compute the gradient (magnitude and orientation) in every pixel within the axa patch so we need
            // one more row and column. For instance, for a 23x23 patch we need a 25x25 patch.
            cv::resize( cv::cvarrToMat(tPatch.getAs<IplImage>(),false ), out_mat_patch, cv::Size(a+2,a+2) );
            IplImage aux_img = IplImage(out_mat_patch);
            CImage rsPatch( &aux_img );

//            cout << " ::: Patch extracted and resized" << endl;
            vector<double> auxOriVector;
            if( !vision::computeMainOrientations( rsPatch, a/2+1, a/2+1, a, auxOriVector, opts.sg2 ))
            {
                cout << "computeMainOrientations returned false" << endl;
                mrpt::system::pause();
            }

//            cout << " ::: Orientation computed" << endl;
            vector< vector<int32_t> > auxDescVector;
            vector< vector<int32_t> > auxHashCoeffs;
            auxDescVector.resize( auxOriVector.size() );
            auxHashCoeffs.resize( auxOriVector.size() );
            for( unsigned int m = 0; m < auxOriVector.size(); ++m )
            {
//                cout << " :::: Descriptor for orientation " << auxOriVector[m];
                computeHistogramOfOrientations(
                            rsPatch,
                            a/2+1, a/2+1, a,
                            auxOriVector[m],
                            auxDescVector[m], opts,
                            auxHashCoeffs[m] );
//                cout << " ...done" << endl;
            } // end-for
            outputFeat->multiOrientations.push_front( auxOriVector );
            outputFeat->descriptors.multiSIFTDescriptors.push_front( auxDescVector );
            outputFeat->multiHashCoeffs.push_front( auxHashCoeffs );
        } // end-for
    }
    else
    {
//        cout << " :: Higher scales" << endl;
        vector<double> thisScales(4);
        thisScales[0] = 1.2;
        thisScales[1] = 1.5;
        thisScales[2] = 1.8;
        thisScales[3] = 2.0;

        size_t nCurrScales = outputFeat->multiScales.size();
        outputFeat->multiScales.push_back( thisScales[0]*outputFeat->multiScales[nCurrScales-1] );
        outputFeat->multiScales.push_back( thisScales[1]*outputFeat->multiScales[nCurrScales-1] );
        outputFeat->multiScales.push_back( thisScales[2]*outputFeat->multiScales[nCurrScales-1] );
        outputFeat->multiScales.push_back( thisScales[3]*outputFeat->multiScales[nCurrScales-1] );

//        for( int k = nCurrScales; k < (int)outputFeat->multiScales.size(); ++k )
        for( int k = 0; k < (int)thisScales.size(); ++k )
        {
//            cout << " ::: For scale " << k+nCurrScales << endl;

            npSize = round( a*thisScales[k] );
            npSize = npSize%2 ? npSize : npSize+1;          // round to the closest odd number
            hpSize = npSize/2;                              // half size of the patch

            CImage tPatch;

            // LEFT IMAGE:
            smLeftImg.extract_patch( tPatch, inputFeat->x-hpSize, inputFeat->y-hpSize, npSize, npSize );

            cv::Mat out_mat_patch;
            // The size is a+2xa+2 because we have to compute the gradient (magnitude and orientation) in every pixel within the axa patch so we need
            // one more row and column. For instance, for a 23x23 patch we need a 25x25 patch.
            cv::resize( cv::cvarrToMat( tPatch.getAs<IplImage>(),false), out_mat_patch, cv::Size(a+2,a+2) );
            IplImage aux_img = IplImage(out_mat_patch);
            CImage rsPatch( &aux_img );

//            cout << " ::: Patch extracted and resized" << endl;

            vector<double> auxOriVector;
            vision::computeMainOrientations(
                            rsPatch, a/2+1, a/2+1, a,
                            auxOriVector, opts.sg2 );

//            cout << " ::: Orientation computed" << endl;

            vector< vector<int32_t> > auxDescVector;
            vector< vector<int32_t> > auxCoefVector;
            auxDescVector.resize( auxOriVector.size() );
            auxCoefVector.resize( auxOriVector.size() );
            for( unsigned int m = 0; m < auxOriVector.size(); ++m )
            {
//                cout << " :::: Descriptor for orientation " << auxOriVector[m];
                computeHistogramOfOrientations(
                            rsPatch,
                            a/2+1, a/2+1, a,
                            auxOriVector[m],
                            auxDescVector[m], opts,
                            auxCoefVector[m] );
//                cout << " ...done" << endl;
            } // end-for
            outputFeat->multiOrientations.push_back( auxOriVector );
            outputFeat->descriptors.multiSIFTDescriptors.push_back( auxDescVector );
            outputFeat->multiHashCoeffs.push_back( auxCoefVector );
        } // end-for
    } // end else
    ASSERT_( outputFeat->descriptors.hasDescriptorMultiSIFT() );
    return 1;
    MRPT_END
#else
	THROW_EXCEPTION("This function needs OpenCV 2.1+")
	return 0;
#endif
} // end-computeMoreDescriptors

/*-------------------------------------------------------------
					setProperScales
-------------------------------------------------------------*/
void vision::setProperScales(
                const CFeaturePtr                   & feat1,
                const CFeaturePtr                   & feat2,
                int                                 & firstScale,
                int                                 & lastScale )
{
    // Find the range of scales (of those within feat1) where the descriptor should be matched
    // For the feat1 we use "initialDepth" since all the scales are relative to this depth while for the
    // feat2 it is used "depth" which is the actual scale of the feature.
    MRPT_START
    int numScales = int(feat1->multiScales.size());
//    if( numScales <= 1 )
//    {
//        cout << "BAD NUMBER OF SCALES: " << endl;
//        feat1->dumpToConsole();
//        feat2->dumpToConsole();
        ASSERT_( numScales > 1 );
//    }

    firstScale = 0;
    lastScale  = numScales-1;

    // Determine the range of scale where to look for in the list1
    double smin = (feat2->depth-0.15*feat1->initialDepth)/feat1->initialDepth;
    double smax = (feat2->depth+0.15*feat1->initialDepth)/feat1->initialDepth;

    if( smin <= feat1->multiScales[1] )
        firstScale = 0;
    else
    {
        if( smin > feat1->multiScales[numScales-2] )
            firstScale = numScales-2;
        else    // it is in between the limits
        {
            for( int k = 1; k <= numScales-3; ++k )
                if( smin > feat1->multiScales[k] )
                    firstScale = k;
        } // end else
    } // end else

    if( smax <= feat1->multiScales[1] )
        lastScale = 1;
    else
    {
        if( smax > feat1->multiScales[numScales-2] )
            lastScale = numScales-1;
        else
        {
            for( int k = 1; k <= numScales-3; ++k )
                if( smax <= feat1->multiScales[k] )
                {
                    lastScale = k;
                    break;
                } // end if
        } // end else
    } // end else

    ASSERT_( firstScale >= 0 && lastScale < numScales && firstScale < lastScale );
    MRPT_END
} // end setProperScales

/*-------------------------------------------------------------
                computeMultiResolutionDescriptors
-------------------------------------------------------------*/
void vision::computeMultiResolutionDescriptors(
                const CImage                        & imageLeft,
                const CImage                        & imageRight,
                CMatchedFeatureList                 & matchedFeats,
                const TMultiResDescOptions          & opts )
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x211

    MRPT_START
    CTimeLogger tlogger;
    tlogger.disable();
    ASSERT_( matchedFeats.size() > 0 );

    //**************************************************************************
    // Pre-smooth the image with sigma = sg1 (typically 0.5)
    //**************************************************************************
    tlogger.enter("smooth");
    cv::Mat tempImg1, tempImg2;
    IplImage aux1, aux2;

    const cv::Mat inImg1 = cv::cvarrToMat(imageLeft.getAs<IplImage>());
    const cv::Mat inImg2 = cv::cvarrToMat(imageRight.getAs<IplImage>());

    cv::GaussianBlur( inImg1, tempImg1, cvSize(0,0), opts.sg1 /*sigmaX*/, opts.sg1 /*sigmaY*/ );
    aux1 = tempImg1;
    CImage smLeftImg( &aux1 );

    cv::GaussianBlur( inImg2, tempImg2, cvSize(0,0), opts.sg1 /*sigmaX*/, opts.sg1 /*sigmaY*/ );
    aux2 = tempImg2;
    CImage smRightImg( &aux2 );
    tlogger.leave("smooth");
    //--------------------------------------------------------------------------

    unsigned int a              = opts.basePSize;
    unsigned int feat_counter   = 0;
    unsigned int good_matches   = 0;

    int largestSize = round(a*opts.scales[opts.scales.size()-1]);
    largestSize = largestSize%2 ? largestSize : largestSize+1;          // round to the closest odd number
    int hLargestSize = largestSize/2;

    unsigned int npSize;
    unsigned int hpSize;

    for( CMatchedFeatureList::iterator itMatch = matchedFeats.begin(); itMatch != matchedFeats.end(); ++itMatch, ++feat_counter )
    {
        // We don't take into account the matching if it is too close to the image border (the largest patch cannot be extracted)
        if( itMatch->first->x+hLargestSize > smLeftImg.getWidth()-1 || itMatch->first->x-hLargestSize < 0 ||
            itMatch->first->y+hLargestSize > smLeftImg.getHeight()-1 || itMatch->first->y-hLargestSize < 0 ||
            itMatch->second->x+hLargestSize > smRightImg.getWidth()-1 || itMatch->second->x-hLargestSize < 0 ||
            itMatch->second->y+hLargestSize > smRightImg.getHeight()-1 || itMatch->second->y-hLargestSize < 0 )
                continue;

        // We have found a proper match to obtain the multi-descriptor
        // Compute the depth and store the coords:
        tlogger.enter("compute depth");
        if( opts.computeDepth )
        {
            double disp = itMatch->first->x - itMatch->second->x;
            double aux  = opts.baseline/disp;
            double x3D  = (itMatch->first->x-opts.cx)*aux;
            double y3D  = (itMatch->first->y-opts.cy)*aux;
            double z3D  = opts.fx*aux;

            itMatch->first->depth     = sqrt( x3D*x3D + y3D*y3D + z3D*z3D );
            itMatch->second->depth    = sqrt( (x3D-opts.baseline)*(x3D-opts.baseline) + y3D*y3D + z3D*z3D );
        }
        tlogger.leave("compute depth");

        tlogger.enter("cp scales");
        itMatch->first->multiScales.resize( opts.scales.size() );
        itMatch->first->multiOrientations.resize( opts.scales.size() );
        itMatch->first->descriptors.multiSIFTDescriptors.resize( opts.scales.size() );
        itMatch->first->multiHashCoeffs.resize( opts.scales.size() );

        itMatch->second->multiScales.resize( opts.scales.size() );
        itMatch->second->multiOrientations.resize( opts.scales.size() );
        itMatch->second->descriptors.multiSIFTDescriptors.resize( opts.scales.size() );
        itMatch->second->multiHashCoeffs.resize( opts.scales.size() );

        // Copy the scale values within the feature.
        memcpy( &itMatch->first->multiScales[0], &opts.scales[0], opts.scales.size()*sizeof(double) );
        memcpy( &itMatch->second->multiScales[0], &opts.scales[0], opts.scales.size()*sizeof(double) );
        tlogger.leave("cp scales");

        // For each of the involved scales
        for( unsigned int k = 0; k < opts.scales.size(); ++k )
        {
            npSize = round(a*opts.scales[k]);
            npSize = npSize%2 ? npSize : npSize+1;          // round to the closest odd number
            hpSize = npSize/2;                              // half size of the patch

            CImage tPatch;

            // LEFT IMAGE:
            tlogger.enter("extract & resize");
            smLeftImg.extract_patch( tPatch, itMatch->first->x-hpSize, itMatch->first->y-hpSize, npSize, npSize );

            cv::Mat out_mat_patch;
            // The size is a+2xa+2 because we have to compute the gradient (magnitude and orientation) in every pixel within the axa patch so we need
            // one more row and column. For instance, for a 23x23 patch we need a 25x25 patch.
            cv::resize( cv::cvarrToMat( tPatch.getAs<IplImage>(),false ), out_mat_patch, cv::Size(a+2,a+2) );
            IplImage aux_img = IplImage(out_mat_patch);
            CImage rsPatch( &aux_img );
            tlogger.leave("extract & resize");

            tlogger.enter("main orientations");
            // Compute the main orientations for the axa patch, taking into account that the actual patch has a size of a+2xa+2
            // (being the center point the one with coordinates a/2+1,a/2+1).
            // A sigma = opts.sg2 is used to smooth the entries in the orientations histograms
            // Orientation vector will be resize inside the function, so don't reserve space for it
            vision::computeMainOrientations(
                            rsPatch, a/2+1, a/2+1, a,
                            itMatch->first->multiOrientations[k], opts.sg2 );
            tlogger.leave("main orientations");

            size_t nMainOris = itMatch->first->multiOrientations[k].size();
            itMatch->first->descriptors.multiSIFTDescriptors[k].resize( nMainOris );
            for( unsigned int m = 0; m < nMainOris; ++m )
            {
                tlogger.enter("compute histogram");
                computeHistogramOfOrientations(
                            rsPatch,
                            a/2+1, a/2+1, a,
                            itMatch->first->multiOrientations[k][m],
                            itMatch->first->descriptors.multiSIFTDescriptors[k][m], opts,
                            itMatch->first->multiHashCoeffs[k][m] );
                tlogger.leave("compute histogram");
            } // end for

            // RIGHT IMAGE:
            tlogger.enter("extract & resize");
            imageRight.extract_patch( tPatch, itMatch->second->x-hpSize, itMatch->second->y-hpSize, npSize, npSize );

            cv::resize( cv::cvarrToMat( tPatch.getAs<IplImage>(),false), out_mat_patch, cv::Size(a+2,a+2) );
            IplImage aux_img2 = IplImage(out_mat_patch);
            CImage rsPatch2( &aux_img2 );
            tlogger.leave("extract & resize");

            tlogger.enter("main orientations");
            vision::computeMainOrientations(
                                rsPatch2, a/2+1, a/2+1, a,
                                itMatch->second->multiOrientations[k], opts.sg2 );
            tlogger.leave("main orientations");

            nMainOris = itMatch->second->multiOrientations[k].size();
            itMatch->second->descriptors.multiSIFTDescriptors[k].resize( nMainOris );

            for( unsigned int m = 0; m < nMainOris; ++m )
            {
                tlogger.enter("compute histogram");
                computeHistogramOfOrientations(
                            rsPatch2,
                            a/2+1, a/2+1, a,
                            itMatch->second->multiOrientations[k][m],
                            itMatch->second->descriptors.multiSIFTDescriptors[k][m], opts,
                            itMatch->second->multiHashCoeffs[k][m] );
                tlogger.leave("compute histogram");
            } // end for
        } // end scales for
        good_matches++;
    } // end matches for
    MRPT_END
#else
	THROW_EXCEPTION("This function needs OpenCV 2.1+")
#endif
} // end-vision::computeMultiResolutionDescriptors

/*-------------------------------------------------------------
					computeMultiResolutionDescriptors
-------------------------------------------------------------*/
bool vision::computeMultiResolutionDescriptors(
                const CImage                        & image,
                CFeaturePtr                         & feat,
                const TMultiResDescOptions          & opts )
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x211

    MRPT_START
    int a           = opts.basePSize;
    int maxScale    = opts.scales.size();
    int h           = image.getHeight();
    int w           = image.getWidth();
    int npSize, hpSize;

    int largestSize     = round(a*opts.scales[maxScale-1]);
    largestSize         = largestSize%2 ? largestSize : largestSize+1;          // round to the closest odd number
    int hLargestSize    = largestSize/2;

    if( feat->x+hLargestSize > (w-1) || feat->x-hLargestSize < 0 ||
        feat->y+hLargestSize > (h-1) || feat->y-hLargestSize < 0 )
    {
        cout << endl << "[computeMultiResolutionDescriptors] WARNING: Feature is too close to the border. MultiDescriptor computation skipped." << endl;
        return false;
    }

    feat->multiScales.resize( maxScale );
    feat->multiOrientations.resize( maxScale );
    feat->descriptors.multiSIFTDescriptors.resize( maxScale );
    // If the hash coeffs have to be computed, resize the vector of hash coeffs.
    if( opts.computeHashCoeffs )
        feat->multiHashCoeffs.resize( maxScale );

    // Copy the scale values within the feature.
    for( int k = 0; k < maxScale; ++k )
        feat->multiScales[k] = opts.scales[k];

    // For each of the involved scales
    for( int k = 0; k < maxScale; ++k )
    {
        npSize = round( a*opts.scales[k] );
        npSize = npSize%2 ? npSize : npSize+1;          // round to the closest odd number
        hpSize = npSize/2;                              // half size of the patch

        CImage tPatch(npSize,npSize);

        // LEFT IMAGE:
//        if( feat->x+hpSize > image.getWidth()-1 || feat->y+hpSize > image.getHeight()-1 || feat->x-hpSize < 0 || feat->y-hpSize < 0 )
//            cout << "(" << feat->x << "," << feat->y << ") and hpSize: " << hpSize << " with scales ";
//            cout << opts.scales << " imSize: " << image.getWidth() << "x" << image.getHeight() << endl;
        image.extract_patch( tPatch, feat->x-hpSize, feat->y-hpSize, npSize, npSize );

        cv::Mat out_mat_patch;
        // The size is a+2xa+2 because we have to compute the gradient (magnitude and orientation) in every pixel within the axa patch so we need
        // one more row and column. For instance, for a 23x23 patch we need a 25x25 patch.
        cv::resize( cv::cvarrToMat(tPatch.getAs<IplImage>(),false), out_mat_patch, cv::Size(a+2,a+2) );
        IplImage aux_img = IplImage(out_mat_patch);
        CImage rsPatch( &aux_img );

        // Compute the main orientations for the axa patch, taking into account that the actual patch has a size of a+2xa+2
        // (being the center point the one with coordinates a/2+1,a/2+1).
        // A sigma = opts.sg2 is used to smooth the entries in the orientations histograms
        // Orientation vector will be resized inside the function, so don't reserve space for it
        vision::computeMainOrientations(
                        rsPatch, a/2+1, a/2+1, a,
                        feat->multiOrientations[k], opts.sg2 );

        size_t nMainOris = feat->multiOrientations[k].size();
        feat->descriptors.multiSIFTDescriptors[k].resize( nMainOris );
        if( opts.computeHashCoeffs )
           feat->multiHashCoeffs[k].resize( nMainOris );

        for( unsigned int m = 0; m < nMainOris; ++m )
        {
            if( opts.computeHashCoeffs )
            {
                computeHistogramOfOrientations(
                        rsPatch,
                        a/2+1, a/2+1, a,
                        feat->multiOrientations[k][m],
                        feat->descriptors.multiSIFTDescriptors[k][m],
                        opts,
                        feat->multiHashCoeffs[k][m] );
            } // end-if
            else
            {
                vector<int32_t> vec;
                computeHistogramOfOrientations(
                        rsPatch,
                        a/2+1, a/2+1, a,
                        feat->multiOrientations[k][m],
                        feat->descriptors.multiSIFTDescriptors[k][m],
                        opts,
                        vec );
            } // end-else
        } // end orientations for (m)
    } // end scales for (k)
    return true;
    MRPT_END
#else
	THROW_EXCEPTION("This function needs OpenCV 2.1+")
#endif
} // end-computeMultiResolutionDescriptors

/*-------------------------------------------------------------
					computeMultiResolutionDescriptors
-------------------------------------------------------------*/
vector<bool> vision::computeMultiResolutionDescriptors(
                const CImage                        & image,
                CFeatureList                        & list,
                const TMultiResDescOptions          & opts )
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x211
    MRPT_START
    CTimeLogger tlogger;
    tlogger.disable();
    ASSERT_( list.size() > 0 );
    CImage smLeftImg;
    if( opts.blurImage )
    {
        //**************************************************************************
        // Pre-smooth the image with sigma = sg1 (typically 0.5)
        //**************************************************************************
        cv::Mat tempImg;
        IplImage aux;

        const cv::Mat inImg = cv::cvarrToMat(image.getAs<IplImage>());

        cv::GaussianBlur( inImg, tempImg, cvSize(0,0), opts.sg1 /*sigmaX*/, opts.sg1 /*sigmaY*/ );
        aux = tempImg;
        smLeftImg.loadFromIplImage( &aux );
        //--------------------------------------------------------------------------
    }
    else
        smLeftImg = image;

    TMultiResDescOptions auxOpts    = opts;
    auxOpts.blurImage               = false;
    vector<bool> st( list.size() );
    int k = 0;
    for( CFeatureList::iterator it = list.begin(); it != list.end(); ++it, ++k )
        st[k] = computeMultiResolutionDescriptors( smLeftImg, (*it), auxOpts );
    return st;
    MRPT_END
#else
    THROW_EXCEPTION("This function needs OpenCV 2.1+")
#endif

} // end computeMultiResolutionDescriptors

/*-------------------------------------------------------------
					computeMultiResolutionDescriptors
-------------------------------------------------------------*/
void vision::computeMultiOrientations(
                const CImage                        & image,
                CFeatureList                        & list,
                const TMultiResDescOptions          & opts )
{
    #if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x211
    MRPT_START
    CTimeLogger tlogger;
    tlogger.disable();
    ASSERT_( list.size() > 0 );

    //**************************************************************************
    // Pre-smooth the image with sigma = sg1 (typically 0.5)
    //**************************************************************************
    tlogger.enter("smooth");
    cv::Mat tempImg1;
    IplImage aux1;

    const cv::Mat inImg1 = cv::cvarrToMat(image.getAs<IplImage>(),false);

    cv::GaussianBlur( inImg1, tempImg1, cvSize(0,0), opts.sg1 /*sigmaX*/, opts.sg1 /*sigmaY*/ );
    aux1 = tempImg1;
    CImage smLeftImg( &aux1 );
    //--------------------------------------------------------------------------

    unsigned int a = opts.basePSize;

    int largestSize = round(a*opts.scales[opts.scales.size()-1]);
    largestSize = largestSize%2 ? largestSize : largestSize+1;          // round to the closest odd number
    int hLargestSize = largestSize/2;

    unsigned int npSize;
    unsigned int hpSize;

    for( CFeatureList::iterator it = list.begin(); it != list.end(); ++it )
    {
        // We don't take into account the matching if it is too close to the image border (the largest patch cannot be extracted)
        if( (*it)->x+hLargestSize > smLeftImg.getWidth()-1 || (*it)->x-hLargestSize < 0 ||
            (*it)->y+hLargestSize > smLeftImg.getHeight()-1 || (*it)->y-hLargestSize < 0 )
                continue;

        // We have found a proper match to obtain the multi-descriptor
        tlogger.enter("cp scales");
        (*it)->multiScales.resize( opts.scales.size() );
        (*it)->multiOrientations.resize( opts.scales.size() );

        // Copy the scale values within the feature.
        memcpy( &(*it)->multiScales[0], &opts.scales[0], opts.scales.size()*sizeof(double) );
        tlogger.leave("cp scales");

        // For each of the involved scales
        for( unsigned int k = 0; k < opts.scales.size(); ++k )
        {
            npSize = round(a*opts.scales[k]);
            npSize = npSize%2 ? npSize : npSize+1;          // round to the closest odd number
            hpSize = npSize/2;                              // half size of the patch

            CImage tPatch;

            // LEFT IMAGE:
            tlogger.enter("extract & resize");
            smLeftImg.extract_patch( tPatch, (*it)->x-hpSize, (*it)->y-hpSize, npSize, npSize );

            cv::Mat out_mat_patch;
            // The size is a+2xa+2 because we have to compute the gradient (magnitude and orientation) in every pixel within the axa patch so we need
            // one more row and column. For instance, for a 23x23 patch we need a 25x25 patch.
            cv::resize( cv::cvarrToMat(tPatch.getAs<IplImage>(),false), out_mat_patch, cv::Size(a+2,a+2) );
            IplImage aux_img = IplImage(out_mat_patch);
            CImage rsPatch( &aux_img );
            tlogger.leave("extract & resize");

            tlogger.enter("main orientations");
            // Compute the main orientations for the axa patch, taking into account that the actual patch has a size of a+2xa+2
            // (being the center point the one with coordinates a/2+1,a/2+1).
            // A sigma = opts.sg2 is used to smooth the entries in the orientations histograms
            // Orientation vector will be resize inside the function, so don't reserve space for it
            vision::computeMainOrientations(
                            rsPatch, a/2+1, a/2+1, a,
                            (*it)->multiOrientations[k], opts.sg2 );
            tlogger.leave("main orientations");

        } // end scales for
    } // end matches for
    MRPT_END
#else
	THROW_EXCEPTION("This function needs OpenCV 2.1+")
#endif
} // end computeMultiOrientations

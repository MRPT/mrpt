#ifndef __POSES_BINDINGS_H__
#define __POSES_BINDINGS_H__

/* BOOST */
#include <boost/python.hpp>

/* MRPT */
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/utils/CStream.h>

/* CPosePDF */
struct CPosePDFWrap : mrpt::poses::CPosePDF, boost::python::wrapper<mrpt::poses::CPosePDF>
{
    // from inherited class CProbabilityDensityFunction
    mrpt::utils::CObject *duplicate() const;
    void writeToStream(mrpt::utils::CStream& stream, int* pos) const;
    void readFromStream(mrpt::utils::CStream& stream, int pos);
    void getMean(mrpt::poses::CPose2D &mean_point) const;
    void getCovarianceAndMean(mrpt::math::CMatrixFixedNumeric<double,3ul,3ul> &cov, mrpt::poses::CPose2D &mean_point) const;
    void saveToTextFile(const std::string &file) const;
    void drawSingleSample(mrpt::poses::CPose2D &outPart) const;
    void changeCoordinatesReference(const mrpt::poses::CPose3D &newReferenceBase);

    // from CPosePDF
    void copyFrom(const mrpt::poses::CPosePDF &o);
    void bayesianFusion(const mrpt::poses::CPosePDF &p1, const mrpt::poses::CPosePDF &p2, const double &minMahalanobisDistToDrop);
    void inverse(mrpt::poses::CPosePDF &o) const;
};

#endif

#ifndef __UTILS_BINDINGS_H__
#define __UTILS_BINDINGS_H__

/* bindings */
#include "bindings.h"

/* MRPT */
#include <mrpt/utils/CObject.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CRobotSimulator.h>

// CObject wrapper
struct CObjectWrap : mrpt::utils::CObject, boost::python::wrapper<mrpt::utils::CObject>
{
    mrpt::utils::CObject *duplicate() const;
};

// CSerializable wrapper
struct CSerializableWrap : mrpt::utils::CSerializable, boost::python::wrapper<mrpt::utils::CSerializable>
{
    mrpt::utils::CObject *duplicate() const;
    void writeToStream(mrpt::utils::CStream &out, int *getVersion) const;
    void readFromStream(mrpt::utils::CStream &in, int version);
};

#endif

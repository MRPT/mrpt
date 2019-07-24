/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#define MRPT_NO_WARN_BIG_HDR  // Yes, we really want to include all classes.
#include <mrpt/obs.h>

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <sstream>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::io;
using namespace mrpt::math;
using namespace mrpt::serialization;
using namespace std;

#define TEST_CLASS_MOVE_COPY_CTORS(_classname) \
	template class mrpt::CTraitsTest<_classname>

TEST_CLASS_MOVE_COPY_CTORS(CObservation2DRangeScan);
TEST_CLASS_MOVE_COPY_CTORS(CObservation3DRangeScan);
TEST_CLASS_MOVE_COPY_CTORS(CObservationRGBD360);
TEST_CLASS_MOVE_COPY_CTORS(CObservationBearingRange);
TEST_CLASS_MOVE_COPY_CTORS(CObservationBatteryState);
TEST_CLASS_MOVE_COPY_CTORS(CObservationWirelessPower);
TEST_CLASS_MOVE_COPY_CTORS(CObservationRFID);
TEST_CLASS_MOVE_COPY_CTORS(CObservationBeaconRanges);
TEST_CLASS_MOVE_COPY_CTORS(CObservationComment);
TEST_CLASS_MOVE_COPY_CTORS(CObservationGasSensors);
TEST_CLASS_MOVE_COPY_CTORS(CObservationGPS);
TEST_CLASS_MOVE_COPY_CTORS(CObservationReflectivity);
TEST_CLASS_MOVE_COPY_CTORS(CObservationIMU);
TEST_CLASS_MOVE_COPY_CTORS(CObservationOdometry);
TEST_CLASS_MOVE_COPY_CTORS(CObservationRange);
#if MRPT_HAS_OPENCV  // These classes need CImage serialization
TEST_CLASS_MOVE_COPY_CTORS(CObservationImage);
TEST_CLASS_MOVE_COPY_CTORS(CObservationStereoImages);
#endif
TEST_CLASS_MOVE_COPY_CTORS(CObservationCANBusJ1939);
TEST_CLASS_MOVE_COPY_CTORS(CObservationRawDAQ);
TEST_CLASS_MOVE_COPY_CTORS(CObservation6DFeatures);
TEST_CLASS_MOVE_COPY_CTORS(CObservationVelodyneScan);
TEST_CLASS_MOVE_COPY_CTORS(CObservationRotatingScan);
TEST_CLASS_MOVE_COPY_CTORS(CActionRobotMovement2D);
TEST_CLASS_MOVE_COPY_CTORS(CActionRobotMovement3D);

const mrpt::rtti::TRuntimeClassId* lstClasses[] = {
	// Observations:
	CLASS_ID(CObservation2DRangeScan), CLASS_ID(CObservation3DRangeScan),
	CLASS_ID(CObservationRGBD360), CLASS_ID(CObservationBearingRange),
	CLASS_ID(CObservationBatteryState), CLASS_ID(CObservationWirelessPower),
	CLASS_ID(CObservationRFID), CLASS_ID(CObservationBeaconRanges),
	CLASS_ID(CObservationComment), CLASS_ID(CObservationGasSensors),
	CLASS_ID(CObservationGPS), CLASS_ID(CObservationReflectivity),
	CLASS_ID(CObservationIMU), CLASS_ID(CObservationOdometry),
	CLASS_ID(CObservationRange),
#if MRPT_HAS_OPENCV  // These classes need CImage serialization
	CLASS_ID(CObservationImage), CLASS_ID(CObservationStereoImages),
#endif
	CLASS_ID(CObservationCANBusJ1939), CLASS_ID(CObservationRawDAQ),
	CLASS_ID(CObservation6DFeatures), CLASS_ID(CObservationVelodyneScan),
	CLASS_ID(CObservationRotatingScan),
	// Actions:
	CLASS_ID(CActionRobotMovement2D), CLASS_ID(CActionRobotMovement3D)};

// Create a set of classes, then serialize and deserialize to test possible
// bugs:
TEST(Observations, WriteReadToMem)
{
	for (auto& cl : lstClasses)
	{
		try
		{
			CMemoryStream buf;
			auto arch = mrpt::serialization::archiveFrom(buf);
			{
				auto o =
					mrpt::ptr_cast<CSerializable>::from(cl->createObject());
				arch << *o;
				o.reset();
			}

			CSerializable::Ptr recons;
			buf.Seek(0);
			arch >> recons;
		}
		catch (const std::exception& e)
		{
			GTEST_FAIL() << "Exception during serialization test for class '"
						 << cl->className << "':\n"
						 << e.what() << endl;
		}
	}
}

// Also try to convert them to octect vectors:
TEST(Observations, WriteReadToOctectVectors)
{
	for (auto& cl : lstClasses)
	{
		try
		{
			std::vector<uint8_t> buf;
			{
				auto o =
					mrpt::ptr_cast<CSerializable>::from(cl->createObject());
				mrpt::serialization::ObjectToOctetVector(o.get(), buf);
				o.reset();
			}

			CSerializable::Ptr recons;
			mrpt::serialization::OctetVectorToObject(buf, recons);
		}
		catch (const std::exception& e)
		{
			GTEST_FAIL() << "Exception during serialization test for class '"
						 << cl->className << "':\n"
						 << e.what() << endl;
		}
	}
}

// Try to invoke a copy ctor and = operator:
template <class T>
void run_copy_tests()
{
	std::stringstream ss;
	{
		auto ptr_org = T::Create();
		auto ptr_copy_op = T::Create();
		*ptr_copy_op = *ptr_org;  // copy op
		ptr_org.reset();
		// make sure the copy works without erroneous mem accesses,etc.
		ptr_copy_op->getDescriptionAsText(ss);
	}
	{
		auto ptr_org = T::Create();
		auto ptr_copy_ctor = T::Create(*ptr_org);  // copy ctor
		ptr_org.reset();
		// make sure the copy works without erroneous mem accesses,etc.
		ptr_copy_ctor->getDescriptionAsText(ss);
	}
}

TEST(Observations, CopyCtorAssignOp)
{
	run_copy_tests<CObservation2DRangeScan>();
	run_copy_tests<CObservation3DRangeScan>();
	run_copy_tests<CObservationGPS>();
	run_copy_tests<CObservationIMU>();
	run_copy_tests<CObservationOdometry>();
}

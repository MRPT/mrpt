#ifndef CDetectableObject_H
#define CDetectableObject_H

#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
	namespace vision
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CDetectableObject, mrpt::utils::CSerializable, VISION_IMPEXP )

		class VISION_IMPEXP CDetectableObject: public mrpt::utils::CSerializable
		{
			DEFINE_VIRTUAL_SERIALIZABLE( CDetectableObject )

		}; // End of class

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CDetectable2D, mrpt::utils::CSerializable, VISION_IMPEXP )

		class VISION_IMPEXP CDetectable2D: public CDetectableObject
		{
			DEFINE_SERIALIZABLE( CDetectable2D )
		};
	}

}

#endif
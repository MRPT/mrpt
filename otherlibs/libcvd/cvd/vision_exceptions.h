#ifndef CVD_INCLUDE_VISION_EXCEPTIONS_H
#define CVD_INCLUDE_VISION_EXCEPTIONS_H

#include <cvd/exceptions.h>

namespace CVD {

	namespace Exceptions {

		/// %Exceptions specific to vision algorithms
		/// @ingroup gException
		namespace Vision {
			/// Base class for all Image_IO exceptions
			/// @ingroup gException
			struct All: public CVD::Exceptions::All {};

			/// Input images have incompatible dimensions
			/// @ingroup gException
			struct IncompatibleImageSizes : public All {
				IncompatibleImageSizes(const std::string & function)
				{
					what = "Incompatible image sizes in " + function;
				};
			};

			/// Input ImageRef not within image dimensions
			/// @ingroup gException
			struct ImageRefNotInImage : public All {
				ImageRefNotInImage(const std::string & function)
				{
					what = "Input ImageRefs not in image in " + function;
				};
			};


			struct BadInput: public All{
				BadInput(const std::string& function)
				{
					what = "Bad input in " + function;
				};
			};
		};
	}
}

#endif


#ifndef CVD_INTERNAL_IO_PARAMETER_H
#define CVD_INTERNAL_IO_PARAMETER_H

#include <string>
#include <typeinfo>
#include <map>
#include <memory>

namespace CVD {

	#ifndef DOXYGEN_IGNORE_INTERNAL
	namespace Internal{
		class UntypedParameter;
	}
	template<class C=void> class Parameter;

	template<> class Parameter<Internal::UntypedParameter>
	{
			virtual const void* get_data_pointer() const=0;

			public: 
				virtual Parameter* duplicate() const=0;

	};
	#endif
	
	///Class for holding parameters for image savers, with type erasure. Use like:
	///@code
	///  map<string, Parameter<> > p;
	///  p["jpeg.quality"] = Parameter<int>(75);
	///  img_save(image, "foo.jpg", p);
	///@endcode
	///@ingroup gImageIO
	template<class C> class Parameter: public Parameter<Internal::UntypedParameter>
	{
		private:
			C parameter;
		public:
			Parameter(const C& c)
			:parameter(c)
			{
			}

			Parameter* duplicate() const 
			{
				return new Parameter(*this);
			}

			virtual const void* get_data_pointer() const
			{
				return &parameter;
			}
	};

	template<> class Parameter<void>
	{
		private:
			std::unique_ptr<Parameter<Internal::UntypedParameter> > param;
			
		public:
			Parameter()
			{}

			Parameter(const Parameter& p)
			{
				if(p.param.get())
					std::unique_ptr<Parameter<Internal::UntypedParameter> >(p.param->duplicate());
			}

			template<class C> Parameter(const Parameter<C>& p)
			:param(std::unique_ptr<Parameter<Internal::UntypedParameter> >(static_cast<Parameter<Internal::UntypedParameter>*>(new Parameter<C>(p))))
			{}

			template<class C> Parameter& operator=(const Parameter<C>& p)
			{
				param = std::unique_ptr<Parameter<Internal::UntypedParameter> >(static_cast<Parameter<Internal::UntypedParameter>*>(new Parameter<C>(p)));
				return *this;
			}
			
			template<class C> bool is() const 
			{
				if(param.get() == 0)
					return 0;

				const Parameter<C>* p = dynamic_cast<const Parameter<C>* >(param.get());
				return p != 0;
			}

			template<class C> const C& get() const
			{	
				if(param.get() == 0)
					throw std::bad_cast();

				const Parameter<C>& p = dynamic_cast<const Parameter<C>& >(*param);
				return *(const C*) p.get_data_pointer();
			}
	};

}
#endif

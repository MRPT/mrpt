/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARCONFIGARG_H
#define ARCONFIGARG_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArFunctor.h"

class ArArgumentBuilder;

/// Argument class for ArConfig
/** 
    @swignote Swig cannot determine the correct constructor to use
     based on most target langugages types, so you must use subclasses
     defined for various types. Or, use the constructor that accepts 
     functors for dealing with arguments.  Also, Swig cannot use pointers
     to change variables, so you must create ArConfigArg objects, passing
     in default values, and retain references to those objects,
     in addition to passing them to ArConfig, and read new values from those
     objects if ArConfig changes; or pass functors to ArConfigArg instead
     of the initial value.
*/
class ArConfigArg
{
public:

  typedef enum 
  { 
    INVALID, ///< An invalid argument, the argument wasn't created correctly
    INT, ///< Integer argument
    DOUBLE, ///< Double argument
    STRING, ///< String argument
    BOOL, ///< Boolean argument
    FUNCTOR, ///< Argument that handles things with functors
    DESCRIPTION_HOLDER, ///< Argument that just holds a description
    STRING_HOLDER, ///< this one is for holding strings and reading them in and writing them out but not really letting them get sent anywhere (its for unknown config parameters (so they don't get lost if a feature is turned off)
    SEPARATOR, ///< Empty argument that merely acts as a separator within a (large) section.
    LAST_TYPE = SEPARATOR ///< Last value in the enumeration
  } Type;

  enum {
    TYPE_COUNT = LAST_TYPE + 1 ///< Number of argument types
  };

  /// Default empty contructor
  AREXPORT ArConfigArg();
  /// Constructor for making an integer argument by pointer (4 bytes)
  AREXPORT ArConfigArg(const char * name, int *pointer, 
		       const char * description = "", 
		       int minInt = INT_MIN, 
		       int maxInt = INT_MAX); 
  /// Constructor for making an int argument thats a short (2 bytes)
  AREXPORT ArConfigArg(const char * name, short *pointer, 
		       const char * description = "", 
		       int minInt = SHRT_MIN, 
		       int maxInt = SHRT_MAX); 
  /// Constructor for making an int argument thats a ushort (2 bytes)
  AREXPORT ArConfigArg(const char * name, unsigned short *pointer, 
		       const char * description = "", 
		       int minInt = 0, 
		       int maxInt = USHRT_MAX); 
  /// Constructor for making an char (1 byte) argument by pointer (treated as int)
  AREXPORT ArConfigArg(const char * name, unsigned char *pointer, 
		       const char * description = "", 
		       int minInt = 0,
		       int maxInt = 255); 
  /// Constructor for making a double argument by pointer
  AREXPORT ArConfigArg(const char * name, double *pointer,
		       const char * description = "", 
		       double minDouble = -HUGE_VAL,
		       double maxDouble = HUGE_VAL); 
  /// Constructor for making a boolean argument by pointer
  AREXPORT ArConfigArg(const char * name, bool *pointer,
		       const char * description = ""); 
  /// Constructor for making an argument of a string by pointer (see details)
  AREXPORT ArConfigArg(const char *name, char *str, 
		       const char *description,
		       size_t maxStrLen);
  /// Constructor for making an integer argument
  AREXPORT ArConfigArg(const char * name, int val, 
		       const char * description = "", 
		       int minInt = INT_MIN, 
		       int maxInt = INT_MAX); 
  /// Constructor for making a double argument
  AREXPORT ArConfigArg(const char * name, double val,
		       const char * description = "", 
		       double minDouble = -HUGE_VAL,
		       double maxDouble = HUGE_VAL); 
  /// Constructor for making a boolean argument
  AREXPORT ArConfigArg(const char * name, bool val,
		       const char * description = ""); 
  /// Constructor for making an argument that has functors to handle things
  AREXPORT ArConfigArg(const char *name, 
		 ArRetFunctor1<bool, ArArgumentBuilder *> *setFunctor, 
		 ArRetFunctor<const std::list<ArArgumentBuilder *> *> *getFunctor,
		 const char *description);

  /// Constructor for just holding a description (for ArConfig)
  AREXPORT ArConfigArg(const char *str, Type type = DESCRIPTION_HOLDER);
  /// Constructor for holding an unknown argument (STRING_HOLDER)
  AREXPORT ArConfigArg(const char *name, const char *str);
  /// Constructs a new argument of the specified type.
  AREXPORT ArConfigArg(Type type);


  /// Copy constructor
  AREXPORT ArConfigArg(const ArConfigArg & arg);
  /// Assignment operator
  AREXPORT ArConfigArg &operator=(const ArConfigArg &arg);
  /// Destructor
  AREXPORT virtual ~ArConfigArg();

  /// Gets the type of the argument
  AREXPORT ArConfigArg::Type getType(void) const;
  /// Gets the name of the argument
  AREXPORT const char *getName(void) const;
  /// Gets the long description of the argument
  AREXPORT const char *getDescription(void) const;


  /// Sets the argument value, for int arguments
  AREXPORT bool setInt(int val, char *errorBuffer = NULL,
		       size_t errorBufferLen = 0, bool doNotSet = false);
  /// Sets the argument value, for double arguments
  AREXPORT bool setDouble(double val, char *errorBuffer = NULL,
			  size_t errorBufferLen = 0, bool doNotSet = false);
  /// Sets the argument value, for bool arguments
  AREXPORT bool setBool(bool val, char *errorBuffer = NULL,
			size_t errorBufferLen = 0, bool doNotSet = false);
  /// Sets the argument value for ArArgumentBuilder arguments
  AREXPORT bool setString(const char *str, char *errorBuffer = NULL,
			  size_t errorBufferLen = 0, bool doNotSet = false);
  /// Sets the argument by calling the setFunctor callback
  AREXPORT bool setArgWithFunctor(ArArgumentBuilder *argument, 
				  char *errorBuffer = NULL,
				  size_t errorBufferLen = 0,
				  bool doNotSet = false);

  /// Gets the argument value, for int arguments
  AREXPORT int getInt(void) const; 
  /// Gets the argument value, for double arguments
  AREXPORT double getDouble(void) const;
  /// Gets the argument value, for bool arguments
  AREXPORT bool getBool(void) const;
  /// Gets the argument value, for string arguments
  AREXPORT const char *getString(void) const;
  /// Gets the argument value, which is a list of argumentbuilders here
  AREXPORT const std::list<ArArgumentBuilder *> *getArgsWithFunctor(void) const;

  /// Logs the type, name, and value of this argument
  AREXPORT void log(bool verbose = false) const;
  
  /// Gets the minimum int value
  AREXPORT int getMinInt(void) const;
  /// Gets the maximum int value
  AREXPORT int getMaxInt(void) const;
  /// Gets the minimum double value
  AREXPORT double getMinDouble(void) const;
  /// Gets the maximum double value
  AREXPORT double getMaxDouble(void) const;

  /// Gets the priority (only used by ArConfig)
  AREXPORT ArPriority::Priority getConfigPriority(void) const;
  /// Sets the priority (only used by ArConfig)
  AREXPORT void setConfigPriority(ArPriority::Priority priority);

  /// Returns the display hint for this arg, or NULL if none is defined.
  AREXPORT const char *getDisplayHint() const;
  /// Sets the display hint for this arg.
  AREXPORT void setDisplayHint(const char *hintText);

  /// Sets whether to ignore bounds or not (default is to not to
  AREXPORT void setIgnoreBounds(bool ignoreBounds = false);

  /// Checks only the name, type, and value attributes and returns whether they are equal.
  AREXPORT bool isValueEqual(const ArConfigArg &other) const;
  
  /// Gets whether this value has been set since it was last cleared or not
  bool isValueSet(void) { return myValueSet; }
  
  /// Tells the configArg that the value hasn't been set
  void clearValueSet(void) { myValueSet = false; }
  
private:
  /// Internal helper function
  void clear(void);
  void copy(const ArConfigArg &arg);
  
  void set(ArConfigArg::Type type,
           const char *name,
           const char *description);

protected:
  enum IntType {
    INT_NOT, ///< Not an int
    INT_INT, ///< An int (4 bytes) 
    INT_SHORT, ///< A short (2 bytes)
    INT_UNSIGNED_SHORT, ///< An unsigned short (2 bytes)
    INT_UNSIGNED_CHAR ///< An unsigned char (1 byte)
  };
  
  ArConfigArg::Type myType;
  std::string myName;
  std::string myDescription;
  bool myOwnPointedTo;
  int *myIntPointer;
  short *myIntShortPointer;
  unsigned short *myIntUnsignedShortPointer;
  unsigned char *myIntUnsignedCharPointer;
  int myMinInt, myMaxInt;
  double *myDoublePointer;
  double myMinDouble, myMaxDouble;
  bool *myBoolPointer;
  char *myStringPointer;
  size_t myMaxStrLen;
  bool myUsingOwnedString;
  std::string myString;
  ArPriority::Priority myConfigPriority;
  ArConfigArg::IntType myIntType;
  bool myIgnoreBounds;
  ArRetFunctor1<bool, ArArgumentBuilder *> *mySetFunctor;
  ArRetFunctor<const std::list<ArArgumentBuilder *> *> *myGetFunctor;
  std::string myDisplayHint;
  bool myValueSet;
};

#endif // ARARGUMENT_H

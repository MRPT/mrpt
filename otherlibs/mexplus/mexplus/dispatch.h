/** MEX dispatch library.
 *
 * This helper contains MEX_DEFINE() macro to help create a dispatchable MEX
 * file. Two files are required to create a new mex function. Suppose you are
 * creating two MEX functions `myfunc` and `myfunc2`. Then, make the following
 * files.
 *
 * myfunc.m
 *
 *     function output_args = myfunc( varargin )
 *     %MYFUNC Description of the function.
 *     %
 *     %   Details go here.
 *     %
 *       output_args = mylibrary(mfilename, varargin{:})
 *     end
 *
 * myfunc2.m
 *
 *     function output_args = myfunc( varargin )
 *     %MYFUNC Description of the function.
 *     %
 *     %   Details go here.
 *     %
 *       output_args = mylibrary(mfilename, varargin{:})
 *     end
 *
 * These files contains help documentation and a line to invoke the mex
 * function.
 *
 * mylibrary.cc
 *
 *     #include <mexplus/dispatch.h>
 *
 *     MEX_DEFINE(myfunc) (int nlhs, mxArray* plhs[],
 *                         int nrhs, const mxArray* prhs[]) {
 *       ...
 *     }
 *
 *     MEX_DEFINE(myfunc2) (int nlhs, mxArray* plhs[],
 *                          int nrhs, const mxArray* prhs[]) {
 *       ...
 *     }
 *
 *     MEX_DISPATCH
 *
 * This file is the implementation of the mex function. The MEX_DEFINE macro
 * defines an entry point of the function. MEX_DISPATCH macro at the end
 * inserts necessary codes to dispatch function calls to an appropriate
 * function.
 *
 * Similarly, you can write another pair of .m (and C++) file to add to your
 * library. You may split MEX_DEFINE macros in multiple C++ files. In that
 * case, have MEX_DISPATCH macro in one of the files.
 *
 * Kota Yamaguchi 2014 <kyamagu@cs.stonybrook.edu>
 */

#ifndef __MEXPLUS_DISPATCH_H__
#define __MEXPLUS_DISPATCH_H__

#include <map>
#include <memory>
#include <mex.h>
#include <string>

// Auxiliar code for testing
#define __MEXPLUS_TEST__
#ifdef __MEXPLUS_TEST__
    #define mexLock() printf("mexLock here\n")
#endif

namespace mexplus {

class OperationCreator;
inline void CreateOperation(const std::string& name,
                            OperationCreator* creator);

/** Abstract operation class. Child class must implement operator().
 */
class Operation {
public:
  /** Destructor.
   */
  virtual ~Operation() {}
  /** Execute the operation.
   */
  virtual void operator()(int nlhs,
                          mxArray *plhs[],
                          int nrhs,
                          const mxArray *prhs[]) = 0;
};

/** Base class for operation creators.
 */
class OperationCreator {
public:
  /** Register an operation in the constructor.
   */
  OperationCreator(const std::string& name) {
    CreateOperation(name, this);
  }
  /** Destructor.
   */
  virtual ~OperationCreator() {}
  /** Implementation must return a new instance of the operation.
   */
  virtual Operation* create() = 0;
};

/** Implementation of the operation creator to be used as composition in an
 * Operator class.
 */
template <class OperationClass>
class OperationCreatorImpl : public OperationCreator {
public:
  OperationCreatorImpl(const std::string& name) : OperationCreator(name) {}
  virtual Operation* create() { return new OperationClass; }
};

/** Factory class for operations.
 */
class OperationFactory {
public:
  /** Register a new creator.
   */
  friend void CreateOperation(const std::string& name,
                              OperationCreator* creator);
  /** Create a new instance of the registered operation.
   */
  static Operation* create(const std::string& name) {
    std::map<std::string, OperationCreator*>::const_iterator it =
        registry()->find(name);
    if (it == registry()->end())
      return static_cast<Operation*>(nullptr);
    else
      return it->second->create();
  }

private:
  /** Obtain a pointer to the registration table.
   */
  static std::map<std::string, OperationCreator*>* registry() {
    static std::map<std::string, OperationCreator*> registry_table;
    return &registry_table;
  }
};

/** Register a new creator in OperationFactory.
 */
inline void CreateOperation(const std::string& name,
                            OperationCreator* creator) {
  OperationFactory::registry()->insert(make_pair(name, creator));
}

/** Key-value storage to make a stateful MEX function.
 *
 *    #include <mexplus/dispatch.h>
 *
 *    using namespace std;
 *    using namespace mexplus;
 *
 *    class Database;
 *
 *    template class Session<Database>;
 *
 *    MEX_DEFINE(open) (int nlhs, mxArray* plhs[],
 *                      int nrhs, const mxArray* prhs[]) {
 *      unique_ptr<Database> database(new Database(...));
 *      database->open(...);
 *      intptr_t session_id = Session<Database>::create(database.release());
 *      plhs[0] = mxCreateDoubleScalar(session_id);
 *    }
 *
 *    MEX_DEFINE(query) (int nlhs, mxArray* plhs[],
 *                       int nrhs, const mxArray* prhs[]) {
 *      intptr_t session_id = mxGetScalar(prhs[0]);
 *      Database* database = Session<Database>::get(session_id);
 *      database->query(...);
 *    }
 *
 *    MEX_DEFINE(close) (int nlhs, mxArray* plhs[],
 *                       int nrhs, const mxArray* prhs[]) {
 *      intptr_t session_id = mxGetScalar(prhs[0]);
 *      Session<Database>::destroy(session_id);
 *    }
 */
template<class T>
class Session {
public:
  typedef std::map<intptr_t, std::shared_ptr<T> > InstanceMap;

  /** Create an instance.
   */
  static intptr_t create(T* instance) {
    InstanceMap* instances = getInstances();
    intptr_t id = reinterpret_cast<intptr_t>(instance);
    instances->insert(std::make_pair(id, std::shared_ptr<T>(instance)));
    mexLock();
    return id;
  }
  /** Destroy an instance.
   */
  static void destroy(intptr_t id) {
    getInstances()->erase(id);
    mexUnlock();
  }
  static void destroy(const mxArray* pointer) {
    destroy(getIntPointer(pointer));
  }
  /** Retrieve an instance or throw if no instance is found.
   */
  static T* get(intptr_t id) {
    InstanceMap* instances = getInstances();
    typename InstanceMap::iterator instance = instances->find(id);
    if (instance == instances->end())
      mexErrMsgIdAndTxt("mexplus:session:notFound",
                        "Invalid id %d. Did you create?",
                        id);
    return instance->second.get();
  }
  static T* get(const mxArray* pointer) {
    return get(getIntPointer(pointer));
  }
  /** Retrieve a const instance or throw if no instance is found.
   */
  static const T& getConst(intptr_t id) {
    return *get(id);
  }
  static const T& getConst(const mxArray* pointer) {
    return getConst(getIntPointer(pointer));
  }
  /** Check if the given id exists.
   */
  static bool exist(intptr_t id) {
    InstanceMap* instances = getInstances();
    typename InstanceMap::iterator instance = instances->find(id);
    return instance != instances->end();
  }
  static bool exist(const mxArray* pointer) {
    return exist(getIntPointer(pointer));
  }
  /** Clear all session instances.
   */
  static void clear() {
    for (int i = 0; i < getInstances()->size(); ++i)
      mexUnlock();
    getInstances()->clear();
  }

private:
  /** Constructor prohibited.
   */
  Session() {}
  ~Session() {}
  /** Convert mxArray to intptr_t.
   */
  static intptr_t getIntPointer(const mxArray* pointer) {
    if (mxIsEmpty(pointer))
      mexErrMsgIdAndTxt("mexplus:session:invalidType", "Id is empty.");
    if (sizeof(intptr_t) == 8 && !mxIsInt64(pointer) && !mxIsUint64(pointer))
      mexErrMsgIdAndTxt("mexplus:session:invalidType",
                        "Invalid id type %s.",
                        mxGetClassName(pointer));
    if (sizeof(intptr_t) == 4 && !mxIsInt32(pointer) && !mxIsUint32(pointer))
      mexErrMsgIdAndTxt("mexplus:session:invalidType",
                        "Invalid id type %s.",
                        mxGetClassName(pointer));
    return *reinterpret_cast<intptr_t*>(mxGetData(pointer));
  }
  /** Get static instance storage.
   */
  static InstanceMap* getInstances() {
    static InstanceMap instances;
    return &instances;
  }
};

} // namespace mexplus

/** Define a MEX API function. Example:
 *
 * MEX_DEFINE(myfunc) (int nlhs, mxArray *plhs[],
 *                     int nrhs, const mxArray *prhs[]) {
 *   if (nrhs != 1 || nlhs > 1)
 *     mexErrMsgTxt("Wrong number of arguments.");
 *   ...
 * }
 */
#define MEX_DEFINE(name) \
class Operation_##name : public mexplus::Operation { \
public: \
  virtual void operator()(int nlhs, \
                          mxArray *plhs[], \
                          int nrhs, \
                          const mxArray *prhs[]); \
private: \
  static const mexplus::OperationCreatorImpl<Operation_##name> creator_; \
}; \
const mexplus::OperationCreatorImpl<Operation_##name> \
    Operation_##name::creator_(#name); \
void Operation_##name::operator()

/** Insert a function dispatching code. Use once per MEX binary.
 */
#define MEX_DISPATCH \
void mexFunction(int nlhs, mxArray *plhs[], \
                 int nrhs, const mxArray *prhs[]) { \
  if (nrhs < 1 || !mxIsChar(prhs[0])) \
    mexErrMsgIdAndTxt("mexplus:dispatch:argumentError", \
                      "Invalid argument: missing operation."); \
  std::string operation_name( \
      mxGetChars(prhs[0]), \
      mxGetChars(prhs[0]) + mxGetNumberOfElements(prhs[0])); \
  std::auto_ptr<mexplus::Operation> operation( \
      mexplus::OperationFactory::create(operation_name)); \
  if (operation.get() == nullptr) \
    mexErrMsgIdAndTxt("mexplus:dispatch:argumentError", \
        "Invalid operation: %s", operation_name.c_str()); \
  (*operation)(nlhs, plhs, nrhs - 1, prhs + 1); \
}

#endif // __MEXPLUS_DISPATCH_H__

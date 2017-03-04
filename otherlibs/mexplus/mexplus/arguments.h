/** MEX function arguments helper library.
 *
 * Example: writing a MEX function that takes 2 input arguments and 1 optional
 *          flag, and returns one output.
 *
 * % myFunction.m
 * function result = myFunction(arg1, arg2, varargin)
 *
 * // myFunction.cc
 * #include "mexplus/arguments.h"
 * using namespace std;
 * using namespace mexplus;
 * void mexFunction(int nlhs, mxArray* plhs[],
 *                  int nrhs, const mxArray* prhs) {
 *   InputArguments input(nrhs, prhs, 2, 1, "Flag");
 *   OutputArguments output(nlhs, plhs, 1);
 *   vector<double> result = myFunction(input.get<vector<double> >(0),
 *                                      input.get<string>(1),
 *                                      input.get<int>("Flag", 0));
 *   output.set(0, result);
 * }
 *
 * % Build
 * >> mex myFunction.cc src/mexplus/arguments.cc
 *
 * Kota Yamaguchi 2014 <kyamagu@cs.stonybrook.edu>
 */

#ifndef __MEXPLUS_ARGUMENTS_H__
#define __MEXPLUS_ARGUMENTS_H__

#include <map>
#include <mexplus/mxarray.h>
#include <sstream>
#include <stdarg.h>

namespace mexplus {

/** Utility to parse input arguments.
 *
 * Example: parse 2 mandatory and 2 optional arguments.
 *
 *     InputArguments input(nrhs, prhs, 2, 2, "option1", "option2");
 *     myFunction2(input.get<double>(0),
 *                 input.get<int>(1),
 *                 input.get<string>("option1", "foo"),
 *                 input.get<int>("option2", 10));
 *
 * Example: parse 1 + 2 argument format or 2 + 2 argument format.
 *
 *     InputArguments input;
 *     input.define("format1", 1, 2, "option1", "option2");
 *     input.define("format2", 2, 2, "option1", "option2");
 *     input.parse(nrhs, prhs);
 *     if (input.is("format1"))
 *         myFunction(input.get<int>(0),
 *                    input.get<string>("option1", "foo"),
 *                    input.get<int>("option2", 10));
 *     else if (input.is("format2"))
 *         myFunction2(input.get<int>(0),
 *                     input.get<string>(1),
 *                     input.get<string>("option1", "foo"),
 *                     input.get<int>("option2", 10));
 *
 */
class InputArguments {
public:
  /** Case-insensitive comparator for std::string.
   */
  struct CaseInsensitiveComparator {
    struct CaseInsensitiveElementComparator {
      bool operator() (const char& c1, const char& c2) const {
          return tolower(c1) < tolower(c2);
      }
    };
    bool operator() (const std::string & s1, const std::string & s2) const {
      return std::lexicographical_compare(s1.begin(),
                                          s1.end(),
                                          s2.begin(),
                                          s2.end(),
                                          CaseInsensitiveElementComparator());
    }
  };

  typedef std::map<std::string, const mxArray*, CaseInsensitiveComparator>
      OptionMap;
  /** Definition of arguments.
   */
  typedef struct Definition_tag {
    std::vector<const mxArray*> mandatories;
    OptionMap optionals;
  } Definition;

  /** Empty constructor.
   */
  InputArguments() {}
  /** Shorthand constructor for a single argument definition.
   */
  InputArguments(int nrhs,
                 const mxArray* prhs[],
                 int mandatory_size = 1,
                 int option_size = 0,
                 ...) {
    Definition* definition = &definitions_["default"];
    definition->mandatories.resize(mandatory_size, nullptr);
    va_list variable_list;
    va_start(variable_list, option_size);
    fillOptionalDefinition(option_size, &definition->optionals, variable_list);
    va_end(variable_list);
    parse(nrhs, prhs);
  }
  virtual ~InputArguments() {}
  /** Define a new argument format.
   */
  void define(const std::string name,
              int mandatory_size,
              int option_size = 0,
              ...) {
    Definition* definition = &definitions_[name];
    definition->mandatories.resize(mandatory_size);
    va_list variable_list;
    va_start(variable_list, option_size);
    fillOptionalDefinition(option_size, &definition->optionals, variable_list);
    va_end(variable_list);
  }
  /** Parse arguments from mexFunction input.
   */
  void parse(int nrhs, const mxArray* prhs[]) {
    if (definitions_.empty())
      mexErrMsgIdAndTxt("mexplus:arguments:error", "No format defined.");
    std::map<std::string, Definition>::iterator entry;
    std::vector<std::map<std::string, Definition>::iterator> delete_positions;
    for (entry = definitions_.begin(); entry != definitions_.end(); ++entry)
      if (!parseDefinition(nrhs, prhs, &entry->second))
        delete_positions.push_back(entry);
    for (size_t i = 0; i < delete_positions.size(); ++i)
      definitions_.erase(delete_positions[i]);
    if (definitions_.empty())
      mexErrMsgIdAndTxt("mexplus:arguments:error",
                        (error_message_.empty()) ?
                        "Invalid arguments." : error_message_.c_str());
    if (definitions_.size() > 1)
      mexWarnMsgIdAndTxt("mexplus:arguments:warning",
                         "Input arguments match more than one signature.");
  }
  /** Return which format is chosen.
   */
  bool is(const std::string& name) const {
    std::map<std::string, Definition>::const_iterator entry =
        definitions_.find(name);
    return (entry != definitions_.end());
  }
  /** Get a parsed mandatory argument.
   */
  const mxArray* get(size_t index) const {
    if (definitions_.empty())
      mexErrMsgIdAndTxt("mexplus:arguments:error", "No format defined.");
    const Definition& definition = definitions_.begin()->second;
    if (index >= definition.mandatories.size())
      mexErrMsgIdAndTxt("mexplus:arguments:error", "Index out of range.");
    return definition.mandatories[index];
  }
  /** Get a parsed mandatory argument.
   */
  template <typename T>
  T get(size_t index) const;
  template <typename T>
  void get(size_t index, T* value) const;
  /** Get a parsed optional argument.
   */
  const mxArray* get(const std::string& option_name) const {
    if (definitions_.empty())
      mexErrMsgIdAndTxt("mexplus:arguments:error", "No format defined.");
    const Definition& definition = definitions_.begin()->second;
    std::map<std::string, const mxArray*>::const_iterator entry =
        definition.optionals.find(option_name);
    if (entry == definition.optionals.end())
      mexErrMsgIdAndTxt("mexplus:arguments:error",
                        "Unknown option %s.",
                        option_name.c_str());
    return entry->second;
  }
  /** Get a parsed optional argument.
   */
  template <typename T>
  T get(const std::string& option_name, const T& default_value) const;
  template <typename T>
  void get(const std::string& option_name,
           const T& default_value,
           T* value) const;
  /** Access raw mxArray* pointer.
   */
  const mxArray* operator[] (size_t index) const { return get(index); }
  /** Access raw mxArray* pointer.
   */
  const mxArray* operator[] (const std::string& option_name) const {
    return get(option_name);
  }

private:
  /** Fill in optional arguments definition.
   */
  void fillOptionalDefinition(int option_size,
                              OptionMap* optionals,
                              va_list variable_list) {
    for (int i = 0; i < option_size; ++i) {
      const char* option_name = va_arg(variable_list, const char*);
      (*optionals)[std::string(option_name)] = nullptr;
    }
  }
  /** Try to parse one definition or return false on failure.
   */
  bool parseDefinition(size_t nrhs,
                       const mxArray* prhs[],
                       Definition* definition) {
    std::stringstream message;
    if (nrhs < definition->mandatories.size()) {
      message << "Too few arguments: " << nrhs << " for at least "
              << definition->mandatories.size() << ".";
      error_message_.assign(message.str());
      return false;
    }
    size_t index = 0;
    for (; index < definition->mandatories.size(); ++index)
      definition->mandatories[index] = prhs[index];
    for (; index < nrhs; ++index) {
      // Check if option name is valid.
      const mxArray* option_name_array = prhs[index];
      if (!mxIsChar(option_name_array)) {
        message << "Option name must be char but is given "
                << mxGetClassName(option_name_array) << ".";
        error_message_.assign(message.str());
        return false;
      }
      char option_name[64];
      if (mxGetString(option_name_array, option_name, sizeof(option_name))) {
        message << "Option name too long.";
        error_message_.assign(message.str());
        return false;
      }
      std::map<std::string, const mxArray*>::iterator entry =
          definition->optionals.find(option_name);
      if (entry == definition->optionals.end()) {
        message << "Invalid option name: '" << option_name << "'.";
        error_message_.assign(message.str());
        return false;
      }
      // Check if options are even.
      if (++index >= nrhs) {
        message << "Missing option value for option '" << option_name << "'.";
        error_message_.assign(message.str());
        return false;
      }
      if (entry->second)
        mexErrMsgIdAndTxt("mexplus:arguments:warning",
                          "Option '%s' appeared more than once.",
                          option_name);
      entry->second = prhs[index];
    }
    return true;
  }
  /** Format definitions.
   */
  std::map<std::string, Definition> definitions_;
  /** Last error message.
   */
  std::string error_message_;
};

template <typename T>
T InputArguments::get(size_t index) const {
  T value;
  get<T>(index, &value);
  return value;
}

template <typename T>
void InputArguments::get(size_t index, T* value) const {
  MxArray::to<T>(get(index), value);
}

template <typename T>
T InputArguments::get(const std::string& option_name,
                      const T& default_value) const {
  T value;
  get<T>(option_name, default_value, &value);
  return value;
}

template <typename T>
void InputArguments::get(const std::string& option_name,
                         const T& default_value,
                         T* value) const {
  MxArray option(get(option_name));
  if (option)
    option.to<T>(value);
  else
    *value = default_value;
}

/** Output arguments wrapper.
 *
 * Example:
 *
 *     OutputArguments output(nlhs, plhs, 3);
 *     MxArray cell = MxArray::Cell(1, 3);
 *     cell.set(0, 0);
 *     cell.set(1, 1);
 *     cell.set(2, "value");
 *     output.set(0, 1);
 *     output.set(1, "foo");
 *     output.set(2, cell.release());
 */
class OutputArguments {
public:
  /** Construct output argument wrapper.
   */
  OutputArguments(int nlhs,
                  mxArray** plhs,
                  int maximum_size = 1,
                  int mandatory_size = 0) : nlhs_(nlhs), plhs_(plhs) {
    if (mandatory_size > nlhs)
      mexErrMsgIdAndTxt("mexplus:arguments:error",
                        "Too few output: %d for %d.",
                        nlhs,
                        mandatory_size);
    if (maximum_size < nlhs)
      mexErrMsgIdAndTxt("mexplus:arguments:error",
                        "Too many output: %d for %d.",
                        nlhs,
                        maximum_size);
  }
  /** Safely assign mxArray to the output.
   */
  void set(size_t index, mxArray* value) {
    if (index < nlhs_)
      plhs_[index] = value;
  }
  /** Safely assign T to the output.
   */
  template <typename T>
  void set(size_t index, const T& value) {
	set(index, from(value));
  }
  /** Size of the output.
   */
  size_t size() const { return nlhs_; }
  /** Const square bracket operator.
   */
  mxArray* const& operator[] (size_t index) const {
    if (index >= nlhs_)
      mexErrMsgIdAndTxt("mexplus:arguments:error",
                        "Output index out of range: %d.",
                        index);
    return plhs_[index];
  }
  /** Mutable square bracket operator.
   */
  mxArray*& operator[] (size_t index) {
    if (index >= nlhs_)
      mexErrMsgIdAndTxt("mexplus:arguments:error",
                        "Output index out of range: %d.",
                        index);
    return plhs_[index];
  }

private:
  /** Number of output arguments.
   */
  size_t nlhs_;
  /** Output argument array.
   */
  mxArray** plhs_;
};

} // namespace mexplus

#endif // __MEXPLUS_ARGUMENTS_H__

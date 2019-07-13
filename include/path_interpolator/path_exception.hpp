#ifndef INCLUDE_PATH_EXCEPTION_HPP_
#define INCLUDE_PATH_EXCEPTION_HPP_

#include <cstdlib>

#include <string>
#include <sstream>
#include <iostream>

#include <exception>
#include <stdexcept>


/////////////////////////////////////////////////////////////////////////////////////////

namespace interp {

/// Path Exception code
enum ExceptionCode {
  PATH_EXCEPTION=0,
  PATH_UNDEF_EXCEPTION,
  PATH_IVALID_INPUT_SIZE
};

/// convert number type to string for(C++98)
/// @param[in] num number
/// @return converted string of the argument number
template<class T> const std::string g_num2str(const T& num) {
  std::stringstream s;
  s << num;
  return s.str();
};

/// wrapper of throw()
#define THROW( exception_class_name, message )                          \
  throw exception_class_name( message + std::string(" -- at ")      \
                              + std::string(__FILE__) + std::string(": l.") \
                              + g_num2str(__LINE__) + std::string(": ") \
                              + std::string(__func__ ) + std::string("().") )

/// Path Exception class
class PathException : public std::runtime_error {
public:
  /// Constructor
  /// @param[in] message explaination of this exception
  /// @param[in] name the name of this exception class
  explicit PathException( const std::string& message=" ",
                          const std::string& name="PathException" ) :
    std::runtime_error( "["+ name +"]: "+ message ) {}

  /// Destructor
  ~PathException() throw() {}

  /// get Exception Code
  /// @return PATH_EXCEPTION
  virtual const ExceptionCode code() { return PATH_EXCEPTION; }
};

/// Undefined Path Exception class
class UndefPathException : public PathException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit UndefPathException( const std::string& message,
                               const std::string& name="UndefPathException" ) :
    PathException( message, name ) {}

  /// get Exception Code
  /// @return PATH_UNDEF_EXCEPTION
  virtual const ExceptionCode code() { return PATH_UNDEF_EXCEPTION; }
};

/// Undefined Exception Handler
/// Call this handler from std::set_unexpected(g_~_hander)
/// before calling following classes of interp namespace at the main().
/// If not call this, undefined exceptions may not be catched and std::termniate().
/// void g_undef_exception_handler() {
///   throw UndefPathException("not specified");
/// };

/////////////////////////////////////////////////////////////////////////////////////////

/// Invalid Input Size Exception Class
class InvalidInputSize : public PathException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit InvalidInputSize( const std::string& message,
                             const std::string& name="InvaliInputSize" ) :
    PathException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return PATH_IVALID_INPUT_SIZE; }
};

}

#endif // INCLUDE_PATH_EXCEPTION_HPP_

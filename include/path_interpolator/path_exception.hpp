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
  PATH_UNDEFINED_EXCEPTION,
  PATH_IVALID_TYPE_ARGUMENT,
  PATH_IVALID_ARGUMENT_SIZE,
  PATH_IVALID_ARGUMENT_VALUE,
  PATH_IVALID_INDEX_ACCESS,
  PATH_QUEUE_SIZE_EMPTY,
  PATH_NOT_GENERATED,
  PATH_NOT_DEF_VEL_LIMIT,
  PATH_TIME_IS_OUT_OF_RANGE
};

/////////////////////////////////////////////////////////////////////////////////////////

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

 /////////////////////////////////////////////////////////////////////////////////////////

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
  /// @return PATH_UNDEFINED_EXCEPTION
  virtual const ExceptionCode code() { return PATH_UNDEFINED_EXCEPTION; }
};

/// Undefined Exception Handler
/// Call this handler from std::set_unexpected(g_~_hander)
/// before calling following classes of interp namespace at the main().
/// If not call this, undefined exceptions may not be catched and std::termniate().
/// void g_undef_exception_handler() {
///   throw UndefPathException("not specified");
/// };

/// Invalid Type Argument Exception Class
class InvalidTypeArgument : public PathException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit InvalidTypeArgument( const std::string& message,
                                const std::string& name="InvalidTypeArgument" ) :
    PathException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return PATH_IVALID_TYPE_ARGUMENT; }
};


/// Invalid Argument Size Exception Class
class InvalidArgumentSize : public PathException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit InvalidArgumentSize( const std::string& message,
                                const std::string& name="InvalidArgumentSize" ) :
    PathException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return PATH_IVALID_ARGUMENT_SIZE; }
};


/// Invalid Argument Value Exception Class
class InvalidArgumentValue : public PathException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit InvalidArgumentValue( const std::string& message,
                                 const std::string& name="InvalidArgumentValue" ) :
    PathException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return PATH_IVALID_ARGUMENT_VALUE; }
};


/// Invalid Index Access Exception Class
class InvalidIndexAccess : public PathException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit InvalidIndexAccess( const std::string& message,
                               const std::string& name="InvalidIndexAccess" ) :
    PathException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return PATH_IVALID_INDEX_ACCESS; }
};


// there is no time queue. (queue size is empty)
class QueueSizeEmpty : public PathException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit QueueSizeEmpty( const std::string& message,
                           const std::string& name="QueueSizeEmpty" ) :
    PathException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return PATH_QUEUE_SIZE_EMPTY; }
};


/// Path has not generated.
class NotPathGenerated : public PathException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit NotPathGenerated( const std::string& message,
                             const std::string& name="NotPathGenerated" ) :
    PathException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return PATH_NOT_GENERATED; }
};


/// Velocity limit is not defined.
class NoVelocityLimit : public PathException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit NoVelocityLimit( const std::string& message,
                             const std::string& name="NoVelocityLimit" ) :
    PathException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return PATH_NOT_DEF_VEL_LIMIT; }
};


/// Ordered Time is out of range
class TimeOutOfRange : public PathException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit TimeOutOfRange( const std::string& message,
                           const std::string& name="TimeOutOfRange" ) :
    PathException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return PATH_TIME_IS_OUT_OF_RANGE; }
};


} // End of namespace interp

#endif // INCLUDE_PATH_EXCEPTION_HPP_

#ifndef INCLUDE_SPLINE_EXCEPTION_HPP_
#define INCLUDE_SPLINE_EXCEPTION_HPP_

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
  SPLINE_EXCEPTION=0,
  SPLINE_UNDEFINED_EXCEPTION,
  SPLINE_IVALID_TYPE_ARGUMENT,
  SPLINE_IVALID_ARGUMENT_SIZE,
  SPLINE_IVALID_ARGUMENT_VALUE,
  SPLINE_IVALID_INDEX_ACCESS,
  SPLINE_QUEUE_SIZE_EMPTY,
  SPLINE_NOT_GENERATED,
  SPLINE_NOT_DEF_VEL_LIMIT,
  SPLINE_TIME_IS_OUT_OF_RANGE
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

/// Spline Exception class
class SplineException : public std::runtime_error {
public:
  /// Constructor
  /// @param[in] message explaination of this exception
  /// @param[in] name the name of this exception class
  explicit SplineException( const std::string& message=" ",
                          const std::string& name="SplineException" ) :
    std::runtime_error( "["+ name +"]: "+ message ) {}

  /// Destructor
  ~SplineException() throw() {}

  /// get Exception Code
  /// @return SPLINE_EXCEPTION
  virtual const ExceptionCode code() { return SPLINE_EXCEPTION; }
};

/// Undefined Spline Exception class
class UndefSplineException : public SplineException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit UndefSplineException( const std::string& message,
                               const std::string& name="UndefSplineException" ) :
    SplineException( message, name ) {}

  /// get Exception Code
  /// @return SPLINE_UNDEFINED_EXCEPTION
  virtual const ExceptionCode code() { return SPLINE_UNDEFINED_EXCEPTION; }
};

/// Undefined Exception Handler
/// Call this handler from std::set_unexpected(g_~_hander)
/// before calling following classes of interp namespace at the main().
/// If not call this, undefined exceptions may not be catched and std::termniate().
/// void g_undef_exception_handler() {
///   throw UndefSplineException("not specified");
/// };

/// Invalid Type Argument Exception Class
class InvalidTypeArgument : public SplineException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit InvalidTypeArgument( const std::string& message,
                                const std::string& name="InvalidTypeArgument" ) :
    SplineException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return SPLINE_IVALID_TYPE_ARGUMENT; }
};


/// Invalid Argument Size Exception Class
class InvalidArgumentSize : public SplineException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit InvalidArgumentSize( const std::string& message,
                                const std::string& name="InvalidArgumentSize" ) :
    SplineException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return SPLINE_IVALID_ARGUMENT_SIZE; }
};


/// Invalid Argument Value Exception Class
class InvalidArgumentValue : public SplineException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit InvalidArgumentValue( const std::string& message,
                                 const std::string& name="InvalidArgumentValue" ) :
    SplineException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return SPLINE_IVALID_ARGUMENT_VALUE; }
};


/// Invalid Index Access Exception Class
class InvalidIndexAccess : public SplineException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit InvalidIndexAccess( const std::string& message,
                               const std::string& name="InvalidIndexAccess" ) :
    SplineException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return SPLINE_IVALID_INDEX_ACCESS; }
};


// there is no time queue. (queue size is empty)
class QueueSizeEmpty : public SplineException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit QueueSizeEmpty( const std::string& message,
                           const std::string& name="QueueSizeEmpty" ) :
    SplineException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return SPLINE_QUEUE_SIZE_EMPTY; }
};


/// Spline has not generated.
class NotSplineGenerated : public SplineException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit NotSplineGenerated( const std::string& message,
                             const std::string& name="NotSplineGenerated" ) :
    SplineException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return SPLINE_NOT_GENERATED; }
};


/// Velocity limit is not defined.
class NoVelocityLimit : public SplineException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit NoVelocityLimit( const std::string& message,
                             const std::string& name="NoVelocityLimit" ) :
    SplineException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return SPLINE_NOT_DEF_VEL_LIMIT; }
};


/// Ordered Time is out of range
class TimeOutOfRange : public SplineException {
public:
  /// Constructor
  /// @param[in] message Explaination of this exception
  explicit TimeOutOfRange( const std::string& message,
                           const std::string& name="TimeOutOfRange" ) :
    SplineException( message, name ) {}

  /// get Exception Code
  /// @return code_
  virtual const ExceptionCode code() { return SPLINE_TIME_IS_OUT_OF_RANGE; }
};


} // End of namespace interp

#endif // INCLUDE_SPLINE_EXCEPTION_HPP_

spline_interpolator
===

This is practice for implementing spline-interpolator

&nbsp;

# 1. Dependency

## OS

Linux(Ubuntu 18.04), QNX 6.5.0 SP  

## C++ Compiler

- C++ version \>= -std=c++03  
- gcc version \>= 7.5.0  
- qcc : -Vgcc\_ntox86\_cpp

## 3rd party libraries

These libraries are used at unit_test.

- googletest(>=1.8.1) 
- gnuplot( for unit_test )

&nbsp;

# 2. Directory Map

```
.
├── README.md
├── Makefile
├── bin/ : Destination of executing binaries (spline_interpolator, unit_test)
├── lib/ : Destination of static library (libspline_interpolator.a)
├── images/ : Destination of plotting graph & csv by unit_test
├── include/
│       └── spline_interpolator/
│           ├── spline_data.hpp : Time-queue data class definition.
│           ├── spline_exception.hpp : Excpetion class definition.
│           ├── spline_interpolator.hpp : Parent class SplineInterpolator defeinition
│           ├── non_uniform_rounding_spline.hpp : velocity interploation
│           └── cubic_spline_exception.hpp : CubicSplineInterpolator inherited SplineInterpolator
├── src/
│   ├── main.cpp
│   ├── spline_data.cpp
│   ├── spline_interpolator.cpp
│   ├── non_uniform_rounding_spline.cpp
│   └── cubic_spline_interpolator.cpp
└── test/
    ├── test_spline_data.cpp
    ├── test_spline_interpolator.cpp
    ├── test_cubic_spline_interpolator.cpp
    ├── unit_test.cpp
    └── util/
        ├── gnuplot_realtime.cpp
        ├── gnuplot_realtime.hpp
        └── test_graph_plot.hpp

```

&nbsp;

# 3. Make

```
$ make
```

&nbsp;

# 4. The output files

## Binary

- bin/spline\_interpolator
- bin/unit\_test

## Library

- lib/libspline\_interpolator.a

&nbsp;

# 5. Test

```
$ ./bin/unit_test
```

Interpolated graphs will be generated into ./images/*.{png,csv}  
when the fllowing tests are executed.

- CubicSplineTest


&nbsp;

# 6. Documents

See the below documents which simply explains this library.

- [./doc/spline_interpolator_JP](./doc/spline_interpolator_JP.md)


&nbsp;


# 7. License

CopyLight(c) 2021, kdaic 
All right reserved.  

This software is made under the MIT License.  
http://opensource.org/licenses/mit-license.php  

<div align="right"> That's All. </div>

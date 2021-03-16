spline_interpolator
===

This is practice for implementing spline-interpolator

&nbsp;

# Other Dependency

g++, googletest(>=1.8.1), gnuplot( for unit_test )

&nbsp;

# Directory Map

```
.
├── README.md
├── Makefile
├── bin/ : Destination of executing binaries (spline_interpolator, unit_test)
├── lib/ : Destination of static library (libspline_interpolator.a)
├── images/ : Destination of plotting graph & csv by unit_test
├── include/
│   ├── spline_interpolator/
│   ├── non_uniform_rounding_spline.hpp
│   ├── spline_interpolator.hpp
│   └── spline_exception.hpp
├── src/
│   ├── main.cpp
│   ├── spline_interpolator.cpp
│   ├── non_uniform_rounding_spline.cpp
│   └── cubic_spline_interpolator.cpp
└── test/
    ├── test_cubic_spline_interpolator.cpp
    ├── test_spline_interpolator.cpp
    ├── unit_test.cpp
    └── util/
        ├── gnuplot_realtime.cpp
        ├── gnuplot_realtime.hpp
        └── test_graph_plot.hpp

```

&nbsp;

# Make

```
$ make
```

&nbsp;

# The output

## Binary

- bin/spline\_interpolator
- bin/unit\_test

## Library

- lib/libspline\_interpolator.a

&nbsp;

# Test

```
$ ./bin/unit_test
```

Interpolated graphs will be generated into ./images/*.{png,csv}  
when the fllowing tests are executed.

- CubicSplineTest


&nbsp;

<div align="right"> That's All. </div>

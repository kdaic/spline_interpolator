spline_interpolator
===

This is practice for implementing spline-interpolator

# Submodule

plog is used for logger.

```
$ git subomodule update --init
```

# Dependency

g++, googletest, gnuplot

# Directory Map

```
.
├── README.md
├── Makefile
├── bin : Destination of binaries splne_interpolator, unit_test
├── submodules
│   └── plog : 3rdparty logger tool.
├── include
│   └── spline_interpolator
│       ├── cubic_spline_interpolator.hpp
│       ├── spline_interpolator.hpp
├── lib : Destination of static library libspline_interpolator.a
├── src
│   ├── main.cpp
│   ├── spline_interpolator.cpp
│   └── cubic_spline_interpolator.cpp
└── test
    ├── unit_test.cpp
    └── util
        ├── gnuplot_realtime.cpp
        └── gnuplot_realtime.hpp

```

# Make

```
$ make
```

# Destination

## Binary

- spline\_interpolator
- unit\_test

## Library

- libspline\_interpolator.a

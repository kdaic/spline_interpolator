path_interpolator
===

This is practice for implementing path-interpolator

# Submodule

plog is used for logger.

```
$ git subomodule update --init
```

# Other Dependency

g++, googletest, gnuplot

# Directory Map

```
.
├── README.md
├── Makefile
├── bin : Destination of binaries (path_interpolator, unit_test)
├── lib : Destination of static library (libpath_interpolator.a)
├── include
│   ├── spline_interpolator
│   │   ├── cubic_spline_interpolator.hpp
│   │   └── path_interpolator.hpp
│   └── submodules
│       └── plog : 3rdparty logger tool.
├── src
│   ├── main.cpp
│   ├── path_interpolator.cpp
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

- path\_interpolator
- unit\_test

## Library

- libspath\_interpolator.a

path_interpolator
===

This is practice for implementing path-interpolator

# Submodule

plog( for logger ).

```
$ git submodule update --init
```

# Other Dependency

g++, googletest, gnuplot( for unit_test )

# Directory Map

```
.
├── README.md
├── Makefile
├── bin/ : Destination of executing binaries (path_interpolator, unit_test)
├── lib/ : Destination of static library (libpath_interpolator.a)
├── images/ : Destination of plotting graph & csv by unit_test
├── include/
│   ├── path_interpolator/
│   │   ├── cubic_spline_interpolator.hpp
│   │   ├── non_uniform_rounding_spline.hpp
│   │   ├── path.hpp
│   │   ├── path_interpolator.hpp
│   │   └── path_exception.hpp
│   └── submodules/
│       └── plog/ : 3rdparty logger tool.
├── src/
│   ├── main.cpp
│   ├── path_interpolator.cpp
│   ├── non_uniform_rounding_spline.cpp
│   └── cubic_spline_interpolator.cpp
└── test/
    ├── test_cubic_spline_interpolator.cpp
    ├── test_path_interpolator.cpp
    ├── unit_test.cpp
    └── util/
        ├── gnuplot_realtime.cpp
        ├── gnuplot_realtime.hpp
        └── test_graph_plot.hpp

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

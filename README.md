# SGMwSMC++

This is an R/C++ implementation of [Sequential decision model for inference and prediction on nonuniform hypergraphs
with application to knot matching from computational forestry](https://doi.org/10.1214/19-AOAS1255).
Its name derives from the original [SGMwSMC](https://github.com/junseonghwan/sgmwsmc) codebase, which this is a partial
port of; specifically, this is a port of the TrainAndPredict experiment.

## Installation

### As an R package (recommended)

Install the latest package version from R with

```r
devtools::install_github("tyxchen/sgmwsmcpp")
```

### As a standalone C++ executable

Make sure you have Boost ≥ 1.69 and Eigen ≥ 3.3 installed and visible to CMake before continuing.

```
$ git clone https://github.com/tyxchen/sgmwsmcpp
$ cd sgmwsmcpp/src
$ mkdir build
$ cd build
$ cmake ..
$ make sgmwsmc
```

The default build type is `Release`; change this with `-DCMAKE_BUILD_TYPE` when running CMake.

#### With a custom Boost installation

Inside the build folder, run

```
$ export BOOST_ROOT=path/to/boost/root
$ cmake \
  -DBoost_NO_BOOST_CMAKE=TRUE \
  -DBoost_NO_SYSTEM_PATHS=TRUE \
  -DBoost_INCLUDE_DIRS=path/to/boost/include \
  -DBoost_LIBRARY_DIRS=path/to/boost/libraries \
  ..
```

## Usage

### Through R

The easiest way to use this package is through R by calling the `train`, `predict`, or `train_and_predict` functions.

- If you have a relatively small training dataset and don't care about saving the intermediate parameters, running
  `train_and_predict` is the easiest and most efficient method. An example can be found in `demo/train_and_predict.R`.
- If you have a larger training dataset, or you wish to run predictions on multiple datasets, save the intermediate
  parameters from running `train`, and then pass those parameters into `predict`. An example can be found in
  `demo/train_predict_separate.R`.

### Through C++

A driver function is written in `src/src/main.cpp`. This is the main function that runs in the `sgmwsmc` executable.
Feel free to tweak to your liking, but if you choose to run it vanilla, the barebones command line is

```
$ sgmwsmc --data-directories path/to/training/data --test-data-directories path/to/testing/data --output-dir path/to/output/folder
```

Further options are documented in the driver function.

In the future, I may investigate making this buildable as a CMake library, as there are multiple methods exposed in
`src/src/Experiments.h` that can be called from external applications.

## External libraries

The following libraries are used heavily in this project:

- [Boost](https://boost.org/)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [TBB](https://github.com/oneapi-src/oneTBB)
- [The Parallel Hashmap](https://github.com/greg7mdp/parallel-hashmap/)

Although not linked to or used in either the R interface or the shared library, these libraries also find usage
elsewhere:

- [fast-cpp-csv-parser](https://github.com/ben-strasser/fast-cpp-csv-parser)
- [cxxopts](https://github.com/jarro2783/cxxopts/)

## License

LGPL 2.1 or later, see LICENSE for details.

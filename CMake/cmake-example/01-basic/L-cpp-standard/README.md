# C++ Standard

Since the release of C++11 and C++14 a common use case is to invoke the compiler to use these standards. As CMake has evolved, it has added features to make this easier and new versions of CMake have changed how this is achieved.

The following examples show different methods of setting the C++ standard and what versions of CMake they are available from.

The examples include:

- [common-method](https://github.com/ttroy50/cmake-examples/blob/master/01-basic/L-cpp-standard/i-common-method). A simple version that should work with most versions of CMake.
- [cxx-standard](https://github.com/ttroy50/cmake-examples/blob/master/01-basic/L-cpp-standard/ii-cxx-standard). Using the `CMAKE_CXX_STANDARD` variable introduced in CMake v3.1.
- [compile-features](https://github.com/ttroy50/cmake-examples/blob/master/01-basic/L-cpp-standard/iii-compile-features). Using the `target_compile_features` function introduced in CMake v3.1.
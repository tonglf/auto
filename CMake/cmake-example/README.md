# CMake Examples

项目地址：https://github.com/ttroy50/cmake-examples.git

## Introduction

[CMake](https://cmake.org/) is a cross-platform open-source meta-build system which can build, test and package software. It can be used to support multiple native build environments including make, Apple’s xcode and Microsoft Visual Studio.

This repository includes some example modern CMake configurations which I have picked up when exploring it’s usage for various projects. The examples are laid out in a tutorial like format. The first examples are very basic and slowly increase in complexity drawing on previous examples to show more complex use cases.

These examples have been tested on Ubuntu 16.04 but should work under any Linux system that supports CMake v3.5+.

This branch works with the CMake version 3.5 onwards.

- For examples that use CMake version 2.x see the branch [v2-style-includes](https://github.com/ttroy50/cmake-examples/tree/v2-style-includes).
- For examples that use CMake version 3.0 see the branch [v3.0-minimum](https://github.com/ttroy50/cmake-examples/tree/v3.0-minimum)

## Requirements

The basic requirements for most examples are:

- CMake v3.5+
- A c++ compiler (defaults to gcc)
- make

### Installation on Ubuntu

The easiest way to install the above on Ubuntu is as follows

```bash
$ sudo apt-get install build-essential
$ sudo apt-get install cmake
```

Some specific examples may require other tools including:

- [boost](http://www.boost.org/)

    ```bash
    $ sudo apt-get install libboost-all-dev
    ```

- [protobuf](https://github.com/google/protobuf)

    ```bash
    $ sudo apt-get install libprotobuf-dev
    $ sudo apt-get install protobuf-compiler
    ```

- [cppcheck](http://cppcheck.sourceforge.net/)

    ```bash
    $ sudo apt-get install cppcheck
    ```

- [clang](http://clang.llvm.org/)

    ```bash
    $ sudo apt-get install clang-3.6
    ```

- [ninja](https://ninja-build.org/)

    ```bash
    $ sudo apt-get install ninja-build
    ```

- [conan](https://conan.io/)

    ```bash
    $ sudo apt-get install python3 python3-pip
    $ sudo pip3 install conan
    ```

### Docker

Docker containers with all requirements and various versions of CMake are generated to help make testing the examples easier. These are available from the docker hub repository [matrim/cmake-examples](https://hub.docker.com/r/matrim/cmake-examples/).

To build the full set of cmake-examples test cases you can run:

```bash
docker run -it matrim/cmake-examples:3.5.1
cd ~
git clone https://github.com/ttroy50/cmake-examples.git code
cd code
./test.sh
```

For more details on build and running the docker containers [dockerfiles](https://github.com/ttroy50/cmake-examples/blob/master/here).

# Other Links

There are many CMake tutorials and examples online. The list below includes links to some of these which I have found helpful in my CMake journey.

- [Modern CMake Slides](https://web.archive.org/web/20160314094326/https://www.kdab.com/~stephen/moderncmake.pdf)
- [rix0r Modern CMake Blog](https://rix0r.nl/blog/2015/08/13/cmake-guide/)
- [Official CMake Tutorial](https://cmake.org/cmake-tutorial/)
- [Official CMake Wiki](https://gitlab.kitware.com/cmake/community/wikis/home)
- [CMake Useful Variables](https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/Useful-Variables)
- [Derek Molloy - Intro to CMake](http://derekmolloy.ie/hello-world-introductions-to-cmake/)
- [Modular C++ Projects](http://techminded.net/blog/modular-c-projects-with-cmake.html)
- [Common CMake Anti-Patterns](https://web.archive.org/web/20190320121339/http://voices.canonical.com/jussi.pakkanen/2013/03/26/a-list-of-common-cmake-antipatterns/)
- [Using clang static analyser with CMake](http://baptiste-wicht.com/posts/2014/04/install-use-clang-static-analyzer-cmake.html)
- [Static Analysis with CDash](https://cmake.org/pipermail/cmake/2011-April/043709.html) - Includes some info about using CppCheck with CMake
- [CMake Tips](https://samthursfield.wordpress.com/2015/10/20/some-cmake-tips/)
- [John Lamp - CMake Tutorial](https://www.johnlamp.net/cmake-tutorial.html)
- [Conan Documentation](https://docs.conan.io/)
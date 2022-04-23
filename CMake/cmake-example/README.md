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

## 例子解释

- 01-basic  一些基础的cmake例子。
    - hello-cmake. A hello world example.
    - hello-headers. 一个稍微复杂一点的hello world示例，使用单独的源代码和包含文件夹。
    - static-library. 使用静态库的示例。
    - shared-library. 使用共享库的示例。
    - installing. 演示如何创建将安装二进制文件和库的“make install”目标。
    - build-type. 演示如何为项目设置默认生成和优化标志的示例。
    - compile-flags. 演示如何设置其他编译标志。
    - third-party-library. 显示了如何链接第三方库的示例。
    - compiling-with-clang. 调用clang编译器的示例。
    - building-with-ninja. 演示如何生成 ninja 构建文件。
    - imported-targets - 演示如何使用新导入的目标链接boost。
    - cpp-standard - 介绍C++标准的各种设置方法。
- 02-sub-projects  包含子文件的情况，这时父目录与子目录均有 CMakeLists.txt 文件。
- 03-code-generation  代码生成，其对于从公共描述文件创建不同语言的源代码非常有用，使用变量或借助工具可以实现，本文只尝试了使用变量。
- 04-static-analysis  静态分析，在不执行代码的情况下对代码进行分析。它可以用来发现常见的编程错误并执行编码准则。介绍了四种工具进行分析，本文只尝试使用了 clang-analyzer。
- 05-unit-testing  单元测试是一个软件开发过程中应用程序的最小可测试部分，单元被单独和独立地检查是否正常运行。介绍了三种单元测试工具，本文尝试使用了 boost 和 googletest。
- 06-installer  创建 deb 安装包。
- 07-package-management  包管理。例如：使用系统提供的 “find_package()” 函数管理第三方库。

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
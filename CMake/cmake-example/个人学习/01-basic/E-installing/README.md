# Installing

## Introduction

This example shows how to generate a `make install` target to install files and binaries on your system. This is based on the previous shared library example.

The files in this tutorial are below:

```bash
$ tree
.
├── cmake-examples.conf
├── CMakeLists.txt
├── include
│   └── installing
│       └── Hello.h
├── README.adoc
└── src
    ├── Hello.cpp
    └── main.cpp
```

  * CMakeLists.txt - Contains the CMake commands you wish to run
  * cmake-examples.conf - An example configuration file
  * include/installing/Hello.h - The header file to include
  * src/Hello.cpp - A source file to compile
  * src/main.cpp - The source file with main

## Concepts

### Installing

CMake offers the ability to add a `make install` target to allow a user to install binaries, libraries and other files. The base install location is controlled by the variable CMAKE_INSTALL_PREFIX which can be set using ccmake or by calling cmake with `cmake .. -DCMAKE_INSTALL_PREFIX=/install/location`

安装的文件由 install() 函数控制。

```cmake
install (TARGETS cmake_examples_inst_bin
    DESTINATION bin)
```

将从目标cmake_examples_inst_bin目标生成的二进制文件安装到目标 ${cmake_Install_PREFIX}/bin

```cmake
install (TARGETS cmake_examples_inst
    LIBRARY DESTINATION lib)
```

将从目标cmake_examples_inst目标生成的共享库安装到目标 ${cmake_Install_PREFIX}/lib

> NOTE
>
> 这在windows上可能不起作用。在具有DLL目标的平台上，可能需要添加以下内容
> 
>```cmake
> install (TARGETS cmake_examples_inst
>  LIBRARY DESTINATION lib
>     RUNTIME DESTINATION bin)
>    ```
> 

```cmake
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION include)
```

将针对cmake_examples_inst库开发的头文件安装到${cmake_Install_PREFIX}/include目录中。

```cmake
install (FILES cmake-examples.conf
    DESTINATION etc)
```

将配置文件安装到目标${CMAKE_Install_PREFIX}/etc

运行make install后，CMake会生成一个安装清单。txt文件，包括所有已安装文件的详细信息。

> NOTE
>
> 如果以root用户身份运行make install命令，则安装清单。txt文件将归root所有。

## Building the Example

```bash
$ mkdir build

$ cd build/

$ cmake ..

$ make

$ sudo make install

$ cat install_manifest.txt
/usr/local/bin/cmake_examples_inst_bin
/usr/local/lib/libcmake_examples_inst.so
/usr/local/etc/cmake-examples.conf

$ ls /usr/local/bin/
cmake_examples_inst_bin

$ ls /usr/local/lib
libcmake_examples_inst.so

$ ls /usr/local/etc/
cmake-examples.conf

$ LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib cmake_examples_inst_bin
Hello Install!
```

> NOTE
>
> 如果/usr/local/lib不在库路径中，则可能需要在运行二进制文件之前将其添加到路径中。

### Extra Notes(附加说明)

#### Overriding the default install location(覆盖默认安装位置)

如前所述，默认安装位置是从CMAKE_INSTALL_PREFIX 设置的，该前缀默认为/usr/local/

如果要更改所有用户的默认位置，可以在添加任何二进制文件或库之前，将以下代码添加到顶级CmakeList.txt。

```cmake
if( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )
  message(STATUS "Setting default CMAKE_INSTALL_PREFIX path to ${CMAKE_BINARY_DIR}/install")
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING "The path to use for make install" FORCE)
endif()
```

This example sets the default install location to under your build directory.

#### DESTDIR

如果您希望分期安装以确认包含所有文件，则make install target支持DESTDIR参数。

```bash
make install DESTDIR=/tmp/stage
```

这将为所有安装文件创建安装路径\${DESTDIR}/${CMAKE_install_PREFIX}。在本例中，它将在路径 /tmp/stage/usr/local 下安装所有文件

```
$ tree /tmp/stage
/tmp/stage
└── usr
    └── local
        ├── bin
        │   └── cmake_examples_inst_bin
        ├── etc
        │   └── cmake-examples.conf
        └── lib
            └── libcmake_examples_inst.so
```

#### Uninstall

By default CMake does not add a `make uninstall` target. For details on how to generate an uninstall target see this [FAQ](https://cmake.org/Wiki/CMake_FAQ#Can_I_do_.22make_uninstall.22_with_CMake.3F)

For an easy way to remove the files from this example, you can use:

```bash
sudo xargs rm < install_manifest.txt
```
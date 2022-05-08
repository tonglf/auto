# Glog

## 安装

GitHub 地址：https://github.com/google/glog.git

```bash
git clone https://github.com/google/glog.git

cd glog

mkdir build

cd build

cmake ..

make

sudo make install		# 头文件位置：/usr/local/include
```

## 测试

### main.cpp

```cpp
#include <iostream>

#include "glog/logging.h"   // glog 头文件

using namespace std;

int main(int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);    // 初始化
    //将所有日志输出到文件和stderr(终端)
    FLAGS_alsologtostderr = 1;

    //FLAGS_log_dir设置日志输出目录。
    FLAGS_log_dir = "/home/tong/testglog/log"; 
    // FLAGS_log_dir=".";   

    cout << "jjj" << endl;
    LOG(INFO) << "hello glog";     // 打印log：“hello glog.  类似于C++ stream

    return 0;
}
```



### CMakeLists.txt

```cmake
cmake_minimum_required (VERSION 3.16)
project (myproj VERSION 1.0)

find_package (glog 0.6.0 REQUIRED)

add_executable (myapp main.cpp)
target_link_libraries (myapp glog::glog)
```



### 文件布局

```bash
$ tree .
.
├── build
├── CMakeLists.txt
├── log				# 用于存放 log 文件
└── main.cpp
```



### 编译执行

```bash
$ ./myapp
jjj
I20220508 21:11:11.637158 22086 main.cpp:18] hello glog
```



### 文件布局

```bash
$ tree .
.
├── build
│   ├── CMakeCache.txt
│   ...
│   └── myapp
├── CMakeLists.txt
├── log
│   ├── myapp.INFO -> myapp.tong-virtual-machine.tong.log.INFO.20220508-211111.22086
│   └── myapp.tong-virtual-machine.tong.log.INFO.20220508-211111.22086
└── main.cpp
```



### 日志输出

在 log 文件夹下生成两个文件：myapp.INFO、myapp.tong-virtual-machine.tong.log.INFO.20220508-211111.22086

两个文件内容一样：

```txt
Log file created at: 2022/05/08 21:11:11
Running on machine: tong-virtual-machine
Running duration (h:mm:ss): 0:00:00
Log line format: [IWEF]yyyymmdd hh:mm:ss.uuuuuu threadid file:line] msg
I20220508 21:11:11.637158 22086 main.cpp:18] hello glog
```


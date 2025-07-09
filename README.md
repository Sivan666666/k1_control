# k1_control

jaka k1 dual-arm robot contorl of c++

## 1. Install dependence

install protobuf-3.6.1 (include libprotobuf.so.17):

### 下载并安装 protobuf 3.6.x

```
wget https://github.com/protocolbuffers/protobuf/releases/download/v3.6.1/protobuf-cpp-3.6.1.tar.gz
tar -xzf protobuf-cpp-3.6.1.tar.gz
cd protobuf-3.6.1
./configure --prefix=/usr/local/protobuf-3.6.1
make -j$(nproc)
sudo make install
```

更新库路径
```
sudo ldconfig
export LD_LIBRARY_PATH=/usr/local/protobuf-3.6.1/lib:$LD_LIBRARY_PATH
```

## 2. How to use cpp build:

### build

```
cd k1_control
rm -r build # 可选
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release    # 生成Makefile
make -j$(nproc)    # 并行编译
```

if make error, please check the path of protobuf-3.6.1

### run

`cd build`

所有可执行文件会生成在build目录下，for example:

`./turn_cloth_demo`

### add new controller

在 `src/` 下编辑新的 .cpp 文件。并且在 `/k1_control/CMakeLists.txt` 文件中的源文件声明处添加新的 .cpp 文件

```
# 显式声明源文件
set(SOURCE_FILES
    src/main.cpp
    src/main2.cpp
    src/turn_cloth_demo.cpp
    src/new_xxxxx.cpp
)
```

build 时会基于 .cpp 文件在 `build/` 目录下生成可执行文件

## 3. How to build in Ros workspace:

set `k1_control\` root path in `/k1_control/ros/CMakeLists.txt` :

```
# 设置源码路径
set(K1_CONTROL_ROOT "/home/shu/robo_arm/k1_control")
```

setting workspace:

`mkdir robo_ws/src && cd robo_ws`

`ln -sf path/to/k1_control/ros src/k1_control_ros`

install dependence:

`rosdep install --from-paths src --ignore-src -y`

build Ros package:

`catkin build k1_control_ros`

run node:

`source devel/setup.bash`

`roslaunch k1_control_ros state_pub.launch`
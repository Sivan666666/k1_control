# k1_control

jaka k1 dual-arm robot contorl of c++

## 1. install dependence

install protobuf-3.6.1 (include libprotobuf.so.17):

### 下载并安装 protobuf 3.6.x

`wget https://github.com/protocolbuffers/protobuf/releases/download/v3.6.1/protobuf-cpp-3.6.1.tar.gz`

`tar -xzf protobuf-cpp-3.6.1.tar.gz`

`cd protobuf-3.6.1`

`./configure --prefix=/usr/local/protobuf-3.6.1`

`make -j$(nproc)`

`sudo make install`

### 更新库路径

`sudo ldconfig`

`export LD_LIBRARY_PATH=/usr/local/protobuf-3.6.1/lib:$LD_LIBRARY_PATH`

## 2. How to use:

### build

`cd k1_control/Linux`

`rm -r build && mkdir build && cd build`

`cmake ..`

`make`

if make error, please check the path of protobuf-3.6.1


### run

`cd build`

for example:
`./turn_cloth_demo`
# 1. 安装第三方库

## 1.1 eigen3

```
# 直接安装
sudo apt-get install libeigen3-dev
# 安装结束后，系统默认将头文件安装在/usr/include/eigen3/Eigen上，但在c++中习惯地使用Eigen/Core的形式
# 来include，因此还需要把Eigen文件夹单独放置在/usr/include中
sudo cp -r /usr/include/eigen3/Eigen /usr/include
```

## 1.2 cddlib

参考[官网](https://github.com/cddlib/cddlib)中**Build the Latest Released Version**，依次执行：

```
# 下载后解压，make
tar zxf cddlib-*.tar.gz
cd cddlib-*
./configure
make

# 安装，安装位置默认是/usr/local/include
sudo make install
# 也可以将它统一放到/usr/include
sudo cp -r /usr/local/include/cddlib /usr/include/
```

## 1.3 mosek

参考[这里的操作](https://blog.csdn.net/qq_37390296/article/details/88697300)

## 1.4 pybind11

https://blog.csdn.net/qq_27149279/article/details/121352696
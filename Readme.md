# 4WIS MPC四轮独立求解路径跟踪(move_base框架)

## 项目依赖：

```
ubuntu20.04
matplotlibcpp
cmake
Eigen
osqp
osqpeigen
IPOPT
CPPAD
```

- cmake安装：

```
sudo apt install cmake
```

- Eigen安装：

```
sudo apt-get install libeigen3-dev
```

- Ipopt和Cppad安装：

1.拉取代码：

```
https://github.com/Night-elf-1/4WISMPC-movebase.git
```

2.在工作空间根目录下编译：

```
catkin_make
```

## 使用方法：

1.启动小车gazebo仿真模型：

```
roslaunch car_model start.launch
```

2.启动move_base：

```
roslaunch mpc_move_base movebasestart.launch
```


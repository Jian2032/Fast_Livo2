# FAST-LIVO2

## FAST-LIVO2: 快速和紧密耦合稀疏-直接 LiDAR-惯性-可视里程计

## 1. 环境配置

### 1.1 Ubuntu and ROS

Ubuntu 20.04.  [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 1.2 PCL && Eigen && OpenCV

PCL>=1.6, Follow [PCL Installation](https://pointclouds.org/). 

```bash
sudo apt install libpcl-dev
```
Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

```bash
sudo apt install libeigen3-dev
```

OpenCV>=3.2, Follow [Opencv Installation](http://opencv.org/).

### 1.3 Sophus

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

上述步骤可能会报错,解决方案

```bash
/home/sun/Sophus/sophus/so2.cpp:32:26: error: lvalue required as left operand of assignment
   unit_complex_.real() = 1.;
                          ^~
/home/sun/Sophus/sophus/so2.cpp:33:26: error: lvalue required as left operand of assignment
```

打开其位置so2.cpp:32:26改为

```bash
SO2::SO2()
{
  unit_complex_.real(1.);
  unit_complex_.imag(0.);
}
```

### 1.4 Livox SDK2

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

## 2. 构建

### 2.1 livox_ros_driver2

下载livox_ros_driver2并单独安装，不然后面会报错

```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```

### 2.2 Vikit

```bash
# Different from the one used in fast-livo1
cd ws_livox/src
git clone https://github.com/xuankuzcr/rpg_vikit.git 
```

### 2.3 FAST-LIVO2

```bash
cd ~/ws_livox/src
git clone https://github.com/hku-mars/FAST-LIVO2
cd ../
catkin_make
source devel/setup.bash
```

将FAST-LIVO2中有关livox_ros_driver报错的部分，将livox_ros_driver改为livox_ros_driver2

报错:包含在 /home/Fast_LIVO2_ws/src/FAST-LIVO2/src/IMU_Processing.cpp：13 的文件中：
/home/Fast_LIVO2_ws/src/FAST-LIVO2/include/IMU_Processing.h：52：12： 错误：字段 'fout_imu' 的类型不完整 'std：：ofstream' {又名 'std：：basic_ofstream'}

解决：在IMU_Processing.h里include一个fstream的头文件，重新catkin_make就可以了  #include <fstream.h>

## 3. 运行

通过 OneDrive （FAST-LIVO-Datasets） 下载收集的 rosbag 文件，其中包含 4 个 rosbag 文件。([**FAST-LIVO2-Dataset**](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z)). 

```
roslaunch fast_livo mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag
```
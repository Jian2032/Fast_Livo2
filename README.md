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

将CMakeLists.txt中的ROS2部分删除，避免编译出错

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

### 3.1 运行rosbag文件

通过 OneDrive （FAST-LIVO-Datasets） 下载收集的 rosbag 文件，其中包含 4 个 rosbag 文件。([**FAST-LIVO2-Dataset**](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z)). 

```bash
roslaunch fast_livo mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag
```

### 3.2 运行海相机和mid360

```bash
roslaunch mvs_ros_driver mvs_camera_trigger.launch
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_livo mapping_mid360.launch
```

## 4. NVIDIA Xavier NX安装支持CUDA加速的opencv4.6.0

### 4.1 卸载自带opencv4.2.0

```bash
sudo apt-get purge libopencv*
sudo apt-get purge python-numpy
sudo apt autoremove 
sudo apt-get update
```

### 4.2 安装依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libv4l-dev v4l-utils libopenblas-dev liblapack-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran libhdf5-dev protobuf-compiler libgoogle-glog-dev libgflags-dev

```

### 4.3 安装opencv4.6

下载 opencv 源码，选择所需要的版本 opencv 4.6.0，相应的扩展 opencv_contrib 4.6.0，以及用于桥接 ROS 和 opencv 的 cv_bridge

https://github.com/opencv/opencv/releases/tag/4.6.0
https://github.com/opencv/opencv_contrib/releases/tag/4.6.0
https://github.com/ros-perception/vision_opencv/tree/noetic

解压后将opencv_contrib 4.6.0放入opencv 4.6.0文件夹中

```bash
cd ~/opencv-4.6.0/
mkdir build && cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.6.0/modules \
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN=7.2 \
      -D CUDA_ARCH_PTX=7.2 \
      -D ENABLE_FAST_MATH=ON \
      -D CUDA_FAST_MATH=ON \
      -D WITH_CUBLAS=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_GSTREAMER=ON \
      -D WITH_GSTREAMER_0_10=OFF \
      -D WITH_QT=OFF \
      -D WITH_OPENGL=ON \
      -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
      -D WITH_TBB=ON \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_EXAMPLES=OFF \
      ..

make -j$(nproc)  # 自动使用所有 CPU 核心
sudo make install

```

CMAKE_INSTALL_PREFIX=/usr/local/ 为安装地址，

OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.6.0/modules 为扩展模块所在路径，

CUDA_ARCH_BIN=7.2 为 GPU 算力

### 4.4 pkg-config --modversion opencv4依旧显示4.2的问题

删除opencv4.2.0残留文件
检查 OpenCV 冲突的 Bash 脚本 check_opencv_conflict.sh

```bash
chmod +x check_opencv_conflict.sh
./check_opencv_conflict.sh
```

安全删除系统中所有 OpenCV 旧版本（如 4.2）库和 .pc 文件，仅保留 OpenCV 4.6 的 Bash 脚本 remove_old_opencv.sh

```bash
chmod +x remove_old_opencv.sh
./remove_old_opencv.sh
```

一键清理并重新构建 cv_bridge 以链接 OpenCV 4.6 的脚本 rebuild_cv_bridge.sh

```bash
chmod +x rebuild_cv_bridge.sh
./rebuild_cv_bridge.sh
```

手动替换 opencv4.pc 为 4.6 的版本

第一步：确认 OpenCV 4.6 的安装路径

```bash
/usr/local/lib/pkgconfig/opencv4.pc
```

第二步：删除旧 .pc 文件

```bash
sudo rm -f /usr/lib/pkgconfig/opencv4.pc
sudo rm -f /usr/share/pkgconfig/opencv4.pc
```

第三步：为 OpenCV 4.6 手动生成 opencv4.pc 文件

给chatgpt提供

```bash
ls /usr/local/lib | grep opencv
ls /usr/local/include/opencv4
```
生成更精确的 opencv4.pc

我就能帮你自动生成更精确的 opencv4.pc

```bash
sudo tee /usr/local/lib/pkgconfig/opencv4.pc > /dev/null <<EOF
prefix=/usr/local
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib
includedir=\${prefix}/include/opencv4

Name: OpenCV
Description: Open Source Computer Vision Library
Version: 4.6.0
Libs: -L\${libdir} \
  -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio \
  -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_ml -lopencv_objdetect \
  -lopencv_photo -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_video \
  -lopencv_videostab -lopencv_xfeatures2d -lopencv_ximgproc -lopencv_xphoto \
  -lopencv_dnn -lopencv_dnn_objdetect -lopencv_dpm -lopencv_face -lopencv_text \
  -lopencv_tracking -lopencv_phase_unwrapping -lopencv_saliency -lopencv_bgsegm \
  -lopencv_bioinspired -lopencv_ccalib -lopencv_datasets -lopencv_freetype \
  -lopencv_hdf -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -lopencv_rapid \
  -lopencv_reg -lopencv_rgbd -lopencv_surface_matching -lopencv_wechat_qrcode \
  -lopencv_alphamat -lopencv_aruco \
  -lopencv_cudaarithm -lopencv_cudabgsegm -lopencv_cudacodec -lopencv_cudafeatures2d \
  -lopencv_cudafilters -lopencv_cudaimgproc -lopencv_cudalegacy -lopencv_cudaobjdetect \
  -lopencv_cudaoptflow -lopencv_cudastereo -lopencv_cudawarping -lopencv_cudev
Cflags: -I\${includedir}
EOF
```

第四步：导出PKG_CONFIG_PATH

```bash
echo 'export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig' >> ~/.bashrc
source ~/.bashrc
```

第五步：验证

```bash
pkg-config --modversion opencv4

```

## 5. Github教程

### 5.1 在命令行上创建新存储库

```bash
git init
git add README.md
git commit -m "Fast_Livo2"
git branch -M main
git remote add origin https://github.com/Jian2032/Fast_Livo2.git
git push -u origin main
```

### 5.2 从命令行推送现有存储库

```bash
git remote add origin https://github.com/Jian2032/Fast_Livo2.git
git branch -M main
git push -u origin main
```

## 6. 相机标定 Camera Calibration

参考：https://zhaoxuhui.top/blog/2021/02/02/ros-camera-calibration.html


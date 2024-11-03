

# The Mercator Descriptor: A Novel Global Descriptor for Loop Closure Detection

Welcome to the Mercator Descriptor repository! The Mercator Descriptor is a novel descriptor for loop closure detection and place recognition. This descriptor first uses a Mercator-like projection to generate a rotation-invariant descriptor from a frame of point clouds. Then, by constructing a database to store this new descriptor, it enables efficient matching between historical and current frames for loop closure detection.
![](https://github.com/wangzika/Mercator-Descriptor/blob/main/Graphical%20abstract.jpg)


# ** Prerequisites**

## ** Ubuntu and [ROS](https://www.ros.org/)**
We tested our code on Ubuntu18.04 with ros melodic and Ubuntu20.04 with noetic. Additional ROS package is required:
```
sudo apt-get install ros-xxx-pcl-conversions
```

## ** Eigen**
Following the official [Eigen installation](eigen.tuxfamily.org/index.php?title=Main_Page), or directly install Eigen by:
```
sudo apt-get install libeigen3-dev
```
## ** ceres-solver (version>=2.1)**
Please kindly install ceres-solver by following the guide on [ceres Installation](http://ceres-solver.org/installation.html). Notice that the version of ceres-solver should higher than [ceres-solver 2.1.0](https://github.com/ceres-solver/ceres-solver/releases/tag/2.1.0)

## ** GTSAM**
Following the official [GTSAM installation](https://gtsam.org/get_started/), or directly install GTSAM 4.x stable release by:
```
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```
**!! IMPORTANT !!**: Please do not install the GTSAM of ***develop branch***, which are not compatible with our code! We are still figuring out this issue.


## ** Prepare for the data**
Since this repo does not implement any method (i.e., LOAM, LIO, etc) for solving the pose for registering the LiDAR scan. So, you have to prepare two set of data for reproducing our results, include: **1) the LiDAR point cloud data. 2) the point cloud registration pose.**

## Examples 


video: [Youtube link](https://www.youtube.com/watch?v=uXGNIHsfkAw)


### ** Example-1: place recognition with KITTI Odometry dataset**
<div align="center">
<img src="https://github.com/wangzika/Mercator-Descriptor/blob/main/loop%20detection1.GIF"  width="48%" loop/>
<img src="https://github.com/wangzika/Mercator-Descriptor/blob/main/loop%20detection.GIF"  width="48%" loop/>
</div>



### ** Example-2: loop closure correction on our dataset**
<div align="center">
<img src="https://github.com/wangzika/Mercator-Descriptor/blob/main/captum_r.jpg"  width="80%" loop/>
<img src="https://github.com/wangzika/Mercator-Descriptor/blob/main/captum_lc.jpg"  width="80%" loop/>
</div>
The figure depicts an overview of the Backpack laser scanning system and the campus dataset, as well as the resulting graphs with and without loop closure.

# Developers:
Wang Zhibo(e-mail:582796566@qq.com)


# Credits

We hereby recommend reading [Scan Context](https://github.com/gisbi-kim/scancontext_tro) ,[FAST_LIO](https://github.com/hku-mars/FAST_LIO) ,[A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and [STD](https://github.com/hku-mars/STD) for reference and thank them for making their work public.

# License

The source code is released under GPLv3 license.

I am constantly working on improving this code. For any technical issues or commercial use, please contact me(582796566@qq.com).

# LiDAR Obstacle Detection

## Introduction

This project shows how to process raw point cloud data obtained from a LiDAR sensor to perform obstacle detection. In this project, the point cloud processing is done using C++ and Point Cloud Library (PCL). Point cloud segmentation is done using Random Sampling Consensus (RANSAC) algorithm and Euclidean Clustering is used to find clusters in the point cloud data. Finally, a bounding box is created around the detected clusters which show the obstacles detected in the scene. The LiDAR data is pre-processed before applying all the steps mentioned above. Pre-processing is done to downsample the LiDAR data so that obstacle detection can be done efficiently without unnecessarily processing a large number of data points. Voxel grid is used for downsampling the LiDAR data points. Voxel grid allows only one point per voxel which decreases the resolution of the point cloud allowing faster processing. The result of LiDAR point cloud processing is shown in the GIF above.


## Project Build Instructions
### Ubuntu
```bash
git clone https://github.com/sumukhpatil/LiDAR-Obstacle-Detection.git
cd LiDAR-Object-Detection
mkdir build && cd build
cmake ..
make
./environment
```

## Build Dependencies
- cmake version 2.8 or above ([click here for installation instructions](https://cmake.org/install/))
- make version 4.1 or above
- g++/gcc version 5.4 and above
- pcl version 1.2 or above

## Point Cloud Library (PCL) Installation
### Ubuntu

```bash
sudo apt update
sudo apt upgrade
sudo apt install libpcl-dev

```

### Windows

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew
	```bash
	brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps)
	```bash
	brew tap brewsci/science
	```
4. view pcl install options
	```bash
	brew options pcl
	```
5. install PCL
	```bash
	brew install pcl
	```

# Lidar Obstacle Detection

This project is an example of filtering, segmenting, clustering, and viewing lidar data taken from a self-driving car.

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

The filtering and viewing of the data is provided by the <a href="http://pointclouds.org/">Point Cloud Library or PCL</a>.  The segmentation, obstacle clustering, and boxing of obstacles is performed by the code in the project. 

## Dependencies and Requirements

This project was developed on Ubuntu 16.04.  It may work on Windows, Mac, and other Ubuntu versions but has not been tested as such.

The code requires the <a href="http://pointclouds.org/">Point Cloud Library or PCL</a>.  Installation instructions are included below.


## Installing PCL

To install the Point Cloud Library, follow the instructions below for your environment.  These instructions were copied from https://github.com/udacity/SFND_Lidar_Obstacle_Detection/edit/master/README.md.  You may also find additional installation instructions on the <a href="http://pointclouds.org/">Point Cloud Library or PCL</a> website.

### Ubuntu 
This installation for Ubuntu is quite lengthy - up to an hour - so be patient.  Several additional dependencies, like Eigen, are installed as well.

$> sudo apt install libpcl-dev


### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)

## Compiling and Running The Project

To acquire, compile, and run the project in Ubuntu, follow the statements below.  Windows and Mac environments should be modified accordingly.

```bash
$> git clone https://github.com/TheOnceAndFutureSmalltalker/lidar_obstacle_detection.git
$> cd lidar_obstacle_detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

If all goes well, you will see a window pop up with an animation of the modified point cloud that looks similar to the graphic at the top of this readme.

## Discussion

For a more in depth discussion ...

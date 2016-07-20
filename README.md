ofxPCL - custom
=========

Description
-----------
update:
* As of June 2016 homebrew installs PCL version 1.8, which i have not yet tested.
* In order to make my current apps work i copied the lib folder contained in [lib_172.zip] (https://github.com/antimodular/ofxPCL/blob/master/lib_172.zip) in to usr/local/opt/pcl .

original:
* None of the other ofxPCL addons worked for me. Maybe they are too old or i am too old.
* This version uses [PCL 1.7.2] (http://pointclouds.org/downloads/macosx.html)
* Download this ofxPCL and place it in your addon folder.
* Open your terminal and install homebrew as described here: <http://brew.sh>; i.e. copy past that one line and execute.
* Follow the instruction on how to install PCL via homebrew: <http://www.pointclouds.org/documentation/tutorials/installing_homebrew.php>; i.e. `brew tap homebrew/science` and then `brew install pcl`
* Here are some notes i took during troubleshooting this setup: <https://gist.github.com/antimodular/75abe99baacb3aee0dab#gistcomment-1727264> , incase i installed some stuff deep in the belly of the computer that all this needs in order to run.
* I want to thank Roy from www.morethantechnical.com for helping me through some of the tricky bits.

System
------
* This was tested on OSX 10.10.5 with OF 0.9.3 and Xcode 6.4.
* Does not work on OSX 10.8.3, because PCL does not want to install.

Examples
--------
* ICP (Interactive Iterative Closest Point):  
taken from http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
It is an algorithm used to minimize the difference between two clouds of points. The example shows how to find the transformation matrix between two point clouds and use it to move cloud A in the same place as cloud B. ICP can only finding the transformation matrix when the difference between the two clouds is not too big.

* PCA (Principal Component Analysis):  
taken from http://pointclouds.org/documentation/tutorials/normal_estimation.php
It is a statistical procedure that extracts the most important features of a dataset.
The example shows how to get the bounding box and finds the direction along which the point data varies the most.
PCA has it's limits, it lacks up and down orientation information. That's why you can see the direction line flip after 180˚ rotation.

* SAC_IA (Sample Consensus Initial Alignment):  
taken from http://pointclouds.org/documentation/tutorials/template_alignment.php
This example does not work yet. Somehow the 2 point clouds do not return any reasonable transformation matrix and fitness score.


Setup New Project
-----------------
* Don't use Project Generator to add ofxPCL to your project. It seems to take super long. Instead add ofxPCL by hand.

* Drag the .h files from ../../../addons/ofxPCL/src in to your Xcode project, without copying them

* Add all PCL dynamic library files (dylib) from ofxPCL/libs/pcl/lib to your xcode projects Build Phases -> Link Binary With Libraries section.
* This will also automatically add this line to your Build Settings -> Library Search Paths: 
$(SYSTEM_APPS_DIR)/of_v0.9.3_osx_release/addons/ofxPCL/libs/pcl/lib
* Also the dylibs will now appear in the left hand navigator area.

* Add the following lines to your Build Settings -> User Header Search Path:  
../../../addons/ofxPCL/libs/pcl/include/pcl-1.7  
../../../addons/ofxPCL/libs/pcl/include  
../../../addons/ofxPCL/libs/pcl/include/flann  
../../../addons/ofxPCL/libs/pcl/include/eigen3  

Hints
-----
* When copying one of the example folders in to a new location you might have to run the project through project generator to have all other addons be linked correctly. Then follow the setup steps to add ofPCL

Screenshots
-----------------
* Here are some screenshots of my project setup:
![](https://raw.githubusercontent.com/antimodular/ofxPCL/master/add_headerSearchPath.jpg)
![](https://raw.githubusercontent.com/antimodular/ofxPCL/master/add_dylibs.jpg)


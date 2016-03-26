ofxPCL - custom
=========


Description
-----------
* None of the other ofxPCL addons worked for me. Maybe they are too old or i am not old enough.

Setup New Project
-----------------
* Don't use Project Generator to add ofxPCL to your project. It seems to take super long. Instead add ofxPCL by hand.

* Drag the .h files from ../../../addons/ofxPCL/src in to your xcode project, without copying them

* Add all PCL dynamic library files (dylib) from ofxPCL/libs/pcl/lib to your xcode projects Build Phases -> Link Binary With Libraries section.
* This will also automatically add this line to you Build Settings -> Linrary Search Paths: 
$(SYSTEM_APPS_DIR)/of_v0.9.3_osx_release/addons/ofxPCL/libs/pcl/lib
* Also the dylibs will now appeare in the left hand navigator area.

* Add the following lines to your Build Settings -> User Header Search Path:
../../../addons/ofxPCL/libs/pcl/include/pcl-1.7
../../../addons/ofxPCL/libs/pcl/include
../../../addons/ofxPCL/libs/pcl/include/flann
../../../addons/ofxPCL/libs/pcl/include/eigen3


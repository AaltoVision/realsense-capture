# RealSense capture

Tool for capturing all output from Intel RealSense Tracking Camera T265.

## Compile

* Compile librealsense using their instructions (you can omit examples and such)
* Compile odometry (or at least recorder lib)
* Create build dir `mkdir target`
* Go to build dir `cd target`
* Cmake and make the build `cmake .. && make`

## Run

* Run the program from the root dir i.e. `cd ..` if you are in target/, then `target/rscapture`
* Stop recording with Enter
* Output is stored under output/ dir timestamp from when recording started 

# Device spec

More detailed info about T265 hardware: https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
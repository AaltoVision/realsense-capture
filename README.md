# RealSense capture

Tool for capturing all output from Intel RealSense Tracking Camera T265.

## Dependencies

* odometry for Recoder library (included as git submodule)
* librealsense for Recoder library (included as git submodule, optional for Linux)
* OpenCV for writing video files (install opencv using your favorite package manager)

## Compile

* Install librealsense using their instructions
  - on Linux, it's highly recommended to install the prebuilt packages as instructed [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)... unless you are comfortable with manually compiling and installing kernel modules.
   The Ubuntu packages also seem to work on Debian and also with kernel versions not on their list: Successfully tested on Debian Stretch with Linux kernel 4.14 by installing `librealsense2-dkms`, `librealsense2-utils` and `librealsense2-dev` from Intel's Ubuntu Xenial repository (because Xenial has kernel 4.4, which is older than the 4.14 in Stretch). Note that then you do not have to even checkout the `librealsense` submodule
* Compile odometry (or at least recorder lib). Note that if you are really lazy and have an existing installation, just `rmdir odometry` and symlink: `ln -s /path/to/existing/odometry .`. Then you do not have to recompile
* Install `libsdl2-dev` from package management (on Linux)
* Create build dir `mkdir target`
* Go to build dir `cd target`
* Cmake and make the build `cmake .. && make`

## Run

* Make output directory if it doesn't exist yet `mkdir output`
* Run the program from the root dir i.e. `cd ..` if you are in target/, then `target/rscapture`
* Stop recording with Enter
* Output is stored under output/ dir with filenames including a timestamp from when recording started

## Example output

Output is 3 files, timestampped by the UTC start time of the recording session. Timestamps in jsonl file are seconds after first recorded measurement with tiny offset to avoid 0.0. Device has two cameras, `left` (cameraInd 0) corresponds to left "eye" of the device if you were to hold it so you can read "RealSense" on the back of the device and `right` (cameraInd 1) is the camera closer to USB connector.

```
recording-2020-03-10T13-11-11Z-left.avi
recording-2020-03-10T13-11-11Z-right.avi
recording-2020-03-10T13-11-11Z.jsonl
```

Information about the camera lenses is also stored to jsonl. This is only stored once, for example:

```
{"cameraInd":0,"coeffs":[-0.00200599804520607,0.03895416110754013,-0.03715667128562927,0.0061612860299646854,0.0],"model":"RS2_DISTORTION_KANNALA_BRANDT4"}
{"cameraInd":1,"coeffs":[-0.002046247012913227,0.037283919751644135,-0.03449400141835213,0.005099817179143429,0.0],"model":"RS2_DISTORTION_KANNALA_BRANDT4"}
```

# Device spec

More detailed info about T265 hardware: https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md

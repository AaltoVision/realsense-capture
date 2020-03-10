# RealSense capture

Tool for capturing all output from Intel RealSense Tracking Camera T265.

## Dependencies

* odometry for Recoder library (included as git submodule)
* librealsense for Recoder library (included as git submodule)
* OpenCV for writing video files (install opencv using your favorite package manager)

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

## Example output

Output is 3 files, timestampped by the UTC start time of the recording session. Timestamps in jsonl file are seconds after first recorded measurement with tiny offset to avoid 0.0. Device has two cameras, `left` corresponds to left "eye" of the device if you were to hold it so you can read "RealSense" on the back of the device and `right` is the camera closer to USB connector.

```
recording-2020-03-10T13-11-11Z-left.avi
recording-2020-03-10T13-11-11Z-right.avi
recording-2020-03-10T13-11-11Z.jsonl
```

# Device spec

More detailed info about T265 hardware: https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
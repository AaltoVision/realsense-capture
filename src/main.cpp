// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
//#include <librealsense2/rs.hpp>
#include <librealsense2/rs.hpp>
#include <recorder.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

std::string currentISO8601TimeUTC() {
  auto now = std::chrono::system_clock::now();
  auto itt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << std::put_time(gmtime(&itt), "%FT%H-%M-%SZ");

  return ss.str();
}

void printPoseData(rs2_pose& pose_data, double time) {
    std::cout << "\r" << std::setprecision(3) << std::fixed
        << "Time[" << time << "]"
        << ", Pos[" << pose_data.translation.x << "," <<  pose_data.translation.y << "," << pose_data.translation.z << "]"
        << ", Vel[" << pose_data.velocity.x << "," <<  pose_data.velocity.y << "," << pose_data.velocity.z << "]"
        << ", Acc[" << pose_data.acceleration.x << "," <<  pose_data.acceleration.y << "," << pose_data.acceleration.z << "]"
        << ", Rot[" << pose_data.rotation.x << "," <<  pose_data.rotation.y << "," << pose_data.rotation.z << pose_data.rotation.w << "]"
        << "                                       ";
}

int main(int argc, char * argv[]) try {

    std::cout << "Connecting to device...\n";

    auto startTime = std::chrono::high_resolution_clock::now();
    auto startTimeString = currentISO8601TimeUTC();
    auto outputPrefix = "output/recording-" + startTimeString;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // 6 Degrees of Freedom pose data, calculated by RealSense device
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF /*, 15 (fps)*/);
    // Native stream of gyroscope motion data produced by RealSense device
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F /*, 15 (fps)*/);
    // Native stream of accelerometer motion data produced by RealSense device
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F /*, 15 (fps)*/);
    // Native stream of fish-eye (wide) data captured from the dedicate motion camera
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    auto recorder = recorder::Recorder::build(outputPrefix + ".json");
    cv::VideoWriter* videoWriters[2];

    std::mutex dataMutex;

    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    auto callback = [&](const rs2::frame& frame) {
        std::lock_guard<std::mutex> lock(dataMutex);

        // Cast the frame that arrived to motion frame, accelerometer + gyro
        auto motion = frame.as<rs2::motion_frame>();
        // If casting succeeded and the arrived frame is from gyro stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
            rs2_vector gyro_data = motion.get_motion_data();
            recorder->addGyroscope(motion.get_timestamp(), gyro_data.x, gyro_data.y, gyro_data.z);
        }
        // If casting succeeded and the arrived frame is from accelerometer stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
            rs2_vector accel_data = motion.get_motion_data();
            recorder->addAccelerometer(motion.get_timestamp(), accel_data.x, accel_data.y, accel_data.z);
        }

        // Cast to pose frame
        auto pose = frame.as<rs2::pose_frame>();
        if (pose && pose.get_profile().stream_type() == RS2_STREAM_POSE && pose.get_profile().format() == RS2_FORMAT_6DOF) {
            auto poseData = pose.get_pose_data();
            // Print some values for user to see everything is working
            printPoseData(poseData, pose.get_timestamp());
            // Store data to JSON
            recorder->addOdometryOutput({
                .time = pose.get_timestamp(),
                .position = {
                    .x = poseData.translation.x,
                    .y = poseData.translation.y,
                    .z = poseData.translation.z
                },
                .orientation = {
                    .x = poseData.rotation.x,
                    .y = poseData.rotation.y,
                    .z = poseData.rotation.z,
                    .w = poseData.rotation.w
                }
            }, {
                .x = poseData.velocity.x,
                .y = poseData.velocity.y,
                .z = poseData.velocity.z
            });
        }

        // Cast to frameset that contains video feed from all cameras
        auto frameset = frame.as<rs2::frameset>();
        if (frameset && frameset.get_profile().stream_type() == RS2_STREAM_FISHEYE && frameset.get_profile().format() == RS2_FORMAT_Y8) {
            // Process feed from both cameras
            std::vector<recorder::Recorder::FrameData> frameGroup;
            for (int index = 0; index < 2; index++) {
                rs2::video_frame vf = frameset.get_fisheye_frame(index + 1); // Camera index starts at 1
                // Save frame metadata
                auto vprofile = vf.get_profile().as<rs2::video_stream_profile>();
                auto intrinsics = vprofile.get_intrinsics();
                recorder::Recorder::FrameData frameData({
                    .t = vf.get_timestamp(),
                   .cameraInd = index,
                   .focalLength = intrinsics.fx,
                   .px = intrinsics.ppx,
                   .py = intrinsics.ppy
                });
                frameGroup.push_back(frameData);
                // Save frame
                const uint8_t* imageData = (const uint8_t*)(vf.get_data());
                int width = vf.get_width();
                int height = vf.get_height();
                cv::Mat grayFrame(height, width, CV_8UC1, (void*)imageData);
                if (!videoWriters[index]) {
                    const std::string path = outputPrefix + "-" + (index == 0 ? "left" : "right") + ".avi";
                    const auto codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
                    const auto backend = cv::CAP_OPENCV_MJPEG;
                    const auto fps = (float)vprofile.fps();
                    videoWriters[index] = new cv::VideoWriter(path, backend, codec, fps, grayFrame.size(), true);
                }
                cv::Mat colorFrame; // Grayscale video is only supported on Windows, so we must convert to BGR
                cv::cvtColor(grayFrame, colorFrame, cv::COLOR_GRAY2BGR);
                videoWriters[index]->write(colorFrame);
            }
            recorder->addFrameGroup(frameGroup[0].t, frameGroup);
        }
    };

    // Start pipeline with chosen configuration
    auto profile = pipe.start(cfg, callback);

    std::cout << "!!! Press Enter to stop !!!\n";

    // Wait for user input before stopping
    std::cin.ignore();

    std::cout << "\nExiting. Waiting recorder thread to finish...\n";

    pipe.stop();

    for (int index = 0; index < 2; index++) {
        delete videoWriters[index];
    }

    std::cout << "Bye!\n";

    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
//#include <librealsense2/rs.hpp>
#include <librealsense2/rs.hpp>
#include <recorder.hpp>
#include <iostream>
#include <iomanip>


void printPoseData(rs2_pose pose_data) {
    /*  
    rs2_vector 	translation
    rs2_vector 	velocity
    rs2_vector 	acceleration
    rs2_quaternion 	rotation
    rs2_vector 	angular_velocity
    rs2_vector 	angular_acceleration
    unsigned int 	tracker_confidence
    unsigned int 	mapper_confidence
    */
    std::cout << "\r" << std::setprecision(3) << std::fixed
        << "Pos[" << pose_data.translation.x << "," <<  pose_data.translation.y << "," << pose_data.translation.z << "]"
        << ", Vel[" << pose_data.velocity.x << "," <<  pose_data.velocity.y << "," << pose_data.velocity.z << "]"
        << ", Acc[" << pose_data.acceleration.x << "," <<  pose_data.acceleration.y << "," << pose_data.acceleration.z << "]"
        ;
}


int main(int argc, char * argv[]) try {
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF /*, 15 (fps)*/);

    // https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#a01b4027af33139de861408872dd11b93
    // RS2_STREAM_ANY 	
    // RS2_STREAM_DEPTH 	Native stream of depth data produced by RealSense device
    // RS2_STREAM_COLOR 	Native stream of color data captured by RealSense device
    // RS2_STREAM_INFRARED  Native stream of infrared data captured by RealSense device
    // RS2_STREAM_FISHEYE 	Native stream of fish-eye (wide) data captured from the dedicate motion camera
    // RS2_STREAM_GYRO 	    Native stream of gyroscope motion data produced by RealSense device
    // RS2_STREAM_ACCEL 	Native stream of accelerometer motion data produced by RealSense device
    // RS2_STREAM_GPIO 	    Signals from external device connected through GPIO
    // RS2_STREAM_POSE 	    6 Degrees of Freedom pose data, calculated by RealSense device
    // RS2_STREAM_CONFIDENCE 	4 bit per-pixel depth confidence level
    // RS2_STREAM_COUNT 

    // Start pipeline with chosen configuration
    pipe.start(cfg);

    auto recorder = recorder::Recorder::build("output/recording"); // TODO: Read from args

    // Main loop
    while (true)     {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        // Print the x, y, z values of the translation, relative to initial position
        printPoseData(pose_data);
        //std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
        //    pose_data.translation.y << " " << pose_data.translation.z << " (meters)";
    }
    
    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

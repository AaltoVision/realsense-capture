// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
//#include <librealsense2/rs.hpp>
#include <librealsense2/rs.hpp>
#include <recorder.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>

std::atomic<bool> userTerminated(false);

void printPoseData(rs2_pose& pose_data, double time) {
    /*  
 typedef struct rs2_pose
 {
     rs2_vector      translation;          
     rs2_vector      velocity;             
     rs2_vector      acceleration;         
     rs2_quaternion  rotation;             
     rs2_vector      angular_velocity;     
     rs2_vector      angular_acceleration; 
     unsigned int    tracker_confidence;   
     unsigned int    mapper_confidence;    
 } rs2_pose;
    */
    std::cout << "\r" << std::setprecision(3) << std::fixed
        << "Time[" << time << "]"
        << "Pos[" << pose_data.translation.x << "," <<  pose_data.translation.y << "," << pose_data.translation.z << "]"
        << ", Vel[" << pose_data.velocity.x << "," <<  pose_data.velocity.y << "," << pose_data.velocity.z << "]"
        << ", Acc[" << pose_data.acceleration.x << "," <<  pose_data.acceleration.y << "," << pose_data.acceleration.z << "]"
        << "                                       ";
}

int main(int argc, char * argv[]) try {

    std::cout << "Connecting to device...\n";

    auto startTime = std::chrono::high_resolution_clock::now();

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#a01b4027af33139de861408872dd11b93
    // RS2_STREAM_ANY 	
    // RS2_STREAM_DEPTH 	Native stream of depth data produced by RealSense device    
    // RS2_STREAM_COLOR 	Native stream of color data captured by RealSense device
    // RS2_STREAM_INFRARED  Native stream of infrared data captured by RealSense device
    // RS2_STREAM_FISHEYE 	Native stream of fish-eye (wide) data captured from the dedicate motion camera
    // RS2_STREAM_GPIO 	    Signals from external device connected through GPIO
    // RS2_STREAM_POSE 	    
    // RS2_STREAM_CONFIDENCE 	4 bit per-pixel depth confidence level
    // RS2_STREAM_COUNT 

    // 6 Degrees of Freedom pose data, calculated by RealSense device
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF /*, 15 (fps)*/);
    // Native stream of gyroscope motion data produced by RealSense device
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F /*, 15 (fps)*/);
    // Native stream of accelerometer motion data produced by RealSense device
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F /*, 15 (fps)*/);

    std::cout << "Device connected\n";

    auto recorder = recorder::Recorder::build("output/recording"); // TODO: Read from args

    // Start pipeline with chosen configuration
    auto profile = pipe.start(cfg, [&](rs2::frame frame) {
        // Cast the frame that arrived to motion frame
        auto motion = frame.as<rs2::motion_frame>();
        // If casting succeeded and the arrived frame is from gyro stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
            // Get the timestamp of the current frame
            double ts = motion.get_timestamp();
            // Get gyro measures
            rs2_vector gyro_data = motion.get_motion_data();
            recorder->addGyroscope(ts, gyro_data.x, gyro_data.y, gyro_data.z);    
        }
        // If casting succeeded and the arrived frame is from accelerometer stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // Get the timestamp of the current frame
            double ts = motion.get_timestamp();
            // Get accelerometer measures
            rs2_vector accel_data = motion.get_motion_data();            
            recorder->addAccelerometer(ts, accel_data.x, accel_data.y, accel_data.z);    
        }
        auto pose = frame.as<rs2::pose_frame>();
        if (pose && pose.get_profile().stream_type() == RS2_STREAM_POSE && pose.get_profile().format() == RS2_FORMAT_6DOF) {
            auto poseData = pose.get_pose_data();; //f.as<rs2::pose_frame>().get_pose_data();
            // Get time since recording started
            //std::chrono::duration<double> time = std::chrono::high_resolution_clock::now() - startTime;
            //double dtime = time.count();    
            double ts = pose.get_timestamp();        
            // Print some values for user to see everything is working
            printPoseData(poseData, ts);
            // Store data to JSON
            recorder->addOdometryOutput({
                .time = ts,
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
    });

    std::cout << "Recording!\n";

    std::cout << "!!! Press Enter to stop !!!\n";

    // Wait for user input before stopping
    std::cin.ignore();

    std::cout << "\nExiting. Waiting recorder thread to finish...\n";
    
    pipe.stop();

    std::cout << "Bye!\n";
    
    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

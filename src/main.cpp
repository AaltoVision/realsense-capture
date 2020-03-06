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

/*
std::unique_ptr<API::Pose> convertPose(rs2_pose pose_data, double time) {
    auto dazzling_pose = std::unique_ptr<API::Pose>(new API::Pose());
    dazzling_pose->time = time;
    dazzling_pose->position = {
        pose_data.translation.x,
        pose_data.translation.y,
        pose_data.translation.z
    };
    dazzling_pose->orientation = {
        pose_data.rotation.x,
        pose_data.rotation.y,
        pose_data.rotation.z,
        pose_data.rotation.w
    };
    return dazzling_pose;
}
*/

void record() {
    std::cout << "Connecting to device...\n";

    auto startTime = std::chrono::high_resolution_clock::now();

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

    std::cout << "Device connected\n";

    auto recorder = recorder::Recorder::build("output/recording"); // TODO: Read from args

    std::cout << "Recording!\n";

    std::cout << "!!! Press Enter to stop !!!\n";

    // Main loop
    while (userTerminated == false)     {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        // Print the x, y, z values of the translation, relative to initial position
        printPoseData(pose_data);

        std::chrono::duration<double> time = std::chrono::high_resolution_clock::now() - startTime;

        recorder->addGroundTruth({
            .time = time.count(),
            .position = {
                .x = pose_data.translation.x,
                .y = pose_data.translation.y,
                .z = pose_data.translation.z
            },
            .orientation = {                
                .x = pose_data.rotation.x,
                .y = pose_data.rotation.y,
                .z = pose_data.rotation.z,
                .w = pose_data.rotation.w
            }
        });
        //std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
        //    pose_data.translation.y << " " << pose_data.translation.z << " (meters)";
    }
    std::cout << "\n";

    std::cout << "Closing files\n";

    recorder->closeOutputFile();
}

int main(int argc, char * argv[]) try {

    std::thread recorderThread(record);

    // TODO: switch to while loop to not terminate on everything
    //std::string userInput;
    std::cin.ignore(); // >> userInput;

    std::cout << "Exiting. Waiting recorder thread to finish...\n";
    userTerminated = true;
    recorderThread.join();

    std::cout << "Bye!\n";
    
    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

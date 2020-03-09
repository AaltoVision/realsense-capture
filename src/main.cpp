// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
//#include <librealsense2/rs.hpp>
#include <librealsense2/rs.hpp>
#include <recorder.hpp>
//#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>
#include <iomanip>
#include <sstream>

std::string currentISO8601TimeUTC() {
  auto now = std::chrono::system_clock::now();
  auto itt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << std::put_time(gmtime(&itt), "%FT%TZ");
  return ss.str();
}

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

    // Unused streams, not all are supported by the tracking device model, T265.
    // https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#a01b4027af33139de861408872dd11b93
    // RS2_STREAM_ANY 	
    // RS2_STREAM_DEPTH 	Native stream of depth data produced by RealSense device    
    // RS2_STREAM_COLOR 	Native stream of color data captured by RealSense device
    // RS2_STREAM_INFRARED  Native stream of infrared data captured by RealSense device
    // RS2_STREAM_GPIO 	    Signals from external device connected through GPIO
    // RS2_STREAM_CONFIDENCE 	4 bit per-pixel depth confidence level
    // RS2_STREAM_COUNT 

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


    // https://intelrealsense.github.io/librealsense/doxygen/structrs2__intrinsics.html

    auto recorder = recorder::Recorder::build("output/recording-" + currentISO8601TimeUTC());

    std::mutex dataMutex;


    /*
    uint64_t pose_counter = 0;
    uint64_t frame_counter = 0;
    bool first_data = true;
    auto last_print = std::chrono::system_clock::now();
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        // Only start measuring time elapsed once we have received the
        // first piece of data
        if (first_data) {
            first_data = false;
            last_print = std::chrono::system_clock::now();
        }

        if (auto fp = frame.as<rs2::pose_frame>()) {
            pose_counter++;
        }
        else if (auto fs = frame.as<rs2::frameset>()) {
            frame_counter++;
        }

        // Print the approximate pose and image rates once per second
        auto now = std::chrono::system_clock::now();
        if (now - last_print >= std::chrono::seconds(1)) {
            std::cout << "\r" << std::setprecision(0) << std::fixed
                      << "Pose rate: "  << pose_counter << " "
                      << "Image rate: " << frame_counter << std::flush;
            pose_counter = 0;
            frame_counter = 0;
            last_print = now;
        }
    };*/

    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    auto callback = [&](const rs2::frame& frame) {
        std::lock_guard<std::mutex> lock(dataMutex);

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

        // Cast to pose frame
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

        // Cast to fisheye frame
        auto frameset = frame.as<rs2::frameset>();        

/*
         	video_frame (const frame &f)
int 	get_width () const
int 	get_height () const
int 	get_stride_in_bytes () const
int 	get_bits_per_pixel () const
int 	get_bytes_per_pixel () const
*/
        if (frameset && frameset.get_profile().stream_type() == RS2_STREAM_FISHEYE && frameset.get_profile().format() == RS2_FORMAT_Y8) {            
            // Process feed from both cameras
            for (int index = 0; index < 2; index++) {
                rs2::video_frame vf = frameset.get_fisheye_frame(index);
                

                const uint8_t* imageData = (const uint8_t*)(vf.get_data());
                auto numBytes = vf.get_stride_in_bytes();

                // TODO: read numBytes from imageData
                
                //std::cout << "Video frame " << index << ", " << vf.get_width() << "x" << vf.get_height() << "\n";
            }

            // Get and cast to video profile
            /*
            auto videoProfile = video.get_profile().as<rs2::video_stream_profile>();
            // TODO: Save metadata about lens etc. here?
            int index = videoProfile.stream_index(); // 1 or 2
            std::cout << "Video frame " << index << "\n";
*/

            // TODO: Process video here.
        }
    };


    // Start pipeline with chosen configuration
    auto profile = pipe.start(cfg, callback);

    
    // TODO: Read this stuff from frame profile within callback?
    // Get fisheye sensor intrinsics parameters
    // int fisheye_sensor_idx = 1 and then fisheye_sensor_idx = 2 to get both. Will probably be identical?
    //rs2::stream_profile fisheye_stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
    //rs2_intrinsics intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();



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

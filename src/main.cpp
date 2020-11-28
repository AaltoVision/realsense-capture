#include <librealsense2/rs.hpp>
#include <jsonl-recorder/recorder.hpp>
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

std::string distortionToString(rs2_distortion distortionModel) {
    switch(distortionModel) {
        case rs2_distortion::RS2_DISTORTION_NONE: return "RS2_DISTORTION_NONE";
        case rs2_distortion::RS2_DISTORTION_MODIFIED_BROWN_CONRADY: return "RS2_DISTORTION_MODIFIED_BROWN_CONRADY";
        case rs2_distortion::RS2_DISTORTION_INVERSE_BROWN_CONRADY: return "RS2_DISTORTION_INVERSE_BROWN_CONRADY";
        case rs2_distortion::RS2_DISTORTION_FTHETA: return "RS2_DISTORTION_FTHETA";
        case rs2_distortion::RS2_DISTORTION_BROWN_CONRADY: return "RS2_DISTORTION_BROWN_CONRADY";
        case rs2_distortion::RS2_DISTORTION_KANNALA_BRANDT4: return "RS2_DISTORTION_KANNALA_BRANDT4";
        case rs2_distortion::RS2_DISTORTION_COUNT: return "RS2_DISTORTION_COUNT";
    }
    return "UNKNOWN";
}

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

    constexpr bool RECORD_VIDEO = true;
    constexpr bool RECORD_POSE = true;
    constexpr bool RECORD_IMU = true;

    auto startTime = std::chrono::high_resolution_clock::now();
    auto startTimeString = currentISO8601TimeUTC();
    auto outputPrefix = "output/recording-" + startTimeString;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    if (RECORD_POSE) {
      // 6 Degrees of Freedom pose data, calculated by RealSense device
      cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF /*, 15 (fps)*/);
    }

    if (RECORD_IMU) {
      // Native stream of gyroscope motion data produced by RealSense device
      cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F /*, 15 (fps)*/);
      // Native stream of accelerometer motion data produced by RealSense device
      cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F /*, 15 (fps)*/);
    }

    if (RECORD_VIDEO) {
      // Native stream of fish-eye (wide) data captured from the dedicate motion camera
      // Note: It is not currently possible to enable only one
      cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
      cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
    }

    auto recorder = recorder::Recorder::build(outputPrefix + ".jsonl", outputPrefix + "-video.avi");
    bool videoInitialized[2] = { false, false };
    // reuse for color conversion to avoid memory allocation on each frame
    cv::Mat colorFrames[2];

    std::mutex dataMutex;

    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    double firstMeasurementTime = -1.;
    auto callback = [&](const rs2::frame& frame) {
        std::lock_guard<std::mutex> lock(dataMutex);

        // Convert timestamp to seconds after first measurement
        double timeStamp = frame.get_timestamp();
        if (firstMeasurementTime < 0.0) {
            firstMeasurementTime = timeStamp;
        }
        timeStamp = (timeStamp - firstMeasurementTime) / 1000.;
        if (timeStamp <= 0.0) { // Ensure time is always non-zero and positive
            timeStamp = 0.00000001;
        }

        // Cast the frame that arrived to motion frame, accelerometer + gyro
        auto motion = frame.as<rs2::motion_frame>();
        // If casting succeeded and the arrived frame is from gyro stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
            rs2_vector gyro_data = motion.get_motion_data();
            recorder->addGyroscope(timeStamp, gyro_data.x, gyro_data.y, gyro_data.z);
        }
        // If casting succeeded and the arrived frame is from accelerometer stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
            rs2_vector accel_data = motion.get_motion_data();
            recorder->addAccelerometer(timeStamp, accel_data.x, accel_data.y, accel_data.z);
        }

        // Cast to pose frame
        auto pose = frame.as<rs2::pose_frame>();
        if (pose && pose.get_profile().stream_type() == RS2_STREAM_POSE && pose.get_profile().format() == RS2_FORMAT_6DOF) {
            auto poseData = pose.get_pose_data();
            // Print some values for user to see everything is working
            printPoseData(poseData, timeStamp);
            // Store data to JSON
            recorder->addOdometryOutput({
                .time = timeStamp,
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
            std::vector<recorder::FrameData> frameGroup;
            for (int index = 0; index < 2; index++) {
                rs2::video_frame vf = frameset.get_fisheye_frame(index + 1); // Camera index starts at 1
                // Save frame metadata
                auto vprofile = vf.get_profile().as<rs2::video_stream_profile>();
                auto intrinsics = vprofile.get_intrinsics();

                // Save frame
                const uint8_t* imageData = (const uint8_t*)(vf.get_data());
                assert(imageData != nullptr);

                int width = vf.get_width();
                int height = vf.get_height();
                cv::Mat grayFrame(height, width, CV_8UC1, (void*)imageData);

                // Grayscale video is only supported on Windows, so we must convert to BGR
                cv::cvtColor(grayFrame, colorFrames[index], cv::COLOR_GRAY2BGR);

                recorder::FrameData frameData({
                   .t = timeStamp,
                   .cameraInd = index,
                   .focalLengthX = intrinsics.fx,
                   .focalLengthY = intrinsics.fy,
                   .px = intrinsics.ppx,
                   .py = intrinsics.ppy,
                   .frameData = colorFrames + index
                });
                frameGroup.push_back(frameData);
                if (!videoInitialized[index]) {
                    recorder->setVideoRecordingFps((float)vprofile.fps());
                    videoInitialized[index] = true;

                    // Store lens metadata once
                    nlohmann::json lensMetadata;
                    lensMetadata["model"] = distortionToString(intrinsics.model);
                    auto coeffs = nlohmann::json::array();
                    for(auto i = 0; i < 5; ++i) {
                        coeffs.push_back(intrinsics.coeffs[i]);
                    }
                    lensMetadata["coeffs"] = coeffs;
                    lensMetadata["cameraInd"] = index;
                    recorder->addJson(lensMetadata);
                }
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

    std::cout << "Bye!\n";

    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

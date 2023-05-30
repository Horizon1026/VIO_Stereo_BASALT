#include "string"
#include "fstream"
#include "sstream"
#include "unistd.h"
#include "thread"

#include "opencv2/opencv.hpp"

#include "frontend.h"
#include "backend.h"
#include "data_loader.h"
#include "data_manager.h"

#include "log_report.h"
#include "datatype_basic.h"
#include "datatype_image.h"

void PublishImuData(const std::string &csv_file_path, const int32_t period_us = 5000) {
    std::ifstream file(csv_file_path.c_str());
    if (!file.is_open()) {
        ReportError("Faile to load imu data file " << csv_file_path);
        return;
    }

    // Print header of data file.
    std::string one_line;
    std::getline(file, one_line);
    if (one_line.empty()) {
        ReportError("Imu data file is empty. " << csv_file_path);
        return;
    } else {
        ReportInfo("Imu data file header is [ " << one_line << " ].");
    }

    // Publish each line of data file.
    double time_stamp_s = 0.0;
    TVec3<double> accel;
    TVec3<double> gyro;
    while (std::getline(file, one_line) && !one_line.empty()) {
        std::istringstream imu_data(one_line);
        imu_data >> time_stamp_s >> gyro.x() >> gyro.y() >> gyro.z() >> accel.x() >> accel.y() >> accel.z();

        // Send data to dataloader of vio.
        // TODO:

        usleep(period_us);
    }

    file.close();
}

void PublishCameraData(const std::string &csv_file_path, const int32_t period_us = 50000) {
    std::ifstream file(csv_file_path.c_str());
    if (!file.is_open()) {
        ReportError("Faile to load camera data file " << csv_file_path);
        return;
    }

    // Print header of data file.
    std::string one_line;
    std::getline(file, one_line);
    if (one_line.empty()) {
        ReportError("Camera data file is empty. " << csv_file_path);
        return;
    } else {
        ReportInfo("Camera data file header is [ " << one_line << " ].");
    }

    // Publish each line of data file.
    double time_stamp_s = 0.0;
    std::string image_file_name;
    while (std::getline(file, one_line) && !one_line.empty()) {
        std::istringstream camera_left_data(one_line);
        camera_left_data >> time_stamp_s >> image_file_name;

        cv::Mat image = cv::imread(image_file_name.c_str(), 0);
        if (image.empty()) {
            ReportError("Failed to load image file.");
            return;
        }

        // Send data to dataloader of vio.
        // TODO:

        usleep(period_us);
    }

    file.close();
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test vio stereo orb slam3 on euroc dataset." RESET_COLOR);

    return 0;
}

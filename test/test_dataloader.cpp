#include "fstream"
#include "sstream"
#include "string"
#include "thread"
#include "unistd.h"

#include "tick_tock.h"
#include "visualizor_2d.h"

#include "backend.h"
#include "data_loader.h"
#include "data_manager.h"
#include "frontend.h"

#include "basic_type.h"
#include "datatype_image.h"
#include "slam_log_reporter.h"

using namespace SLAM_VISUALIZOR;

VIO::DataLoader dataloader;
double time_stamp_offset = 1403638518.0;

void PublishImuData(const std::string &csv_file_path, const float period_ms) {
    std::ifstream file(csv_file_path.c_str());
    if (!file.is_open()) {
        ReportError("Failed to load imu data file " << csv_file_path);
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
    while (std::getline(file, one_line) && !one_line.empty()) {
        TickTock timer;

        std::istringstream imu_data(one_line);
        std::string one_item;
        uint32_t i = 0;
        double temp[7] = {};
        while (std::getline(imu_data, one_item, ',') && !one_item.empty()) {
            std::istringstream item_data(one_item);
            item_data >> temp[i];
            ++i;
        }

        // Send data to dataloader of vio.
        const double time_stamp_s = temp[0];
        const Vec3 accel = Vec3(temp[4], temp[5], temp[6]);
        const Vec3 gyro = Vec3(temp[1], temp[2], temp[3]);
        dataloader.PushImuMeasurement(accel, gyro, static_cast<float>(time_stamp_s * 1e-9 - time_stamp_offset));

        // Waiting for next timestamp.
        while (timer.TockInMillisecond() < period_ms) {
            usleep(100);
        }
    }

    file.close();
}

void PublishCameraData(const std::string &csv_file_path, const std::string &image_file_root, const float period_ms, const bool is_left_camera) {
    std::ifstream file(csv_file_path.c_str());
    if (!file.is_open()) {
        ReportError("Failed to load camera data file " << csv_file_path);
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
        TickTock timer;

        std::istringstream camera_data(one_line);
        camera_data >> time_stamp_s >> image_file_name;
        image_file_name.erase(std::remove(image_file_name.begin(), image_file_name.end(), ','), image_file_name.end());

        GrayImage image;
        Visualizor2D::LoadImage(image_file_root + image_file_name, image);
        image.memory_owner() = false;
        if (image.data() == nullptr) {
            ReportError("Failed to load image file.");
            return;
        }

        // Send data to dataloader of vio.
        dataloader.PushImageMeasurement(image.data(), image.rows(), image.cols(), static_cast<float>(time_stamp_s * 1e-9 - time_stamp_offset), is_left_camera);

        // Waiting for next timestamp.
        while (timer.TockInMillisecond() < period_ms) {
            usleep(100);
        }
    }

    file.close();
}

void TestPopSingleMeasurement(const int32_t period_us = 5000, const int32_t max_wait_ticks = 10) {
    int32_t cnt = max_wait_ticks;

    while (cnt) {
        usleep(period_us);

        VIO::SingleMeasurement meas;
        const bool res = dataloader.PopSingleMeasurement(meas);

        if (!res) {
            --cnt;
            continue;
        }

        ReportInfo(">> Data loader popped one single measurement.");

        if (meas.imu != nullptr) {
            ReportInfo("Data loader pop imu measure at time " << meas.imu->time_stamp_s << " s.");
            cnt = max_wait_ticks;
        }

        if (meas.left_image != nullptr) {
            Visualizor2D::ShowImage("Left camera image", GrayImage(meas.left_image->image));
            ReportInfo("Data loader pop left camera measure at time " << meas.left_image->time_stamp_s << " s.");
        }

        if (meas.right_image != nullptr) {
            Visualizor2D::ShowImage("Right camera image", GrayImage(meas.right_image->image));
            ReportInfo("Data loader pop right camera measure at time " << meas.right_image->time_stamp_s << " s.");
        }
        Visualizor2D::WaitKey(1);

        cnt = max_wait_ticks;
    }
}

void TestPopPackedMeasurement(const int32_t period_us = 50000, const int32_t max_wait_ticks = 10) {
    int32_t cnt = max_wait_ticks;

    while (cnt) {
        usleep(period_us);

        VIO::PackedMeasurement meas;
        const bool res = dataloader.PopPackedMeasurement(meas);

        if (!res) {
            --cnt;
            continue;
        }

        ReportInfo(">> Data loader popped one packed measurement.");

        if (!meas.imus.empty()) {
            ReportInfo("Data loader pop " << meas.imus.size() << " imu measure at time " << meas.imus.front()->time_stamp_s << " - "
                                          << meas.imus.back()->time_stamp_s << " s.");
            cnt = max_wait_ticks;
        }

        if (meas.left_image != nullptr) {
            Visualizor2D::ShowImage("Left camera image", GrayImage(meas.left_image->image));
            ReportInfo("Data loader pop left camera measure at time " << meas.left_image->time_stamp_s << " s.");
        }

        if (meas.right_image != nullptr) {
            Visualizor2D::ShowImage("Right camera image", GrayImage(meas.right_image->image));
            ReportInfo("Data loader pop right camera measure at time " << meas.right_image->time_stamp_s << " s.");
        }
        Visualizor2D::WaitKey(1);

        cnt = max_wait_ticks;
    }
}

int main(int argc, char **argv) {
    // Root direction of Euroc dataset.
    std::string dataset_root_dir;
    if (argc == 2) {
        dataset_root_dir = argv[1];
    }

    ReportInfo(YELLOW ">> Test data loader." RESET_COLOR);

    // Start threads for data pipeline and vio node.
    const float imu_timeout_ms = 3.5f;
    const float image_timeout_ms = 30.0f;
    std::thread thread_pub_imu_data {PublishImuData, dataset_root_dir + "mav0/imu0/data.csv", imu_timeout_ms};
    std::thread thread_pub_cam_left_data(PublishCameraData, dataset_root_dir + "mav0/cam0/data.csv", dataset_root_dir + "mav0/cam0/data/", image_timeout_ms,
                                         true);
    std::thread thread_pub_cam_right_data(PublishCameraData, dataset_root_dir + "mav0/cam1/data.csv", dataset_root_dir + "mav0/cam1/data/", image_timeout_ms,
                                          false);
    // std::thread thread_test_pop(TestPopSingleMeasurement, 1000, 10);
    std::thread thread_test_pop(TestPopPackedMeasurement, 2000, 10);

    // Waiting for the end of the threads. Recovery their resources.
    thread_pub_imu_data.join();
    thread_pub_cam_left_data.join();
    thread_pub_cam_right_data.join();
    thread_test_pop.join();

    return 0;
}

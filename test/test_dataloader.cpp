#include "string"
#include "fstream"
#include "sstream"
#include "unistd.h"
#include "thread"

#include "visualizor.h"

#include "frontend.h"
#include "backend.h"
#include "data_loader.h"
#include "data_manager.h"

#include "log_report.h"
#include "datatype_basic.h"
#include "datatype_image.h"

// Root direction of Euroc dataset.
std::string dataset_root_dir;
double time_stamp_offset = 1403638518.0;

VIO::DataLoader dataloader;

void PublishImuData(const std::string &csv_file_path, const int32_t period_us = 5000) {
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
    double time_stamp_s = 0.0;
    TVec3<double> accel;
    TVec3<double> gyro;
    while (std::getline(file, one_line) && !one_line.empty()) {
        std::istringstream imu_data(one_line);
        imu_data >> time_stamp_s >> gyro.x() >> gyro.y() >> gyro.z() >> accel.x() >> accel.y() >> accel.z();

        // Send data to dataloader of vio.
        dataloader.PushImuMeasurement(accel.cast<float>(), gyro.cast<float>(), static_cast<float>(time_stamp_s * 1e-9 - time_stamp_offset));

        usleep(period_us);
    }

    file.close();
}

void PublishCameraData(const std::string &csv_file_path, const std::string &image_file_root, const int32_t period_us = 50000, const bool is_left_camera = true) {
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
        std::istringstream camera_data(one_line);
        camera_data >> time_stamp_s >> image_file_name;
        image_file_name.erase(std::remove(image_file_name.begin(), image_file_name.end(), ','), image_file_name.end());

        GrayImage image;
        Visualizor::LoadImage(image_file_root + image_file_name, image);
        image.memory_owner() = false;
        if (image.data() == nullptr) {
            ReportError("Failed to load image file.");
            return;
        }

        // Send data to dataloader of vio.
        dataloader.PushImageMeasurement(image.data(), image.rows(), image.cols(), static_cast<float>(time_stamp_s * 1e-9 - time_stamp_offset), is_left_camera);

        usleep(period_us);
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
            Visualizor::ShowImage("Left camera image", meas.left_image->image);
            Visualizor::WaitKey(1);
            ReportInfo("Data loader pop left camera measure at time " << meas.left_image->time_stamp_s << " s.");
        }

        if (meas.right_image != nullptr) {
            Visualizor::ShowImage("Right camera image", meas.right_image->image);
            Visualizor::WaitKey(1);
            ReportInfo("Data loader pop right camera measure at time " << meas.right_image->time_stamp_s << " s.");
        }

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
            ReportInfo("Data loader pop " << meas.imus.size() << " imu measure at time " << meas.imus.front()->time_stamp_s <<
                " - " << meas.imus.back()->time_stamp_s << " s.");
            cnt = max_wait_ticks;
        }

        if (meas.left_image != nullptr) {
            Visualizor::ShowImage("Left camera image", meas.left_image->image);
            Visualizor::WaitKey(1);
            ReportInfo("Data loader pop left camera measure at time " << meas.left_image->time_stamp_s << " s.");
        }

        if (meas.right_image != nullptr) {
            Visualizor::ShowImage("Right camera image", meas.right_image->image);
            Visualizor::WaitKey(1);
            ReportInfo("Data loader pop right camera measure at time " << meas.right_image->time_stamp_s << " s.");
        }

        cnt = max_wait_ticks;
    }
}

int main(int argc, char **argv) {
    if (argc == 2) {
        dataset_root_dir = argv[1];
    }

    ReportInfo(YELLOW ">> Test data loader." RESET_COLOR);

    std::thread thread_pub_imu_data(PublishImuData, dataset_root_dir + "mav0/imu0/data.csv", 5000);
    std::thread thread_pub_cam_left_data(PublishCameraData, dataset_root_dir + "mav0/cam0/data.csv", dataset_root_dir + "mav0/cam0/data/", 50000, true);
    std::thread thread_pub_cam_right_data(PublishCameraData, dataset_root_dir + "mav0/cam1/data.csv", dataset_root_dir + "mav0/cam1/data/", 50000, false);
    // std::thread thread_test_pop(TestPopSingleMeasurement, 5000, 10);
    std::thread thread_test_pop(TestPopPackedMeasurement, 50000, 10);

    // Waiting for the end of the threads. Recovery their resources.
    thread_pub_imu_data.join();
    thread_pub_cam_left_data.join();
    thread_pub_cam_right_data.join();
    thread_test_pop.join();

    return 0;
}

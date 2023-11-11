#ifndef _VIO_STEREO_ORB_SLAM3_BACKEND_H_
#define _VIO_STEREO_ORB_SLAM3_BACKEND_H_

#include "imu.h"
#include "data_manager.h"
#include "binary_data_log.h"
#include "general_graph_optimizor.h"

namespace VIO {

using namespace SLAM_UTILITY;
using namespace SLAM_DATA_LOG;
using namespace SLAM_SOLVER;
using namespace SENSOR_MODEL;

/* Options for Backend. */
struct BackendOptions {
    bool kEnableRecordBinaryCurveLog = true;
    uint32_t kMethodIndexToEstimateGyroBiasForInitialization = 1;
};

/* Packages of log to be recorded. */
#pragma pack(1)
struct BackendLog {
    uint8_t is_initialized = 0;
};
#pragma pack()

/* Status of Backend. */
union BackendStatus {
    struct {
        uint32_t is_initialized : 1;
        uint32_t reserved : 31;
    };
    uint32_t all_bits = 0;
};
static_assert(sizeof(BackendStatus) == sizeof(uint32_t), "Size of BackendStatus should be equal to size of uint32_t.");

/* Class Backend Declaration. */
class Backend final {

public:
    Backend() = default;
    ~Backend() = default;

    bool RunOnce();
    void Reset();
    void ResetToReintialize();

    // Support for vio initialization.
    bool TryToInitialize();
    bool ConvertNewFramesToCovisibleGraphForInitialization();

    // Estimate gyro bias for initialization.
    bool EstimateGyroBiasForInitialization();
    bool EstimateGyroBiasByMethodOneForInitialization();
    bool EstimateGyroBiasByMethodTwoForInitialization();
    bool EstimateGyroBiasByMethodThreeForInitialization();

    // Estimate velocity and gravity for initialization.
    bool EstimateVelocityAndGravityForInitialization();
    bool SelectTwoFramesWithMaxParallax(CovisibleGraphType *local_map,
                                        const FeatureType &feature,
                                        int32_t &frame_id_l,
                                        int32_t &frame_id_r);

    // Support for backend.
    void RecomputeImuPreintegration();

    // Reference for member variables.
    BackendOptions &options() { return options_; }
    DataManager *&data_manager() { return data_manager_; }
    std::unique_ptr<Imu> &imu_model() { return imu_model_; }
    bool &should_quit() { return should_quit_; }

    // Const reference for member variables.
    const BackendOptions &options() const { return options_; }
    const std::unique_ptr<Imu> &imu_model() const { return imu_model_; }
    const bool &should_quit() const { return should_quit_; }

private:
    // Options and status of backend.
    BackendOptions options_;
    BackendStatus status_;

    // Register some relative components.
    DataManager *data_manager_ = nullptr;
    std::unique_ptr<Imu> imu_model_ = nullptr;

    // Record log.
    BinaryDataLog logger_;
    BackendLog log_package_data_;

    // Signal flags.
    bool should_quit_ = false;  // You can kill all relative threads by checking this flag.
};

}

#endif // end of _VIO_STEREO_ORB_SLAM3_BACKEND_H_

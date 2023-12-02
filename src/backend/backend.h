#ifndef _VIO_STEREO_BASALT_BACKEND_H_
#define _VIO_STEREO_BASALT_BACKEND_H_

#include "datatype_basic.h"
#include "data_manager.h"
#include "binary_data_log.h"
#include "imu.h"
#include "visual_frontend.h"
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
    Vec3 kGravityInWordFrame = Vec3(0.0f, 0.0f, 9.8f);
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
        uint32_t is_imu_static : 1;
        uint32_t is_camera_static : 1;
        uint32_t reserved : 29;
    };
    uint32_t all_bits = 0;
};
static_assert(sizeof(BackendStatus) == sizeof(uint32_t), "Size of BackendStatus should be equal to size of uint32_t.");

/* Class Backend Declaration. */
class Backend final {

public:
    Backend() = default;
    ~Backend() = default;

    // Backend operations.
    bool RunOnce();
    void Reset();
    void ResetToReintialize();

    // Backend initializor.
    bool TryToInitialize();
    bool ConvertNewFramesToCovisibleGraphForInitialization();
    bool EstimatePureRotationOfCameraFrame(const uint32_t ref_frame_id,
                                           const uint32_t cur_frame_id,
                                           const uint32_t min_frame_id,
                                           std::vector<Vec2> &ref_norm_xy,
                                           std::vector<Vec2> &cur_norm_xy,
                                           Quat &q_cr);
    // Estimate gyro bias for initialization.
    bool EstimateGyroBiasAndRotationForInitialization();
    bool EstimateGyroBiasByMethodOneForInitialization();
    bool EstimateGyroBiasByMethodTwoForInitialization();
    bool EstimateGyroBiasByMethodThreeForInitialization();
    // Estimate velocity and gravity for initialization.
    bool EstimateVelocityAndGravityForInitialization(Vec3 &gravity_i0);
    bool SelectTwoFramesWithMaxParallax(CovisibleGraphType *local_map, const FeatureType &feature, int32_t &frame_id_l, int32_t &frame_id_r);
    bool ComputeImuPreintegrationBasedOnFirstFrameForInitialization(std::vector<ImuPreintegrateBlock> &imu_blocks);
    bool ConstructLigtFunction(const std::vector<ImuPreintegrateBlock> &imu_blocks, Mat6 &A, Vec6 &b, float &Q);
    bool RefineGravityForInitialization(const Mat &M, const Vec &m, const float Q, const float gravity_mag, Vec &rhs);
    bool PropagateAllBasedOnFirstCameraFrameForInitializaion(const std::vector<ImuPreintegrateBlock> &imu_blocks, const Vec3 &v_i0i0, const Vec3 &gravity_i0);
    bool TransformAllStatesToWorldFrameForInitialization(const Vec3 &gravity_i0);

    // Backend estimator.
    bool TryToEstimate();

    // Backend data processor.
    void RecomputeImuPreintegration();
    bool TriangulizeAllVisualFeatures();

    // Backend visualizor.
    void ShowFeaturePairsBetweenTwoFrames(const uint32_t ref_frame_id, const uint32_t cur_frame_id, const bool use_rectify = false);
    void ShowLocalMapWithFrames();

    // Reference for member variables.
    BackendOptions &options() { return options_; }
    VisualFrontend *&visual_frontend() { return visual_frontend_; }
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
    VisualFrontend *visual_frontend_ = nullptr;
    DataManager *data_manager_ = nullptr;
    std::unique_ptr<Imu> imu_model_ = nullptr;

    // Record log.
    BinaryDataLog logger_;
    BackendLog log_package_data_;

    // Signal flags.
    bool should_quit_ = false;  // You can kill all relative threads by checking this flag.
};

}

#endif // end of _VIO_STEREO_BASALT_BACKEND_H_

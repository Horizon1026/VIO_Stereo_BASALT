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
using DorF = float;

/* Options for Backend. */
struct BackendOptions {
    bool kEnableRecordBinaryCurveLog = true;
    uint32_t kMethodIndexToEstimateGyroBiasForInitialization = 1;
    Vec3 kGravityInWordFrame = Vec3(0.0f, 0.0f, 9.8f);
    float kMaxValidFeatureDepthInMeter = 50.0f;
    float kMinValidFeatureDepthInMeter = 0.1f;
};

/* Packages of log to be recorded. */
#pragma pack(1)
struct BackendLog {
    uint8_t is_initialized = 0;
};
#pragma pack()

/* Status of Backend. */
enum class BackendMarginalizeType : uint8_t {
    kNotMarginalize = 0,
    kMarginalizeOldestFrame = 1,
    kMarginalizeSubnewFrame = 2,
};

struct BackendStates {
    // Status bits.
    struct {
        uint32_t is_initialized : 1;
        uint32_t reserved : 31;
    };
    BackendMarginalizeType marginalize_type = BackendMarginalizeType::kNotMarginalize;

    // Prior information.
    struct {
        bool is_valid = false;
        TMat<DorF> hessian;
        TVec<DorF> bias;
        TMat<DorF> jacobian;
        TMat<DorF> jacobian_t_inv;
        TVec<DorF> residual;
    } prior;

    // Motion states.
    struct {
        Vec3 p_wi = Vec3::Zero();
        Quat q_wi = Quat::Identity();
        Vec3 v_wi = Vec3::Zero();
        Vec3 ba = Vec3::Zero();
        Vec3 bg = Vec3::Zero();
        float time_stamp_s = 0.0f;
    } motion;
};

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
    bool AddNewestFrameWithBiasIntoLocalMap();
    TMat2<DorF> GetVisualObserveInformationMatrix();

    // Backend maginalizor.
    bool TryToMarginalize();
    bool MarginalizeOldestFrame();
    bool MarginalizeSubnewFrame();

    // Backend data processor.
    void RecomputeImuPreintegration();
    bool TriangulizeAllVisualFeatures();

    // Backend visualizor.
    void ShowFeaturePairsBetweenTwoFrames(const uint32_t ref_frame_id, const uint32_t cur_frame_id, const bool use_rectify = false);
    void ShowLocalMapWithFrames(const int32_t delay_ms);

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
    BackendStates states_;

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

#ifndef _VIO_STEREO_ORB_SLAM3_BACKEND_H_
#define _VIO_STEREO_ORB_SLAM3_BACKEND_H_

#include "data_manager.h"
#include "binary_data_log.h"
#include "general_graph_optimizor.h"

namespace VIO {

using namespace SLAM_UTILITY;
using namespace SLAM_DATA_LOG;
using namespace SLAM_SOLVER;

/* Options for Backend. */
struct BackendOptions {
    bool kEnableRecordBinaryCurveLog = true;
};

/* Packages of log to be recorded. */
#pragma pack(1)
struct BackendLog {
    uint8_t is_initialized = 0;
};
#pragma pack()

/* Class Backend Declaration. */
class Backend final {

public:
    Backend() = default;
    ~Backend() = default;

    // Support for vio initialization.
    bool TryToInitialize();

    // Reference for member variables.
    BackendOptions &options() { return options_; }
    DataManager *&data_manager() { return data_manager_; }

    // Const reference for member variables.
    const BackendOptions &options() const { return options_; }

private:
    // Options for backend.
    BackendOptions options_;

    // Regiser data manager.
    DataManager *data_manager_ = nullptr;

    // Record log.
    BinaryDataLog logger_;
    BackendLog log_package_data_;
};

}

#endif // end of _VIO_STEREO_ORB_SLAM3_BACKEND_H_

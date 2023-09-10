#include "vio.h"
#include "log_report.h"

namespace VIO {

bool Vio::ConfigAllComponents(const VioOptions &options) {

    if (!ConfigComponentOfDataManager(options)) {
        ReportError("[Vio] Failed to config data manager.");
        return false;
    }

    if (!ConfigComponentOfDataLoader(options)) {
        ReportError("[Vio] Failed to config data loader.");
        return false;
    }

    if (!ConfigComponentOfFrontend(options)) {
        ReportError("[Vio] Failed to config visual frontend.");
        return false;
    }

    return true;
}

bool Vio::ConfigComponentOfDataManager(const VioOptions &options) {
    data_manager_ = std::make_unique<DataManager>();
    return true;
}

bool Vio::ConfigComponentOfDataLoader(const VioOptions &options) {
    data_loader_ = std::make_unique<DataLoader>();
    return true;
}

bool Vio::ConfigComponentOfFrontend(const VioOptions &options) {
    frontend_ = std::make_unique<VisualFrontend>(
        options.image_rows, options.image_cols);
    return true;
}

}

#include "data_manager.h"

namespace VIO {

// Transform packed measurements to a new frame.
bool DataManager::ProcessMeasure(std::unique_ptr<PackedMeasurement> &new_packed_measure,
                                 FrontendOutputData &visual_frontend_data) {
    if (new_packed_measure == nullptr) {
        ReportError("[DataManager] Input new_packed_measure is nullptr.");
        return false;
    }

    return true;
}

}

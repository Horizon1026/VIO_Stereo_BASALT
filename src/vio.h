#ifndef _VIO_STEREO_BASALT_H_
#define _VIO_STEREO_BASALT_H_

#include "data_manager.h"
#include "data_loader.h"
#include "visual_frontend.h"

#include "memory"

namespace VIO {

/* Options for vio. */
struct VioOptions {
    uint32_t image_rows = 0;
    uint32_t image_cols = 0;
};

/* Class Vio Declaration. */
class Vio final {

public:
    Vio() = default;
    virtual ~Vio() = default;

    // Config all components of vio.
    bool ConfigAllComponents(const VioOptions &options);
    bool ConfigComponentOfDataManager(const VioOptions &options);
    bool ConfigComponentOfDataLoader(const VioOptions &options);
    bool ConfigComponentOfFrontend(const VioOptions &options);

    // Reference for member variables.
    std::unique_ptr<DataManager> &data_manager() { return data_manager_; }
    std::unique_ptr<DataLoader> &data_loader() { return data_loader_; }
    std::unique_ptr<VisualFrontend> &frontend() { return frontend_; }

    // Const reference for member variables.
    const std::unique_ptr<DataManager> &data_manager() const { return data_manager_; }
    const std::unique_ptr<DataLoader> &data_loader() const { return data_loader_; }
    const std::unique_ptr<VisualFrontend> &frontend() const { return frontend_; }

private:
    // All components.
    std::unique_ptr<DataManager> data_manager_ = nullptr;
    std::unique_ptr<DataLoader> data_loader_ = nullptr;
    std::unique_ptr<VisualFrontend> frontend_ = nullptr;

};

}

#endif

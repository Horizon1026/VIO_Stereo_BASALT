#ifndef _VIO_STEREO_BASALT_H_
#define _VIO_STEREO_BASALT_H_

#include "vio_config.h"

#include "data_manager.h"
#include "data_loader.h"
#include "visual_frontend.h"

#include "memory"

namespace VIO {

/* Class Vio Declaration. */
class Vio final {

public:
    Vio() = default;
    virtual ~Vio() = default;

    // Run once without loop.
    bool RunOnce();
    // Config all components of vio.
    bool ConfigAllComponents(const VioOptions &options);

    // Reference for member variables.
    std::unique_ptr<DataManager> &data_manager() { return data_manager_; }
    std::unique_ptr<DataLoader> &data_loader() { return data_loader_; }
    std::unique_ptr<VisualFrontend> &frontend() { return frontend_; }

    // Const reference for member variables.
    const std::unique_ptr<DataManager> &data_manager() const { return data_manager_; }
    const std::unique_ptr<DataLoader> &data_loader() const { return data_loader_; }
    const std::unique_ptr<VisualFrontend> &frontend() const { return frontend_; }

private:
    // Config all components of vio.
    bool ConfigComponentOfDataManager(const VioOptions &options);
    bool ConfigComponentOfDataLoader(const VioOptions &options);
    bool ConfigComponentOfFrontend(const VioOptions &options);

private:
    // All components.
    std::unique_ptr<DataManager> data_manager_ = nullptr;
    std::unique_ptr<DataLoader> data_loader_ = nullptr;
    std::unique_ptr<VisualFrontend> frontend_ = nullptr;

};

}

#endif

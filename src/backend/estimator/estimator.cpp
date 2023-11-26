#include "backend.h"
#include "log_report.h"
#include "visual_inertial_vetices.h"
#include "visual_inertial_edges.h"

namespace VIO {

bool Backend::TryToEstimate() {
    ReportDebug("[Backend] Backend try to estimate all states.");
    return true;
}

}

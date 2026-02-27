// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/planners/ilqr.hpp"

// iLQR is a template class — implementation is in the header.
// This file provides explicit instantiations for common robot models.

#include "kinetra/robots/differential_drive.hpp"
#include "kinetra/robots/ackermann.hpp"
#include "kinetra/robots/omnidirectional.hpp"

namespace kinetra::planners {

// Explicit instantiations will be added as implementations are completed.
// template class iLQR<DiffDriveSimple>;
// template class iLQR<AckermannSimple>;
// template class iLQR<OmniSimple>;

}  // namespace kinetra::planners

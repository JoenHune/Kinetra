// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/planners/mpcc.hpp"

// MPCC is a template class — implementation is in mpcc_impl.hpp.
// This file provides explicit instantiations for common robot models.

#include "kinetra/robots/differential_drive.hpp"
#include "kinetra/robots/ackermann.hpp"
#include "kinetra/robots/omnidirectional.hpp"

namespace kinetra::planners {

// Explicit instantiations for all LinearizableModel types
template class MPCC<DiffDriveSimple>;
template class MPCC<DiffDriveAccel>;
template class MPCC<DiffDriveJerk>;
template class MPCC<DiffDriveSnap>;

template class MPCC<AckermannSimple>;
template class MPCC<AckermannAccel>;
template class MPCC<AckermannJerk>;

template class MPCC<OmniSimple>;

}  // namespace kinetra::planners

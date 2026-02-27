// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Omnidirectional (holonomic) robot model — can move in any direction.
// State: (x, y, theta) | Control: (vx, vy, omega)

#pragma once

#include <cmath>

#include "kinetra/core/concepts.hpp"
#include "kinetra/core/types.hpp"

namespace kinetra {

/// Simple omnidirectional model with velocity control.
struct OmniSimple {
    using StateType   = Eigen::Matrix<Scalar, 3, 1>;
    using ControlType = Eigen::Matrix<Scalar, 3, 1>;

    static constexpr std::size_t kStateDim   = 3;
    static constexpr std::size_t kControlDim = 3;

    Scalar max_vx{1.0};
    Scalar max_vy{1.0};
    Scalar max_omega{2.0};

    [[nodiscard]] StateType dynamics(const StateType& x, const ControlType& u,
                                      Scalar dt) const noexcept {
        Scalar vx    = clamp(u[0], -max_vx, max_vx);
        Scalar vy    = clamp(u[1], -max_vy, max_vy);
        Scalar omega = clamp(u[2], -max_omega, max_omega);

        // Transform body-frame velocities to world frame
        Scalar cos_th = std::cos(x[2]);
        Scalar sin_th = std::sin(x[2]);

        StateType x_next;
        x_next[0] = x[0] + (vx * cos_th - vy * sin_th) * dt;
        x_next[1] = x[1] + (vx * sin_th + vy * cos_th) * dt;
        x_next[2] = normalizeAngle(x[2] + omega * dt);
        return x_next;
    }

    [[nodiscard]] StateType stateLowerBound() const noexcept {
        return StateType{-constants::kInfinity, -constants::kInfinity, -constants::kPi};
    }
    [[nodiscard]] StateType stateUpperBound() const noexcept {
        return StateType{constants::kInfinity, constants::kInfinity, constants::kPi};
    }
    [[nodiscard]] ControlType controlLowerBound() const noexcept {
        return ControlType{-max_vx, -max_vy, -max_omega};
    }
    [[nodiscard]] ControlType controlUpperBound() const noexcept {
        return ControlType{max_vx, max_vy, max_omega};
    }

    [[nodiscard]] Eigen::Matrix<Scalar, 3, 3> jacobianState(
        const StateType& x, const ControlType& u, Scalar dt) const noexcept {
        Scalar vx = clamp(u[0], -max_vx, max_vx);
        Scalar vy = clamp(u[1], -max_vy, max_vy);
        Scalar cos_th = std::cos(x[2]);
        Scalar sin_th = std::sin(x[2]);

        Eigen::Matrix<Scalar, 3, 3> A = Eigen::Matrix<Scalar, 3, 3>::Identity();
        A(0, 2) = (-vx * sin_th - vy * cos_th) * dt;
        A(1, 2) = (vx * cos_th - vy * sin_th) * dt;
        return A;
    }

    [[nodiscard]] Eigen::Matrix<Scalar, 3, 3> jacobianControl(
        const StateType& x, [[maybe_unused]] const ControlType& u,
        Scalar dt) const noexcept {
        Scalar cos_th = std::cos(x[2]);
        Scalar sin_th = std::sin(x[2]);

        Eigen::Matrix<Scalar, 3, 3> B = Eigen::Matrix<Scalar, 3, 3>::Zero();
        B(0, 0) = cos_th * dt;
        B(0, 1) = -sin_th * dt;
        B(1, 0) = sin_th * dt;
        B(1, 1) = cos_th * dt;
        B(2, 2) = dt;
        return B;
    }
};

static_assert(RobotModel<OmniSimple>);
static_assert(LinearizableModel<OmniSimple>);

}  // namespace kinetra

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Differential drive robot model.
// State: (x, y, theta, v, omega) — position, heading, linear/angular velocity
// Control: (a, alpha) — linear/angular acceleration
// Supports up to jerk-level optimization when extended state is used.

#pragma once

#include <cmath>

#include "kinetra/core/concepts.hpp"
#include "kinetra/core/types.hpp"

namespace kinetra {

// ─── Simple differential drive (velocity control) ────────────────────────────
// State: (x, y, theta) | Control: (v, omega)
struct DiffDriveSimple {
    using StateType   = Eigen::Matrix<Scalar, 3, 1>;
    using ControlType = Eigen::Matrix<Scalar, 2, 1>;

    static constexpr std::size_t kStateDim   = 3;
    static constexpr std::size_t kControlDim = 2;

    Scalar max_v{1.0};        // max linear velocity (m/s)
    Scalar max_omega{1.5};    // max angular velocity (rad/s)
    Scalar wheel_base{0.5};   // distance between wheels (m)

    /// Forward dynamics: x_{k+1} = f(x_k, u_k, dt)
    [[nodiscard]] StateType dynamics(const StateType& x, const ControlType& u,
                                      Scalar dt) const noexcept {
        Scalar v     = clamp(u[0], -max_v, max_v);
        Scalar omega = clamp(u[1], -max_omega, max_omega);
        StateType x_next;
        x_next[0] = x[0] + v * std::cos(x[2]) * dt;
        x_next[1] = x[1] + v * std::sin(x[2]) * dt;
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
        return ControlType{-max_v, -max_omega};
    }
    [[nodiscard]] ControlType controlUpperBound() const noexcept {
        return ControlType{max_v, max_omega};
    }

    /// Linearized Jacobian A = df/dx at (x, u)
    [[nodiscard]] Eigen::Matrix<Scalar, 3, 3> jacobianState(
        const StateType& x, const ControlType& u, Scalar dt) const noexcept {
        Scalar v = clamp(u[0], -max_v, max_v);
        Eigen::Matrix<Scalar, 3, 3> A = Eigen::Matrix<Scalar, 3, 3>::Identity();
        A(0, 2) = -v * std::sin(x[2]) * dt;
        A(1, 2) =  v * std::cos(x[2]) * dt;
        return A;
    }

    /// Linearized Jacobian B = df/du at (x, u)
    [[nodiscard]] Eigen::Matrix<Scalar, 3, 2> jacobianControl(
        const StateType& x, [[maybe_unused]] const ControlType& u,
        Scalar dt) const noexcept {
        Eigen::Matrix<Scalar, 3, 2> B = Eigen::Matrix<Scalar, 3, 2>::Zero();
        B(0, 0) = std::cos(x[2]) * dt;
        B(1, 0) = std::sin(x[2]) * dt;
        B(2, 1) = dt;
        return B;
    }
};

// ─── Extended differential drive (acceleration control) ──────────────────────
// State: (x, y, theta, v, omega) | Control: (a, alpha)
struct DiffDriveAccel {
    using StateType   = Eigen::Matrix<Scalar, 5, 1>;
    using ControlType = Eigen::Matrix<Scalar, 2, 1>;

    static constexpr std::size_t kStateDim   = 5;
    static constexpr std::size_t kControlDim = 2;

    Scalar max_v{1.0};
    Scalar max_omega{1.5};
    Scalar max_a{2.0};         // max linear acceleration (m/s²)
    Scalar max_alpha{3.0};     // max angular acceleration (rad/s²)

    [[nodiscard]] StateType dynamics(const StateType& x, const ControlType& u,
                                      Scalar dt) const noexcept {
        Scalar v     = x[3];
        Scalar omega = x[4];
        Scalar a     = clamp(u[0], -max_a, max_a);
        Scalar alpha = clamp(u[1], -max_alpha, max_alpha);

        StateType x_next;
        x_next[0] = x[0] + v * std::cos(x[2]) * dt;
        x_next[1] = x[1] + v * std::sin(x[2]) * dt;
        x_next[2] = normalizeAngle(x[2] + omega * dt);
        x_next[3] = clamp(v + a * dt, -max_v, max_v);
        x_next[4] = clamp(omega + alpha * dt, -max_omega, max_omega);
        return x_next;
    }

    [[nodiscard]] StateType stateLowerBound() const noexcept {
        StateType lb;
        lb << -constants::kInfinity, -constants::kInfinity, -constants::kPi,
              -max_v, -max_omega;
        return lb;
    }
    [[nodiscard]] StateType stateUpperBound() const noexcept {
        StateType ub;
        ub << constants::kInfinity, constants::kInfinity, constants::kPi,
              max_v, max_omega;
        return ub;
    }
    [[nodiscard]] ControlType controlLowerBound() const noexcept {
        return ControlType{-max_a, -max_alpha};
    }
    [[nodiscard]] ControlType controlUpperBound() const noexcept {
        return ControlType{max_a, max_alpha};
    }

    [[nodiscard]] Eigen::Matrix<Scalar, 5, 5> jacobianState(
        const StateType& x, [[maybe_unused]] const ControlType& u,
        Scalar dt) const noexcept {
        Scalar v = x[3];
        Eigen::Matrix<Scalar, 5, 5> A = Eigen::Matrix<Scalar, 5, 5>::Identity();
        A(0, 2) = -v * std::sin(x[2]) * dt;
        A(0, 3) = std::cos(x[2]) * dt;
        A(1, 2) = v * std::cos(x[2]) * dt;
        A(1, 3) = std::sin(x[2]) * dt;
        A(2, 4) = dt;
        return A;
    }

    [[nodiscard]] Eigen::Matrix<Scalar, 5, 2> jacobianControl(
        [[maybe_unused]] const StateType& x,
        [[maybe_unused]] const ControlType& u,
        Scalar dt) const noexcept {
        Eigen::Matrix<Scalar, 5, 2> B = Eigen::Matrix<Scalar, 5, 2>::Zero();
        B(3, 0) = dt;
        B(4, 1) = dt;
        return B;
    }
};

// Concept verification
static_assert(RobotModel<DiffDriveSimple>);
static_assert(LinearizableModel<DiffDriveSimple>);
static_assert(RobotModel<DiffDriveAccel>);
static_assert(LinearizableModel<DiffDriveAccel>);

}  // namespace kinetra

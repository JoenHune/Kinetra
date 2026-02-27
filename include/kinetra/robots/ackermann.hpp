// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Ackermann steering robot model (car-like vehicle).
// State: (x, y, theta, v, steering_angle) or simpler (x, y, theta)
// The bicycle model approximation is used.

#pragma once

#include <cmath>

#include "kinetra/core/concepts.hpp"
#include "kinetra/core/types.hpp"

namespace kinetra {

// ─── Simple Ackermann (velocity + steering control) ──────────────────────────
// State: (x, y, theta) | Control: (v, steering_angle)
struct AckermannSimple {
    using StateType   = Eigen::Matrix<Scalar, 3, 1>;
    using ControlType = Eigen::Matrix<Scalar, 2, 1>;

    static constexpr std::size_t kStateDim   = 3;
    static constexpr std::size_t kControlDim = 2;

    Scalar wheelbase{2.5};              // L: distance between front and rear axle (m)
    Scalar max_v{5.0};                  // max linear velocity (m/s)
    Scalar max_steering_angle{0.5};     // max steering angle (rad, ~28.6°)

    [[nodiscard]] StateType dynamics(const StateType& x, const ControlType& u,
                                      Scalar dt) const noexcept {
        Scalar v   = clamp(u[0], -max_v, max_v);
        Scalar phi = clamp(u[1], -max_steering_angle, max_steering_angle);
        StateType x_next;
        x_next[0] = x[0] + v * std::cos(x[2]) * dt;
        x_next[1] = x[1] + v * std::sin(x[2]) * dt;
        x_next[2] = normalizeAngle(x[2] + (v / wheelbase) * std::tan(phi) * dt);
        return x_next;
    }

    [[nodiscard]] StateType stateLowerBound() const noexcept {
        return StateType{-constants::kInfinity, -constants::kInfinity, -constants::kPi};
    }
    [[nodiscard]] StateType stateUpperBound() const noexcept {
        return StateType{constants::kInfinity, constants::kInfinity, constants::kPi};
    }
    [[nodiscard]] ControlType controlLowerBound() const noexcept {
        return ControlType{-max_v, -max_steering_angle};
    }
    [[nodiscard]] ControlType controlUpperBound() const noexcept {
        return ControlType{max_v, max_steering_angle};
    }

    [[nodiscard]] Eigen::Matrix<Scalar, 3, 3> jacobianState(
        const StateType& x, const ControlType& u, Scalar dt) const noexcept {
        Scalar v = clamp(u[0], -max_v, max_v);
        Eigen::Matrix<Scalar, 3, 3> A = Eigen::Matrix<Scalar, 3, 3>::Identity();
        A(0, 2) = -v * std::sin(x[2]) * dt;
        A(1, 2) =  v * std::cos(x[2]) * dt;
        return A;
    }

    [[nodiscard]] Eigen::Matrix<Scalar, 3, 2> jacobianControl(
        const StateType& x, const ControlType& u, Scalar dt) const noexcept {
        Scalar phi = clamp(u[1], -max_steering_angle, max_steering_angle);
        Scalar v   = clamp(u[0], -max_v, max_v);
        Eigen::Matrix<Scalar, 3, 2> B = Eigen::Matrix<Scalar, 3, 2>::Zero();
        B(0, 0) = std::cos(x[2]) * dt;
        B(1, 0) = std::sin(x[2]) * dt;
        B(2, 0) = std::tan(phi) / wheelbase * dt;
        Scalar sec_phi = static_cast<Scalar>(1.0) / std::cos(phi);
        B(2, 1) = v / wheelbase * sec_phi * sec_phi * dt;
        return B;
    }

    /// Minimum turning radius for this vehicle
    [[nodiscard]] Scalar minTurningRadius() const noexcept {
        return wheelbase / std::tan(max_steering_angle);
    }
};

// ─── Extended Ackermann (acceleration + steering rate control) ───────────────
// State: (x, y, theta, v, phi) | Control: (a, dphi)
struct AckermannAccel {
    using StateType   = Eigen::Matrix<Scalar, 5, 1>;
    using ControlType = Eigen::Matrix<Scalar, 2, 1>;

    static constexpr std::size_t kStateDim   = 5;
    static constexpr std::size_t kControlDim = 2;

    Scalar wheelbase{2.5};
    Scalar max_v{5.0};
    Scalar max_a{3.0};
    Scalar max_steering_angle{0.5};
    Scalar max_steering_rate{0.8};      // max steering angular velocity (rad/s)

    [[nodiscard]] StateType dynamics(const StateType& x, const ControlType& u,
                                      Scalar dt) const noexcept {
        Scalar v     = x[3];
        Scalar phi   = x[4];
        Scalar a     = clamp(u[0], -max_a, max_a);
        Scalar dphi  = clamp(u[1], -max_steering_rate, max_steering_rate);

        StateType x_next;
        x_next[0] = x[0] + v * std::cos(x[2]) * dt;
        x_next[1] = x[1] + v * std::sin(x[2]) * dt;
        x_next[2] = normalizeAngle(x[2] + (v / wheelbase) * std::tan(phi) * dt);
        x_next[3] = clamp(v + a * dt, -max_v, max_v);
        x_next[4] = clamp(phi + dphi * dt, -max_steering_angle, max_steering_angle);
        return x_next;
    }

    [[nodiscard]] StateType stateLowerBound() const noexcept {
        StateType lb;
        lb << -constants::kInfinity, -constants::kInfinity, -constants::kPi,
              -max_v, -max_steering_angle;
        return lb;
    }
    [[nodiscard]] StateType stateUpperBound() const noexcept {
        StateType ub;
        ub << constants::kInfinity, constants::kInfinity, constants::kPi,
              max_v, max_steering_angle;
        return ub;
    }
    [[nodiscard]] ControlType controlLowerBound() const noexcept {
        return ControlType{-max_a, -max_steering_rate};
    }
    [[nodiscard]] ControlType controlUpperBound() const noexcept {
        return ControlType{max_a, max_steering_rate};
    }

    [[nodiscard]] Eigen::Matrix<Scalar, 5, 5> jacobianState(
        const StateType& x, [[maybe_unused]] const ControlType& u,
        Scalar dt) const noexcept {
        Scalar v = x[3], phi = x[4];
        Scalar tan_phi = std::tan(phi);
        Scalar sec_phi = static_cast<Scalar>(1.0) / std::cos(phi);

        Eigen::Matrix<Scalar, 5, 5> A = Eigen::Matrix<Scalar, 5, 5>::Identity();
        A(0, 2) = -v * std::sin(x[2]) * dt;
        A(0, 3) = std::cos(x[2]) * dt;
        A(1, 2) = v * std::cos(x[2]) * dt;
        A(1, 3) = std::sin(x[2]) * dt;
        A(2, 3) = tan_phi / wheelbase * dt;
        A(2, 4) = v / wheelbase * sec_phi * sec_phi * dt;
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

static_assert(RobotModel<AckermannSimple>);
static_assert(LinearizableModel<AckermannSimple>);
static_assert(RobotModel<AckermannAccel>);
static_assert(LinearizableModel<AckermannAccel>);

// ─── Jerk-level Ackermann (with steering acceleration) ──────────────────────
// State: (x, y, theta, v, phi, a, dphi) | Control: (jerk, ddphi)
// Enables smooth acceleration + steering rate profiles.
struct AckermannJerk {
    using StateType   = Eigen::Matrix<Scalar, 7, 1>;
    using ControlType = Eigen::Matrix<Scalar, 2, 1>;

    static constexpr std::size_t kStateDim   = 7;
    static constexpr std::size_t kControlDim = 2;

    Scalar wheelbase{2.5};
    Scalar max_v{5.0};
    Scalar max_a{3.0};
    Scalar max_steering_angle{0.5};
    Scalar max_steering_rate{0.8};
    Scalar max_jerk{8.0};               // max linear jerk (m/s³)
    Scalar max_steering_accel{2.0};      // max steering angular acceleration (rad/s²)

    [[nodiscard]] StateType dynamics(const StateType& x, const ControlType& u,
                                      Scalar dt) const noexcept {
        Scalar v    = x[3];
        Scalar phi  = x[4];
        Scalar a    = x[5];
        Scalar dphi = x[6];
        Scalar j    = clamp(u[0], -max_jerk, max_jerk);
        Scalar ddp  = clamp(u[1], -max_steering_accel, max_steering_accel);

        StateType x_next;
        x_next[0] = x[0] + v * std::cos(x[2]) * dt;
        x_next[1] = x[1] + v * std::sin(x[2]) * dt;
        x_next[2] = normalizeAngle(x[2] + (v / wheelbase) * std::tan(phi) * dt);
        x_next[3] = clamp(v + a * dt, -max_v, max_v);
        x_next[4] = clamp(phi + dphi * dt, -max_steering_angle, max_steering_angle);
        x_next[5] = clamp(a + j * dt, -max_a, max_a);
        x_next[6] = clamp(dphi + ddp * dt, -max_steering_rate, max_steering_rate);
        return x_next;
    }

    [[nodiscard]] StateType stateLowerBound() const noexcept {
        StateType lb;
        lb << -constants::kInfinity, -constants::kInfinity, -constants::kPi,
              -max_v, -max_steering_angle, -max_a, -max_steering_rate;
        return lb;
    }
    [[nodiscard]] StateType stateUpperBound() const noexcept {
        StateType ub;
        ub << constants::kInfinity, constants::kInfinity, constants::kPi,
              max_v, max_steering_angle, max_a, max_steering_rate;
        return ub;
    }
    [[nodiscard]] ControlType controlLowerBound() const noexcept {
        return ControlType{-max_jerk, -max_steering_accel};
    }
    [[nodiscard]] ControlType controlUpperBound() const noexcept {
        return ControlType{max_jerk, max_steering_accel};
    }

    [[nodiscard]] Eigen::Matrix<Scalar, 7, 7> jacobianState(
        const StateType& x, [[maybe_unused]] const ControlType& u,
        Scalar dt) const noexcept {
        Scalar v   = x[3];
        Scalar phi = x[4];
        Scalar tan_phi = std::tan(phi);
        Scalar sec_phi = static_cast<Scalar>(1.0) / std::cos(phi);

        Eigen::Matrix<Scalar, 7, 7> A = Eigen::Matrix<Scalar, 7, 7>::Identity();
        A(0, 2) = -v * std::sin(x[2]) * dt;
        A(0, 3) = std::cos(x[2]) * dt;
        A(1, 2) = v * std::cos(x[2]) * dt;
        A(1, 3) = std::sin(x[2]) * dt;
        A(2, 3) = tan_phi / wheelbase * dt;
        A(2, 4) = v / wheelbase * sec_phi * sec_phi * dt;
        A(3, 5) = dt;
        A(4, 6) = dt;
        return A;
    }

    [[nodiscard]] Eigen::Matrix<Scalar, 7, 2> jacobianControl(
        [[maybe_unused]] const StateType& x,
        [[maybe_unused]] const ControlType& u,
        Scalar dt) const noexcept {
        Eigen::Matrix<Scalar, 7, 2> B = Eigen::Matrix<Scalar, 7, 2>::Zero();
        B(5, 0) = dt;
        B(6, 1) = dt;
        return B;
    }
};

static_assert(RobotModel<AckermannJerk>);
static_assert(LinearizableModel<AckermannJerk>);

}  // namespace kinetra

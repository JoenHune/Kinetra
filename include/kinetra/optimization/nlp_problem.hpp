// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Composable NLP formulation inspired by ifopt.
// Users define independent VariableSet, ConstraintSet, and CostTerm objects,
// which are automatically composed into a full optimization problem.

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "kinetra/core/types.hpp"

namespace kinetra::optimization {

// ═════════════════════════════════════════════════════════════════════════════
// VariableSet — a named block of decision variables
// ═════════════════════════════════════════════════════════════════════════════
class VariableSet {
public:
    using Ptr = std::shared_ptr<VariableSet>;

    VariableSet(int n, std::string name)
        : n_(n), name_(std::move(name)), values_(VecX::Zero(n)) {}

    virtual ~VariableSet() = default;

    [[nodiscard]] int size() const noexcept { return n_; }
    [[nodiscard]] const std::string& name() const noexcept { return name_; }

    [[nodiscard]] VecX values() const { return values_; }
    void setValues(const VecX& x) { values_ = x; }

    /// Get bounds for each variable (override for custom bounds)
    [[nodiscard]] virtual VecX lowerBounds() const {
        return VecX::Constant(n_, -constants::kInfinity);
    }
    [[nodiscard]] virtual VecX upperBounds() const {
        return VecX::Constant(n_, constants::kInfinity);
    }

    // Index management (set by NLPProblem during composition)
    void setStartIndex(int idx) noexcept { start_idx_ = idx; }
    [[nodiscard]] int startIndex() const noexcept { return start_idx_; }

private:
    int n_;
    std::string name_;
    VecX values_;
    int start_idx_{0};
};

// ═════════════════════════════════════════════════════════════════════════════
// ConstraintSet — a named block of constraints: lb <= g(x) <= ub
// ═════════════════════════════════════════════════════════════════════════════
class ConstraintSet {
public:
    using Ptr = std::shared_ptr<ConstraintSet>;

    ConstraintSet(int m, std::string name)
        : m_(m), name_(std::move(name)) {}

    virtual ~ConstraintSet() = default;

    [[nodiscard]] int size() const noexcept { return m_; }
    [[nodiscard]] const std::string& name() const noexcept { return name_; }

    /// Evaluate constraint values g(x)
    [[nodiscard]] virtual VecX evaluate() const = 0;

    /// Constraint bounds
    [[nodiscard]] virtual VecX lowerBound() const = 0;
    [[nodiscard]] virtual VecX upperBound() const = 0;

    /// Fill the Jacobian block for a specific variable set.
    /// Row start = constraint start index, Col start = variable start index.
    virtual void fillJacobianBlock(const std::string& var_name, MatX& jac) const = 0;

    /// Link to the variable sets this constraint depends on
    void linkVariables(std::vector<VariableSet::Ptr> vars) {
        linked_vars_ = std::move(vars);
    }

    [[nodiscard]] const std::vector<VariableSet::Ptr>& linkedVariables() const {
        return linked_vars_;
    }

protected:
    /// Helper: get values of a linked variable set by name
    [[nodiscard]] VecX getVariableValues(const std::string& name) const {
        for (const auto& v : linked_vars_) {
            if (v->name() == name) return v->values();
        }
        return VecX{};
    }

private:
    int m_;
    std::string name_;
    std::vector<VariableSet::Ptr> linked_vars_;
};

// ═════════════════════════════════════════════════════════════════════════════
// CostTerm — a scalar cost function
// ═════════════════════════════════════════════════════════════════════════════
class CostTerm {
public:
    using Ptr = std::shared_ptr<CostTerm>;

    explicit CostTerm(std::string name) : name_(std::move(name)) {}
    virtual ~CostTerm() = default;

    [[nodiscard]] const std::string& name() const noexcept { return name_; }

    /// Evaluate cost value
    [[nodiscard]] virtual Scalar evaluate() const = 0;

    /// Fill gradient block for a specific variable set
    virtual void fillGradientBlock(const std::string& var_name, VecX& grad) const = 0;

    void linkVariables(std::vector<VariableSet::Ptr> vars) {
        linked_vars_ = std::move(vars);
    }

    [[nodiscard]] const std::vector<VariableSet::Ptr>& linkedVariables() const {
        return linked_vars_;
    }

protected:
    [[nodiscard]] VecX getVariableValues(const std::string& name) const {
        for (const auto& v : linked_vars_) {
            if (v->name() == name) return v->values();
        }
        return VecX{};
    }

private:
    std::string name_;
    std::vector<VariableSet::Ptr> linked_vars_;
};

// ═════════════════════════════════════════════════════════════════════════════
// NLPProblem — the composed optimization problem
// ═════════════════════════════════════════════════════════════════════════════
class NLPProblem {
public:
    void addVariableSet(VariableSet::Ptr vars) {
        vars->setStartIndex(total_vars_);
        total_vars_ += vars->size();
        variable_sets_.push_back(std::move(vars));
    }

    void addConstraintSet(ConstraintSet::Ptr cons) {
        total_constraints_ += cons->size();
        constraint_sets_.push_back(std::move(cons));
    }

    void addCostSet(CostTerm::Ptr cost) {
        cost_terms_.push_back(std::move(cost));
    }

    // ── Problem assembly ─────────────────────────────────────────────────────

    /// Total number of decision variables
    [[nodiscard]] int numVariables() const noexcept { return total_vars_; }

    /// Total number of constraints
    [[nodiscard]] int numConstraints() const noexcept { return total_constraints_; }

    /// Get full variable vector (concatenated from all sets)
    [[nodiscard]] VecX variableValues() const;

    /// Set full variable vector (distributed to all sets)
    void setVariableValues(const VecX& x);

    /// Variable bounds
    [[nodiscard]] VecX variableLowerBounds() const;
    [[nodiscard]] VecX variableUpperBounds() const;

    /// Evaluate all constraints (concatenated)
    [[nodiscard]] VecX constraintValues() const;

    /// Constraint bounds
    [[nodiscard]] VecX constraintLowerBounds() const;
    [[nodiscard]] VecX constraintUpperBounds() const;

    /// Full Jacobian of constraints w.r.t. variables
    [[nodiscard]] MatX constraintJacobian() const;

    /// Total cost (sum of all cost terms)
    [[nodiscard]] Scalar totalCost() const;

    /// Cost gradient w.r.t. all variables
    [[nodiscard]] VecX costGradient() const;

    // ── Accessors ────────────────────────────────────────────────────────────
    [[nodiscard]] const auto& variableSets() const { return variable_sets_; }
    [[nodiscard]] const auto& constraintSets() const { return constraint_sets_; }
    [[nodiscard]] const auto& costTerms() const { return cost_terms_; }

private:
    std::vector<VariableSet::Ptr> variable_sets_;
    std::vector<ConstraintSet::Ptr> constraint_sets_;
    std::vector<CostTerm::Ptr> cost_terms_;
    int total_vars_{0};
    int total_constraints_{0};
};

}  // namespace kinetra::optimization

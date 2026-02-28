// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/optimization/nlp_problem.hpp"

namespace kinetra::optimization {

VecX NLPProblem::variableValues() const {
    VecX x(total_vars_);
    for (const auto& vs : variable_sets_) {
        x.segment(vs->startIndex(), vs->size()) = vs->values();
    }
    return x;
}

void NLPProblem::setVariableValues(const VecX& x) {
    for (auto& vs : variable_sets_) {
        vs->setValues(x.segment(vs->startIndex(), vs->size()));
    }
}

VecX NLPProblem::variableLowerBounds() const {
    VecX lb(total_vars_);
    for (const auto& vs : variable_sets_) {
        lb.segment(vs->startIndex(), vs->size()) = vs->lowerBounds();
    }
    return lb;
}

VecX NLPProblem::variableUpperBounds() const {
    VecX ub(total_vars_);
    for (const auto& vs : variable_sets_) {
        ub.segment(vs->startIndex(), vs->size()) = vs->upperBounds();
    }
    return ub;
}

VecX NLPProblem::constraintValues() const {
    VecX g(total_constraints_);
    int row = 0;
    for (const auto& cs : constraint_sets_) {
        g.segment(row, cs->size()) = cs->evaluate();
        row += cs->size();
    }
    return g;
}

VecX NLPProblem::constraintLowerBounds() const {
    VecX lb(total_constraints_);
    int row = 0;
    for (const auto& cs : constraint_sets_) {
        lb.segment(row, cs->size()) = cs->lowerBound();
        row += cs->size();
    }
    return lb;
}

VecX NLPProblem::constraintUpperBounds() const {
    VecX ub(total_constraints_);
    int row = 0;
    for (const auto& cs : constraint_sets_) {
        ub.segment(row, cs->size()) = cs->upperBound();
        row += cs->size();
    }
    return ub;
}

MatX NLPProblem::constraintJacobian() const {
    MatX jac = MatX::Zero(total_constraints_, total_vars_);
    int row = 0;
    for (const auto& cs : constraint_sets_) {
        for (const auto& vs : cs->linkedVariables()) {
            // Create a sub-block for this constraint-variable pair
            MatX block = MatX::Zero(cs->size(), vs->size());
            cs->fillJacobianBlock(vs->name(), block);
            jac.block(row, vs->startIndex(), cs->size(), vs->size()) = block;
        }
        row += cs->size();
    }
    return jac;
}

SpMatX NLPProblem::constraintJacobianSparse() const {
    // Assemble sparse Jacobian via triplet list — avoids allocating dense zeros.
    // Each constraint's fillJacobianBlock still produces a dense sub-block,
    // but we only extract the nonzero entries into the sparse matrix.
    std::vector<Triplet> triplets;
    triplets.reserve(total_constraints_ * 8);  // conservative estimate

    int row = 0;
    for (const auto& cs : constraint_sets_) {
        for (const auto& vs : cs->linkedVariables()) {
            MatX block = MatX::Zero(cs->size(), vs->size());
            cs->fillJacobianBlock(vs->name(), block);
            const int col0 = vs->startIndex();
            for (int i = 0; i < cs->size(); ++i) {
                for (int j = 0; j < vs->size(); ++j) {
                    if (block(i, j) != Scalar(0)) {
                        triplets.emplace_back(row + i, col0 + j, block(i, j));
                    }
                }
            }
        }
        row += cs->size();
    }

    SpMatX jac(total_constraints_, total_vars_);
    jac.setFromTriplets(triplets.begin(), triplets.end());
    return jac;
}

Scalar NLPProblem::totalCost() const {
    Scalar total = 0;
    for (const auto& ct : cost_terms_) {
        total += ct->evaluate();
    }
    return total;
}

VecX NLPProblem::costGradient() const {
    VecX grad = VecX::Zero(total_vars_);
    for (const auto& ct : cost_terms_) {
        for (const auto& vs : ct->linkedVariables()) {
            VecX block = VecX::Zero(vs->size());
            ct->fillGradientBlock(vs->name(), block);
            grad.segment(vs->startIndex(), vs->size()) += block;
        }
    }
    return grad;
}

}  // namespace kinetra::optimization

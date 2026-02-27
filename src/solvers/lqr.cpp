// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/solvers/lqr.hpp"

#include <Eigen/Dense>
#include <cassert>

namespace kinetra::solvers {

LQRGains solveLQR(
    const std::vector<MatX>& A,
    const std::vector<MatX>& B,
    const std::vector<MatX>& Q,
    const std::vector<VecX>& q,
    const std::vector<MatX>& R,
    const std::vector<VecX>& r,
    Scalar reg) {

    int T = static_cast<int>(A.size());
    assert(T > 0);
    assert(B.size() == static_cast<std::size_t>(T));
    assert(Q.size() == static_cast<std::size_t>(T + 1));
    assert(q.size() == static_cast<std::size_t>(T + 1));
    assert(R.size() == static_cast<std::size_t>(T));
    assert(r.size() == static_cast<std::size_t>(T));

    int nx = static_cast<int>(A[0].rows());
    int nu = static_cast<int>(B[0].cols());

    LQRGains gains;
    gains.K.resize(static_cast<std::size_t>(T));
    gains.k.resize(static_cast<std::size_t>(T));
    gains.V.resize(static_cast<std::size_t>(T + 1));
    gains.v.resize(static_cast<std::size_t>(T + 1));

    // Terminal condition
    gains.V[static_cast<std::size_t>(T)] = Q[static_cast<std::size_t>(T)];
    gains.v[static_cast<std::size_t>(T)] = q[static_cast<std::size_t>(T)];

    gains.expectedImprovement = 0;

    // Backward pass (Riccati recursion)
    for (int t = T - 1; t >= 0; --t) {
        auto ti = static_cast<std::size_t>(t);
        auto ti1 = static_cast<std::size_t>(t + 1);

        const MatX& At = A[ti];
        const MatX& Bt = B[ti];
        const MatX& Qt = Q[ti];
        const VecX& qt = q[ti];
        const MatX& Rt = R[ti];
        const VecX& rt = r[ti];
        const MatX& Vt1 = gains.V[ti1];
        const VecX& vt1 = gains.v[ti1];

        // Q-function expansion
        MatX Qxx = Qt + At.transpose() * Vt1 * At;
        MatX Qux = Bt.transpose() * Vt1 * At;
        MatX Quu = Rt + Bt.transpose() * Vt1 * Bt;
        VecX Qx  = qt + At.transpose() * vt1;
        VecX Qu  = rt + Bt.transpose() * vt1;

        // Regularization (Levenberg-Marquardt)
        Quu += reg * MatX::Identity(nu, nu);

        // Solve for gains: K = -Quu^{-1} Qux, k = -Quu^{-1} Qu
        Eigen::LDLT<MatX> Quu_ldlt(Quu);
        gains.K[ti] = -Quu_ldlt.solve(Qux);
        gains.k[ti] = -Quu_ldlt.solve(Qu);

        // Value function update
        gains.V[ti] = Qxx + Qux.transpose() * gains.K[ti];
        gains.v[ti] = Qx + Qux.transpose() * gains.k[ti];

        // Expected improvement
        gains.expectedImprovement += gains.k[ti].dot(Qu);
    }

    return gains;
}

}  // namespace kinetra::solvers

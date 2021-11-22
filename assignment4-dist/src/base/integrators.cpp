#include "integrators.hpp"
#include "particle_systems.hpp"
#include "utility.hpp"

auto takeStep(const State &state, const State &f, float step) {
    auto next_state = State(state.size());
    int idx = 0;
    for (auto &particle_state : next_state) {
        particle_state = state[idx] + f[idx] * step;
        idx++;
    }
    return next_state;
}

auto eulerHelper(ParticleSystem &ps, float step) {
    auto const &current_state = ps.state();
    auto f = ps.evalF(current_state);
    auto next_state = State(current_state.size());
    int idx = 0;
    for (auto &particle_state : next_state) {
        particle_state = current_state[idx] + f[idx] * step;
        idx++;
    }
    return next_state;
};

void eulerStep(ParticleSystem &ps, float step) {
    // YOUR CODE HERE (R1)
    // Implement an Euler integrator.
    ps.set_state(eulerHelper(ps, step));
};

void trapezoidStep(ParticleSystem &ps, float step) {
    // YOUR CODE HERE (R3)
    // Implement a trapezoid integrator.
    auto const &current_state = ps.state();
    auto const f0 = ps.evalF(current_state);
    auto f1 = ps.evalF(eulerHelper(ps, step));
    auto next_state = State(f0.size());
    int idx = 0;
    for (auto &particle_state : next_state) {
        particle_state = current_state[idx] + step * 0.5f * (f0[idx] + f1[idx]);
        idx++;
    }
    ps.set_state(next_state);
}

void midpointStep(ParticleSystem &ps, float step) {
    const auto &x0 = ps.state();
    auto n = x0.size();
    auto f0 = ps.evalF(x0);
    auto xm = State(n), x1 = State(n);
    for (auto i = 0u; i < n; ++i) {
        xm[i] = x0[i] + (0.5f * step) * f0[i];
    }
    auto fm = ps.evalF(xm);
    for (auto i = 0u; i < n; ++i) {
        x1[i] = x0[i] + step * fm[i];
    }
    ps.set_state(x1);
}

void rk4Step(ParticleSystem &ps, float step) {
    // EXTRA: Implement the RK4 Runge-Kutta integrator.
    const auto &x0 = ps.state();
    auto k1 = ps.evalF(x0);
    auto n = x0.size();

    auto state_for_k2 = takeStep(x0, k1, step / 2);
    auto k2 = ps.evalF(state_for_k2);
    auto state_for_k3 = takeStep(x0, k2, step / 2);
    auto k3 = ps.evalF(state_for_k3);
    auto state_for_k4 = takeStep(x0, k3, step);
    auto k4 = ps.evalF(state_for_k4);

    auto next_state = State(n);
    int idx = 0;
    for (auto &particle_state : next_state) {
        particle_state = x0[idx] + step * 0.1667f * (k1[idx] + 2.0f * k2[idx] + 2.0f * k3[idx] + k4[idx]);
        idx++;
    }
    ps.set_state(next_state);
}

#ifdef EIGEN_SPARSECORE_MODULE_H

void implicit_euler_step(ParticleSystem &ps, float step, SparseMatrix &J, SparseLU &solver, bool initial) {
    // EXTRA: Implement the implicit Euler integrator. (Note that the related formula on page 134 on the lecture slides is missing a 'h'; the formula should be (I-h*Jf(Yi))DY=-F(Yi))
}

void implicit_midpoint_step(ParticleSystem &ps, float step, SparseMatrix &J, SparseLU &solver, bool initial) {
    // EXTRA: Implement the implicit midpoint integrator.
}

void crank_nicolson_step(ParticleSystem &ps, float step, SparseMatrix &J, SparseLU &solver, bool initial) {
    // EXTRA: Implement the crank-nicolson integrator.
}
#endif

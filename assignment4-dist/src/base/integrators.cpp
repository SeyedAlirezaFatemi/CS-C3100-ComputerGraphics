
#include "utility.hpp"
#include "particle_systems.hpp"
#include "integrators.hpp"

void eulerStep(ParticleSystem& ps, float step) {
	// YOUR CODE HERE (R1)
	// Implement an Euler integrator.
};

void trapezoidStep(ParticleSystem& ps, float step) {
	// YOUR CODE HERE (R3)
	// Implement a trapezoid integrator.
}

void midpointStep(ParticleSystem& ps, float step) {
	const auto& x0 = ps.state();
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

void rk4Step(ParticleSystem& ps, float step) {
	// EXTRA: Implement the RK4 Runge-Kutta integrator.
}

#ifdef EIGEN_SPARSECORE_MODULE_H

void implicit_euler_step(ParticleSystem& ps, float step, SparseMatrix& J, SparseLU& solver, bool initial) {
	// EXTRA: Implement the implicit Euler integrator. (Note that the related formula on page 134 on the lecture slides is missing a 'h'; the formula should be (I-h*Jf(Yi))DY=-F(Yi))
}

void implicit_midpoint_step(ParticleSystem& ps, float step, SparseMatrix& J, SparseLU& solver, bool initial) {
	// EXTRA: Implement the implicit midpoint integrator.
}

void crank_nicolson_step(ParticleSystem & ps, float step, SparseMatrix & J, SparseLU & solver, bool initial) {
		// EXTRA: Implement the crank-nicolson integrator.
}
#endif

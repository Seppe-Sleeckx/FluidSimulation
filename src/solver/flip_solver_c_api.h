#include "flip_solver.h"

extern "C" {
	typedef void* FLIPSolverHandle; //opaque handle

	FLIPSolverHandle Create(const Solver_Utils::SolverConfig* config);
	void Destroy(FLIPSolverHandle solver);

	void Initialize(FLIPSolverHandle solver);
	void Simulate(FLIPSolverHandle solver, float dt);

	int   GetParticleCount(FLIPSolverHandle solver);
	const float* GetParticlePositions(FLIPSolverHandle solver);
}
#include "flip_solver_c_api.h"

FLIPSolverHandle Create(const Solver_Utils::SolverConfig* config)
{
	if (!config)
		return nullptr;
	return new FLIPSolver(*config);
}

void Destroy(FLIPSolverHandle solver)
{
	if (!solver)
		return;
	delete static_cast<FLIPSolver*>(solver);
}

void Initialize(FLIPSolverHandle solver)
{
	if (!solver)
		return;
	static_cast<FLIPSolver*>(solver)->Initialize();
}

void Simulate(FLIPSolverHandle solver, float dt)
{
	if (!solver)
		return;
	static_cast<FLIPSolver*>(solver)->Simulate(dt);
}

int GetParticleCount(FLIPSolverHandle solver)
{
	if (!solver)
		return 0;
	return static_cast<FLIPSolver*>(solver)->GetParticleCount();
}

const float* GetParticlePositions(FLIPSolverHandle solver)
{
	if (!solver)
		return nullptr;
	return static_cast<FLIPSolver*>(solver)->GetParticlePositions().data();
}

const int GetGridXDimension(FLIPSolverHandle solver)
{
	if (!solver)
		return 0;
	return static_cast<FLIPSolver*>(solver)->GetGridXDimension();
}

const int GetGridYDimension(FLIPSolverHandle solver)
{
	if (!solver)
		return 0;
	return static_cast<FLIPSolver*>(solver)->GetGridYDimension();
}

const int GetGridZDimension(FLIPSolverHandle solver)
{
	if (!solver)
		return 0;
	return static_cast<FLIPSolver*>(solver)->GetGridZDimension();
}

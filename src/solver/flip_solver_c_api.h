#include "flip_solver.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define WASM EMSCRIPTEN_KEEPALIVE //only add this when building to WASM with emscriptem
#else
#define WASM
#endif

extern "C" {
	typedef void* FLIPSolverHandle; //opaque handle

	WASM FLIPSolverHandle Create(const Solver_Utils::SolverConfig* config);
	WASM void Destroy(FLIPSolverHandle solver);

	WASM void Initialize(FLIPSolverHandle solver);
	WASM void Simulate(FLIPSolverHandle solver, float dt);

	WASM int   GetParticleCount(FLIPSolverHandle solver);
	WASM const float* GetParticlePositions(FLIPSolverHandle solver);

	WASM const int GetGridXDimension(FLIPSolverHandle solver);
	WASM const int GetGridYDimension(FLIPSolverHandle solver);
	WASM const int GetGridZDimension(FLIPSolverHandle solver);
}
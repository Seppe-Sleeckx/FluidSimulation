#include "flip_solver.h"
#include <limits>
#include <algorithm>
using namespace Solver_Utils;

//DEBUG
#include <iostream>
#include <fstream>

FLIPSolver::FLIPSolver(const SolverConfig& config) :
	m_cellNumX{ config.gridX },
	m_cellNumY{ config.gridY },
	m_cellNumZ{ config.gridZ },
	m_numParticles{ config.numParticles },
	m_particleRadius{ config.particleRadius },

	m_gridVU{ m_cellNumX + 1, m_cellNumY,     m_cellNumZ },
	m_gridVV{ m_cellNumX,     m_cellNumY + 1, m_cellNumZ },
	m_gridVW{ m_cellNumX,     m_cellNumY,     m_cellNumZ + 1 },

	m_gridVUBefore{ m_cellNumX + 1, m_cellNumY,     m_cellNumZ },
	m_gridVVBefore{ m_cellNumX,     m_cellNumY + 1, m_cellNumZ },
	m_gridVWBefore{ m_cellNumX,     m_cellNumY,     m_cellNumZ + 1 },

	m_gridVUAfter{ m_cellNumX + 1, m_cellNumY,     m_cellNumZ },
	m_gridVVAfter{ m_cellNumX,     m_cellNumY + 1, m_cellNumZ },
	m_gridVWAfter{ m_cellNumX,     m_cellNumY,     m_cellNumZ + 1 },

	m_gridWeightU{ m_cellNumX + 1, m_cellNumY,     m_cellNumZ },
	m_gridWeightV{ m_cellNumX,     m_cellNumY + 1, m_cellNumZ },
	m_gridWeightW{ m_cellNumX,     m_cellNumY,     m_cellNumZ + 1 },

	m_gridPressure{ m_cellNumX, m_cellNumY, m_cellNumZ },
	m_gridDivergence{ m_cellNumX, m_cellNumY, m_cellNumZ },
	m_gridDensity{ m_cellNumX, m_cellNumY, m_cellNumZ },
	m_gridCellType{ m_cellNumX, m_cellNumY, m_cellNumZ },
	m_particlePos(m_numParticles, 3),
	m_particleV(m_numParticles, 3),
	m_alphaPIC{ config.alphaPic },
	m_useAdaptiveMixing{ config.useAdaptiveMixing }
{
	m_particleV.setZero();
}

void FLIPSolver::Initialize()
{
	//1. Spread particles over grid
	const float diameter = 2.0f * m_particleRadius;
	float maxParticlesX = (m_cellNumX * m_CellSize) / diameter;
	float maxParticlesY = (m_cellNumY * m_CellSize) / diameter;
	float maxParticlesZ = (m_cellNumZ * m_CellSize) / diameter;

	int pIdx = 0;
	for (int ix = 0; ix < maxParticlesX; ++ix)
	{
		if (pIdx >= m_numParticles)
			break;
		float x = ix * diameter;

		for (int iy = 0; iy < maxParticlesY; ++iy)
		{
			if (pIdx >= m_numParticles)
				break;
			float y = (m_cellNumY * m_CellSize) - iy * diameter; //top down

			for (int iz = 0; iz < maxParticlesZ; ++iz)
			{
				if (pIdx >= m_numParticles)
					break;
				float z = iz * diameter;
				m_particlePos.row(pIdx) = Eigen::Vector3f(x, y, z);
				++pIdx;
			}
		}
	}

	//Initialize grid for first update
	m_gridCellType.setConstant(CellType::Air); //start all cells at air, maybe change later to be configurable
	ClearGrid();
	MarkFluidCells(); //needs to be done BEFORE UpdatingParticleDensity

	

	//2. Compute particle rest density, make sure particles are at rest positions
	UpdateParticleDensity(); //very important, m_gridDensity will be zero otherwise
	m_particleRestDensity = 0.0f;
	int numFluidCells = 0;

	for (int x = 0; x < m_cellNumX; x++)
		for (int y = 0; y < m_cellNumY; y++)
			for (int z = 0; z < m_cellNumZ; z++)
			{
				if (m_gridCellType(x, y, z) == CellType::Fluid)
				{
					m_particleRestDensity += m_gridDensity(x, y, z);
					numFluidCells++;
				}
			}

	if (numFluidCells > 0)
		m_particleRestDensity /= float(numFluidCells);

	std::cout << "particle rest density: " << m_particleRestDensity << std::endl;
}

void FLIPSolver::Simulate(float dt)
{
	//Advection step
	IntegrateParticles(dt);
	PushParticlesApart(6);
	ResolveParticleCollisions(); //make sure particles stay inside the domain
	

	//clear grid
	ClearGrid();

	//mark cells with particles inside as fluid cells, leave solid cells untouched
	MarkFluidCells();

	//Particle velocity to grid (P2G)
	TransferP2G();

	//Save grid velocities (Only necessary for FLIP)
	SaveGridBefore();

	//Apply external forces (gravity)
	ApplyGravity(dt);


	//Solve for incompressibility (only applicable for fluids, not gasses)
	UpdateParticleDensity();
	SolveIncompressibility(dt, 20);

	//store grid velocity after projection (Only necessary for FLIP)
	SaveGridAfter();

	//Grid to particle (G2P)
	TransferG2P(dt);
}

void FLIPSolver::ClearGrid()
{
	m_gridVU.setZero();
	m_gridVV.setZero();
	m_gridVW.setZero();

	m_gridVUBefore.setZero();
	m_gridVVBefore.setZero();
	m_gridVWBefore.setZero();

	m_gridVUAfter.setZero();
	m_gridVVAfter.setZero();
	m_gridVWAfter.setZero();

	m_gridWeightU.setZero();
	m_gridWeightV.setZero();
	m_gridWeightW.setZero();

	m_gridPressure.setZero();
	m_gridDivergence.setZero();
}

void FLIPSolver::MarkFluidCells() {
	Solver_Utils::ParallelFor(0, m_cellNumX, [&](int x)
	{
		for (int y = 0; y < m_cellNumY; ++y) {
			for (int z = 0; z < m_cellNumZ; ++z)
			{
				if (m_gridCellType(x, y, z) != CellType::Solid)
					m_gridCellType(x, y, z) = CellType::Air; //set all non solid cells to air
			}
		}
	});

	for (int p = 0; p < m_numParticles; ++p)
	{
		const Eigen::Vector3f& pos = m_particlePos.row(p);

		int ix, iy, iz;
		Eigen::Vector3f f;
		ComputeCellCoordinates(pos, ix, iy, iz, f);

		if (m_gridCellType(ix, iy, iz) != CellType::Solid)
			m_gridCellType(ix, iy, iz) = CellType::Fluid; //mark all cells with particles inside as fluid (except for solid cells)
	}
}

void FLIPSolver::IntegrateParticles(float dt)
{
	Solver_Utils::ParallelFor(0, m_numParticles, [&](int i)
	{
		m_particlePos.row(i) += dt * m_particleV.row(i); //update positions
	});
}

void FLIPSolver::TransferP2G()
{
	for (int p = 0; p < m_numParticles; ++p)
	{
		const Eigen::Vector3f pos = m_particlePos.row(p);
		const Eigen::Vector3f vel = m_particleV.row(p);

		{//X faces
			Eigen::Vector3f uPos = pos;

			//base cell idx + fractional offset (f) inside cell
			int ix, iy, iz;
			Eigen::Vector3f f;
			ComputeCellCoordinates(uPos, ix, iy, iz, f);

			//Calulcate weights
			Solver_Utils::Weight3D W;
			Solver_Utils::ComputeBSplineWeights(f, W);

			//Scatter particle velocity to grid (27 neighbours)
			for (int n = 0; n < 27; ++n)
			{
				const int gridX = ix + W.offsetsX[n];
				const int gridY = iy + W.offsetsY[n];
				const int gridZ = iz + W.offsetsZ[n];

				//bounds check
				if (gridX < 0 || gridX >= m_cellNumX + 1 ||
					gridY < 0 || gridY >= m_cellNumY ||
					gridZ < 0 || gridZ >= m_cellNumZ)
					continue;


				const float w = W.w[n];

				//accumulate weight
				m_gridWeightU(gridX, gridY, gridZ) += w;

				//accumulate weighted velocity
				m_gridVU(gridX, gridY, gridZ) += w * vel.x();
			}
		}

		{//Y faces
			Eigen::Vector3f vPos = pos;

			//base cell idx + fractional offset (f) inside cell
			int ix, iy, iz;
			Eigen::Vector3f f;
			ComputeCellCoordinates(vPos, ix, iy, iz, f);

			//Calulcate weights
			Solver_Utils::Weight3D W;
			Solver_Utils::ComputeBSplineWeights(f, W);

			//Scatter particle velocity to grid (27 neighbours)
			for (int n = 0; n < 27; ++n)
			{
				const int gridX = ix + W.offsetsX[n];
				const int gridY = iy + W.offsetsY[n];
				const int gridZ = iz + W.offsetsZ[n];

				//bounds check
				if (gridX < 0 || gridX >= m_cellNumX ||
					gridY < 0 || gridY >= m_cellNumY + 1 ||
					gridZ < 0 || gridZ >= m_cellNumZ)
					continue;


				const float w = W.w[n];

				//accumulate weight
				m_gridWeightV(gridX, gridY, gridZ) += w;

				//accumulate weighted velocity
				m_gridVV(gridX, gridY, gridZ) += w * vel.y();
			}
		}

		{//Z faces
			Eigen::Vector3f uPos = pos;

			//base cell idx + fractional offset (f) inside cell
			int ix, iy, iz;
			Eigen::Vector3f f;
			ComputeCellCoordinates(uPos, ix, iy, iz, f);

			//Calulcate weights
			Solver_Utils::Weight3D W;
			Solver_Utils::ComputeBSplineWeights(f, W);

			//Scatter particle velocity to grid (27 neighbours)
			for (int n = 0; n < 27; ++n)
			{
				const int gridX = ix + W.offsetsX[n];
				const int gridY = iy + W.offsetsY[n];
				const int gridZ = iz + W.offsetsZ[n];

				//bounds check
				if (gridX < 0 || gridX >= m_cellNumX ||
					gridY < 0 || gridY >= m_cellNumY ||
					gridZ < 0 || gridZ >= m_cellNumZ + 1)
					continue;


				const float w = W.w[n];

				//accumulate weight
				m_gridWeightW(gridX, gridY, gridZ) += w;

				//accumulate weighted velocity
				m_gridVW(gridX, gridY, gridZ) += w * vel.z();
			}
		}
	}

	Solver_Utils::ParallelFor(0, m_cellNumX + 1, [&](int x) { //Normalize grid velocities on X (U) faces
		for (int y = 0; y < m_cellNumY; ++y) {
			for (int z = 0; z < m_cellNumZ; ++z) {
				if (m_gridWeightU(x, y, z) > 0.0f)
					m_gridVU(x, y, z) /= m_gridWeightU(x, y, z);
			}
		}
	});

	Solver_Utils::ParallelFor(0, m_cellNumX, [&](int x) {
		for (int y = 0; y < m_cellNumY + 1; ++y) {//Normalize grid velocities on Y (V) faces
			for (int z = 0; z < m_cellNumZ; ++z) {
				if (m_gridWeightV(x, y, z) > 0.0f)
					m_gridVV(x, y, z) /= m_gridWeightV(x, y, z);
			}
		}
	});

	Solver_Utils::ParallelFor(0, m_cellNumX, [&](int x) {
		for (int y = 0; y < m_cellNumY; ++y) {
			for (int z = 0; z < m_cellNumZ + 1; ++z) { //Normalize grid velocities on Z (W) faces
				if (m_gridWeightW(x, y, z) > 0.0f)
					m_gridVW(x, y, z) /= m_gridWeightW(x, y, z);
			}
		}
	});
}

void FLIPSolver::SaveGridBefore()
{
	m_gridVUBefore = m_gridVU; //copy gridvelocities for later use
	m_gridVVBefore = m_gridVV;
	m_gridVWBefore = m_gridVW;
}

void FLIPSolver::ApplyGravity(float dt)
{
	//Note: we apply gravity to our cells, not our particles!
	//X faces
	Solver_Utils::ParallelFor(0, m_cellNumX + 1, [&](int x) {
		for (int y = 0; y < m_cellNumY; ++y) {
			for (int z = 0; z < m_cellNumZ; ++z)
			{
				bool leftFluid = (x > 0) && m_gridCellType(x - 1, y, z) == CellType::Fluid;
				bool rightFluid = (x < m_cellNumX) && m_gridCellType(x, y, z) == CellType::Fluid;

				if (leftFluid || rightFluid)
				{
					m_gridVU(x, y, z) += dt * m_gravity.x();
				}
			}
		}
	});

	//Y faces
	Solver_Utils::ParallelFor(0, m_cellNumX, [&](int x) {
		for (int y = 0; y < m_cellNumY + 1; ++y) {
			for (int z = 0; z < m_cellNumZ; ++z)
			{
				bool bottomFluid = (y > 0) && m_gridCellType(x, y - 1, z) == CellType::Fluid;
				bool topFluid = (y < m_cellNumY) && m_gridCellType(x, y, z) == CellType::Fluid;

				if (bottomFluid || topFluid)
				{
					m_gridVV(x, y, z) += dt * m_gravity.y();
				}
			}
		}
	});

	//Z faces
	Solver_Utils::ParallelFor(0, m_cellNumX, [&](int x) {
		for (int y = 0; y < m_cellNumY; ++y) {
			for (int z = 0; z < m_cellNumZ + 1; ++z)
			{
				bool frontAir = (z > 0) && m_gridCellType(x, y, z - 1) == CellType::Fluid;
				bool backAir = (z < m_cellNumZ) && m_gridCellType(x, y, z) == CellType::Fluid;

				if (frontAir || backAir)
				{
					m_gridVW(x, y, z) += dt * m_gravity.z();
				}
			}
		}
	});
}

void FLIPSolver::UpdateParticleDensity()
{
	m_gridDensity.setZero();

	const float& h = m_CellSize;
	const float h2 = m_CellSize * 0.5f;

	for (int p = 0; p < m_numParticles; ++p) //calculate grid density
	{
		Eigen::Vector3f pos = m_particlePos.row(p);


		Eigen::Vector3f fractionalOffset;
		int ix, iy, iz;

		ComputeCellCoordinates(pos, ix, iy, iz, fractionalOffset);


		//Compute neighbour weights
		Solver_Utils::Weight3D W;
		Solver_Utils::ComputeBSplineWeights(fractionalOffset, W);

		for (int n = 0; n < 27; ++n) 
		{
			int gridX = ix + W.offsetsX[n];
			int gridY = iy + W.offsetsY[n];
			int gridZ = iz + W.offsetsZ[n];

			//ignore cells out of bounds
			if (gridX < 0 || gridX >= m_cellNumX ||
				gridY < 0 || gridY >= m_cellNumY ||
				gridZ < 0 || gridZ >= m_cellNumZ)
				continue;

			//allow both fluid and air cells
			if (m_gridCellType(gridX, gridY, gridZ) == Solver_Utils::CellType::Solid)
				continue;

			m_gridDensity(gridX, gridY, gridZ) += W.w[n];
		}
	}
}

void FLIPSolver::SolveIncompressibility(float dt, int iterations)
{
	constexpr float overrelaxation = 1.6f; //use overrelaxation to converge faster, 1.0 == default gauss Seidel

	m_gridPressure.setZero();
	m_gridVUAfter = m_gridVU;
	m_gridVVAfter = m_gridVV;
	m_gridVWAfter = m_gridVW;
	const float scale = dt / m_CellSize;

	auto IsValid = [&](int x, int y, int z) {	//is inside domain? (out-of-bounds treated like solid = 0 velocity)
		return x >= 0 && x < m_cellNumX &&
			y >= 0 && y < m_cellNumY &&
			z >= 0 && z < m_cellNumZ;
		};

	for (int iter = 0; iter < iterations; iter++)
	{
		for (int x = 0; x < m_cellNumX; x++) {
			for (int y = 0; y < m_cellNumY; y++) {
				for (int z = 0; z < m_cellNumZ; z++)
				{
					if (m_gridCellType(x, y, z) != CellType::Fluid)
						continue;

					//Check which of our neighbours are fluid
					bool validX0 = IsValid(x - 1, y, z);
					bool validX1 = IsValid(x + 1, y, z);
					bool validY0 = IsValid(x, y - 1, z);
					bool validY1 = IsValid(x, y + 1, z);
					bool validZ0 = IsValid(x, y, z - 1);
					bool validZ1 = IsValid(x, y, z + 1);

					//we need to know how many valid neighbours we have to know in how many direction we can push our pressure can push our fluid, converting bool to int (kinda ugly but works)
					int numValidNeighbours = validX0 + validX1 + validY0 + validY1 + validZ0 + validZ1;

					if (numValidNeighbours == 0)
						continue; //no usable neighbours


					float div = 0.0f;

					// X
					if (validX1) div += m_gridVU(x + 1, y, z);  //right face
					if (validX0) div -= m_gridVU(x, y, z);      //left face

					// Y
					if (validY1) div += m_gridVV(x, y + 1, z);  //top face
					if (validY0) div -= m_gridVV(x, y, z);      //bottom face

					// Z
					if (validZ1) div += m_gridVW(x, y, z + 1);  //front face
					if (validZ0) div -= m_gridVW(x, y, z);      //back face



					if (m_particleRestDensity > 0.0f) { //compensate density drift
						float compression = m_gridDensity(x, y, z) - m_particleRestDensity;
						if (compression > 0.0f)
						{
							const float driftScale = 0.1f; //test, tune later
							div -= driftScale * compression;
						}
					}


					float pressure = -div / numValidNeighbours;
					pressure *= overrelaxation;

					m_gridPressure(x, y, z) += pressure * scale;

					//x
					if (validX0)
						m_gridVU(x, y, z) -= pressure;
					if (validX1)
						m_gridVU(x + 1, y, z) += pressure;
					//y
					if (validY0)
						m_gridVV(x, y, z) -= pressure;
					if (validY1)
						m_gridVV(x, y + 1, z) += pressure;
					//z
					if (validZ0)
						m_gridVW(x, y, z) -= pressure;
					if (validZ1)
						m_gridVW(x, y, z + 1) += pressure;
				}
			}
		}
	}
}

void FLIPSolver::ComputeDivergence()
{
	auto IsValid = [&](int x, int y, int z) {
		return x >= 0 && x < m_cellNumX &&
			y >= 0 && y < m_cellNumY &&
			z >= 0 && z < m_cellNumZ;
		};

	Solver_Utils::ParallelFor(0, m_cellNumX, [&](int x)
	{
		for (int y = 0; y < m_cellNumY; y++)
		{
			for (int z = 0; z < m_cellNumZ; z++)
			{
				if (m_gridCellType(x, y, z) != CellType::Fluid)
				{
					m_gridDivergence(x, y, z) = 0.0f;
					continue;
				}

				bool validX0 = IsValid(x - 1, y, z);
				bool validX1 = IsValid(x + 1, y, z);
				bool validY0 = IsValid(x, y - 1, z);
				bool validY1 = IsValid(x, y + 1, z);
				bool validZ0 = IsValid(x, y, z - 1);
				bool validZ1 = IsValid(x, y, z + 1);

				float divergence = 0.f;

				//X
				if (validX1) //right neighbour
					divergence += m_gridVU(x + 1, y, z);	//right face
				if (validX0) //left neigbour
					divergence -= m_gridVU(x, y, z); //left face, not x - 1 because staggered grid

				//Y
				if (validY1) //top neigbour
					divergence += m_gridVV(x, y + 1, z);	//up face
				if (validY0) //bottom neighbour
					divergence -= m_gridVV(x, y, z);	//down face

				//Z
				if (validZ1) //back neighbour
					divergence += m_gridVW(x, y, z + 1); //back face
				if (validZ0) //front neighbour
					divergence -= m_gridVW(x, y, z);	//front face


				m_gridDivergence(x, y, z) = divergence;
			}
		}
	});
}

void FLIPSolver::ResolveParticleCollisions()
{
	//bounds in world space
	const float xMin = m_particleRadius;
	const float xMax = (m_cellNumX)*m_CellSize - m_particleRadius;
	const float yMin = m_particleRadius;
	const float yMax = (m_cellNumY)*m_CellSize - m_particleRadius;
	const float zMin = m_particleRadius;
	const float zMax = (m_cellNumZ)*m_CellSize - m_particleRadius;

	const Eigen::Vector3f minBound(xMin, yMin, zMin);
	const Eigen::Vector3f maxBound(xMax, yMax, zMax);

	Solver_Utils::ParallelFor(0, m_numParticles, [&](int p)
	{
		Eigen::Ref<Eigen::RowVector3f> pos = m_particlePos.row(p);
		Eigen::Ref<Eigen::RowVector3f> vel = m_particleV.row(p);

		//foreach axis; 0 = x; 1 = y; 2 = z;
		for (int i = 0; i < 3; ++i)
		{
			if (pos[i] < minBound[i])
			{
				pos[i] = minBound[i];
				float velocityNormal = vel[i]; //velocity along axis
				if (velocityNormal < 0.0f)
					vel[i] -= velocityNormal; //remove normal, keep tangential, helps with velocity up the hill
			}
			else if (pos[i] > maxBound[i])
			{
				pos[i] = maxBound[i];
				float velocityNormal = vel[i];
				if (velocityNormal > 0.0f)
					vel[i] -= velocityNormal;
			}
		}
	});
}

void FLIPSolver::SaveGridAfter()
{
	m_gridVUAfter = m_gridVU; //copy gridvelocities for later use
	m_gridVVAfter = m_gridVV;
	m_gridVWAfter = m_gridVW;
}

void FLIPSolver::TransferG2P(float dt)
{
	ComputeDivergence(); //Necessary for adaptive mixing!!!
	Solver_Utils::ParallelFor(0, m_numParticles, [&](int p)
	{
		const Eigen::Vector3f pos = m_particlePos.row(p).transpose();

		Eigen::Vector3f picVelocity(0.0f, 0.0f, 0.0f);
		Eigen::Vector3f flipVelocity(0.0f, 0.0f, 0.0f);

		{//X faces
			Eigen::Vector3f uPos = pos;
			uPos.y() += 0.5f * m_CellSize;
			uPos.z() += 0.5f * m_CellSize;

			uPos.x() = std::clamp(uPos.x(), 0.0f, m_cellNumX * m_CellSize);
			uPos.y() = std::clamp(uPos.y(), 0.5f * m_CellSize, (m_cellNumY - 0.5f) * m_CellSize);
			uPos.z() = std::clamp(uPos.z(), 0.5f * m_CellSize, (m_cellNumZ - 0.5f) * m_CellSize);


			//base cell idx + fractional offset (f) inside cell
			int ix, iy, iz;
			Eigen::Vector3f f;
			ComputeCellCoordinates(uPos, ix, iy, iz, f);

			//Compute 27 B-spline weights
			Solver_Utils::Weight3D W;
			Solver_Utils::ComputeBSplineWeights(f, W);


			float weightSum = 0.0f;
			float picSum = 0.0f;
			float flipSum = 0.0f;

			//get from grid
			for (int n = 0; n < 27; ++n)
			{
				const int gridX = ix + W.offsetsX[n];
				const int gridY = iy + W.offsetsY[n];
				const int gridZ = iz + W.offsetsZ[n];

				//bounds check
				if (gridX < 0 || gridX >= m_cellNumX + 1 ||
					gridY < 0 || gridY >= m_cellNumY ||
					gridZ < 0 || gridZ >= m_cellNumZ)
					continue;

				bool leftAir = (gridX > 0) && m_gridCellType(gridX - 1, gridY, gridZ) == CellType::Air;
				bool rightAir = (gridX < m_cellNumX) && m_gridCellType(gridX, gridY, gridZ) == CellType::Air;

				//bool valid = !((gridX > 0 && m_gridCellType(gridX - 1, gridY, gridZ) == CellType::Air) && (gridX < m_cellNumX && m_gridCellType(gridX, gridY, gridZ) == CellType::Air));

				if (leftAir && rightAir) //if both cells are air we dont contribute
					continue;

				const float w = W.w[n];
				const float vel = m_gridVU(gridX, gridY, gridZ);
				const float velBefore = m_gridVUBefore(gridX, gridY, gridZ);

				picSum += w * vel;
				flipSum += w * (vel - velBefore);
				weightSum += w;
			}

			if (weightSum > 0.0f)
			{
				picVelocity.x() = picSum / weightSum;
				flipVelocity.x() = flipSum / weightSum;
			}
		}

		{//Y faces
			Eigen::Vector3f vPos = pos;

			vPos.x() += 0.5f * m_CellSize;
			vPos.z() += 0.5f * m_CellSize;

			vPos.x() = std::clamp(vPos.x(), 0.5f * m_CellSize, (m_cellNumX - 0.5f) * m_CellSize);
			vPos.y() = std::clamp(vPos.y(), 0.0f, m_cellNumY * m_CellSize);
			vPos.z() = std::clamp(vPos.z(), 0.5f * m_CellSize, (m_cellNumZ - 0.5f) * m_CellSize);

			//base cell idx + fractional offset (f) inside cell
			int ix, iy, iz;
			Eigen::Vector3f f;
			ComputeCellCoordinates(vPos, ix, iy, iz, f);

			//Compute 27 B-spline weights
			Solver_Utils::Weight3D W;
			Solver_Utils::ComputeBSplineWeights(f, W);


			float weightSum = 0.0f;
			float picSum = 0.0f;
			float flipSum = 0.0f;

			//get from grid
			for (int n = 0; n < 27; ++n)
			{
				const int gridX = ix + W.offsetsX[n];
				const int gridY = iy + W.offsetsY[n];
				const int gridZ = iz + W.offsetsZ[n];

				//bounds check
				if (gridX < 0 || gridX >= m_cellNumX ||
					gridY < 0 || gridY >= m_cellNumY + 1 ||
					gridZ < 0 || gridZ >= m_cellNumZ)
					continue;

				bool bottomAir = (gridY > 0) && m_gridCellType(gridX, gridY - 1, gridZ) == CellType::Air;
				bool topAir = (gridY < m_cellNumY) && m_gridCellType(gridX, gridY, gridZ) == CellType::Air;


				if (bottomAir && topAir) //if both neighbours are air dont contribute
					continue;

				const float w = W.w[n];
				const float vel = m_gridVV(gridX, gridY, gridZ);
				const float velBefore = m_gridVVBefore(gridX, gridY, gridZ);

				picSum += w * vel;
				flipSum += w * (vel - velBefore);
				weightSum += w;
			}

			if (weightSum > 0.0f)
			{
				picVelocity.y() = picSum / weightSum;
				flipVelocity.y() = flipSum / weightSum;
			}
		}

		{//Z faces
			Eigen::Vector3f wPos = pos;

			wPos.z() += 0.5f * m_CellSize;
			wPos.y() += 0.5f * m_CellSize;

			wPos.x() = std::clamp(wPos.x(), 0.5f * m_CellSize, (m_cellNumX - 0.5f) * m_CellSize);
			wPos.y() = std::clamp(wPos.y(), 0.5f * m_CellSize, (m_cellNumY - 0.5f) * m_CellSize);
			wPos.z() = std::clamp(wPos.z(), 0.0f, m_cellNumZ * m_CellSize);

			//base cell idx + fractional offset (f) inside cell
			int ix, iy, iz;
			Eigen::Vector3f f;
			ComputeCellCoordinates(wPos, ix, iy, iz, f);

			//Compute 27 B-spline weights
			Solver_Utils::Weight3D W;
			Solver_Utils::ComputeBSplineWeights(f, W);


			float weightSum = 0.0f;
			float picSum = 0.0f;
			float flipSum = 0.0f;

			//get from grid
			for (int n = 0; n < 27; ++n)
			{
				const int gridX = ix + W.offsetsX[n];
				const int gridY = iy + W.offsetsY[n];
				const int gridZ = iz + W.offsetsZ[n];

				//bounds check
				if (gridX < 0 || gridX >= m_cellNumX ||
					gridY < 0 || gridY >= m_cellNumY ||
					gridZ < 0 || gridZ >= m_cellNumZ + 1)
					continue;

				bool frontAir = (gridZ > 0) && m_gridCellType(gridX, gridY, gridZ - 1) == CellType::Air;
				bool backAir = (gridZ < m_cellNumZ) && m_gridCellType(gridX, gridY, gridZ) == CellType::Air;

				//bool valid = !((gridZ > 0 && m_gridCellType(gridX, gridY, gridZ - 1) == CellType::Air) && (gridZ < m_cellNumZ && m_gridCellType(gridX, gridY, gridZ) == CellType::Air));

				if (frontAir && backAir) //dont let air cells contribute
					continue;

				const float w = W.w[n];
				const float vel = m_gridVW(gridX, gridY, gridZ);
				const float velBefore = m_gridVWBefore(gridX, gridY, gridZ);

				picSum += w * vel;
				flipSum += w * (vel - velBefore);
				weightSum += w;
			}

			if (weightSum > 0.0f)
			{
				picVelocity.z() = picSum / weightSum;
				flipVelocity.z() = flipSum / weightSum;
			}
		}

		//Blend PIC and FLIP
		Eigen::Vector3f vOld = m_particleV.row(p).transpose();
		float picAlpha = m_alphaPIC;
		if (m_useAdaptiveMixing)
			picAlpha = m_alphaPIC;
		else
			picAlpha = CalculatePicAlpha(dt, pos);
		Eigen::Vector3f vNew = picAlpha * picVelocity + (1.f - picAlpha) * (vOld + flipVelocity);
		m_particleV.row(p) = vNew.transpose(); //transpose because m_particleV is RowMajor
	});
}


//===============
//Adaptive mixing
//===============
float FLIPSolver::CalculatePicAlpha(float dt, const Eigen::Vector3f& particlePos) const
{
	constexpr float divergenceScale = 0.1f; //tune!!!
	constexpr float maxPic = 1.0f;

	float divergence = std::abs(GetWeightedDivergenceAtPos(particlePos));
	float alpha = divergence * dt * divergenceScale;
	alpha = std::clamp(alpha, 0.0f, maxPic);
	return alpha;
}

float FLIPSolver::GetWeightedDivergenceAtPos(const Eigen::Vector3f& pos) const
{
	int ix, iy, iz;
	Eigen::Vector3f f;
	ComputeCellCoordinates(pos, ix, iy, iz, f);

	Solver_Utils::Weight3D W;
	Solver_Utils::ComputeBSplineWeights(f, W);

	float divergence = 0.0f;
	float weightSum = 0.0f;

	for (int n = 0; n < 27; ++n)
	{
		int gridX = ix + W.offsetsX[n];
		int gridY = iy + W.offsetsY[n];
		int gridZ = iz + W.offsetsZ[n];

		if (gridX < 0 || gridX >= m_cellNumX ||
			gridY < 0 || gridY >= m_cellNumY ||
			gridZ < 0 || gridZ >= m_cellNumZ)
			continue;

		if (m_gridCellType(gridX, gridY, gridZ) != CellType::Fluid)
			continue;

		float w = W.w[n];
		divergence += w * m_gridDivergence(gridX, gridY, gridZ);
		weightSum += w;
	}

	return (weightSum > 0.0f) ? divergence / weightSum : 0.0f; //prevent division by zero
}



//=========
//Helpers
//=========
void FLIPSolver::PushParticlesApart(int iterations)
{
	const float minDist = 2.0f * m_particleRadius;
	const float sqrMinDist = std::pow(minDist, 2);


	//Build particle in cell map
	std::vector<std::vector<int>> cellParticles(totalNumCells());

	for (int p = 0; p < m_numParticles; p++)
	{
		int ix, iy, iz;
		Eigen::Vector3f f; //unused
		ComputeCellCoordinates(m_particlePos.row(p), ix, iy, iz, f);

		int cellIndex = (ix * m_cellNumY * m_cellNumZ) + (iy * m_cellNumZ) + iz;
		cellParticles[cellIndex].push_back(p);
	}

	for (int iter = 0; iter < iterations; ++iter)
	{
		for (int p = 0; p < m_numParticles; ++p)
		{
			Eigen::Ref<Eigen::Vector3f> posP = m_particlePos.row(p);

			int ix, iy, iz;
			Eigen::Vector3f f; //unused
			ComputeCellCoordinates(posP, ix, iy, iz, f);

			float iterFactor = 0.5f * (1.0f / (iter + 1));

			//loop over neighbouring cells
			for (int nx = std::max(ix - 1, 0); nx <= std::min(ix + 1, m_cellNumX - 1); ++nx) {
				for (int ny = std::max(iy - 1, 0); ny <= std::min(iy + 1, m_cellNumY - 1); ++ny) {
					for (int nz = std::max(iz - 1, 0); nz <= std::min(iz + 1, m_cellNumZ - 1); ++nz)
					{
						int neighborCell = nx * m_cellNumY * m_cellNumZ + ny * m_cellNumZ + nz;

						for (int q : cellParticles[neighborCell])
						{
							if (q <= p) continue; // avoid double-counting and self

							Eigen::Ref<Eigen::Vector3f> posQ = m_particlePos.row(q);
							Eigen::Vector3f delta = posQ - posP;
							float dist2 = delta.squaredNorm();

							if (dist2 == 0.0f || dist2 >= sqrMinDist)
								continue;

							float dist = std::sqrt(dist2);
							float pushFactor = 0.5f * (minDist - dist) / dist;
							Eigen::Vector3f correction = pushFactor * delta;

							// Move both particles apart
							posP -= correction;
							posQ += correction;
						}
					}
				}
			}
		}
	}
}

void FLIPSolver::ComputeCellCoordinates(const Eigen::Vector3f& particle, int& ix, int& iy, int& iz, Eigen::Vector3f& f) const
{
	//convert from grid space to world space
	Eigen::Vector3f gridPos = particle / m_CellSize;

	ix = static_cast<int>(std::floor(gridPos.x()));
	iy = static_cast<int>(std::floor(gridPos.y()));
	iz = static_cast<int>(std::floor(gridPos.z()));

	ix = std::clamp(ix, 0, m_cellNumX - 1);
	iy = std::clamp(iy, 0, m_cellNumY - 1);
	iz = std::clamp(iz, 0, m_cellNumZ - 1);

	//f = normalized distance to grid cell origin
	f.x() = gridPos.x() - ix;
	f.y() = gridPos.y() - iy;
	f.z() = gridPos.z() - iz;
}



//=========
//Logging
//=========
void FLIPSolver::StartMeasurement()
{
	m_measureStart = std::chrono::high_resolution_clock::now();
}

void FLIPSolver::EndMeasurement()
{
	FrameMeasurement fm;
	//measure time taken
	auto endTime = std::chrono::high_resolution_clock::now();
	float stepTime = std::chrono::duration<float, std::milli>(endTime - m_measureStart).count(); //in muilliseconds
	fm.stepTime = stepTime; 

	//compute avg weighted divergence, fix this this is pointless, dont use abs()
	ComputeDivergence();
	float totalDivergence = 0.0f;
	for (int p = 0; p < m_numParticles; ++p)
	{
		float weightedDivergence = std::abs(GetWeightedDivergenceAtPos(m_particlePos.row(p)));
		totalDivergence += weightedDivergence;
	}
	fm.averageDivergence = totalDivergence / m_numParticles;


	//compute density error
	float totalCompression = 0.0f;
	fm.maxCompression = 0.0f;
	int compressedCellCount = 0;

	for (int ix = 0; ix < m_cellNumX; ix++){
		for (int iy = 0; iy < m_cellNumY; iy++){
			for (int iz = 0; iz < m_cellNumZ; iz++)
			{
				//if (m_gridDensity(ix, iy, iz) < 0.5f * m_particleRestDensity) //avoid counting free surfaces
				//	continue;

				float compression = m_gridDensity(ix, iy, iz) - m_particleRestDensity;
				if (compression > 0.0f)
				{
					totalCompression += compression;
					compressedCellCount++;
					fm.maxCompression = std::max(fm.maxCompression, compression);
				}
					
			}
		}
	}
	if(compressedCellCount > 0)
		fm.averageCompression = totalCompression / compressedCellCount;


	m_frameMeasurements.emplace_back(fm);
}

void FLIPSolver::WriteLog(const std::string& filename) const
{
	std::cout << "(Solver) Writing output to: " << filename << std::endl;

	std::ofstream outFile(filename);
	if (!outFile.is_open())
		return;

	// Write header
	outFile << "FrameIdx,StepTime (in ms),AverageDivergence\n";

	for (size_t i = 0; i < m_frameMeasurements.size(); ++i)
	{
		std::cout << "frame: " << i << std::endl;
		const FrameMeasurement& fm = m_frameMeasurements[i];
		outFile << i << "," << fm.stepTime << "," << fm.averageDivergence << "\n";
	}
	std::cout << "(Solver) Succesfully written output to: " << filename << std::endl;
}
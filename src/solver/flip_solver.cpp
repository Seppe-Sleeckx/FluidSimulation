#include "flip_solver.h"
#include <limits>
using namespace Solver_Utils;

//DEBUG
#include <iostream>

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
	m_particleV(m_numParticles, 3)
{
	m_particleV.setZero();
}

void FLIPSolver::Initialize()
{
	for (size_t p = 0; p < m_numParticles; p++)
	{
		m_particlePos.row(p) = Eigen::Vector3f(p * .05f, 5, p * .1f);
	}

	m_gridCellType.setConstant(CellType::Air); //start all cells at air, maybe change later to be configurable

	ClearGrid();
	MarkFluidCells();
}

void FLIPSolver::Simulate(float dt)
{
	//Advection step
	IntegrateParticles(dt);
	PushParticlesApart(3);
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
	SolveIncompressibility(dt, 40);

	//store grid velocity after projection (Only necessary for FLIP)
	SaveGridAfter();

	//Grid to particle (G2P)
	TransferG2P();
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
	for (int x = 0; x < m_cellNumX; ++x) {
		for (int y = 0; y < m_cellNumY; ++y) {
			for (int z = 0; z < m_cellNumZ; ++z)
			{
				if (m_gridCellType(x, y, z) != CellType::Solid)
					m_gridCellType(x, y, z) = CellType::Air; //set all non solid cells to air
			}
		}
	}

	for (int p = 0; p < m_numParticles; ++p)
	{
		const Eigen::Vector3f& pos = m_particlePos.row(p);

		int ix, iy, iz;
		Eigen::Vector3f f;
		ComputeCellCoordinates(pos, ix, iy, iz, f);

		if (m_gridCellType(ix, iy, iz) != CellType::Solid)
			m_gridCellType(ix, iy, iz) = CellType::Fluid; //mark all cells with particles inside as fluid (except for solid cells)
	}

	//DEBUG:
	//constexpr auto cellTypeToString = [](CellType type) -> const char* {
	//	switch (type)
	//	{
	//	case CellType::Air:   return "Air";
	//	case CellType::Fluid: return "Fluid";
	//	case CellType::Solid: return "Solid";
	//	default:              return "-";
	//	}
	//	};
	//for (int x = 0; x < m_cellNumX; ++x) {
	//	for (int y = 0; y < m_cellNumY; ++y) {
	//		for (int z = 0; z < m_cellNumZ; ++z)
	//		{
	//			auto type = m_gridCellType(x, y, z);
	//			if (type != CellType::Fluid)
	//				continue;
	//			std::cout << "CELL{" << x << ',' << y << ',' << z << "} has type " << cellTypeToString(type) << '\n';
	//		}
	//	}
	//}
}

void FLIPSolver::IntegrateParticles(float dt)
{
	for (int i = 0; i < m_numParticles; ++i)
	{
		m_particlePos.row(i) += dt * m_particleV.row(i); //update positions
	}
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

	for (int x = 0; x < m_cellNumX + 1; ++x) { //Normalize grid velocities on X (U) faces
		for (int y = 0; y < m_cellNumY; ++y) {
			for (int z = 0; z < m_cellNumZ; ++z) {
				if (m_gridWeightU(x, y, z) > 0.0f)
					m_gridVU(x, y, z) /= m_gridWeightU(x, y, z);
			}
		}
	}

	for (int x = 0; x < m_cellNumX; ++x) {
		for (int y = 0; y < m_cellNumY + 1; ++y) {//Normalize grid velocities on Y (V) faces
			for (int z = 0; z < m_cellNumZ; ++z) {
				if (m_gridWeightV(x, y, z) > 0.0f)
					m_gridVV(x, y, z) /= m_gridWeightV(x, y, z);
			}
		}
	}

	for (int x = 0; x < m_cellNumX; ++x) {
		for (int y = 0; y < m_cellNumY; ++y) {
			for (int z = 0; z < m_cellNumZ + 1; ++z) { //Normalize grid velocities on Z (W) faces
				if (m_gridWeightW(x, y, z) > 0.0f)
					m_gridVW(x, y, z) /= m_gridWeightW(x, y, z);
			}
		}
	}
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
	for (int x = 0; x < m_cellNumX + 1; ++x) {
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
	}

	//Y faces
	for (int x = 0; x < m_cellNumX; ++x) {
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
	}

	//Z faces
	for (int x = 0; x < m_cellNumX; ++x) {
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
	}

	//Gravity applied directly to particles
	//for (int p = 0; p < m_numParticles; p++)
	//{
	//	m_particleV.row(p) += m_gravity * dt;
	//}
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
	//Gauss Seidel (use PCG later when more time)
	constexpr float overrelaxation = 1.6f; //use overrelaxation to converge faster, 1.0 == default gauss Seidel

	m_gridPressure.setZero();
	m_gridVUAfter = m_gridVU;
	m_gridVVAfter = m_gridVV;
	m_gridVWAfter = m_gridVW;
	const float scale = dt / m_CellSize;


	auto IsValidNeighbor = [&](int x, int y, int z) {
		return x >= 0 && x < m_cellNumX &&
			y >= 0 && y < m_cellNumY &&
			z >= 0 && z < m_cellNumZ &&
			m_gridCellType(x, y, z) != CellType::Solid;
		};

	for (int iter = 0; iter < iterations; iter++)
	{
		ComputeDivergence();
		for (int x = 0; x < m_cellNumX; x++) {
			for (int y = 0; y < m_cellNumY; y++) {
				for (int z = 0; z < m_cellNumZ; z++)
				{
					if (m_gridCellType(x, y, z) != CellType::Fluid)
						continue;

					//Check which of our neighbours are fluid
					bool validX0 = IsValidNeighbor(x - 1, y, z);
					bool validX1 = IsValidNeighbor(x + 1, y, z);
					bool validY0 = IsValidNeighbor(x, y - 1, z);
					bool validY1 = IsValidNeighbor(x, y + 1, z);
					bool validZ0 = IsValidNeighbor(x, y, z - 1);
					bool validZ1 = IsValidNeighbor(x, y, z + 1);

					//we need to know how many valid neighbours we have to know in how many direction we can push our pressure can push our fluid, converting bool to int (kinda ugly but works)
					int numValidNeighbours = validX0 + validX1 + validY0 + validY1 + validZ0 + validZ1;

					if (numValidNeighbours == 0)
						continue; //no usable neighbours


					if (m_particleRestDensity > 0.0f) //compensate for drift (prevents growing/shrinking of total volume)
					{
						float density = m_gridDensity(x, y, z);

						float compression = (density - m_particleRestDensity) / m_particleRestDensity;

						if (compression > 0.0f)
						{
							const float driftScale = 0.1f; //test, tune later
							m_gridDivergence(x, y, z) -= driftScale * compression;
						}
					}



					float pressure = -m_gridDivergence(x, y, z) / numValidNeighbours;
					pressure *= overrelaxation; //we use overrelaxation to converge faster with less iterations
					pressure *= scale;

					m_gridPressure(x, y, z) += pressure;

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
	auto IsValidNeighbor = [&](int x, int y, int z) {
		return x >= 0 && x < m_cellNumX &&
			y >= 0 && y < m_cellNumY &&
			z >= 0 && z < m_cellNumZ &&
			m_gridCellType(x, y, z) != CellType::Solid;
		};

	for (int x = 0; x < m_cellNumX; x++)
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

				float divergence = 0.f;

				//X
				if (IsValidNeighbor(x + 1, y, z)) //right neighbour
					divergence += m_gridVU(x + 1, y, z);	//right face
				if (IsValidNeighbor(x - 1, y, z)) //left neigbour
					divergence -= m_gridVU(x, y, z); //left face, not x - 1 because staggered grid

				//Y
				if (IsValidNeighbor(x, y + 1, z)) //top neigbour
					divergence += m_gridVV(x, y + 1, z);	//up face
				if (IsValidNeighbor(x, y - 1, z)) //bottom neighbour
					divergence -= m_gridVV(x, y, z);	//down face

				//Z
				if (IsValidNeighbor(x, y, z + 1)) //back neighbour
					divergence += m_gridVW(x, y, z + 1); //back face
				if (IsValidNeighbor(x, y, z - 1)) //front neighbour
					divergence -= m_gridVW(x, y, z);	//front face


				m_gridDivergence(x, y, z) = divergence * inverseGridSpacing();
			}
		}
	}
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

	for (int p = 0; p < m_numParticles; p++)
	{
		Eigen::Ref<Eigen::RowVector3f> particle_p = m_particlePos.row(p).transpose();	//Eigen::ref prevents a copy and is writable, need to use rowVector3f since m_particlePos is rowMajor
		Eigen::Ref<Eigen::RowVector3f> particle_v = m_particleV.row(p).transpose(); //need to transpose because m_particleV is rowMajor, and Vector3f is a column vector
		//transpose does NOT return a copy, it returns a view that references our original (rowMajor) velocity/position

		//x
		if (particle_p.x() < xMin) {
			particle_p.x() = xMin;
			if (particle_v.x() < 0.0f) //prevents sticking to walls
				particle_v.x() = 0.0f;
		}
		else if (particle_p.x() > xMax) {
			particle_p.x() = xMax;
			if (particle_v.x() > 0.0f)
				particle_v.x() = 0.0f;
		}

		//y
		if (particle_p.y() < yMin) {
			particle_p.y() = yMin;
			if (particle_v.y() < 0.0f) //prevents sticking to walls
				particle_v.y() = 0.0f;
		}
		else if (particle_p.y() > yMax) {
			particle_p.y() = yMax;
			if (particle_v.y() > 0.0f)
				particle_v.y() = 0.0f;
		}

		//z
		if (particle_p.z() < zMin) {
			particle_p.z() = zMin;
			if (particle_v.z() < 0.0f) //prevents sticking to walls
				particle_v.z() = 0.0f;
		}
		else if (particle_p.z() > zMax) {
			particle_p.z() = zMax;
			if (particle_v.z() > 0.0f)
				particle_v.z() = 0.0f;
		}
	}
}

void FLIPSolver::SaveGridAfter()
{
	m_gridVUAfter = m_gridVU; //copy gridvelocities for later use
	m_gridVVAfter = m_gridVV;
	m_gridVWAfter = m_gridVW;
}

void FLIPSolver::TransferG2P()
{
	for (int p = 0; p < m_numParticles; ++p)
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

		//Blend PIC and FLIP, change later to use Adaptive mixing
		Eigen::Vector3f vOld = m_particleV.row(p).transpose();
		Eigen::Vector3f vNew = (1.f - m_alphaPICFLIP) * picVelocity + m_alphaPICFLIP * (vOld + flipVelocity);
		m_particleV.row(p) = vNew.transpose(); //transpose because m_particleV is RowMajor
	}
}



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
							float pushFactor = 0.1f * (minDist - dist) / dist;
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

void FLIPSolver::ComputeCellCoordinates(const Eigen::Vector3f& particle, int& ix, int& iy, int& iz, Eigen::Vector3f& f)
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
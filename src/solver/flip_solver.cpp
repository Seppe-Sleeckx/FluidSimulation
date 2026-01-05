#include "flip_solver.h"
#include <limits>
using namespace Solver_Utils;

FLIPSolver::FLIPSolver(const SolverConfig& config) :
	m_gridWeight{ Eigen::Tensor<float, 3>(m_cellNumX, m_cellNumY, m_cellNumZ) }
{
}

void FLIPSolver::Simulate(float dt)
{
	//1. clear grid
	ClearGrid();

	//2. Particle velocity to grid (P2G)
	TransferP2G();

	//3 Save grid velocities (Only necessary for FLIP)
	SaveGridBefore();

	//4 Apply external forces (gravity)
	ApplyGravity(dt);

	//5. Solve for incompressibility (only applicable for fluids, not gasses)
	ComputeDivergence();
	SolvePressure(40);
	ProjectVelocity();

	//6. Enforce boundaries
	ApplyGridBoundaryConditions();

	//7. store grid velocity after projection (Only necessary for FLIP)
	SaveGridAfter();

	//8. Grid to particle (G2P)
	TransferG2P();

	//9. Advection step
	IntegrateParticles(dt);
}

void FLIPSolver::ClearGrid()
{
	m_gridV.setZero();
	m_gridVBefore.setZero();
	m_gridVAfter.setZero();
	m_gridWeight.setZero();
	m_gridPressure.setZero();
	m_gridDivergence.setZero();
}

void FLIPSolver::TransferP2G()
{
	for (int p = 0; p < m_numParticles; ++p)
	{
		const Eigen::Vector3f pos = m_particlePos.row(p);
		const Eigen::Vector3f vel = m_particleV.row(p);

		//base cell idx + fractional offset (f) inside cell
		int ix, iy, iz;
		Eigen::Vector3f f;
		ComputeCellCoordinates(pos, ix, iy, iz, f);

		//Calulcate weights
		Solver_Utils::Weight3D W;
		Solver_Utils::ComputeBSplineWeights(f, W);

		//Scatter particle velocity to grid (27 neighbours)
		for (int n = 0; n < 27; ++n)
		{
			const int gridX = ix + W.ox[n];
			const int gridY = iy + W.oy[n];
			const int gridZ = iz + W.oz[n];

			//bounds check
			if (gridX < 0 || gridX >= m_cellNumX ||
				gridY < 0 || gridY >= m_cellNumY ||
				gridZ < 0 || gridZ >= m_cellNumZ)
				continue;


			const float w = W.w[n];

			//accumulate weight
			m_gridWeight(gridX, gridY, gridZ) += w;

			//accumulate weighted velocity
			m_gridV(gridX, gridY, gridZ, 0) += w * vel.x();
			m_gridV(gridX, gridY, gridZ, 1) += w * vel.y();
			m_gridV(gridX, gridY, gridZ, 2) += w * vel.z();
		}
	}
}

void FLIPSolver::SaveGridBefore()
{
	m_gridVBefore = m_gridV; //copy gridvelocities for later use
}

void FLIPSolver::ApplyGravity(float dt)
{
	for (int i = 0; i < m_numParticles; ++i)
	{
		m_particleV.row(i) += dt * m_gravity.transpose(); //apply gravity
	}
}

void FLIPSolver::ComputeDivergence()
{
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

				//X out (+)
				if ((x + 1 < m_cellNumX) && m_gridCellType(x + 1, y, z) != CellType::Solid)
					divergence += m_gridV(x + 1, y, z, 0);

				//X in (-)
				if (m_gridCellType(x, y, z) != CellType::Solid)
					divergence -= m_gridV(x, y, z, 0);

				//Y out (+)
				if ((y + 1 < m_cellNumY) && m_gridCellType(x, y + 1, z) != CellType::Solid)
					divergence += m_gridV(x, y + 1, z, 1);

				//Y in (-)
				if (m_gridCellType(x, y, z) != CellType::Solid)
					divergence -= m_gridV(x, y, z, 1);

				//Z out (+)
				if ((z + 1 < m_cellNumZ) && m_gridCellType(x, y, z + 1) != CellType::Solid)
					divergence += m_gridV(x, y, z + 1, 2);

				//Z in (-)
				if (m_gridCellType(x, y, z) != CellType::Solid)
					divergence -= m_gridV(x, y, z, 2);



				m_gridDivergence(x, y, z) = divergence;
			}
		}
	}
}

void FLIPSolver::SolvePressure(int iterations)
{
	//Gauss Seidel (use PCG later when more time)
	m_gridPressure.setZero();

	for (int i = 0; i < iterations; i++)
	{
		for (int x = 0; x < m_cellNumX; x++) {
			for (int y = 0; y < m_cellNumY; y++) {
				for (int z = 0; z < m_cellNumZ; z++)
				{
					if (m_gridCellType(x, y, z) != CellType::Fluid)
						continue;

					float sum = 0.f;
					int count = 0;

					auto addNeighbor = [&](int nx, int ny, int nz)
						{
							if (nx < 0 || nx >= m_cellNumX ||
								ny < 0 || ny >= m_cellNumY ||
								nz < 0 || nz >= m_cellNumZ)
								return;

							if (m_gridCellType(nx, ny, nz) == CellType::Solid)
								return;

							sum += m_gridPressure(nx, ny, nz);
							count++;
						};

					addNeighbor(x + 1, y, z);
					addNeighbor(x - 1, y, z);
					addNeighbor(x, y + 1, z);
					addNeighbor(x, y - 1, z);
					addNeighbor(x, y, z + 1);
					addNeighbor(x, y, z - 1);

					if (count > 0)
					{
						m_gridPressure(x, y, z) = (sum - m_gridDivergence(x, y, z)) / float(count);
					}
				}
			}
		}
	}
}

void FLIPSolver::ProjectVelocity()
{
	for (int x = 0; x < m_cellNumX; x++) {
		for (int y = 0; y < m_cellNumY; y++) {
			for (int z = 0; z < m_cellNumZ; z++)
			{
				if (m_gridCellType(x, y, z) != CellType::Fluid)
					continue;

				//X
				if (x + 1 < m_cellNumX && m_gridCellType(x + 1, y, z) != CellType::Solid)
				{
					//pressureGradient = gridpressure_neighbour - gridpressure_current
					float pressureGradient = m_gridPressure(x + 1, y, z) - m_gridPressure(x, y, z);
					m_gridV(x, y, z, 0) -= pressureGradient;
				}

				//Y
				if (y + 1 < m_cellNumY && m_gridCellType(x, y + 1, z) != CellType::Solid)
				{
					float pressureGradient = m_gridPressure(x, y + 1, z) - m_gridPressure(x, y, z);
					m_gridV(x, y, z, 1) -= pressureGradient;
				}

				//Z
				if (z + 1 < m_cellNumZ && m_gridCellType(x, y, z + 1) != CellType::Solid)
				{
					float pressureGradient = m_gridPressure(x, y, z + 1) - m_gridPressure(x, y, z);
					m_gridV(x, y, z, 2) -= pressureGradient;
				}
			}
		}
	}
}

void FLIPSolver::ApplyGridBoundaryConditions()
{
	//1. Set velocities to zero inside solid cells
	for (int x = 0; x < m_cellNumX; ++x)
	{
		for (int y = 0; y < m_cellNumY; ++y)
		{
			for (int z = 0; z < m_cellNumZ; ++z)
			{
				if (m_gridCellType(x, y, z) == CellType::Solid)
				{
					m_gridV(x, y, z, 0) = 0.0f;
					m_gridV(x, y, z, 1) = 0.0f;
					m_gridV(x, y, z, 2) = 0.0f;
				}
			}
		}
	}


	//2. Prevent flow into solid neighbours
	for (int x = 0; x < m_cellNumX; ++x)
	{
		for (int y = 0; y < m_cellNumY; ++y)
		{
			for (int z = 0; z < m_cellNumZ; ++z)
			{
				if (m_gridCellType(x, y, z) != CellType::Fluid)
					continue;


				//X out (+)
				if (x + 1 < m_cellNumX && m_gridCellType(x + 1, y, z) == CellType::Solid)
					m_gridV(x, y, z, 0) = std::min(0.0f, m_gridV(x, y, z, 0));

				//X in (-)
				if (x > 0 && m_gridCellType(x - 1, y, z) == CellType::Solid)
					m_gridV(x, y, z, 0) = std::max(0.0f, m_gridV(x, y, z, 0));

				//Y out (+)
				if (y + 1 < m_cellNumY && m_gridCellType(x, y + 1, z) == CellType::Solid)
					m_gridV(x, y, z, 1) = std::min(0.0f, m_gridV(x, y, z, 1));

				//Y in (-)
				if (y > 0 && m_gridCellType(x, y - 1, z) == CellType::Solid)
					m_gridV(x, y, z, 1) = std::max(0.0f, m_gridV(x, y, z, 1));
				
				//Z out (+)
				if (z + 1 < m_cellNumZ && m_gridCellType(x, y, z + 1) == CellType::Solid)
					m_gridV(x, y, z, 2) = std::min(0.0f, m_gridV(x, y, z, 2));

				//Z in (-)
				if (z > 0 && m_gridCellType(x, y, z - 1) == CellType::Solid)
					m_gridV(x, y, z, 2) = std::max(0.0f, m_gridV(x, y, z, 2));
			}
		}
	}


	//3. Enforce grid edge boundaries 

	//X boundaries
	for (int y = 0; y < m_cellNumY; ++y)
	{
		for (int z = 0; z < m_cellNumZ; ++z)
		{
			m_gridV(0, y, z, 0) = 0.f;
			m_gridV(m_cellNumX - 1, y, z, 0) = 0.f;
		}
	}

	//Y boundaries
	for (int x = 0; x < m_cellNumX; ++x)
	{
		for (int z = 0; z < m_cellNumZ; ++z)
		{
			m_gridV(x, 0, z, 1) = 0.f;
			m_gridV(x, m_cellNumY - 1, z, 1) = 0.f;
		}
	}

	//Z boundaries
	for (int x = 0; x < m_cellNumX; ++x)
	{
		for (int y = 0; y < m_cellNumY; ++y)
		{
			m_gridV(x, y, 0, 2) = 0.f;
			m_gridV(x, y, m_cellNumZ - 1, 2) = 0.f;
		}
	}
}

void FLIPSolver::SaveGridAfter()
{
	m_gridVAfter = m_gridV; //copy gridvelocities for later use
}

void FLIPSolver::TransferG2P()
{
	for (int p = 0; p < m_numParticles; ++p)
	{
		const Eigen::Vector3f pos = m_particlePos.row(p);

		//base cell idx + fractional offset (f) inside cell
		int ix, iy, iz;
		Eigen::Vector3f f;
		ComputeCellCoordinates(pos, ix, iy, iz, f);

		//Compute 27 B-spline weights
		Solver_Utils::Weight3D W;
		Solver_Utils::ComputeBSplineWeights(f, W);

		Eigen::Vector3f picVelocity(0.0f, 0.0f, 0.0f);
		Eigen::Vector3f flipDelta(0.0f, 0.0f, 0.0f);

		//get from grid
		for (int n = 0; n < 27; ++n)
		{
			const int gridX = ix + W.ox[n];
			const int gridY = iy + W.oy[n];
			const int gridZ = iz + W.oz[n];

			//bounds check
			if (gridX < 0 || gridX >= m_cellNumX ||
				gridY < 0 || gridY >= m_cellNumY ||
				gridZ < 0 || gridZ >= m_cellNumZ)
				continue;

			const float w = W.w[n];
			const float gridW = m_gridWeight(gridX, gridY, gridZ);

			if (gridW < std::numeric_limits<float>::epsilon())
				continue; //continue

			//normalized grid velocity
			Eigen::Vector3f gridVel(
				m_gridV(gridX, gridY, gridZ, 0) / gridW,
				m_gridV(gridX, gridY, gridZ, 1) / gridW,
				m_gridV(gridX, gridY, gridZ, 2) / gridW
			);

			//PIC velocity contribution
			picVelocity += w * gridVel;

			//FLIP velocity delta
			Eigen::Vector3f gridDelta(
				(m_gridV(gridX, gridY, gridZ, 0) - m_gridVBefore(gridX, gridY, gridZ, 0)) / gridW,
				(m_gridV(gridX, gridY, gridZ, 1) - m_gridVBefore(gridX, gridY, gridZ, 1)) / gridW,
				(m_gridV(gridX, gridY, gridZ, 2) - m_gridVBefore(gridX, gridY, gridZ, 2)) / gridW
			);

			flipDelta += w * gridDelta;
		}

		//Blend PIC and FLIP, change later to use Adaptive mixing
		m_particleV.row(p) = (1.f - m_alphaPICFLIP) * picVelocity + m_alphaPICFLIP * (m_particleV.row(p) + flipDelta.transpose());
	}
}

void FLIPSolver::IntegrateParticles(float dt)
{
	for (int i = 0; i < m_numParticles; ++i)
	{
		m_particlePos.row(i) += dt * m_particleV.row(i); //update positions
	}
}

void FLIPSolver::ComputeCellCoordinates(const Eigen::Vector3f& particle, int& ix, int& iy, int& iz, Eigen::Vector3f& f)
{
	//convert from grid space to world space
	float h = 1.0f / m_particleInvSpacing;
	float gx = particle.x() * m_particleInvSpacing;
	float gy = particle.y() * m_particleInvSpacing;
	float gz = particle.z() * m_particleInvSpacing;

	ix = std::clamp(int(std::floor(gx)), 0, m_cellNumX - 1);
	iy = std::clamp(int(std::floor(gy)), 0, m_cellNumY - 1);
	iz = std::clamp(int(std::floor(gz)), 0, m_cellNumZ - 1);

	//f = normalized distance to grid cell origin
	f.x() = gx - ix;
	f.y() = gy - iy;
	f.z() = gz - iz;
}
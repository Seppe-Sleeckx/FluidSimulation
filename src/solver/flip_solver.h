#include "solver_utils.h"
#include <unsupported/Eigen/CXX11/Tensor>

class FLIPSolver final
{
public:
	FLIPSolver() = delete;
	FLIPSolver(const SolverConfig& config);
	~FLIPSolver() = delete;

	FLIPSolver(const FLIPSolver& other) = delete;
	FLIPSolver(FLIPSolver&& other) = delete;
	FLIPSolver& operator=(const FLIPSolver& other) = delete;
	FLIPSolver& operator=(FLIPSolver&& other) = delete;

	void Simulate(float dt);

private:
	Eigen::Vector3f m_gravity;
	float m_alphaPICFLIP;

	//-- Grid --
	Eigen::Tensor<float, 4> m_gridV; //(X, Y, Z) + (0 = u, 1 = v, 2 = w)
	Eigen::Tensor<float, 4> m_gridVBefore; //before projection
	Eigen::Tensor<float, 4> m_gridVAfter; //after projection
	Eigen::Tensor<float, 3> m_gridWeight;
	Eigen::Tensor<float, 3> m_gridPressure;  //Pressure per cell
	Eigen::Tensor<float, 3> m_gridDivergence;//Divergence per cell
	Eigen::Tensor<Solver_Utils::CellType, 3>   m_gridCellType;		//0 = fluid, 1 = air, 2 = solid boundary
	
	int m_cellNumX; //Num cells in X
	int m_cellNumY; //Num cells in Y
	int m_cellNumZ; //Num cells in Z
	int m_NumCells; //Num of total cells Numx * NumY * NumZ;

	//-- Particles --
	int m_numParticles;
	float m_particleRadius;
	float m_particleInvSpacing; //(1.0 / 2.2 * radius), we use 2.2 because its slightly bigger than 2 so we dont miss our neighbours

	Eigen::MatrixXf m_particleV;
	Eigen::MatrixXf m_particlePos;

	void ClearGrid();
	void TransferP2G();
	void SaveGridBefore();
	void ApplyGridBoundaryConditions();
	void ComputeDivergence();
	void SolvePressure(int iterations);
	void ProjectVelocity();
	void SaveGridAfter();
	void TransferG2P();
	void IntegrateParticles(float dt);
	void ApplyGravity(float dt);


	//helpers
	void ComputeCellCoordinates(const Eigen::Vector3f& particle, int& ix, int& iy, int& iz, Eigen::Vector3f& f);
};
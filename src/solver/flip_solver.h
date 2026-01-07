#include "solver_utils.h"
#include <unsupported/Eigen/CXX11/Tensor>

class FLIPSolver final
{
public:
	FLIPSolver() = delete;
	FLIPSolver(const Solver_Utils::SolverConfig& config);
	~FLIPSolver() = default;

	FLIPSolver(const FLIPSolver& other) = delete;
	FLIPSolver(FLIPSolver&& other) = delete;
	FLIPSolver& operator=(const FLIPSolver& other) = delete;
	FLIPSolver& operator=(FLIPSolver&& other) = delete;

	void Initialize();
	void Simulate(float dt);
	int GetParticleCount() const {return m_numParticles;};
	const Eigen::MatrixXf& GetParticlePositions() const { return m_particlePos; };

private:
	const Eigen::Vector3f m_gravity = Eigen::Vector3f(0.0f, -9.81f, 0.0f);;
	const float m_alphaPICFLIP = 0.95f; //temp

	//-- Grid --
	Eigen::Tensor<float, 4> m_gridV; //(X, Y, Z) + (0 = u, 1 = v, 2 = w)  
	Eigen::Tensor<float, 4> m_gridVBefore; //before projection
	Eigen::Tensor<float, 4> m_gridVAfter; //after projection
	Eigen::Tensor<float, 3> m_gridWeight;
	Eigen::Tensor<float, 3> m_gridPressure;  //Pressure per cell
	Eigen::Tensor<float, 3> m_gridDivergence;//Divergence per cell
	Eigen::Tensor<Solver_Utils::CellType, 3>   m_gridCellType;		//0 = fluid, 1 = air, 2 = solid boundary
	
	const int m_cellNumX; //Num cells in X
	const int m_cellNumY; //Num cells in Y
	const int m_cellNumZ; //Num cells in Z

	//-- Particles --
	const int m_numParticles;
	const float m_particleRadius;

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
	float particleInvSpacing() const {return 1.0 / (2.1 * m_particleRadius); } //2.1 because we dont want to miss our neighbours
	int totalNumCells() const { return m_cellNumX * m_cellNumY * m_cellNumZ; }
};
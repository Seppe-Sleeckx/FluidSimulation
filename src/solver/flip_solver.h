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
	int GetParticleCount() const { return m_numParticles; };
	const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& GetParticlePositions() const { return m_particlePos; };
	const int GetGridXDimension() const { return m_cellNumX; };
	const int GetGridYDimension() const { return m_cellNumY; };
	const int GetGridZDimension() const { return m_cellNumZ; };

	//Logging
	void StartMeasurement();
	void EndMeasurement();
	void WriteLog(const std::string& filename) const;

private:
	const Eigen::Vector3f m_gravity = Eigen::Vector3f(0.0f, -9.81f, 0.0f);;
	const float m_alphaPIC = 0.05f;
	bool m_useAdaptiveMixing = false;

	//-- Grid --
	const int m_cellNumX; //Num cells in X
	const int m_cellNumY; //Num cells in Y
	const int m_cellNumZ; //Num cells in Z

	const float m_CellSize = 1.0f;

	Eigen::Tensor<float, 3> m_gridVU;        // (x+1, y, z)
	Eigen::Tensor<float, 3> m_gridVV;        // (x, y+1, z)
	Eigen::Tensor<float, 3> m_gridVW;		// (x, y, z+1)

	Eigen::Tensor<float, 3> m_gridVUBefore;        // (x+1, y, z)
	Eigen::Tensor<float, 3> m_gridVVBefore;        // (x, y+1, z)
	Eigen::Tensor<float, 3> m_gridVWBefore;		// (x, y, z+1)

	Eigen::Tensor<float, 3> m_gridVUAfter;        // (x+1, y, z)
	Eigen::Tensor<float, 3> m_gridVVAfter;        // (x, y+1, z)
	Eigen::Tensor<float, 3> m_gridVWAfter;		// (x, y, z+1)

	Eigen::Tensor<float, 3> m_gridWeightU;		// (x+1, y, z)
	Eigen::Tensor<float, 3> m_gridWeightV;		// (x, y+1, z)
	Eigen::Tensor<float, 3> m_gridWeightW;		// (x, y, z+1)

	Eigen::Tensor<float, 3> m_gridPressure;  //Pressure per cell
	Eigen::Tensor<float, 3> m_gridDivergence;//Divergence per cell
	Eigen::Tensor<float, 3> m_gridDensity; //particle density per cell
	Eigen::Tensor<Solver_Utils::CellType, 3>   m_gridCellType;		//0 = fluid, 1 = air, 2 = solid boundary

	//-- Particles --
	const int m_numParticles;
	const float m_particleRadius;
	float m_particleRestDensity{}; //what is our reference density of our fluid

	Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> m_particleV;
	Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> m_particlePos;


	//-- Logging --
	std::vector<Solver_Utils::FrameMeasurement> m_frameMeasurements;
	std::chrono::high_resolution_clock::time_point m_measureStart;


	void ClearGrid();
	void MarkFluidCells();
	void TransferP2G();
	void SaveGridBefore();
	void ResolveParticleCollisions();
	void ComputeDivergence();
	void SaveGridAfter();
	void TransferG2P(float dt);
	void IntegrateParticles(float dt);
	void PushParticlesApart(int iterations);
	void ApplyGravity(float dt);
	void UpdateParticleDensity();
	void SolveIncompressibility(float dt, int iterations);

	//Adaptive mixing
	float CalculatePicAlpha(float dt, const Eigen::Vector3f& partilePos) const;
	float GetWeightedDivergenceAtPos(const Eigen::Vector3f& pos) const;

	//helpers
	void ComputeCellCoordinates(const Eigen::Vector3f& particle, int& ix, int& iy, int& iz, Eigen::Vector3f& f) const;
	float inverseGridSpacing() const { return 1.0f / m_CellSize; } //Inverse of our grid cell size
	int totalNumCells() const { return m_cellNumX * m_cellNumY * m_cellNumZ; }
};
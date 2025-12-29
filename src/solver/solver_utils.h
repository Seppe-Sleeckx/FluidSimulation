#include <array>
#include <Eigen/Dense>

namespace Solver_Utils
{
    struct SolverConfig
    {
        std::array<int, 3> gridResolution;
        int numParticles;
    };

    struct Weight3D
    {
        std::array<float, 27> w;    //weights
        std::array<int, 27> ox; //neighbour offsets in x
        std::array<int, 27> oy; //neighbour offsets in y
        std::array<int, 27> oz; //neighbour offsets in z
    };

    enum class CellType : uint8_t {
        Air = 0,
        Fluid = 1,
        Solid = 2
    };

    static Eigen::Vector3f BSplineWeights(float f)
    {
        Eigen::Vector3f w;

        w[0] = 0.5f * (1.5f - f) * (1.5f - f); //left node
        w[1] = 0.75f - (f - 1.0f) * (f - 1.0f); //central node
        w[2] = 0.5f * (f - 0.5f) * (f - 0.5f); //right node

        return w;
    }

    static void ComputeBSplineWeights(const Eigen::Vector3f& f, Weight3D& out)
    {
        auto wx = BSplineWeights(f.x());
        auto wy = BSplineWeights(f.y());
        auto wz = BSplineWeights(f.z());

        int idx = 0;

        for (int k = 0; k < 3; ++k)          //z
        {
            for (int j = 0; j < 3; ++j)      //y
            {
                for (int i = 0; i < 3; ++i)  //x
                {
                    out.w[idx] = wx[i] * wy[j] * wz[k];
                    out.ox[idx] = i - 1;     //neighbors: -1, 0, +1
                    out.oy[idx] = j - 1;
                    out.oz[idx] = k - 1;
                    idx++;
                }
            }
        }    
    }
}

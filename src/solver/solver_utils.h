#include <array>
#include <Eigen/Dense>

namespace Solver_Utils
{
    struct SolverConfig //only use POD, Eigen or STL doesnt work well when needing to use it in js/typescript later
    {
        int gridX;
        int gridY;
        int gridZ;
        int numParticles;
        float particleRadius;
        float alphaPic;
        bool useAdaptiveMixing = false;
    };

    struct FrameMeasurement {
        float stepTime;
        float averageDivergence;
        float averageCompression;
        float maxCompression;
        float totalVolume;
        //maybe also track pressure?
    };

    struct Weight3D
    {
        std::array<float, 27> w;    //weights
        std::array<int, 27> offsetsX; //neighbour offsets in x
        std::array<int, 27> offsetsY; //neighbour offsets in y
        std::array<int, 27> offsetsZ; //neighbour offsets in z
    };

    enum class CellType : uint8_t {
        Air = 0,
        Fluid = 1,
        Solid = 2
    };

    static Eigen::Vector3f BSplineWeights(float f)
    {
        f = std::clamp(f, 0.0f, 1.0f);
        Eigen::Vector3f w;

        w[0] = 0.5f * ((1.0f - f) * (1.0f - f)); //left node
        w[1] = 0.75f - (f - 0.5f) * (f - 0.5f); //central node
        w[2] = 0.5f * (f * f); //right node

        return w; //all our weight will be positive here, if not big problem
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
                    out.offsetsX[idx] = i - 1;     //neighbors: -1, 0, +1
                    out.offsetsY[idx] = j - 1;
                    out.offsetsZ[idx] = k - 1;
                    idx++;
                }
            }
        }    
    }
}

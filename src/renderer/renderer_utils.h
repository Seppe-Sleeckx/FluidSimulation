#include <Eigen/Dense>

namespace Renderer_Utils {
    struct ParticleGPU
    {
        Eigen::Vector3f position;
        Eigen::Vector3f velocity;

        float radius;
        float padding; // alignment
    };
}
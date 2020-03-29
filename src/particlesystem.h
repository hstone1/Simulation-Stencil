#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include <libs/Eigen/StdVector>
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f);
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4i);

#include <libs/Eigen/Core>
#include <vector>


class ParticleSystem {
public:
    ParticleSystem(std::vector<Eigen::Vector4i> &tets, std::vector<Eigen::Vector3f> &locs,
                   float incompressiblity, float rigidity, float viscous_incompressibility, float viscous_rigidity,
                   float gravity);

public:
    void computeAcceleration(
            const std::vector<Eigen::Vector3f> &positions,
            const std::vector<Eigen::Vector3f> &velocities,
            std::vector<Eigen::Vector3f> &accelerations) const;

private:
    unsigned long nParticles;
    std::vector<float> masses;
    float gravity;

    unsigned long nTets;
    std::vector<Eigen::Vector4i> tets;
    std::vector<Eigen::Vector3f> opos;
    std::vector<Eigen::Matrix3f> betas;

    float incompressiblity, rigidity;
    float viscous_incompressibility, viscous_rigidity;
};

#endif // PARTICLESYSTEM_H

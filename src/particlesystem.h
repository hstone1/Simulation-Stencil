#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include <eigen3/Eigen/Core>
#include <vector>


class ParticleSystem {
public:
    ParticleSystem(unsigned long nParticles);

public:
    void computeAcceleration(
            const std::vector<Eigen::Vector3f> &positions,
            const std::vector<Eigen::Vector3f> &velocities,
            std::vector<Eigen::Vector3f> &accelerations) const;

private:
    unsigned long nParticles;
    float gravity;
    std::vector<Eigen::Vector4i> tets;
};

#endif // PARTICLESYSTEM_H

#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H


#include "HEigenLib.h"
#include "collider.h"
#include <vector>


class ParticleSystem {
public:
    ParticleSystem(std::vector<Eigen::Vector4i> &tets, std::vector<Eigen::Vector3f> &locs,
                   float incompressiblity, float rigidity,
                   float viscous_incompressibility, float viscous_rigidity,
                   float density,
                   float gravity);
    ~ParticleSystem();

public:
    void computeAcceleration(
            const std::vector<Eigen::Vector3f> &positions,
            const std::vector<Eigen::Vector3f> &velocities,
            std::vector<Eigen::Vector3f> &accelerations) const;
    void addCollider(Collider *col);

private:
    std::vector<Collider *> colliders;

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

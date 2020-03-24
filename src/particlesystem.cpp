#include "particlesystem.h"

#include <iostream>

typedef unsigned long ulong;

using namespace std;
using namespace Eigen;

ParticleSystem::ParticleSystem(unsigned long nParticles) : nParticles(nParticles), gravity(0.001f)
{

}

void ParticleSystem::computeAcceleration(const std::vector<Vector3f> &positions,
                                         const std::vector<Vector3f> &velocities,
                                         std::vector<Vector3f> &accelerations) const {

    for (ulong i = 0; i < nParticles; i++) {
        accelerations[i] = {0, -gravity, 0};
    }
}

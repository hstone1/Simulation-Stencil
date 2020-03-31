#include "particlesystem.h"

#include <libs/Eigen/Dense>
#include <iostream>

typedef unsigned long ulong;

using namespace std;
using namespace Eigen;

ParticleSystem::ParticleSystem(std::vector<Vector4i> &tets, std::vector<Vector3f> &locs,
                               float incompressiblity, float rigidity,
                               float viscous_incompressibility, float viscous_rigidity,
                               float density,
                               float gravity) :
    nParticles(locs.size()),
    gravity(gravity),
    nTets(tets.size()),
    tets(tets.begin(), tets.end()),
    opos(locs.begin(), locs.end()),
    incompressiblity(incompressiblity),
    rigidity(rigidity),
    viscous_incompressibility(viscous_incompressibility),
    viscous_rigidity(viscous_rigidity)
{
    masses.reserve(nParticles);
    for (int i = 0; i < nParticles; i++) {
        masses.push_back(0.f);
    }

    betas.reserve(nTets);
    for (Vector4i tet : tets) {
        Matrix3f beta_inv;
        const Vector3f v1 = locs[tet[0]] - locs[tet[3]];
        const Vector3f v2 = locs[tet[1]] - locs[tet[3]];
        const Vector3f v3 = locs[tet[2]] - locs[tet[3]];
        beta_inv << v1, v2, v3;

        float volume = abs(v1.cross(v2).dot(v3) / 6);
        masses[tet[0]] += volume * density / 4;
        masses[tet[1]] += volume * density / 4;
        masses[tet[2]] += volume * density / 4;
        masses[tet[3]] += volume * density / 4;

        betas.push_back(beta_inv.inverse());
    }
}

ParticleSystem::~ParticleSystem() {
    for (auto *col : colliders) {
        delete col;
    }
}

void ParticleSystem::computeAcceleration(const std::vector<Vector3f> &pos,
                                         const std::vector<Vector3f> &vel,
                                         std::vector<Vector3f> &accelerations) const {

    for (ulong i = 0; i < nParticles; i++) {
        accelerations[i] = {0, -gravity, 0};
        for (auto *collider : colliders) {
            accelerations[i] += 10000.f * collider->dirToOutside(pos[i]);
        }
    }

    for (ulong i  = 0; i < nTets; i++) {
        Vector4i tet = tets[i];
        Matrix3f P;
        P << pos[tet[0]] - pos[tet[3]], pos[tet[1]] - pos[tet[3]], pos[tet[2]] - pos[tet[3]];
        Matrix3f V;
        V << vel[tet[0]] - vel[tet[3]], vel[tet[1]] - vel[tet[3]], vel[tet[2]] - vel[tet[3]];
        Matrix3f F = P * betas[i];
        Matrix3f Fp = V * betas[i];

        Matrix3f strain = F.transpose() * F - Eigen::Matrix3f::Identity();
        Matrix3f strain_prime = F.transpose() * Fp + Fp.transpose() * F;

        Matrix3f stress = (incompressiblity * strain.trace() * Matrix3f::Identity() + 2 * rigidity * strain) +
                (viscous_incompressibility * strain_prime.trace() * Matrix3f::Identity() + 2 * viscous_rigidity * strain_prime);

        Vector3f af1n = (opos[tet[0]] - opos[tet[2]]).cross(opos[tet[1]] - opos[tet[2]]) / 2.f;
        Vector3f f1force = F * stress * af1n;
        // +
        accelerations[tet[0]] += f1force / masses[tet[0]] / 3;
        accelerations[tet[1]] += f1force / masses[tet[1]] / 3;
        accelerations[tet[2]] += f1force / masses[tet[2]] / 3;

        Vector3f af2n = (opos[tet[0]] - opos[tet[3]]).cross(opos[tet[1]] - opos[tet[3]]) / 2.f;
        Vector3f f2force = F * stress * af2n;
        // -
        accelerations[tet[0]] -= f2force / masses[tet[0]] / 3;
        accelerations[tet[1]] -= f2force / masses[tet[1]] / 3;
        accelerations[tet[3]] -= f2force / masses[tet[3]] / 3;

        Vector3f af3n = (opos[tet[0]] - opos[tet[2]]).cross(opos[tet[3]] - opos[tet[2]]) / 2.f;
        Vector3f f3force = F * stress * af3n;
        // -
        accelerations[tet[0]] -= f3force / masses[tet[0]] / 3;
        accelerations[tet[2]] -= f3force / masses[tet[2]] / 3;
        accelerations[tet[3]] -= f3force / masses[tet[3]] / 3;

        Vector3f af4n = (opos[tet[3]] - opos[tet[2]]).cross(opos[tet[1]] - opos[tet[2]]) / 2.f;
        Vector3f f4force = F * stress * af4n;
        // -
        accelerations[tet[1]] -= f4force / masses[tet[1]] / 3;
        accelerations[tet[2]] -= f4force / masses[tet[2]] / 3;
        accelerations[tet[3]] -= f4force / masses[tet[3]] / 3;
    }

//    cout << accelerations[0].norm() << endl;


}

void ParticleSystem::addCollider(Collider *col) {
    colliders.emplace_back(col);
}

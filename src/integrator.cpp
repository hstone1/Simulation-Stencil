#include "integrator.h"

using namespace std;
using namespace Eigen;

Integrator::Integrator(ParticleSystem *ps, std::vector<Eigen::Vector3f> positions) : pos(positions.begin(), positions.end()), system_(ps) {
    vel.reserve(pos.size());
    for (ulong i = 0;i < pos.size(); i++) {
        vel.push_back({0, 0, 0});
    }
}

const std::vector<Eigen::Vector3f> &Integrator::positions() const {
    return pos;
}

void Integrator::step(float dT) {
    vector<Vector3f> acc1(pos.size());
    vector<Vector3f> acc2(pos.size());
    vector<Vector3f> acc3(pos.size());
    vector<Vector3f> acc4(pos.size());

    vector<Vector3f> pos1(pos.begin(), pos.end());
    vector<Vector3f> vel1(vel.begin(), vel.end());
    vector<Vector3f> avel(vel.begin(), vel.end());

    // RK4
    system_->computeAcceleration(pos, vel, acc1);
    for (ulong i = 0;i < pos.size(); i++) {
        pos[i] = pos1[i] + 0.5 * vel[i] * dT;
        vel[i] = vel1[i] + 0.5 * acc1[i] * dT;
        avel[i] += 2 * vel[i];
    }

    system_->computeAcceleration(pos, vel, acc2);
    for (ulong i = 0;i < pos.size(); i++) {
        pos[i] = pos1[i] + 0.5 * vel[i] * dT;
        vel[i] = vel1[i] + 0.5 * acc2[i] * dT;
        avel[i] += 2 * vel[i];
    }

    system_->computeAcceleration(pos, vel, acc3);
    for (ulong i = 0;i < pos.size(); i++) {
        pos[i] = pos1[i] + vel[i] * dT;
        vel[i] = vel1[i] + acc3[i] * dT;
        avel[i] += vel[i];
    }

     system_->computeAcceleration(pos, vel, acc4);
     for (ulong i = 0;i < pos.size(); i++) {
         pos[i] = pos1[i] + avel[i] / 6.f * dT;
         vel[i] = vel1[i] + (acc1[i] + 2 * acc2[i] + 2 * acc3[i] + acc4[i]) / 6.0f * dT;
     }



}
